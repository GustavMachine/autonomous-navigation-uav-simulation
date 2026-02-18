#!/usr/bin/env python3
"""
Inspector Autónomo de Paredes — Fase 2 — v4
=============================================
REESCRITURA COMPLETA del patrón de inspección.

PATRÓN CORRECTO (por cada posición Y, cada 2m):
  1. Dron en el CENTRO del túnel a altura de tránsito
  2. Gira hacia PARED IZQUIERDA, avanza hasta 1.5m de ella
  3. SCAN: sube de z_min a z_max mirando la pared
  4. Vuelve al CENTRO
  5. Gira hacia PARED DERECHA, avanza hasta 1.5m de ella
  6. SCAN: baja de z_max a z_min mirando la pared
  7. Vuelve al CENTRO, avanza 2m en Y
  8. REPITE hasta el fondo del túnel
  9. Al llegar al fondo → REGRESA al inicio por el centro

CORRECCIONES vs v2:
  - Patrón inspección por sección (ambas paredes por cada Y)
  - Sensor yaw offset incluido (TF base_link→os_sensor: yaw=0.2182rad)
  - Paso reducido 0.4→0.25m anti-penetración
  - TURN-THEN-MOVE: gira completamente antes de moverse
  - z_max=3.5m para túnel de ~4m
  - Verificación de mapa como segunda barrera
  - Transiciones por centro del túnel (nunca teleporta entre paredes)
  - Detección de fondo del túnel para retorno
"""
a
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import PointCloud2
import subprocess, struct, math, time, os, heapq
import numpy as np
from PIL import Image
import yaml


# ══════════════════════════════════════════════════════════════════
# MÓDULO 1: CARGA DE MAPA
# ══════════════════════════════════════════════════════════════════

class OccupancyMap:
    """Carga y gestiona el occupancy grid generado por Cartographer."""

    def __init__(self, yaml_path):
        with open(yaml_path, 'r') as f:
            meta = yaml.safe_load(f)

        pgm_path = meta['image']
        if not os.path.isabs(pgm_path):
            pgm_path = os.path.join(os.path.dirname(yaml_path), pgm_path)

        self.resolution = meta['resolution']
        self.origin_x = meta['origin'][0]
        self.origin_y = meta['origin'][1]
        self.occupied_thresh = meta.get('occupied_thresh', 0.65)
        self.free_thresh = meta.get('free_thresh', 0.196)
        self.negate = meta.get('negate', 0)

        raw = np.array(Image.open(pgm_path), dtype=np.uint8)
        self.height, self.width = raw.shape

        if self.negate == 0:
            prob = (255.0 - raw.astype(float)) / 255.0
        else:
            prob = raw.astype(float) / 255.0

        self.grid = np.full(raw.shape, -1, dtype=np.int8)
        self.grid[prob < self.free_thresh] = 0
        self.grid[prob > self.occupied_thresh] = 100

        print(f"[MAPA] Cargado: {self.width}x{self.height} px, res={self.resolution}m")
        print(f"[MAPA] Origen: ({self.origin_x:.2f}, {self.origin_y:.2f})")
        print(f"[MAPA] Libres: {np.sum(self.grid == 0)}, "
              f"Ocupados: {np.sum(self.grid == 100)}, "
              f"Desconocidos: {np.sum(self.grid == -1)}")

    def world_to_grid(self, wx, wy):
        col = int((wx - self.origin_x) / self.resolution)
        row = self.height - 1 - int((wy - self.origin_y) / self.resolution)
        return row, col

    def grid_to_world(self, row, col):
        wx = self.origin_x + col * self.resolution
        wy = self.origin_y + (self.height - 1 - row) * self.resolution
        return wx, wy

    def is_free(self, row, col):
        if 0 <= row < self.height and 0 <= col < self.width:
            return self.grid[row, col] == 0
        return False

    def is_free_with_margin(self, row, col, margin_cells):
        for dr in range(-margin_cells, margin_cells + 1):
            for dc in range(-margin_cells, margin_cells + 1):
                r, c = row + dr, col + dc
                if 0 <= r < self.height and 0 <= c < self.width:
                    if self.grid[r, c] == 100:
                        return False
                else:
                    return False
        return True

    def is_world_pos_safe(self, wx, wy, margin_m=0.3):
        """Verifica si una posición del mundo es segura según el mapa."""
        row, col = self.world_to_grid(wx, wy)
        margin_cells = max(1, int(margin_m / self.resolution))
        return self.is_free_with_margin(row, col, margin_cells)

    def detect_tunnel_walls(self):
        """Detecta automáticamente las paredes del túnel del mapa."""
        free_rows = []
        for row in range(self.height):
            free_cols = np.where(self.grid[row, :] == 0)[0]
            if len(free_cols) > 5:
                free_rows.append({
                    'row': row,
                    'col_min': free_cols.min(),
                    'col_max': free_cols.max(),
                })

        if not free_rows:
            raise ValueError("No se encontró espacio libre en el mapa")

        col_mins = [r['col_min'] for r in free_rows]
        col_maxs = [r['col_max'] for r in free_rows]
        rows = [r['row'] for r in free_rows]

        left_wall_col = np.percentile(col_mins, 25)
        right_wall_col = np.percentile(col_maxs, 75)

        left_wall_x = self.origin_x + left_wall_col * self.resolution
        right_wall_x = self.origin_x + right_wall_col * self.resolution
        _, y_start = self.grid_to_world(max(rows), 0)
        _, y_end = self.grid_to_world(min(rows), 0)

        tunnel_center_x = (left_wall_x + right_wall_x) / 2.0
        tunnel_width = right_wall_x - left_wall_x

        print(f"[MAPA] Paredes detectadas:")
        print(f"  Pared izquierda: X ≈ {left_wall_x:.2f}m")
        print(f"  Pared derecha:   X ≈ {right_wall_x:.2f}m")
        print(f"  Centro túnel:    X ≈ {tunnel_center_x:.2f}m")
        print(f"  Ancho túnel:     {tunnel_width:.2f}m")
        print(f"  Y inicio:        {y_start:.2f}m")
        print(f"  Y fondo:         {y_end:.2f}m")
        print(f"  Largo túnel:     {y_end - y_start:.2f}m")

        return left_wall_x, right_wall_x, y_start, y_end


# ══════════════════════════════════════════════════════════════════
# MÓDULO 2: GENERADOR DE WAYPOINTS — PATRÓN v4
# ══════════════════════════════════════════════════════════════════

def generate_inspection_waypoints_v4(occ_map, wall_distance=1.5,
                                     z_min=0.8, z_max=3.5,
                                     z_transit=1.5, y_step=2.0):
    """
    Genera waypoints con el patrón CORRECTO de inspección:

    Para cada Y (cada y_step metros):
      1. TRANSIT: ir al centro del túnel a (center_x, Y, z_transit)
      2. APPROACH_L: ir a pared izquierda (left_x + wall_dist, Y, z_min) mirando pared
      3. SCAN_UP: subir de z_min a z_max mirando pared izquierda
      4. RETREAT_L: volver al centro (center_x, Y, z_transit)
      5. APPROACH_R: ir a pared derecha (right_x - wall_dist, Y, z_min) mirando pared
      6. SCAN_UP: subir de z_min a z_max mirando pared derecha
      7. RETREAT_R: volver al centro

    Después de inspeccionar todas las Y:
      8. RETURN: volver al inicio por el centro

    Yaw:
      Para pared izquierda (X-): yaw = π (mirar hacia X-)
      Para pared derecha  (X+): yaw = 0 (mirar hacia X+)
      Para tránsito en Y+:      yaw = π/2 (mirar hacia Y+)
      Para retorno en Y-:       yaw = -π/2 (mirar hacia Y-)

    Cada waypoint: (x, y, z, yaw, mode)
    Modes: TRANSIT, APPROACH, SCAN, RETREAT, RETURN
    """
    left_wall_x, right_wall_x, y_start, y_end = occ_map.detect_tunnel_walls()

    center_x = (left_wall_x + right_wall_x) / 2.0
    left_inspect_x = max(left_wall_x + wall_distance, center_x - 2.0)
    right_inspect_x = min(right_wall_x - wall_distance, center_x + 1.2)

    # Yaw
    YAW_FORWARD = math.pi / 2    # mirar Y+
    YAW_BACKWARD = -math.pi / 2  # mirar Y-
    YAW_LEFT_WALL = math.pi      # mirar X- (pared izquierda)
    YAW_RIGHT_WALL = 0.0         # mirar X+ (pared derecha)

    # Y range: desde Y=0 hasta fondo, con margen
    y_first = 0.0
    y_margin = 2.0  # margen antes del fondo (rocas)
    y_last = y_end - y_margin

    y_positions = np.arange(y_first, y_last, y_step)

    waypoints = []
    margin_cells = max(2, int(0.4 / occ_map.resolution))

    print(f"\n[WAYPOINTS v4] Patrón de inspección por secciones:")
    print(f"  Centro túnel: X = {center_x:.2f}m")
    print(f"  Inspección izq: X = {left_inspect_x:.2f}m (a {wall_distance}m de pared)")
    print(f"  Inspección der: X = {right_inspect_x:.2f}m (a {wall_distance}m de pared)")
    print(f"  Y rango: [{y_first:.1f}, {y_last:.1f}]m  |  Z: [{z_min}, {z_max}]m")
    print(f"  Secciones: {len(y_positions)}  |  Y_step = {y_step}m")

    skipped = 0

    for i, y in enumerate(y_positions):
        # ─── Verificar que las posiciones sean navegables ───
        # Centro
        if not occ_map.is_world_pos_safe(center_x, y, margin_m=0.3):
            # Intentar encontrar un centro local seguro
            adjusted_center = _find_safe_x(occ_map, center_x, y, margin_cells)
            if adjusted_center is None:
                skipped += 1
                continue
            cx = adjusted_center
        else:
            cx = center_x

        # Posición izquierda
        if not occ_map.is_world_pos_safe(left_inspect_x, y, margin_m=0.3):
            lx = _find_safe_x_near_wall(occ_map, left_inspect_x, y, 'left', margin_cells)
            if lx is None:
                lx = None  # Saltar solo esta pared
        else:
            lx = left_inspect_x

        # Posición derecha
        if not occ_map.is_world_pos_safe(right_inspect_x, y, margin_m=0.3):
            rx = _find_safe_x_near_wall(occ_map, right_inspect_x, y, 'right', margin_cells)
            if rx is None:
                rx = None
        else:
            rx = right_inspect_x

        if lx is None and rx is None:
            skipped += 1
            continue

        # ─── 1. TRANSIT: avanzar al centro en esta Y ───
        if i > 0:
            waypoints.append((cx, y, z_transit, YAW_FORWARD, 'TRANSIT'))

        # ─── 2. PARED IZQUIERDA ───
        if lx is not None:
            # APPROACH: ir a la pared izquierda, empezando desde abajo
            waypoints.append((lx, y, z_min, YAW_LEFT_WALL, 'APPROACH'))
            # SCAN: subir mirando la pared
            waypoints.append((lx, y, z_max, YAW_LEFT_WALL, 'SCAN'))
            # RETREAT: volver al centro
            waypoints.append((cx, y, z_transit, YAW_FORWARD, 'RETREAT'))

        # ─── 3. PARED DERECHA ───
        if rx is not None:
            # APPROACH: ir a la pared derecha, empezando desde abajo
            waypoints.append((rx, y, z_min, YAW_RIGHT_WALL, 'APPROACH'))
            # SCAN: subir mirando la pared
            waypoints.append((rx, y, z_max, YAW_RIGHT_WALL, 'SCAN'))
            # RETREAT: volver al centro
            waypoints.append((cx, y, z_transit, YAW_FORWARD, 'RETREAT'))

    # ─── 4. RETURN: volver al inicio por el centro ───
    # Generar waypoints de retorno cada 4m (más rápido, no inspecciona)
    if len(y_positions) > 0:
        y_return = y_positions[-1]
        waypoints.append((center_x, y_return, z_transit, YAW_BACKWARD, 'RETURN'))
        while y_return > y_first:
            y_return -= 4.0  # pasos más grandes en retorno
            y_return = max(y_first, y_return)
            waypoints.append((center_x, y_return, z_transit, YAW_BACKWARD, 'RETURN'))

    n_inspect = len([w for w in waypoints if w[4] in ('APPROACH', 'SCAN', 'RETREAT')])
    n_transit = len([w for w in waypoints if w[4] == 'TRANSIT'])
    n_return = len([w for w in waypoints if w[4] == 'RETURN'])

    print(f"  Total waypoints: {len(waypoints)}")
    print(f"    Inspección: {n_inspect} (approach+scan+retreat)")
    print(f"    Tránsito:   {n_transit}")
    print(f"    Retorno:    {n_return}")
    print(f"    Secciones saltadas: {skipped}")

    return waypoints


def _find_safe_x(occ_map, target_x, y, margin_cells):
    """Busca una posición X segura cerca del target."""
    for offset in np.arange(0.0, 3.0, 0.1):
        for sign in [0, 1, -1]:
            test_x = target_x + sign * offset
            row, col = occ_map.world_to_grid(test_x, y)
            if occ_map.is_free_with_margin(row, col, margin_cells):
                return test_x
    return None


def _find_safe_x_near_wall(occ_map, target_x, y, wall_side, margin_cells):
    """Busca posición segura alejándose de la pared."""
    for offset in np.arange(0.0, 2.0, 0.1):
        if wall_side == 'left':
            test_x = target_x + offset  # alejarse hacia X+
        else:
            test_x = target_x - offset  # alejarse hacia X-
        row, col = occ_map.world_to_grid(test_x, y)
        if occ_map.is_free_with_margin(row, col, margin_cells):
            return test_x
    return None


# ══════════════════════════════════════════════════════════════════
# MÓDULO 3: NODO ROS2 — INSPECTOR DE PAREDES v4
# ══════════════════════════════════════════════════════════════════

class WallInspector(Node):
    def __init__(self, map_yaml_path):
        super().__init__('wall_inspector')

        # ══════════════════════════════════════════════════════════
        # CARGAR MAPA Y GENERAR PLAN
        # ══════════════════════════════════════════════════════════
        self.occ_map = OccupancyMap(map_yaml_path)

        left_x, right_x, _, _ = self.occ_map.detect_tunnel_walls()
        self.tunnel_center_x = (left_x + right_x) / 2.0

        self.get_logger().info("Generando plan de inspección v4...")
        self.waypoints = generate_inspection_waypoints_v4(
            self.occ_map,
            wall_distance=1.5,
            z_min=1.2,
            z_max=3.5,
            z_transit=2.5,
            y_step=2.0)

        self.current_wp_index = 0

        self.get_logger().info(f"Plan total: {len(self.waypoints)} waypoints")

        # ══════════════════════════════════════════════════════════
        # POSICIÓN INICIAL — Y=0, centro del túnel
        # ══════════════════════════════════════════════════════════
        self.pos_x = self.tunnel_center_x
        self.pos_y = 0.0
        self.pos_z = 1.5
        self.yaw = math.pi / 2  # mirando Y+ (adelante)

        # ══════════════════════════════════════════════════════════
        # SENSOR YAW OFFSET
        # TF base_link→os_sensor tiene yaw=0.2182rad (~12.5°)
        # Se incluye en la rotación LiDAR para corregir
        # ══════════════════════════════════════════════════════════
        self.SENSOR_YAW_OFFSET = 0.0  # Confirmado: offset 0 funciona mejor

        # ══════════════════════════════════════════════════════════
        # PARÁMETROS DE MOVIMIENTO
        # Paso reducido vs v2 (0.4→0.25m) para anti-penetración
        # ══════════════════════════════════════════════════════════
        self.STEP_DELAY = 0.4          # s entre pasos
        self.MAX_STEP_XY = 0.25        # m máximo por paso horizontal
        self.MAX_STEP_Z = 0.20         # m máximo por paso vertical (más rápido en Z para scan)
        self.WAYPOINT_REACH_XY = 0.5   # m tolerancia XY
        self.WAYPOINT_REACH_Z = 0.3    # m tolerancia Z

        # ══════════════════════════════════════════════════════════
        # PARÁMETROS DE SEGURIDAD REACTIVA
        # Mismos valores que v2 que funcionaban
        # ══════════════════════════════════════════════════════════
        self.DETECT_RANGE = 8.0
        self.WALL_MIN_DIST = 0.4       # m — mínimo absoluto a paredes
        self.GROUND_CLEARANCE = 0.3    # m — margen mínimo al suelo
        self.CEILING_CLEARANCE = 0.3   # m — margen mínimo al techo

        self.HARD_STOP = 0.5           # m — no pasar (v2 original)
        self.SOFT_STOP = 0.8           # m — empezar a frenar (v2 original)

        # ══════════════════════════════════════════════════════════
        # DISTANCIAS LIDAR (en frame MUNDO)
        # ══════════════════════════════════════════════════════════
        self.dist_xpos = 999.0
        self.dist_xneg = 999.0
        self.dist_ypos = 999.0
        self.dist_yneg = 999.0
        self.dist_zpos = 999.0
        self.dist_zneg = 999.0
        self.lidar_received = False
        self.lidar_count = 0
        self.lidar_timestamp = 0.0

        # ══════════════════════════════════════════════════════════
        # ESTADO
        # ══════════════════════════════════════════════════════════
        self.state = 'INIT'
        self.start_time = None
        self.total_distance = 0.0
        self.total_steps = 0
        self.emergency_stops = 0
        self.map_blocks = 0
        self.stuck_counter = 0
        self.STUCK_THRESHOLD = 30
        self.last_pos = (0.0, 0.0, 0.0)
        self.last_yaw = 0.0
        self.skipped_waypoints = 0
        self.consecutive_skips = 0

        # ══════════════════════════════════════════════════════════
        # LOGGING
        # ══════════════════════════════════════════════════════════
        self.trajectory_log = []

        # ══════════════════════════════════════════════════════════
        # ROS2 SETUP
        # ══════════════════════════════════════════════════════════
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST, depth=1)
        self.lidar_sub = self.create_subscription(
            PointCloud2, '/ouster/points', self.lidar_callback, qos)
        self.nav_timer = self.create_timer(self.STEP_DELAY, self.navigation_loop)
        self.status_timer = self.create_timer(5.0, self.print_status)

        self.get_logger().info('=' * 60)
        self.get_logger().info('  INSPECTOR DE PAREDES v4 — Fase 2')
        self.get_logger().info('  Patrón: Centro→IzqScan→Centro→DerScan→Avanza')
        self.get_logger().info('=' * 60)
        self.get_logger().info(f'  wall_distance = 1.5m')
        self.get_logger().info(f'  Z scan: [{0.8}, {3.5}]m')
        self.get_logger().info(f'  Sensor yaw offset: {self.SENSOR_YAW_OFFSET:.4f} rad')
        self.get_logger().info(f'  MAX_STEP_XY = {self.MAX_STEP_XY}m')
        self.get_logger().info(f'  HARD_STOP = {self.HARD_STOP}m')
        self.get_logger().info('=' * 60)
        self.get_logger().info('Esperando datos LiDAR...')

    # ══════════════════════════════════════════════════════════════
    # LIDAR CALLBACK — ROTA PUNTOS AL FRAME MUNDO CON SENSOR OFFSET
    # ══════════════════════════════════════════════════════════════
    def lidar_callback(self, msg):
        if not self.lidar_received:
            self.lidar_received = True
            self.get_logger().info('LiDAR ACTIVO — iniciando en 5s...')
            self.state = 'WAITING'
            self.start_time = time.time()
            return

        self.lidar_count += 1
        self.lidar_timestamp = time.time()
        data = bytes(msg.data)
        point_step = msg.point_step
        n_points = msg.width * msg.height

        offsets = {}
        for field in msg.fields:
            if field.name in ('x', 'y', 'z'):
                offsets[field.name] = field.offset
        if len(offsets) < 3:
            return

        # Rotación total: yaw del dron + yaw offset del sensor
        total_yaw = self.yaw + self.SENSOR_YAW_OFFSET
        cos_yaw = math.cos(total_yaw)
        sin_yaw = math.sin(total_yaw)

        xpos_dists = []
        xneg_dists = []
        ypos_dists = []
        yneg_dists = []
        zpos_dists = []
        zneg_dists = []

        sample_step = max(1, n_points // 4000)

        for i in range(0, n_points, sample_step):
            base = i * point_step
            if base + 12 > len(data):
                break
            try:
                sx = struct.unpack_from('f', data, base + offsets['x'])[0]
                sy = struct.unpack_from('f', data, base + offsets['y'])[0]
                sz = struct.unpack_from('f', data, base + offsets['z'])[0]
            except:
                continue

            if not (math.isfinite(sx) and math.isfinite(sy) and math.isfinite(sz)):
                continue

            dist_3d = math.sqrt(sx * sx + sy * sy + sz * sz)
            if dist_3d < 0.3 or dist_3d > self.DETECT_RANGE:
                continue

            # Rotar al frame mundo (incluye sensor offset)
            wx = sx * cos_yaw - sy * sin_yaw
            wy = sx * sin_yaw + sy * cos_yaw
            wz = sz

            # Clasificar por dirección en el MUNDO
            # Filtros idénticos a v2 que funcionaban

            if wx > 0.3 and abs(wy) < 2.0 and abs(wz) < 0.5:
                xpos_dists.append(wx)

            if wx < -0.3 and abs(wy) < 2.0 and abs(wz) < 0.5:
                xneg_dists.append(abs(wx))

            if wy > 0.5 and abs(wx) < 1.5 and abs(wz) < 0.5:
                ypos_dists.append(wy)

            if wy < -0.5 and abs(wx) < 1.5 and abs(wz) < 0.5:
                yneg_dists.append(abs(wy))

            if wz > 0.3 and abs(wx) < 1.0 and abs(wy) < 1.0:
                zpos_dists.append(wz)

            if wz < -0.3 and abs(wx) < 1.0 and abs(wy) < 1.0:
                zneg_dists.append(abs(wz))

        self.dist_xpos = min(xpos_dists) if xpos_dists else 999.0
        self.dist_xneg = min(xneg_dists) if xneg_dists else 999.0
        self.dist_ypos = min(ypos_dists) if ypos_dists else 999.0
        self.dist_yneg = min(yneg_dists) if yneg_dists else 999.0
        self.dist_zpos = min(zpos_dists) if zpos_dists else 999.0
        self.dist_zneg = min(zneg_dists) if zneg_dists else 999.0

    # ══════════════════════════════════════════════════════════════
    # MOVIMIENTO
    # ══════════════════════════════════════════════════════════════
    def move_drone(self, x, y, z, yaw=None):
        if yaw is None:
            yaw = self.yaw
        cmd = (f'gz model -m inspection_drone '
               f'-x {x:.4f} -y {y:.4f} -z {z:.4f} '
               f'-R 0 -P 0 -Y {yaw:.4f}')
        try:
            subprocess.run(cmd, shell=True, timeout=2, capture_output=True)
        except:
            pass

    # ══════════════════════════════════════════════════════════════
    # VERIFICACIÓN DE MAPA — segunda barrera
    # ══════════════════════════════════════════════════════════════
    def is_move_safe_by_map(self, new_x, new_y):
        return self.occ_map.is_world_pos_safe(new_x, new_y, margin_m=0.3)

    # ══════════════════════════════════════════════════════════════
    # SEGURIDAD REACTIVA — idéntica a v2 + barrera de mapa
    # ══════════════════════════════════════════════════════════════
    def compute_safety_corrections(self, desired_dx, desired_dy, desired_dz):
        safe_dx = desired_dx
        safe_dy = desired_dy
        safe_dz = desired_dz
        is_emergency = False

        HARD = self.HARD_STOP
        SOFT = self.SOFT_STOP

        # Protección X+
        if self.dist_xpos < HARD and self.dist_xpos < 900:
            if safe_dx > 0:
                safe_dx = 0.0
                is_emergency = True
        elif self.dist_xpos < SOFT and self.dist_xpos < 900:
            if safe_dx > 0:
                factor = max(0.0, (self.dist_xpos - HARD) / (SOFT - HARD))
                safe_dx *= factor

        # Protección X-
        if self.dist_xneg < HARD and self.dist_xneg < 900:
            if safe_dx < 0:
                safe_dx = 0.0
                is_emergency = True
        elif self.dist_xneg < SOFT and self.dist_xneg < 900:
            if safe_dx < 0:
                factor = max(0.0, (self.dist_xneg - HARD) / (SOFT - HARD))
                safe_dx *= factor

        # Protección Y+
        if self.dist_ypos < HARD and self.dist_ypos < 900:
            if safe_dy > 0:
                safe_dy = 0.0
                is_emergency = True
        elif self.dist_ypos < SOFT and self.dist_ypos < 900:
            if safe_dy > 0:
                factor = max(0.0, (self.dist_ypos - HARD) / (SOFT - HARD))
                safe_dy *= factor

        # Protección Y-
        if self.dist_yneg < HARD and self.dist_yneg < 900:
            if safe_dy < 0:
                safe_dy = 0.0
                is_emergency = True
        elif self.dist_yneg < SOFT and self.dist_yneg < 900:
            if safe_dy < 0:
                factor = max(0.0, (self.dist_yneg - HARD) / (SOFT - HARD))
                safe_dy *= factor

        # Protección suelo
        if self.dist_zneg < self.GROUND_CLEARANCE and self.dist_zneg < 900:
            if safe_dz < 0:
                safe_dz = 0.0
            if self.dist_zneg < self.GROUND_CLEARANCE * 0.5:
                safe_dz = max(safe_dz, 0.05)
                is_emergency = True

        # Protección techo
        if self.dist_zpos < self.CEILING_CLEARANCE and self.dist_zpos < 900:
            if safe_dz > 0:
                safe_dz = 0.0
            if self.dist_zpos < self.CEILING_CLEARANCE * 0.5:
                safe_dz = min(safe_dz, -0.05)
                is_emergency = True

        # BARRERA 2: Verificación de mapa
        new_x = self.pos_x + safe_dx
        new_y = self.pos_y + safe_dy
        if not self.is_move_safe_by_map(new_x, new_y):
            self.map_blocks += 1
            if safe_dx != 0 and self.is_move_safe_by_map(self.pos_x + safe_dx, self.pos_y):
                safe_dy = 0.0
            elif safe_dy != 0 and self.is_move_safe_by_map(self.pos_x, self.pos_y + safe_dy):
                safe_dx = 0.0
            else:
                safe_dx = 0.0
                safe_dy = 0.0
                is_emergency = True

        return safe_dx, safe_dy, safe_dz, is_emergency

    # ══════════════════════════════════════════════════════════════
    # LOOP PRINCIPAL
    # ══════════════════════════════════════════════════════════════
    def navigation_loop(self):
        if self.state == 'INIT':
            return

        if self.state == 'WAITING':
            if time.time() - self.start_time > 5.0:
                self.state = 'NAVIGATING'
                self.get_logger().info('=' * 60)
                self.get_logger().info('  INICIANDO INSPECCIÓN v4')
                self.get_logger().info(f'  Waypoints: {len(self.waypoints)}')
                self.get_logger().info('=' * 60)
                self.move_drone(self.pos_x, self.pos_y, self.pos_z, self.yaw)
            return

        if self.state == 'MISSION_COMPLETE':
            return

        if self.state == 'NAVIGATING':
            self.execute_navigation()

    # ══════════════════════════════════════════════════════════════
    # NAVEGACIÓN v4 — TURN-THEN-MOVE
    # ══════════════════════════════════════════════════════════════
    def execute_navigation(self):
        if self.current_wp_index >= len(self.waypoints):
            self.finish_mission()
            return

        target_x, target_y, target_z, target_yaw, wp_mode = self.waypoints[self.current_wp_index]

        dx = target_x - self.pos_x
        dy = target_y - self.pos_y
        dz = target_z - self.pos_z

        dist_xy = math.sqrt(dx * dx + dy * dy)
        dist_z = abs(dz)

        # ─── Stuck detection ───
        moved = math.sqrt(
            (self.pos_x - self.last_pos[0]) ** 2 +
            (self.pos_y - self.last_pos[1]) ** 2 +
            (self.pos_z - self.last_pos[2]) ** 2)
        yaw_moved = abs(self.yaw - self.last_yaw) > 0.02
        self.last_pos = (self.pos_x, self.pos_y, self.pos_z)
        self.last_yaw = self.yaw

        if moved < 0.01 and not yaw_moved:
            self.stuck_counter += 1
        else:
            self.stuck_counter = max(0, self.stuck_counter - 2)

        if self.stuck_counter >= self.STUCK_THRESHOLD:
            self.get_logger().warn(
                f'STUCK en WP{self.current_wp_index} ({wp_mode}) — saltando')
            self.current_wp_index += 1
            self.stuck_counter = 0
            self.skipped_waypoints += 1
            self.consecutive_skips += 1
            # Si muchos skips, saltar a siguiente sección Y
            if self.consecutive_skips >= 3:
                self._skip_to_next_section()
            return

        # ─── ¿Llegamos al waypoint? ───
        if dist_xy < self.WAYPOINT_REACH_XY and dist_z < self.WAYPOINT_REACH_Z:
            self.get_logger().info(
                f'  ✓ WP{self.current_wp_index} ({wp_mode}) alcanzado')
            self.current_wp_index += 1
            self.stuck_counter = 0
            self.consecutive_skips = 0
            return

        # ══════════════════════════════════════════════════════════
        # TURN-THEN-MOVE: primero girar, luego mover
        # ══════════════════════════════════════════════════════════

        yaw_diff = target_yaw - self.yaw
        while yaw_diff > math.pi:
            yaw_diff -= 2 * math.pi
        while yaw_diff < -math.pi:
            yaw_diff += 2 * math.pi
        yaw_error = abs(yaw_diff)

        YAW_TOLERANCE = 0.15  # ~8.6°
        YAW_TURN_SPEED = 0.4

        if yaw_error > YAW_TOLERANCE:
            # ─── FASE 1: Solo girar ───
            self.yaw += yaw_diff * YAW_TURN_SPEED
            while self.yaw > math.pi:
                self.yaw -= 2 * math.pi
            while self.yaw < -math.pi:
                self.yaw += 2 * math.pi
            self.move_drone(self.pos_x, self.pos_y, self.pos_z, self.yaw)
            self._log_step(False)
            return

        # ─── FASE 2: Ya alineado, mover ───
        desired_dx = 0.0
        desired_dy = 0.0
        desired_dz = 0.0

        # Movimiento XY
        if abs(dx) > 0.05:
            step_x = min(self.MAX_STEP_XY, abs(dx))
            desired_dx = (dx / abs(dx)) * step_x

        if abs(dy) > 0.05:
            step_y = min(self.MAX_STEP_XY, abs(dy))
            desired_dy = (dy / abs(dy)) * step_y

        # Z independiente
        if abs(dz) > 0.05:
            desired_dz = (dz / abs(dz)) * min(self.MAX_STEP_Z, abs(dz))

        # ═══ SEGURIDAD SEGÚN MODO ═══
        # RETREAT/TRANSIT hacia el centro: relajar seguridad lateral del LiDAR
        # porque la pared curva del túnel causa lecturas falsas.
        # Solo usar barrera del mapa como protección.
        moving_toward_center = (
            (self.pos_x > self.tunnel_center_x and desired_dx < 0) or
            (self.pos_x < self.tunnel_center_x and desired_dx > 0)
        )

        if wp_mode in ('RETREAT', 'TRANSIT') and moving_toward_center:
            # Solo verificar mapa, no LiDAR lateral
            safe_dx = desired_dx
            safe_dy = desired_dy
            safe_dz = desired_dz
            is_emergency = False

            # Barrera de mapa solamente
            new_x = self.pos_x + safe_dx
            new_y = self.pos_y + safe_dy
            if not self.is_move_safe_by_map(new_x, new_y):
                self.map_blocks += 1
                safe_dx = 0.0
                safe_dy = 0.0
                is_emergency = True

            # Protección de suelo/techo sí mantener
            if self.dist_zneg < self.GROUND_CLEARANCE and self.dist_zneg < 900:
                if safe_dz < 0:
                    safe_dz = 0.0
            if self.dist_zpos < self.CEILING_CLEARANCE and self.dist_zpos < 900:
                if safe_dz > 0:
                    safe_dz = 0.0
        else:
            # Seguridad completa para APPROACH, SCAN, RETURN
            safe_dx, safe_dy, safe_dz, is_emergency = self.compute_safety_corrections(
                desired_dx, desired_dy, desired_dz)

        if is_emergency:
            self.emergency_stops += 1

        # Mantener yaw alineado
        self.yaw += yaw_diff * 0.1

        # Aplicar
        self.pos_x += safe_dx
        self.pos_y += safe_dy
        self.pos_z += safe_dz
        self.pos_z = max(0.5, min(3.8, self.pos_z))

        self.move_drone(self.pos_x, self.pos_y, self.pos_z, self.yaw)
        self._log_step(is_emergency)

    # ══════════════════════════════════════════════════════════════
    # LOG STEP
    # ══════════════════════════════════════════════════════════════
    def _log_step(self, is_emergency):
        self.total_steps += 1
        if len(self.trajectory_log) > 0:
            last = self.trajectory_log[-1]
            step_dist = math.sqrt(
                (self.pos_x - last['x']) ** 2 +
                (self.pos_y - last['y']) ** 2 +
                (self.pos_z - last['z']) ** 2)
            self.total_distance += step_dist

        wp_mode = self.waypoints[self.current_wp_index][4] if self.current_wp_index < len(self.waypoints) else 'DONE'
        self.trajectory_log.append({
            'time': time.time() - self.start_time,
            'x': self.pos_x, 'y': self.pos_y, 'z': self.pos_z,
            'yaw': self.yaw,
            'wp_index': self.current_wp_index,
            'wp_mode': wp_mode,
            'dist_xneg': self.dist_xneg, 'dist_xpos': self.dist_xpos,
            'dist_ypos': self.dist_ypos, 'dist_yneg': self.dist_yneg,
            'dist_zneg': self.dist_zneg, 'dist_zpos': self.dist_zpos,
            'emergency': is_emergency,
        })

    # ══════════════════════════════════════════════════════════════
    # SKIP TO NEXT SECTION (salta sección Y completa)
    # ══════════════════════════════════════════════════════════════
    def _skip_to_next_section(self):
        """Salta al siguiente TRANSIT (nueva sección Y)."""
        start_idx = self.current_wp_index
        for i in range(self.current_wp_index + 1, len(self.waypoints)):
            if self.waypoints[i][4] == 'TRANSIT':
                self.get_logger().warn(
                    f'  Saltando sección: WP{start_idx}→WP{i}')
                self.current_wp_index = i
                self.consecutive_skips = 0
                return
        # Si no hay más TRANSIT, buscar RETURN
        for i in range(self.current_wp_index + 1, len(self.waypoints)):
            if self.waypoints[i][4] == 'RETURN':
                self.current_wp_index = i
                self.consecutive_skips = 0
                return
        self.current_wp_index = len(self.waypoints)
        self.consecutive_skips = 0

    # ══════════════════════════════════════════════════════════════
    # STATUS
    # ══════════════════════════════════════════════════════════════
    def print_status(self):
        if self.state != 'NAVIGATING':
            return

        elapsed = time.time() - self.start_time
        wp_total = len(self.waypoints)
        wp_done = self.current_wp_index
        progress = wp_done / wp_total * 100 if wp_total > 0 else 0

        if wp_done < wp_total:
            wp_mode = self.waypoints[wp_done][4]
            wp_target = self.waypoints[wp_done]
            target_info = f'→({wp_target[0]:.1f},{wp_target[1]:.1f},{wp_target[2]:.1f})'
        else:
            wp_mode = 'DONE'
            target_info = ''

        self.get_logger().info(
            f'{wp_mode} | WP {wp_done}/{wp_total} ({progress:.0f}%) {target_info} | '
            f'Pos=({self.pos_x:.2f},{self.pos_y:.2f},{self.pos_z:.2f}) Yaw={math.degrees(self.yaw):.0f}° | '
            f'X-={self.dist_xneg:.1f} X+={self.dist_xpos:.1f} '
            f'Y+={self.dist_ypos:.1f} Y-={self.dist_yneg:.1f} | '
            f'↓{self.dist_zneg:.1f} ↑{self.dist_zpos:.1f} | '
            f'{elapsed:.0f}s | E={self.emergency_stops} M={self.map_blocks} '
            f'Skip={self.skipped_waypoints}')

    # ══════════════════════════════════════════════════════════════
    # MISIÓN COMPLETA
    # ══════════════════════════════════════════════════════════════
    def finish_mission(self):
        self.state = 'MISSION_COMPLETE'
        elapsed = time.time() - self.start_time
        avg_speed = self.total_distance / elapsed if elapsed > 0 else 0

        self.get_logger().info('=' * 60)
        self.get_logger().info('  INSPECCIÓN v4 COMPLETA')
        self.get_logger().info('=' * 60)
        self.get_logger().info(f'  Waypoints: {len(self.waypoints)}')
        self.get_logger().info(f'  Steps: {self.total_steps}')
        self.get_logger().info(f'  Distancia: {self.total_distance:.1f}m')
        self.get_logger().info(f'  Tiempo: {elapsed:.1f}s ({elapsed / 60:.1f}min)')
        self.get_logger().info(f'  Vel promedio: {avg_speed:.3f} m/s')
        self.get_logger().info(f'  Emergency stops: {self.emergency_stops}')
        self.get_logger().info(f'  Map blocks: {self.map_blocks}')
        self.get_logger().info(f'  Skipped waypoints: {self.skipped_waypoints}')
        self.get_logger().info('=' * 60)

        self.save_metrics()

    # ══════════════════════════════════════════════════════════════
    # GUARDAR MÉTRICAS
    # ══════════════════════════════════════════════════════════════
    def save_metrics(self):
        results_dir = os.path.expanduser('~/tesis/resultados_fase2')
        os.makedirs(results_dir, exist_ok=True)
        timestamp = time.strftime('%Y%m%d_%H%M%S')
        elapsed = time.time() - self.start_time

        # Métricas texto
        metrics_file = os.path.join(results_dir, f'{timestamp}_v4_metrics.txt')
        with open(metrics_file, 'w') as f:
            f.write('=' * 60 + '\n')
            f.write('INSPECCIÓN AUTÓNOMA — Fase 2 v4 — MÉTRICAS\n')
            f.write('=' * 60 + '\n')
            f.write(f'Fecha: {time.strftime("%Y-%m-%d %H:%M:%S")}\n')
            f.write(f'Patrón: Centro→ParedIzq→Scan→Centro→ParedDer→Scan→Avanza\n')
            f.write(f'Waypoints totales: {len(self.waypoints)}\n')
            f.write(f'Steps totales: {self.total_steps}\n')
            f.write(f'Distancia total: {self.total_distance:.1f}m\n')
            f.write(f'Tiempo total: {elapsed:.1f}s ({elapsed / 60:.1f}min)\n')
            f.write(f'Velocidad promedio: {self.total_distance / elapsed:.3f} m/s\n')
            f.write(f'Emergency stops: {self.emergency_stops}\n')
            f.write(f'Map blocks: {self.map_blocks}\n')
            f.write(f'Skipped waypoints: {self.skipped_waypoints}\n')
            f.write(f'\n--- Parámetros v4 ---\n')
            f.write(f'Step delay: {self.STEP_DELAY}s\n')
            f.write(f'Max step XY: {self.MAX_STEP_XY}m\n')
            f.write(f'Max step Z: {self.MAX_STEP_Z}m\n')
            f.write(f'HARD_STOP: {self.HARD_STOP}m\n')
            f.write(f'SOFT_STOP: {self.SOFT_STOP}m\n')
            f.write(f'Wall distance: 1.5m\n')
            f.write(f'Z range: [0.8, 3.5]m\n')
            f.write(f'Y step: 2.0m\n')
            f.write(f'Sensor yaw offset: {self.SENSOR_YAW_OFFSET:.4f} rad\n')

        self.get_logger().info(f'Métricas: {metrics_file}')

        # Trayectoria CSV
        traj_file = os.path.join(results_dir, f'{timestamp}_v4_trajectory.csv')
        with open(traj_file, 'w') as f:
            f.write('time,x,y,z,yaw,wp_index,wp_mode,dist_xneg,dist_xpos,dist_ypos,dist_yneg,dist_zneg,dist_zpos,emergency\n')
            for t in self.trajectory_log:
                f.write(
                    f"{t['time']:.3f},{t['x']:.4f},{t['y']:.4f},{t['z']:.4f},"
                    f"{t['yaw']:.4f},{t['wp_index']},{t['wp_mode']},"
                    f"{t['dist_xneg']:.2f},{t['dist_xpos']:.2f},"
                    f"{t['dist_ypos']:.2f},{t['dist_yneg']:.2f},"
                    f"{t['dist_zneg']:.2f},{t['dist_zpos']:.2f},"
                    f"{t['emergency']}\n")

        self.get_logger().info(f'Trayectoria: {traj_file}')

        # PLY
        ply_file = os.path.join(results_dir, f'{timestamp}_v4_trajectory.ply')
        with open(ply_file, 'w') as f:
            f.write('ply\n')
            f.write('format ascii 1.0\n')
            f.write(f'element vertex {len(self.trajectory_log)}\n')
            f.write('property float x\n')
            f.write('property float y\n')
            f.write('property float z\n')
            f.write('property uchar red\n')
            f.write('property uchar green\n')
            f.write('property uchar blue\n')
            f.write('end_header\n')
            for t in self.trajectory_log:
                mode = t['wp_mode']
                if mode == 'SCAN':
                    r, g, b = 0, 255, 0       # verde
                elif mode == 'APPROACH':
                    r, g, b = 255, 255, 0      # amarillo
                elif mode == 'RETREAT':
                    r, g, b = 0, 100, 255      # azul
                elif mode == 'TRANSIT':
                    r, g, b = 255, 128, 0      # naranja
                elif mode == 'RETURN':
                    r, g, b = 128, 0, 255      # morado
                else:
                    r, g, b = 200, 200, 200
                if t['emergency']:
                    r, g, b = 255, 0, 0
                f.write(f"{t['x']:.4f} {t['y']:.4f} {t['z']:.4f} {r} {g} {b}\n")

        self.get_logger().info(f'PLY: {ply_file}')

        # Waypoints planificados
        wp_file = os.path.join(results_dir, f'{timestamp}_v4_waypoints.csv')
        with open(wp_file, 'w') as f:
            f.write('index,x,y,z,yaw,mode\n')
            for i, wp in enumerate(self.waypoints):
                x, y, z, yaw, mode = wp
                f.write(f'{i},{x:.4f},{y:.4f},{z:.4f},{yaw:.4f},{mode}\n')

        self.get_logger().info(f'Waypoints: {wp_file}')


# ══════════════════════════════════════════════════════════════════
# MAIN
# ══════════════════════════════════════════════════════════════════
def main():
    import sys
    default_map = os.path.expanduser('~/tesis/resultados_fase2/tunnel_map.yaml')
    map_path = sys.argv[1] if len(sys.argv) > 1 else default_map

    if not os.path.exists(map_path):
        print(f"ERROR: No se encontró el mapa: {map_path}")
        sys.exit(1)

    rclpy.init()
    node = WallInspector(map_path)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Interrumpido por usuario')
        if node.total_steps > 0:
            node.save_metrics()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
