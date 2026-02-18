#!/usr/bin/env python3
"""
Explorador Autónomo de Socavones Mineros v2
=============================================
Navegación reactiva pura basada en LiDAR — CERO hardcoding de coordenadas.
Funciona en cualquier entorno subterráneo sin conocimiento previo.

Algoritmo:
  - Avanza en Y+ mientras haya espacio libre adelante
  - Se centra automáticamente usando distancias laterales del LiDAR
  - Oscila en Z para cobertura vertical (escanea piso, paredes, techo)
  - Al detectar fondo (sin espacio adelante), pasada completada
  - Teletransporte al inicio para siguiente pasada con offset lateral
  - Condición de parada: N pasadas completadas

Pasadas:
  1. Centro (X=0.0) — cobertura principal
  2. Izquierda (X=-0.5) — pared izquierda
  3. Derecha (X=+0.5) — pared derecha

Diseñado para:
  - LiDAR Ouster OS0-128 (PointCloud2)
  - Movimiento cinemático (gz model)
  - Gazebo Classic 11 con software rendering (~2.5 Hz LiDAR)
  - Cartographer 3D corriendo en paralelo para SLAM
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import PointCloud2
import subprocess, struct, math, time, os


class AutonomousExplorer(Node):
    def __init__(self):
        super().__init__('autonomous_explorer')

        # ══════════════════════════════════════════════════════════
        # POSICIÓN INICIAL
        # ══════════════════════════════════════════════════════════
        self.start_x = 0.0
        self.start_y = 0.0
        self.start_z = 1.2
        self.yaw = 1.5708  # Siempre mirando Y+

        self.pos_x = self.start_x
        self.pos_y = self.start_y
        self.pos_z = self.start_z

        # ══════════════════════════════════════════════════════════
        # PARÁMETROS DE MOVIMIENTO
        # ══════════════════════════════════════════════════════════
        self.STEP_DELAY = 0.8          # s entre pasos (~2 scans LiDAR a 2.5Hz)
        self.STEP_SIZE = 0.3           # m por paso de avance
        self.LATERAL_STEP = 0.08       # m por paso lateral (evasión/centrado)

        # ══════════════════════════════════════════════════════════
        # PARÁMETROS DE DETECCIÓN
        # ══════════════════════════════════════════════════════════
        self.DETECT_RANGE = 8.0        # m — rango máximo de detección
        self.SLOW_DOWN_DIST = 5.0      # m — empieza a reducir velocidad
        self.AVOID_DIST = 2.5          # m — evasión lateral activa
        self.STOP_DIST = 1.0           # m — freno (considerar fondo)
        self.WALL_MARGIN = 0.5         # m — margen mínimo a paredes
        self.CENTERING_GAIN = 0.35     # fuerza de centrado (0-1)

        # ══════════════════════════════════════════════════════════
        # OSCILACIÓN EN Z (cobertura vertical)
        # ══════════════════════════════════════════════════════════
        self.Z_MIN = 0.8
        self.Z_MAX = 2.2
        self.Z_PERIOD_STEPS = 40
        self.z_direction = 1
        self.Z_OSCILLATION_STEP = (self.Z_MAX - self.Z_MIN) / (self.Z_PERIOD_STEPS / 2)

        # ══════════════════════════════════════════════════════════
        # MÚLTIPLES PASADAS
        # ══════════════════════════════════════════════════════════
        self.MAX_PASSES = 3
        self.current_pass = 0
        self.LATERAL_OFFSETS = [0.0, -0.5, 0.5]
        self.DEAD_END_PATIENCE = 15
        self.dead_end_counter = 0
        self.TELEPORT_PAUSE = 5.0      # s de pausa después de teletransporte

        # ══════════════════════════════════════════════════════════
        # ESTADO
        # ══════════════════════════════════════════════════════════
        self.state = 'INIT'
        # Estados: INIT -> WAITING -> EXPLORING -> TELEPORTING -> MISSION_COMPLETE
        self.avoidance_direction = 0
        self.obstacle_cleared_count = 0
        self.total_distance = 0.0
        self.total_steps = 0
        self.start_time = None
        self.current_speed_factor = 1.0
        self.teleport_time = 0.0

        # Tracking por pasada
        self.pass_max_y = -999.0
        self.pass_steps = 0
        self.pass_distance = 0.0

        # ══════════════════════════════════════════════════════════
        # DISTANCIAS LIDAR (siempre en frame del sensor, sin invertir)
        # ══════════════════════════════════════════════════════════
        self.front_dist = 999.0        # Y+ (adelante)
        self.left_dist = 999.0         # X- (izquierda)
        self.right_dist = 999.0        # X+ (derecha)
        self.above_dist = 999.0        # Z+ (arriba)
        self.below_dist = 999.0        # Z- (abajo)
        self.lidar_received = False
        self.lidar_count = 0

        # ══════════════════════════════════════════════════════════
        # LOGGING
        # ══════════════════════════════════════════════════════════
        self.trajectory_log = []
        self.pass_logs = []

        # ══════════════════════════════════════════════════════════
        # ROS2 SETUP
        # ══════════════════════════════════════════════════════════
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST, depth=1)
        self.lidar_sub = self.create_subscription(
            PointCloud2, '/ouster/points', self.lidar_callback, qos)
        self.nav_timer = self.create_timer(self.STEP_DELAY, self.navigation_loop)
        self.status_timer = self.create_timer(3.0, self.print_status)

        self.get_logger().info('=' * 60)
        self.get_logger().info('  EXPLORADOR AUTÓNOMO DE SOCAVONES v2')
        self.get_logger().info('  Navegación Reactiva + Oscilación Z')
        self.get_logger().info('=' * 60)
        self.get_logger().info(f'Pasadas: {self.MAX_PASSES} | Step: {self.STEP_SIZE}m cada {self.STEP_DELAY}s')
        self.get_logger().info(f'Oscilación Z: [{self.Z_MIN}, {self.Z_MAX}]m')
        self.get_logger().info(f'Offsets laterales: {self.LATERAL_OFFSETS}')
        self.get_logger().info('Esperando datos LiDAR...')

    # ══════════════════════════════════════════════════════════════
    # LIDAR CALLBACK — Sin inversión de ejes, siempre frame fijo
    # ══════════════════════════════════════════════════════════════
    def lidar_callback(self, msg):
        if not self.lidar_received:
            self.lidar_received = True
            self.get_logger().info('LiDAR ACTIVO — iniciando en 5s...')
            self.state = 'WAITING'
            self.start_time = time.time()
            return

        self.lidar_count += 1
        data = bytes(msg.data)
        point_step = msg.point_step
        n_points = msg.width * msg.height

        offsets = {}
        for field in msg.fields:
            if field.name in ('x', 'y', 'z'):
                offsets[field.name] = field.offset
        if len(offsets) < 3:
            return

        front_dists = []
        left_dists = []
        right_dists = []
        above_dists = []
        below_dists = []

        sample_step = max(1, n_points // 3000)

        for i in range(0, n_points, sample_step):
            base = i * point_step
            if base + 12 > len(data):
                break
            try:
                px = struct.unpack_from('f', data, base + offsets['x'])[0]
                py = struct.unpack_from('f', data, base + offsets['y'])[0]
                pz = struct.unpack_from('f', data, base + offsets['z'])[0]
            except:
                continue

            if not (math.isfinite(px) and math.isfinite(py) and math.isfinite(pz)):
                continue

            dist_3d = math.sqrt(px*px + py*py + pz*pz)
            if dist_3d < 0.5 or dist_3d > self.DETECT_RANGE:
                continue

            # ─── FRONTAL (Y+): banda ampliada para detectar paredes irregulares ───
            if (py > 0.8 and
                abs(px) < 1.2 and
                pz > -0.5 and pz < 0.5):
                front_dists.append(py)

            # ─── LATERAL IZQUIERDA (X-) ───
            if (px < -0.3 and
                abs(py) < 2.0 and
                pz > -0.3 and pz < 0.3):
                left_dists.append(abs(px))

            # ─── LATERAL DERECHA (X+) ───
            if (px > 0.3 and
                abs(py) < 2.0 and
                pz > -0.3 and pz < 0.3):
                right_dists.append(abs(px))

            # ─── ARRIBA (Z+) ───
            if (pz > 0.3 and
                abs(px) < 1.0 and
                abs(py) < 1.0):
                above_dists.append(pz)

            # ─── ABAJO (Z-) ───
            if (pz < -0.3 and
                abs(px) < 1.0 and
                abs(py) < 1.0):
                below_dists.append(abs(pz))

        self.front_dist = min(front_dists) if front_dists else 999.0
        self.left_dist = min(left_dists) if left_dists else 999.0
        self.right_dist = min(right_dists) if right_dists else 999.0
        self.above_dist = min(above_dists) if above_dists else 999.0
        self.below_dist = min(below_dists) if below_dists else 999.0

    # ══════════════════════════════════════════════════════════════
    # MOVIMIENTO
    # ══════════════════════════════════════════════════════════════
    def move_drone(self, x, y, z):
        cmd = f'gz model -m inspection_drone -x {x:.4f} -y {y:.4f} -z {z:.4f}'
        try:
            subprocess.run(cmd, shell=True, timeout=2, capture_output=True)
        except:
            pass

    # ══════════════════════════════════════════════════════════════
    # VELOCIDAD ADAPTATIVA
    # ══════════════════════════════════════════════════════════════
    def compute_speed_factor(self):
        f = self.front_dist
        if f >= self.SLOW_DOWN_DIST:
            return 1.0
        elif f <= self.STOP_DIST:
            return 0.05
        elif f <= self.AVOID_DIST:
            t = (f - self.STOP_DIST) / (self.AVOID_DIST - self.STOP_DIST)
            return 0.1 + 0.2 * t
        else:
            t = (f - self.AVOID_DIST) / (self.SLOW_DOWN_DIST - self.AVOID_DIST)
            return 0.3 + 0.7 * t

    # ══════════════════════════════════════════════════════════════
    # CENTRADO EN EL TÚNEL
    # ══════════════════════════════════════════════════════════════
    def compute_centering(self):
        if self.avoidance_direction != 0:
            return 0.0

        L = self.left_dist if self.left_dist < 900 else 5.0
        R = self.right_dist if self.right_dist < 900 else 5.0

        error = L - R
        correction = error * self.CENTERING_GAIN * self.LATERAL_STEP
        max_correction = self.LATERAL_STEP * 0.6
        return max(-max_correction, min(max_correction, correction))

    # ══════════════════════════════════════════════════════════════
    # REPULSIÓN DE PAREDES
    # ══════════════════════════════════════════════════════════════
    def compute_wall_repulsion(self):
        move_x = 0.0
        if self.left_dist < self.WALL_MARGIN and self.left_dist < 900:
            repulsion = (self.WALL_MARGIN - self.left_dist) / self.WALL_MARGIN
            move_x += self.LATERAL_STEP * repulsion * 0.8
        if self.right_dist < self.WALL_MARGIN and self.right_dist < 900:
            repulsion = (self.WALL_MARGIN - self.right_dist) / self.WALL_MARGIN
            move_x -= self.LATERAL_STEP * repulsion * 0.8
        return move_x

    # ══════════════════════════════════════════════════════════════
    # EVASIÓN DE OBSTÁCULOS
    # ══════════════════════════════════════════════════════════════
    def compute_avoidance(self):
        move_x = 0.0
        f = self.front_dist

        if f < self.AVOID_DIST:
            if self.avoidance_direction == 0:
                L = self.left_dist if self.left_dist < 900 else 5.0
                R = self.right_dist if self.right_dist < 900 else 5.0
                self.avoidance_direction = -1 if L > R else 1
                dir_name = "IZQ" if self.avoidance_direction == -1 else "DER"
                self.get_logger().info(
                    f'OBSTÁCULO a {f:.1f}m | L={L:.1f} R={R:.1f} -> {dir_name}')
                self.obstacle_cleared_count = 0

            urgency = 1.0 - (f / self.AVOID_DIST)
            urgency = max(0.3, min(1.0, urgency))
            move_x = self.avoidance_direction * self.LATERAL_STEP * urgency * 2.0

        elif self.avoidance_direction != 0:
            self.obstacle_cleared_count += 1
            if self.obstacle_cleared_count > 15:
                self.avoidance_direction = 0
                self.obstacle_cleared_count = 0

        return move_x

    # ══════════════════════════════════════════════════════════════
    # OSCILACIÓN EN Z
    # ══════════════════════════════════════════════════════════════
    def compute_z_oscillation(self):
        if self.pos_z >= self.Z_MAX:
            self.z_direction = -1
        elif self.pos_z <= self.Z_MIN:
            self.z_direction = 1

        if self.above_dist < 0.4 and self.above_dist < 900:
            self.z_direction = -1
        if self.below_dist < 0.3 and self.below_dist < 900:
            self.z_direction = 1

        return self.z_direction * self.Z_OSCILLATION_STEP

    # ══════════════════════════════════════════════════════════════
    # DETECCIÓN DE FONDO
    # ══════════════════════════════════════════════════════════════
    def check_dead_end(self):
        if self.front_dist < self.STOP_DIST:
            self.dead_end_counter += 1
        else:
            self.dead_end_counter = max(0, self.dead_end_counter - 1)
        return self.dead_end_counter >= self.DEAD_END_PATIENCE

    # ══════════════════════════════════════════════════════════════
    # TELETRANSPORTE AL INICIO (entre pasadas)
    # ══════════════════════════════════════════════════════════════
    def teleport_to_start(self):
        """Teletransporta el dron al inicio con el offset de la siguiente pasada."""
        offset = self.LATERAL_OFFSETS[min(self.current_pass, len(self.LATERAL_OFFSETS) - 1)]
        self.pos_x = self.start_x + offset
        self.pos_y = self.start_y
        self.pos_z = self.start_z
        self.z_direction = 1  # resetear oscilación

        self.move_drone(self.pos_x, self.pos_y, self.pos_z)
        self.teleport_time = time.time()

        self.get_logger().info(
            f'TELETRANSPORTE al inicio: ({self.pos_x:.1f}, {self.pos_y:.1f}, {self.pos_z:.1f})')

    # ══════════════════════════════════════════════════════════════
    # LOOP PRINCIPAL
    # ══════════════════════════════════════════════════════════════
    def navigation_loop(self):
        # ─── INIT ───
        if self.state == 'INIT':
            return

        # ─── WAITING: estabilización inicial ───
        if self.state == 'WAITING':
            if time.time() - self.start_time > 5.0:
                self.state = 'EXPLORING'
                self.current_pass = 0
                self.pass_max_y = -999.0
                self.pass_steps = 0
                self.pass_distance = 0.0
                self.dead_end_counter = 0
                self.get_logger().info('=' * 60)
                self.get_logger().info(f'  PASADA {self.current_pass + 1}/{self.MAX_PASSES} — ADELANTE')
                offset = self.LATERAL_OFFSETS[self.current_pass]
                self.get_logger().info(f'  Offset lateral: {offset}m')
                self.get_logger().info('=' * 60)
            return

        # ─── TELEPORTING: esperando estabilización después de teletransporte ───
        if self.state == 'TELEPORTING':
            if time.time() - self.teleport_time > self.TELEPORT_PAUSE:
                self.state = 'EXPLORING'
                self.dead_end_counter = 0
                self.avoidance_direction = 0
                self.obstacle_cleared_count = 0
                self.pass_max_y = -999.0
                self.pass_steps = 0
                self.pass_distance = 0.0
                self.get_logger().info('=' * 60)
                self.get_logger().info(f'  PASADA {self.current_pass + 1}/{self.MAX_PASSES} — ADELANTE')
                offset = self.LATERAL_OFFSETS[min(self.current_pass, len(self.LATERAL_OFFSETS) - 1)]
                self.get_logger().info(f'  Offset lateral: {offset}m')
                self.get_logger().info('=' * 60)
            return

        # ─── MISSION COMPLETE ───
        if self.state == 'MISSION_COMPLETE':
            return

        # ─── EXPLORING ───
        if self.state == 'EXPLORING':
            self.execute_exploration()

    # ══════════════════════════════════════════════════════════════
    # EXPLORACIÓN (siempre avanza en Y+)
    # ══════════════════════════════════════════════════════════════
    def execute_exploration(self):
        # ─── Detectar fondo ───
        if self.check_dead_end():
            self.end_pass()
            return

        # ─── Velocidad adaptativa ───
        self.current_speed_factor = self.compute_speed_factor()

        # ─── Avance en Y+ (siempre adelante) ───
        move_y = self.STEP_SIZE * self.current_speed_factor

        # ─── Evasión de obstáculos ───
        move_x_avoid = self.compute_avoidance()

        # ─── Centrado en túnel ───
        move_x_center = self.compute_centering()

        # ─── Repulsión de paredes ───
        move_x_walls = self.compute_wall_repulsion()

        # ─── Atracción al offset de la pasada ───
        target_offset = self.LATERAL_OFFSETS[min(self.current_pass, len(self.LATERAL_OFFSETS) - 1)]
        offset_correction = (target_offset - self.pos_x) * 0.02

        # ─── Combinar fuerzas laterales ───
        if abs(move_x_avoid) > 0.001:
            move_x = move_x_avoid + move_x_walls
        else:
            move_x = move_x_center + move_x_walls + offset_correction

        # ─── Oscilación en Z ───
        move_z = self.compute_z_oscillation()

        # ─── Limitar movimientos ───
        move_x = max(-self.LATERAL_STEP * 2, min(self.LATERAL_STEP * 2, move_x))
        move_z = max(-self.Z_OSCILLATION_STEP * 1.5, min(self.Z_OSCILLATION_STEP * 1.5, move_z))

        # ─── Aplicar movimiento ───
        self.pos_x += move_x
        self.pos_y += move_y
        self.pos_z += move_z

        # ─── Límites de Z ───
        self.pos_z = max(self.Z_MIN, min(self.Z_MAX, self.pos_z))

        # ─── Mover el dron ───
        self.move_drone(self.pos_x, self.pos_y, self.pos_z)

        # ─── Tracking ───
        self.total_steps += 1
        self.pass_steps += 1
        step_dist = math.sqrt(move_x**2 + move_y**2 + move_z**2)
        self.total_distance += step_dist
        self.pass_distance += step_dist

        if self.pos_y > self.pass_max_y:
            self.pass_max_y = self.pos_y

        # ─── Log ───
        self.trajectory_log.append({
            'time': time.time() - self.start_time,
            'x': self.pos_x, 'y': self.pos_y, 'z': self.pos_z,
            'pass': self.current_pass + 1,
            'front': self.front_dist, 'left': self.left_dist,
            'right': self.right_dist, 'speed': self.current_speed_factor,
        })

    # ══════════════════════════════════════════════════════════════
    # FIN DE PASADA
    # ══════════════════════════════════════════════════════════════
    def end_pass(self):
        elapsed = time.time() - self.start_time

        self.get_logger().info(
            f'PASADA {self.current_pass + 1} COMPLETADA | '
            f'Y_max={self.pass_max_y:.1f}m | '
            f'Dist={self.pass_distance:.1f}m | '
            f'Steps={self.pass_steps} | '
            f'Tiempo total: {elapsed:.0f}s')

        self.pass_logs.append({
            'pass': self.current_pass + 1,
            'y_max': self.pass_max_y,
            'distance': self.pass_distance,
            'steps': self.pass_steps,
        })

        self.current_pass += 1

        if self.current_pass >= self.MAX_PASSES:
            self.finish_mission()
        else:
            # Teletransporte al inicio para siguiente pasada
            self.state = 'TELEPORTING'
            self.teleport_to_start()

    # ══════════════════════════════════════════════════════════════
    # MISIÓN COMPLETA
    # ══════════════════════════════════════════════════════════════
    def finish_mission(self):
        self.state = 'MISSION_COMPLETE'
        elapsed = time.time() - self.start_time
        avg_speed = self.total_distance / elapsed if elapsed > 0 else 0

        self.get_logger().info('=' * 60)
        self.get_logger().info('  MISIÓN COMPLETA')
        self.get_logger().info('=' * 60)
        self.get_logger().info(f'  Pasadas: {self.MAX_PASSES}')
        self.get_logger().info(f'  Steps totales: {self.total_steps}')
        self.get_logger().info(f'  Distancia total: {self.total_distance:.1f}m')
        self.get_logger().info(f'  Tiempo: {elapsed:.1f}s ({elapsed/60:.1f}min)')
        self.get_logger().info(f'  Vel promedio: {avg_speed:.3f} m/s')
        self.get_logger().info('=' * 60)

        for p in self.pass_logs:
            self.get_logger().info(
                f"  Pasada {p['pass']}: Y_max={p['y_max']:.1f}m | "
                f"Dist={p['distance']:.1f}m | Steps={p['steps']}")

        self.save_metrics()

    # ══════════════════════════════════════════════════════════════
    # STATUS PERIÓDICO
    # ══════════════════════════════════════════════════════════════
    def print_status(self):
        if self.state in ('INIT', 'WAITING', 'TELEPORTING', 'MISSION_COMPLETE'):
            return

        elapsed = time.time() - self.start_time

        self.get_logger().info(
            f'P{self.current_pass+1}/{self.MAX_PASSES} >>> '
            f'Pos=({self.pos_x:.2f}, {self.pos_y:.2f}, {self.pos_z:.2f}) | '
            f'F={self.front_dist:.1f} L={self.left_dist:.1f} R={self.right_dist:.1f} | '
            f'Vel={self.current_speed_factor:.0%} | '
            f'Dist={self.pass_distance:.1f}m | '
            f'{elapsed:.0f}s')

    # ══════════════════════════════════════════════════════════════
    # GUARDAR MÉTRICAS
    # ══════════════════════════════════════════════════════════════
    def save_metrics(self):
        results_dir = os.path.expanduser('~/tesis/resultados_exploracion')
        os.makedirs(results_dir, exist_ok=True)

        timestamp = time.strftime('%Y%m%d_%H%M%S')
        elapsed = time.time() - self.start_time

        # Métricas de texto
        metrics_file = os.path.join(results_dir, f'{timestamp}_metrics.txt')
        with open(metrics_file, 'w') as f:
            f.write('=' * 60 + '\n')
            f.write('EXPLORACIÓN AUTÓNOMA — MÉTRICAS\n')
            f.write('=' * 60 + '\n')
            f.write(f'Fecha: {time.strftime("%Y-%m-%d %H:%M:%S")}\n')
            f.write(f'Pasadas: {self.MAX_PASSES}\n')
            f.write(f'Steps totales: {self.total_steps}\n')
            f.write(f'Distancia total: {self.total_distance:.1f}m\n')
            f.write(f'Tiempo total: {elapsed:.1f}s ({elapsed/60:.1f}min)\n')
            f.write(f'Velocidad promedio: {self.total_distance/elapsed:.3f} m/s\n')
            f.write(f'Step size: {self.STEP_SIZE}m\n')
            f.write(f'Step delay: {self.STEP_DELAY}s\n')
            f.write(f'Oscilación Z: [{self.Z_MIN}, {self.Z_MAX}]m\n')
            f.write(f'Offsets laterales: {self.LATERAL_OFFSETS}\n')
            f.write('\n--- Pasadas ---\n')
            for p in self.pass_logs:
                f.write(
                    f"Pasada {p['pass']}: Y_max={p['y_max']:.1f}m | "
                    f"Dist={p['distance']:.1f}m | Steps={p['steps']}\n")

        self.get_logger().info(f'Métricas: {metrics_file}')

        # Trayectoria CSV
        traj_file = os.path.join(results_dir, f'{timestamp}_trajectory.csv')
        with open(traj_file, 'w') as f:
            f.write('time,x,y,z,pass,front,left,right,speed\n')
            for t in self.trajectory_log:
                f.write(
                    f"{t['time']:.3f},{t['x']:.4f},{t['y']:.4f},{t['z']:.4f},"
                    f"{t['pass']},{t['front']:.2f},"
                    f"{t['left']:.2f},{t['right']:.2f},{t['speed']:.3f}\n")

        self.get_logger().info(f'Trayectoria: {traj_file}')

        # PLY de trayectoria (coloreado por pasada)
        ply_file = os.path.join(results_dir, f'{timestamp}_trajectory.ply')
        colors = [(0, 255, 0), (255, 0, 0), (0, 0, 255)]  # verde, rojo, azul
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
                p = t['pass'] - 1
                r, g, b = colors[min(p, len(colors) - 1)]
                f.write(f"{t['x']:.4f} {t['y']:.4f} {t['z']:.4f} {r} {g} {b}\n")

        self.get_logger().info(f'PLY: {ply_file}')


# ══════════════════════════════════════════════════════════════
# MAIN
# ══════════════════════════════════════════════════════════════
def main():
    rclpy.init()
    node = AutonomousExplorer()
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
