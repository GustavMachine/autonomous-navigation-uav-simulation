-- ============================================================
-- tunnel_sim_3d_v2.lua
-- Cartographer 3D config MEJORADA para túnel simulado
-- Cambios vs v1:
--   - max_range reducido (25→15m) para eliminar puntos fuera del túnel
--   - motion_filter menos agresivo (conserva más nodos)
--   - optimize_every_n_nodes reducido (60→30) para más loop closures
--   - voxel_filter más fino para mejor resolución
--   - min_score aumentado (0.65→0.55) pero con mejor filtrado previo
--   - Submaps más pequeños para mejor resolución local
-- ============================================================

include "map_builder.lua"
include "trajectory_builder.lua"

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,
  map_frame = "map",
  tracking_frame = "imu_link",
  published_frame = "base_link",
  odom_frame = "odom",
  provide_odom_frame = true,
  publish_frame_projected_to_2d = false,
  use_odometry = false,
  use_nav_sat = false,
  use_landmarks = false,
  num_laser_scans = 0,
  num_multi_echo_laser_scans = 0,
  num_subdivisions_per_laser_scan = 1,
  num_point_clouds = 1,
  lookup_transform_timeout_sec = 0.5,
  submap_publish_period_sec = 0.3,
  pose_publish_period_sec = 5e-3,
  trajectory_publish_period_sec = 30e-3,
  rangefinder_sampling_ratio = 1.0,
  odometry_sampling_ratio = 1.0,
  fixed_frame_pose_sampling_ratio = 1.0,
  imu_sampling_ratio = 1.0,
  landmarks_sampling_ratio = 1.0,
}

-- ============================================================
-- TRAJECTORY BUILDER 3D
-- ============================================================
TRAJECTORY_BUILDER_3D.num_accumulated_range_data = 1

-- --- Rango del LiDAR ---
-- CLAVE: reducir max_range elimina puntos que "escapan" del túnel
-- Un túnel de 3-6m de ancho no necesita más de 12-15m de rango
TRAJECTORY_BUILDER_3D.min_range = 0.5        -- era 0.4, subir un poco para evitar ruido cercano
TRAJECTORY_BUILDER_3D.max_range = 12.0       -- era 25.0 → REDUCIDO para eliminar outliers lejanos

-- --- Voxel filters ---
-- Más finos = mejor resolución pero más lento
-- Para túnel, 0.08m da buen balance
TRAJECTORY_BUILDER_3D.high_resolution_adaptive_voxel_filter.max_length = 2.0
TRAJECTORY_BUILDER_3D.high_resolution_adaptive_voxel_filter.min_num_points = 150
TRAJECTORY_BUILDER_3D.high_resolution_adaptive_voxel_filter.max_range = 12.0

TRAJECTORY_BUILDER_3D.low_resolution_adaptive_voxel_filter.max_length = 4.0
TRAJECTORY_BUILDER_3D.low_resolution_adaptive_voxel_filter.min_num_points = 100
TRAJECTORY_BUILDER_3D.low_resolution_adaptive_voxel_filter.max_range = 12.0

-- --- Tamaño de voxel en submaps ---
-- Más fino = más detalle pero más memoria
TRAJECTORY_BUILDER_3D.submaps.high_resolution = 0.08          -- era default 0.10
TRAJECTORY_BUILDER_3D.submaps.high_resolution_max_range = 10.0
TRAJECTORY_BUILDER_3D.submaps.low_resolution = 0.30           -- era default 0.45

-- --- Submaps más pequeños para mejor resolución local ---
-- Menos range_data por submap = submaps más compactos y precisos
TRAJECTORY_BUILDER_3D.submaps.num_range_data = 80             -- era default 160

-- --- Scan matching ---
-- Pesos para el matching de alta y baja resolución
TRAJECTORY_BUILDER_3D.ceres_scan_matcher.occupied_space_weight_0 = 1.0   -- high-res
TRAJECTORY_BUILDER_3D.ceres_scan_matcher.occupied_space_weight_1 = 6.0   -- low-res
TRAJECTORY_BUILDER_3D.ceres_scan_matcher.translation_weight = 5.0
TRAJECTORY_BUILDER_3D.ceres_scan_matcher.rotation_weight = 4e2

-- --- IMU ---
TRAJECTORY_BUILDER_3D.imu_gravity_time_constant = 10.0

-- --- Motion filter ---
-- CLAVE: menos agresivo = conserva más nodos = mapa más denso
-- v1 conservaba solo 18.2% de nodos (muy bajo)
TRAJECTORY_BUILDER_3D.motion_filter.max_time_seconds = 0.3    -- era 0.5
TRAJECTORY_BUILDER_3D.motion_filter.max_distance_meters = 0.08 -- era default 0.1 (reducido)
TRAJECTORY_BUILDER_3D.motion_filter.max_angle_radians = 0.004  -- ~0.23° (era default 0.004)

-- ============================================================
-- MAP BUILDER (optimización global)
-- ============================================================
MAP_BUILDER.use_trajectory_builder_3d = true
MAP_BUILDER.num_background_threads = 6       -- usar 6 de tus 8 cores para optimización

-- ============================================================
-- POSE GRAPH (loop closure y optimización)
-- ============================================================
-- Optimizar más frecuentemente para mejor consistencia global
POSE_GRAPH.optimize_every_n_nodes = 25       -- era 60 → más frecuente

-- --- Constraint builder ---
-- sampling_ratio: qué fracción de nodos se evalúan para loop closure
POSE_GRAPH.constraint_builder.sampling_ratio = 0.4             -- era default 0.3
POSE_GRAPH.constraint_builder.max_constraint_distance = 10.0   -- metros
POSE_GRAPH.constraint_builder.min_score = 0.55                 -- score mínimo para aceptar constraint

-- --- Fast correlative scan matcher (para encontrar loop closures) ---
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher_3d.linear_xy_search_window = 3.0
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher_3d.linear_z_search_window = 2.0
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher_3d.angular_search_window = math.rad(20.0)
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher_3d.min_rotational_score = 0.6
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher_3d.min_low_resolution_score = 0.5

-- --- Ceres para refinamiento de constraints ---
POSE_GRAPH.constraint_builder.ceres_scan_matcher_3d.occupied_space_weight_0 = 5.0
POSE_GRAPH.constraint_builder.ceres_scan_matcher_3d.occupied_space_weight_1 = 30.0
POSE_GRAPH.constraint_builder.ceres_scan_matcher_3d.translation_weight = 10.0
POSE_GRAPH.constraint_builder.ceres_scan_matcher_3d.rotation_weight = 1e2

-- --- Pesos de optimización global ---
POSE_GRAPH.optimization_problem.huber_scale = 5e2
POSE_GRAPH.optimization_problem.acceleration_weight = 1e3
POSE_GRAPH.optimization_problem.rotation_weight = 3e5

-- Log de constraints para diagnóstico
POSE_GRAPH.constraint_builder.log_matches = true
POSE_GRAPH.optimization_problem.log_solver_summary = true

-- Global localization: buscar loop closures globales
POSE_GRAPH.global_sampling_ratio = 0.003
POSE_GRAPH.global_constraint_search_after_n_seconds = 10.0

return options
