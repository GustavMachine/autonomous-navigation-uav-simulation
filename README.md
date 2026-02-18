# 🚁 Navegación Autónoma de Dron para Inspección de Socavones Mineros

Sistema de navegación autónoma basado en SLAM 3D y LiDAR para la inspección de túneles mineros subterráneos mediante un cuadricóptero.

**Tesis para optar el título profesional — Pontificia Universidad Católica del Perú (PUCP)**

---

## 📋 Descripción

Este proyecto implementa un sistema de navegación autónoma en dos fases para la inspección de socavones mineros utilizando un dron equipado con LiDAR 3D:

- **Fase 1 — Exploración y Mapeo:** El dron recorre el túnel autónomamente generando un mapa 3D mediante SLAM (Cartographer).
- **Fase 2 — Inspección de Paredes:** Utilizando el mapa generado, el dron ejecuta un patrón de zigzag vertical para escanear ambas paredes del túnel.

El sistema fue validado en simulación con Gazebo Classic 11 y ROS2 Humble.

---

## 🏗️ Arquitectura del Sistema

![Arquitectura del Sistema](docs/figuras/diagrama_arquitectura.png)

El sistema se organiza en 4 capas:

| Capa | Componentes | Función |
|------|-------------|---------|
| Percepción | Ouster OS0-128, IMU ICM-42688-P | Adquisición de datos LiDAR e inerciales |
| Localización | Google Cartographer 3D | SLAM, mapa de ocupación, árbol TF |
| Navegación | autonomous_explorer.py, wall_inspector_v4.py | Algoritmos de exploración e inspección |
| Actuación | gz model (Gazebo) | Posicionamiento cinemático del dron |

---

## 📂 Estructura del Repositorio

```
├── README.md
├── src/
│   ├── autonomous_explorer.py      # Fase 1: Exploración y mapeo
│   └── wall_inspector_v4.py        # Fase 2: Inspección de paredes
│
├── config/
│   ├── cartographer.lua             # Configuración de Cartographer SLAM 3D
│   └── cartographer.launch.py       # Launch file de Cartographer para ROS2
│
├── gazebo/
│   ├── tunnel_world_drone.sdf       # Mundo de simulación (túnel + dron)
│   └── models/                      # Modelos Gazebo (túnel, dron, obstáculos)
│
├── resultados/
│   ├── fase1/
│   │   ├── 20260216_210947_metrics.txt
│   │   ├── 20260216_210947_trajectory.csv
│   │   ├── 20260216_210947_trajectory.ply
│   │   ├── 20260216_211232_metrics.txt
│   │   ├── 20260216_211232_trajectory.csv
│   │   ├── 20260216_211232_trajectory.ply
│   │   ├── 20260216_213550_metrics.txt
│   │   ├── 20260216_213550_trajectory.csv
│   │   └── 20260216_213550_trajectory.ply
│   │
│   └── fase2/
│       ├── 20260217_130828_v4_metrics.txt
│       ├── 20260217_130828_v4_trajectory.csv
│       ├── 20260217_130828_v4_trajectory.ply
│       └── 20260217_130828_v4_waypoints.csv
│
├── docs/
│   ├── figuras/
│       ├── diagrama_arquitectura.png
│       ├── diagrama_control.png
│       └── diagrama_flujo_navegacion.png
│  
└── mapa/
    ├── mapa_ocupacion.pgm            # Mapa 2D generado por Cartographer
    └── mapa_ocupacion.yaml           # Metadatos del mapa
    └── tunnel_multipass.pbstream           # Metadatos del mapa
```

---

## 🔧 Hardware

| Componente | Modelo | Especificación |
|-----------|--------|----------------|
| Autopiloto | Pixhawk 6C | STM32H753 480MHz, 2MB Flash |
| Computador de misión | NVIDIA Jetson Orin NX | 16GB RAM, 100 TOPS GPU |
| LiDAR 3D | Ouster OS0-128 | 128 canales, 10-20 Hz, 50m rango, FOV 360°×90° |
| IMU | TDK ICM-42688-P | 6 ejes, 250 Hz, ±16g / ±2000°/s |

---

## 💻 Software y Dependencias
```bash
# ROS2 Humble instalado
# Gazebo Classic 11 instalado
# Cartographer ROS2 instalado
```
# 1) Sistema base
```bash
Ubuntu 22.04 
sudo apt update
sudo apt install ros-humble-ros-base ros-humble-sensor-msgs ros-humble-rclpy
source /opt/ros/humble/setup.bash
sudo apt install gazebo11
sudo apt install ros-humble-gazebo-ros-pkgs
sudo apt install ros-humble-cartographer \
                 ros-humble-cartographer-ros \
                 ros-humble-cartographer-ros-msgs
pip install numpy pillow pyyaml
pip install matplotlib
```
---

## 🚀 Ejecución
```

### Terminal 1 — Simulador

```bash
export GAZEBO_MODEL_PATH=~/gazebo_tunnel/models:$GAZEBO_MODEL_PATH
gzserver ~/gazebo_tunnel/tunnel_world_drone.sdf --verbose
```

### Terminal 2 — Transformadas TF (esperar ~30s)

```bash
ros2 run tf2_ros static_transform_publisher 0.1888 -0.1406 0.0813 0 0 0.2182 base_link os_sensor &
ros2 run tf2_ros static_transform_publisher 0 0 0.036 0 0 0 os_sensor os_lidar &
ros2 run tf2_ros static_transform_publisher 0.1888 0.120 0 0 0 0 base_link imu_link &
```

### Terminal 3 — Cartographer SLAM (esperar ~10s)

```bash
ros2 launch cartographer_ros cartographer.launch.py
```

### Terminal 4 — Navegación autónoma

```bash
# Fase 1: Exploración y mapeo
python3 src/autonomous_explorer.py

# Fase 2: Inspección de paredes (requiere mapa de Fase 1)
python3 src/wall_inspector_v4.py
```

---

## 📊 Resultados

### Fase 1 — Exploración y Mapeo

Se ejecutaron 3 corridas experimentales con 3 pasadas cada una:

| Corrida | Distancia (m) | Tiempo (min) | Vel. prom. (m/s) | Y máx. (m) |
|---------|---------------|-------------|-------------------|-------------|
| 1 | 81.4 | 5.6 | 0.244 | 23.3 |
| 2 | 81.4 | 8.3 | 0.163 | 23.3 |
| 3 | 82.4 | 5.6 | 0.244 | 23.5 |

**Mapa generado:** 503 × 1114 px a resolución 0.05 m/px.

### Fase 2 — Inspección de Paredes

![Algoritmo de Navegación](docs/figuras/diagrama_flujo_navegacion.png)

| Parámetro | Valor |
|-----------|-------|
| Waypoints totales | 142 |
| Waypoints alcanzados | 129 (90.8%) |
| Waypoints saltados | 13 (9.2%) |
| Distancia total | 241.6 m |
| Tiempo de misión | 23.4 min |
| Velocidad promedio | 0.172 m/s |
| Paradas de emergencia | 266 |
| Bloqueos por mapa | 0 |
| Cobertura longitudinal | Y = 0 a 36 m |
| Secciones inspeccionadas | 18 de 19 (94.7%) |

### Sistema de Control

![Sistema de Control](docs/figuras/diagrama_control.png)

El sistema utiliza un modelo cinemático con bucle de control TURN-THEN-MOVE y seguridad reactiva de doble barrera (LiDAR + mapa de ocupación).

---

## ⚠️ Limitaciones Identificadas

- **Modelo cinemático sin colisión:** El dron se mueve por teleportación (gz model), sin motor de física.
- **Mapa con 80% de celdas desconocidas:** Cobertura angular insuficiente en las zonas superiores del túnel.
- **Lecturas LiDAR espurias:** La curvatura del techo del túnel genera falsos obstáculos laterales.
- **Rendimiento de simulación:** Gazebo Classic a RTF = 0.27, LiDAR simulado a 2.5 Hz.
- **Asimetría en inspección:** Pared derecha limitada a centro + 1.2m vs centro − 2.0m para la izquierda.

---

## 📄 Licencia

Este proyecto es parte de una tesis académica de la PUCP. Uso exclusivamente educativo y de investigación.

---

## 👤 Autor

**[Tu nombre completo]**
Pontificia Universidad Católica del Perú
Facultad de Ciencias e Ingeniería
