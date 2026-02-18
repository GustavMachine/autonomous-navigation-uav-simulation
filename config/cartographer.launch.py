import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():

    home = os.path.expanduser('~')
    gazebo_model_path = os.path.join(home, 'gazebo_tunnel', 'models')
    world_file = os.path.join(home, 'gazebo_tunnel', 'mine_tunnel_drone.world')
    px4_dir = os.path.join(home, 'PX4-Autopilot')

    # === ARGUMENTOS ===
    headless = LaunchConfiguration('headless')
    declare_headless = DeclareLaunchArgument('headless', default_value='true')

    # === VARIABLES DE ENTORNO ===
    # Agregar nuestros modelos al path de Gazebo
    gazebo_model_env = SetEnvironmentVariable(
        'GAZEBO_MODEL_PATH',
        gazebo_model_path + ':' + os.environ.get('GAZEBO_MODEL_PATH', '')
    )

    software_render = SetEnvironmentVariable(
        'LIBGL_ALWAYS_SOFTWARE', '1'
    )

    # === 1. GAZEBO SERVER (headless) ===
    gzserver = ExecuteProcess(
        cmd=[
            'gzserver', world_file,
            '--verbose',
            '-s', 'libgazebo_ros_init.so',
            '-s', 'libgazebo_ros_factory.so',
        ],
        output='screen',
    )

    # === 2. PX4 SITL ===
    px4_sitl = ExecuteProcess(
        cmd=[
            os.path.join(px4_dir, 'build', 'px4_sitl_default', 'bin', 'px4'),
            os.path.join(px4_dir, 'ROMFS', 'px4fmu_common'),
            '-s', os.path.join(px4_dir, 'ROMFS', 'px4fmu_common', 'init.d-posix', 'rcS'),
            '-t', os.path.join(px4_dir, 'test_data'),
        ],
        cwd=os.path.join(px4_dir, 'build', 'px4_sitl_default'),
        output='screen',
        additional_env={
            'PX4_SIM_MODEL': 'gazebo-classic_iris',
        },
    )

    # === 3. MAVROS (puente PX4 <-> ROS2) ===
    mavros_node = TimerAction(
        period=10.0,
        actions=[
            Node(
                package='mavros',
                executable='mavros_node',
                name='mavros',
                output='screen',
                parameters=[{
                    'fcu_url': 'udp://:14540@127.0.0.1:14557',
                    'gcs_url': '',
                    'target_system_id': 1,
                    'target_component_id': 1,
                    'fcu_protocol': 'v2.0',
                    'use_sim_time': True,
                }],
            )
        ],
    )

    # === 4. TF ESTATICOS (sensor frames) ===
    # base_link -> os_sensor (LiDAR mount)
    tf_lidar = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tf_base_to_os_sensor',
        arguments=[
            '--x', '0.0',
            '--y', '0.0',
            '--z', '0.075',
            '--roll', '0', '--pitch', '0', '--yaw', '0',
            '--frame-id', 'base_link',
            '--child-frame-id', 'os_sensor',
        ],
        parameters=[{'use_sim_time': True}],
    )

    # os_sensor -> os_lidar
    tf_os_lidar = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tf_os_sensor_to_lidar',
        arguments=[
            '--x', '0.0',
            '--y', '0.0',
            '--z', '0.036',
            '--roll', '0', '--pitch', '0', '--yaw', '0',
            '--frame-id', 'os_sensor',
            '--child-frame-id', 'os_lidar',
        ],
        parameters=[{'use_sim_time': True}],
    )

    # os_sensor -> os_imu
    tf_os_imu = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tf_os_sensor_to_imu',
        arguments=[
            '--x', '0.006',
            '--y', '-0.012',
            '--z', '0.008',
            '--roll', '0', '--pitch', '0', '--yaw', '0',
            '--frame-id', 'os_sensor',
            '--child-frame-id', 'os_imu',
        ],
        parameters=[{'use_sim_time': True}],
    )

    # base_link -> imu_link
    tf_imu = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tf_base_to_imu',
        arguments=[
            '--x', '0.0',
            '--y', '0.0',
            '--z', '0.136',
            '--roll', '0', '--pitch', '0', '--yaw', '0',
            '--frame-id', 'base_link',
            '--child-frame-id', 'imu_link',
        ],
        parameters=[{'use_sim_time': True}],
    )

    # === 5. CARTOGRAPHER (SLAM) ===
    cartographer_node = TimerAction(
        period=15.0,
        actions=[
            Node(
                package='cartographer_ros',
                executable='cartographer_node',
                name='cartographer_node',
                output='screen',
                parameters=[{'use_sim_time': True}],
                arguments=[
                    '-configuration_directory', os.path.join(home, 'config'),
                    '-configuration_basename', 'tunnel_3d.lua',
                ],
                remappings=[
                    ('points2', '/ouster/points'),
                    ('imu', '/imu/data'),
                ],
            )
        ],
    )

    # === 6. OCCUPANCY GRID (mapa 2D) ===
    occupancy_grid = TimerAction(
        period=15.0,
        actions=[
            Node(
                package='cartographer_ros',
                executable='cartographer_occupancy_grid_node',
                name='occupancy_grid_node',
                output='screen',
                parameters=[
                    {'use_sim_time': True},
                    {'resolution': 0.05},
                    {'publish_period_sec': 2.0},
                ],
            )
        ],
    )

    return LaunchDescription([
        declare_headless,
        gazebo_model_env,
        software_render,
        gzserver,
        px4_sitl,
        mavros_node,
        tf_lidar,
        tf_os_lidar,
        tf_os_imu,
        tf_imu,
        cartographer_node,
        occupancy_grid,
    ])
