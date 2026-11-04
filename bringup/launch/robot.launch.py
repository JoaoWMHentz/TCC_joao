"""
robot.launch.py — sobe o robô físico (camada de percepção + atuação).

Nós:
  robot_state_publisher  URDF -> TF estático dos sensores
  hardware_node          motores + IMU -> /odom, /imu/data, TF odom->base_link
  sllidar_node           RPLidar -> /scan_raw
  scan_filter_chain      /scan_raw -> /scan (remove ecos da própria estrutura)
  astra_camera_node      Orbbec Astra -> /camera/color/*, /camera/depth/*
  foxglove_bridge        visualização remota (opcional)

O mapeamento/localização (RTAB-Map) e a navegação (Nav2) sobem à parte, pelos
launches do pacote `navigation`, os mesmos usados na simulação.

Uso:
  ros2 launch bringup robot.launch.py
  ros2 launch bringup robot.launch.py use_camera:=false use_foxglove:=true
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


# Perfis da câmera Orbbec (driver astra_camera / OpenNI2)
CAMERA_PROFILES = {
    'astra_pro': {
        'depth_registration': True,
        'depth_width': 640, 'depth_height': 480, 'depth_fps': 15,
        'use_uvc_camera': True, 'uvc_vendor_id': 0x2bc5, 'uvc_product_id': 0x0501,
    },
    'astra_mini': {
        'depth_registration': True,
        'depth_width': 320, 'depth_height': 240, 'depth_fps': 30,
        'use_uvc_camera': False,
    },
}


def generate_launch_description():
    pkg_description = get_package_share_directory('description')
    pkg_bringup = get_package_share_directory('bringup')

    arg = LaunchConfiguration
    args = [
        DeclareLaunchArgument('motor_port', default_value='/dev/ttyMOTOR'),
        DeclareLaunchArgument('imu_port', default_value='/dev/ttyIMU'),
        DeclareLaunchArgument('lidar_port', default_value='/dev/ttyLIDAR'),
        DeclareLaunchArgument('lidar_baudrate', default_value='460800',
                              description='RPLidar C1/A3/S: 460800 · A1/A2: 115200'),
        DeclareLaunchArgument('use_imu', default_value='true'),
        DeclareLaunchArgument('use_camera', default_value='true'),
        DeclareLaunchArgument('camera_model', default_value='astra_pro',
                              description=' | '.join(CAMERA_PROFILES)),
        DeclareLaunchArgument('use_foxglove', default_value='false'),
    ]

    robot_description = ParameterValue(
        Command(['xacro ', os.path.join(pkg_description, 'urdf', 'robot.urdf.xacro')]),
        value_type=str)

    robot_state_publisher = Node(
        package='robot_state_publisher', executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description}],
    )

    hardware_node = Node(
        package='hardware', executable='hardware_node', output='screen',
        parameters=[
            os.path.join(pkg_bringup, 'config', 'hardware.yaml'),
            {
                'motor_port': arg('motor_port'),
                'imu_port': arg('imu_port'),
                'use_imu': ParameterValue(arg('use_imu'), value_type=bool),
            },
        ],
    )

    lidar = Node(
        package='sllidar_ros2', executable='sllidar_node', output='screen',
        parameters=[{
            'serial_port': arg('lidar_port'),
            'serial_baudrate': ParameterValue(arg('lidar_baudrate'), value_type=int),
            'frame_id': 'lidar_scan_link',
            'angle_compensate': True,
            'scan_mode': 'Standard',
        }],
        remappings=[('scan', '/scan_raw')],
        respawn=True, respawn_delay=4.0,
    )

    scan_filter = Node(
        package='laser_filters', executable='scan_to_scan_filter_chain',
        name='scan_filter_chain',
        parameters=[os.path.join(pkg_bringup, 'config', 'lidar_filters.yaml')],
        remappings=[('scan', '/scan_raw'), ('scan_filtered', '/scan')],
    )

    camera_nodes = [
        Node(
            package='astra_camera', executable='astra_camera_node',
            name='camera', namespace='camera', output='log',
            parameters=[{
                'camera_name': 'camera',
                'enable_color': True, 'enable_depth': True, 'enable_ir': False,
                'enable_point_cloud': True, 'enable_colored_point_cloud': False,
                'color_width': 640, 'color_height': 480, 'color_fps': 30,
                'publish_tf': False,  # os frames vêm do URDF
                **profile,
            }],
            condition=IfCondition(PythonExpression([
                "'", arg('use_camera'), "' == 'true' and '",
                arg('camera_model'), f"' == '{name}'"])),
            respawn=True, respawn_delay=4.0,
        )
        for name, profile in CAMERA_PROFILES.items()
    ]

    foxglove = Node(
        package='foxglove_bridge', executable='foxglove_bridge', output='log',
        parameters=[{'port': 8765, 'address': '0.0.0.0'}],
        condition=IfCondition(arg('use_foxglove')),
    )

    return LaunchDescription([
        *args,
        robot_state_publisher,
        hardware_node,
        lidar,
        scan_filter,
        *camera_nodes,
        foxglove,
    ])
