"""
robot_sim.launch.py — equivalente simulado do bringup/robot.launch.py.

Sobe o software do robô sobre o Gazebo: robot_state_publisher, filtro do
laser e a nuvem de pontos da câmera. Com start_world:=true (padrão) também
abre o mundo (world.launch.py).

Os tópicos resultantes são os mesmos do robô físico, então o RTAB-Map e o
Nav2 sobem com os launches do pacote `navigation` e use_sim_time:=true.

Uso:
  ros2 launch sim robot_sim.launch.py
  ros2 launch sim robot_sim.launch.py start_world:=false   # mundo já aberto
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_sim = get_package_share_directory('sim')
    pkg_bringup = get_package_share_directory('bringup')
    arg = LaunchConfiguration
    sim_time = {'use_sim_time': True}

    world = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_sim, 'launch', 'world.launch.py')),
        launch_arguments={'world': arg('world'), 'gui': arg('gui'),
                          'people': arg('people')}.items(),
        condition=IfCondition(arg('start_world')),
    )

    robot_description = ParameterValue(
        Command(['xacro ', os.path.join(pkg_sim, 'urdf', 'robot_sim.urdf.xacro')]),
        value_type=str)

    robot_state_publisher = Node(
        package='robot_state_publisher', executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description, **sim_time}],
    )

    scan_filter = Node(
        package='laser_filters', executable='scan_to_scan_filter_chain',
        name='scan_filter_chain',
        parameters=[os.path.join(pkg_bringup, 'config', 'lidar_filters.yaml'), sim_time],
        remappings=[('scan', '/scan_raw'), ('scan_filtered', '/scan')],
    )

    # Nuvem /camera/depth/points (usada pelos costmaps), como o driver real publica
    point_cloud = Node(
        package='rtabmap_util', executable='point_cloud_xyz', name='point_cloud_xyz',
        ros_arguments=['--log-level', 'warn'],
        parameters=[{'decimation': 4, 'max_depth': 2.5, 'approx_sync': False, **sim_time}],
        remappings=[('depth/image', '/camera/depth/image_raw'),
                    ('depth/camera_info', '/camera/color/camera_info'),
                    ('cloud', '/camera/depth/points')],
    )

    return LaunchDescription([
        DeclareLaunchArgument('start_world', default_value='true'),
        DeclareLaunchArgument('world', default_value='loja'),
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('people', default_value='false'),
        world,
        robot_state_publisher,
        scan_filter,
        point_cloud,
    ])
