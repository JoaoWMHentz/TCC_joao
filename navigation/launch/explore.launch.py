"""
explore.launch.py — exploração automática por fronteiras.

Requer o mapeamento rodando (slam.launch.py mode:=mapping) e o Nav2
(nav2.launch.py allow_unknown:=true). Pela interface web, use o botão
"Explorar automaticamente" durante o mapeamento.

Uso:
  ros2 launch navigation explore.launch.py use_sim_time:=true
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    config = os.path.join(get_package_share_directory('navigation'), 'config', 'explore.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('return_to_start', default_value='true'),
        Node(
            package='navigation', executable='frontier_explorer.py', name='frontier_explorer',
            output='screen',
            parameters=[config, {
                'use_sim_time': ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool),
                'return_to_start': ParameterValue(LaunchConfiguration('return_to_start'),
                                                  value_type=bool),
            }],
        ),
    ])
