"""
rviz.launch.py — RViz2 com mapa, costmaps, laser, caminho planejado e robô.

Ferramentas na barra do RViz:
  "2D Pose Estimate" -> /initialpose (corrigir a localização)
  "2D Goal Pose"     -> /goal_pose   (mandar o robô a um ponto)

Uso:
  ros2 launch navigation rviz.launch.py                     # robô real
  ros2 launch navigation rviz.launch.py use_sim_time:=true  # simulação
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    config = os.path.join(get_package_share_directory('navigation'), 'config', 'robot.rviz')
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        Node(
            package='rviz2', executable='rviz2', output='log',
            arguments=['-d', config],
            parameters=[{'use_sim_time': ParameterValue(
                LaunchConfiguration('use_sim_time'), value_type=bool)}],
        ),
    ])
