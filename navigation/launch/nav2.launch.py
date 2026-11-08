"""
nav2.launch.py — pilha de navegação (gestão de rotas).

Requer o TF map->odom (RTAB-Map) e odom->base_link (hardware ou simulação).

Cadeia de velocidade:
  controller_server -> /cmd_vel_nav -> velocity_smoother -> /cmd_vel

O lifecycle_manager (que configura e ativa os nós) sobe `activation_delay`
segundos depois dos outros. Na simulação, um nó ativado antes de receber o
primeiro /clock marca o início da espera pelo TF em t = 0; quando o /clock
chega com o tempo da simulação (centenas de segundos), o costmap acha que
estourou o initial_transform_timeout (60 s), a ativação falha e o
bt_navigator fica inativo recusando todos os destinos.

Uso:
  ros2 launch navigation nav2.launch.py
  ros2 launch navigation nav2.launch.py allow_unknown:=true   # durante mapeamento
  ros2 launch navigation nav2.launch.py use_sim_time:=true
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from nav2_common.launch import RewrittenYaml

LIFECYCLE_NODES = [
    'controller_server',
    'smoother_server',
    'planner_server',
    'behavior_server',
    'bt_navigator',
    'waypoint_follower',
    'velocity_smoother',
]


def generate_launch_description():
    pkg = get_package_share_directory('navigation')
    use_sim_time = LaunchConfiguration('use_sim_time')
    sim_time = ParameterValue(use_sim_time, value_type=bool)

    params = RewrittenYaml(
        source_file=LaunchConfiguration('params_file'),
        param_rewrites={'allow_unknown': LaunchConfiguration('allow_unknown')},
        convert_types=True,
    )

    def nav2_node(package, executable, remappings=()):
        return Node(package=package, executable=executable, name=executable,
                    output='screen', parameters=[params, {'use_sim_time': sim_time}],
                    remappings=list(remappings))

    return LaunchDescription([
        DeclareLaunchArgument('params_file',
                              default_value=os.path.join(pkg, 'config', 'nav2_params.yaml')),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('allow_unknown', default_value='false',
                              description='Planejar por espaço desconhecido '
                                          '(true durante o mapeamento)'),
        DeclareLaunchArgument('activation_delay', default_value='3.0',
                              description='Espera (s) antes de ativar os nós (ver docstring)'),

        nav2_node('nav2_controller', 'controller_server', [('cmd_vel', 'cmd_vel_nav')]),
        nav2_node('nav2_smoother', 'smoother_server'),
        nav2_node('nav2_planner', 'planner_server'),
        nav2_node('nav2_behaviors', 'behavior_server'),
        nav2_node('nav2_bt_navigator', 'bt_navigator'),
        nav2_node('nav2_waypoint_follower', 'waypoint_follower'),
        nav2_node('nav2_velocity_smoother', 'velocity_smoother',
                  [('cmd_vel', 'cmd_vel_nav'), ('cmd_vel_smoothed', 'cmd_vel')]),

        TimerAction(period=LaunchConfiguration('activation_delay'), actions=[Node(
            package='nav2_lifecycle_manager', executable='lifecycle_manager',
            name='lifecycle_manager_navigation', output='screen',
            parameters=[{'use_sim_time': sim_time, 'autostart': True,
                         'node_names': LIFECYCLE_NODES}],
        )]),
    ])
