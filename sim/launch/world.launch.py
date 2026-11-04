"""
world.launch.py — mundo Gazebo Harmonic com o robô.

Sobe: gz sim + ponte ROS<->gz + relay /cmd_vel (watchdog) + robô
(+ "pessoa" caminhando, com people:=true).

O mundo fica aberto a sessão inteira; a pilha de software do robô
(robot_sim.launch.py) pode ser reiniciada sem derrubá-lo, o que mantém o
relógio /clock contínuo.

Uso:
  ros2 launch sim world.launch.py
  ros2 launch sim world.launch.py people:=true gui:=false
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (AppendEnvironmentVariable, DeclareLaunchArgument, ExecuteProcess,
                            IncludeLaunchDescription, RegisterEventHandler, TimerAction)
from launch.event_handlers import OnProcessExit
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (Command, LaunchConfiguration, PathJoinSubstitution,
                                  PythonExpression)
from launch_ros.actions import Node


def generate_launch_description():
    pkg_sim = get_package_share_directory('sim')
    arg = LaunchConfiguration

    world_file = PathJoinSubstitution([pkg_sim, 'worlds', [arg('world'), '.sdf']])
    # Só o servidor (-s), rodando (-r); a janela sobe à parte, depois do robô.
    # Sem janela, os sensores renderizam via EGL (--headless-rendering).
    gz_args = [world_file, ' -r -s -v 2',
               PythonExpression(["'' if '", arg('gui'), "' == 'true' else ' --headless-rendering'"])]

    # Malhas "package://description/..." resolvidas pelo Gazebo
    resource_path = AppendEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH',
        os.path.dirname(get_package_share_directory('description')))

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': gz_args, 'on_exit_shutdown': 'true'}.items(),
    )

    bridge = Node(
        package='ros_gz_bridge', executable='parameter_bridge', output='screen',
        parameters=[{'config_file': os.path.join(pkg_sim, 'config', 'ros_gz_bridge.yaml'),
                     'use_sim_time': True}],
    )

    relay = Node(
        package='sim', executable='cmd_vel_relay.py', output='screen',
        parameters=[{'cmd_vel_timeout': 0.5, 'use_sim_time': True}],
    )

    robot_urdf = Command(['xacro ', os.path.join(pkg_sim, 'urdf', 'robot_sim.urdf.xacro')])
    spawn_robot = Node(
        package='ros_gz_sim', executable='create', output='screen',
        arguments=['-world', arg('world'), '-name', 'tcc_robot', '-string', robot_urdf,
                   '-x', arg('spawn_x'), '-y', arg('spawn_y'), '-z', '0.02',
                   '-Y', arg('spawn_yaw')],
    )
    spawn_person = Node(
        package='ros_gz_sim', executable='create', output='screen',
        arguments=['-world', arg('world'), '-name', 'pessoa',
                   '-file', os.path.join(pkg_sim, 'models', 'pessoa.sdf'),
                   '-x', '-3.0', '-y', '-1.8', '-z', '0.0'],
        condition=IfCondition(arg('people')),
    )

    # Janela do Gazebo: aberta só depois que o robô foi criado. Se abrir antes,
    # ela pode perder o aviso do modelo novo e o robô não aparece. Fechar a
    # janela não encerra a simulação (reabra com: gz sim -g).
    gui = ExecuteProcess(
        cmd=['gz', 'sim', '-g', '-v', '1'], name='gazebo_gui', output='log',
        condition=IfCondition(arg('gui')),
    )
    gui_after_spawn = RegisterEventHandler(OnProcessExit(target_action=spawn_robot, on_exit=[gui]))

    return LaunchDescription([
        DeclareLaunchArgument('world', default_value='loja'),
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('people', default_value='false',
                              description='Adiciona uma pessoa caminhando (obstáculo dinâmico)'),
        DeclareLaunchArgument('spawn_x', default_value='-4.0'),
        DeclareLaunchArgument('spawn_y', default_value='-0.8'),
        DeclareLaunchArgument('spawn_yaw', default_value='0.0'),
        resource_path,
        gazebo,
        bridge,
        relay,
        gui_after_spawn,
        TimerAction(period=3.0, actions=[spawn_robot, spawn_person]),
    ])
