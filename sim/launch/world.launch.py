"""
world.launch.py — mundo Gazebo Harmonic com o robô.

Sobe: gz sim + ponte ROS<->gz + relay /cmd_vel (watchdog) + robô
(+ pessoas caminhando, com people:=true).

Mundos: supermercado (padrão, 30 x 20 m) e loja (12 x 8 m). A posição
inicial do robô e os caminhos das pessoas de cada mundo ficam em
config/mundos/<world>.yaml.

As pessoas são <actor> do Gazebo: humanos animados, vistos pelo LiDAR e pela
câmera de profundidade (sensores por renderização), mas SEM colisão física.

O mundo fica aberto a sessão inteira; a pilha de software do robô
(robot_sim.launch.py) pode ser reiniciada sem derrubá-lo, o que mantém o
relógio /clock contínuo.

Uso:
  ros2 launch sim world.launch.py
  ros2 launch sim world.launch.py people:=true gui:=false
  ros2 launch sim world.launch.py world:=loja
"""

import math
import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (AppendEnvironmentVariable, DeclareLaunchArgument, ExecuteProcess,
                            IncludeLaunchDescription, OpaqueFunction, RegisterEventHandler,
                            TimerAction)
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
    # Malha das pessoas: "model://humano/..."
    models_path = AppendEnvironmentVariable('GZ_SIM_RESOURCE_PATH',
                                            os.path.join(pkg_sim, 'models'))

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

    return LaunchDescription([
        DeclareLaunchArgument('world', default_value='supermercado',
                              description='Mundo: supermercado ou loja (sim/worlds/<world>.sdf)'),
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('people', default_value='false',
                              description='Adiciona pessoas caminhando (config/mundos/<world>.yaml)'),
        # Vazio = posição de config/mundos/<world>.yaml
        DeclareLaunchArgument('spawn_x', default_value=''),
        DeclareLaunchArgument('spawn_y', default_value=''),
        DeclareLaunchArgument('spawn_yaw', default_value=''),
        resource_path,
        models_path,
        gazebo,
        bridge,
        relay,
        TimerAction(period=3.0, actions=[OpaqueFunction(function=_spawn)]),
    ])


def _world_config(context):
    """config/mundos/<world>.yaml: posição inicial do robô e pessoas."""
    world = LaunchConfiguration('world').perform(context)
    path = os.path.join(get_package_share_directory('sim'), 'config', 'mundos', f'{world}.yaml')
    if not os.path.exists(path):
        return world, {}
    with open(path) as f:
        return world, yaml.safe_load(f) or {}


def _spawn(context):
    """Cria o robô (e as pessoas, com people:=true) e depois abre a janela."""
    world, cfg = _world_config(context)
    robo = cfg.get('robo', {})

    def pose(name, key, default):
        v = LaunchConfiguration(name).perform(context)
        return v if v else str(robo.get(key, default))

    robot_urdf = Command(['xacro ', os.path.join(get_package_share_directory('sim'),
                                                 'urdf', 'robot_sim.urdf.xacro')])
    spawn_robot = Node(
        package='ros_gz_sim', executable='create', output='screen',
        arguments=['-world', world, '-name', 'tcc_robot', '-string', robot_urdf,
                   '-x', pose('spawn_x', 'x', 0.0), '-y', pose('spawn_y', 'y', 0.0),
                   '-z', '0.02', '-Y', pose('spawn_yaw', 'yaw', 0.0)],
    )

    # Janela do Gazebo: aberta só depois que o robô foi criado. Se abrir antes,
    # ela pode perder o aviso do modelo novo e o robô não aparece. Fechar a
    # janela não encerra a simulação (reabra com: gz sim -g).
    gui = ExecuteProcess(
        cmd=['gz', 'sim', '-g', '-v', '1'], name='gazebo_gui', output='log',
        condition=IfCondition(LaunchConfiguration('gui')),
    )
    actions = [spawn_robot,
               RegisterEventHandler(OnProcessExit(target_action=spawn_robot, on_exit=[gui]))]

    if LaunchConfiguration('people').perform(context).lower() == 'true':
        actions += [
            Node(package='ros_gz_sim', executable='create', output='screen',
                 arguments=['-world', world, '-name', p['nome'],
                            '-string', _actor_sdf(p['nome'], p['caminho'],
                                                  p.get('velocidade', 0.5), p.get('atraso', 0.0))])
            for p in cfg.get('pessoas', [])
        ]
    return actions


# ---------------------------------------------------------------- pessoas

ALTURA_ACTOR = 1.0     # z da origem da malha walk.dae (pés no chão)
TEMPO_GIRO = 1.0       # s para a pessoa dar meia-volta / virar num canto


def _actor_sdf(nome, caminho, velocidade, atraso):
    """<actor> que anda em vai-e-vem pelo caminho, com a animação de caminhada."""
    ida = [tuple(map(float, p)) for p in caminho]
    pontos = ida + ida[-2::-1]           # ida e volta, fechando no ponto inicial

    waypoints, t, yaw_ant = [], 0.0, None
    for (x0, y0), (x1, y1) in zip(pontos, pontos[1:]):
        yaw = math.atan2(y1 - y0, x1 - x0)
        if yaw_ant is not None:
            # gira parado, pelo menor ângulo (evita volta de 360°)
            yaw = yaw_ant + math.remainder(yaw - yaw_ant, 2 * math.pi)
            t += TEMPO_GIRO
        waypoints.append((t, x0, y0, yaw))
        t += math.hypot(x1 - x0, y1 - y0) / velocidade
        waypoints.append((t, x1, y1, yaw))
        yaw_ant = yaw
    # gira de volta para a direção inicial antes de repetir o ciclo
    yaw0 = waypoints[0][3]
    yaw_fim = yaw_ant + math.remainder(yaw0 - yaw_ant, 2 * math.pi)
    t += TEMPO_GIRO
    waypoints.append((t, *pontos[-1], yaw_fim))

    wp_xml = '\n'.join(
        f'<waypoint><time>{t:.3f}</time>'
        f'<pose>{x:.3f} {y:.3f} {ALTURA_ACTOR} 0 0 {yaw:.4f}</pose></waypoint>'
        for t, x, y, yaw in waypoints)
    malha = 'model://humano/meshes/walk.dae'
    return f"""<?xml version="1.0"?>
<sdf version="1.10">
  <actor name="{nome}">
    <skin><filename>{malha}</filename><scale>1.0</scale></skin>
    <animation name="walk">
      <filename>{malha}</filename>
      <interpolate_x>true</interpolate_x>
    </animation>
    <script>
      <loop>true</loop>
      <delay_start>{float(atraso):.2f}</delay_start>
      <auto_start>true</auto_start>
      <trajectory id="0" type="walk">
{wp_xml}
      </trajectory>
    </script>
  </actor>
</sdf>"""
