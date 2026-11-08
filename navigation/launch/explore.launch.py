"""
explore.launch.py — exploração automática.

Estratégias (argumento strategy):
  wall_bfs  (padrão) segue a parede e, quando ela deixa de revelar mapa,
            usa BFS no /map para ir à fronteira alcançável mais próxima
            (scripts/wall_bfs_explorer.py);
  pocket    desbaste, como o pocketing de uma CNC: contorna a parede
            externa e depois faz passes em offset do caminho já percorrido,
            nível a nível, até o centro (scripts/pocket_explorer.py);
  frontier  escolhe fronteiras por nota distância x tamanho e vai sempre
            pelo Nav2 (scripts/frontier_explorer.py).

Requer o mapeamento rodando (slam.launch.py mode:=mapping) e o Nav2
(nav2.launch.py allow_unknown:=true). Pela interface web, use o botão
"Iniciar exploração autônoma" (escolhendo a estratégia) no mapeamento.

Uso:
  ros2 launch navigation explore.launch.py use_sim_time:=true
  ros2 launch navigation explore.launch.py strategy:=pocket
  ros2 launch navigation explore.launch.py strategy:=frontier
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import EqualsSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    config = os.path.join(get_package_share_directory('navigation'), 'config', 'explore.yaml')
    overrides = {
        'use_sim_time': ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool),
        'return_to_start': ParameterValue(LaunchConfiguration('return_to_start'), value_type=bool),
    }

    def explorer(strategy, executable):
        return Node(
            package='navigation', executable=executable, name=executable.removesuffix('.py'),
            output='screen', parameters=[config, overrides],
            condition=IfCondition(EqualsSubstitution(LaunchConfiguration('strategy'), strategy)),
        )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('return_to_start', default_value='true'),
        DeclareLaunchArgument('strategy', default_value='wall_bfs',
                              choices=['wall_bfs', 'pocket', 'frontier'],
                              description='wall_bfs: seguir parede + BFS; pocket: desbaste '
                                          '(contorno + passes em offset); '
                                          'frontier: fronteiras via Nav2'),
        explorer('wall_bfs', 'wall_bfs_explorer.py'),
        explorer('pocket', 'pocket_explorer.py'),
        explorer('frontier', 'frontier_explorer.py'),
    ])
