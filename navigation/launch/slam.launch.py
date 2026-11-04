"""
slam.launch.py — RTAB-Map em modo mapeamento ou localização.

Usado igual no robô físico e na simulação (use_sim_time:=true). Espera que
os sensores já estejam publicando: /scan, /odom (+TF odom->base_link),
/imu/data e, com câmera, /camera/color/* e /camera/depth/image_raw.

Uso:
  ros2 launch navigation slam.launch.py mode:=mapping db_path:=~/maps/loja.db
  ros2 launch navigation slam.launch.py mode:=localization db_path:=~/maps/loja.db
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch_ros.actions import Node


def _rtabmap(context):
    cfg = os.path.join(get_package_share_directory('navigation'), 'config')
    lc = context.launch_configurations
    mode = lc['mode']
    if mode not in ('mapping', 'localization'):
        raise RuntimeError(f"mode deve ser 'mapping' ou 'localization' (recebido '{mode}')")
    use_camera = lc['use_camera'].lower() == 'true'
    use_sim_time = lc['use_sim_time'].lower() == 'true'

    overrides = {
        'database_path': os.path.expanduser(lc['db_path']),
        'use_sim_time': use_sim_time,
        # Sem câmera o RTAB-Map não pode esperar imagens no sincronizador
        'subscribe_rgb': use_camera,
        'subscribe_depth': use_camera,
    }
    if mode == 'localization':
        # Com câmera, a relocalização usa a aparência (visual); sem ela, ICP.
        overrides['Reg/Strategy'] = '0' if use_camera else '1'

    return [Node(
        package='rtabmap_slam', executable='rtabmap', name='rtabmap', output='screen',
        ros_arguments=['--log-level', 'warn'],
        parameters=[
            os.path.join(cfg, 'rtabmap_common.yaml'),
            os.path.join(cfg, f'rtabmap_{mode}.yaml'),
            overrides,
        ],
        remappings=[
            ('rgb/image', '/camera/color/image_raw'),
            ('rgb/camera_info', '/camera/color/camera_info'),
            ('depth/image', '/camera/depth/image_raw'),
            ('scan', '/scan'),
            ('odom', '/odom'),
            ('imu', '/imu/data'),
        ],
    )]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('mode', default_value='mapping',
                              description='mapping | localization'),
        DeclareLaunchArgument('db_path', default_value='~/maps/mapa.db',
                              description='Banco de dados do mapa (.db do RTAB-Map)'),
        DeclareLaunchArgument('use_camera', default_value='true',
                              description='Usar a câmera RGB-D (fechamento de loop visual)'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        OpaqueFunction(function=_rtabmap),
    ])
