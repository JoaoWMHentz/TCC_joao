"""Configuração do backend, lida de variáveis de ambiente."""

import os
import pathlib

# Carrega um .env na raiz do repositório, se existir (sem sobrescrever o ambiente)
_ENV_FILE = pathlib.Path(__file__).resolve().parents[2] / '.env'
if _ENV_FILE.exists():
    for _line in _ENV_FILE.read_text().splitlines():
        _line = _line.strip()
        if _line and not _line.startswith('#') and '=' in _line:
            _key, _value = _line.split('=', 1)
            os.environ.setdefault(_key.strip(), _value.strip())

# Modo simulação: a "base" do robô é o Gazebo em vez do hardware real
SIM = os.environ.get('ROBOT_SIM', '0') == '1'

ROS_SETUP = os.environ.get('ROS_SETUP', '/opt/ros/jazzy/setup.bash')
WS_SETUP = os.environ.get('WS_SETUP', os.path.expanduser('~/ros2_ws/install/setup.bash'))

MAPS_DIR = pathlib.Path(os.path.expanduser(
    os.environ.get('MAPS_DIR', '~/tcc_maps_sim' if SIM else '~/tcc_maps')))

DATABASE_URL = os.environ.get('DATABASE_URL', 'postgresql://tcc:tcc@localhost:5433/tcc')

USE_CAMERA = os.environ.get('USE_CAMERA', 'true').lower() == 'true'

HOST = os.environ.get('WEB_HOST', '0.0.0.0')
PORT = int(os.environ.get('WEB_PORT', '5000'))
