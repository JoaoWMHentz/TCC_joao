"""Configuração comum dos testes."""

import os
import sys

import pytest

HERE = os.path.dirname(__file__)
for sub in (('navigation', 'scripts'), ('hardware',)):
    path = os.path.join(HERE, '..', *sub)
    if path not in sys.path:
        sys.path.insert(0, path)


@pytest.fixture(scope='session')
def ros():
    """Contexto do rclpy para os testes que criam nós (pula sem ROS 2)."""
    rclpy = pytest.importorskip(
        'rclpy', reason='ROS 2 não carregado (source /opt/ros/<distro>/setup.bash)')
    rclpy.init()
    yield rclpy
    rclpy.try_shutdown()
