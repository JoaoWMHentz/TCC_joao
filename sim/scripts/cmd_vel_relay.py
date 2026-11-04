#!/usr/bin/env python3
"""
cmd_vel_relay.py — /cmd_vel -> /sim/cmd_vel com watchdog.

Reproduz na simulação a proteção do hardware_node real: sem comando novo por
`cmd_vel_timeout` segundos, o robô para. Sem isso o DiffDrive do Gazebo
manteria o último comando indefinidamente.
"""

import rclpy
from geometry_msgs.msg import Twist
from rclpy.clock import Clock
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node


class CmdVelRelay(Node):

    def __init__(self):
        super().__init__('cmd_vel_relay')
        self.declare_parameter('cmd_vel_timeout', 0.5)
        self._timeout = float(self.get_parameter('cmd_vel_timeout').value)
        self._wall = Clock()  # relógio de parede: independe da velocidade da simulação
        self._last = None
        self._moving = False
        self._pub = self.create_publisher(Twist, '/sim/cmd_vel', 10)
        self.create_subscription(Twist, '/cmd_vel', self._on_cmd, 10)
        self.create_timer(0.1, self._watchdog, clock=self._wall)

    def _on_cmd(self, msg: Twist):
        self._last = self._wall.now()
        self._moving = any((msg.linear.x, msg.angular.z))
        self._pub.publish(msg)

    def _watchdog(self):
        if self._moving and (self._wall.now() - self._last).nanoseconds * 1e-9 > self._timeout:
            self._pub.publish(Twist())
            self._moving = False


def main():
    rclpy.init()
    node = CmdVelRelay()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except Exception:
        if rclpy.ok():  # erro real; durante o desligamento é esperado
            raise
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
