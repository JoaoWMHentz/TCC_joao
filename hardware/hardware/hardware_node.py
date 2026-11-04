"""
hardware_node.py — driver ROS 2 da base diferencial.

Faz a ponte entre o ROS e os dois microcontroladores:

  /cmd_vel  ──►  cinemática inversa  ──►  serial dos motores ("C vl vr")
  serial dos motores ("M ...")  ──►  /odom, TF odom→base_link, /joint_states
  serial da IMU ("I ...")       ──►  /imu/data

Segurança:
  - sem /cmd_vel novo por `cmd_vel_timeout` s → para os motores;
  - enlace dos motores sem dados por `serial_timeout` s → recusa comandos;
  - o firmware dos motores tem watchdog próprio (para sozinho sem comandos).

Odometria: a velocidade linear vem dos encoders das rodas; a angular vem do
giroscópio da IMU quando disponível (não sofre com o escorregamento das rodas),
senão da diferença entre as rodas.
"""

from __future__ import annotations

import math
import threading
import time

import rclpy
from rclpy.node import Node

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import Quaternion, TransformStamped, Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, JointState
from tf2_ros import TransformBroadcaster

from hardware.protocol import (
    STOP_COMMAND, WheelMapping, apply_wheel_floor, clamp, euler_to_quaternion,
    format_velocity_command, integrate_pose, parse_imu_frame, parse_motor_frame,
    twist_to_wheels, wheels_to_twist)
from hardware.serial_link import SerialLink


def _quaternion(roll: float, pitch: float, yaw: float) -> Quaternion:
    x, y, z, w = euler_to_quaternion(roll, pitch, yaw)
    return Quaternion(x=x, y=y, z=z, w=w)


class HardwareNode(Node):

    def __init__(self):
        super().__init__('hardware_node')

        p = self.declare_parameter
        p('motor_port', '/dev/ttyMOTOR')
        p('motor_baudrate', 115200)
        p('imu_port', '/dev/ttyIMU')
        p('imu_baudrate', 115200)
        p('use_imu', True)
        p('imu_has_orientation', True)
        p('wheel_radius', 0.0825)
        p('wheel_separation', 0.310)
        p('max_linear_speed', 0.5)        # m/s
        p('max_angular_speed', 1.5)       # rad/s
        p('min_wheel_speed', 1.5)         # rad/s — abaixo disso o BLDC vibra
        p('wheel_deadband', 0.15)         # rad/s — abaixo disso = parado
        p('cmd_vel_timeout', 0.5)         # s
        p('serial_timeout', 1.0)          # s
        p('odom_rate', 25.0)              # Hz
        p('publish_tf', True)
        p('odom_frame', 'odom')
        p('base_frame', 'base_link')
        p('imu_frame', 'imu_link')
        # Convenção de sinais do firmware de referência (ver protocol.WheelMapping)
        p('invert_command', True)
        p('swap_command', True)
        p('invert_feedback', True)
        p('swap_feedback', False)

        g = lambda name: self.get_parameter(name).value  # noqa: E731
        self._r = float(g('wheel_radius'))
        self._sep = float(g('wheel_separation'))
        self._max_v = float(g('max_linear_speed'))
        self._max_w = float(g('max_angular_speed'))
        self._min_wheel = float(g('min_wheel_speed'))
        self._deadband = float(g('wheel_deadband'))
        self._cmd_timeout = float(g('cmd_vel_timeout'))
        self._serial_timeout = float(g('serial_timeout'))
        self._use_imu = bool(g('use_imu'))
        self._imu_has_orientation = bool(g('imu_has_orientation'))
        self._publish_tf = bool(g('publish_tf'))
        self._odom_frame = g('odom_frame')
        self._base_frame = g('base_frame')
        self._imu_frame = g('imu_frame')
        self._mapping = WheelMapping(
            invert_command=bool(g('invert_command')), swap_command=bool(g('swap_command')),
            invert_feedback=bool(g('invert_feedback')), swap_feedback=bool(g('swap_feedback')))

        # Estado compartilhado entre as threads das seriais e o executor ROS
        self._lock = threading.Lock()
        self._wheel_vel = (0.0, 0.0)
        self._wheel_pos = (0.0, 0.0)
        self._gyro_z = 0.0
        self._imu_frames = 0
        self._motor_frames = 0

        self._x = self._y = self._yaw = 0.0
        self._last_odom_t = time.monotonic()
        self._last_cmd_t = 0.0
        self._stopped = True

        # Publicadores
        self._odom_pub = self.create_publisher(Odometry, 'odom', 10)
        self._joint_pub = self.create_publisher(JointState, 'joint_states', 10)
        self._imu_pub = self.create_publisher(Imu, 'imu/data', 10)
        self._diag_pub = self.create_publisher(DiagnosticArray, 'diagnostics', 10)
        self._tf = TransformBroadcaster(self)
        self.create_subscription(Twist, 'cmd_vel', self._on_cmd_vel, 10)

        # Enlaces seriais
        log = self.get_logger()
        self._motor = SerialLink('MOTOR', g('motor_port'), int(g('motor_baudrate')),
                                 self._on_motor_line, log)
        self._motor.start()
        self._imu = None
        if self._use_imu:
            self._imu = SerialLink('IMU', g('imu_port'), int(g('imu_baudrate')),
                                   self._on_imu_line, log)
            self._imu.start()

        self.create_timer(1.0 / float(g('odom_rate')), self._odom_tick)
        self.create_timer(0.1, self._watchdog_tick)
        self.create_timer(1.0, self._diagnostics_tick)

        log.info(f'hardware_node: r={self._r} m, L={self._sep} m, '
                 f'v_max={self._max_v} m/s, w_max={self._max_w} rad/s, '
                 f'IMU {"ativa" if self._use_imu else "desativada"}')

    # ── Saúde dos enlaces ────────────────────────────────────────────────────

    def _motor_ok(self) -> bool:
        return self._motor.seconds_since_rx() < self._serial_timeout and self._motor_frames > 0

    def _imu_ok(self) -> bool:
        return (self._imu is not None
                and self._imu.seconds_since_rx() < self._serial_timeout
                and self._imu_frames > 0)

    # ── Seriais (threads próprias) ───────────────────────────────────────────

    def _on_motor_line(self, line: str):
        if line.startswith('#'):
            self.get_logger().info(f'[MOTOR/fw] {line[1:].strip()}')
            return
        frame = parse_motor_frame(line)
        if frame is None:
            return
        vel = self._mapping.from_firmware(frame.vel_left, frame.vel_right)
        pos = self._mapping.from_firmware(frame.pos_left, frame.pos_right)
        with self._lock:
            self._wheel_vel, self._wheel_pos = vel, pos
            self._motor_frames += 1

    def _on_imu_line(self, line: str):
        if line.startswith('#'):
            self.get_logger().info(f'[IMU/fw] {line[1:].strip()}')
            return
        f = parse_imu_frame(line)
        if f is None:
            return
        with self._lock:
            self._gyro_z = f.gz
            self._imu_frames += 1

        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self._imu_frame
        if self._imu_has_orientation:
            msg.orientation = _quaternion(f.roll, f.pitch, f.yaw)
            msg.orientation_covariance = [0.01, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.01]
        else:
            msg.orientation_covariance[0] = -1.0  # REP-145: orientação ausente
        msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z = f.gx, f.gy, f.gz
        msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z = f.ax, f.ay, f.az
        msg.angular_velocity_covariance = [0.001, 0.0, 0.0, 0.0, 0.001, 0.0, 0.0, 0.0, 0.001]
        msg.linear_acceleration_covariance = [0.04, 0.0, 0.0, 0.0, 0.04, 0.0, 0.0, 0.0, 0.04]
        self._imu_pub.publish(msg)

    # ── Comando de velocidade ────────────────────────────────────────────────

    def _send_stop(self):
        self._motor.write_line(STOP_COMMAND)
        self._stopped = True

    def _on_cmd_vel(self, msg: Twist):
        v, w = float(msg.linear.x), float(msg.angular.z)
        if not (math.isfinite(v) and math.isfinite(w)):
            self.get_logger().warn('cmd_vel não finito ignorado', throttle_duration_sec=2.0)
            self._send_stop()
            return
        if not self._motor_ok():
            self.get_logger().warn('cmd_vel ignorado: controlador dos motores sem resposta',
                                   throttle_duration_sec=2.0)
            self._send_stop()
            return

        v, w = clamp(v, self._max_v), clamp(w, self._max_w)
        left, right = twist_to_wheels(v, w, self._r, self._sep)
        left, right = apply_wheel_floor(left, right, self._deadband, self._min_wheel)
        left, right = self._mapping.to_firmware(left, right)
        self._motor.write_line(format_velocity_command(left, right))
        self._last_cmd_t = time.monotonic()
        self._stopped = (left == 0.0 and right == 0.0)

    def _watchdog_tick(self):
        if not self._stopped and time.monotonic() - self._last_cmd_t > self._cmd_timeout:
            self.get_logger().info('cmd_vel expirou — parando motores')
            self._send_stop()

    # ── Odometria ────────────────────────────────────────────────────────────

    def _odom_tick(self):
        now_mono = time.monotonic()
        dt = now_mono - self._last_odom_t
        self._last_odom_t = now_mono

        with self._lock:
            vel_l, vel_r = self._wheel_vel
            pos_l, pos_r = self._wheel_pos
            gyro_z = self._gyro_z

        if self._motor_ok() and 0.0 < dt < 1.0:
            v, w = wheels_to_twist(vel_l, vel_r, self._r, self._sep)
            if self._imu_ok():
                w = gyro_z
            self._x, self._y, self._yaw = integrate_pose(self._x, self._y, self._yaw, v, w, dt)
        else:
            v = w = 0.0

        stamp = self.get_clock().now().to_msg()
        q = _quaternion(0.0, 0.0, self._yaw)

        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = self._odom_frame
        odom.child_frame_id = self._base_frame
        odom.pose.pose.position.x = self._x
        odom.pose.pose.position.y = self._y
        odom.pose.pose.orientation = q
        odom.twist.twist.linear.x = v
        odom.twist.twist.angular.z = w
        # Pose: incerteza alta (deriva acumulada); twist: medida instantânea
        odom.pose.covariance[0] = odom.pose.covariance[7] = odom.pose.covariance[35] = 0.5
        odom.twist.covariance[0] = 0.005
        odom.twist.covariance[35] = 0.01
        self._odom_pub.publish(odom)

        if self._publish_tf:
            tf = TransformStamped()
            tf.header.stamp = stamp
            tf.header.frame_id = self._odom_frame
            tf.child_frame_id = self._base_frame
            tf.transform.translation.x = self._x
            tf.transform.translation.y = self._y
            tf.transform.rotation = q
            self._tf.sendTransform(tf)

        js = JointState()
        js.header.stamp = stamp
        js.name = ['left_wheel_joint', 'right_wheel_joint']
        js.position = [pos_l, pos_r]
        js.velocity = [vel_l, vel_r]
        self._joint_pub.publish(js)

    # ── Diagnóstico ──────────────────────────────────────────────────────────

    def _link_status(self, name: str, link: SerialLink | None, ok: bool, required: bool):
        st = DiagnosticStatus(name=f'hardware/{name}', hardware_id=name)
        if link is None:
            st.level, st.message = DiagnosticStatus.OK, 'desabilitado'
        elif ok:
            st.level, st.message = DiagnosticStatus.OK, 'ok'
        else:
            st.level = DiagnosticStatus.ERROR if required else DiagnosticStatus.WARN
            st.message = 'porta fechada' if not link.is_open else 'sem dados'
        age = link.seconds_since_rx() if link is not None else float('inf')
        st.values = [KeyValue(key='seconds_since_rx', value=f'{age:.2f}')]
        return st

    def _diagnostics_tick(self):
        arr = DiagnosticArray()
        arr.header.stamp = self.get_clock().now().to_msg()
        arr.status = [
            self._link_status('motors', self._motor, self._motor_ok(), True),
            self._link_status('imu', self._imu, self._imu_ok(), self._use_imu),
        ]
        self._diag_pub.publish(arr)

    # ── Encerramento ─────────────────────────────────────────────────────────

    def shutdown(self):
        try:
            self._send_stop()
        finally:
            self._motor.stop()
            if self._imu is not None:
                self._imu.stop()


def main(args=None):
    rclpy.init(args=args)
    node = HardwareNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
