"""Testes do driver ROS 2 da base (hardware/hardware/hardware_node.py).

As portas seriais são trocadas por FakeLink e os publicadores por listas;
os callbacks e timers são chamados direto, sem spin.
"""

import math

import pytest

pytest.importorskip('rclpy')

from diagnostic_msgs.msg import DiagnosticStatus  # noqa: E402, I100
from geometry_msgs.msg import Twist  # noqa: E402

from hardware import hardware_node as hw  # noqa: E402
from hardware.protocol import STOP_COMMAND  # noqa: E402


class FakeLink:

    def __init__(self, name, port, baudrate, on_line, logger, **_):
        self.name, self.port, self.on_line = name, port, on_line
        self.written = []
        self.age = 0.0
        self.is_open = True
        self.started = self.stopped = False

    def start(self):
        self.started = True

    def stop(self):
        self.stopped = True

    def seconds_since_rx(self):
        return self.age

    def write_line(self, text):
        self.written.append(text)
        return True


class FakePub:

    def __init__(self):
        self.msgs = []

    def publish(self, msg):
        self.msgs.append(msg)

    sendTransform = publish


@pytest.fixture
def node(ros, monkeypatch):
    monkeypatch.setattr(hw, 'SerialLink', FakeLink)
    n = hw.HardwareNode()
    n._odom_pub, n._joint_pub, n._imu_pub, n._diag_pub, n._tf = (FakePub() for _ in range(5))
    yield n
    n.destroy_node()


def _motor_alive(node):
    node._on_motor_line('M 1.0 2.0 10.0 20.0 R')


def test_inicia_os_dois_enlaces(node):
    assert node._motor.started and node._imu.started
    assert node._motor.port == '/dev/ttyMOTOR' and node._imu.port == '/dev/ttyIMU'


def test_quaternion():
    q = hw._quaternion(0.0, 0.0, math.pi / 2)
    assert q.z == pytest.approx(math.sin(math.pi / 4))
    assert q.w == pytest.approx(math.cos(math.pi / 4))


# ── Serial dos motores / IMU ─────────────────────────────────────────────────

def test_linha_de_motor(node):
    node._on_motor_line('# firmware v1')
    node._on_motor_line('lixo')
    assert node._motor_frames == 0
    _motor_alive(node)
    assert node._motor_frames == 1
    # convenção padrão: feedback invertido, sem troca
    assert node._wheel_vel == (-1.0, -2.0)
    assert node._wheel_pos == (-10.0, -20.0)


def test_linha_de_imu_com_orientacao(node):
    node._on_imu_line('# imu ok')
    node._on_imu_line('I 1 2')
    assert node._imu_pub.msgs == []
    node._on_imu_line('I 0.1 0.2 0.3 0.01 0.02 0.03 0.0 0.0 9.81')
    msg = node._imu_pub.msgs[-1]
    assert node._imu_frames == 1 and node._gyro_z == pytest.approx(0.03)
    assert msg.header.frame_id == 'imu_link'
    assert msg.orientation_covariance[0] == pytest.approx(0.01)
    assert msg.linear_acceleration.z == pytest.approx(9.81)


def test_linha_de_imu_sem_orientacao(node):
    node._imu_has_orientation = False
    node._on_imu_line('I 0.1 0.2 0.3 0.01 0.02 0.03 0.0 0.0 9.81')
    assert node._imu_pub.msgs[-1].orientation_covariance[0] == -1.0


# ── cmd_vel e watchdog ───────────────────────────────────────────────────────

def _twist(v, w):
    t = Twist()
    t.linear.x, t.angular.z = v, w
    return t


def test_cmd_vel_nao_finito_para(node):
    _motor_alive(node)
    node._on_cmd_vel(_twist(float('nan'), 0.0))
    assert node._motor.written == [STOP_COMMAND] and node._stopped


def test_cmd_vel_sem_motor_para(node):
    node._on_cmd_vel(_twist(0.5, 0.0))
    assert node._motor.written == [STOP_COMMAND]


def test_cmd_vel_com_motor_envia(node):
    _motor_alive(node)
    node._on_cmd_vel(_twist(5.0, 0.0))             # satura em max_linear_speed
    cmd = node._motor.written[-1]
    assert cmd.startswith('C ') and cmd != STOP_COMMAND
    assert not node._stopped
    node._on_cmd_vel(_twist(0.0, 0.0))
    assert node._stopped


def test_watchdog_para_sem_cmd_vel(node):
    _motor_alive(node)
    node._on_cmd_vel(_twist(0.5, 0.0))
    node._watchdog_tick()
    assert node._motor.written[-1] != STOP_COMMAND  # comando recente
    node._last_cmd_t -= 10.0
    node._watchdog_tick()
    assert node._motor.written[-1] == STOP_COMMAND and node._stopped


# ── Odometria ────────────────────────────────────────────────────────────────

def test_odom_sem_motor_fica_parado(node):
    node._odom_tick()
    odom = node._odom_pub.msgs[-1]
    assert odom.twist.twist.linear.x == 0.0
    assert odom.header.frame_id == 'odom' and odom.child_frame_id == 'base_link'
    assert len(node._tf.msgs) == 1
    assert node._joint_pub.msgs[-1].name == ['left_wheel_joint', 'right_wheel_joint']


def test_odom_integra_e_usa_giroscopio(node):
    node._on_motor_line('M -2.0 -2.0 0 0 R')       # para frente (feedback invertido)
    node._on_imu_line('I 0 0 0 0 0 0.5 0 0 9.81')
    node._last_odom_t -= 0.1
    node._odom_tick()
    odom = node._odom_pub.msgs[-1]
    assert odom.twist.twist.linear.x == pytest.approx(2.0 * 0.0825)
    assert odom.twist.twist.angular.z == pytest.approx(0.5)
    assert node._x > 0


def test_odom_sem_imu_usa_rodas_e_sem_tf(node):
    node._publish_tf = False
    node._imu.age = 99.0
    node._on_motor_line('M -1.0 1.0 0 0 R')
    node._last_odom_t -= 0.1
    node._odom_tick()
    assert node._odom_pub.msgs[-1].twist.twist.angular.z != 0.0
    assert node._tf.msgs == []


# ── Diagnóstico ──────────────────────────────────────────────────────────────

def test_diagnostico(node):
    _motor_alive(node)
    node._imu.is_open = False
    node._diagnostics_tick()
    motors, imu = node._diag_pub.msgs[-1].status
    assert motors.level == DiagnosticStatus.OK and motors.message == 'ok'
    assert imu.level == DiagnosticStatus.ERROR and imu.message == 'porta fechada'


def test_link_status_variantes(node):
    st = node._link_status('imu', None, False, False)
    assert st.message == 'desabilitado' and st.values[0].value == 'inf'
    st = node._link_status('imu', node._imu, False, False)
    assert st.level == DiagnosticStatus.WARN and st.message == 'sem dados'


# ── Encerramento ─────────────────────────────────────────────────────────────

def test_shutdown_para_e_fecha(node):
    node.shutdown()
    assert node._motor.written[-1] == STOP_COMMAND
    assert node._motor.stopped and node._imu.stopped


def test_shutdown_sem_imu(node):
    node._imu = None
    node.shutdown()
    assert node._motor.stopped


def test_main(ros, monkeypatch):
    monkeypatch.setattr(hw, 'SerialLink', FakeLink)
    monkeypatch.setattr(hw.rclpy, 'init', lambda args=None: None)
    monkeypatch.setattr(hw.rclpy, 'try_shutdown', lambda: None)
    created = []

    def spin(node):
        created.append(node)
        raise KeyboardInterrupt

    monkeypatch.setattr(hw.rclpy, 'spin', spin)
    hw.main()
    assert created and created[0]._motor.stopped
