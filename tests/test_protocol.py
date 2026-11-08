"""Testes do protocolo serial e da cinemática (hardware/hardware/protocol.py)."""

import math
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'hardware'))

from hardware.protocol import (  # noqa: E402
    apply_wheel_floor, clamp, euler_to_quaternion, format_velocity_command,
    integrate_pose, parse_imu_frame, parse_motor_frame, STOP_COMMAND,
    twist_to_wheels, WheelMapping, wheels_to_twist)
import pytest  # noqa: E402

R, L = 0.0825, 0.310


# ── Parsing ──────────────────────────────────────────────────────────────────

def test_motor_frame_valid():
    f = parse_motor_frame('M 1.5000 -2.2500 10.0 -3.5 R')
    assert ((f.vel_left, f.vel_right, f.pos_left, f.pos_right, f.flag)
            == (1.5, -2.25, 10.0, -3.5, 'R'))


@pytest.mark.parametrize('line', [
    '', 'M', 'M 1 2 3 4',            # campos faltando
    'X 1 2 3 4 R',                   # prefixo errado
    'M 1 abc 3 4 R',                 # número inválido
    'M nan 2 3 4 R', 'M inf 2 3 4 R',  # não finitos
    '# calibrando motores...',
])
def test_motor_frame_invalid(line):
    assert parse_motor_frame(line) is None


def test_imu_frame_valid_and_extra_fields_ignored():
    f = parse_imu_frame('I 0.1 0.2 0.3 0.01 0.02 0.03 0.0 0.0 9.81 1.2 inf')
    assert f.yaw == 0.1 and f.gz == 0.03 and f.az == 9.81


@pytest.mark.parametrize('line', ['I 1 2 3', 'M 1 2 3 4 5 6 7 8 9', 'I 1 2 3 4 5 6 7 8 nan'])
def test_imu_frame_invalid(line):
    assert parse_imu_frame(line) is None


def test_velocity_command_format():
    assert format_velocity_command(1.23456, -0.5) == 'C 1.2346 -0.5000'
    assert STOP_COMMAND == 'C 0.0000 0.0000'


# ── Convenção de sinais ──────────────────────────────────────────────────────

def test_wheel_mapping_identity():
    assert WheelMapping().to_firmware(1.0, 2.0) == (1.0, 2.0)


def test_wheel_mapping_swap_and_invert():
    m = WheelMapping(invert_command=True, swap_command=True)
    assert m.to_firmware(1.0, 2.0) == (-2.0, -1.0)
    m = WheelMapping(invert_feedback=True)
    assert m.from_firmware(1.0, 2.0) == (-1.0, -2.0)


# ── Cinemática ───────────────────────────────────────────────────────────────

@pytest.mark.parametrize('v,w', [(0.2, 0.0), (0.0, 1.0), (0.15, -0.6), (-0.1, 0.3)])
def test_kinematics_roundtrip(v, w):
    left, right = twist_to_wheels(v, w, R, L)
    v2, w2 = wheels_to_twist(left, right, R, L)
    assert v2 == pytest.approx(v) and w2 == pytest.approx(w)


def test_straight_line_equal_wheels():
    left, right = twist_to_wheels(0.2, 0.0, R, L)
    assert left == pytest.approx(right) == pytest.approx(0.2 / R)


def test_turning_left_right_wheel_faster():
    left, right = twist_to_wheels(0.0, 1.0, R, L)
    assert right > 0 > left


def test_wheel_floor():
    assert apply_wheel_floor(0.1, -0.1, 0.15, 1.5) == (0.0, 0.0)      # zona morta
    assert apply_wheel_floor(0.6, -0.6, 0.15, 1.5) == (1.5, -1.5)     # sobe ao piso
    assert apply_wheel_floor(2.4, 3.0, 0.15, 1.5) == (2.4, 3.0)       # acima: intocado
    assert apply_wheel_floor(0.6, 0.6, 0.15, 0.0) == (0.6, 0.6)       # piso desligado


def test_clamp():
    assert clamp(3.0, 0.5) == 0.5 and clamp(-3.0, 0.5) == -0.5 and clamp(0.2, 0.5) == 0.2


# ── Odometria ────────────────────────────────────────────────────────────────

def test_integrate_straight():
    x, y, yaw = integrate_pose(0.0, 0.0, 0.0, v=0.2, w=0.0, dt=1.0)
    assert (x, y, yaw) == pytest.approx((0.2, 0.0, 0.0))


def test_integrate_full_circle_returns_to_start():
    x = y = yaw = 0.0
    v, w = 0.2, 0.5
    steps = 2000
    dt = (2 * math.pi / w) / steps
    for _ in range(steps):
        x, y, yaw = integrate_pose(x, y, yaw, v, w, dt)
    assert math.hypot(x, y) < 1e-3
    assert abs(math.atan2(math.sin(yaw), math.cos(yaw))) < 1e-6


def test_yaw_stays_normalized():
    _, _, yaw = integrate_pose(0.0, 0.0, 3.0, v=0.0, w=1.0, dt=1.0)
    assert -math.pi <= yaw <= math.pi


def test_quaternion_yaw():
    x, y, z, w = euler_to_quaternion(0.0, 0.0, math.pi / 2)
    assert (x, y) == pytest.approx((0.0, 0.0))
    assert z == pytest.approx(math.sqrt(0.5)) and w == pytest.approx(math.sqrt(0.5))
