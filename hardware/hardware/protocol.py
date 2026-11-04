"""
protocol.py — protocolo serial e cinemática da base diferencial.

Módulo puro (sem ROS e sem pyserial) para poder ser testado isoladamente.

Protocolo com o controlador dos motores (ESP32 + SimpleFOC), texto ASCII,
uma mensagem por linha, 115200 bps:

    host -> MCU :  C <vel_esq> <vel_dir>                 (rad/s por roda)
    MCU  -> host:  M <vel_esq> <vel_dir> <pos_esq> <pos_dir> <flag>
                   vel em rad/s, pos em rad (acumulada), flag 'R' = controle
                   pelo host
    MCU  -> host:  # <texto>                             (log informativo)

Protocolo com o microcontrolador da IMU (STM32F401 + BNO085), 50 Hz:

    MCU  -> host:  I <yaw> <pitch> <roll> <gx> <gy> <gz> <ax> <ay> <az> [...]
                   ângulos em rad, giroscópio em rad/s, aceleração em m/s².
                   Campos extras após os nove primeiros são ignorados.
"""

from __future__ import annotations

import math
from dataclasses import dataclass


# ── Mensagens ────────────────────────────────────────────────────────────────

@dataclass(frozen=True)
class MotorFrame:
    vel_left: float    # rad/s
    vel_right: float   # rad/s
    pos_left: float    # rad
    pos_right: float   # rad
    flag: str


@dataclass(frozen=True)
class ImuFrame:
    yaw: float
    pitch: float
    roll: float
    gx: float
    gy: float
    gz: float
    ax: float
    ay: float
    az: float


def _floats(tokens: list[str]) -> list[float] | None:
    try:
        values = [float(t) for t in tokens]
    except ValueError:
        return None
    if not all(math.isfinite(v) for v in values):
        return None
    return values


def parse_motor_frame(line: str) -> MotorFrame | None:
    """'M vl vr pl pr flag' -> MotorFrame; None se a linha não for um frame válido."""
    parts = line.strip().split()
    if len(parts) < 6 or parts[0] != 'M':
        return None
    values = _floats(parts[1:5])
    if values is None:
        return None
    return MotorFrame(*values, flag=parts[5])


def parse_imu_frame(line: str) -> ImuFrame | None:
    """'I yaw pitch roll gx gy gz ax ay az ...' -> ImuFrame; None se inválido."""
    parts = line.strip().split()
    if len(parts) < 10 or parts[0] != 'I':
        return None
    values = _floats(parts[1:10])
    if values is None:
        return None
    return ImuFrame(*values)


def format_velocity_command(vel_left: float, vel_right: float) -> str:
    return f'C {vel_left:.4f} {vel_right:.4f}'


STOP_COMMAND = format_velocity_command(0.0, 0.0)


# ── Convenção de sinais entre o firmware e o robô ────────────────────────────

@dataclass
class WheelMapping:
    """Ajusta sentido e lado das rodas entre o firmware e a convenção do ROS.

    A montagem física (motor espelhado, fios trocados) pode exigir inverter o
    sinal e/ou trocar esquerda/direita. Comando e realimentação têm ajustes
    independentes porque o firmware pode tratá-los de forma diferente.
    """
    invert_command: bool = False
    swap_command: bool = False
    invert_feedback: bool = False
    swap_feedback: bool = False

    @staticmethod
    def _apply(left: float, right: float, invert: bool, swap: bool):
        if swap:
            left, right = right, left
        if invert:
            left, right = -left, -right
        return left, right

    def to_firmware(self, left: float, right: float) -> tuple[float, float]:
        return self._apply(left, right, self.invert_command, self.swap_command)

    def from_firmware(self, left: float, right: float) -> tuple[float, float]:
        return self._apply(left, right, self.invert_feedback, self.swap_feedback)


# ── Cinemática diferencial ───────────────────────────────────────────────────

def twist_to_wheels(v: float, w: float, radius: float, separation: float) -> tuple[float, float]:
    """(v [m/s], w [rad/s]) do robô -> velocidade angular de cada roda [rad/s]."""
    left = (v - w * separation / 2.0) / radius
    right = (v + w * separation / 2.0) / radius
    return left, right


def wheels_to_twist(vel_left: float, vel_right: float,
                    radius: float, separation: float) -> tuple[float, float]:
    """Velocidade das rodas [rad/s] -> (v [m/s], w [rad/s]) do robô."""
    v = (vel_left + vel_right) * radius / 2.0
    w = (vel_right - vel_left) * radius / separation
    return v, w


def apply_wheel_floor(left: float, right: float,
                      deadband: float, minimum: float) -> tuple[float, float]:
    """Velocidade mínima por roda.

    Motores BLDC com sensor Hall vibram em vez de girar abaixo de uma certa
    velocidade. |w| < deadband vira 0; entre deadband e minimum sobe para
    minimum, preservando o sentido. minimum = 0 desliga o piso.
    """
    def floor(w: float) -> float:
        if abs(w) < deadband:
            return 0.0
        return math.copysign(max(abs(w), minimum), w)
    return floor(left), floor(right)


def clamp(value: float, limit: float) -> float:
    return max(-limit, min(limit, value))


def integrate_pose(x: float, y: float, yaw: float,
                   v: float, w: float, dt: float) -> tuple[float, float, float]:
    """Integra a pose 2D com a aproximação de ponto médio (Runge-Kutta 2)."""
    mid_yaw = yaw + w * dt / 2.0
    x += v * math.cos(mid_yaw) * dt
    y += v * math.sin(mid_yaw) * dt
    yaw = math.atan2(math.sin(yaw + w * dt), math.cos(yaw + w * dt))
    return x, y, yaw


def euler_to_quaternion(roll: float, pitch: float, yaw: float) -> tuple[float, float, float, float]:
    """Retorna (x, y, z, w)."""
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    return (
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    )
