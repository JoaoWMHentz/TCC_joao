#!/usr/bin/env python3
"""
fake_firmware.py — emula os dois microcontroladores em portas seriais virtuais.

Permite testar o hardware_node sem o robô. Cria dois pseudo-terminais e
imprime os caminhos; aponte o driver para eles:

  python3 tools/fake_firmware.py
  ros2 run hardware hardware_node --ros-args \
      -p motor_port:=<porta MOTOR> -p imu_port:=<porta IMU>

Motores: responde a "C <vl> <vr>" reproduzindo as convenções de sinal do
firmware de referência (esp32_motor_controller.ino), integra a posição das
rodas e envia "M ..." a 50 Hz. Zera a velocidade sem comandos por 0,5 s.
IMU: envia "I ..." a 50 Hz com o giro coerente com as rodas comandadas.
"""

import math
import os
import pty
import select
import time

RADIUS, SEPARATION = 0.0825, 0.310
WATCHDOG_S = 0.5
PERIOD_S = 0.02


def open_pty(label: str) -> int:
    master, slave = pty.openpty()
    print(f'{label}: {os.ttyname(slave)}', flush=True)
    return master


def main():
    motor = open_pty('MOTOR')
    imu = open_pty('IMU')

    # Estado "físico" (convenção do firmware)
    cmd_l = cmd_r = 0.0
    last_cmd = 0.0
    pos_motor_l = pos_motor_r = 0.0
    yaw = 0.0
    buffer = b''
    next_tick = time.monotonic()

    while True:
        timeout = max(0.0, next_tick - time.monotonic())
        ready, _, _ = select.select([motor], [], [], timeout)
        if ready:
            buffer += os.read(motor, 1024)
            while b'\n' in buffer:
                line, buffer = buffer.split(b'\n', 1)
                parts = line.decode(errors='ignore').split()
                if len(parts) == 3 and parts[0] == 'C':
                    cmd_l, cmd_r = float(parts[1]), float(parts[2])
                    last_cmd = time.monotonic()
            continue

        next_tick += PERIOD_S
        if time.monotonic() - last_cmd > WATCHDOG_S:
            cmd_l = cmd_r = 0.0

        # Mesmo mapeamento do firmware: motor direito recebe o 1º valor,
        # motor esquerdo o 2º negado; a telemetria nega o esquerdo de volta.
        shaft_r, shaft_l = cmd_l, -cmd_r
        pos_motor_r += shaft_r * PERIOD_S
        pos_motor_l += shaft_l * PERIOD_S
        tele_l, tele_r = -shaft_l, shaft_r
        os.write(motor, f'M {tele_l:.4f} {tele_r:.4f} {-pos_motor_l:.4f} {pos_motor_r:.4f} R\n'.encode())

        # Giro "real" do robô, a partir das rodas na convenção do ROS
        # (o driver nega e troca o comando antes de enviar; aqui desfazemos).
        ros_l, ros_r = -tele_l, -tele_r
        gz = (ros_r - ros_l) * RADIUS / SEPARATION
        yaw = math.atan2(math.sin(yaw + gz * PERIOD_S), math.cos(yaw + gz * PERIOD_S))
        os.write(imu, f'I {yaw:.4f} 0.0 0.0 0.0 0.0 {gz:.4f} 0.0 0.0 9.81\n'.encode())


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        pass
