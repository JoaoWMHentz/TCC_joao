#!/usr/bin/env bash
# Robô físico: interface web, que sobe sensores, RTAB-Map e Nav2 sob demanda.
#
#   ./scripts/start_robot.sh
#
# Pré-requisitos: regras udev instaladas (bringup/udev/99-robot.rules) e
# firmwares gravados (hardware/firmware). Abra http://<ip-do-robô>:5000.
set -m
source "$(dirname "${BASH_SOURCE[0]}")/common.sh"

prepare_web

trap 'trap - INT TERM; stop_backend; exit 0' INT TERM
export ROBOT_SIM=0
start_backend
wait
