#!/usr/bin/env bash
# Simulação completa: Gazebo (supermercado ou loja) + interface web.
#
#   ./scripts/start_sim.sh                # com janela do Gazebo
#   ./scripts/start_sim.sh --headless     # sem janela
#   ./scripts/start_sim.sh --people       # com pessoas caminhando (sim/config/mundos/<mundo>.yaml)
#   ./scripts/start_sim.sh --world loja   # mundo: supermercado (padrão) ou loja
#   ./scripts/start_sim.sh --no-rviz      # sem o RViz2 (abre por padrão)
#   ./scripts/start_sim.sh --no-build     # pula o colcon build
#
# Antes de subir, encerra qualquer ROS/Gazebo/RViz/backend que tenha ficado
# aberto de uma execução anterior (kill_ros_leftovers, em common.sh).
#
# Depois, abra http://localhost:5000, crie um mapa e conduza o robô.
set -m
source "$(dirname "${BASH_SOURCE[0]}")/common.sh"

GUI=true PEOPLE=false BUILD=1 RVIZ=1 WORLD=supermercado
while [ $# -gt 0 ]; do
    arg="$1"; shift
    case "$arg" in
        --headless) GUI=false ;;
        --people)   PEOPLE=true ;;
        --no-build) BUILD=0 ;;
        --no-rviz)  RVIZ=0 ;;
        --world)    WORLD="$1"; shift ;;
        --world=*)  WORLD="${arg#--world=}" ;;
        *) echo "argumento desconhecido: $arg"; exit 1 ;;
    esac
done

kill_ros_leftovers
[ "$BUILD" = 1 ] && build_workspace
prepare_web
source "$WS_DIR/install/setup.bash"

cleanup() {
    trap - INT TERM
    stop_backend
    [ -n "$RVIZ_PID" ] && kill -INT -- -"$RVIZ_PID" 2>/dev/null
    [ -n "$WORLD_PID" ] && kill -INT -- -"$WORLD_PID" 2>/dev/null && wait "$WORLD_PID" 2>/dev/null
    exit 0
}
trap cleanup INT TERM

# O mundo fica aberto a sessão toda; a interface sobe e derruba o software do robô
ros2 launch sim world.launch.py world:=$WORLD gui:=$GUI people:=$PEOPLE &
WORLD_PID=$!
sleep 5

if [ "$RVIZ" = 1 ]; then
    ros2 launch navigation rviz.launch.py use_sim_time:=true &
    RVIZ_PID=$!
fi

export ROBOT_SIM=1
start_backend
wait
