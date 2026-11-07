# Funções compartilhadas pelos scripts de inicialização (carregado com "source").
REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
WS_DIR="${WS_DIR:-$(cd "$REPO_DIR/../.." && pwd)}"
WEB_DIR="$REPO_DIR/web"
VENV="$WEB_DIR/backend/.venv"

[ -f "$REPO_DIR/.env" ] && set -a && . "$REPO_DIR/.env" && set +a

# Terminal do VS Code instalado via snap: ele exporta caminhos de bibliotecas
# do snap que quebram o RViz e o Gazebo ("symbol lookup error ... core20").
# Remove toda variável dessas que aponte para /snap/.
for var in GTK_PATH GTK_EXE_PREFIX GTK_IM_MODULE_FILE GDK_PIXBUF_MODULE_FILE \
           GDK_PIXBUF_MODULEDIR GIO_MODULE_DIR GSETTINGS_SCHEMA_DIR LOCPATH \
           LD_LIBRARY_PATH LD_PRELOAD; do
    case "${!var}" in *"/snap/"*) unset "$var" ;; esac
done
[ -n "$XDG_DATA_DIRS_VSCODE_SNAP_ORIG" ] && export XDG_DATA_DIRS="$XDG_DATA_DIRS_VSCODE_SNAP_ORIG"
[ -n "$XDG_CONFIG_DIRS_VSCODE_SNAP_ORIG" ] && export XDG_CONFIG_DIRS="$XDG_CONFIG_DIRS_VSCODE_SNAP_ORIG"
case "$XDG_DATA_DIRS" in *"/snap/code"*) export XDG_DATA_DIRS=/usr/local/share:/usr/share ;; esac

source /opt/ros/jazzy/setup.bash
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST

# Encerra o que sobrou de execuções anteriores (Gazebo, launches, nós, RViz,
# backend). Duas simulações abertas ao mesmo tempo dividem os mesmos tópicos
# do Gazebo e do ROS e se misturam: robôs/pessoas criados no mundo errado,
# leituras de sensor de outra simulação. Não mexe no PostgreSQL (docker).
LEFTOVER_PATTERNS=(
    "ros2 launch" "ros2 run" "gz sim" "rviz2"
    "/opt/ros/jazzy/lib/" "$WS_DIR/install/"
    "$VENV/bin/python app.py"
)

# PIDs que casam com LEFTOVER_PATTERNS, exceto este script e quem o chamou
# (o terminal que rodou o start_sim.sh não pode ser morto).
_leftover_pids() {
    local protect=" " pid=$$ p
    while [ -n "$pid" ] && [ "$pid" -gt 1 ]; do
        protect+="$pid "
        pid=$(ps -o ppid= -p "$pid" 2>/dev/null | tr -d ' ')
    done
    protect+="$BASHPID "
    for p in "${LEFTOVER_PATTERNS[@]}"; do pgrep -f -- "$p"; done | sort -un |
        while read -r pid; do
            case "$protect" in *" $pid "*) ;; *) echo "$pid" ;; esac
        done
}

kill_ros_leftovers() {
    local pids
    pids=$(_leftover_pids)
    if [ -n "$pids" ]; then
        echo "[start] encerrando processos ROS/Gazebo de execuções anteriores:"
        ps -o pid=,args= -p $pids 2>/dev/null | cut -c1-150 | sed 's/^/    /'
        # Ctrl+C primeiro (deixa salvar/fechar direito), depois força
        kill -INT $pids 2>/dev/null
        for _ in $(seq 10); do
            pids=$(_leftover_pids)
            [ -z "$pids" ] && break
            sleep 0.5
        done
        [ -n "$pids" ] && kill -KILL $pids 2>/dev/null
    fi
    # O daemon do ros2 guarda a lista de nós antigos; reinicia limpo
    ros2 daemon stop >/dev/null 2>&1
}

build_workspace() {
    echo "[start] compilando o workspace ($WS_DIR)..."
    (cd "$WS_DIR" && colcon build --symlink-install \
        --packages-select description hardware bringup navigation sim) || exit 1
}

# docker compose (v2) ou docker-compose (v1), o que estiver instalado
compose() {
    local cmd
    if docker compose version >/dev/null 2>&1; then cmd="docker compose"; else cmd="docker-compose"; fi
    # Usuário recém-adicionado ao grupo docker (install.sh) ainda sem novo login:
    # a sessão atual não tem o grupo, então roda o comando via "sg docker".
    if ! docker info >/dev/null 2>&1 && getent group docker | grep -qw "$USER"; then
        sg docker -c "$cmd $(printf '%q ' "$@")"
    else
        $cmd "$@"
    fi
}

prepare_web() {
    if [ ! -d "$VENV" ]; then
        echo "[start] criando o ambiente Python do backend..."
        python3 -m venv --system-site-packages "$VENV"
        "$VENV/bin/pip" install -q -r "$WEB_DIR/backend/requirements.txt" || exit 1
    fi
    if [ ! -f "$WEB_DIR/frontend/dist/index.html" ]; then
        echo "[start] compilando a interface web..."
        (cd "$WEB_DIR/frontend" && npm ci && npm run build) || exit 1
    fi
    echo "[start] subindo o PostgreSQL (docker compose)..."
    (cd "$WEB_DIR" && compose up -d) || exit 1
}

start_backend() {
    source "$WS_DIR/install/setup.bash"
    (cd "$WEB_DIR/backend" && exec "$VENV/bin/python" app.py) &
    BACKEND_PID=$!
    echo "[start] interface: http://localhost:${WEB_PORT:-5000}"
}

# Ctrl+C: o backend encerra a sessão (o RTAB-Map salva o mapa) antes de sair
stop_backend() {
    if [ -n "$BACKEND_PID" ] && kill -0 "$BACKEND_PID" 2>/dev/null; then
        echo "[start] encerrando o backend (aguardando salvar o mapa)..."
        kill -INT "$BACKEND_PID"
        wait "$BACKEND_PID" 2>/dev/null
    fi
}
