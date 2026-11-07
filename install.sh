#!/usr/bin/env bash
# Instala tudo o que o projeto precisa em um Ubuntu 24.04 (Noble):
#   ROS 2 Jazzy + RTAB-Map + Nav2 + Gazebo Harmonic, Docker (PostgreSQL),
#   Node.js 22 (frontend), ambiente Python do backend e compila o workspace.
#
# Uso:
#   ./install.sh            # simulação / desenvolvimento
#   ./install.sh --robot    # também drivers do RPLidar e da Astra + regras udev
#
# Pode ser executado de novo sem problemas: etapas já feitas são puladas.
set -euo pipefail

ROS_DISTRO=jazzy
NODE_MAJOR=22

REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_DIR="${WS_DIR:-$(cd "$REPO_DIR/../.." && pwd)}"
WEB_DIR="$REPO_DIR/web"
VENV="$WEB_DIR/backend/.venv"

ROBOT=false
for arg in "$@"; do
    case "$arg" in
        --robot) ROBOT=true ;;
        -h|--help) sed -n '2,10p' "$0" | sed 's/^# \{0,1\}//'; exit 0 ;;
        *) echo "Opção desconhecida: $arg (use --help)"; exit 1 ;;
    esac
done

log() { printf '\n\033[1;32m[install]\033[0m %s\n' "$*"; }
warn() { printf '\033[1;33m[install]\033[0m %s\n' "$*"; }

# --------------------------------------------------------------------------
# Verificações
# --------------------------------------------------------------------------
if [ "$(id -u)" -eq 0 ]; then
    echo "Execute como usuário normal (o script usa sudo quando precisa)."; exit 1
fi
. /etc/os-release
if [ "${VERSION_CODENAME:-}" != "noble" ]; then
    warn "Testado no Ubuntu 24.04 (noble); este sistema é '${VERSION_CODENAME:-?}'. Continuando..."
fi
if [ "$(basename "$(dirname "$REPO_DIR")")" != "src" ]; then
    warn "O repositório deveria estar em <workspace>/src/. Workspace assumido: $WS_DIR"
fi

sudo -v

# --------------------------------------------------------------------------
# 1. Repositório do ROS 2
# --------------------------------------------------------------------------
if [ ! -f /etc/apt/sources.list.d/ros2.list ] && ! dpkg -s ros2-apt-source >/dev/null 2>&1; then
    log "Adicionando o repositório apt do ROS 2..."
    sudo apt-get update
    sudo apt-get install -y software-properties-common curl ca-certificates
    sudo add-apt-repository -y universe
    ROS_APT_SOURCE_VERSION=$(curl -fsSL https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest \
        | grep -F '"tag_name"' | awk -F\" '{print $4}')
    curl -fsSL -o /tmp/ros2-apt-source.deb \
        "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.${VERSION_CODENAME}_all.deb"
    sudo dpkg -i /tmp/ros2-apt-source.deb
    rm -f /tmp/ros2-apt-source.deb
fi

# --------------------------------------------------------------------------
# 2. Repositório do Node.js (o do Ubuntu é o 18, o Vite 7 exige >= 20.19)
# --------------------------------------------------------------------------
node_ok() {
    command -v node >/dev/null 2>&1 || return 1
    local v; v=$(node -v | tr -d v); local major=${v%%.*}
    [ "$major" -ge "$NODE_MAJOR" ]
}
if ! node_ok && [ ! -f /etc/apt/sources.list.d/nodesource.list ]; then
    log "Adicionando o repositório do Node.js $NODE_MAJOR (NodeSource)..."
    sudo apt-get install -y curl ca-certificates gnupg
    sudo mkdir -p /etc/apt/keyrings
    curl -fsSL https://deb.nodesource.com/gpgkey/nodesource-repo.gpg.key \
        | sudo gpg --dearmor --yes -o /etc/apt/keyrings/nodesource.gpg
    echo "deb [signed-by=/etc/apt/keyrings/nodesource.gpg] https://deb.nodesource.com/node_${NODE_MAJOR}.x nodistro main" \
        | sudo tee /etc/apt/sources.list.d/nodesource.list >/dev/null
fi

# --------------------------------------------------------------------------
# 3. Pacotes apt
# --------------------------------------------------------------------------
log "Instalando pacotes apt (pode demorar)..."
sudo apt-get update
sudo apt-get upgrade -y

APT_PKGS=(
    # ROS 2 base e ferramentas de build
    ros-$ROS_DISTRO-desktop
    ros-dev-tools
    python3-colcon-common-extensions
    python3-rosdep
    # Mapeamento, localização e navegação
    ros-$ROS_DISTRO-rtabmap-ros
    ros-$ROS_DISTRO-navigation2
    ros-$ROS_DISTRO-nav2-bringup
    # Descrição do robô, filtros e visualização
    ros-$ROS_DISTRO-xacro
    ros-$ROS_DISTRO-robot-state-publisher
    ros-$ROS_DISTRO-laser-filters
    ros-$ROS_DISTRO-foxglove-bridge
    ros-$ROS_DISTRO-rviz2
    # Simulação (Gazebo Harmonic)
    ros-$ROS_DISTRO-ros-gz
    # Python (driver, ferramentas, testes, backend)
    python3-serial
    python3-numpy
    python3-pil
    python3-psycopg2
    python3-pytest
    python3-venv
    python3-pip
    # Interface web e banco de dados
    nodejs
    docker.io
    docker-compose-v2
    git
)
if $ROBOT; then
    # Dependências de compilação do driver da Orbbec Astra
    APT_PKGS+=(libuvc-dev libgflags-dev libgoogle-glog-dev nlohmann-json3-dev
               ros-$ROS_DISTRO-image-transport ros-$ROS_DISTRO-image-publisher
               ros-$ROS_DISTRO-camera-info-manager)
fi
sudo apt-get install -y "${APT_PKGS[@]}"

# --------------------------------------------------------------------------
# 4. Drivers do robô físico (código-fonte no workspace)
# --------------------------------------------------------------------------
if $ROBOT; then
    log "Baixando drivers do RPLidar e da Astra em $WS_DIR/src..."
    mkdir -p "$WS_DIR/src"
    [ -d "$WS_DIR/src/sllidar_ros2" ] || \
        git clone https://github.com/Slamtec/sllidar_ros2.git "$WS_DIR/src/sllidar_ros2"
    [ -d "$WS_DIR/src/ros2_astra_camera" ] || \
        git clone https://github.com/JoaoWMHentz/ros2_astra_camera.git "$WS_DIR/src/ros2_astra_camera"
fi

# --------------------------------------------------------------------------
# 5. rosdep (dependências restantes declaradas nos package.xml)
# --------------------------------------------------------------------------
log "Resolvendo dependências com rosdep..."
[ -f /etc/ros/rosdep/sources.list.d/20-default.list ] || sudo rosdep init
rosdep update --rosdistro "$ROS_DISTRO"
set +u; source /opt/ros/$ROS_DISTRO/setup.bash; set -u
SKIP_KEYS="description hardware bringup navigation sim"
$ROBOT || SKIP_KEYS="$SKIP_KEYS sllidar_ros2 astra_camera"
rosdep install --from-paths "$WS_DIR/src" --ignore-src -y -r \
    --rosdistro "$ROS_DISTRO" --skip-keys "$SKIP_KEYS" || \
    warn "rosdep não resolveu algumas chaves (veja acima)."

# --------------------------------------------------------------------------
# 6. Docker: serviço e grupo do usuário
# --------------------------------------------------------------------------
log "Configurando o Docker..."
sudo systemctl enable --now docker
NEED_RELOGIN=false
if ! id -nG "$USER" | grep -qw docker; then
    sudo usermod -aG docker "$USER"
    NEED_RELOGIN=true
fi

# --------------------------------------------------------------------------
# 7. Regras udev (robô físico)
# --------------------------------------------------------------------------
if $ROBOT; then
    log "Instalando regras udev (/dev/ttyMOTOR, /dev/ttyIMU, /dev/ttyLIDAR)..."
    sudo cp "$REPO_DIR/bringup/udev/99-robot.rules" /etc/udev/rules.d/
    for f in "$WS_DIR"/src/ros2_astra_camera/astra_camera/scripts/56-orbbec-usb.rules; do
        [ -f "$f" ] && sudo cp "$f" /etc/udev/rules.d/
    done
    sudo udevadm control --reload-rules && sudo udevadm trigger
    if ! id -nG "$USER" | grep -qw dialout; then
        sudo usermod -aG dialout "$USER"
        NEED_RELOGIN=true
    fi
    warn "Edite os números de série em /etc/udev/rules.d/99-robot.rules (SERIAL_DO_LIDAR, SERIAL_DO_FTDI)."
fi

# --------------------------------------------------------------------------
# 8. Backend web (venv Python)
# --------------------------------------------------------------------------
log "Criando o ambiente Python do backend..."
[ -d "$VENV" ] || python3 -m venv --system-site-packages "$VENV"
"$VENV/bin/pip" install -q --upgrade pip
"$VENV/bin/pip" install -q -r "$WEB_DIR/backend/requirements.txt"

# --------------------------------------------------------------------------
# 9. Frontend web
# --------------------------------------------------------------------------
log "Instalando e compilando o frontend ($(node -v))..."
(cd "$WEB_DIR/frontend" && npm ci && npm run build)

# --------------------------------------------------------------------------
# 10. Configuração local e compilação do workspace
# --------------------------------------------------------------------------
[ -f "$REPO_DIR/.env" ] || cp "$REPO_DIR/.env.example" "$REPO_DIR/.env"

log "Compilando o workspace ($WS_DIR)..."
if $ROBOT; then
    (cd "$WS_DIR" && colcon build --symlink-install)   # inclui os drivers
else
    (cd "$WS_DIR" && colcon build --symlink-install \
        --packages-select description hardware bringup navigation sim)
fi

BASHRC_LINE="source /opt/ros/$ROS_DISTRO/setup.bash"
grep -qxF "$BASHRC_LINE" ~/.bashrc || echo "$BASHRC_LINE" >> ~/.bashrc

log "Instalação concluída."
echo "  Simulação:   ./scripts/start_sim.sh   (interface em http://localhost:5000)"
$ROBOT && echo "  Robô físico: ./scripts/start_robot.sh"
if $NEED_RELOGIN; then
    warn "Seu usuário foi adicionado a grupos (docker/dialout): faça logout/login (ou reinicie) antes de usar."
fi
