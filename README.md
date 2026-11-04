# Mapeamento, localização e gestão de rotas para robô móvel em ambiente comercial

Software do Trabalho de Conclusão de Curso de Engenharia de Software (Católica
de Santa Catarina) de João Witor Müller Hentz.

Plataforma robótica móvel diferencial, construída sobre **ROS 2 Jazzy**, que:

- **mapeia e se localiza** com o **RTAB-Map**, fundindo LiDAR 2D, câmera RGB-D
  (Orbbec Astra), IMU e odometria das rodas, com fechamento de loop para
  corrigir a deriva;
- **planeja e replaneja rotas** com o **Nav2**, desviando de obstáculos
  dinâmicos (pessoas);
- é operada por uma **interface web** (React + TypeScript), em que o operador
  vê o mapa, define destinos e rotas e acompanha o robô sem precisar de
  conhecimento em robótica;
- funciona **igual em simulação (Gazebo Harmonic) e no robô físico**.

Licença: [Apache-2.0](LICENSE).

---

## Arquitetura

```
┌──────────────── Interface ────────────────┐
│ web/frontend  React + TypeScript (canvas)  │
│ web/backend   Flask + Socket.IO + rclpy    │──── PostgreSQL (destinos, rotas, histórico)
└───────────────────┬────────────────────────┘
                    │ tópicos e actions ROS 2
┌─────────────── Computação ────────────────┐
│ navigation   RTAB-Map (mapa + localização) │
│              Nav2 (planejamento e controle)│
└───────────────────┬────────────────────────┘
        /scan /odom /imu/data /camera/*      /cmd_vel
┌──────── Percepção e atuação ──────────────┐
│ robô real : bringup + hardware (driver)    │
│ simulação : sim (Gazebo Harmonic)          │
└────────────────────────────────────────────┘
```

Os dois "fundos" (real e simulado) publicam **os mesmos tópicos**, então o
RTAB-Map, o Nav2 e a interface não sabem em qual estão rodando. Diagramas de
fluxo de dados em [docs/arquitetura.md](docs/arquitetura.md).

| Pacote / pasta | Conteúdo |
|---|---|
| `description` | URDF do robô (geometria, rodas, LiDAR, câmera, IMU) |
| `hardware` | Driver ROS 2 da base (`hardware_node`) e firmwares dos microcontroladores |
| `bringup` | Launch do robô físico: driver, RPLidar, Astra, filtro do laser |
| `navigation` | Configuração e launches do RTAB-Map e do Nav2 |
| `sim` | Mundo Gazebo de uma loja, robô simulado e ponte de tópicos |
| `web` | Interface de operação (backend Python e frontend TypeScript) |
| `tools` | Métricas de avaliação e emulador dos firmwares |
| `tests` | Testes automatizados |

### Hardware de referência

| Componente | Modelo | Ligação |
|---|---|---|
| Computador | NVIDIA Jetson Orin Nano Super (JetPack, Ubuntu) | — |
| Motores | 2 × BLDC de hoverboard com sensores Hall | ESP32 + SimpleFOC → Serial2 (GPIO16/17) → USB-serial |
| IMU | BNO085 | STM32F401 (I2C) → USB CDC |
| LiDAR 2D | Slamtec RPLidar | USB-serial |
| Câmera RGB-D | Orbbec Astra | USB |

---

## Instalação

Ubuntu 24.04 com ROS 2 Jazzy ([guia oficial](https://docs.ros.org/en/jazzy/Installation.html)).

```bash
sudo apt install \
  ros-jazzy-rtabmap-ros ros-jazzy-navigation2 ros-jazzy-nav2-bringup \
  ros-jazzy-laser-filters ros-jazzy-xacro ros-jazzy-robot-state-publisher \
  ros-jazzy-ros-gz ros-jazzy-foxglove-bridge python3-serial \
  docker.io docker-compose nodejs npm

mkdir -p ~/ros2_ws/src && cd ~/ros2_ws/src
git clone <este repositório> TCC_joao

# Somente no robô físico: drivers do LiDAR e da câmera
git clone https://github.com/Slamtec/sllidar_ros2.git
git clone https://github.com/JoaoWMHentz/ros2_astra_camera.git   # fork compatível com o Jazzy

cd ~/ros2_ws && colcon build --symlink-install
```

O usuário precisa estar no grupo `docker` (`sudo usermod -aG docker $USER`).

---

## Uso

### Simulação

```bash
cd ~/ros2_ws/src/TCC_joao
./scripts/start_sim.sh              # Gazebo com janela + interface web
./scripts/start_sim.sh --people     # com uma pessoa caminhando (obstáculo dinâmico)
./scripts/start_sim.sh --headless   # sem janela do Gazebo
```

Abra **http://localhost:5000**. O script compila o workspace e a interface, e
sobe o PostgreSQL na primeira execução.

### Robô físico

1. Grave os firmwares (ver [hardware/firmware](#firmwares)).
2. Instale as regras udev, que criam `/dev/ttyMOTOR`, `/dev/ttyIMU` e
   `/dev/ttyLIDAR`. Edite os números de série no arquivo antes:
   `sudo cp bringup/udev/99-robot.rules /etc/udev/rules.d/ && sudo udevadm trigger`
3. Execute `./scripts/start_robot.sh` e acesse `http://<ip-do-robô>:5000`.

### Fluxo na interface

1. **Mapear**: em *Operar*, dê um nome e toque em *Mapear*. Conduza o robô
   pelo ambiente tocando no mapa (*Ir para*) ou com as setas. Toque em
   *Concluir e salvar mapa*.
2. **Navegar**: escolha o mapa salvo e toque em *Abrir*. Se o robô não
   aparecer no lugar certo, use *Corrigir posição*.
3. **Destinos**: salve pontos com nome (*Marcar destino no mapa* ou *Aqui*,
   na posição atual) e envie o robô com *Ir*.
4. **Rotas**: monte uma sequência de destinos e toque em *Iniciar*.
5. **Histórico**: resultado, tempo e distância de cada navegação.

### Sem a interface (linha de comando)

```bash
# simulação
ros2 launch sim robot_sim.launch.py people:=true
# ou robô real
ros2 launch bringup robot.launch.py

ros2 launch navigation slam.launch.py mode:=mapping db_path:=~/tcc_maps/loja.db [use_sim_time:=true]
ros2 launch navigation nav2.launch.py allow_unknown:=true [use_sim_time:=true]
```

---

## Avaliação

As métricas do TCC saem de duas ferramentas:

```bash
# Precisão de localização (simulação): pose do RTAB-Map × pose real no Gazebo
python3 tools/metrics/localization_error.py --duration 120 --csv erro_localizacao.csv

# Navegações registradas pela interface: taxa de sucesso, recuperações,
# menor folga até obstáculos (colisões) e erro final
web/backend/.venv/bin/python tools/metrics/export_history.py --csv historico.csv
```

Cada navegação disparada pela interface é gravada na tabela `navigation_log`
com: resultado, duração, distância percorrida, número de recuperações do Nav2,
menor distância do corpo do robô a um obstáculo (pelo LiDAR) e erro final em
relação ao destino.

---

## Firmwares

| Arquivo | Placa | Bibliotecas |
|---|---|---|
| `hardware/firmware/esp32_motor_controller` | ESP32 | Simple FOC |
| `hardware/firmware/stm32_imu` | STM32F401 "Black Pill" (STM32duino) | Adafruit BNO08x |

O protocolo serial (texto, uma mensagem por linha) está documentado em
[hardware/hardware/protocol.py](hardware/hardware/protocol.py). Para testar o
driver sem o robô:

```bash
python3 tools/fake_firmware.py      # imprime duas portas virtuais
ros2 run hardware hardware_node --ros-args -p motor_port:=<MOTOR> -p imu_port:=<IMU>
```

---

## Testes

```bash
python3 -m pytest tests                       # protocolo, cinemática e odometria
cd web/frontend && npm run typecheck          # tipos do frontend
```
