# Arquitetura do software

## 1. Camadas

| Camada | Robô físico | Simulação |
|---|---|---|
| Percepção | RPLidar, Orbbec Astra, IMU BNO085 (STM32F401) | sensores do Gazebo (`gpu_lidar`, `rgbd_camera`, `imu`) |
| Atuação | ESP32 + SimpleFOC, motores BLDC | plugin `DiffDrive` do Gazebo |
| Driver | `hardware/hardware_node` | ponte `ros_gz_bridge` + `cmd_vel_relay` |
| Computação | RTAB-Map + Nav2 (pacote `navigation`) | idem, com `use_sim_time` |
| Interface | backend Flask/rclpy + frontend React/TypeScript | idem, com `ROBOT_SIM=1` |

As duas colunas convergem para os mesmos tópicos (`/scan`, `/odom`,
`/imu/data`, `/camera/*`, `/cmd_vel`). Por isso a computação e a interface são
idênticas nos dois casos.

## 2. Fluxo de dados entre os módulos

```mermaid
flowchart LR
    subgraph Percepcao[Percepção e atuação]
        LIDAR[RPLidar] -->|/scan_raw| FILT[scan_filter_chain]
        CAM[Orbbec Astra] -->|/camera/color/*<br>/camera/depth/*| X(( ))
        IMU[STM32 + BNO085] -->|serial 'I ...'| HW[hardware_node]
        ESC[ESP32 + SimpleFOC] <-->|serial 'M ...' / 'C ...'| HW
    end

    FILT -->|/scan| RTAB
    FILT -->|/scan| NAV
    X --> RTAB
    X -->|/camera/depth/points| NAV
    HW -->|/odom + TF odom→base_link| RTAB
    HW -->|/imu/data| RTAB
    HW -->|/odom| NAV

    subgraph Computacao[Computação]
        RTAB[RTAB-Map] -->|/map + TF map→odom| NAV[Nav2]
    end

    NAV -->|/cmd_vel| HW
    RTAB -->|/map| WEB
    NAV -->|/plan| WEB
    WEB[web_bridge] -->|NavigateToPose| NAV
    WEB -->|/initialpose| RTAB
    WEB -->|/cmd_vel teleop| HW

    subgraph Interface
        WEB <-->|HTTP + Socket.IO| UI[Navegador]
        WEB <--> DB[(PostgreSQL)]
    end
```

Cadeia de velocidade dentro do Nav2:
`controller_server → /cmd_vel_nav → velocity_smoother → /cmd_vel`.

## 3. Árvore de transformações (TF)

```mermaid
flowchart TD
    map -->|RTAB-Map: correção de deriva| odom
    odom -->|hardware_node / DiffDrive: odometria| base_link
    base_link --> chassis_link
    chassis_link --> left_wheel_link & right_wheel_link
    chassis_link --> imu_link
    chassis_link --> lidar_link --> lidar_scan_link
    chassis_link --> camera_link
    camera_link --> camera_color_frame --> camera_color_optical_frame
    camera_link --> camera_depth_frame --> camera_depth_optical_frame
```

- `odom → base_link` é contínua, sem saltos: a velocidade linear vem dos
  encoders e a angular do giroscópio.
- `map → odom` é publicada pelo RTAB-Map e absorve as correções do
  ICP/fechamento de loop. O robô "salta" no mapa, nunca na odometria.

## 4. Mapeamento e localização (RTAB-Map)

| | Mapeamento | Localização |
|---|---|---|
| Memória | incremental (grava o `.db`) | somente leitura |
| Registro | ICP sobre o LiDAR | visual (com câmera) ou ICP |
| Fechamento de loop | aparência visual (GFTT/ORB, bag-of-words) | idem, para relocalizar |
| Grid 2D para o Nav2 | só LiDAR (`Grid/Sensor=0`) | carregado do `.db` |

Arquivos: `navigation/config/rtabmap_common.yaml` (comum) mais
`rtabmap_mapping.yaml` ou `rtabmap_localization.yaml`.

## 5. Gestão de rotas (Nav2)

- **Planejador global**: NavFn (A*) sobre o costmap global, que é o mapa do
  RTAB-Map somado aos obstáculos atuais. A behavior tree padrão replaneja a
  1 Hz.
- **Controlador local**: Regulated Pure Pursuit com detecção de colisão à
  frente.
- **Obstáculos dinâmicos**: camada de obstáculos com o LiDAR (marca e limpa) e
  com a nuvem da câmera (objetos acima/abaixo do plano do laser).
- **Recuperações**: girar, recuar e esperar, quando o caminho fica bloqueado.
- **Rotas da interface**: o backend executa cada destino da rota como um
  `NavigateToPose` separado, o que permite medir cada trecho.

## 6. Interface web

```mermaid
sequenceDiagram
    actor Op as Operador
    participant UI as Navegador
    participant API as Backend (Flask)
    participant L as Launcher
    participant R as web_bridge (rclpy)
    participant N as Nav2

    Op->>UI: Abrir mapa "loja"
    UI->>API: POST /api/session/navigation
    API->>L: start_session
    L->>L: base → (5 s) → slam (localização) + nav
    R-->>UI: map_update, pose_update (Socket.IO)
    Op->>UI: Destino "caixa" → Ir
    UI->>API: POST /api/points/1/go
    API->>R: start_mission
    R->>N: NavigateToPose
    R->>API: navigation_log (início)
    N-->>R: feedback (distância restante)
    R-->>UI: goal_status, path_update
    N-->>R: resultado
    R->>API: navigation_log (resultado e métricas)
    R-->>UI: goal_status, history_update
```

O backend sobe a pilha ROS como três processos (`base`, `slam`, `nav`). Ao
parar, envia SIGINT a cada um, e o RTAB-Map grava o mapa antes de sair.

### Banco de dados

```mermaid
erDiagram
    points {
        int id PK
        text map
        text name
        float x
        float y
        float theta
    }
    routes {
        int id PK
        text map
        text name
        int_array point_ids
    }
    navigation_log {
        int id PK
        text map
        text kind
        text label
        jsonb target
        text result
        float duration_s
        float distance_m
        int recoveries
        float min_clearance_m
        float final_error_m
    }
    routes }o--o{ points : "sequência"
```

## 7. Protocolo serial com os microcontroladores

Texto ASCII, uma mensagem por linha, 115200 bps.

| Direção | Mensagem | Significado |
|---|---|---|
| host → ESP32 | `C <vel_esq> <vel_dir>` | velocidade-alvo das rodas (rad/s) |
| ESP32 → host | `M <vel_esq> <vel_dir> <pos_esq> <pos_dir> R` | telemetria a 50 Hz |
| STM32 → host | `I <yaw> <pitch> <roll> <gx> <gy> <gz> <ax> <ay> <az>` | IMU a 50 Hz (rad, rad/s, m/s²) |
| qualquer → host | `# <texto>` | log |

Segurança em três níveis:

1. o firmware zera a velocidade sem comandos por 500 ms;
2. o `hardware_node` para sem `/cmd_vel` por 0,5 s;
3. o `hardware_node` recusa comandos se a telemetria dos motores parar.
