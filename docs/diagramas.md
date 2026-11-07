# Diagramas de especificação

Diagramas exigidos e recomendados pela
[direction de IoT](https://github.com/CatolicaSC-Portfolio/The-Portfolio-Playbook/blob/main/directions/portfolio-directions-iot.md),
cada um com a descrição e a fundamentação. Todos estão em Mermaid, renderizados
pelo próprio GitHub, e foram feitos a partir do código deste repositório (os
nomes de tópicos, pinos, frequências e limites são os reais).

| # | Diagrama | Exigência na direction de IoT |
|---|---|---|
| 1 | [Requisitos (SysML, hierárquico)](#1-diagrama-hierárquico-de-requisitos-sysml) | obrigatório |
| 2 | [Arquitetura em camadas IoT](#2-arquitetura-em-camadas-iot) | obrigatório (C4 ou camadas) |
| 3 | [C4 — contexto, containers e componentes](#3-c4-model) | obrigatório (C4 ou camadas) |
| 4 | [Definição de blocos (SysML BDD)](#4-diagrama-de-definição-de-blocos-sysml-bdd) | desejável |
| 5 | [Blocos internos (SysML IBD)](#5-diagrama-de-blocos-internos-sysml-ibd) | desejável |
| 6 | [Malhas de controle](#6-malhas-de-controle) | complementar |
| 7 | [Máquinas de estado da exploração autônoma](#7-máquinas-de-estado-da-exploração-autônoma) | complementar |
| 8 | [Camadas de segurança](#8-camadas-de-segurança) | complementar |

Fluxo de dados entre os nós ROS, árvore de TF, sequência da interface e modelo
de dados estão em [arquitetura.md](arquitetura.md).

---

## 1. Diagrama hierárquico de requisitos (SysML)

O requisito raiz (R0) se desdobra em requisitos funcionais (RF), de desempenho
e de segurança (RNF), e de interface (RI). Cada folha aponta o elemento que a
**satisfaz** (código ou configuração) e o que a **verifica** (teste automatizado
ou ferramenta de métrica).

### 1.1 Visão geral

```mermaid
requirementDiagram

    requirement R0 {
        id: R0
        text: "Robô móvel que mapeia, se localiza e navega com autonomia em ambiente comercial"
        risk: high
        verifymethod: demonstration
    }

    functionalRequirement RF1 {
        id: RF1
        text: "Mapear o ambiente (SLAM)"
        risk: high
        verifymethod: demonstration
    }
    functionalRequirement RF2 {
        id: RF2
        text: "Localizar-se em um mapa salvo"
        risk: high
        verifymethod: analysis
    }
    functionalRequirement RF3 {
        id: RF3
        text: "Planejar e seguir rotas desviando de obstáculos"
        risk: high
        verifymethod: demonstration
    }
    functionalRequirement RF4 {
        id: RF4
        text: "Controlar a base diferencial e estimar a odometria"
        risk: high
        verifymethod: test
    }
    functionalRequirement RF5 {
        id: RF5
        text: "Explorar o ambiente sem operador"
        risk: medium
        verifymethod: test
    }
    functionalRequirement RF6 {
        id: RF6
        text: "Ser operado e acompanhado por interface"
        risk: low
        verifymethod: demonstration
    }
    requirement RNF {
        id: RNF
        text: "Segurança, desempenho e portabilidade"
        risk: high
        verifymethod: test
    }
    interfaceRequirement RI1 {
        id: RI1
        text: "Protocolo serial com os microcontroladores"
        risk: medium
        verifymethod: test
    }

    R0 - contains -> RF1
    R0 - contains -> RF2
    R0 - contains -> RF3
    R0 - contains -> RF4
    R0 - contains -> RF5
    R0 - contains -> RF6
    R0 - contains -> RNF
    R0 - contains -> RI1
```

### 1.2 Mapeamento, localização e rotas (RF1–RF3)

```mermaid
requirementDiagram

    functionalRequirement RF1 {
        id: RF1
        text: "Mapear o ambiente (SLAM)"
        risk: high
        verifymethod: demonstration
    }
    functionalRequirement RF1_1 {
        id: RF1.1
        text: "Fundir LiDAR 2D, RGB-D, IMU e odometria das rodas"
        risk: high
        verifymethod: inspection
    }
    functionalRequirement RF1_2 {
        id: RF1.2
        text: "Corrigir a deriva com fechamento de loop"
        risk: high
        verifymethod: analysis
    }
    functionalRequirement RF2 {
        id: RF2
        text: "Localizar-se em um mapa salvo"
        risk: high
        verifymethod: analysis
    }
    functionalRequirement RF2_1 {
        id: RF2.1
        text: "Relocalizar e aceitar correção manual de pose"
        risk: medium
        verifymethod: demonstration
    }
    functionalRequirement RF3 {
        id: RF3
        text: "Planejar e seguir rotas desviando de obstáculos"
        risk: high
        verifymethod: demonstration
    }
    functionalRequirement RF3_1 {
        id: RF3.1
        text: "Replanejar diante de obstáculos dinâmicos (pessoas)"
        risk: high
        verifymethod: demonstration
    }
    functionalRequirement RF3_2 {
        id: RF3.2
        text: "Executar rotas como sequência de destinos"
        risk: low
        verifymethod: demonstration
    }

    element rtabmap_cfg {
        type: "configuração"
        docref: "navigation/config/rtabmap_*.yaml"
    }
    element nav2_cfg {
        type: "configuração"
        docref: "navigation/config/nav2_params.yaml"
    }
    element ros_bridge {
        type: "módulo Python"
        docref: "web/backend/ros_bridge.py"
    }
    element loc_metric {
        type: "ferramenta de métrica"
        docref: "tools/metrics/localization_error.py"
    }
    element nav_metric {
        type: "ferramenta de métrica"
        docref: "tools/metrics/export_history.py"
    }

    RF1 - contains -> RF1_1
    RF1 - contains -> RF1_2
    RF2 - contains -> RF2_1
    RF3 - contains -> RF3_1
    RF3 - contains -> RF3_2
    rtabmap_cfg - satisfies -> RF1_1
    rtabmap_cfg - satisfies -> RF1_2
    rtabmap_cfg - satisfies -> RF2_1
    nav2_cfg - satisfies -> RF3_1
    ros_bridge - satisfies -> RF3_2
    loc_metric - verifies -> RF2
    nav_metric - verifies -> RF3
```

### 1.3 Controle da base, exploração e interface (RF4–RF6, RI1)

```mermaid
requirementDiagram

    functionalRequirement RF4 {
        id: RF4
        text: "Controlar a base diferencial e estimar a odometria"
        risk: high
        verifymethod: test
    }
    functionalRequirement RF4_1 {
        id: RF4.1
        text: "Controle de velocidade das rodas em malha fechada (FOC + Hall)"
        risk: high
        verifymethod: demonstration
    }
    functionalRequirement RF4_2 {
        id: RF4.2
        text: "Cinemática diferencial direta e inversa"
        risk: medium
        verifymethod: test
    }
    functionalRequirement RF4_3 {
        id: RF4.3
        text: "Odometria: linear dos encoders, angular do giroscópio"
        risk: medium
        verifymethod: test
    }
    functionalRequirement RF5 {
        id: RF5
        text: "Explorar o ambiente sem operador"
        risk: medium
        verifymethod: test
    }
    functionalRequirement RF5_1 {
        id: RF5.1
        text: "Estratégias: fronteiras, parede + BFS, desbaste em offsets"
        risk: medium
        verifymethod: test
    }
    functionalRequirement RF5_2 {
        id: RF5.2
        text: "Voltar ao ponto de partida ao terminar"
        risk: low
        verifymethod: test
    }
    functionalRequirement RF6 {
        id: RF6
        text: "Ser operado e acompanhado por interface"
        risk: low
        verifymethod: demonstration
    }
    interfaceRequirement RI1 {
        id: RI1
        text: "Serial ASCII 115200 bps, uma mensagem por linha, telemetria a 50 Hz"
        risk: medium
        verifymethod: test
    }

    element fw_esp32 {
        type: "firmware"
        docref: "hardware/firmware/esp32_motor_controller"
    }
    element protocol {
        type: "módulo Python"
        docref: "hardware/hardware/protocol.py"
    }
    element hw_node {
        type: "nó ROS 2"
        docref: "hardware/hardware/hardware_node.py"
    }
    element explorers {
        type: "nós ROS 2"
        docref: "navigation/scripts/*_explorer.py"
    }
    element webui {
        type: "aplicação web"
        docref: "web/"
    }
    element t_protocol {
        type: "teste"
        docref: "tests/test_protocol.py"
    }
    element t_hw {
        type: "teste"
        docref: "tests/test_hardware_node.py"
    }
    element t_explore {
        type: "teste"
        docref: "tests/test_*_explorer.py"
    }

    RF4 - contains -> RF4_1
    RF4 - contains -> RF4_2
    RF4 - contains -> RF4_3
    RF5 - contains -> RF5_1
    RF5 - contains -> RF5_2
    fw_esp32 - satisfies -> RF4_1
    protocol - satisfies -> RF4_2
    hw_node - satisfies -> RF4_3
    protocol - satisfies -> RI1
    explorers - satisfies -> RF5_1
    explorers - satisfies -> RF5_2
    webui - satisfies -> RF6
    t_protocol - verifies -> RF4_2
    t_protocol - verifies -> RI1
    t_hw - verifies -> RF4_3
    t_explore - verifies -> RF5_1
    t_explore - verifies -> RF5_2
```

### 1.4 Requisitos não funcionais (RNF)

```mermaid
requirementDiagram

    requirement RNF {
        id: RNF
        text: "Segurança, desempenho e portabilidade"
        risk: high
        verifymethod: test
    }
    designConstraint RNF1 {
        id: RNF1
        text: "Parar os motores sem comando: 500 ms no firmware, 0,5 s no driver"
        risk: high
        verifymethod: test
    }
    designConstraint RNF2 {
        id: RNF2
        text: "Recusar comandos sem telemetria dos motores por mais de 1 s"
        risk: high
        verifymethod: test
    }
    performanceRequirement RNF3 {
        id: RNF3
        text: "Velocidade máxima de 1,0 m/s e 1,5 rad/s"
        risk: medium
        verifymethod: test
    }
    performanceRequirement RNF4 {
        id: RNF4
        text: "Telemetria a 50 Hz, odometria a 25 Hz, replanejamento a 1 Hz"
        risk: medium
        verifymethod: inspection
    }
    designConstraint RNF5 {
        id: RNF5
        text: "Mesmo software no robô físico e na simulação (mesmos tópicos)"
        risk: medium
        verifymethod: demonstration
    }
    physicalRequirement RNF6 {
        id: RNF6
        text: "Processamento embarcado em Jetson Orin Nano, motores BLDC a 24 V"
        risk: medium
        verifymethod: inspection
    }

    element hw_yaml {
        type: "configuração"
        docref: "bringup/config/hardware.yaml"
    }
    element sim_pkg {
        type: "pacote ROS 2"
        docref: "sim/"
    }
    element t_hw {
        type: "teste"
        docref: "tests/test_hardware_node.py"
    }

    RNF - contains -> RNF1
    RNF - contains -> RNF2
    RNF - contains -> RNF3
    RNF - contains -> RNF4
    RNF - contains -> RNF5
    RNF - contains -> RNF6
    hw_yaml - satisfies -> RNF1
    hw_yaml - satisfies -> RNF2
    hw_yaml - satisfies -> RNF3
    sim_pkg - satisfies -> RNF5
    t_hw - verifies -> RNF1
    t_hw - verifies -> RNF2
    t_hw - verifies -> RNF3
```

**Fundamentação.** A hierarquia segue o objetivo do projeto, que é de controle
e navegação: RF1–RF4 formam a pilha de autonomia (perceber → estimar →
planejar → atuar), e RF4 é a base de todos os outros, porque mapa e
localização dependem de uma odometria confiável. A interface (RF6) é um
requisito de baixo risco, complementar: tudo o que ela faz também pode ser
feito pela linha de comando (`ros2 launch`, `ros2 action send_goal`). Os
requisitos de segurança (RNF1–RNF3) são verificados por testes automatizados,
porque são os únicos cuja falha causa dano físico.

---

## 2. Arquitetura em camadas IoT

```mermaid
flowchart TB
    subgraph L4["Camada de aplicação"]
        direction LR
        UI["Interface web<br/>React + TypeScript"]
        CLI["Linha de comando<br/>ros2 launch / ros2 action"]
        DB[("PostgreSQL<br/>destinos, rotas, histórico")]
    end

    subgraph L3["Camada de processamento de dados (Jetson Orin Nano, ROS 2 Jazzy)"]
        direction LR
        RTAB["RTAB-Map<br/>SLAM e localização"]
        NAV["Nav2<br/>planejamento e controle"]
        EXP["Exploração autônoma<br/>3 estratégias"]
        HWN["hardware_node<br/>cinemática, odometria, segurança"]
    end

    subgraph L2["Camada de rede"]
        direction LR
        WIFI["Wi-Fi / Ethernet (TCP/IP)<br/>HTTP + Socket.IO"]
        DDS["DDS (ROS 2)<br/>tópicos, serviços, actions"]
        SER["USB-serial / USB CDC<br/>ASCII 115200 bps"]
    end

    subgraph L1["Camada de sensores e atuadores"]
        direction LR
        LID["RPLidar 2D"]
        AST["Orbbec Astra RGB-D"]
        IMU["BNO085<br/>(STM32F401)"]
        MOT["2 motores BLDC + Hall<br/>(ESP32 + SimpleFOC)"]
    end

    UI <--> WIFI
    CLI <--> DDS
    WIFI <--> DDS
    DB <--> WIFI
    DDS <--> RTAB & NAV & EXP & HWN
    HWN <--> SER
    SER <--> IMU & MOT
    LID -->|USB| DDS
    AST -->|USB| DDS
```

**Descrição das camadas.**

- **Sensores e atuadores.** Dois microcontroladores com firmware próprio
  isolam o tempo real do computador: o ESP32 fecha a malha de velocidade
  dos motores (FOC com sensores Hall) e o STM32 lê a IMU por I2C. O LiDAR e a
  câmera usam os drivers ROS dos fabricantes.
- **Rede.** Três meios, cada um escolhido pelo requisito que atende: serial de
  texto com os microcontroladores (simples, legível no terminal, sem
  dependências no firmware); DDS entre os nós ROS (publicação/assinatura com
  QoS, inclusive entre máquinas); HTTP e WebSocket (Socket.IO) para a
  interface, que precisa de atualização contínua da pose e do mapa sem
  *polling*.
- **Processamento.** Embarcado no Jetson, porque SLAM e controle precisam de
  latência baixa e não podem depender de link externo. É a camada que
  concentra o trabalho do TCC.
- **Aplicação.** Interface web servida pelo próprio robô, mais a linha de
  comando, que dá acesso a todas as funções sem a interface.

---

## 3. C4 Model

### 3.1 Nível 1 — Contexto

```mermaid
C4Context
    title Contexto — robô móvel autônomo para ambiente comercial

    Person(op, "Operador", "Funcionário da loja. Mapeia o ambiente, cadastra destinos e rotas, envia o robô.")
    Person(pub, "Pessoas no ambiente", "Clientes e funcionários: obstáculos dinâmicos.")
    Person(dev, "Desenvolvedor", "Opera pela linha de comando, RViz ou Foxglove.")

    System(robot, "Robô móvel autônomo", "Mapeia, se localiza e navega com LiDAR, câmera RGB-D, IMU e odometria.")
    System_Ext(gz, "Gazebo Harmonic", "Simulação do robô e do ambiente (supermercado e loja), com pessoas caminhando.")

    Rel(op, robot, "Opera pela interface web", "HTTP / WebSocket")
    Rel(dev, robot, "Comanda e inspeciona", "ROS 2 / DDS")
    Rel(robot, pub, "Detecta e desvia", "LiDAR / RGB-D")
    Rel(gz, robot, "Substitui sensores e atuadores", "mesmos tópicos ROS 2")
```

### 3.2 Nível 2 — Containers

```mermaid
C4Container
    title Containers — robô móvel autônomo

    Person(op, "Operador")

    System_Boundary(robot, "Robô móvel autônomo") {
        Container(fw_m, "Firmware dos motores", "C++ / ESP32 / SimpleFOC", "Controle FOC de velocidade de 2 BLDC; telemetria a 50 Hz; watchdog de 500 ms.")
        Container(fw_i, "Firmware da IMU", "C++ / STM32F401", "Lê a BNO085 por I2C e transmite orientação, giro e aceleração a 50 Hz.")
        Container(drv, "Driver da base", "Python / ROS 2 (hardware_node)", "Cinemática, odometria com fusão da IMU, segurança, diagnóstico.")
        Container(sens, "Drivers de sensores", "ROS 2 (sllidar, astra_camera, laser_filters)", "Publicam /scan e /camera/*.")
        Container(slam, "SLAM", "RTAB-Map", "Mapa 2D, localização e fechamento de loop; mapas em arquivos .db.")
        Container(nav, "Navegação", "Nav2", "Planejador global (NavFn), controlador Regulated Pure Pursuit, recuperações.")
        Container(exp, "Exploração autônoma", "Python / ROS 2", "Fronteiras, parede + BFS, desbaste em offsets.")
        Container(api, "Backend web", "Python / Flask / Socket.IO / rclpy", "API REST, ponte com o ROS, sobe e derruba a pilha.")
        Container(spa, "Frontend web", "TypeScript / React", "Mapa em canvas, destinos, rotas, teleoperação, histórico.")
        ContainerDb(db, "Banco", "PostgreSQL 16 (Docker)", "Destinos, rotas e histórico de navegações.")
    }

    Rel(op, spa, "Usa", "navegador")
    Rel(spa, api, "Chama e assina", "HTTP / Socket.IO")
    Rel(api, db, "Lê e grava", "SQL")
    Rel(api, nav, "NavigateToPose", "action ROS 2")
    Rel(api, slam, "/initialpose", "tópico ROS 2")
    Rel(exp, nav, "NavigateToPose / ThroughPoses", "action ROS 2")
    Rel(sens, slam, "/scan, /camera/*", "DDS")
    Rel(drv, slam, "/odom, /imu/data, TF", "DDS")
    Rel(slam, nav, "/map, TF map→odom", "DDS")
    Rel(nav, drv, "/cmd_vel", "DDS")
    Rel(drv, fw_m, "C vl vr / M ...", "USB-serial 115200")
    Rel(fw_i, drv, "I ...", "USB CDC 115200")
```

### 3.3 Nível 3 — Componentes do driver da base

```mermaid
C4Component
    title Componentes — driver da base (pacote hardware)

    Container_Boundary(drv, "hardware_node") {
        Component(node, "HardwareNode", "rclpy Node", "Assina /cmd_vel; publica /odom, TF, /joint_states, /imu/data e /diagnostics; timers de odometria (25 Hz), watchdog (10 Hz) e diagnóstico (1 Hz).")
        Component(proto, "protocol", "funções puras", "Parsing dos quadros M e I, formatação do comando C, cinemática direta/inversa, piso de velocidade, integração da pose, convenção de sinais.")
        Component(link, "SerialLink", "thread + pyserial", "Leitura por linhas, reconexão automática, idade da última mensagem.")
    }

    Container_Ext(fw_m, "Firmware dos motores", "ESP32")
    Container_Ext(fw_i, "Firmware da IMU", "STM32")
    Container_Ext(nav, "Nav2 / teleoperação")
    Container_Ext(slam, "RTAB-Map")

    Rel(nav, node, "/cmd_vel")
    Rel(node, proto, "converte e valida")
    Rel(node, link, "write_line / on_line")
    Rel(link, fw_m, "USB-serial")
    Rel(fw_i, link, "USB CDC")
    Rel(node, slam, "/odom, /imu/data, TF odom→base_link")
```

### 3.4 Nível 3 — Componentes do backend web

```mermaid
C4Component
    title Componentes — backend web

    Container(spa, "Frontend", "React")

    Container_Boundary(api, "Backend web") {
        Component(app, "app", "Flask + Socket.IO", "Servidor, eventos em tempo real, inicialização.")
        Component(rest, "api", "Blueprint Flask", "Rotas REST: sessão, mapas, destinos, rotas, objetivo, teleoperação, exploração.")
        Component(bridge, "ros_bridge", "rclpy", "Mapa → PNG, pose, plano, missões NavigateToPose, métricas de cada navegação.")
        Component(launch, "launcher", "subprocess", "Sobe e derruba base, slam e nav como grupos de processos.")
        Component(dbm, "db", "psycopg2", "Esquema e consultas.")
    }

    ContainerDb(db, "PostgreSQL")
    Container_Ext(ros, "Pilha ROS 2", "RTAB-Map, Nav2, drivers")

    Rel(spa, rest, "HTTP")
    Rel(spa, app, "Socket.IO")
    Rel(rest, bridge, "comandos")
    Rel(rest, launch, "sessões")
    Rel(rest, dbm, "CRUD")
    Rel(bridge, dbm, "navigation_log")
    Rel(bridge, ros, "tópicos e actions")
    Rel(launch, ros, "ros2 launch")
    Rel(dbm, db, "SQL")
```

**Fundamentação.** O nível 2 mostra a decisão central da arquitetura: o
**tempo real fica nos microcontroladores** (malha de velocidade de cada roda
com FOC, watchdog local) e a **autonomia fica no ROS 2** (SLAM, planejamento),
ligados por um protocolo de texto simples. Os componentes do driver separam a
lógica pura (`protocol`, testável sem ROS nem hardware) da E/S (`SerialLink`) e
da integração com o ROS (`HardwareNode`).

---

## 4. Diagrama de definição de blocos (SysML BDD)

```mermaid
classDiagram
    direction TB

    class Robo {
        <<block>>
        tracao: diferencial
        raio_roda: 0,0825 m
        bitola: 0,310 m
        v_max: 1,0 m/s
        w_max: 1,5 rad/s
    }
    class Computador {
        <<block>>
        modelo: Jetson Orin Nano Super
        so: Ubuntu 24.04 + ROS 2 Jazzy
    }
    class ControladorMotores {
        <<block>>
        mcu: ESP32
        biblioteca: SimpleFOC
        controle: velocidade em malha fechada
        telemetria: 50 Hz
        watchdog: 500 ms
    }
    class MotorBLDC {
        <<block>>
        origem: hoverboard
        pares_de_polos: 15
        tensao: 24 V
    }
    class SensorHall {
        <<block>>
        sinais: 3
    }
    class PonteTrifasica {
        <<block>>
        entrada: 3 PWM
    }
    class ModuloIMU {
        <<block>>
        mcu: STM32F401 Black Pill
        sensor: BNO085 (I2C 0x4A)
        taxa: 50 Hz
    }
    class LiDAR {
        <<block>>
        modelo: Slamtec RPLidar
        tipo: 2D 360 graus
    }
    class CameraRGBD {
        <<block>>
        modelo: Orbbec Astra
    }
    class ConversorUSBSerial {
        <<block>>
    }

    Robo *-- "1" Computador
    Robo *-- "1" ControladorMotores
    Robo *-- "2" MotorBLDC
    Robo *-- "1" ModuloIMU
    Robo *-- "1" LiDAR
    Robo *-- "1" CameraRGBD
    Robo *-- "1" ConversorUSBSerial
    ControladorMotores *-- "2" PonteTrifasica
    MotorBLDC *-- "1" SensorHall
```

**Descrição.** O robô é composto por um computador de alto nível e por dois
subsistemas de tempo real com microcontrolador próprio. Cada motor tem sua
ponte trifásica, comandada por 3 sinais PWM, e seus 3 sensores Hall, lidos pelo
mesmo ESP32. Os valores (raio, bitola, limites, pares de polos) são os de
`bringup/config/hardware.yaml`, do URDF e do firmware.

---

## 5. Diagrama de blocos internos (SysML IBD)

```mermaid
flowchart LR
    subgraph ROBO["ibd Robô"]
        subgraph JET["Computador: Jetson Orin Nano"]
            HWN["hardware_node"]
            DRV_L["sllidar_node"]
            DRV_C["astra_camera_node"]
        end

        subgraph ESP["ControladorMotores: ESP32"]
            U2["UART2<br/>GPIO16 RX / GPIO17 TX"]
            PWMR["PWM dir.<br/>GPIO27/26/25"]
            PWML["PWM esq.<br/>GPIO23/19/18"]
            HALR["Hall dir.<br/>GPIO13/14/15"]
            HALL["Hall esq.<br/>GPIO32/33/34"]
        end

        subgraph STM["ModuloIMU: STM32F401"]
            I2C["I2C1<br/>SCL PB6 / SDA PB7"]
            CDC["USB CDC"]
        end

        USBSER["ConversorUSBSerial"]
        BNO["BNO085<br/>end. 0x4A"]
        PR["Ponte trifásica dir."]
        PL["Ponte trifásica esq."]
        MR["Motor BLDC dir."]
        ML["Motor BLDC esq."]
        LID["RPLidar"]
        CAM["Orbbec Astra"]
        PWR["Alimentação 24 V"]
    end

    HWN <-->|"/dev/ttyMOTOR<br/>115200 8N1"| USBSER
    USBSER <-->|"TTL serial"| U2
    HWN <-->|"/dev/ttyIMU<br/>115200"| CDC
    I2C <-->|"I2C"| BNO
    PWMR -->|"3 PWM"| PR -->|"3 fases"| MR
    PWML -->|"3 PWM"| PL -->|"3 fases"| ML
    MR -->|"3 sinais Hall"| HALR
    ML -->|"3 sinais Hall"| HALL
    PWR --> PR & PL
    DRV_L <-->|"/dev/ttyLIDAR (USB)"| LID
    DRV_C <-->|"USB"| CAM
```

**Descrição.** Mostra as portas e os conectores reais: pinos do ESP32
(`hardware/firmware/esp32_motor_controller`), barramento I2C do STM32
(`hardware/firmware/stm32_imu`) e os nomes fixos de dispositivo criados pelas
regras udev (`bringup/udev/99-robot.rules`). O ESP32 fala com o Jetson pela
UART2 através de um conversor USB-serial, o que deixa a USB nativa livre para
depuração; o STM32 usa a USB CDC nativa.

---

## 6. Malhas de controle

```mermaid
flowchart LR
    REF(["Destino<br/>(x, y, θ)"]) --> PLAN["Planejador global<br/>NavFn (A*) · 1 Hz"]
    MAP[("Mapa + costmap")] --> PLAN
    PLAN -->|"caminho"| CTRL["Controlador local<br/>Regulated Pure Pursuit"]
    CTRL -->|"/cmd_vel_nav"| SMO["velocity_smoother"]
    SMO -->|"/cmd_vel (v, ω)"| IK["Cinemática inversa<br/>+ limites + piso de velocidade"]
    IK -->|"C ωL ωR<br/>serial"| PID["Controle de velocidade<br/>SimpleFOC (por roda)"]
    PID -->|"tensões trifásicas"| MOT["Motores BLDC"]
    MOT -->|"Hall"| PID
    MOT -->|"M ωL ωR θL θR · 50 Hz"| FK["Cinemática direta"]
    GYR["Giroscópio BNO085<br/>I ... · 50 Hz"] -->|"ω_z"| ODO
    FK -->|"v"| ODO["Odometria<br/>integração da pose · 25 Hz"]
    ODO -->|"/odom, TF odom→base_link"| SLAM["RTAB-Map<br/>ICP + fechamento de loop"]
    LID["LiDAR /scan"] --> SLAM
    LID --> CTRL
    SLAM -->|"TF map→odom"| CTRL
    SLAM --> MAP
```

**Descrição.** Três malhas em cascata, cada uma mais rápida que a de fora:

1. **Malha interna (ESP32):** controle de velocidade de cada roda com FOC e
   realimentação dos sensores Hall; roda no microcontrolador e não depende do
   computador.
2. **Malha intermediária (hardware_node):** converte (v, ω) em velocidade das
   rodas, aplica limites e o piso de velocidade (abaixo de 1,5 rad/s o BLDC com
   Hall vibra em vez de girar), e devolve a odometria. A velocidade linear vem
   dos encoders e a angular do **giroscópio**, que não sofre com o
   escorregamento das rodas nas curvas.
3. **Malha externa (Nav2 + RTAB-Map):** segue o caminho com Regulated Pure
   Pursuit sobre a pose corrigida pelo SLAM (TF `map→odom`), replanejando a
   1 Hz.

---

## 7. Máquinas de estado da exploração autônoma

### 7.1 Seguir parede + BFS (`wall_bfs_explorer.py`)

```mermaid
stateDiagram-v2
    [*] --> WAIT
    WAIT --> WALL: mapa, scan, TF e Nav2 prontos
    WALL --> BFS_GOAL: revisitou caminho · parado · tempo esgotado · mapa não cresceu
    WALL --> WALL: BFS sem fronteira (menos de N vezes)
    BFS_GOAL --> WALL: chegou à fronteira
    BFS_GOAL --> BFS_GOAL: falhou → fronteira na lista negra, nova BFS
    WALL --> RETURNING: BFS sem fronteira N vezes seguidas
    BFS_GOAL --> DONE: destino cancelado por outro cliente
    RETURNING --> DONE: chegou ou falhou
    DONE --> [*]
```

### 7.2 Desbaste em offsets (`pocket_explorer.py`)

```mermaid
stateDiagram-v2
    [*] --> WAIT
    WAIT --> CANTO: pronto
    state CANTO {
        [*] --> procurar
        procurar --> ir_ao_canto: canto externo encontrado no mapa
        procurar --> revelar: nenhum canto (BFS a uma fronteira)
        revelar --> procurar: chegou
        ir_ao_canto --> procurar: falhou (canto descartado)
    }
    CANTO --> CONTORNO: no canto, ou desistiu de procurar
    CONTORNO --> CANTO: volta fechada em torno de uma ilha
    CONTORNO --> PASS: volta externa fechada ou trecho encerrado
    PASS --> PASS: passe concluído → próximo offset
    PASS --> WAIT_POSE: sem pose
    WAIT_POSE --> PASS: pose de volta
    PASS --> ACABAMENTO: região esgotada ou falhas seguidas
    ACABAMENTO --> PASS: fronteira revelou área nova
    ACABAMENTO --> RETURNING: nenhuma fronteira
    RETURNING --> DONE
    DONE --> [*]
```

**Descrição.** As duas estratégias combinam um comportamento **reativo**
(seguir parede, comandando `/cmd_vel` direto a partir do `/scan`) com um
**deliberativo** (BFS no mapa ou passes planejados, executados pelo Nav2). As
transições vêm de critérios medidos (distância percorrida sobre caminho já
feito, crescimento da área conhecida, tempo sem progresso), todos
parametrizados em `navigation/config/explore.yaml` e cobertos por testes em
`tests/test_*_explorer.py`.

---

## 8. Camadas de segurança

```mermaid
flowchart TB
    CMD["/cmd_vel (Nav2, exploração ou teleoperação)"] --> F{"valor finito?"}
    F -- não --> STOP["envia C 0 0"]
    F -- sim --> T{"telemetria dos motores<br/>há menos de 1 s?"}
    T -- não --> STOP
    T -- sim --> LIM["limita a 1,0 m/s e 1,5 rad/s"] --> SEND["envia C ωL ωR"]
    WD1{"watchdog do driver:<br/>sem /cmd_vel por 0,5 s?"} -- sim --> STOP
    SEND --> ESP["ESP32"]
    STOP --> ESP
    WD2{"watchdog do firmware:<br/>sem comando por 500 ms?"} -- sim --> ZERO["velocidade-alvo = 0"]
    ESP --> WD2
    SIG["Ctrl+C / SIGTERM nos nós"] --> STOP2["Twist zero + cancela destino no Nav2"]
```

**Descrição.** A parada é garantida em camadas independentes: se o computador
travar, o firmware para sozinho; se o firmware parar de responder, o driver
recusa comandos; se ninguém mandar comando, o driver para. Todas as decisões
do driver são verificadas em `tests/test_hardware_node.py`.
