# Decisões de arquitetura (ADRs)

Registro das principais decisões técnicas do projeto, no formato de
*Architecture Decision Records*: o contexto que levou à decisão, o que foi
decidido, as alternativas e as consequências (inclusive as negativas).

| # | Decisão | Área |
|---|---|---|
| [ADR-01](#adr-01-ros-2-jazzy-como-middleware) | ROS 2 Jazzy como middleware | plataforma |
| [ADR-02](#adr-02-microcontroladores-dedicados-para-o-tempo-real) | Microcontroladores dedicados para o tempo real | hardware |
| [ADR-03](#adr-03-protocolo-serial-de-texto) | Protocolo serial de texto | firmware |
| [ADR-04](#adr-04-foc-com-sensores-hall-e-piso-de-velocidade) | FOC com sensores Hall e piso de velocidade | controle |
| [ADR-05](#adr-05-velocidade-angular-da-odometria-pelo-giroscópio) | Velocidade angular da odometria pelo giroscópio | controle |
| [ADR-06](#adr-06-rtab-map-para-slam) | RTAB-Map para SLAM | autonomia |
| [ADR-07](#adr-07-mapa-2d-só-do-lidar-câmera-para-laços-e-obstáculos) | Mapa 2D só do LiDAR; câmera para laços e obstáculos | autonomia |
| [ADR-08](#adr-08-nav2-com-navfn-e-regulated-pure-pursuit) | Nav2 com NavFn e Regulated Pure Pursuit | autonomia |
| [ADR-09](#adr-09-exploração-própria-em-três-estratégias) | Exploração própria, em três estratégias | autonomia |
| [ADR-10](#adr-10-mesmos-tópicos-na-simulação-e-no-robô) | Mesmos tópicos na simulação e no robô | arquitetura |
| [ADR-11](#adr-11-lógica-pura-separada-do-ros) | Lógica pura separada do ROS | qualidade |
| [ADR-12](#adr-12-backend-como-supervisor-de-processos) | Backend como supervisor de processos | aplicação |
| [ADR-13](#adr-13-interface-web-servida-pelo-robô-na-rede-local) | Interface web servida pelo robô, na rede local | aplicação |
| [ADR-14](#adr-14-postgresql-em-docker-para-os-dados-da-interface) | PostgreSQL em Docker para os dados da interface | aplicação |

---

## ADR-01. ROS 2 Jazzy como middleware

**Contexto.** O robô integra LiDAR, câmera RGB-D, IMU, dois motores, SLAM,
planejamento e uma interface. Escrever a comunicação, a geometria (frames) e
os algoritmos do zero inviabilizaria o foco em controle e navegação.

**Decisão.** ROS 2 Jazzy (LTS até 2029) sobre Ubuntu 24.04.

**Alternativas.** ROS 1 Noetic (fim de suporte em 2025); pilha própria sobre
ZeroMQ/MQTT (sem drivers, SLAM nem planejamento prontos).

**Consequências.** Acesso direto a RTAB-Map, Nav2 e aos drivers dos sensores;
comunicação tipada com QoS; TF2 para a geometria; ferramentas de depuração
(RViz, Foxglove, `ros2 topic`). Em troca, dependência de uma distribuição
específica e do Ubuntu 24.04 no robô e no desenvolvimento.

## ADR-02. Microcontroladores dedicados para o tempo real

**Contexto.** O controle FOC dos motores exige um laço rápido e de período
regular, e a parada de emergência não pode depender do Linux do computador
embarcado.

**Decisão.** ESP32 para os motores e STM32 para a IMU, cada um com firmware
próprio; o Jetson só envia velocidades-alvo e recebe telemetria.

**Alternativas.** Controlar os motores pelos GPIO do Jetson (sem tempo real
garantido); controladora de motor comercial com protocolo fechado.

**Consequências.** O laço de controle fica imune à carga do Jetson (SLAM e
planejamento); o watchdog do firmware para os motores mesmo com o computador
travado. Em troca, há firmware a manter e um protocolo a definir (ADR-03).

## ADR-03. Protocolo serial de texto

**Contexto.** Era preciso um protocolo entre o driver e os microcontroladores.

**Decisão.** Texto ASCII, uma mensagem por linha, 115200 bps (`C`, `M`, `I`,
`#`), com parsing em funções puras testadas.

**Alternativas.** Protocolo binário com checksum; micro-ROS nos
microcontroladores.

**Consequências.** Depuração direta num terminal serial; firmware simples;
mensagens corrompidas são descartadas pelo parser (número errado de campos).
A banda usada é pequena (50 mensagens de ~40 bytes por segundo). Em troca, não
há checksum: uma corrupção que mantenha o formato passaria. O risco é mitigado
pelos limites de velocidade no driver e pelos watchdogs.

## ADR-04. FOC com sensores Hall e piso de velocidade

**Contexto.** Os motores de hoverboard têm torque alto e são baratos, mas
vêm só com sensores Hall (6 estados por ciclo elétrico), que dão pouca
resolução de ângulo.

**Decisão.** Controle FOC de velocidade com SimpleFOC no ESP32 e um **piso de
velocidade por roda** no driver: abaixo de 0,15 rad/s a roda para; entre 0,15
e 1,5 rad/s sobe para 1,5 rad/s. As velocidades mínimas do Nav2 (0,17 m/s) e
do seguidor de parede (0,12 m/s) foram ajustadas a esse piso.

**Alternativas.** Encoder de alta resolução nos motores (custo e mecânica);
aceitar a vibração a baixa velocidade.

**Consequências.** Movimento suave dentro da faixa útil; o robô não faz
movimentos muito lentos (as aproximações finais são mais "secas").

## ADR-05. Velocidade angular da odometria pelo giroscópio

**Contexto.** Em giros e curvas as rodas escorregam, e o ângulo é o erro de
odometria mais caro: um grau errado vira metros depois de uma reta.

**Decisão.** Na odometria, a velocidade linear vem das rodas e a angular do
giroscópio da BNO085 (quando saudável), integradas pelo método do ponto médio.

**Alternativas.** Só rodas; filtro de Kalman estendido (`robot_localization`)
fundindo rodas e IMU.

**Consequências.** Odometria angular muito melhor com uma implementação
simples e testável. O EKF seria mais geral, mas acrescentaria um nó e muitos
parâmetros para um ganho pequeno, já que o RTAB-Map corrige a deriva restante.

## ADR-06. RTAB-Map para SLAM

**Contexto.** O ambiente comercial é grande, repetitivo (corredores parecidos)
e o robô precisa mapear uma vez e depois se localizar muitas vezes.

**Decisão.** RTAB-Map, em dois modos (mapeamento e localização) sobre o mesmo
`.db`.

**Alternativas.** `slam_toolbox` e Cartographer, focados em LiDAR 2D.

**Consequências.** Fusão de LiDAR, câmera, IMU e odometria num só pacote;
fechamento de loop por **aparência**, que reconhece um lugar mesmo com muita
deriva, e relocalização "do zero" em qualquer ponto do mapa; mapa, grafo e
vocabulário num único arquivo por ambiente. Em troca, muitos parâmetros e
maior consumo de CPU por causa da parte visual.

## ADR-07. Mapa 2D só do LiDAR; câmera para laços e obstáculos

**Contexto.** A câmera RGB-D tem campo de visão estreito e alcance útil curto,
e vê objetos móveis (carrinhos, pessoas).

**Decisão.** O grid 2D do RTAB-Map é construído só pelo LiDAR
(`Grid/Sensor: 0`). A câmera serve a duas coisas: fechamento de loop por
aparência no RTAB-Map e obstáculos fora do plano do laser nos costmaps do Nav2
(tempo real, sem ficar no mapa).

**Consequências.** Mapa permanente limpo e estável; mesas e prateleiras acima
do laser ainda são evitadas na navegação.

## ADR-08. Nav2 com NavFn e Regulated Pure Pursuit

**Contexto.** Robô diferencial em ambiente com pessoas, com velocidade mínima
imposta pelo piso das rodas (ADR-04).

**Decisão.** Planejador NavFn (A*) com replanejamento a 1 Hz; controlador
Regulated Pure Pursuit com *lookahead* proporcional à velocidade, desaceleração
em curvas, detecção de colisão e giro no lugar antes de andar.

**Alternativas.** Controladores DWB ou MPPI, que amostram trajetórias.

**Consequências.** Seguimento de caminho suave e previsível, com poucos
parâmetros e todos com significado físico (raio de curva, tempo até colisão).
O RPP não desvia sozinho de obstáculos no caminho: o desvio vem do
replanejamento global a 1 Hz, e a detecção de colisão para o robô enquanto o
caminho novo não chega.

## ADR-09. Exploração própria, em três estratégias

**Contexto.** O mapeamento manual (conduzir o robô) é lento e depende do
operador. Pacotes prontos de exploração por fronteiras (como o `explore_lite`)
escolhem destinos sem considerar a distância pelo caminho nem a cobertura
sistemática.

**Decisão.** Implementar a exploração no projeto, com três estratégias
selecionáveis: fronteiras (referência clássica), seguir parede + BFS e
desbaste em offsets (original). Detalhes em [exploracao.md](exploracao.md).

**Consequências.** Controle total dos critérios (lista negra, revisita com
sentido, estagnação, volta ao início) e possibilidade de comparar as
estratégias no mesmo ambiente simulado. Em troca, mais código a manter e a
testar (coberto em `tests/test_*_explorer.py`).

## ADR-10. Mesmos tópicos na simulação e no robô

**Contexto.** Testar no robô físico é lento e arriscado; os ambientes alvo
(lojas) não estão sempre disponíveis.

**Decisão.** A simulação (Gazebo Harmonic, `ros_gz_bridge`) publica
exatamente os mesmos tópicos e frames do robô físico; um `cmd_vel_relay`
reproduz o watchdog do driver real.

**Consequências.** RTAB-Map, Nav2, exploração e interface são idênticos nos
dois casos; a simulação funciona como homologação e permite medir o erro de
localização contra a pose real. Em troca, a física simulada não reproduz
tudo (escorregamento, piso das rodas, ruído dos sensores).

## ADR-11. Lógica pura separada do ROS

**Contexto.** Nós ROS são difíceis de testar: dependem de mensagens, TF,
relógio e de outros nós.

**Decisão.** Toda regra de negócio fica em módulos sem ROS: `protocol.py`
(protocolo e cinemática), `frontiers.py`, `wall_bfs.py`, `pocket.py`. Os nós só
convertem mensagens e chamam essas funções.

**Consequências.** As regras são testadas com `numpy` puro, rápido e sem
simulador; os nós são testados com dublês do Nav2 e do TF
(`tests/ros_fakes.py`). Os testes encontraram um defeito real de travamento no
enlace serial.

## ADR-12. Backend como supervisor de processos

**Contexto.** A interface precisa mapear, abrir mapas, explorar e trocar de
mapa sem terminal. O RTAB-Map só grava o `.db` ao encerrar com SIGINT.

**Decisão.** O backend sobe a pilha como processos `ros2 launch` separados
(`base`, `slam`, `nav`, `explore`), cada um no próprio grupo de processos, e os
encerra com SIGINT em ordem inversa.

**Alternativas.** Um único launch com tudo; serviços do systemd.

**Consequências.** O mapa é sempre salvo; partes podem ser reiniciadas
isoladamente (o Nav2 é reiniciado se falhar ao ativar); os logs de cada
processo aparecem na interface.

## ADR-13. Interface web servida pelo robô, na rede local

**Contexto.** O operador de uma loja não tem conhecimento de robótica e não
deve instalar nada. O comando de um robô que anda entre pessoas não deve sair
do ambiente.

**Decisão.** Frontend React servido pelo backend Flask no próprio robô;
atualizações em tempo real por Socket.IO (WebSocket); acesso pela rede local.

**Consequências.** Qualquer celular ou tablet na rede opera o robô pelo
navegador; mapa, pose, caminho e laser chegam em tempo real sem *polling*.
O acesso fica restrito à rede local (ver
[conformidade-portfolio.md](conformidade-portfolio.md), J1 e J13).

## ADR-14. PostgreSQL em Docker para os dados da interface

**Contexto.** Destinos, rotas e o histórico de navegações precisam persistir
entre sessões e permitir consultas agregadas (taxa de sucesso, médias).

**Decisão.** PostgreSQL 16 em container Docker (`web/docker-compose.yml`),
acessado com `psycopg2`.

**Alternativas.** SQLite (vedado pelo Portfólio Directions para dados de
produção).

**Consequências.** Consultas agregadas com `FILTER` e tipos ricos (`JSONB`
para poses, `INTEGER[]` para a sequência de uma rota); instalação reproduzível
pelo `docker compose`. Os mapas não ficam no banco: são os `.db` do RTAB-Map,
em `MAPS_DIR`.
