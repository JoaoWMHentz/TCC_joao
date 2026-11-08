# Conformidade com o Portfólio (linha IoT) e justificativas

Este documento cruza o projeto com os requisitos do
[The Portfolio Playbook](https://github.com/CatolicaSC-Portfolio/The-Portfolio-Playbook)
— [Disciplina de Portfólio](https://github.com/CatolicaSC-Portfolio/The-Portfolio-Playbook/blob/main/Portfolio.md),
[direction de IoT](https://github.com/CatolicaSC-Portfolio/The-Portfolio-Playbook/blob/main/directions/portfolio-directions-iot.md),
[Portfólio Directions](https://github.com/CatolicaSC-Portfolio/The-Portfolio-Playbook/blob/main/directions/portfolio-directions-GERAL.md)
e [Normas](https://github.com/CatolicaSC-Portfolio/The-Portfolio-Playbook/blob/main/documentation/normas.md).
Para cada requisito que não é atendido **da forma literal prevista**,
registra o motivo técnico e o que o projeto oferece no lugar.

> Legenda: ✅ atendido · 🟡 atendido com adaptação justificada · ❌ não atendido

---

## 0. Natureza do projeto

O projeto é uma **pesquisa aplicada em controle e navegação de robôs móveis**.
O produto não é uma aplicação de software sobre infraestrutura pronta, e sim
um sistema ciberfísico completo, desenvolvido em todas as camadas:

| Camada | Desenvolvido no projeto |
|---|---|
| Mecânica e eletrônica | plataforma diferencial com motores BLDC de hoverboard, integração de LiDAR, câmera RGB-D e IMU |
| Firmware | controle de velocidade em malha fechada (FOC com sensores Hall) no ESP32; aquisição da IMU no STM32; protocolo serial próprio; watchdogs |
| Controle | cinemática, odometria com fusão de encoders e giroscópio, camadas de segurança, ajuste do controlador do Nav2 para a dinâmica da base |
| Autonomia | SLAM com RTAB-Map, planejamento com Nav2, três estratégias de exploração autônoma (uma original, inspirada no *pocketing* de CNC) |
| Simulação | réplica do robô e de dois ambientes comerciais no Gazebo, com pessoas em movimento |
| Aplicação | interface web, backend e banco de dados (complementar: todas as funções também estão disponíveis pela linha de comando do ROS 2) |

O hardware e o firmware vêm sendo desenvolvidos há mais de dois anos, e o
software de navegação há mais de um ano (ver [J11](#j11-histórico-de-commits-e-autoria)).

Os requisitos do Playbook foram escritos para cobrir todas as linhas de
projeto, a maioria delas centrada em aplicações hospedadas em nuvem. Vários
deles (deploy em nuvem, entrega contínua até o dispositivo, autenticação de usuários)
pressupõem esse tipo de produto e não se aplicam bem a um robô que opera
fisicamente num ambiente. As adaptações abaixo priorizam o objetivo da
pesquisa: **controle, segurança física e autonomia de navegação**.

---

## 1. Matriz de conformidade

### 1.1 Obrigatórios da linha IoT

| Requisito | Status | Evidência / justificativa |
|---|---|---|
| Diagrama Hierárquico de Requisitos (SysML) | ✅ | [diagramas.md §1](diagramas.md#1-diagrama-hierárquico-de-requisitos-sysml) |
| Diagrama de arquitetura (C4 ou camadas) descrito e fundamentado | ✅ | [diagramas.md §2 e §3](diagramas.md#2-arquitetura-em-camadas-iot) e [arquitetura.md](arquitetura.md) |
| Implementação coerente com a arquitetura | ✅ | pacotes `hardware`, `bringup`, `navigation`, `web`, `sim` seguem as camadas |
| Código-fonte e artefatos de especificação no repositório | ✅ | este repositório; firmwares em `hardware/firmware` |
| Internet como meio de comunicação | 🟡 | [J1](#j1-camada-de-aplicação-e-comunicação-em-rede) |
| Interação via web/mobile | ✅ | interface React + Flask/Socket.IO em `web/` |
| Camada de aplicação pública e estável | 🟡 | [J1](#j1-camada-de-aplicação-e-comunicação-em-rede) e [J2](#j2-disponibilidade-do-produto) |

### 1.2 Núcleo comum de engenharia

| Requisito | Status | Evidência / justificativa |
|---|---|---|
| Documentação em Wiki | ✅ | Wiki do GitHub, publicada a partir de `docs/`: funcionamento de cada componente, integração, diagramas e registro de decisões ([decisoes-arquitetura.md](decisoes-arquitetura.md)) |
| Pipeline de CI/CD | 🟡 | CI no GitHub Actions (estilo, testes com cobertura mínima, build da interface, publicação da Wiki); entrega no robô dispensada pelos professores — [J3](#j3-integração-e-entrega-contínuas) |
| Análise estática de código e segurança | ✅ | `ament_flake8` e `tsc` no CI, sem pendências — [J4](#j4-análise-estática) |
| Monitoramento / observabilidade | 🟡 | [J5](#j5-observabilidade) |

### 1.3 Desejáveis da linha IoT

| Requisito | Status | Evidência / justificativa |
|---|---|---|
| Esquemático de hardware (SysML BDD e IBD) | ✅ | [diagramas.md §4 e §5](diagramas.md#4-diagrama-de-definição-de-blocos-sysml-bdd) |
| Protocolos aderentes ao escopo (não só HTTP) | ✅ | serial ASCII com os microcontroladores, DDS entre os nós ROS, WebSocket na interface ([diagramas.md §2](diagramas.md#2-arquitetura-em-camadas-iot)) |
| Testes automatizados da aplicação e do processamento de dados | ✅ | `tests/`: protocolo, cinemática, driver, enlace serial e as três estratégias de exploração, com cobertura mínima de 80% verificada por `tests/run_tests.sh` |
| Testes das rotinas críticas de firmware | 🟡 | [J7](#j7-testes-de-firmware) |

### 1.4 Diferenciais da linha IoT

| Diferencial | Situação |
|---|---|
| Aplicação web ou mobile sobre a solução IoT | ✅ interface web de operação |
| Análise de dados / dashboards | ✅ métricas de navegação e de localização (`tools/metrics`, histórico na interface) |
| Prática de cibersegurança | 🟡 isolamento em rede local ([J13](#j13-autenticação-na-interface)) |
| Integração com sistema externo | — |

### 1.5 "Deve ser evitado" e "não pode ter"

| Item | Situação |
|---|---|
| Plataformas IoT com camadas prontas | não se aplica — [J8](#j8-ros-2-rtab-map-e-nav2-não-são-plataforma-iot-pronta) |
| Simular comportamento com LEDs | não se aplica — atuadores reais |
| Apenas simulação | não se aplica — o mesmo software roda no robô físico; a simulação é homologação ([J6](#j6-ambientes-de-desenvolvimento-homologação-e-produção)) |
| Escopo simplista | não se aplica — SLAM, planejamento, controle e exploração autônoma |
| Escopo de projeto pronto | não se aplica — [J8](#j8-ros-2-rtab-map-e-nav2-não-são-plataforma-iot-pronta) |
| Apenas embarcado | não se aplica — há camada de aplicação web e banco de dados |

### 1.6 Guia do Portfólio: estrutura, documentação, DevEx e Definição de Pronto

Itens do texto-guia da disciplina (o "guia/bússola" que complementa o
manifesto).

**Estrutura do repositório**

| Item | Status | Evidência / justificativa |
|---|---|---|
| `README.md` | ✅ | propósito, arquitetura, instalação, uso (interface e linha de comando), avaliação, testes |
| `CONTRIBUTING.md` | ✅ | [CONTRIBUTING.md](../CONTRIBUTING.md): ambiente, guia de estilo, testes, issues com etiquetas, pull requests |
| `LICENSE` | ✅ | Apache-2.0 |
| `CODE_OF_CONDUCT.md` | ✅ | [CODE_OF_CONDUCT.md](../CODE_OF_CONDUCT.md), adaptado do Contributor Covenant 2.1 |

**Documentação**

| Item | Status | Evidência / justificativa |
|---|---|---|
| Arquitetura (C4 recomendado) | ✅ | C4 de contexto, contêineres e componentes em [diagramas.md](diagramas.md); camadas em [arquitetura.md](arquitetura.md) |
| Wiki com registro de decisões e guias | ✅ | Wiki gerada de `docs/` ([publish_wiki.sh](../scripts/publish_wiki.sh)); [ADRs](decisoes-arquitetura.md); guias de cada componente |
| Documentação de API (Pydoc, Swagger etc.) | ✅ | [api.md](api.md): REST, eventos Socket.IO e equivalentes ROS 2; docstrings em todos os módulos Python (padrão PEP 257, verificado pelo linter) |
| UX: designs de interface e jornada do usuário | ✅ | [ux.md](ux.md): perfis, jornadas, fluxo de telas, layout, decisões de interação. Capturas de tela: [informar após capturar, em `docs/img/`] |

**Desenvolvimento e experiência do desenvolvedor**

| Item | Status | Evidência / justificativa |
|---|---|---|
| Guia de estilo com verificação automática | ✅ | guia de estilo do ROS 2 (PEP 8, 99 colunas, PEP 257), verificado pelo `ament_flake8`, o equivalente do Checkstyle no ecossistema ROS 2, e `tsc` na interface: `scripts/lint.sh` |
| Testes automatizados | ✅ | `pytest`, 189 testes, ver [J9](#j9-cobertura-de-testes-e-tdd) |
| Contêineres e mocks | ✅ | PostgreSQL em Docker; simulação Gazebo como ambiente completo; `tools/fake_firmware.py` emula os microcontroladores; dublês de Nav2/TF/serial nos testes |
| CI/CD | 🟡 | CI no GitHub Actions; entrega no robô privada — [J3](#j3-integração-e-entrega-contínuas) |
| Issues e pull requests com etiquetas e processo documentado | ✅ | modelos em `.github/ISSUE_TEMPLATE` e `.github/PULL_REQUEST_TEMPLATE.md`; etiquetas e fluxo em [CONTRIBUTING.md](../CONTRIBUTING.md). Revisão por pares: [J12](#j12-revisão-por-pares) |
| Análise estática | ✅ | `ament_flake8` e `tsc` no CI — [J4](#j4-análise-estática) |

**Sustentabilidade e evolução**

| Item | Status | Evidência / justificativa |
|---|---|---|
| Monitoramento e log | 🟡 | [J5](#j5-observabilidade) |
| Feedback e melhoria contínua | ✅ | modelos de issue para defeito e melhoria, com campo de como medir o resultado; métricas de navegação para avaliar cada mudança |
| Acessibilidade | 🟡 | [ux.md §5](ux.md#5-acessibilidade) e WCAG em [Normas](#3-normas-aplicáveis) |

**Definição de Pronto**

| Critério | Status | Evidência / justificativa |
|---|---|---|
| Software funcional em ambiente produtivo | 🟡 | roda no robô físico, que é o ambiente de produção ([J2](#j2-disponibilidade-do-produto), [J6](#j6-ambientes-de-desenvolvimento-homologação-e-produção)) |
| CI/CD | 🟡 | CI implementado (testes e Wiki automáticos); entrega no robô dispensada pelos professores — [J3](#j3-integração-e-entrega-contínuas) |
| Monitoramento | 🟡 | [J5](#j5-observabilidade) |
| Cobertura de testes unitários ≥ 75% | ✅ | 98% (mínimo de 80% verificado por `tests/run_tests.sh`) |
| Sem problemas significativos em análise estática | ✅ | `scripts/lint.sh` sem pendências (`ament_flake8` + `tsc`), verificado no CI |
| Documentação atualizada: modelagem, decisões, guia de contribuição, URLs, prints | 🟡 | modelagem ([diagramas.md](diagramas.md)), decisões ([ADRs](decisoes-arquitetura.md)), contribuição ([CONTRIBUTING.md](../CONTRIBUTING.md)), URLs ([api.md](api.md), README). Faltam as capturas de tela em [ux.md](ux.md#capturas-de-tela) |

---

## 2. Justificativas

### J1. Camada de aplicação e comunicação em rede

**Requisito:** camada de aplicação hospedada em ambiente público e estável;
internet como meio de comunicação.

**Justificativa.** A camada de aplicação **comanda um veículo em movimento**
entre pessoas. Expô-la na internet pública transfere o comando do robô para
fora do ambiente onde ele opera, o que contraria o princípio das normas de
segurança de robôs móveis (ver [J10](#j10-normas-de-segurança-de-máquinas-e-robôs)):
o comando deve partir de um operador presente, com o robô à vista. Além disso,
SLAM, planejamento e controle exigem latência baixa e não podem depender de um
link externo; por isso todo o processamento é embarcado.

**Como o projeto atende ao propósito do requisito.**

- A comunicação entre a aplicação e o robô é feita por rede IP (TCP/IP,
  HTTP e WebSocket via Wi-Fi ou Ethernet), com os mesmos protocolos usados na
  internet. O robô é acessado de qualquer dispositivo da rede local do
  ambiente, sem instalação.
- Entre os módulos de software, a comunicação é DDS, que funciona em rede e
  permite operar e inspecionar o robô de outra máquina (RViz, Foxglove).
- A interface web é **complementar** ao objetivo da pesquisa: todas as
  funções estão disponíveis pela linha de comando do ROS 2.

### J2. Disponibilidade do produto

**Requisito:** produto disponível de forma pública e estável até a divulgação
das notas, sem depender de computador pessoal.

**Justificativa.** O produto é um **dispositivo físico**, e não depende de
computador pessoal: todo o software roda no computador embarcado do robô. O
[Portfolio.md](https://github.com/CatolicaSC-Portfolio/The-Portfolio-Playbook/blob/main/Portfolio.md)
(pilar DevOps) admite que o software esteja acessível "na nuvem, na loja de
apps **ou em dispositivo físico**". O robô funciona a bateria e fica no
ambiente de operação, por isso não permanece ligado continuamente.

**Evidências complementares:** demonstração ao vivo no Demo Day, vídeo do robô
físico e reprodução completa em simulação com um comando
(`./scripts/start_sim.sh`), em qualquer máquina com Ubuntu 24.04.

### J3. Integração e entrega contínuas

**Requisito:** pipeline de CI/CD (ex.: GitHub Actions).

**Situação:** **CI implementado** no repositório público (GitHub Actions); a
**entrega no robô** foi dispensada pelos professores [informar quem e quando:
ex.: orientador e professor da disciplina, na orientação de DD/MM/2026] e
continua no pipeline privado da empresa.

**Integração contínua** — [.github/workflows/ci.yml](../.github/workflows/ci.yml),
a cada push e pull request:

| Job | O que faz |
|---|---|
| Estilo e testes (ROS 2 Jazzy) | contêiner `ros:jazzy-ros-base`; roda `scripts/lint.sh` (`ament_flake8`) e `tests/run_tests.sh` (189 testes, **falha abaixo de 80%** de cobertura); publica o `coverage.xml` e o resumo da cobertura na execução |
| Interface | `npm ci` e `npm run build` (verificação de tipos e build de produção) |
| Publicar a Wiki | só na `main` e com os anteriores verdes: regera a Wiki a partir de `docs/` (`scripts/publish_wiki.sh`) |

O workflow chama **os mesmos scripts usados localmente**: o que passa na máquina
do desenvolvedor passa no CI, sem configuração duplicada. Os testes não
precisam de Gazebo nem do robô, porque usam dublês de Nav2, TF e porta serial
([tests/ros_fakes.py](../tests/ros_fakes.py)).

**Entrega contínua.** A entrega automatizada do repositório público é a da
**documentação** (Wiki). A entrega do software no robô não é automatizada aqui,
por estes motivos:

- os robôs usam um **sistema operacional modificado**, uma imagem própria em
  que ROS 2, drivers, dependências e o processo de build e atualização já vêm
  configurados; esse pipeline existe, mas é **privado e pertence à empresa**
  (ver [J11](#j11-histórico-de-commits-e-autoria));
- publicar automaticamente uma versão nova num robô que pode estar em
  movimento interromperia o driver dos motores e a navegação no meio de uma
  tarefa;
- deploy por SSH/FTP, a alternativa para um dispositivo, é vedado pelo
  Directions.

**Ficaram de fora do CI, de propósito:**

- **simulação Gazebo**: lenta (10 a 20 min), dependente de GPU e com
  resultado não determinístico; os testes de integração com simulação são
  feitos antes de cada ida ao robô ([J6](#j6-ambientes-de-desenvolvimento-homologação-e-produção));
- **compilação dos firmwares**: os `.ino` dependem de bibliotecas e núcleos de
  placa instalados no Arduino IDE ([J7](#j7-testes-de-firmware)).

No destino, o restante da sequência é automatizado por scripts versionados:

| Etapa | Automação |
|---|---|
| Provisionamento | `install.sh` instala ROS 2, RTAB-Map, Nav2, Gazebo, Docker, Node.js e o ambiente do backend, e compila o workspace |
| Build e execução | `scripts/start_sim.sh` e `scripts/start_robot.sh` compilam o workspace e a interface, sobem o banco e a pilha |
| Atualização no robô | o robô baixa a versão do repositório (`git pull`) e o `start_robot.sh` recompila, por ação do operador com o robô parado |

### J4. Análise estática

**Requisito:** análise estática de código e segurança (ex.: SonarCloud).

**Situação:** análise estática **automatizada no CI** com o linter oficial do
ROS 2; o serviço externo (SonarCloud) não foi adotado.

**Justificativa.** A análise roda a cada push e pull request
([J3](#j3-integração-e-entrega-contínuas)), com as ferramentas do próprio
ecossistema:

- **`scripts/lint.sh`** roda o `ament_flake8`, o linter oficial
  do ROS 2 (PEP 8, ordem de imports, docstrings PEP 257, nomes que escondem
  funções nativas, compreensões desnecessárias), em todo o código Python, e
  termina **sem pendências**; o CI falha se surgir alguma;
- verificação de tipos do frontend com o compilador TypeScript
  (`npm run typecheck`, também chamado pelo `scripts/lint.sh`), que é análise
  estática;
- anotações de tipo no Python e separação entre lógica pura e E/S
  (`protocol.py`, `frontiers.py`, `wall_bfs.py`, `pocket.py` não dependem de
  ROS nem de hardware), o que permite testar as regras isoladamente;
- testes automatizados com cobertura medida, que já encontraram e corrigiram
  um defeito real (travamento do driver em falha de escrita serial, ver
  [J7](#j7-testes-de-firmware)).

### J5. Observabilidade

**Requisito:** monitoramento ou observabilidade da aplicação e da telemetria
dos dispositivos.

**Como é atendido**, com ferramentas próprias do domínio de robótica em vez de
um servidor de métricas genérico:

| O que é observado | Como |
|---|---|
| Saúde dos enlaces com os microcontroladores | tópico `/diagnostics` (padrão ROS), publicado a 1 Hz pelo `hardware_node`: estado de cada porta e idade da última mensagem |
| Telemetria dos dispositivos | `/odom`, `/imu/data`, `/joint_states`, `/scan`, visualizáveis ao vivo no RViz ou no Foxglove (`use_foxglove:=true`) |
| Logs dos processos | logs do ROS 2 por nó; a interface web mostra o log de cada processo da pilha (`GET /api/logs/<nome>`) |
| Desempenho da navegação | tabela `navigation_log`: resultado, duração, distância, recuperações do Nav2, menor folga até obstáculos e erro final de cada navegação |
| Precisão da localização | `tools/metrics/localization_error.py` compara a pose estimada com a pose real na simulação |

### J6. Ambientes de desenvolvimento, homologação e produção

**Como é atendido:** a simulação no Gazebo publica **os mesmos tópicos** do
robô físico (ver [arquitetura.md](arquitetura.md)) e serve de homologação:
toda versão é exercitada em simulação (mundos `supermercado` e `loja`, com
pessoas em movimento) antes de ir ao robô, que é o ambiente de produção.

### J7. Testes de firmware

**Requisito (desejável):** testes das rotinas críticas de firmware.

**Justificativa.** Os firmwares rodam em microcontroladores (ESP32 com
SimpleFOC, STM32 com BNO085) e dependem do hardware para ler sensores e
acionar motores. Uma bancada de testes com hardware no loop está fora do
escopo do projeto.

**O que é feito no lugar:** as rotinas críticas do lado do computador têm
testes automatizados. Isso inclui o protocolo serial (todos os quadros,
quadros malformados) e a cinemática. Inclui também o enlace serial (reconexão,
falha de escrita, linhas corrompidas) e o driver (watchdog do `cmd_vel`,
recusa de comando sem telemetria, diagnóstico). Um emulador dos firmwares
(`tools/fake_firmware.py`) permite testar o driver sem o robô. Os testes do
enlace encontraram um travamento real: uma falha de escrita na serial
deixava o driver bloqueado para sempre (*lock* não reentrante). O defeito foi
corrigido.

### J8. ROS 2, RTAB-Map e Nav2 não são plataforma IoT pronta

ROS 2, RTAB-Map e Nav2 são **bibliotecas e frameworks** de robótica, como um
framework web, e não plataformas IoT que entregam dispositivo, nuvem e painel
prontos. Ver na [seção 0](#0-natureza-do-projeto) o que foi desenvolvido no
projeto.

### J9. Cobertura de testes e TDD

A meta de cobertura é obrigatória só na linha de Web Apps; em IoT, TDD é
recomendado e não há meta. Mesmo assim, o código Python do robô (driver,
protocolo, enlace serial e exploração) tem cobertura medida, com mínimo de 80%
verificado automaticamente, acima dos 75% pedidos pela Definição de Pronto do
guia da disciplina (medido: 98%). Os testes exercitam regras de comportamento
(timeouts, lista negra de destinos, retorno ao ponto de partida, falhas do
Nav2), não apenas o percentual.

### J10. Normas de segurança de máquinas e robôs

Certificação segundo ISO 3691-4 (veículos industriais sem condutor),
ISO 13482 (robôs de serviço) ou NR-12 exige ensaios e organismo certificador,
o que está fora do alcance de um protótipo de pesquisa. Medidas adotadas,
inspiradas nelas ([diagramas.md §8](diagramas.md#8-camadas-de-segurança)):

- watchdog no firmware dos motores (500 ms sem comando → para);
- watchdog no driver (0,5 s sem `/cmd_vel` → para);
- recusa de comandos quando o microcontrolador não responde;
- limites de velocidade linear e angular;
- parada e cancelamento do destino ao encerrar os nós (Ctrl+C/SIGTERM);
- desvio de obstáculos dinâmicos pelo Nav2;
- operação somente com supervisão humana.

### J11. Histórico de commits e autoria

**Requisito:** histórico de commits com evolução contínua.

**Justificativa.** Este repositório é um *fork* público de um projeto privado.
O projeto original e a sua infraestrutura (histórico, pipeline de CI/CD e
imagem do sistema operacional dos robôs) pertencem à empresa. O histórico
anterior não pôde ser publicado, por isso o repositório público tem poucos
commits. O desenvolvimento é contínuo e anterior ao repositório:

- hardware e firmware: desenvolvidos do zero há mais de dois anos;
- software de navegação: em desenvolvimento há mais de um ano;
- repositórios anteriores do mesmo autor: [informar os repositórios e
  períodos que podem ser mostrados aos avaliadores].

**Uso de assistentes de IA.** Assistentes de IA foram usados no
desenvolvimento, como permitem as regras da disciplina, sobre uma base de
código, hardware e firmware construída pelo autor ao longo desse período. O
autor responde por todas as decisões e por todo o código entregue.

### J12. Revisão por pares

O projeto é individual e [informar a situação do squad de validação do
PAC VIII]. As revisões de código foram feitas pelo autor com apoio de
assistente de IA. Os defeitos encontrados viraram correções rastreáveis em
commits (ex.: a correção do enlace serial em [J7](#j7-testes-de-firmware)).

### J13. Autenticação na interface

A interface é servida pelo robô **somente na rede local** do ambiente onde
ele opera, e o operador está presente, com o robô à vista. Ela não é exposta
na internet ([J1](#j1-camada-de-aplicação-e-comunicação-em-rede)). Nesse
cenário, o controle de acesso é o da própria rede (senha do Wi-Fi), e uma
camada de login não aumentaria a segurança física, que é garantida pelas
medidas de [J10](#j10-normas-de-segurança-de-máquinas-e-robôs).

---

## 3. Normas aplicáveis

| Norma | Aplicação no projeto |
|---|---|
| **LGPD** | Projeto de pesquisa científica e tecnológica em ambiente acadêmico. A LGPD prevê o tratamento de dados para estudos por órgão de pesquisa (Art. 7º, IV), com anonimização sempre que possível. Os sensores servem à **localização do robô**, e não à identificação de pessoas: o LiDAR não capta imagem, as pessoas são tratadas apenas como obstáculos, e o que a câmera capta fica nos arquivos de mapa armazenados no próprio robô. Esses arquivos não são publicados, compartilhados nem usados para outro fim. A interface não coleta dados pessoais de usuários. |
| **Licenças de terceiros** | Projeto sob Apache-2.0. Dependências: ROS 2, Nav2, RTAB-Map, SimpleFOC, Adafruit BNO08x, React, Flask, entre outras. Modelo 3D de pessoa da simulação sob CC BY 4.0, com atribuição em [sim/models/humano/README.md](../sim/models/humano/README.md). |
| **Segurança de máquinas** (ISO 3691-4, ISO 13482, NR-12) | Ver [J10](#j10-normas-de-segurança-de-máquinas-e-robôs). |
| **WCAG 2.1** | A interface é complementar ([J1](#j1-camada-de-aplicação-e-comunicação-em-rede)) e destinada ao operador do robô. O mapa é um canvas e depende de visão. O estado do robô e os resultados das navegações também aparecem em texto. |
