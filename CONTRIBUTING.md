# Como contribuir

Obrigado pelo interesse. Este é um projeto de pesquisa (TCC) em controle e
navegação de robôs móveis, e contribuições são bem-vindas: correções, testes,
novas estratégias de exploração, ajustes de parâmetros com medições e melhorias
na documentação.

Antes de começar, leia [Como tudo funciona](docs/integracao.md) e as
[decisões de arquitetura](docs/decisoes-arquitetura.md). Elas explicam o motivo
de várias escolhas que, à primeira vista, parecem estranhas (por exemplo, o
piso de velocidade das rodas ou o RTAB-Map sem odometria visual).

Ao participar, você concorda com o [Código de Conduta](CODE_OF_CONDUCT.md).

---

## 1. Ambiente de desenvolvimento

Siga a seção **Instalação** do [README](README.md). Em resumo: Ubuntu 24.04,
ROS 2 Jazzy, Gazebo Harmonic, Docker (para o PostgreSQL) e Node.js.

Todo desenvolvimento começa na **simulação**, que publica os mesmos tópicos que
o robô físico ([ADR-02](docs/decisoes-arquitetura.md)):

```bash
./scripts/start_sim.sh                 # supermercado simulado + interface em http://localhost:5000
./scripts/start_sim.sh --people        # com pessoas caminhando
python3 tools/fake_firmware.py         # firmware falso, para testar o driver sem o robô
```

Mudanças que afetam o robô físico (driver, firmware, parâmetros de controle)
devem ser testadas também no hardware, com as rodas suspensas na primeira vez.

## 2. Guia de estilo

| Linguagem | Padrão | Verificação |
|---|---|---|
| Python | [Guia de estilo do ROS 2](https://docs.ros.org/en/jazzy/The-ROS2-Project/Contributing/Code-Style-Language-Versions.html#python): PEP 8 com linhas de até 99 colunas, imports em ordem (biblioteca padrão, terceiros, locais), docstrings PEP 257, aspas simples | `ament_flake8`, o linter oficial do ROS 2 |
| TypeScript / React | modo `strict` do TypeScript, componentes funcionais com hooks | `tsc` |
| C++ (firmwares) | estilo do Arduino/SimpleFOC, nomes descritivos, sem alocação dinâmica no laço de controle | revisão |
| Launch / YAML | um parâmetro por linha, com comentário explicando valores não óbvios | revisão |

Rode antes de abrir um pull request:

```bash
scripts/lint.sh        # ament_flake8 (Python) + tsc (interface)
tests/run_tests.sh     # testes + cobertura (mínimo de 80%)
```

**Convenções do projeto:**

- **Lógica pura separada de E/S.** Algoritmos (`protocol.py`, `frontiers.py`,
  `wall_bfs.py`, `pocket.py`) não importam ROS nem acessam hardware. O nó ROS
  só converte mensagens e chama essas funções. Isso permite testá-las sem
  simulação.
- **Comentários e documentação em português**; nomes de código em inglês.
- **Parâmetros ajustáveis vão no YAML ou como parâmetro ROS**, nunca fixos no
  código, e o comentário deve dizer de onde vem o valor (medição, limite do
  hardware, recomendação da documentação).
- **Segurança primeiro**: qualquer caminho de erro que envolva o robô em
  movimento precisa terminar com velocidade zero
  ([controle-base.md §7](docs/controle-base.md)).

## 3. Testes

- Testes em `tests/`, com `pytest`. Dublês do Nav2, do TF e do enlace serial
  estão em [tests/ros_fakes.py](tests/ros_fakes.py).
- Toda correção de defeito vem com um teste que falhava antes da correção.
- Toda função nova de lógica pura vem com testes.
- A cobertura mínima de 80% é verificada pelo `tests/run_tests.sh`.

## 4. Issues

Use os modelos em *New issue*:

- **Defeito**: o que aconteceu, o que era esperado, como reproduzir, se foi
  na simulação ou no robô, e os logs (`/api/logs/<processo>` ou o terminal).
- **Melhoria**: o problema que ela resolve e como seria medido o resultado.

Etiquetas (*labels*):

| Etiqueta | Uso |
|---|---|
| `bug` | comportamento incorreto |
| `melhoria` | funcionalidade nova ou aperfeiçoamento |
| `hardware` | driver, firmware, eletrônica |
| `navegacao` | SLAM, Nav2, exploração |
| `interface` | backend e frontend web |
| `documentacao` | docs, README, Wiki |
| `seguranca` | qualquer risco de o robô se mover de forma indevida (prioridade máxima) |

## 5. Pull requests

1. Crie um ramo a partir de `main`: `tipo/descricao-curta`
   (ex.: `fix/watchdog-serial`, `feat/exploracao-espiral`).
2. Commits no padrão [Conventional Commits](https://www.conventionalcommits.org/pt-br/):
   `feat:`, `fix:`, `docs:`, `test:`, `refactor:`, `chore:`.
3. Abra o PR preenchendo o modelo: o que mudou, por quê, como foi testado
   (simulação, robô físico ou ambos) e a issue relacionada.
4. `scripts/lint.sh` e `tests/run_tests.sh` devem passar. O CI
   ([ci.yml](.github/workflows/ci.yml)) roda os dois, mais o build da
   interface, em todo pull request; PR com CI vermelho não é integrado.
5. Mudanças de comportamento do robô precisam de evidência: um vídeo, um
   gráfico ou as métricas de `tools/metrics/`.
6. Mudanças de arquitetura atualizam a documentação em `docs/` e, se for uma
   decisão nova, ganham um ADR em
   [decisoes-arquitetura.md](docs/decisoes-arquitetura.md).

A documentação da Wiki é gerada a partir de `docs/` por
`scripts/publish_wiki.sh`. Edite sempre os arquivos do repositório, nunca a Wiki
diretamente.
