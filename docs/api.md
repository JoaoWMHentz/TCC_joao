# API da interface (REST e Socket.IO)

O backend ([web/backend](../web/backend/)) expõe duas interfaces na porta
**5000**:

- **REST** (`/api/...`): comandos e consultas pontuais, como iniciar uma sessão,
  enviar um destino ou ler o histórico.
- **Socket.IO**: eventos enviados pelo servidor, como pose, mapa, laser,
  andamento da navegação e diagnóstico.

Toda a funcionalidade também existe **sem a interface**, pela linha de comando
do ROS 2 (seção 4). A API é uma camada de conveniência para o operador.

As respostas são JSON. Em caso de erro, o corpo é `{"error": "<mensagem>"}`:

| Código | Quando |
|---|---|
| `400` | parâmetro ausente, não numérico ou inválido |
| `404` | mapa, destino ou rota inexistente; mapa ainda não publicado |
| `409` | conflito de estado: sessão errada, nome repetido, mapa em uso, robô sem localização |
| `503` | ROS não inicializado, Nav2 indisponível ou banco de dados fora do ar |

---

## 1. Sessão

| Método | Rota | Corpo | Descrição |
|---|---|---|---|
| `GET` | `/api/status` | — | estado geral (ver abaixo) |
| `GET` | `/api/maps` | — | mapas salvos (`.db` do RTAB-Map) |
| `DELETE` | `/api/maps/<nome>` | — | exclui o mapa e seus destinos e rotas. `409` se estiver em uso |
| `POST` | `/api/session/mapping` | `{name, explore?, strategy?}` | cria um mapa novo. Com `explore: true`, já começa explorando sozinho |
| `POST` | `/api/session/navigation` | `{name}` | abre um mapa salvo, em modo localização |
| `POST` | `/api/session/stop` | — | cancela a navegação e encerra a sessão. No mapeamento, salva o mapa |
| `GET` | `/api/logs/<processo>` | — | últimas linhas de `base`, `slam`, `nav` ou `explore` |

Nome de mapa: letras, números, `-` e `_`.

Exemplo de `GET /api/status`:

```json
{
  "mode": "navigation",
  "map": "loja",
  "sim": true,
  "error": null,
  "processes": {"base": true, "slam": true, "nav": true, "explore": false},
  "explore": {"running": false, "message": null, "strategy": null},
  "ros_ok": true,
  "mission": null,
  "hardware": {"hardware/motors": {"level": 0, "message": "..."}}
}
```

`mode` pode ser `idle`, `mapping` ou `navigation`.

## 2. Exploração autônoma

Disponível somente no modo `mapping`.

| Método | Rota | Corpo | Descrição |
|---|---|---|---|
| `POST` | `/api/explore/start` | `{strategy?}` | `wall_bfs` (padrão), `pocket` ou `frontier` ([exploracao.md](exploracao.md)) |
| `POST` | `/api/explore/stop` | — | para o explorador; o robô para e o mapeamento continua |
| `GET` | `/api/explore/log` | — | decisões do explorador nesta sessão: `[{t, level, text}]`, da mais antiga para a mais nova (até 100) |

Qualquer comando manual (destino, rota, teleoperação, cancelar) **encerra a
exploração automática**, para que os dois não disputem o Nav2.

## 3. Mapa e controle

| Método | Rota | Corpo | Descrição |
|---|---|---|---|
| `GET` | `/api/map` | — | mapa de ocupação como PNG em base64, mais `width`, `height`, `resolution`, `origin_x`, `origin_y` |
| `POST` | `/api/goal` | `{x, y, theta?}` | navega até um ponto do mapa (metros e radianos, referencial `map`) |
| `POST` | `/api/cancel` | — | cancela a navegação em andamento |
| `POST` | `/api/initial_pose` | `{x, y, theta?}` | informa onde o robô está (corrige a localização) |
| `POST` | `/api/teleop` | `{linear, angular}` | velocidade manual, limitada a ±0,8 m/s e ±1,0 rad/s. A interface reenvia a 5 Hz; se os comandos param de chegar, o driver para o robô |

## 4. Destinos e rotas

Exigem um mapa aberto em modo `navigation`. Destinos e rotas pertencem ao mapa
em que foram criados.

| Método | Rota | Corpo | Descrição |
|---|---|---|---|
| `GET` | `/api/points` | — | destinos do mapa aberto |
| `POST` | `/api/points` | `{name, x, y, theta?}` ou `{name, here: true}` | cria um destino; com `here`, usa a pose atual do robô. Responde `201` com `{id}` |
| `DELETE` | `/api/points/<id>` | — | exclui o destino |
| `POST` | `/api/points/<id>/go` | — | navega até o destino |
| `GET` | `/api/routes` | — | rotas do mapa aberto |
| `POST` | `/api/routes` | `{name, point_ids: [id, ...]}` | cria uma rota com dois ou mais destinos, na ordem. Responde `201` com `{id}` |
| `DELETE` | `/api/routes/<id>` | — | exclui a rota |
| `POST` | `/api/routes/<id>/run` | — | percorre a rota (um `NavigateToPose` por destino) |

## 5. Histórico

| Método | Rota | Descrição |
|---|---|---|
| `GET` | `/api/history?limit=50` | últimas navegações (`navigation_log`) |
| `GET` | `/api/history/summary` | totais: navegações, sucessos, duração média, distância, recuperações, menor folga, erro final médio |

Cada linha do histórico tem: tipo (`goal`, `point`, `route`), destino,
resultado (`succeeded`, `aborted`, `canceled`, `rejected`), duração, distância
percorrida, recuperações do Nav2, menor folga entre o robô e um obstáculo e erro
final até o destino.

---

## 6. Eventos Socket.IO (servidor → cliente)

| Evento | Frequência | Conteúdo |
|---|---|---|
| `status_update` | 1 Hz e ao conectar | o mesmo de `GET /api/status` |
| `pose_update` | 5 Hz | `{x, y, theta, available}`: pose em `map`. `available: false` quando o robô não está localizado |
| `map_update` | quando o mapa muda (verificação a cada 1 s) | metadados do mapa; o cliente busca a imagem com `GET /api/map` |
| `scan_update` | 2 Hz | `{points: [[x, y], ...]}`: até 360 pontos do LiDAR, já transformados para `map` |
| `path_update` | 2 Hz | `{points: [[x, y], ...]}`: caminho planejado pelo Nav2 |
| `goal_status` | a cada feedback | `{kind, label, index, total, target, status, distance_remaining | text}` |
| `hardware_update` | a cada `/diagnostics` | estado dos motores e da IMU (`level` 0 = OK, 1 = alerta, 2 = erro) |
| `history_update` | ao fim de cada navegação | sinal para a interface recarregar o histórico |

Os eventos de alta frequência (`pose`, `scan`, `path`, `map`, `hardware`) usam
uma fila que **mantém só o último valor**: se o cliente estiver lento, recebe
o estado mais recente em vez de um acúmulo de eventos antigos. Os demais são
entregues em ordem.

---

## 7. Equivalentes pela linha de comando

| Interface | ROS 2 |
|---|---|
| Mapear | `ros2 launch navigation slam.launch.py mode:=mapping db_path:=~/tcc_maps/loja.db` e `ros2 launch navigation nav2.launch.py allow_unknown:=true` |
| Abrir mapa | `ros2 launch navigation slam.launch.py mode:=localization db_path:=...` e `nav2.launch.py` |
| Explorar | `ros2 launch navigation explore.launch.py strategy:=pocket` |
| Ir para | `ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose "{pose: {header: {frame_id: map}, pose: {position: {x: 2.0, y: 1.0}}}}"` |
| Corrigir posição | `ros2 topic pub --once /initialpose geometry_msgs/msg/PoseWithCovarianceStamped ...` |
| Teleoperação | `ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.2}}"` (ou `teleop_twist_keyboard`) |
| Diagnóstico | `ros2 topic echo /diagnostics` |
