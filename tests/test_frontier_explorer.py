"""Testes do nó de exploração por fronteiras (navigation/scripts/frontier_explorer.py)."""

import math

import numpy as np
import pytest

pytest.importorskip('rclpy')

from action_msgs.msg import GoalStatus  # noqa: E402, I100
import frontier_explorer as fe  # noqa: E402
from ros_fakes import FakeActionClient, FakeFuture, make_map, room, set_tf  # noqa: E402

OK, ABORTED, CANCELED = (GoalStatus.STATUS_SUCCEEDED, GoalStatus.STATUS_ABORTED,
                         GoalStatus.STATUS_CANCELED)


def _half_known():
    """Sala 3 x 3 m com a metade direita ainda desconhecida."""
    grid = room(60)
    grid[:, 30:] = -1
    return grid


@pytest.fixture
def node(ros):
    n = fe.FrontierExplorer()
    n.nav = FakeActionClient()
    yield n
    n.destroy_node()


def _ready(node, grid=None, pose=(0.5, 1.5, 0.0)):
    node._on_map(make_map(_half_known() if grid is None else grid))
    node._on_costmap(make_map(np.zeros((60, 60), dtype=np.int16)))
    set_tf(node, *pose)


def test_yaw_to_quaternion():
    x, y, z, w = fe.yaw_to_quaternion(math.pi)
    assert (x, y) == (0.0, 0.0) and z == pytest.approx(1.0) and w == pytest.approx(0.0, abs=1e-12)


def test_pose_pelo_tf(node):
    assert node._robot_pose() is None
    set_tf(node, 1.0, 2.0, 0.5)
    x, y, yaw = node._robot_pose()
    assert (x, y) == (1.0, 2.0) and yaw == pytest.approx(0.5)


def test_custo_do_costmap(node):
    assert node._cost_at(0.0, 0.0) == -1                 # sem costmap
    grid = np.zeros((60, 60), dtype=np.int16)
    grid[10, 20] = 77
    node._on_costmap(make_map(grid))
    assert node._cost_at(20 * 0.05 + 0.01, 10 * 0.05 + 0.01) == 77
    assert node._cost_at(-1.0, 0.0) == -1                # fora do mapa


def test_tick_espera_mapa_pose_e_nav2(node):
    node._tick()                                          # sem mapa nem pose
    _ready(node)
    node.nav.ready = False
    node._tick()
    assert node.nav.sent == []
    node.finished = True
    node.nav.ready = True
    node._tick()
    assert node.nav.sent == []


def test_tick_manda_para_a_fronteira(node):
    _ready(node)
    node._tick()
    assert node.start_pose is not None and node.sending
    gx, gy = node.goal
    assert gx > 1.0                                       # na borda com o desconhecido
    goal = node.nav.last.pose
    assert goal.header.frame_id == 'map' and goal.pose.position.x == pytest.approx(gx)
    node._tick()                                          # aguardando o Nav2 aceitar: nada
    assert len(node.nav.sent) == 1
    handle = node.nav.accept()
    assert not node.sending and node.goal_handle is handle


def test_chegou_na_fronteira(node):
    _ready(node)
    node._tick()
    handle = node.nav.accept()
    goal = node.goal
    handle.finish(OK)
    assert node.goals_reached == 1 and goal in node.blacklist and node.goal is None


def test_nav2_nao_chegou(node):
    _ready(node)
    node._tick()
    goal = node.goal
    node.nav.accept().finish(ABORTED)
    assert goal in node.blacklist and not node.finished


def test_recusado_pelo_nav2(node):
    _ready(node)
    node._tick()
    goal = node.goal
    node.nav.reject()
    assert goal in node.blacklist and node.goal is None


def test_cancelado_por_outro_cliente_encerra(node):
    _ready(node)
    node._tick()
    node.nav.accept().finish(CANCELED)
    assert node.finished


def test_resultado_antigo_e_ignorado(node):
    _ready(node)
    node._tick()
    node.nav.accept()
    node._on_result(object(), OK)
    assert node.goal is not None and node.goals_reached == 0
    node.goal = None
    node._on_result(node.goal_handle, OK)                 # sem destino: nada a fazer
    assert node.goals_reached == 0


def test_sem_progresso_cancela_e_troca(node):
    _ready(node)
    node._tick()
    handle = node.nav.accept()
    goal = node.goal
    node._tick()                                          # robô parado, mas ainda no prazo
    assert not handle.canceled
    x, y, _ = node.last_progress
    node.last_progress = (x, y, 0.0)
    node._tick()
    assert handle.canceled and node.preempting and goal in node.blacklist
    node._tick()                                          # cancelamento em andamento
    assert len(node.nav.sent) == 1
    handle.finish(CANCELED)                               # cancelado por nós: segue explorando
    assert not node.finished and node.goal is None


def test_destino_estourou_o_tempo(node):
    _ready(node)
    node._tick()
    handle = node.nav.accept()
    node.goal_sent_at -= node.goal_timeout + 1
    set_tf(node, 0.9, 1.5)                                # andou: progresso ok
    node._tick()
    assert handle.canceled and node.goal in node.blacklist


def test_fronteira_revelada_troca_de_destino(node):
    _ready(node)
    node._tick()
    handle = node.nav.accept()
    node._on_map(make_map(room(60)))                      # tudo conhecido
    node._tick()
    assert handle.canceled and node.preempting


def test_cancelar_sem_handle(node):
    node.goal = (1.0, 1.0)
    node._cancel_goal()
    assert node.goal is None and node.preempting


def test_lista_negra_e_destinos_proximos(node):
    _ready(node)
    frontiers, res = node._frontiers()
    assert node._choose(frontiers, res, 0.5, 1.5) is not None
    node.min_goal_distance = 10.0                         # tudo perto demais
    assert node._choose(frontiers, res, 0.5, 1.5) is None
    node.min_goal_distance = 0.4
    node._add_blacklist(*node._choose(frontiers, res, 0.5, 1.5)[1:], 'teste')
    assert node._blacklisted(*node.blacklist[0])


def test_sem_fronteiras_volta_ao_inicio(node):
    _ready(node, grid=room(60))
    for _ in range(fe.EMPTY_ROUNDS_TO_FINISH):
        node._tick()
    assert node.returning and node.goal == pytest.approx((0.5, 1.5))
    node._tick()                                          # voltando: o ciclo não faz nada
    assert len(node.nav.sent) == 1
    node.nav.accept().finish(OK)
    assert node.finished


def test_falha_ao_voltar_encerra(node):
    _ready(node, grid=room(60))
    for _ in range(fe.EMPTY_ROUNDS_TO_FINISH):
        node._tick()
    node.nav.reject()                     # recusa ao voltar não vai pra lista negra
    assert node.blacklist == []
    node._send_goal(0.5, 1.5, 0.0, 'de novo')
    node.nav.accept().finish(ABORTED)
    assert node.finished


def test_sem_voltar_ao_inicio(node):
    node.return_to_start = False
    _ready(node, grid=room(60))
    for _ in range(fe.EMPTY_ROUNDS_TO_FINISH):
        node._tick()
    assert node.finished and node.nav.sent == []


def test_shutdown_cancela_destino(node, monkeypatch):
    calls = []
    monkeypatch.setattr(fe.rclpy, 'spin_until_future_complete',
                        lambda n, f, timeout_sec=None: calls.append(f))
    node.shutdown()                                       # sem destino: nada
    _ready(node)
    node._tick()
    handle = node.nav.accept()
    node.shutdown()
    assert handle.canceled and len(calls) == 1


def test_main(ros, monkeypatch):
    monkeypatch.setattr(fe.rclpy, 'init', lambda **_: None)
    monkeypatch.setattr(fe.rclpy, 'try_shutdown', lambda: None)
    monkeypatch.setattr(fe.rclpy, 'ok', lambda: True)
    handlers = {}
    monkeypatch.setattr(fe.signal, 'signal', lambda sig, h: handlers.setdefault(sig, h))

    def spin_once(node, timeout_sec=None):
        handlers[fe.signal.SIGTERM]()                     # SIGTERM vira KeyboardInterrupt

    monkeypatch.setattr(fe.rclpy, 'spin_once', spin_once)
    fe.main()
    assert fe.signal.SIGTERM in handlers


def test_fake_future_ja_pronto():
    seen = []
    FakeFuture(result=1, done=True).add_done_callback(lambda f: seen.append(f.result()))
    assert seen == [1]
