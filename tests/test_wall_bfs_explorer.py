"""Testes do nó seguir parede + BFS (navigation/scripts/wall_bfs_explorer.py)."""

import math

import pytest

pytest.importorskip('rclpy')

import wall_bfs_explorer as wbe  # noqa: E402
from action_msgs.msg import GoalStatus  # noqa: E402
from ros_fakes import FakeActionClient, make_map, make_scan, room, set_tf  # noqa: E402
from wall_bfs_explorer import BFS_GOAL, DONE, RETURNING, WAIT, WALL  # noqa: E402

OK, ABORTED, CANCELED = (GoalStatus.STATUS_SUCCEEDED, GoalStatus.STATUS_ABORTED,
                         GoalStatus.STATUS_CANCELED)


class FakePub:

    def __init__(self):
        self.msgs = []

    def publish(self, msg):
        self.msgs.append(msg)


def _half_known():
    grid = room(60)
    grid[:, 30:] = -1
    return grid


@pytest.fixture
def node(ros):
    n = wbe.WallBfsExplorer()
    n.nav = FakeActionClient()
    n.cmd_pub = FakePub()
    yield n
    n.destroy_node()


def _ready(node, grid=None, pose=(0.5, 1.5, 0.0)):
    node._on_map(make_map(_half_known() if grid is None else grid))
    set_tf(node, 0.0, 0.0, 0.0, parent='base_link', child='laser')
    node._on_scan(make_scan([1.0] * 360))
    set_tf(node, *pose)


def _no_pose(node):
    node._robot_pose = lambda: None


def test_yaw_of():
    q = type('Q', (), dict(x=0.0, y=0.0, z=math.sin(0.3), w=math.cos(0.3)))
    assert wbe.yaw_of(q) == pytest.approx(0.6)


def test_scan_sem_tf_do_lidar(node):
    node._on_scan(make_scan([1.0] * 360))
    assert node.scan is not None and node.scan_yaw is None
    set_tf(node, 0.0, 0.0, 0.2, parent='base_link', child='laser')
    node._on_scan(make_scan([1.0] * 360))
    assert node.scan_yaw == pytest.approx(0.2)


def test_espera_tudo_antes_de_comecar(node):
    node._tick()
    assert node.state == WAIT
    _ready(node)
    node.nav.ready = False
    node._tick()
    assert node.state == WAIT
    node.nav.ready = True
    node._tick()
    assert node.state == WALL and node.start_pose == pytest.approx((0.5, 1.5, 0.0))


def test_segue_parede_publica_cmd_vel(node):
    _ready(node)
    node._tick()
    node._tick()
    assert node.cmd_pub.msgs, 'deveria comandar /cmd_vel'
    assert node.wall_state != ''


def test_sem_pose_na_parede_para_o_robo(node):
    _ready(node)
    node._tick()
    _no_pose(node)
    node._tick()
    cmd = node.cmd_pub.msgs[-1]
    assert cmd.linear.x == 0.0 and cmd.angular.z == 0.0
    node.state = BFS_GOAL
    node._tick()                                          # sem pose fora da parede: só espera


def test_done_nao_faz_nada(node):
    node.state = DONE
    node._tick()
    assert node.cmd_pub.msgs == []


# ── Quando largar a parede ───────────────────────────────────────────────────

def _walking(node):
    _ready(node)
    node._tick()
    assert node.state == WALL


def test_parado_vai_para_bfs(node):
    _walking(node)
    x, y, _ = node.moved_at
    node.moved_at = (x, y, 0.0)
    node._tick()
    assert node.state == BFS_GOAL and node.goal is not None


def test_trecho_longo_vai_para_bfs(node):
    _walking(node)
    node.seg_started_at -= node.wall_max_time + 1
    node._tick()
    assert node.state == BFS_GOAL


def test_mapa_parado_vai_para_bfs(node):
    _walking(node)
    node.known_checked_at -= node.stagnation_time + 1
    node._tick()
    assert node.state == BFS_GOAL


def test_mapa_crescendo_continua_na_parede(node):
    _walking(node)
    node.known_area -= 10.0
    node.known_checked_at -= node.stagnation_time + 1
    node._tick()
    assert node.state == WALL


def test_revisita_caminho_percorrido(node):
    for i in range(121):                                  # 6 m para a direita
        node.path.add(i * 0.05, 0.0, 0.0)
    assert node._revisiting((3.0, 3.0, 0.0)) is None      # longe de tudo
    assert node._revisiting((0.5, 0.0, 0.0)) is None      # começou a revisitar
    assert node.revisit_from is not None
    for i in range(30):                                   # +1.5 m de caminho
        node.path.add(10.0 + i * 0.05, 5.0, 0.0)
    why = node._revisiting((0.6, 0.0, 0.0))
    assert why and 'já percorrido' in why and node.loop_hit is not None
    node.seg_started_at = node.known_checked_at = 1e12
    assert node._wall_exhausted((0.6, 0.0, 0.0), 0.0, 0.0) == why


# ── BFS ──────────────────────────────────────────────────────────────────────

def test_bfs_alvo(node):
    _ready(node)
    gx, gy, yaw, dist = node._bfs_target((0.5, 1.5, 0.0))
    assert gx > 1.0 and dist > 0
    assert node._bfs_target((-5.0, 1.5, 0.0)) is None     # fora do mapa
    node.blacklist.append((gx, gy))
    other = node._bfs_target((0.5, 1.5, 0.0))
    assert other is None or math.dist(other[:2], (gx, gy)) > node.blacklist_radius


def test_sem_fronteira_tenta_parede_e_depois_volta(node):
    _ready(node, grid=room(60))
    node.start_pose = (0.5, 1.5, 0.0)
    pose = (0.5, 1.5, 0.0)
    for _ in range(node.empty_rounds_to_finish - 1):
        node._go_bfs(pose)
        assert node.state == WALL
    node._go_bfs(pose)
    assert node.state == RETURNING and node.goal == (0.5, 1.5)
    node.nav.accept().finish(OK)
    assert node.state == DONE


def test_sem_voltar_ao_inicio(node):
    node.return_to_start = False
    node._finish()
    assert node.state == DONE


# ── Destino do Nav2 ──────────────────────────────────────────────────────────

def _going(node):
    _walking(node)
    node.seg_started_at -= node.wall_max_time + 1
    node._tick()
    return node.nav.accept()


def test_chegou_volta_para_parede(node):
    handle = _going(node)
    goal = node.goal
    node._tick()                                          # a caminho: nada muda
    handle.finish(OK)
    assert node.goals_reached == 1 and goal in node.blacklist and node.state == WALL


def test_chegou_sem_pose_espera(node):
    handle = _going(node)
    _no_pose(node)
    handle.finish(OK)
    assert node.state == WAIT


def test_cancelado_por_outro_cliente(node):
    _going(node).finish(CANCELED)
    assert node.state == DONE


def test_nav2_falhou_busca_outra(node):
    handle = _going(node)
    goal = node.goal
    handle.finish(ABORTED)
    assert goal in node.blacklist


def test_recusado(node):
    _walking(node)
    node.seg_started_at -= node.wall_max_time + 1
    node._tick()
    goal = node.goal
    node.nav.reject()
    assert goal in node.blacklist


def test_resultado_antigo_ignorado(node):
    _going(node)
    node._on_result(object(), OK)
    assert node.goals_reached == 0


def test_sem_progresso_cancela(node):
    handle = _going(node)
    x, y, _ = node.moved_at
    node.moved_at = (x, y, 0.0)
    node._tick()
    assert handle.canceled and node.goal_handle is not handle


def test_destino_estourou_tempo(node):
    handle = _going(node)
    node.goal_sent_at -= node.goal_timeout + 1
    set_tf(node, 1.0, 1.5)                                # andou
    node._tick()
    assert not handle.canceled                            # só atualizou o progresso
    node._tick()
    assert handle.canceled


def test_check_goal_aguardando(node):
    node.sending = True
    node._check_goal((0.0, 0.0, 0.0), 0.0)
    node.sending, node.goal = False, None
    node._check_goal((0.0, 0.0, 0.0), 0.0)


def test_falha_sem_pose_espera(node):
    node.goal = (1.0, 1.0)
    _no_pose(node)
    node._goal_failed('teste')
    assert node.state == WAIT and (1.0, 1.0) in node.blacklist


def test_falha_ao_voltar_encerra(node):
    node.state = RETURNING
    node._goal_failed('teste')
    assert node.state == DONE


def test_shutdown(node, monkeypatch):
    calls = []
    monkeypatch.setattr(wbe.rclpy, 'spin_until_future_complete',
                        lambda n, f, timeout_sec=None: calls.append(f))
    node.state = WALL
    node.shutdown()
    assert node.cmd_pub.msgs and not calls
    node.state = WAIT
    handle = _going(node)
    node.shutdown()
    assert handle.canceled and calls


def test_main(ros, monkeypatch):
    monkeypatch.setattr(wbe.rclpy, 'init', lambda **_: None)
    monkeypatch.setattr(wbe.rclpy, 'try_shutdown', lambda: None)
    monkeypatch.setattr(wbe.rclpy, 'ok', lambda: True)
    handlers = {}
    monkeypatch.setattr(wbe.signal, 'signal', lambda sig, h: handlers.setdefault(sig, h))
    monkeypatch.setattr(wbe.rclpy, 'spin_once',
                        lambda node, timeout_sec=None: handlers[wbe.signal.SIGTERM]())
    wbe.main()
    assert wbe.signal.SIGTERM in handlers
