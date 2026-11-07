"""Testes do nó de exploração em desbaste (navigation/scripts/pocket_explorer.py)."""

import math
from types import SimpleNamespace

import numpy as np
import pytest

pytest.importorskip('rclpy')

import pocket_explorer as pe  # noqa: E402
from action_msgs.msg import GoalStatus  # noqa: E402
from pocket_explorer import PASS, WAIT_POSE  # noqa: E402
from ros_fakes import FakeActionClient, make_map, make_scan, room, set_tf  # noqa: E402
from wall_bfs_explorer import BFS_GOAL, DONE, RETURNING, WAIT, WALL  # noqa: E402

OK, ABORTED, CANCELED = (GoalStatus.STATUS_SUCCEEDED, GoalStatus.STATUS_ABORTED,
                         GoalStatus.STATUS_CANCELED)
CENTER = (2.0, 2.0, 0.0)


class FakePub:

    def __init__(self):
        self.msgs = []

    def publish(self, msg):
        self.msgs.append(msg)


def _patch():
    """Mapa só com um pedaço livre no meio do desconhecido: sem cantos de parede."""
    grid = np.full((80, 80), -1, dtype=np.int16)
    grid[30:50, 30:50] = 0
    return grid


@pytest.fixture
def node(ros):
    n = pe.PocketExplorer()
    n.nav = FakeActionClient()
    n.passes = FakeActionClient()
    n.cmd_pub = FakePub()
    yield n
    n.destroy_node()


def _ready(node, grid=None, pose=CENTER):
    node._on_map(make_map(room(80) if grid is None else grid))
    set_tf(node, 0.0, 0.0, 0.0, parent='base_link', child='laser')
    node._on_scan(make_scan([1.0] * 360))
    set_tf(node, *pose)


def _no_pose(node):
    node._robot_pose = lambda: None


def _square(node, x0, y0, x1, y1, ccw=True, step=0.05):
    pts = [(x0, y0), (x1, y0), (x1, y1), (x0, y1), (x0, y0)]
    if not ccw:
        pts = pts[::-1]
    for (ax, ay), (bx, by) in zip(pts, pts[1:]):
        n = int(math.dist((ax, ay), (bx, by)) / step)
        yaw = math.atan2(by - ay, bx - ax)
        for i in range(n):
            node.path.add(ax + (bx - ax) * i / n, ay + (by - ay) * i / n, yaw)


# ── Canto da parede externa ──────────────────────────────────────────────────

def test_comeca_indo_ao_canto(node):
    _ready(node)
    node._tick()
    assert node.state == BFS_GOAL and node.phase == 'corner' and node.corner is not None
    assert len(node.nav.sent) == 1


def test_no_canto_comeca_o_contorno(node):
    _ready(node)
    node._tick()
    node.nav.accept().finish(OK)
    assert node.at_corner and node.state == WALL and node.phase == ''
    assert node.seg_points
    n = len(node.seg_points)
    node._record((2.5, 2.0, 0.0))
    assert len(node.seg_points) == n + 1


def test_canto_inalcancavel_tenta_outro(node):
    _ready(node)
    node._tick()
    first = node.corner
    node.nav.reject()
    assert first in node.bad_corners
    assert node.corner != first and node.phase == 'corner'


def test_canto_falhou_sem_pose(node):
    _ready(node)
    node._tick()
    _no_pose(node)
    node.nav.reject()
    assert node.phase == '' and node.goal is None


def test_outer_corner_filtros(node):
    assert node._outer_corner(CENTER) is None             # sem mapa
    _ready(node)
    assert node._outer_corner(CENTER) is not None
    node.robot_radius = 5.0                               # nenhum ponto de chegada livre
    assert node._outer_corner(CENTER) is None
    node.robot_radius = 0.33
    node.bad_corners = [(x * 0.05, y * 0.05) for x in (1, 78) for y in (1, 78)]
    assert node._outer_corner(CENTER) is None


def test_sem_canto_revela_mapa(node):
    _ready(node, grid=_patch())
    node._tick()
    assert node.phase == 'reveal' and node.state == BFS_GOAL
    goal = node.goal
    node.nav.reject()                                     # fronteira de revelar falhou
    assert goal in node.blacklist


def test_revelar_falhou_sem_pose(node):
    _ready(node, grid=_patch())
    node._tick()
    _no_pose(node)
    node.nav.reject()
    assert node.phase == '' and node.goal is None


def test_revelou_e_procura_de_novo(node):
    _ready(node, grid=_patch())
    node._tick()
    node.nav.accept().finish(OK)
    assert node.corner_tries == 2


def test_desiste_do_canto_e_segue_parede(node):
    node.corner_search_attempts = 0
    _ready(node, grid=_patch())
    node._tick()
    assert node.at_corner and node.state == WALL


# ── Contorno (nível 0) ───────────────────────────────────────────────────────

def test_contorno_aberto_inicia_passes(node):
    _ready(node)
    node.seg_points = [(2.0, 2.0)]
    node._after_wall(CENTER, 'teste')
    assert node.contour_done and node.state == PASS and node.level == 1


def test_contorno_em_ilha_procura_outro_canto(node):
    _ready(node)
    node.corner = (0.05, 0.05)
    _square(node, 1.5, 1.5, 2.5, 2.5, ccw=False)          # horário: ilha (parede à direita)
    node.loop_hit = 0
    node._after_wall(CENTER, 'voltou')
    assert node.contour_tries == 1 and (0.05, 0.05) in node.bad_corners
    assert node.phase == 'corner' and not node.contour_done


def test_contorno_externo_fechado(node):
    _ready(node)
    _square(node, 0.7, 0.7, 3.3, 3.3, ccw=True)
    node.loop_hit = 0
    node._after_wall(CENTER, 'voltou')
    assert node.contour_done


def test_contorno_so_para_ao_fechar_travar_ou_estourar(node):
    now = 1000.0
    node.seg_started_at = now
    assert node._wall_exhausted(CENTER, now, now) is None
    assert 'parado' in node._wall_exhausted(CENTER, now, now - node.stuck_time - 1)
    node.seg_started_at = now - node.wall_max_time - 1
    assert 'contorno' in node._wall_exhausted(CENTER, now, now)
    node.seg_started_at = now
    _square(node, 0.0, 0.0, 3.0, 3.0)
    assert node._revisiting((0.1, 0.0, 0.0)) is None      # começou a revisitar
    for i in range(30):
        node.path.add(9.0 + i * 0.05, 9.0, 0.0)
    assert 'já percorrido' in node._wall_exhausted((0.1, 0.0, 0.0), now, now)


def test_wall_step_na_parede(node):
    _ready(node)
    node.at_corner = True
    node._tick()
    assert node.state == WALL
    node._tick()
    assert node.cmd_pub.msgs


# ── Acabamento (BFS) ─────────────────────────────────────────────────────────

def test_go_bfs_sem_alvo_antes_do_contorno_vai_aos_passes(node):
    _ready(node)
    node._go_bfs(CENTER)
    assert node.contour_done and node.state == PASS


def test_go_bfs_sem_alvo_depois_do_contorno_termina(node):
    _ready(node)
    node.contour_done = True
    node.start_pose = CENTER
    node._go_bfs(CENTER)
    assert node.state == RETURNING


def test_go_bfs_com_fronteira(node):
    grid = room(80)
    grid[:, 50:] = -1
    _ready(node, grid=grid)
    node._go_bfs(CENTER)
    assert node.state == BFS_GOAL


def test_chegou_na_fronteira_do_acabamento_planeja_passe(node):
    _ready(node)
    node.contour_done = True
    node._after_frontier(CENTER)
    assert node.state == PASS


def test_falha_comum_vai_para_super(node):
    _ready(node)
    node.goal = (1.0, 1.0)
    node._goal_failed('teste')
    assert (1.0, 1.0) in node.blacklist


# ── Passes (níveis 1..N) ─────────────────────────────────────────────────────

def _passing(node):
    _ready(node)
    node.contour_done = True
    node._next_pass(CENTER)
    assert node.state == PASS and node.sending
    return node.passes.sent[-1]


def test_passe_tem_poses_orientadas(node):
    sent = _passing(node)
    poses = sent.goal.poses
    assert len(poses) == len(node.pass_points) > 2
    assert all(p.header.frame_id == 'map' for p in poses)
    sent.feedback(SimpleNamespace(feedback=SimpleNamespace(number_of_poses_remaining=2)))
    assert node.pass_remaining == 2


def test_passe_concluido_planeja_o_proximo(node):
    _passing(node)
    handle = node.passes.accept()
    assert node.pass_handle is handle and not node.sending
    node._tick()                                          # andando: nada muda
    handle.finish(OK)
    assert node.pass_failures == 0 and node.level >= 1 and node.state in (PASS, DONE)


def test_passe_cancelado_por_outro_cliente(node):
    _passing(node)
    node.passes.accept().finish(CANCELED)
    assert node.state == DONE


def test_passe_falhou(node):
    _passing(node)
    node.passes.accept().finish(ABORTED)
    assert node.blocked and node.pass_failures == 1


def test_passe_recusado(node):
    _passing(node)
    node.passes.reject()
    assert node.pass_failures == 1


def test_resultado_de_passe_antigo_ignorado(node):
    _passing(node)
    node.passes.accept()
    node._on_pass_result(object(), ABORTED)
    assert node.pass_failures == 0


def test_passe_sem_progresso_cancela(node):
    _passing(node)
    handle = node.passes.accept()
    x, y, _ = node.moved_at
    node.moved_at = (x, y, 0.0)
    node._tick()
    assert handle.canceled and node.pass_failures == 1


def test_passe_estourou_o_tempo(node):
    _passing(node)
    handle = node.passes.accept()
    node.goal_sent_at -= node.pass_timeout + 1
    node._check_pass((2.5, 2.0, 0.0), node.goal_sent_at + node.pass_timeout + 1)  # andou
    assert not handle.canceled
    node._tick()                                          # volta ao centro: ainda andando
    node._tick()
    assert handle.canceled


def test_check_pass_aguardando(node):
    node.sending = True
    node._check_pass(CENTER, 0.0)
    node.sending, node.pass_handle = False, None
    node._check_pass(CENTER, 0.0)


def test_muitas_falhas_vai_ao_acabamento(node):
    _passing(node)
    node.max_pass_failures = 1
    node.start_pose = CENTER
    node.passes.reject()
    assert node.pass_failures == 0 and node.state in (RETURNING, BFS_GOAL, PASS)


def test_muitas_falhas_sem_pose(node):
    _passing(node)
    node.max_pass_failures = 1
    _no_pose(node)
    node.passes.reject()
    assert node.pass_failures == 0


def test_sem_pose_espera_e_retoma(node):
    _passing(node)
    real = node._robot_pose
    _no_pose(node)
    node._tick()                                          # PASS sem pose: espera
    node.passes.accept().finish(OK)
    assert node.state == WAIT_POSE
    node._robot_pose = real
    node._tick()
    assert node.state != WAIT_POSE


def test_pontos_bloqueados_saem_do_plano(node):
    _ready(node)
    full = node._plan(CENTER)
    node.blocked = list(full)
    assert node._plan(CENTER) != full


def test_desbaste_concluido_vai_ao_acabamento(node, monkeypatch):
    _ready(node)
    monkeypatch.setattr(node, '_plan', lambda pose: None)
    node.contour_done = True
    node.start_pose = CENTER
    node._next_pass(CENTER)
    assert node.state == RETURNING


def test_primeiro_ponto_no_robo_e_pulado(node, monkeypatch):
    _ready(node)
    monkeypatch.setattr(node, '_plan', lambda pose: [(2.1, 2.0), (3.0, 2.0)])
    node._next_pass(CENTER)
    assert node.pass_points == [(3.0, 2.0)]


def test_tick_delegado_fora_dos_passes(node):
    node.state = DONE
    node._tick()
    assert node.state == DONE


# ── Fim ──────────────────────────────────────────────────────────────────────

def test_finish(node):
    node.return_to_start = False
    node._finish()
    assert node.state == DONE


def test_shutdown_cancela_passe(node, monkeypatch):
    calls = []
    monkeypatch.setattr(pe.rclpy, 'spin_until_future_complete',
                        lambda n, f, timeout_sec=None: calls.append(f))
    _passing(node)
    handle = node.passes.accept()
    node.shutdown()
    assert handle.canceled and calls


def test_main(ros, monkeypatch):
    monkeypatch.setattr(pe.rclpy, 'init', lambda **_: None)
    monkeypatch.setattr(pe.rclpy, 'try_shutdown', lambda: None)
    monkeypatch.setattr(pe.rclpy, 'ok', lambda: True)
    handlers = {}
    monkeypatch.setattr(pe.signal, 'signal', lambda sig, h: handlers.setdefault(sig, h))
    monkeypatch.setattr(pe.rclpy, 'spin_once',
                        lambda node, timeout_sec=None: handlers[pe.signal.SIGTERM]())
    pe.main()
    assert pe.signal.SIGTERM in handlers


def test_estado_inicial(node):
    assert node.state == WAIT and node.level == 0
