"""Testes da exploração seguir parede + BFS (navigation/scripts/wall_bfs.py)."""

import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'navigation', 'scripts'))

from wall_bfs import (  # noqa: E402
    ScanView, WallParams, bfs_to_frontier, free_disk, frontier_targets, scan_view,
    traversable_mask, wall_follow_command)

U, F, O = -1, 0, 100   # desconhecido, livre, ocupado


# ── BFS ──────────────────────────────────────────────────────────────────────

def _sala():
    grid = np.full((20, 20), F)
    grid[0, :] = grid[-1, :] = grid[:, 0] = grid[:, -1] = O
    return grid


def test_bfs_sem_fronteira_retorna_none():
    grid = _sala()
    passable = traversable_mask(grid, 0.1, 0.0)
    assert bfs_to_frontier(passable, frontier_targets(grid), (10, 10)) is None


def test_bfs_acha_a_fronteira_e_caminho_contiguo():
    grid = _sala()
    grid[5:10, 19] = U                   # abertura na parede leste
    grid[5:10, 18] = F
    passable = traversable_mask(grid, 0.1, 0.0)
    res = bfs_to_frontier(passable, frontier_targets(grid), (7, 3))
    assert res is not None
    assert res.goal[1] == 18 and 5 <= res.goal[0] < 10
    assert res.path[0] == (7, 3) and res.path[-1] == res.goal
    for (r0, c0), (r1, c1) in zip(res.path, res.path[1:]):
        assert max(abs(r1 - r0), abs(c1 - c0)) == 1


def test_bfs_prefere_a_mais_perto_pelo_caminho_e_nao_em_linha_reta():
    # Robô em (10, 8). Fronteira A logo atrás de uma parede (perto em linha
    # reta, mas o caminho dá a volta); fronteira B mais longe e em linha livre.
    grid = np.full((21, 30), F)
    grid[0, :] = grid[-1, :] = grid[:, 0] = grid[:, -1] = O
    grid[1:19, 10] = O                   # duas paredes verticais, abertas só em cima
    grid[1:19, 12] = O                   # (linha 19); A fica no corredor entre elas
    grid[10, 11] = U                     # fronteira A: célula (10, 11) desconhecida
    grid[10, 1] = U                      # fronteira B: borda oeste, a 7 células livres
    passable = traversable_mask(grid, 0.1, 0.0)
    res = bfs_to_frontier(passable, frontier_targets(grid), (10, 8))
    assert res is not None
    assert res.goal[1] <= 2              # foi para B


def test_bfs_respeita_folga_do_robo():
    grid = _sala()
    grid[1:19, 10] = O
    grid[9, 10] = F                      # porta de uma célula: o robô não cabe
    grid[1:19, 15:19] = U                # desconhecido do outro lado
    passable = traversable_mask(grid, 0.1, 0.15)
    assert bfs_to_frontier(passable, frontier_targets(grid), (9, 4)) is None
    assert bfs_to_frontier(traversable_mask(grid, 0.1, 0.0),
                           frontier_targets(grid), (9, 4)) is not None


def test_free_disk_deixa_sair_de_dentro_da_inflacao():
    grid = _sala()
    grid[5:10, 19] = U
    grid[5:10, 18] = F
    passable = traversable_mask(grid, 0.1, 0.25)
    start = (1, 1)                       # colado no canto: dentro da inflação
    assert not passable[start]
    passable |= free_disk(grid, start, 4)
    assert bfs_to_frontier(passable, frontier_targets(grid), start) is not None


def test_frontier_targets_descarta_agrupamentos_pequenos():
    grid = _sala()
    grid[3, 19] = U                      # 1 célula
    grid[10:16, 19] = U                  # 6 células
    assert frontier_targets(grid, 1).sum() == 7
    t = frontier_targets(grid, 3)
    assert t.sum() == 6 and not t[3, 18]


# ── Seguidor de parede ───────────────────────────────────────────────────────

P = WallParams(side='right')


def test_obstaculo_a_frente_gira_no_lugar_para_longe_da_parede():
    v, w, st = wall_follow_command(ScanView(front=0.4, side=0.55, side_front=0.8,
                                            other_side=2.0), P, True)
    assert st == 'turn' and v == 0.0 and w > 0      # parede à direita: gira à esquerda


def test_longe_da_parede_vira_para_ela():
    v, w, st = wall_follow_command(ScanView(front=5.0, side=0.9, side_front=0.9 * math.sqrt(2),
                                            other_side=5.0), P, True)
    assert st == 'follow' and w < 0 and v >= P.min_linear_speed


def test_perto_da_parede_vira_para_fora():
    _, w, st = wall_follow_command(ScanView(front=5.0, side=0.35, side_front=0.35 * math.sqrt(2),
                                            other_side=5.0), P, True)
    assert st == 'follow' and w > 0


def test_paralelo_na_distancia_certa_anda_reto():
    d = P.wall_distance
    v, w, st = wall_follow_command(ScanView(front=5.0, side=d, side_front=d * math.sqrt(2),
                                            other_side=5.0), P, True)
    assert st == 'follow' and abs(w) < 1e-6 and math.isclose(v, P.linear_speed)


def test_parede_acabou_contorna_a_quina_ou_procura():
    view = ScanView(front=5.0, side=5.0, side_front=5.0, other_side=5.0)
    v, w, st = wall_follow_command(view, P, recently_had_wall=True)
    assert st == 'corner' and w < 0 and v > 0
    v, w, st = wall_follow_command(view, P, recently_had_wall=False)
    assert st == 'seek' and w == 0.0 and v == P.linear_speed


def test_lado_esquerdo_espelha_o_comando():
    left = WallParams(side='left')
    view = ScanView(front=5.0, side=0.9, side_front=0.9 * math.sqrt(2), other_side=5.0)
    _, wr, _ = wall_follow_command(view, P, True)
    _, wl, _ = wall_follow_command(view, left, True)
    assert math.isclose(wl, -wr)


def test_scan_view_aplica_o_yaw_do_lidar():
    # Scan 360° com 1 leitura por grau; só há um obstáculo a 0.5 m no ângulo
    # 180° do LiDAR. Montado girado 180° (yaw_offset = pi), isso é a FRENTE do robô.
    n = 360
    ranges = np.full(n, np.inf)
    ranges[180] = 0.5
    angle_min, inc = -math.pi, 2 * math.pi / n     # índice 180 -> ângulo 0 no LiDAR
    v0 = scan_view(ranges, angle_min, inc, 12.0, 'right', yaw_offset=0.0)
    assert v0.front == 0.5                          # sem offset: ângulo 0 = frente
    ranges = np.full(n, np.inf)
    ranges[0] = 0.5                                 # ângulo -180° do LiDAR
    v1 = scan_view(ranges, angle_min, inc, 12.0, 'right', yaw_offset=math.pi)
    assert v1.front == 0.5 and v1.side == 12.0


def test_desacelera_com_obstaculo_se_aproximando_a_frente():
    d = P.wall_distance
    def v_at(front):
        return wall_follow_command(ScanView(front=front, side=d, side_front=d * math.sqrt(2),
                                            other_side=5.0), P, True)[0]
    assert math.isclose(v_at(P.front_clearance + P.slow_zone + 0.5), P.linear_speed)
    assert v_at(P.front_clearance + P.slow_zone / 2) < P.linear_speed
    assert math.isclose(v_at(P.front_clearance + 0.01), P.min_linear_speed)
    # procurando parede em área aberta também desacelera
    v_seek, _, st = wall_follow_command(ScanView(front=1.0, side=5.0, side_front=5.0,
                                                 other_side=5.0), P, False)
    assert st == 'seek' and P.min_linear_speed <= v_seek < P.linear_speed
