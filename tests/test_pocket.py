"""Testes da exploração em desbaste (navigation/scripts/pocket.py)."""

import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'navigation', 'scripts'))

from pocket import (  # noqa: E402
    cell_to_world, find_outer_corners, is_outer_loop, known_silhouette, path_length, plan_pass,
    rasterize_path, region_contours, remaining_region, resample, signed_area, world_to_cell)

U, F, OCC = -1, 0, 100   # desconhecido, livre, ocupado
RES = 0.1
ORIGIN = (0.0, 0.0)


def _sala(n=100, ilha=None):
    """Sala n x n células (10 x 10 m) com parede em volta; ilha = (r0, r1, c0, c1)."""
    grid = np.full((n, n), F)
    grid[0, :] = grid[-1, :] = grid[:, 0] = grid[:, -1] = OCC
    if ilha:
        r0, r1, c0, c1 = ilha
        grid[r0:r1, c0:c1] = OCC
    return grid


def _retangulo(x0, y0, x1, y1, ccw=True, passo=0.05):
    """Pontos (m) de um retângulo fechado, no sentido pedido."""
    cantos = [(x0, y0), (x1, y0), (x1, y1), (x0, y1), (x0, y0)]
    if not ccw:
        cantos = cantos[::-1]
    pts = []
    for (ax, ay), (bx, by) in zip(cantos, cantos[1:]):
        n = max(1, int(math.dist((ax, ay), (bx, by)) / passo))
        pts += [(ax + (bx - ax) * i / n, ay + (by - ay) * i / n) for i in range(n)]
    return pts + [cantos[-1]]


def _densifica(pts, passo=0.05):
    out = [pts[0]]
    for a, b in zip(pts, pts[1:]):
        n = max(1, int(math.dist(a, b) / passo))
        out += [(a[0] + (b[0] - a[0]) * i / n, a[1] + (b[1] - a[1]) * i / n)
                for i in range(1, n + 1)]
    return out


# ── Geometria básica ─────────────────────────────────────────────────────────

def test_conversao_celula_mundo_ida_e_volta():
    r, c = world_to_cell(3.27, 1.04, ORIGIN, RES)
    assert (r, c) == (10, 32)
    x, y = cell_to_world(r, c, ORIGIN, RES)
    assert abs(x - 3.25) < 1e-9 and abs(y - 1.05) < 1e-9


def test_sentido_da_volta_parede_externa_x_ilha():
    anti = _retangulo(1, 1, 9, 9, ccw=True)
    hor = _retangulo(4, 4, 6, 6, ccw=False)
    assert signed_area(anti) > 0 > signed_area(hor)
    # parede à direita: a externa é percorrida no anti-horário, a ilha no horário
    assert is_outer_loop(anti, 'right') and not is_outer_loop(hor, 'right')
    assert is_outer_loop(hor, 'left') and not is_outer_loop(anti, 'left')


def test_rasterize_liga_pontos_e_nao_liga_saltos():
    m = rasterize_path([(1.05, 1.05), (1.95, 1.05)], ORIGIN, RES, (30, 30))
    assert m[10, 10:20].all()
    m = rasterize_path([(1.0, 1.0), (2.5, 1.0)], ORIGIN, RES, (30, 30), max_jump=1.0)
    assert m.sum() == 2              # salto (relocalização): marca os pontos, sem a reta


def test_resample_espaca_e_mantem_extremos():
    linha = np.array([[0, 0], [0, 10], [10, 10]], dtype=float)
    pts = resample(linha, 3.0)
    assert np.allclose(pts[0], [0, 0]) and np.allclose(pts[-1], [10, 10])
    passos = np.hypot(*np.diff(pts, axis=0).T)
    assert passos.max() <= 3.0 + 1e-9
    assert abs(path_length(linha) - 20.0) < 1e-9


# ── Região restante e passes ─────────────────────────────────────────────────

def test_regiao_fica_a_stepover_do_percorrido_e_com_folga_das_paredes():
    grid = _sala()
    contorno = _retangulo(0.8, 0.8, 9.2, 9.2)               # nível 0, rente às paredes
    trav = rasterize_path(contorno, ORIGIN, RES, grid.shape)
    reg = remaining_region(grid, RES, (50, 50), trav, stepover=1.5, clearance=0.6)
    rows, cols = np.nonzero(reg)
    xs, ys = (cols + 0.5) * RES, (rows + 0.5) * RES
    # tudo a >= 1.5 m do contorno: dentro de [2.3, 7.7] (com tolerância de 1 célula)
    assert xs.min() >= 2.3 - RES and xs.max() <= 7.7 + RES
    assert ys.min() >= 2.3 - RES and ys.max() <= 7.7 + RES
    assert reg[50, 50]


def test_sem_caminho_percorrido_a_regiao_respeita_a_folga():
    grid = _sala()
    reg = remaining_region(grid, RES, (50, 50), np.zeros_like(grid, dtype=bool),
                           stepover=1.5, clearance=0.6)
    rows, cols = np.nonzero(reg)
    assert (cols + 0.5).min() * RES > 0.6 and (rows + 0.5).min() * RES > 0.6


def test_desconhecido_nao_entra_na_regiao():
    grid = _sala()
    grid[:, 60:] = U                                         # metade leste não vista
    reg = remaining_region(grid, RES, (50, 30), np.zeros_like(grid, dtype=bool),
                           stepover=1.5, clearance=0.6)
    assert not reg[:, 55:].any()                             # folga de 0.6 m do desconhecido


def test_passe_comeca_perto_do_robo_e_deixa_o_feito_a_direita():
    grid = _sala()
    trav = rasterize_path(_retangulo(0.8, 0.8, 9.2, 9.2), ORIGIN, RES, grid.shape)
    reg = remaining_region(grid, RES, (8, 8), trav, stepover=1.5, clearance=0.6)
    robo = (8, 8)
    linha = plan_pass(region_contours(reg, 20), reg, robo, 'right')
    assert linha is not None
    assert tuple(linha[0]) == tuple(linha[-1])               # passe fechado
    # começa no canto mais perto do robô (sudoeste)
    assert math.hypot(linha[0][0] - robo[0], linha[0][1] - robo[1]) < 25
    # com a parede à direita, o passe interno também é anti-horário
    xy = [(c, r) for r, c in linha]
    assert signed_area(xy) > 0
    passe_esq = plan_pass(region_contours(reg, 20), reg, robo, 'left')
    assert signed_area([(c, r) for r, c in passe_esq]) < 0


def test_desbaste_completo_vai_em_niveis_ate_o_centro():
    """Simula: contorno externo e depois cada passe vira caminho percorrido.

    Cada nível fica mais para dentro e o desbaste termina.
    """
    grid = _sala()
    caminho = _retangulo(0.8, 0.8, 9.2, 9.2)
    robo = (8, 8)
    folgas = []
    for _ in range(20):
        trav = rasterize_path(caminho, ORIGIN, RES, grid.shape)
        reg = remaining_region(grid, RES, robo, trav, stepover=1.5, clearance=0.6, min_area=1.0)
        linha = plan_pass(region_contours(reg, 20), reg, robo, 'right')
        if linha is None:
            break
        pts = [cell_to_world(r, c, ORIGIN, RES) for r, c in resample(linha, 1.0 / RES)]
        # distância do passe à parede mais próxima (sala de 0 a 10 m)
        folgas.append(min(min(x, y, 10 - x, 10 - y) for x, y in pts))
        caminho = caminho + _densifica(pts)
        robo = world_to_cell(*pts[-1], ORIGIN, RES)
    else:
        raise AssertionError('o desbaste não terminou')
    assert len(folgas) >= 2                                   # contorno + pelo menos 2 níveis
    assert all(b > a + 1.0 for a, b in zip(folgas, folgas[1:]))   # cada nível ~1.5 m para dentro
    assert folgas[0] > 2.0


def test_ilha_vira_contorno_proprio():
    grid = _sala(ilha=(40, 60, 40, 60))                       # gôndola 2 x 2 m no meio
    trav = rasterize_path(_retangulo(0.8, 0.8, 9.2, 9.2), ORIGIN, RES, grid.shape)
    reg = remaining_region(grid, RES, (8, 8), trav, stepover=1.5, clearance=0.6)
    contornos = region_contours(reg, 20)
    assert len(contornos) == 2                                # borda externa + volta da ilha
    assert not reg[40:60, 40:60].any()


def test_pontos_bloqueados_saem_da_regiao():
    grid = _sala()
    bloq = np.zeros_like(grid, dtype=bool)
    bloq[45:55, 45:55] = True
    reg = remaining_region(grid, RES, (20, 20), np.zeros_like(bloq), 1.5, 0.6, bloq)
    assert not reg[45:55, 45:55].any() and reg[20, 20]


# ── Borda externa por visão computacional ────────────────────────────────────

def _supermercado(desconhecido_a_partir_da_coluna=None):
    """Supermercado de 20 x 14 m com parede em volta e 6 gôndolas de 8 m.

    Gôndolas ocupadas por fora e desconhecidas por dentro, como o LiDAR as vê.
    """
    g = np.full((140, 200), F)
    g[0, :] = g[-1, :] = g[:, 0] = g[:, -1] = OCC
    for c0 in range(30, 180, 25):
        g[30:110, c0:c0 + 8] = OCC
        g[33:107, c0 + 2:c0 + 6] = U
    if desconhecido_a_partir_da_coluna is not None:
        g[:, desconhecido_a_partir_da_coluna:] = U
    return g


def test_silhueta_ignora_as_gondolas():
    sil = known_silhouette(_supermercado(), RES)
    assert sil[60, 33] and sil[70, 105]          # dentro das gôndolas: preenchido
    assert not sil[0, 50] and not sil[-1, 50]    # parede externa fica fora


def test_cantos_externos_sao_os_quatro_da_loja_e_nao_das_gondolas():
    cantos = find_outer_corners(_supermercado(), RES)
    pts = sorted((round(k.corner[1] * RES), round(k.corner[0] * RES)) for k in cantos)  # (x, y) m
    assert pts == [(0, 0), (0, 14), (20, 0), (20, 14)]


def test_canto_aproxima_por_dentro_e_sai_com_a_parede_a_direita():
    cantos = find_outer_corners(_supermercado(), RES, wall_distance=0.7)
    sw = min(cantos, key=lambda k: k.corner[0] + k.corner[1])        # canto (0, 0)
    ax, ay = sw.approach[1] * RES, sw.approach[0] * RES
    assert 0.5 < ax < 1.0 and 0.5 < ay < 1.0                          # ~0.7 m das duas paredes
    assert abs(sw.yaw) < 0.1                     # sai para +x: parede y = 0 fica à direita
    esq = find_outer_corners(_supermercado(), RES, side='left')
    sw = min(esq, key=lambda k: k.corner[0] + k.corner[1])
    assert abs(sw.yaw - math.pi / 2) < 0.1       # parede à esquerda: sai para +y


def test_borda_com_o_desconhecido_nao_e_parede():
    """Só a metade oeste foi vista: os cantos do lado desconhecido não contam."""
    cantos = find_outer_corners(_supermercado(desconhecido_a_partir_da_coluna=120), RES)
    xs = sorted(round(k.corner[1] * RES) for k in cantos)
    assert xs == [0, 0]


def test_sala_em_l_acha_os_cantos_convexos_e_nao_o_reentrante():
    g = np.full((100, 100), OCC)
    g[1:40, 1:99] = F                     # braço de baixo
    g[1:99, 1:40] = F                     # braço da esquerda
    cantos = find_outer_corners(g, RES)
    pts = {(round(k.corner[1] * RES), round(k.corner[0] * RES)) for k in cantos}
    assert len(cantos) == 5
    assert (4, 4) not in pts              # canto reentrante (côncavo) do L


def test_mapa_vazio_nao_tem_canto():
    assert find_outer_corners(np.full((50, 50), U), RES) == []
