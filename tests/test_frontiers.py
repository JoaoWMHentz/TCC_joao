"""Testes da detecção de fronteiras (navigation/scripts/frontiers.py)."""

import os
import sys

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'navigation', 'scripts'))

from frontiers import (  # noqa: E402
    find_frontiers, frontier_mask, goal_candidates, min_distance)

U, F, O = -1, 0, 100   # desconhecido, livre, ocupado


def test_mapa_todo_conhecido_nao_tem_fronteira():
    grid = np.full((20, 20), F)
    grid[0, :] = grid[-1, :] = grid[:, 0] = grid[:, -1] = O
    assert find_frontiers(grid, 0.05, (0.0, 0.0), min_size=1) == []


def test_mapa_todo_desconhecido_nao_tem_fronteira():
    assert find_frontiers(np.full((10, 10), U), 0.05, (0.0, 0.0), min_size=1) == []


def test_fronteira_e_a_coluna_livre_vizinha_do_desconhecido():
    grid = np.full((10, 10), U)
    grid[:, :5] = F                      # metade esquerda conhecida e livre
    mask = frontier_mask(grid)
    assert mask[:, 4].all()              # só a última coluna livre
    assert mask.sum() == 10


def test_ocupado_vizinho_do_desconhecido_nao_e_fronteira():
    grid = np.full((10, 10), U)
    grid[:, :5] = F
    grid[:, 4] = O                       # parede separando livre e desconhecido
    assert not frontier_mask(grid).any()


def test_agrupamentos_separados_e_tamanho_minimo():
    grid = np.full((30, 30), O)
    grid[1:29, 1:29] = F
    grid[1:11, 29] = U                   # fronteira de 10 células (borda leste)
    grid[29, 5:8] = U                    # fronteira de 3 células (borda norte)
    fs = find_frontiers(grid, 0.1, (0.0, 0.0), min_size=5)
    assert len(fs) == 1
    assert fs[0].size == 10
    assert len(find_frontiers(grid, 0.1, (0.0, 0.0), min_size=1)) == 2


def test_coordenadas_no_referencial_do_mapa():
    grid = np.full((10, 10), U)
    grid[:, :5] = F
    f = find_frontiers(grid, 0.5, (-2.0, 1.0), min_size=1)[0]
    # coluna 4 -> x = -2.0 + 4.5 * 0.5 = 0.25 ; linhas 0..9 -> y médio = 1.0 + 5 * 0.5
    assert np.allclose(f.cells[:, 0], 0.25)
    assert np.isclose(f.centroid[0], 0.25)
    assert np.isclose(f.centroid[1], 3.5)
    assert np.isclose(f.length_m(0.5), 5.0)


def test_destino_e_a_celula_mais_proxima_do_centroide():
    # fronteira em "L": o centróide cai fora das células
    grid = np.full((20, 20), U)
    grid[:10, :10] = F
    f = find_frontiers(grid, 1.0, (0.0, 0.0), min_size=1)[0]
    cand = goal_candidates(f)
    assert min_distance(f.cells, *cand[0]) == 0.0      # é uma célula da fronteira
    d = np.hypot(cand[:, 0] - f.centroid[0], cand[:, 1] - f.centroid[1])
    assert np.all(np.diff(d) >= 0)                     # ordenadas por distância


def test_min_distance():
    pts = np.array([[0.0, 0.0], [3.0, 4.0]])
    assert min_distance(pts, 3.0, 0.0) == 3.0
    assert min_distance(np.empty((0, 2)), 1.0, 1.0) == float('inf')
