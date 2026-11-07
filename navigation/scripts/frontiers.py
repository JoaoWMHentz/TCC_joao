"""
frontiers.py — detecção de fronteiras num mapa de ocupação (sem ROS).

Fronteira = célula LIVRE vizinha (4-vizinhança) de uma célula DESCONHECIDA:
é a borda entre o que o robô já viu e o que ainda falta ver. Células de
fronteira vizinhas (8-vizinhança) formam um agrupamento; cada agrupamento
grande o bastante é um candidato a destino da exploração (Yamauchi, 1997).

Convenção do nav_msgs/OccupancyGrid (a mesma do RTAB-Map e do Nav2):
  -1 desconhecido · 0..100 probabilidade de ocupação.
"""

from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np
from scipy import ndimage

FREE_MAX = 25          # 0..25 = livre (o RTAB-Map publica 0 / 100 / -1)
EIGHT = np.ones((3, 3), dtype=bool)


@dataclass
class Frontier:
    size: int                       # número de células
    centroid: tuple[float, float]   # (x, y) no referencial do mapa
    cells: np.ndarray = field(repr=False)   # (N, 2) coordenadas x, y das células

    def length_m(self, resolution: float) -> float:
        return self.size * resolution


def frontier_mask(grid: np.ndarray) -> np.ndarray:
    """Máscara booleana das células de fronteira (grid: H x W, int)."""
    free = (grid >= 0) & (grid <= FREE_MAX)
    unknown = grid < 0
    near_unknown = np.zeros_like(unknown)
    near_unknown[1:, :] |= unknown[:-1, :]
    near_unknown[:-1, :] |= unknown[1:, :]
    near_unknown[:, 1:] |= unknown[:, :-1]
    near_unknown[:, :-1] |= unknown[:, 1:]
    return free & near_unknown


def find_frontiers(grid: np.ndarray, resolution: float, origin: tuple[float, float],
                   min_size: int = 8) -> list[Frontier]:
    """Agrupamentos de fronteira com pelo menos min_size células.

    grid[linha, coluna]: linha = y, coluna = x (mesma ordem do OccupancyGrid).
    origin: (x, y) do canto da célula [0, 0] no referencial do mapa.
    """
    labels, n = ndimage.label(frontier_mask(grid), structure=EIGHT)
    if n == 0:
        return []
    rows, cols = np.nonzero(labels)
    ids = labels[rows, cols]
    xs = origin[0] + (cols + 0.5) * resolution
    ys = origin[1] + (rows + 0.5) * resolution
    order = np.argsort(ids, kind='stable')
    ids, xs, ys = ids[order], xs[order], ys[order]
    bounds = np.flatnonzero(np.diff(ids)) + 1
    out = []
    for gx, gy in zip(np.split(xs, bounds), np.split(ys, bounds)):
        if gx.size < min_size:
            continue
        out.append(Frontier(size=int(gx.size), centroid=(float(gx.mean()), float(gy.mean())),
                            cells=np.column_stack([gx, gy])))
    return out


def goal_candidates(frontier: Frontier) -> np.ndarray:
    """Células do agrupamento ordenadas pela distância ao centróide.

    O centróide de uma fronteira curva pode cair fora dela (em área
    desconhecida ou numa parede); o destino é a célula da própria fronteira
    mais próxima dele, e as seguintes são alternativas se ela estiver
    bloqueada no costmap.
    """
    d = np.hypot(frontier.cells[:, 0] - frontier.centroid[0],
                 frontier.cells[:, 1] - frontier.centroid[1])
    return frontier.cells[np.argsort(d)]


def min_distance(points: np.ndarray, x: float, y: float) -> float:
    """Menor distância de (x, y) a um conjunto de pontos (N, 2)."""
    if points.size == 0:
        return float('inf')
    return float(np.min(np.hypot(points[:, 0] - x, points[:, 1] - y)))
