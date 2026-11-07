"""
wall_bfs.py — núcleo da exploração "seguir parede + BFS" (sem ROS).

Duas peças independentes, testadas em tests/test_wall_bfs.py:

1. Seguidor de parede (wall_follow_command): controle reativo sobre o
   LaserScan. Mantém a parede de um lado (direito ou esquerdo) a uma
   distância fixa; gira no lugar quando há obstáculo à frente, contorna
   quinas quando a parede some e anda reto procurando parede quando está
   em área aberta.

2. BFS no mapa de ocupação (bfs_to_frontier): busca em largura a partir
   da célula do robô, só por células livres com folga para o corpo, até a
   primeira célula de fronteira (livre vizinha do desconhecido). Como a BFS
   expande em ordem de distância percorrida, a primeira fronteira achada é
   a mais próxima *pelo caminho*, não em linha reta (atrás de uma parede
   não conta como perto).

Convenção do nav_msgs/OccupancyGrid: -1 desconhecido · 0..100 ocupação;
grid[linha, coluna] com linha = y e coluna = x.
"""

from __future__ import annotations

import math
from collections import deque
from dataclasses import dataclass

import numpy as np
from frontiers import FREE_MAX, frontier_mask
from scipy import ndimage

EIGHT = np.ones((3, 3), dtype=bool)
# 8-vizinhança com o custo de cada passo (diagonal = sqrt 2)
NEIGHBORS = [(-1, 0, 1.0), (1, 0, 1.0), (0, -1, 1.0), (0, 1, 1.0),
             (-1, -1, math.sqrt(2)), (-1, 1, math.sqrt(2)),
             (1, -1, math.sqrt(2)), (1, 1, math.sqrt(2))]


# ── BFS ──────────────────────────────────────────────────────────────────────

def traversable_mask(grid: np.ndarray, resolution: float, robot_radius: float) -> np.ndarray:
    """Células livres a mais de robot_radius do obstáculo mais próximo.

    Desconhecido não é atravessável: a BFS só anda pelo que já foi visto.
    """
    free = (grid >= 0) & (grid <= FREE_MAX)
    occupied = grid > FREE_MAX
    if not occupied.any():
        return free
    dist = ndimage.distance_transform_edt(~occupied) * resolution
    return free & (dist > robot_radius)


def frontier_targets(grid: np.ndarray, min_cells: int = 1) -> np.ndarray:
    """Máscara das células de fronteira cujo agrupamento (8-viz.) tem
    pelo menos min_cells células — descarta buracos isolados de ruído."""
    mask = frontier_mask(grid)
    if min_cells <= 1 or not mask.any():
        return mask
    labels, _ = ndimage.label(mask, structure=EIGHT)
    sizes = np.bincount(labels.ravel())
    keep = sizes >= min_cells
    keep[0] = False
    return keep[labels]


@dataclass
class BfsResult:
    goal: tuple[int, int]               # (linha, coluna) da fronteira
    path: list[tuple[int, int]]         # células do robô até o goal
    length_cells: float                 # comprimento do caminho em células


def bfs_to_frontier(passable: np.ndarray, targets: np.ndarray,
                    start: tuple[int, int]) -> BfsResult | None:
    """BFS (8-vizinhança) de start até a primeira célula de targets.

    passable: células por onde a busca pode andar (H x W, bool).
    targets:  células que encerram a busca (fronteiras); só contam se
              também forem passáveis.
    start:    (linha, coluna) do robô — sempre aceita como ponto de partida.
    Retorna None se nenhuma fronteira for alcançável.
    """
    h, w = passable.shape
    r0, c0 = start
    if not (0 <= r0 < h and 0 <= c0 < w):
        return None
    parent = np.full((h, w), -1, dtype=np.int64)
    depth = np.zeros((h, w), dtype=np.float64)
    seen = np.zeros((h, w), dtype=bool)
    seen[r0, c0] = True
    queue = deque([(r0, c0)])
    while queue:
        r, c = queue.popleft()
        if targets[r, c] and passable[r, c] and (r, c) != start:
            return BfsResult((r, c), _path(parent, w, r, c), float(depth[r, c]))
        for dr, dc, cost in NEIGHBORS:
            nr, nc = r + dr, c + dc
            if 0 <= nr < h and 0 <= nc < w and not seen[nr, nc] and passable[nr, nc]:
                if dr and dc and not (passable[r, nc] and passable[nr, c]):
                    continue        # não corta quina na diagonal
                seen[nr, nc] = True
                parent[nr, nc] = r * w + c
                depth[nr, nc] = depth[r, c] + cost
                queue.append((nr, nc))
    return None


def _path(parent: np.ndarray, w: int, r: int, c: int) -> list[tuple[int, int]]:
    path = [(r, c)]
    idx = parent[r, c]
    while idx >= 0:
        r, c = divmod(int(idx), w)
        path.append((r, c))
        idx = parent[r, c]
    return path[::-1]


def free_disk(grid: np.ndarray, center: tuple[int, int], radius_cells: int) -> np.ndarray:
    """Células livres num disco em volta de center. Somado à máscara de
    passagem, deixa a BFS sair mesmo se o robô estiver colado na parede
    (dentro da inflação)."""
    h, w = grid.shape
    rows, cols = np.ogrid[:h, :w]
    disk = (rows - center[0]) ** 2 + (cols - center[1]) ** 2 <= radius_cells ** 2
    return disk & (grid >= 0) & (grid <= FREE_MAX)


# ── Seguidor de parede ───────────────────────────────────────────────────────

@dataclass
class WallParams:
    side: str = 'right'           # lado em que a parede é mantida
    wall_distance: float = 0.70   # m — distância desejada (medida do LiDAR)
    front_clearance: float = 0.75 # m — obstáculo à frente: gira no lugar
    wall_lost: float = 1.40       # m — além disso, não há parede do lado
    linear_speed: float = 0.8     # m/s — em linha reta
    min_linear_speed: float = 0.12  # m/s — abaixo disso o BLDC trepida
    max_angular_speed: float = 0.8  # rad/s
    kp: float = 1.5               # ganho do erro de distância
    ka: float = 1.0               # ganho do erro de ângulo com a parede
    corner_turn_rate: float = 0.6 # rad/s — curva para contornar a quina
    slow_zone: float = 1.0        # m — além de front_clearance, desacelera até o mínimo


@dataclass
class ScanView:
    """Distâncias mínimas por setor, já no referencial do robô (graus,
    0 = frente, + = esquerda)."""
    front: float
    side: float          # perpendicular, do lado da parede
    side_front: float    # diagonal (45°) à frente, do lado da parede
    other_side: float


def sector_min(ranges: np.ndarray, angles: np.ndarray, center_deg: float,
               half_width_deg: float, range_max: float) -> float:
    d = np.degrees(np.angle(np.exp(1j * (angles - math.radians(center_deg)))))
    sel = ranges[np.abs(d) <= half_width_deg]
    sel = sel[np.isfinite(sel) & (sel > 0)]
    return float(sel.min()) if sel.size else range_max


def scan_view(ranges, angle_min: float, angle_increment: float, range_max: float,
              side: str, yaw_offset: float = 0.0) -> ScanView:
    """Resume o scan em setores. yaw_offset = yaw do frame do LiDAR em
    relação ao base_link (o RPLidar real é montado girado 180°)."""
    r = np.asarray(ranges, dtype=np.float64)
    r = np.where(np.isfinite(r), r, range_max)      # sem retorno = livre
    angles = angle_min + np.arange(r.size) * angle_increment + yaw_offset
    s = -1.0 if side == 'right' else 1.0
    return ScanView(
        front=sector_min(r, angles, 0.0, 25.0, range_max),
        side=sector_min(r, angles, 90.0 * s, 15.0, range_max),
        side_front=sector_min(r, angles, 45.0 * s, 12.0, range_max),
        other_side=sector_min(r, angles, -90.0 * s, 15.0, range_max),
    )


def wall_follow_command(view: ScanView, p: WallParams,
                        recently_had_wall: bool) -> tuple[float, float, str]:
    """(v, w, estado) para a leitura atual.

    Estados:
      'turn'    obstáculo à frente: gira no lugar para longe da parede;
      'corner'  a parede acabou há pouco: curva para o lado dela (contorna
                a quina externa);
      'seek'    área aberta, sem parede: anda reto até achar uma;
      'follow'  controle P na distância + correção do ângulo com a parede.
    """
    s = -1.0 if p.side == 'right' else 1.0   # sinal de w que vira PARA a parede
    if view.front < p.front_clearance:
        return 0.0, -s * p.max_angular_speed, 'turn'

    # distância à parede estimada pelos dois setores (perpendicular e 45°)
    wall_seen = min(view.side, view.side_front * math.cos(math.radians(45))) < p.wall_lost
    if not wall_seen:
        if recently_had_wall:
            return p.min_linear_speed, s * p.corner_turn_rate, 'corner'
        return p.linear_speed * _front_factor(view, p), 0.0, 'seek'

    dist_err = view.side - p.wall_distance            # > 0: longe demais
    # parede "abrindo" à frente (diagonal mais longe que o esperado) => virar para ela;
    # "fechando" (diagonal perto) => virar para fora
    expected_diag = view.side / math.cos(math.radians(45))
    angle_err = (min(view.side_front, 2 * p.wall_lost) - expected_diag) / max(expected_diag, 1e-3)
    w = s * (p.kp * dist_err + p.ka * angle_err)
    w = max(-p.max_angular_speed, min(p.max_angular_speed, w))
    # reduz a velocidade nas curvas e com obstáculo se aproximando à frente
    slow_turn = 1.0 - 0.6 * abs(w) / p.max_angular_speed
    v = max(p.min_linear_speed, p.linear_speed * min(slow_turn, _front_factor(view, p)))
    return v, w, 'follow'


def _front_factor(view: ScanView, p: WallParams) -> float:
    """Fração da velocidade permitida pelo obstáculo à frente: 1 longe,
    cai linearmente na slow_zone até o mínimo em front_clearance."""
    k = (view.front - p.front_clearance) / max(p.slow_zone, 1e-3)
    return max(p.min_linear_speed / p.linear_speed, min(1.0, k))
