"""
pocket.py — núcleo da exploração "desbaste" (como o pocketing de uma CNC), sem ROS.

Ideia: a fresa primeiro contorna a borda da cavidade e depois faz passes
paralelos, cada um deslocado (offset) do anterior por um passo lateral
(stepover), até chegar ao centro. Aqui:

  nível 0     o robô segue a parede externa (wall_bfs.wall_follow_command)
              e fecha a volta: acha os limites do ambiente;
  nível 1..N  cada passe é um offset de TUDO o que o robô já percorreu:
              a borda da região ainda não varrida, que fica a `stepover`
              metros do caminho percorrido (e a `clearance` dos obstáculos).
              Percorrido o passe, ele entra no caminho, a região encolhe e o
              próximo passe é o offset dele — até a região acabar.

Antes do nível 0, a borda externa é identificada no próprio mapa, com visão
computacional (find_outer_corners): o espaço livre tem os buracos preenchidos
(as prateleiras são ilhas cercadas de livre e somem), o contorno externo da
silhueta vira um polígono e os cantos dele com parede dos dois lados e sobre o
fecho convexo são cantos de parede externa. O robô vai até um deles e só
então começa a seguir a parede.

Funções puras sobre o OccupancyGrid (testadas em tests/test_pocket.py).
Convenção: -1 desconhecido · 0..100 ocupação; grid[linha, coluna] com
linha = y e coluna = x. Pontos em células são (linha, coluna).
"""

from __future__ import annotations

from dataclasses import dataclass
import math

import cv2
from frontiers import FREE_MAX
import numpy as np
from scipy import ndimage

EIGHT = np.ones((3, 3), dtype=bool)


# ── Caminho percorrido ───────────────────────────────────────────────────────

def world_to_cell(x: float, y: float, origin: tuple[float, float],
                  res: float) -> tuple[int, int]:
    return int(math.floor((y - origin[1]) / res)), int(math.floor((x - origin[0]) / res))


def cell_to_world(r: float, c: float, origin: tuple[float, float],
                  res: float) -> tuple[float, float]:
    return origin[0] + (c + 0.5) * res, origin[1] + (r + 0.5) * res


def rasterize_path(points_xy, origin: tuple[float, float], res: float,
                   shape: tuple[int, int], max_jump: float = 1.0) -> np.ndarray:
    """Máscara das células por onde o robô passou.

    Pontos em metros, na ordem em que foram registrados. Liga pontos
    consecutivos com uma reta; saltos maiores que max_jump (relocalização do
    SLAM) não são ligados.
    """
    img = np.zeros(shape, dtype=np.uint8)
    prev = None
    for x, y in points_xy:
        r, c = world_to_cell(x, y, origin, res)
        if prev is not None and math.dist(prev[0], (x, y)) <= max_jump:
            cv2.line(img, (prev[1][1], prev[1][0]), (c, r), 1, 1)
        elif 0 <= r < shape[0] and 0 <= c < shape[1]:
            img[r, c] = 1
        prev = ((x, y), (r, c))
    return img.astype(bool)


def signed_area(points_xy) -> float:
    """Área com sinal do polígono (fórmula do laço): > 0 anti-horário."""
    p = np.asarray(points_xy, dtype=np.float64)
    if len(p) < 3:
        return 0.0
    x, y = p[:, 0], p[:, 1]
    return 0.5 * float(np.dot(x, np.roll(y, -1)) - np.dot(y, np.roll(x, -1)))


def is_outer_loop(points_xy, side: str) -> bool:
    """Diz se a volta fechada seguindo a parede foi pela borda EXTERNA do ambiente.

    Com a parede à direita, contornar o ambiente por dentro é andar no
    sentido anti-horário (o interior fica à esquerda); contornar uma ilha
    (gôndola, pilar) é andar no horário. À esquerda, o contrário.
    """
    a = signed_area(points_xy)
    return a > 0 if side == 'right' else a < 0


# ── Região a desbastar e passes ──────────────────────────────────────────────

def clearance_mask(grid: np.ndarray, res: float, clearance: float) -> np.ndarray:
    """Células livres a mais de `clearance` m de qualquer obstáculo ou do desconhecido.

    O passe não encosta no que ainda não foi visto.
    """
    free = (grid >= 0) & (grid <= FREE_MAX)
    blocked = ~free
    if not blocked.any():
        return free
    return free & (ndimage.distance_transform_edt(~blocked) * res > clearance)


def reachable_component(mask: np.ndarray, start: tuple[int, int]) -> np.ndarray:
    """Componente conexa (8-viz.) de mask que contém start.

    Ou a mais próxima dele, se o robô estiver fora da máscara (colado na
    parede).
    """
    if not mask.any():
        return mask
    labels, _ = ndimage.label(mask, structure=EIGHT)
    h, w = mask.shape
    r, c = start
    if 0 <= r < h and 0 <= c < w and labels[r, c]:
        return labels == labels[r, c]
    rows, cols = np.nonzero(mask)
    i = int(np.argmin((rows - r) ** 2 + (cols - c) ** 2))
    return labels == labels[rows[i], cols[i]]


def remaining_region(grid: np.ndarray, res: float, start: tuple[int, int],
                     traveled: np.ndarray, stepover: float, clearance: float,
                     blocked: np.ndarray | None = None, min_area: float = 0.5) -> np.ndarray:
    """Região ainda não varrida.

    Alcançável, com folga dos obstáculos e a mais de `stepover` m de tudo o
    que o robô percorreu. A borda dela é o próximo passe (offset do caminho).
    Pedaços menores que min_area (m²) são descartados (sobras que não valem
    um passe).
    """
    region = reachable_component(clearance_mask(grid, res, clearance), start)
    if traveled.any():
        region &= ndimage.distance_transform_edt(~traveled) * res >= stepover
    if blocked is not None:
        region &= ~blocked
    if not region.any():
        return region
    labels, n = ndimage.label(region, structure=EIGHT)
    sizes = np.bincount(labels.ravel()) * res * res
    keep = sizes >= min_area
    keep[0] = False
    return keep[labels]


def region_contours(region: np.ndarray, min_length_cells: int = 1) -> list[np.ndarray]:
    """Bordas da região como polilinhas fechadas e ordenadas.

    Formato (N, 2) em (linha, coluna). Inclui as bordas externas e as dos
    buracos (ilhas).
    """
    if not region.any():
        return []
    found, _ = cv2.findContours(region.astype(np.uint8), cv2.RETR_LIST, cv2.CHAIN_APPROX_NONE)
    out = []
    for cnt in found:
        pts = cnt.reshape(-1, 2)[:, ::-1]         # (x=col, y=lin) -> (lin, col)
        if len(pts) >= max(min_length_cells, 3):
            out.append(pts.astype(np.int64))
    return out


def orient_contour(contour: np.ndarray, region: np.ndarray, side: str) -> np.ndarray:
    """Ordena o contorno para a região ainda não varrida ficar oposta à parede.

    Parede/caminho já feito à direita => região à esquerda, igual ao sentido
    em que o robô contornou a parede externa.
    """
    h, w = region.shape
    n = len(contour)
    step = max(1, n // 40)
    left_hits = right_hits = 0
    for i in range(0, n, step):
        r, c = contour[i]
        nr, nc = contour[(i + 2) % n]
        dr, dc = nr - r, nc - c
        if dr == 0 and dc == 0:
            continue
        # no plano (x = coluna, y = linha), a normal à esquerda de (dc, dr) é (-dr, dc)
        norm = math.hypot(dr, dc)
        lr, lc = dc / norm, -dr / norm            # deslocamento em (linha, coluna)
        for k, sign in ((1, 1), (1, -1)):
            rr, cc = int(round(r + sign * 2 * lr)), int(round(c + sign * 2 * lc))
            inside = 0 <= rr < h and 0 <= cc < w and region[rr, cc]
            if inside and sign > 0:
                left_hits += 1
            elif inside:
                right_hits += 1
    want_left = side == 'right'
    if (left_hits >= right_hits) != want_left:
        return contour[::-1].copy()
    return contour


def plan_pass(contours: list[np.ndarray], region: np.ndarray, robot: tuple[int, int],
              side: str) -> np.ndarray | None:
    """Próximo passe: o contorno mais perto do robô.

    Orientado e começando no ponto mais próximo dele, fechado (termina onde
    começou).
    """
    best, best_i, best_d = None, 0, math.inf
    for cnt in contours:
        d2 = (cnt[:, 0] - robot[0]) ** 2 + (cnt[:, 1] - robot[1]) ** 2
        i = int(np.argmin(d2))
        if d2[i] < best_d:
            best, best_i, best_d = cnt, i, float(d2[i])
    if best is None:
        return None
    start = tuple(best[best_i])
    oriented = orient_contour(best, region, side)
    j = int(np.flatnonzero((oriented[:, 0] == start[0]) & (oriented[:, 1] == start[1]))[0])
    rolled = np.roll(oriented, -j, axis=0)
    return np.vstack([rolled, rolled[:1]])


def resample(polyline: np.ndarray, spacing: float) -> np.ndarray:
    """Pontos a cada `spacing` (mesma unidade da polilinha) ao longo do comprimento.

    Sempre inclui o primeiro e o último ponto.
    """
    p = np.asarray(polyline, dtype=np.float64)
    if len(p) < 2:
        return p
    seg = np.hypot(*np.diff(p, axis=0).T)
    s = np.concatenate([[0.0], np.cumsum(seg)])
    total = s[-1]
    if total <= 0:
        return p[:1]
    n = max(1, int(math.ceil(total / spacing)))
    t = np.linspace(0.0, total, n + 1)
    return np.column_stack([np.interp(t, s, p[:, 0]), np.interp(t, s, p[:, 1])])


def path_length(polyline: np.ndarray) -> float:
    p = np.asarray(polyline, dtype=np.float64)
    return float(np.hypot(*np.diff(p, axis=0).T).sum()) if len(p) > 1 else 0.0


# ── Borda externa por visão computacional ────────────────────────────────────

@dataclass
class OuterCorner:
    corner: tuple[float, float]     # (linha, coluna) do vértice do polígono
    approach: tuple[float, float]   # (linha, coluna) onde o robô deve parar
    yaw: float                      # rad — direção da parede seguinte (parede à direita)
    walls_m: tuple[float, float]    # comprimento das duas paredes do canto (m)


def known_silhouette(grid: np.ndarray, res: float, close_m: float = 0.3) -> np.ndarray:
    """Silhueta do interior já visto, sem os obstáculos internos.

    Livre -> fechamento morfológico (tapa falhas de leitura) -> preenche os
    buracos. Prateleiras, pilares e o desconhecido cercado de livre viram
    buracos e somem; sobra a maior componente, limitada pelas paredes
    externas (e pelas fronteiras, onde o mapa ainda não chegou).
    """
    free = ((grid >= 0) & (grid <= FREE_MAX)).astype(np.uint8)
    k = max(1, int(round(close_m / res)))
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (2 * k + 1, 2 * k + 1))
    # borda de zeros: o OpenCV trata o lado de fora da imagem como "cheio" na
    # erosão, o que grudaria a parede encostada na borda do grid na silhueta
    padded = cv2.copyMakeBorder(free, k, k, k, k, cv2.BORDER_CONSTANT, value=0)
    closed = cv2.morphologyEx(padded, cv2.MORPH_CLOSE, kernel)[k:-k, k:-k].astype(bool)
    filled = ndimage.binary_fill_holes(closed)
    labels, n = ndimage.label(filled, structure=EIGHT)
    if n == 0:
        return filled
    sizes = np.bincount(labels.ravel())
    sizes[0] = 0
    return labels == int(np.argmax(sizes))


def _wall_backed(a: np.ndarray, b: np.ndarray, occupied: np.ndarray, reach: int) -> float:
    """Fração da aresta a->b com célula ocupada logo do lado de FORA.

    x=col, y=lin; polígono anti-horário. Conta a célula ocupada a até `reach`
    células do lado de FORA (à direita).
    """
    h, w = occupied.shape
    d = b - a
    length = float(np.hypot(*d))
    if length == 0:
        return 0.0
    ux, uy = d / length
    nx, ny = uy, -ux                           # normal à direita = para fora
    samples = max(2, int(length // 2))
    hits = 0
    for t in np.linspace(0.1, 0.9, samples):  # ignora as pontas (o canto)
        px, py = a + t * d
        for k in range(0, reach + 1):
            c, r = int(round(px + nx * k)), int(round(py + ny * k))
            if 0 <= r < h and 0 <= c < w and occupied[r, c]:
                hits += 1
                break
    return hits / samples


def find_outer_corners(grid: np.ndarray, res: float, wall_distance: float = 0.7,
                       min_wall_length: float = 1.5, approx_eps: float = 0.25,
                       hull_tolerance: float = 0.6, wall_backing: float = 0.7,
                       wall_reach: float = 0.4, side: str = 'right') -> list[OuterCorner]:
    """Cantos de parede externa identificados no mapa.

    Um vértice do polígono da silhueta (known_silhouette + approxPolyDP) é
    canto de parede externa se: é convexo com curva entre 45° e 135°; as duas
    arestas têm pelo menos min_wall_length e parede ocupada atrás em
    wall_backing da extensão (bordas com o desconhecido não contam); e está a
    até hull_tolerance do fecho convexo (descarta reentrâncias internas).

    approach = ponto a wall_distance das duas paredes, do lado de dentro;
    yaw = direção em que o robô sai do canto com a parede externa do lado
    `side`: anti-horário (aresta seguinte) com a parede à direita, horário
    (aresta anterior, ao contrário) com ela à esquerda.
    """
    sil = known_silhouette(grid, res)
    if not sil.any():
        return []
    found, _ = cv2.findContours(sil.astype(np.uint8), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    if not found:
        return []
    contour = max(found, key=cv2.contourArea)
    poly = cv2.approxPolyDP(contour, approx_eps / res, True).reshape(-1, 2).astype(np.float64)
    if len(poly) < 3:
        return []
    if signed_area(poly) < 0:                  # (x=col, y=lin) com y para cima: anti-horário
        poly = poly[::-1]
    hull = cv2.convexHull(contour)
    occupied = grid > FREE_MAX
    reach = max(1, int(round(wall_reach / res)))
    out = []
    n = len(poly)
    for i in range(n):
        p, v, q = poly[i - 1], poly[i], poly[(i + 1) % n]
        e_in, e_out = v - p, q - v
        l_in, l_out = float(np.hypot(*e_in)), float(np.hypot(*e_out))
        if min(l_in, l_out) * res < min_wall_length:
            continue
        cross = e_in[0] * e_out[1] - e_in[1] * e_out[0]
        turn = math.degrees(math.atan2(cross, float(np.dot(e_in, e_out))))
        if not 45.0 <= turn <= 135.0:          # > 0: curva à esquerda = canto convexo
            continue
        hull_dist = abs(cv2.pointPolygonTest(hull, (float(v[0]), float(v[1])), True))
        if hull_dist * res > hull_tolerance:
            continue
        if (_wall_backed(p, v, occupied, reach) < wall_backing
                or _wall_backed(v, q, occupied, reach) < wall_backing):
            continue
        u_in, u_out = e_in / l_in, e_out / l_out
        left = np.array([-u_in[1], u_in[0]]) + np.array([-u_out[1], u_out[0]])
        a = v + left * (wall_distance / res)
        out.append(OuterCorner(corner=(float(v[1]), float(v[0])),
                               approach=(float(a[1]), float(a[0])),
                               yaw=(math.atan2(u_out[1], u_out[0]) if side == 'right'
                                    else math.atan2(-u_in[1], -u_in[0])),
                               walls_m=(l_in * res, l_out * res)))
    return out
