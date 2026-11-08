#!/usr/bin/env python3
"""
gerar_supermercado.py — gera sim/worlds/supermercado.sdf.

Supermercado de 30 x 20 m (x de -15 a 15, y de -10 a 10), com:
  - 6 gôndolas duplas com produtos e pontas de gôndola (corredores de 2,1 m);
  - placas suspensas dos corredores;
  - 6 caixas (checkouts) com esteira, monitor, banco e luz de número;
  - geladeiras com portas de vidro na parede norte e 3 ilhas de congelados;
  - padaria (balcão de vidro e prateleiras de pães) na parede oeste;
  - hortifrúti com bancas de caixotes e prateleira na parede leste;
  - entrada com portas de vidro, balcão de atendimento e carrinhos;
  - pilares, paletes de promoção e carrinhos largados nos corredores.

Os produtos são só visuais dentro do modelo de cada móvel (uma colisão por
móvel), para a física continuar leve. Os sensores do robô (LiDAR e câmera)
enxergam os visuais. O resultado é determinístico (semente fixa).

Planta (y para cima = norte; a entrada fica no sul, à direita):

   y=10 ┌──────────────── geladeiras ─────────────────────────┐
        │   [ilha]        [ilha]        [ilha]                 │
        │P  ║   ║   ║   ║   ║   ║          hortifrúti        H │
        │A  ║   ║   ║   ║   ║   ║   ▣ ▣                      O │
        │D  ║   ║   ║   ║   ║   ║   ▣ ▣                      R │
        │   ▪  palete  ▪       ▪  palete       palete          │
        │  ┃caixa┃ ┃ ┃ ┃ ┃ ┃ ┃ ┃ ┃ ┃    atendim.     carrinhos │
   y=-10└─────────────────────────────────────────[ portas ]───┘
       x=-15                                                 x=15

Uso:
  python3 sim/scripts/gerar_supermercado.py            # grava o .sdf
  python3 sim/scripts/gerar_supermercado.py --stdout   # só imprime
"""

import argparse
import math
from pathlib import Path
import random

RNG = random.Random(2026)

# ── dimensões gerais ────────────────────────────────────────────────────────
X_MIN, X_MAX, Y_MIN, Y_MAX = -15.0, 15.0, -10.0, 10.0
WALL_H, WALL_T = 3.5, 0.2
DOOR_X0, DOOR_X1 = 9.5, 12.5            # vão das portas (parede sul)

GONDOLA_XS = [-10.5, -7.5, -4.5, -1.5, 1.5, 4.5]
GONDOLA_YC, GONDOLA_L, GONDOLA_W = 1.2, 6.5, 0.9
CHECKOUT_XS = [-11.0, -8.0, -5.0, -2.0, 1.0, 4.0]
CHECKOUT_YC = -7.4

# ── cores ───────────────────────────────────────────────────────────────────
PRODUTOS = {   # paleta por seção (corredor)
    'mercearia': [(0.85, 0.2, 0.15), (0.95, 0.75, 0.1), (0.9, 0.5, 0.1), (0.6, 0.15, 0.1),
                  (0.95, 0.9, 0.7)],
    'bebidas':   [(0.1, 0.35, 0.8), (0.75, 0.1, 0.1), (0.1, 0.55, 0.25), (0.95, 0.6, 0.1),
                  (0.3, 0.2, 0.15)],
    'limpeza':   [(0.2, 0.6, 0.9), (0.95, 0.95, 0.95), (0.4, 0.8, 0.4), (0.8, 0.3, 0.7),
                  (0.95, 0.85, 0.2)],
    'higiene':   [(0.9, 0.6, 0.75), (0.6, 0.8, 0.95), (0.95, 0.95, 0.95), (0.5, 0.3, 0.7),
                  (0.3, 0.7, 0.7)],
    'biscoitos': [(0.85, 0.55, 0.2), (0.6, 0.3, 0.1), (0.95, 0.8, 0.3), (0.9, 0.2, 0.3),
                  (0.2, 0.4, 0.8)],
    'pet':       [(0.2, 0.5, 0.3), (0.9, 0.5, 0.2), (0.3, 0.3, 0.6), (0.8, 0.8, 0.3),
                  (0.6, 0.4, 0.3)],
}
SECOES = list(PRODUTOS)
COR_SECAO = {'mercearia': (0.85, 0.25, 0.15), 'bebidas': (0.15, 0.4, 0.8),
             'limpeza': (0.2, 0.65, 0.8), 'higiene': (0.75, 0.4, 0.65),
             'biscoitos': (0.9, 0.6, 0.15), 'pet': (0.3, 0.6, 0.3)}
FRUTAS = [(0.85, 0.1, 0.1), (0.98, 0.8, 0.1), (0.95, 0.55, 0.05), (0.3, 0.65, 0.15),
          (0.55, 0.15, 0.4), (0.15, 0.45, 0.1), (0.9, 0.85, 0.5), (0.65, 0.35, 0.15)]
METAL = (0.55, 0.57, 0.6)
METAL_ESCURO = (0.25, 0.26, 0.28)
BRANCO = (0.93, 0.93, 0.92)
MADEIRA = (0.6, 0.42, 0.25)
VIDRO = (0.7, 0.85, 0.95)


def _c(rgb):
    return f'{rgb[0]:.2f} {rgb[1]:.2f} {rgb[2]:.2f} 1'


def _vary(rgb, amt=0.08):
    return tuple(min(1.0, max(0.0, v + RNG.uniform(-amt, amt))) for v in rgb)


class Model:
    """Modelo estático: um link, vários visuais e colisões (pose local)."""

    def __init__(self, name, x, y, yaw=0.0):
        self.name, self.x, self.y, self.yaw = name, x, y, yaw
        self.parts = []
        self._n = 0

    def _id(self, kind):
        self._n += 1
        return f'{kind}{self._n}'

    @staticmethod
    def _geom(shape, size):
        if shape == 'box':
            return f'<box><size>{size[0]:.3f} {size[1]:.3f} {size[2]:.3f}</size></box>'
        if shape == 'cyl':
            return (f'<cylinder><radius>{size[0]:.3f}</radius>'
                    f'<length>{size[1]:.3f}</length></cylinder>')
        if shape == 'sphere':
            return f'<sphere><radius>{size[0]:.3f}</radius></sphere>'
        raise ValueError(shape)

    def visual(self, shape, pos, size, rgb, transp=0.0, rpy=(0, 0, 0), emissive=False):
        extra = f'<transparency>{transp:.2f}</transparency>' if transp else ''
        em = f'<emissive>{_c(rgb)}</emissive>' if emissive else ''
        self.parts.append(
            f'<visual name="{self._id("v")}"><pose>{pos[0]:.3f} {pos[1]:.3f} {pos[2]:.3f} '
            f'{rpy[0]:.3f} {rpy[1]:.3f} {rpy[2]:.3f}</pose>'
            f'<geometry>{self._geom(shape, size)}</geometry>'
            f'<material><ambient>{_c(rgb)}</ambient><diffuse>{_c(rgb)}</diffuse>{em}</material>'
            f'{extra}</visual>')

    def collision(self, shape, pos, size):
        self.parts.append(
            f'<collision name="{self._id("c")}">'
            f'<pose>{pos[0]:.3f} {pos[1]:.3f} {pos[2]:.3f} 0 0 0</pose>'
            f'<geometry>{self._geom(shape, size)}</geometry></collision>')

    def solid(self, pos, size, rgb, **kw):
        """Caixa com visual e colisão iguais."""
        self.visual('box', pos, size, rgb, **kw)
        self.collision('box', pos, size)

    def sdf(self):
        body = '\n        '.join(self.parts)
        return (f'    <model name="{self.name}">\n      <static>true</static>\n'
                f'      <pose>{self.x:.3f} {self.y:.3f} 0 0 0 {self.yaw:.4f}</pose>\n'
                f'      <link name="link">\n        {body}\n      </link>\n    </model>')


def fill_row(m, x, y0, y1, z, depth, palette, h_min, h_max, w_min=0.22, w_max=0.55):
    """Fileira de produtos ao longo de y (de y0 a y1), apoiada em z."""
    y = y0 + 0.03
    while True:
        w = RNG.uniform(w_min, w_max)
        if y + w > y1 - 0.03:
            break
        h = RNG.uniform(h_min, h_max)
        m.visual('box', (x, y + w / 2, z + h / 2), (depth, w - 0.03, h),
                 _vary(RNG.choice(palette)))
        y += w


def fill_row_x(m, x0, x1, y, z, depth, palette, h_min, h_max, w_min=0.15, w_max=0.35):
    """Fileira de produtos ao longo de x."""
    x = x0 + 0.03
    while True:
        w = RNG.uniform(w_min, w_max)
        if x + w > x1 - 0.03:
            break
        h = RNG.uniform(h_min, h_max)
        m.visual('box', (x + w / 2, y, z + h / 2), (w - 0.03, depth, h),
                 _vary(RNG.choice(palette)))
        x += w


# ── móveis ──────────────────────────────────────────────────────────────────

SHELF_Z = [0.12, 0.52, 0.92, 1.32]


def gondola(name, x, yc, secao):
    """Gôndola dupla ao longo de y, com pontas de gôndola nas duas cabeceiras."""
    m = Model(name, x, yc)
    L, W, H = GONDOLA_L, GONDOLA_W, 1.85
    pal = PRODUTOS[secao]
    m.collision('box', (0, 0, H / 2), (W, L, H))
    m.visual('box', (0, 0, 0.06), (W, L, 0.12), METAL_ESCURO)            # rodapé
    m.visual('box', (0, 0, H / 2), (0.04, L, H), METAL)                  # costas
    m.visual('box', (0, 0, H + 0.015), (W, L, 0.03), COR_SECAO[secao])   # tampo
    for yy in [-L / 2 + 0.02, -L / 4, 0, L / 4, L / 2 - 0.02]:          # montantes
        m.visual('box', (0, yy, H / 2), (W, 0.04, H), METAL)
    for s in (-1, 1):
        for z in SHELF_Z:
            m.visual('box', (s * 0.235, 0, z + 0.0125), (0.43, L, 0.025), BRANCO)
            m.visual('box', (s * 0.448, 0, z - 0.005), (0.006, L, 0.045),
                     (0.98, 0.85, 0.1))                                  # etiqueta de preço
            fill_row(m, s * 0.24, -L / 2, L / 2, z + 0.025, 0.34, pal, 0.16, 0.33)
    # pontas de gôndola (expositor de promoção virado para fora)
    for s in (-1, 1):
        yb = s * (L / 2 + 0.225)
        m.collision('box', (0, yb, 0.75), (W, 0.45, 1.5))
        m.visual('box', (0, yb, 0.06), (W, 0.45, 0.12), METAL_ESCURO)
        m.visual('box', (0, yb - s * 0.2, 0.75), (W, 0.04, 1.5), METAL)
        for z in (0.12, 0.55, 0.98):
            m.visual('box', (0, yb + s * 0.02, z + 0.0125), (W, 0.4, 0.025), BRANCO)
            fill_row_x(m, -W / 2, W / 2, yb + s * 0.03, z + 0.025, 0.32, pal, 0.18, 0.36)
        m.visual('box', (0, yb, 1.75), (W, 0.03, 0.35), (0.85, 0.1, 0.1))  # placa "oferta"
        m.visual('box', (0, yb + s * 0.02, 1.75), (W * 0.7, 0.01, 0.15), (0.98, 0.9, 0.2))
    return m


def placa_corredor(name, x, y, rgb):
    """Placa suspensa sobre o corredor (sem colisão)."""
    m = Model(name, x, y)
    m.visual('box', (0, 0, 2.85), (1.4, 0.05, 0.5), rgb)
    m.visual('box', (0, 0.03, 2.85), (1.1, 0.01, 0.3), BRANCO)
    for xx in (-0.6, 0.6):
        m.visual('cyl', (xx, 0, 3.3), (0.008, 0.45), METAL_ESCURO)
    return m


def checkout(name, x, yc, n):
    """Caixa ao longo de y: o cliente passa pelo lado -x, o operador fica no +x."""
    m = Model(name, x, yc)
    L, W = 2.4, 0.7
    m.solid((0, 0, 0.45), (W, L, 0.9), (0.2, 0.3, 0.45))
    m.visual('box', (0, 0.35, 0.905), (0.5, 1.5, 0.02), (0.05, 0.05, 0.05))     # esteira
    m.visual('box', (0, -0.55, 0.91), (0.3, 0.3, 0.02), (0.3, 0.6, 0.3), transp=0.3)  # leitor
    m.visual('box', (0, -1.0, 0.95), (0.6, 0.35, 0.1), METAL)                    # ensacador
    m.visual('box', (0.3, -0.45, 0.95), (0.25, 0.3, 0.1), METAL_ESCURO)          # gaveta
    m.visual('cyl', (0.25, -0.75, 1.1), (0.025, 0.4), METAL_ESCURO)              # monitor
    m.visual('box', (0.25, -0.75, 1.38), (0.04, 0.32, 0.22), METAL_ESCURO)
    m.visual('box', (0.23, -0.75, 1.38), (0.005, 0.28, 0.18), (0.2, 0.5, 0.9), emissive=True)
    m.visual('box', (-0.42, 0.9, 0.6), (0.15, 0.5, 1.1), (0.9, 0.3, 0.1))  # expositor de doces
    m.collision('box', (-0.42, 0.9, 0.6), (0.15, 0.5, 1.1))
    for i in range(4):
        m.visual('box', (-0.42, 0.72 + i * 0.12, 0.35 + 0.22 * (i % 3)), (0.17, 0.1, 0.12),
                 _vary(RNG.choice(PRODUTOS['biscoitos'])))
    # banco do operador
    m.visual('cyl', (0.7, -0.6, 0.3), (0.03, 0.6), METAL_ESCURO)
    m.visual('cyl', (0.7, -0.6, 0.62), (0.2, 0.06), (0.1, 0.1, 0.1))
    m.collision('cyl', (0.7, -0.6, 0.33), (0.2, 0.66))
    # poste com a luz do número do caixa
    m.visual('cyl', (0.3, 1.1, 1.6), (0.03, 1.4), METAL)
    cor = [(0.9, 0.2, 0.2), (0.2, 0.7, 0.3), (0.2, 0.4, 0.9),
           (0.95, 0.7, 0.1), (0.7, 0.3, 0.8), (0.1, 0.7, 0.7)][n % 6]
    m.visual('box', (0.3, 1.1, 2.4), (0.3, 0.3, 0.25), cor, emissive=True)
    return m


def geladeira(name, x0, n_portas, y_face):
    """Geladeiras de portas de vidro encostadas na parede norte (frente em y_face)."""
    w = 1.25
    D, H = 0.8, 2.1
    xc = x0 + n_portas * w / 2
    m = Model(name, xc, y_face + D / 2)
    Ltot = n_portas * w
    m.collision('box', (0, 0, H / 2), (Ltot, D, H))
    m.visual('box', (0, 0.05, H / 2), (Ltot, D - 0.1, H), BRANCO)
    m.visual('box', (0, 0, H - 0.1), (Ltot, D, 0.2), (0.15, 0.35, 0.65))          # faixa superior
    m.visual('box', (0, -D / 2 + 0.05, H - 0.1), (Ltot * 0.9, 0.01, 0.12), BRANCO, emissive=True)
    for i in range(n_portas):
        px = -Ltot / 2 + w * (i + 0.5)
        m.visual('box', (px, -D / 2 + 0.07, 1.05), (w - 0.06, 0.02, 1.75), VIDRO, transp=0.6)
        m.visual('box', (px - w / 2 + 0.015, -D / 2 + 0.06, 1.0), (0.03, 0.04, 1.9), METAL)
        m.visual('cyl', (px + w / 2 - 0.12, -D / 2 + 0.03, 1.1), (0.015, 0.6), METAL)  # puxador
        for z in (0.2, 0.62, 1.04, 1.46):
            m.visual('box', (px, -0.05, z), (w - 0.1, 0.55, 0.02), METAL)
            xx = px - w / 2 + 0.08
            while xx < px + w / 2 - 0.15:
                ww = RNG.uniform(0.08, 0.16)
                h = RNG.uniform(0.18, 0.32)
                m.visual('cyl', (xx + ww / 2, -0.15, z + h / 2), (ww / 2, h),
                         _vary(RNG.choice(PRODUTOS['bebidas'] + [(0.95, 0.95, 0.9)])))
                xx += ww + 0.02
    return m


def ilha_congelados(name, x, y):
    """Freezer horizontal (ilha) com tampa de vidro."""
    m = Model(name, x, y)
    L, W, H = 4.0, 1.0, 0.85
    m.solid((0, 0, H / 2), (L, W, H), BRANCO)
    m.visual('box', (0, 0, 0.1), (L + 0.02, W + 0.02, 0.2), (0.15, 0.35, 0.65))
    m.visual('box', (0, 0, H + 0.04), (L - 0.05, W - 0.05, 0.03), VIDRO, transp=0.55)
    m.visual('box', (0, 0, H + 0.02), (0.03, W, 0.05), METAL)
    pal = [(0.9, 0.9, 0.95), (0.2, 0.5, 0.85), (0.85, 0.2, 0.2), (0.95, 0.75, 0.2),
           (0.3, 0.7, 0.4)]
    for row in (-0.25, 0.25):
        fill_row_x(m, -L / 2 + 0.05, L / 2 - 0.05, row, H - 0.2, 0.4, pal, 0.1, 0.17,
                   w_min=0.2, w_max=0.4)
    return m


def padaria(name):
    """Balcão de vidro da padaria + prateleiras de pães, na parede oeste."""
    yc, L = 2.0, 8.0
    m = Model(name, -14.2, yc)
    pao = [(0.8, 0.55, 0.25), (0.7, 0.45, 0.2), (0.9, 0.7, 0.4), (0.55, 0.35, 0.15)]
    # prateleiras na parede (x local -0.5..-0.75)
    m.solid((-0.5, 0, 0.9), (0.5, L, 1.8), MADEIRA)
    for z in (0.5, 0.95, 1.4):
        for yy in [-L / 2 + 0.4 + i * 0.6 for i in range(int(L / 0.6) - 1)]:
            m.visual('box', (-0.23, yy, z + 0.06), (0.3, 0.45, 0.12), (0.75, 0.6, 0.35))  # cesto
            for k in range(3):
                m.visual('sphere', (-0.23, yy - 0.12 + k * 0.12, z + 0.15), (0.06,),
                         _vary(RNG.choice(pao), 0.05))
    # balcão (x local 0.3..1.2)
    m.solid((0.75, 0, 0.45), (0.9, L, 0.9), (0.45, 0.3, 0.2))
    m.visual('box', (0.6, 0, 1.15), (0.6, L, 0.45), VIDRO, transp=0.6)
    m.visual('box', (0.6, 0, 1.385), (0.62, L, 0.02), METAL)
    for yy in [-L / 2 + 0.3 + i * 0.35 for i in range(int(L / 0.35) - 1)]:
        cor = _vary(RNG.choice(pao + [(0.95, 0.85, 0.75), (0.85, 0.3, 0.35)]), 0.05)
        m.visual('box', (0.6, yy, 0.98), (0.4, 0.22, 0.12), cor)
    # placa
    m.visual('box', (0.0, 0, 2.9), (0.05, 3.0, 0.6), (0.7, 0.4, 0.15))
    m.visual('box', (0.03, 0, 2.9), (0.01, 2.4, 0.35), (0.98, 0.9, 0.7))
    return m


def banca_hortifruti(name, x, y):
    """Banca de caixotes de frutas e verduras (2 x 4 caixotes)."""
    m = Model(name, x, y)
    m.solid((0, 0, 0.35), (1.2, 2.4, 0.7), MADEIRA)
    for i in range(2):
        for j in range(4):
            cx, cy = -0.3 + i * 0.6, -0.9 + j * 0.6
            m.visual('box', (cx, cy, 0.8), (0.55, 0.55, 0.2), (0.7, 0.5, 0.3))
            fruta = RNG.choice(FRUTAS)
            for k in range(4):
                m.visual('sphere', (cx - 0.12 + 0.24 * (k % 2), cy - 0.12 + 0.24 * (k // 2), 0.93),
                         (0.11,), _vary(fruta, 0.06))
    m.visual('cyl', (0, 0, 1.4), (0.02, 1.0), METAL)
    m.visual('box', (0, 0, 1.95), (0.05, 0.9, 0.3), (0.3, 0.65, 0.2))
    return m


def prateleira_hortifruti(name, x, y0, y1):
    """Prateleira inclinada de caixotes na parede leste."""
    L = y1 - y0
    m = Model(name, x, (y0 + y1) / 2)
    m.solid((0, 0, 0.7), (0.6, L, 1.4), MADEIRA)
    for z in (0.45, 0.95):
        yy = -L / 2 + 0.35
        while yy < L / 2 - 0.3:
            m.visual('box', (-0.32, yy, z), (0.06, 0.5, 0.25), (0.7, 0.5, 0.3))
            fruta = RNG.choice(FRUTAS)
            for k in range(3):
                m.visual('sphere', (-0.36, yy - 0.15 + k * 0.15, z + 0.13), (0.07,),
                         _vary(fruta, 0.06))
            yy += 0.6
    m.visual('box', (0.0, 0, 2.9), (0.05, 3.0, 0.6), (0.3, 0.65, 0.2))
    m.visual('box', (-0.03, 0, 2.9), (0.01, 2.4, 0.35), BRANCO)
    return m


def carrinho_visuals(m, cx, cy, yaw=0.0, solid=True):
    """Carrinho de compras (~0.55 x 0.9 x 1.0 m) em coordenadas locais do modelo."""
    c, s = math.cos(yaw), math.sin(yaw)

    def P(dx, dy, z):
        return (cx + dx * c - dy * s, cy + dx * s + dy * c, z)

    rpy = (0, 0, yaw)
    azul = (0.15, 0.35, 0.75)
    m.visual('box', P(0, 0, 0.62), (0.5, 0.8, 0.02), METAL, rpy=rpy)           # fundo da cesta
    for dx in (-0.25, 0.25):
        m.visual('box', P(dx, 0, 0.8), (0.015, 0.8, 0.35), METAL, rpy=rpy, transp=0.3)
    for dy in (-0.4, 0.4):
        m.visual('box', P(0, dy, 0.8), (0.5, 0.015, 0.35), METAL, rpy=rpy, transp=0.3)
    m.visual('box', P(0, -0.48, 1.0), (0.5, 0.04, 0.04), azul, rpy=rpy)       # alça
    m.visual('box', P(0, 0, 0.15), (0.45, 0.75, 0.02), METAL, rpy=rpy)         # bandeja inferior
    for dx in (-0.2, 0.2):
        for dy in (-0.35, 0.35):
            m.visual('cyl', P(dx, dy, 0.38), (0.012, 0.5), METAL)
            m.visual('cyl', P(dx, dy, 0.05), (0.05, 0.03), (0.1, 0.1, 0.1),
                     rpy=(math.pi / 2, 0, yaw))
    if solid:
        m.collision('box', P(0, 0, 0.5), (0.55, 0.9, 1.0))


def carrinho(name, x, y, yaw):
    m = Model(name, x, y, yaw)
    carrinho_visuals(m, 0, 0)
    return m


def fila_carrinhos(name, x, y0, n):
    """Carrinhos encaixados um no outro, ao longo de y."""
    m = Model(name, x, y0)
    for i in range(n):
        carrinho_visuals(m, 0, i * 0.3, solid=False)
    m.collision('box', (0, (n - 1) * 0.15, 0.5), (0.6, 0.9 + (n - 1) * 0.3, 1.0))
    m.visual('box', (0.4, (n - 1) * 0.15, 0.5), (0.05, 0.9 + (n - 1) * 0.3, 1.0),
             (0.2, 0.4, 0.8))                                                 # grade do abrigo
    m.collision('box', (0.4, (n - 1) * 0.15, 0.5), (0.05, 0.9 + (n - 1) * 0.3, 1.0))
    return m


def palete(name, x, y, palette):
    """Palete de promoção com caixas empilhadas."""
    m = Model(name, x, y)
    m.solid((0, 0, 0.075), (1.2, 0.8, 0.15), MADEIRA)
    for i in range(3):
        m.visual('box', (0, -0.3 + i * 0.3, 0.075), (1.22, 0.1, 0.02), (0.45, 0.3, 0.15))
    for lx in (-0.4, 0.0, 0.4):
        for ly in (-0.2, 0.2):
            levels = RNG.randint(2, 4)
            for k in range(levels):
                m.visual('box', (lx, ly, 0.15 + 0.15 + k * 0.3), (0.38, 0.38, 0.28),
                         _vary(RNG.choice(palette), 0.05))
    m.collision('box', (0, 0, 0.75), (1.2, 0.8, 1.2))
    m.visual('cyl', (0.55, 0.35, 1.0), (0.015, 1.7), METAL)
    m.visual('box', (0.55, 0.35, 1.9), (0.5, 0.03, 0.35), (0.9, 0.1, 0.1))
    m.visual('box', (0.55, 0.37, 1.9), (0.35, 0.01, 0.2), (0.98, 0.9, 0.2))
    return m


def pilar(name, x, y):
    m = Model(name, x, y)
    m.solid((0, 0, WALL_H / 2), (0.5, 0.5, WALL_H), (0.85, 0.85, 0.82))
    m.visual('box', (0, 0, 0.15), (0.52, 0.52, 0.3), (0.3, 0.3, 0.32))      # proteção
    m.visual('box', (0, 0, 1.6), (0.51, 0.51, 0.08), (0.95, 0.75, 0.1))     # faixa
    return m


def atendimento(name, x, y):
    m = Model(name, x, y)
    m.solid((0, 0, 0.55), (2.2, 0.7, 1.1), (0.2, 0.3, 0.45))
    m.visual('box', (0, 0, 1.12), (2.3, 0.8, 0.04), MADEIRA)
    m.visual('box', (-0.5, 0.15, 1.3), (0.04, 0.4, 0.28), METAL_ESCURO)
    m.visual('box', (0, 0.3, 2.6), (2.0, 0.05, 0.5), (0.15, 0.35, 0.65))
    m.visual('box', (0, 0.27, 2.6), (1.6, 0.01, 0.3), BRANCO)
    m.visual('box', (0.7, 0, 0.75), (0.4, 0.4, 0.5), (0.95, 0.95, 0.95))    # pilha de cestas
    return m


def porta_entrada(name):
    """Portas automáticas de vidro (fechadas) no vão da parede sul."""
    w = DOOR_X1 - DOOR_X0
    m = Model(name, (DOOR_X0 + DOOR_X1) / 2, Y_MIN)
    m.collision('box', (0, 0, 1.1), (w, 0.1, 2.2))
    m.visual('box', (-w / 4, 0, 1.1), (w / 2 - 0.05, 0.03, 2.2), VIDRO, transp=0.55)
    m.visual('box', (w / 4, 0, 1.1), (w / 2 - 0.05, 0.03, 2.2), VIDRO, transp=0.55)
    for xx in (-w / 2, 0, w / 2):
        m.visual('box', (xx, 0, 1.1), (0.06, 0.08, 2.2), METAL)
    m.visual('box', (0, 0, 2.85), (w, WALL_T, 1.3), (0.15, 0.35, 0.65))     # verga + letreiro
    m.visual('box', (0, 0.12, 2.6), (w * 0.8, 0.01, 0.4), BRANCO, emissive=True)
    m.visual('box', (0, 1.2, 0.006), (w, 2.0, 0.012), (0.25, 0.25, 0.28))   # tapete
    return m


def paredes():
    out = []
    cor = (0.94, 0.93, 0.9)

    def wall(name, x, y, sx, sy):
        m = Model(name, x, y)
        m.solid((0, 0, WALL_H / 2), (sx, sy, WALL_H), cor)
        m.visual('box', (0, 0, 0.1), (sx + 0.01, sy + 0.01, 0.2), (0.35, 0.35, 0.38))  # rodapé
        out.append(m)

    W, H = X_MAX - X_MIN, Y_MAX - Y_MIN
    wall('parede_norte', 0, Y_MAX, W + WALL_T, WALL_T)
    wall('parede_leste', X_MAX, 0, WALL_T, H)
    wall('parede_oeste', X_MIN, 0, WALL_T, H)
    wl = DOOR_X0 - X_MIN
    wall('parede_sul_a', X_MIN + wl / 2, Y_MIN, wl, WALL_T)
    wr = X_MAX - DOOR_X1
    wall('parede_sul_b', DOOR_X1 + wr / 2, Y_MIN, wr, WALL_T)

    # faixas coloridas de seção, no alto das paredes (só visual)
    faixas = Model('faixas_secao', 0, 0)
    faixas.visual('box', (0, Y_MAX - 0.11, 3.0), (W, 0.02, 0.5), (0.15, 0.35, 0.65))     # frios
    faixas.visual('box', (X_MAX - 0.11, 2.5, 3.0), (0.02, 13, 0.5), (0.3, 0.65, 0.2))  # hortifrúti
    faixas.visual('box', (X_MIN + 0.11, 2.0, 3.0), (0.02, 10, 0.5), (0.7, 0.4, 0.15))    # padaria
    out.append(faixas)
    return out


def piso():
    m = Model('piso', 0, 0)
    m.collision('box', (0, 0, -0.05), (40, 30, 0.1))
    m.visual('box', (0, 0, -0.05), (40, 30, 0.1), (0.55, 0.55, 0.55))
    m.visual('box', (0, 0, 0.001), (X_MAX - X_MIN, Y_MAX - Y_MIN, 0.002), (0.86, 0.85, 0.82))
    # faixas de piso de "lajota" (linhas a cada 2 m) para dar textura visual
    for x in range(int(X_MIN) + 2, int(X_MAX), 2):
        m.visual('box', (x, 0, 0.0025), (0.015, Y_MAX - Y_MIN, 0.001), (0.78, 0.77, 0.74))
    for y in range(int(Y_MIN) + 2, int(Y_MAX), 2):
        m.visual('box', (0, y, 0.0025), (X_MAX - X_MIN, 0.015, 0.001), (0.78, 0.77, 0.74))
    # zonas: hortifrúti (verde claro) e caixas (cinza)
    m.visual('box', (10.9, 1.5, 0.003), (8.0, 11.0, 0.002), (0.78, 0.86, 0.74))
    m.visual('box', (-3.5, -8.0, 0.003), (17.0, 3.8, 0.002), (0.72, 0.72, 0.74))
    return m


def luzes():
    out = []
    i = 0
    for x in (-10, 0, 10):
        for y in (-6, 2, 8):
            i += 1
            out.append(f"""    <light type="point" name="luminaria_{i}">
      <pose>{x} {y} 3.3 0 0 0</pose>
      <diffuse>0.55 0.55 0.52 1</diffuse>
      <specular>0.1 0.1 0.1 1</specular>
      <attenuation><range>14</range><constant>0.6</constant><linear>0.05</linear><quadratic>0.005</quadratic></attenuation>
      <cast_shadows>false</cast_shadows>
    </light>""")
    return out


# ── montagem ────────────────────────────────────────────────────────────────

def build():
    models = [piso(), *paredes(), porta_entrada('portas_entrada')]

    for i, x in enumerate(GONDOLA_XS):
        models.append(gondola(f'gondola_{i + 1}', x, GONDOLA_YC, SECOES[i]))
    # placas: uma por corredor, entre as gôndolas e nos corredores laterais
    corredores = [-12.2] + [(a + b) / 2 for a, b in zip(GONDOLA_XS, GONDOLA_XS[1:])] + [6.7]
    for i, x in enumerate(corredores):
        sec = SECOES[min(i, len(SECOES) - 1)]
        models.append(placa_corredor(f'placa_corredor_{i + 1}', x, GONDOLA_YC, COR_SECAO[sec]))

    for i, x in enumerate(CHECKOUT_XS):
        models.append(checkout(f'caixa_{i + 1}', x, CHECKOUT_YC, i))

    # geladeiras (parede norte, frente em y = 9.15) em blocos de 5 portas
    x = -14.5
    for i in range(4):
        n = 5 if i < 3 else 4
        models.append(geladeira(f'geladeiras_{i + 1}', x, n, Y_MAX - WALL_T / 2 - 0.8))
        x += n * 1.25 + 0.3
    for i, x in enumerate((-8.0, -2.0, 4.0)):
        models.append(ilha_congelados(f'ilha_congelados_{i + 1}', x, 7.0))

    models.append(padaria('padaria'))

    for i, (x, y) in enumerate([(9.0, -1.5), (9.0, 1.5), (9.0, 4.5),
                                (12.2, -1.5), (12.2, 1.5), (12.2, 4.5)]):
        models.append(banca_hortifruti(f'banca_hortifruti_{i + 1}', x, y))
    models.append(prateleira_hortifruti('prateleira_hortifruti', X_MAX - WALL_T / 2 - 0.3,
                                        -3.0, 8.0))

    for i, x in enumerate((-6.0, 0.0)):
        models.append(pilar(f'pilar_{i + 1}', x, -4.35))
    models.append(palete('palete_1', -10.5, -4.35, PRODUTOS['bebidas']))
    models.append(palete('palete_2', 4.5, -4.35, PRODUTOS['mercearia']))
    models.append(palete('palete_3', 10.6, -4.35, PRODUTOS['limpeza']))

    models.append(atendimento('atendimento', 7.0, -9.45))
    models.append(fila_carrinhos('carrinhos_entrada', 14.2, -9.3, 8))
    # carrinhos largados nos corredores (obstáculos estáticos para o robô)
    models.append(carrinho('carrinho_corredor_3', -2.45, 2.0, 0.15))
    models.append(carrinho('carrinho_corredor_5', 3.55, 0.0, -0.1))

    body = '\n'.join(m.sdf() for m in models)
    lights = '\n'.join(luzes())
    return f"""<?xml version="1.0" ?>
<!--
  supermercado.sdf — supermercado simulado (30 x 20 m) para o TCC.
  GERADO por sim/scripts/gerar_supermercado.py: não edite à mão, altere o
  script e rode-o de novo. O robô nasce em (-13.2, -4.6), no corredor da
  frente (ver sim/config/mundos/supermercado.yaml).
-->
<sdf version="1.10">
  <world name="supermercado">
    <physics name="1ms" type="ignored">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>
    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
    <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>
    <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
    <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors">
      <render_engine>ogre2</render_engine>
    </plugin>
    <plugin filename="gz-sim-imu-system" name="gz::sim::systems::Imu"/>

    <scene>
      <ambient>0.55 0.55 0.55 1</ambient>
      <background>0.8 0.8 0.85 1</background>
      <shadows>false</shadows>
    </scene>
    <light type="directional" name="sol">
      <pose>0 0 10 0 0 0</pose>
      <diffuse>0.6 0.6 0.6 1</diffuse>
      <specular>0.1 0.1 0.1 1</specular>
      <direction>-0.3 0.2 -1</direction>
      <cast_shadows>false</cast_shadows>
    </light>
{lights}

{body}
  </world>
</sdf>
"""


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[1])
    ap.add_argument('--stdout', action='store_true', help='imprime em vez de gravar')
    a = ap.parse_args()
    sdf = build()
    if a.stdout:
        print(sdf)
        return
    out = Path(__file__).resolve().parents[1] / 'worlds' / 'supermercado.sdf'
    out.write_text(sdf)
    print(f'gravado: {out} ({len(sdf) // 1024} KiB, {sdf.count("<visual")} visuais, '
          f'{sdf.count("<model")} modelos)')


if __name__ == '__main__':
    main()
