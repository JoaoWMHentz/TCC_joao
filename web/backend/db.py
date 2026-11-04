"""
db.py — persistência em PostgreSQL.

Tabelas:
  points          destinos nomeados de um mapa ("caixa", "mesa 2"...)
  routes          rotas = sequência ordenada de destinos
  navigation_log  histórico de cada navegação (base das métricas do TCC)
"""

from __future__ import annotations

from contextlib import contextmanager

import psycopg2
from psycopg2.extras import Json, RealDictCursor

import config

SCHEMA = """
CREATE TABLE IF NOT EXISTS points (
    id         SERIAL PRIMARY KEY,
    map        TEXT NOT NULL,
    name       TEXT NOT NULL,
    x          DOUBLE PRECISION NOT NULL,
    y          DOUBLE PRECISION NOT NULL,
    theta      DOUBLE PRECISION NOT NULL DEFAULT 0,
    created_at TIMESTAMPTZ NOT NULL DEFAULT NOW(),
    UNIQUE (map, name)
);

CREATE TABLE IF NOT EXISTS routes (
    id         SERIAL PRIMARY KEY,
    map        TEXT NOT NULL,
    name       TEXT NOT NULL,
    point_ids  INTEGER[] NOT NULL,
    created_at TIMESTAMPTZ NOT NULL DEFAULT NOW(),
    UNIQUE (map, name)
);

CREATE TABLE IF NOT EXISTS navigation_log (
    id              SERIAL PRIMARY KEY,
    map             TEXT,
    kind            TEXT NOT NULL,          -- goal | point | route
    label           TEXT,
    target          JSONB NOT NULL,         -- destino(s) pedidos
    started_at      TIMESTAMPTZ NOT NULL DEFAULT NOW(),
    finished_at     TIMESTAMPTZ,
    result          TEXT,                   -- succeeded | aborted | canceled | rejected
    duration_s      DOUBLE PRECISION,
    distance_m      DOUBLE PRECISION,       -- percorrido (pose estimada)
    recoveries      INTEGER,                -- comportamentos de recuperação do Nav2
    min_clearance_m DOUBLE PRECISION,       -- menor distância do LiDAR a obstáculos
    final_pose      JSONB,
    final_error_m   DOUBLE PRECISION        -- distância final ao destino
);
"""


@contextmanager
def cursor():
    conn = psycopg2.connect(config.DATABASE_URL, cursor_factory=RealDictCursor)
    try:
        with conn:
            with conn.cursor() as cur:
                yield cur
    finally:
        conn.close()


def init():
    with cursor() as cur:
        cur.execute(SCHEMA)


# ── Destinos ─────────────────────────────────────────────────────────────────

def list_points(map_name: str) -> list[dict]:
    with cursor() as cur:
        cur.execute('SELECT id, name, x, y, theta FROM points WHERE map = %s ORDER BY name',
                    (map_name,))
        return cur.fetchall()


def get_point(point_id: int) -> dict | None:
    with cursor() as cur:
        cur.execute('SELECT id, map, name, x, y, theta FROM points WHERE id = %s', (point_id,))
        return cur.fetchone()


def create_point(map_name: str, name: str, x: float, y: float, theta: float) -> int:
    with cursor() as cur:
        cur.execute('INSERT INTO points (map, name, x, y, theta) VALUES (%s, %s, %s, %s, %s) '
                    'RETURNING id', (map_name, name, x, y, theta))
        return cur.fetchone()['id']


def delete_point(point_id: int) -> bool:
    with cursor() as cur:
        cur.execute('DELETE FROM points WHERE id = %s', (point_id,))
        if cur.rowcount == 0:
            return False
        # Remove o destino das rotas que o usavam
        cur.execute('UPDATE routes SET point_ids = array_remove(point_ids, %s)', (point_id,))
        return True


# ── Rotas ────────────────────────────────────────────────────────────────────

def list_routes(map_name: str) -> list[dict]:
    with cursor() as cur:
        cur.execute('SELECT id, name, point_ids FROM routes WHERE map = %s ORDER BY name',
                    (map_name,))
        return cur.fetchall()


def get_route(route_id: int) -> dict | None:
    with cursor() as cur:
        cur.execute('SELECT id, map, name, point_ids FROM routes WHERE id = %s', (route_id,))
        return cur.fetchone()


def create_route(map_name: str, name: str, point_ids: list[int]) -> int:
    with cursor() as cur:
        cur.execute('INSERT INTO routes (map, name, point_ids) VALUES (%s, %s, %s) RETURNING id',
                    (map_name, name, point_ids))
        return cur.fetchone()['id']


def delete_route(route_id: int) -> bool:
    with cursor() as cur:
        cur.execute('DELETE FROM routes WHERE id = %s', (route_id,))
        return cur.rowcount > 0


def route_points(route: dict) -> list[dict]:
    """Destinos da rota, na ordem da rota."""
    with cursor() as cur:
        cur.execute('SELECT id, name, x, y, theta FROM points WHERE id = ANY(%s)',
                    (route['point_ids'],))
        by_id = {p['id']: p for p in cur.fetchall()}
    return [by_id[i] for i in route['point_ids'] if i in by_id]


# ── Histórico de navegação ───────────────────────────────────────────────────

def log_start(map_name: str | None, kind: str, label: str, target) -> int:
    with cursor() as cur:
        cur.execute('INSERT INTO navigation_log (map, kind, label, target) '
                    'VALUES (%s, %s, %s, %s) RETURNING id',
                    (map_name, kind, label, Json(target)))
        return cur.fetchone()['id']


def log_finish(log_id: int, result: str, duration_s: float, distance_m: float,
               recoveries: int, min_clearance_m: float | None,
               final_pose: dict | None, final_error_m: float | None):
    with cursor() as cur:
        cur.execute(
            'UPDATE navigation_log SET finished_at = NOW(), result = %s, duration_s = %s, '
            'distance_m = %s, recoveries = %s, min_clearance_m = %s, final_pose = %s, '
            'final_error_m = %s WHERE id = %s',
            (result, duration_s, distance_m, recoveries, min_clearance_m,
             Json(final_pose) if final_pose else None, final_error_m, log_id))


def list_history(limit: int = 50) -> list[dict]:
    with cursor() as cur:
        cur.execute('SELECT * FROM navigation_log ORDER BY id DESC LIMIT %s', (limit,))
        return cur.fetchall()


def history_summary() -> dict:
    """Indicadores agregados (todas as navegações concluídas)."""
    with cursor() as cur:
        cur.execute("""
            SELECT COUNT(*)                                             AS total,
                   COUNT(*) FILTER (WHERE result = 'succeeded')         AS succeeded,
                   AVG(duration_s) FILTER (WHERE result = 'succeeded')  AS avg_duration_s,
                   SUM(distance_m)                                      AS total_distance_m,
                   SUM(recoveries)                                      AS total_recoveries,
                   MIN(min_clearance_m)                                 AS min_clearance_m,
                   AVG(final_error_m) FILTER (WHERE result = 'succeeded') AS avg_final_error_m
            FROM navigation_log WHERE result IS NOT NULL
        """)
        return cur.fetchone()
