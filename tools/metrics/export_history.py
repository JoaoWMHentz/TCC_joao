#!/usr/bin/env python3
"""
export_history.py — exporta o histórico de navegação (PostgreSQL) para CSV e
imprime os indicadores usados na avaliação:

  - taxa de navegações concluídas (meta do TCC: > 90%)
  - recuperações do Nav2 (replanejamentos/manobras de desvio)
  - menor folga até obstáculos (colisão quando ~0; corpo de raio 0.245 m)
  - erro final em relação ao destino

Uso:
  python3 tools/metrics/export_history.py --csv historico.csv
  python3 tools/metrics/export_history.py --map loja --since 2026-11-01
"""

import argparse
import csv
import os

import psycopg2
from psycopg2.extras import RealDictCursor

COLLISION_CLEARANCE_M = 0.02


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--db', default=os.environ.get('DATABASE_URL', 'postgresql://tcc:tcc@localhost:5433/tcc'))
    ap.add_argument('--map', help='filtra por mapa')
    ap.add_argument('--since', help='data inicial (AAAA-MM-DD)')
    ap.add_argument('--csv', help='arquivo de saída')
    args = ap.parse_args()

    where, params = ['result IS NOT NULL'], []
    if args.map:
        where.append('map = %s')
        params.append(args.map)
    if args.since:
        where.append('started_at >= %s')
        params.append(args.since)

    with psycopg2.connect(args.db, cursor_factory=RealDictCursor) as conn, conn.cursor() as cur:
        cur.execute(f'SELECT id, map, kind, label, started_at, result, duration_s, distance_m, '
                    f'recoveries, min_clearance_m, final_error_m FROM navigation_log '
                    f'WHERE {" AND ".join(where)} ORDER BY id', params)
        rows = cur.fetchall()

    if not rows:
        print('Nenhuma navegação encontrada.')
        return

    done = [r for r in rows if r['result'] == 'succeeded']
    clear = [r['min_clearance_m'] for r in rows if r['min_clearance_m'] is not None]
    errors = [r['final_error_m'] for r in done if r['final_error_m'] is not None]
    touches = sum(1 for c in clear if c <= COLLISION_CLEARANCE_M)
    print(f'navegações:            {len(rows)}')
    print(f'concluídas:            {len(done)} ({100 * len(done) / len(rows):.1f}%)')
    print(f'distância total:       {sum(r["distance_m"] or 0 for r in rows):.1f} m')
    print(f'recuperações (total):  {sum(r["recoveries"] or 0 for r in rows)}')
    if clear:
        print(f'menor folga:           {min(clear):.3f} m '
              f'(possíveis contatos, folga <= {COLLISION_CLEARANCE_M} m: {touches})')
    if errors:
        print(f'erro final médio:      {sum(errors) / len(errors):.3f} m')

    if args.csv:
        with open(args.csv, 'w', newline='') as f:
            writer = csv.DictWriter(f, fieldnames=list(rows[0]))
            writer.writeheader()
            writer.writerows(rows)
        print(f'gravado em {args.csv}')


if __name__ == '__main__':
    main()
