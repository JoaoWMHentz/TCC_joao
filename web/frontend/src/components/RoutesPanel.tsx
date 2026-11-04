import { useState } from 'react';
import { api } from '../api';
import type { Point, Route } from '../types';

interface Props {
  routes: Route[];
  points: Point[];
  enabled: boolean;
  onChange: () => void;
  notify: (msg: string, error?: boolean) => void;
}

// Rotas: sequências de destinos percorridas em ordem.
export default function RoutesPanel({ routes, points, enabled, onChange, notify }: Props) {
  const [draft, setDraft] = useState<number[]>([]);
  const [name, setName] = useState('');
  const byId = new Map(points.map((p) => [p.id, p]));

  const run = async (fn: () => Promise<unknown>, ok?: string) => {
    try {
      await fn();
      if (ok) notify(ok);
      onChange();
      return true;
    } catch (e) {
      notify((e as Error).message, true);
      return false;
    }
  };

  const save = async () => {
    if (await run(() => api('/routes', { name: name.trim(), point_ids: draft }), `Rota "${name.trim()}" criada`)) {
      setDraft([]);
      setName('');
    }
  };

  const describe = (ids: number[]) => ids.map((id) => byId.get(id)?.name ?? '?').join(' → ');

  return (
    <section className="panel">
      <h2>Rotas</h2>
      {!enabled && <p className="hint">Abra um mapa salvo para usar rotas.</p>}
      <ul className="item-list">
        {routes.map((r) => (
          <li key={r.id}>
            <span className="item-name">
              {r.name}
              <small>{describe(r.point_ids)}</small>
            </span>
            <button className="primary" disabled={!enabled} onClick={() => run(() => api(`/routes/${r.id}/run`, {}))}>
              Iniciar
            </button>
            <button
              className="ghost"
              aria-label={`Apagar ${r.name}`}
              onClick={() => confirm(`Apagar a rota "${r.name}"?`) && run(() => api(`/routes/${r.id}`, undefined, 'DELETE'))}
            >
              ✕
            </button>
          </li>
        ))}
      </ul>

      {enabled && points.length >= 2 && (
        <div className="route-builder">
          <span className="field-label">Nova rota — toque nos destinos na ordem:</span>
          <div className="chips">
            {points.map((p) => (
              <button key={p.id} className="chip" onClick={() => setDraft([...draft, p.id])}>
                + {p.name}
              </button>
            ))}
          </div>
          {draft.length > 0 && (
            <>
              <p className="route-draft">
                {describe(draft)}{' '}
                <button className="ghost" onClick={() => setDraft(draft.slice(0, -1))}>
                  desfazer
                </button>
              </p>
              <div className="row">
                <input value={name} placeholder="Nome da rota" onChange={(e) => setName(e.target.value)} />
                <button className="primary" disabled={!name.trim() || draft.length < 2} onClick={save}>
                  Salvar
                </button>
              </div>
            </>
          )}
        </div>
      )}
    </section>
  );
}
