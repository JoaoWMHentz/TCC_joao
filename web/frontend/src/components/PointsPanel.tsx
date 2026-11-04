import { useState } from 'react';
import { api } from '../api';
import type { Point } from '../types';

interface Props {
  points: Point[];
  enabled: boolean;
  onChange: () => void;
  onAddByClick: () => void;
  notify: (msg: string, error?: boolean) => void;
}

// Destinos nomeados do mapa aberto: ir até eles, criar e apagar.
export default function PointsPanel({ points, enabled, onChange, onAddByClick, notify }: Props) {
  const [name, setName] = useState('');

  const run = async (fn: () => Promise<unknown>, ok?: string) => {
    try {
      await fn();
      if (ok) notify(ok);
      onChange();
    } catch (e) {
      notify((e as Error).message, true);
    }
  };

  const saveHere = () =>
    run(() => api('/points', { name: name.trim(), here: true }), `Destino "${name.trim()}" salvo`).then(() =>
      setName(''),
    );

  return (
    <section className="panel">
      <h2>Destinos</h2>
      {!enabled && <p className="hint">Abra um mapa salvo para usar destinos.</p>}
      <ul className="item-list">
        {points.map((p) => (
          <li key={p.id}>
            <span className="item-name">{p.name}</span>
            <button className="primary" disabled={!enabled} onClick={() => run(() => api(`/points/${p.id}/go`, {}))}>
              Ir
            </button>
            <button
              className="ghost"
              aria-label={`Apagar ${p.name}`}
              onClick={() => confirm(`Apagar o destino "${p.name}"?`) && run(() => api(`/points/${p.id}`, undefined, 'DELETE'))}
            >
              ✕
            </button>
          </li>
        ))}
      </ul>
      {enabled && (
        <>
          <div className="row">
            <input value={name} placeholder="Nome (ex.: caixa)" onChange={(e) => setName(e.target.value)} />
            <button disabled={!name.trim()} onClick={saveHere} title="Salvar a posição atual do robô">
              Aqui
            </button>
          </div>
          <button className="wide" onClick={onAddByClick}>
            Marcar destino no mapa
          </button>
        </>
      )}
    </section>
  );
}
