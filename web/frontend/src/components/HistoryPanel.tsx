import { useEffect, useState } from 'react';
import { api } from '../api';
import type { HistoryEntry, HistorySummary } from '../types';

const RESULT: Record<string, string> = {
  succeeded: 'Concluída',
  aborted: 'Falhou',
  canceled: 'Cancelada',
  rejected: 'Recusada',
};

const fmt = (v: number | null | undefined, digits = 1, unit = '') => (v == null ? '—' : `${v.toFixed(digits)}${unit}`);

// Histórico das navegações e indicadores agregados.
export default function HistoryPanel({ version }: { version: number }) {
  const [entries, setEntries] = useState<HistoryEntry[]>([]);
  const [summary, setSummary] = useState<HistorySummary | null>(null);

  useEffect(() => {
    api<HistoryEntry[]>('/history?limit=30').then(setEntries).catch(() => {});
    api<HistorySummary>('/history/summary').then(setSummary).catch(() => {});
  }, [version]);

  const rate = summary && summary.total ? (100 * summary.succeeded) / summary.total : null;

  return (
    <section className="panel">
      <h2>Histórico</h2>
      {summary && summary.total > 0 && (
        <div className="stats">
          <div>
            <span>{fmt(rate, 0, '%')}</span>
            <small>sucesso ({summary.succeeded}/{summary.total})</small>
          </div>
          <div>
            <span>{fmt(summary.total_distance_m, 0, ' m')}</span>
            <small>percorridos</small>
          </div>
          <div>
            <span>{fmt(summary.min_clearance_m, 2, ' m')}</span>
            <small>menor folga</small>
          </div>
        </div>
      )}
      <table className="history">
        <thead>
          <tr>
            <th>Navegação</th>
            <th>Resultado</th>
            <th>Tempo</th>
            <th>Dist.</th>
          </tr>
        </thead>
        <tbody>
          {entries.map((h) => (
            <tr key={h.id}>
              <td>{h.label}</td>
              <td className={`result-${h.result ?? 'running'}`}>{h.result ? RESULT[h.result] ?? h.result : 'em curso'}</td>
              <td>{fmt(h.duration_s, 0, ' s')}</td>
              <td>{fmt(h.distance_m, 1, ' m')}</td>
            </tr>
          ))}
        </tbody>
      </table>
      {entries.length === 0 && <p className="hint">Nenhuma navegação registrada ainda.</p>}
    </section>
  );
}
