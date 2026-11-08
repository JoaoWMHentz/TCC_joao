import { useEffect, useState } from 'react';
import { api } from '../api';
import type { ExploreLogEntry } from '../types';

const time = (t: number) =>
  new Date(t * 1000).toLocaleTimeString('pt-BR', { hour: '2-digit', minute: '2-digit', second: '2-digit' });

// Decisões do explorador autônomo (seguir parede, ir a uma fronteira, descartar
// um destino, iniciar um passe...), da mais nova para a mais antiga.
// Atualiza a cada 1 s enquanto a exploração roda e uma última vez quando ela para.
export default function ExploreLog({ running }: { running: boolean }) {
  const [entries, setEntries] = useState<ExploreLogEntry[]>([]);

  useEffect(() => {
    const load = () => api<ExploreLogEntry[]>('/explore/log').then(setEntries).catch(() => {});
    load();
    if (!running) return;
    const timer = window.setInterval(load, 1000);
    return () => clearInterval(timer);
  }, [running]);

  if (!running && entries.length === 0) return null;

  return (
    <section className="panel">
      <h2>
        Decisões da exploração <span className="count">{entries.length}</span>
      </h2>
      <ol className="explore-log" role="log" aria-live="polite" aria-label="Decisões da exploração autônoma">
        {entries.length === 0 && <li className="hint">Aguardando o explorador...</li>}
        {[...entries].reverse().map((e, i) => (
          <li key={`${e.t}-${i}`} className={`level-${e.level}`}>
            <time>{time(e.t)}</time>
            <span>{e.text}</span>
          </li>
        ))}
      </ol>
    </section>
  );
}
