import { useEffect, useState } from 'react';
import { api } from '../api';
import type { SavedMap, Status } from '../types';

interface Props {
  status: Status | null;
  notify: (msg: string, error?: boolean) => void;
}

const MODE_TEXT: Record<string, string> = {
  idle: 'Robô parado',
  starting: 'Iniciando...',
  mapping: 'Mapeando',
  navigation: 'Navegando',
  stopping: 'Parando (salvando o mapa)...',
};

// Iniciar um mapeamento novo, abrir um mapa salvo ou parar a sessão.
export default function SessionPanel({ status, notify }: Props) {
  const [maps, setMaps] = useState<SavedMap[]>([]);
  const [newName, setNewName] = useState('');
  const [selected, setSelected] = useState('');
  const mode = status?.mode ?? 'idle';
  const busy = mode !== 'idle';

  const loadMaps = () =>
    api<SavedMap[]>('/maps')
      .then((list) => {
        setMaps(list);
        setSelected((cur) => (list.some((m) => m.name === cur) ? cur : list[0]?.name || ''));
      })
      .catch(() => {});

  useEffect(() => {
    if (mode === 'idle') loadMaps();
  }, [mode]);

  const start = async (kind: 'mapping' | 'navigation', name: string, explore = false) => {
    try {
      await api(`/session/${kind}`, { name, explore });
      notify(
        kind === 'navigation'
          ? `Mapa "${name}" aberto`
          : explore
            ? `Mapeamento "${name}" iniciado; a exploração autônoma começa quando o robô estiver pronto`
            : `Mapeamento "${name}" iniciado`,
      );
      setNewName('');
    } catch (e) {
      notify((e as Error).message, true);
    }
  };

  const removeMap = async (name: string) => {
    if (!window.confirm(`Excluir o mapa "${name}"? Os destinos e rotas dele também serão apagados.`)) return;
    try {
      await api(`/maps/${encodeURIComponent(name)}`, undefined, 'DELETE');
      notify(`Mapa "${name}" excluído`);
      loadMaps();
    } catch (e) {
      notify((e as Error).message, true);
    }
  };

  const exploring = status?.explore?.running ?? false;
  const toggleExplore = async () => {
    try {
      await api(`/explore/${exploring ? 'stop' : 'start'}`, {});
      notify(exploring ? 'Exploração autônoma parada' : 'Exploração autônoma iniciada');
    } catch (e) {
      notify((e as Error).message, true);
    }
  };

  const stop = async () => {
    try {
      await api('/session/stop', {});
      notify(mode === 'mapping' ? 'Mapa salvo' : 'Sessão encerrada');
    } catch (e) {
      notify((e as Error).message, true);
    }
  };

  return (
    <section className="panel">
      <h2>Sessão</h2>
      <p className={`session-state mode-${mode}`}>
        {MODE_TEXT[mode]}
        {status?.map && <strong> · {status.map}</strong>}
      </p>
      {status?.error && <p className="error-text">{status.error}</p>}

      {busy ? (
        <>
          {mode === 'mapping' && (
            <>
              <button className={`wide ${exploring ? '' : 'primary'}`} onClick={toggleExplore}>
                {exploring ? 'Parar exploração autônoma' : 'Iniciar exploração autônoma'}
              </button>
              {status?.explore?.message && <p className="hint">{status.explore.message}</p>}
              {!exploring && (
                <p className="hint">
                  O robô segue as paredes e vai sozinho até o que ainda não foi mapeado, até cobrir o ambiente. Qualquer
                  comando manual (destino, setas) interrompe a exploração.
                </p>
              )}
            </>
          )}
          <button className="danger wide" onClick={stop} disabled={mode === 'stopping'}>
            {mode === 'mapping' ? 'Concluir e salvar mapa' : 'Parar'}
          </button>
        </>
      ) : (
        <>
          <label className="field">
            <span>Abrir mapa salvo</span>
            <div className="row">
              <select value={selected} onChange={(e) => setSelected(e.target.value)} disabled={!maps.length}>
                {maps.length === 0 && <option value="">Nenhum mapa salvo</option>}
                {maps.map((m) => (
                  <option key={m.name} value={m.name}>
                    {m.name} ({m.size_mb} MB)
                  </option>
                ))}
              </select>
              <button className="primary" disabled={!selected} onClick={() => start('navigation', selected)}>
                Abrir
              </button>
              <button className="danger" disabled={!selected} onClick={() => removeMap(selected)} title="Excluir mapa">
                Excluir
              </button>
            </div>
          </label>
          <label className="field">
            <span>Criar mapa novo</span>
            <div className="row">
              <input
                value={newName}
                placeholder="ex.: loja-centro"
                onChange={(e) => setNewName(e.target.value)}
                onKeyDown={(e) => e.key === 'Enter' && newName.trim() && start('mapping', newName.trim())}
              />
              <button disabled={!newName.trim()} onClick={() => start('mapping', newName.trim())}>
                Mapear
              </button>
            </div>
          </label>
          <button className="primary wide" disabled={!newName.trim()} onClick={() => start('mapping', newName.trim(), true)}>
            Mapear com exploração autônoma
          </button>
          <p className="hint">
            No mapeamento, conduza o robô pelo ambiente (clicando no mapa ou com as setas) ou use a exploração
            autônoma: ele segue as paredes e vai sozinho até o que ainda não foi mapeado. Ao concluir, o mapa é salvo.
          </p>
        </>
      )}
    </section>
  );
}
