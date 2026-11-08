import { useEffect, useState } from 'react';
import { api } from '../api';
import type { ExploreStrategy, SavedMap, Status } from '../types';

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

// Estratégias de exploração autônoma (argumento strategy do explore.launch.py)
const STRATEGIES: { id: ExploreStrategy; label: string; hint: string }[] = [
  {
    id: 'wall_bfs',
    label: 'Seguir parede + BFS',
    hint: 'Segue as paredes e, quando param de revelar mapa, vai à fronteira mais próxima pelo caminho (BFS).',
  },
  {
    id: 'pocket',
    label: 'Desbaste (contorno + offsets)',
    hint:
      'Como o desbaste de uma CNC: identifica no mapa um canto da parede externa (ignorando as prateleiras), ' +
      'contorna as paredes externas até voltar a ele e depois faz passes em offset do caminho já percorrido, ' +
      'nível a nível, até o centro.',
  },
  {
    id: 'frontier',
    label: 'Fronteiras (Nav2)',
    hint: 'Escolhe a fronteira com melhor nota distância × tamanho e vai até ela pelo Nav2, até não sobrar nenhuma.',
  },
];
const STRATEGY_KEY = 'explore-strategy';

function savedStrategy(): ExploreStrategy {
  try {
    const v = localStorage.getItem(STRATEGY_KEY);
    if (STRATEGIES.some((s) => s.id === v)) return v as ExploreStrategy;
  } catch {
    /* sem localStorage: usa o padrão */
  }
  return 'wall_bfs';
}

// Iniciar um mapeamento novo, abrir um mapa salvo ou parar a sessão.
export default function SessionPanel({ status, notify }: Props) {
  const [maps, setMaps] = useState<SavedMap[]>([]);
  const [newName, setNewName] = useState('');
  const [selected, setSelected] = useState('');
  const [strategy, setStrategyState] = useState<ExploreStrategy>(savedStrategy);
  const setStrategy = (s: ExploreStrategy) => {
    setStrategyState(s);
    try {
      localStorage.setItem(STRATEGY_KEY, s);
    } catch {
      /* ignora */
    }
  };
  const strategyInfo = STRATEGIES.find((s) => s.id === strategy)!;
  const strategySelect = (
    <label className="field">
      <span>Estratégia de exploração</span>
      <select value={strategy} onChange={(e) => setStrategy(e.target.value as ExploreStrategy)}>
        {STRATEGIES.map((s) => (
          <option key={s.id} value={s.id}>
            {s.label}
          </option>
        ))}
      </select>
    </label>
  );
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
      await api(`/session/${kind}`, { name, explore, strategy });
      notify(
        kind === 'navigation'
          ? `Mapa "${name}" aberto`
          : explore
            ? `Mapeamento "${name}" iniciado; a exploração (${strategyInfo.label}) começa quando o robô estiver pronto`
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
      await api(`/explore/${exploring ? 'stop' : 'start'}`, exploring ? {} : { strategy });
      notify(exploring ? 'Exploração autônoma parada' : `Exploração autônoma iniciada (${strategyInfo.label})`);
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
              {exploring ? (
                <p className="hint">
                  Estratégia: <strong>{STRATEGIES.find((s) => s.id === status?.explore?.strategy)?.label ?? '—'}</strong>
                </p>
              ) : (
                strategySelect
              )}
              <button className={`wide ${exploring ? '' : 'primary'}`} onClick={toggleExplore}>
                {exploring ? 'Parar exploração autônoma' : 'Iniciar exploração autônoma'}
              </button>
              {!exploring && (
                <p className="hint">
                  {strategyInfo.hint} Qualquer comando manual (destino, setas) interrompe a exploração.
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
          {strategySelect}
          <button className="primary wide" disabled={!newName.trim()} onClick={() => start('mapping', newName.trim(), true)}>
            Mapear com exploração autônoma
          </button>
          <p className="hint">
            No mapeamento, conduza o robô pelo ambiente (clicando no mapa ou com as setas) ou use a exploração
            autônoma. {strategyInfo.hint} Ao concluir, o mapa é salvo.
          </p>
        </>
      )}
    </section>
  );
}
