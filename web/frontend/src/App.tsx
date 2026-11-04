import { useCallback, useEffect, useState } from 'react';
import { api } from './api';
import GoalCard from './components/GoalCard';
import HistoryPanel from './components/HistoryPanel';
import MapView, { type MapTool } from './components/MapView';
import PointsPanel from './components/PointsPanel';
import RoutesPanel from './components/RoutesPanel';
import SessionPanel from './components/SessionPanel';
import StatusBar from './components/StatusBar';
import Teleop from './components/Teleop';
import type { MapData, Point, Route } from './types';
import { useRobot } from './useRobot';

type Tab = 'operar' | 'destinos' | 'rotas' | 'historico';
const TABS: { id: Tab; label: string }[] = [
  { id: 'operar', label: 'Operar' },
  { id: 'destinos', label: 'Destinos' },
  { id: 'rotas', label: 'Rotas' },
  { id: 'historico', label: 'Histórico' },
];

const TOOLS: { id: MapTool; label: string; hint: string }[] = [
  { id: 'move', label: 'Mover', hint: 'Arraste para mover o mapa; use a roda do mouse para zoom.' },
  { id: 'goal', label: 'Ir para', hint: 'Toque no destino. Arraste a partir dele para escolher a direção final.' },
  { id: 'pose', label: 'Corrigir posição', hint: 'Toque onde o robô realmente está e arraste para a direção dele.' },
  { id: 'point', label: 'Novo destino', hint: 'Toque no local do destino e dê um nome a ele.' },
];

interface Toast {
  id: number;
  text: string;
  error: boolean;
}

export default function App() {
  const robot = useRobot();
  const { status, mapMeta } = robot;
  const [tab, setTab] = useState<Tab>('operar');
  const [tool, setTool] = useState<MapTool>('goal');
  const [showScan, setShowScan] = useState(true);
  const [map, setMap] = useState<MapData | null>(null);
  const [points, setPoints] = useState<Point[]>([]);
  const [routes, setRoutes] = useState<Route[]>([]);
  const [toasts, setToasts] = useState<Toast[]>([]);

  const mode = status?.mode ?? 'idle';
  const active = mode === 'mapping' || mode === 'navigation';
  const navigation = mode === 'navigation';

  const notify = useCallback((text: string, error = false) => {
    const id = Date.now() + Math.random();
    setToasts((t) => [...t, { id, text, error }]);
    setTimeout(() => setToasts((t) => t.filter((x) => x.id !== id)), error ? 6000 : 3000);
  }, []);

  // Imagem do mapa: buscada quando o backend avisa que o mapa mudou
  useEffect(() => {
    if (!mapMeta) {
      setMap(null);
      return;
    }
    api<MapData>('/map').then(setMap).catch(() => {});
  }, [mapMeta]);

  const reloadLists = useCallback(() => {
    api<Point[]>('/points').then(setPoints).catch(() => setPoints([]));
    api<Route[]>('/routes').then(setRoutes).catch(() => setRoutes([]));
  }, []);

  useEffect(reloadLists, [status?.map, reloadLists]);

  const onPick = async (picked: Exclude<MapTool, 'move'>, x: number, y: number, theta: number) => {
    try {
      if (picked === 'goal') {
        await api('/goal', { x, y, theta });
      } else if (picked === 'pose') {
        await api('/initial_pose', { x, y, theta });
        notify('Posição informada ao robô');
      } else {
        const name = prompt('Nome do destino:')?.trim();
        if (!name) return;
        await api('/points', { name, x, y, theta });
        notify(`Destino "${name}" salvo`);
        reloadLists();
        setTool('goal');
      }
    } catch (e) {
      notify((e as Error).message, true);
    }
  };

  const availableTools = TOOLS.filter((t) => t.id === 'move' || (active && (t.id === 'goal' || navigation)));
  const currentTool = availableTools.some((t) => t.id === tool) ? tool : 'move';
  const target = robot.status?.mission ? robot.goal?.target ?? null : null;

  return (
    <div className="app">
      <StatusBar connected={robot.connected} status={status} pose={robot.pose} />
      <main className="layout">
        <div className="map-column">
          <div className="toolbar">
            {availableTools.map((t) => (
              <button
                key={t.id}
                className={currentTool === t.id ? 'selected' : ''}
                onClick={() => setTool(t.id)}
                title={t.hint}
              >
                {t.label}
              </button>
            ))}
            <label className="toggle">
              <input type="checkbox" checked={showScan} onChange={(e) => setShowScan(e.target.checked)} />
              Laser
            </label>
          </div>
          <p className="tool-hint">{TOOLS.find((t) => t.id === currentTool)?.hint}</p>
          <MapView
            map={map}
            pose={robot.pose}
            path={robot.path}
            scan={robot.scan}
            points={navigation ? points : []}
            target={target}
            tool={currentTool}
            showScan={showScan}
            onPick={onPick}
          />
        </div>

        <aside className="sidebar">
          <nav className="tabs">
            {TABS.map((t) => (
              <button key={t.id} className={tab === t.id ? 'selected' : ''} onClick={() => setTab(t.id)}>
                {t.label}
              </button>
            ))}
          </nav>
          <GoalCard status={status} goal={robot.goal} notify={notify} />
          {tab === 'operar' && (
            <>
              <SessionPanel status={status} notify={notify} />
              <Teleop enabled={active} />
            </>
          )}
          {tab === 'destinos' && (
            <PointsPanel
              points={points}
              enabled={navigation}
              onChange={reloadLists}
              onAddByClick={() => setTool('point')}
              notify={notify}
            />
          )}
          {tab === 'rotas' && (
            <RoutesPanel routes={routes} points={points} enabled={navigation} onChange={reloadLists} notify={notify} />
          )}
          {tab === 'historico' && <HistoryPanel version={robot.historyVersion} />}
        </aside>
      </main>

      <div className="toasts" role="status">
        {toasts.map((t) => (
          <div key={t.id} className={`toast ${t.error ? 'error' : ''}`}>
            {t.text}
          </div>
        ))}
      </div>
    </div>
  );
}
