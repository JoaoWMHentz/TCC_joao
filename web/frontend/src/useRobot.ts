import { useEffect, useState } from 'react';
import { io } from 'socket.io-client';
import type { GoalStatus, MapMeta, Pose, Status, XY } from './types';

export const socket = io({ transports: ['websocket', 'polling'] });

export interface RobotState {
  connected: boolean;
  status: Status | null;
  pose: Pose;
  mapMeta: MapMeta | null;
  path: XY[];
  scan: XY[];
  goal: GoalStatus | null;
  historyVersion: number;
}

const NO_POSE: Pose = { x: 0, y: 0, theta: 0, available: false };

// Estado em tempo real do robô, alimentado pelos eventos Socket.IO do backend.
export function useRobot(): RobotState {
  const [connected, setConnected] = useState(socket.connected);
  const [status, setStatus] = useState<Status | null>(null);
  const [pose, setPose] = useState<Pose>(NO_POSE);
  const [mapMeta, setMapMeta] = useState<MapMeta | null>(null);
  const [path, setPath] = useState<XY[]>([]);
  const [scan, setScan] = useState<XY[]>([]);
  const [goal, setGoal] = useState<GoalStatus | null>(null);
  const [historyVersion, setHistoryVersion] = useState(0);

  useEffect(() => {
    // Cada evento do backend atualiza um pedaço do estado
    // eslint-disable-next-line @typescript-eslint/no-explicit-any
    type Handler = (data: any) => void;
    const handlers: Record<string, Handler> = {
      connect: () => setConnected(true),
      disconnect: () => setConnected(false),
      status_update: (s: Status) => {
        setStatus(s);
        if (s.mode === 'idle') {
          setMapMeta(null);
          setPath([]);
          setScan([]);
        }
      },
      pose_update: (p: Pose) => setPose(p),
      map_update: (m: MapMeta) => setMapMeta(m),
      path_update: (d: { points: XY[] }) => setPath(d.points),
      scan_update: (d: { points: XY[] }) => setScan(d.points),
      goal_status: (g: GoalStatus) => setGoal(g),
      history_update: () => setHistoryVersion((v) => v + 1),
    };
    for (const [event, fn] of Object.entries(handlers)) socket.on(event, fn);
    return () => {
      for (const [event, fn] of Object.entries(handlers)) socket.off(event, fn);
    };
  }, []);

  return { connected, status, pose, mapMeta, path, scan, goal, historyVersion };
}
