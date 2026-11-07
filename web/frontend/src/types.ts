export type SessionMode = 'idle' | 'starting' | 'mapping' | 'navigation' | 'stopping';

export interface HardwareItem {
  level: number; // 0 OK · 1 aviso · 2 erro · 3 sem dados
  message: string;
}

export interface Mission {
  kind: 'goal' | 'point' | 'route';
  label: string;
  index: number;
  total: number;
}

export interface Status {
  mode: SessionMode;
  map: string | null;
  sim: boolean;
  error: string | null;
  ros_ok: boolean;
  processes: Record<'base' | 'slam' | 'nav', boolean>;
  explore?: { running: boolean; message: string | null };
  mission: Mission | null;
  hardware: Record<string, HardwareItem>;
}

export interface Pose {
  x: number;
  y: number;
  theta: number;
  available: boolean;
}

export interface MapMeta {
  width: number;
  height: number;
  resolution: number;
  origin_x: number;
  origin_y: number;
}

export interface MapData extends MapMeta {
  png_b64: string;
}

export interface Target {
  name?: string;
  x: number;
  y: number;
  theta?: number;
}

export interface GoalStatus {
  status: 'running' | 'succeeded' | 'aborted' | 'canceled' | 'rejected';
  text?: string;
  kind: Mission['kind'];
  label: string;
  index: number;
  total: number;
  target: Target;
  distance_remaining?: number;
}

export interface Point {
  id: number;
  name: string;
  x: number;
  y: number;
  theta: number;
}

export interface Route {
  id: number;
  name: string;
  point_ids: number[];
}

export interface SavedMap {
  name: string;
  size_mb: number;
  modified: number;
}

export interface HistoryEntry {
  id: number;
  map: string | null;
  kind: Mission['kind'];
  label: string;
  started_at: string;
  result: GoalStatus['status'] | null;
  duration_s: number | null;
  distance_m: number | null;
  recoveries: number | null;
  min_clearance_m: number | null;
  final_error_m: number | null;
}

export interface HistorySummary {
  total: number;
  succeeded: number;
  avg_duration_s: number | null;
  total_distance_m: number | null;
  total_recoveries: number | null;
  min_clearance_m: number | null;
  avg_final_error_m: number | null;
}

export type XY = [number, number];
