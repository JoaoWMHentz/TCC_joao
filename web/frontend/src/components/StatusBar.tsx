import type { Pose, Status } from '../types';

interface Props {
  connected: boolean;
  status: Status | null;
  pose: Pose;
}

const LEVEL = ['ok', 'warn', 'error', 'stale'];

// Barra superior: conexão, modo, localização e saúde do hardware.
export default function StatusBar({ connected, status, pose }: Props) {
  const hardware = Object.entries(status?.hardware ?? {}).filter(([name]) => name.startsWith('hardware/'));
  const active = status && status.mode !== 'idle';

  return (
    <header className="status-bar">
      <h1>Operação do Robô</h1>
      <div className="badges">
        {status?.sim && <span className="badge info">Simulação</span>}
        <span className={`badge ${connected ? 'ok' : 'error'}`}>{connected ? 'Conectado' : 'Sem conexão'}</span>
        {active && (
          <span className={`badge ${pose.available ? 'ok' : 'warn'}`}>
            {pose.available ? 'Localizado' : 'Sem localização'}
          </span>
        )}
        {hardware.map(([name, item]) => (
          <span key={name} className={`badge ${LEVEL[item.level] ?? 'warn'}`} title={item.message}>
            {name.replace('hardware/', '') === 'motors' ? 'Motores' : 'IMU'}: {item.message}
          </span>
        ))}
      </div>
    </header>
  );
}
