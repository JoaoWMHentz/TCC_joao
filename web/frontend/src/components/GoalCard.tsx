import { api } from '../api';
import type { GoalStatus, Status } from '../types';

interface Props {
  status: Status | null;
  goal: GoalStatus | null;
  notify: (msg: string, error?: boolean) => void;
}

const RESULT_TEXT: Record<string, string> = {
  succeeded: 'Chegou ao destino',
  aborted: 'Não conseguiu chegar',
  canceled: 'Cancelado',
  rejected: 'Destino recusado',
};

// Andamento da navegação atual, com botão de cancelar.
export default function GoalCard({ status, goal, notify }: Props) {
  const mission = status?.mission;
  if (!mission && !goal) return null;

  const running = !!mission;
  const name = goal?.target?.name ?? 'destino';
  const step = running && mission.total > 1 ? ` (${mission.index + 1}/${mission.total})` : '';

  const cancel = () => api('/cancel', {}).catch((e: Error) => notify(e.message, true));

  return (
    <section className={`panel goal-card ${running ? 'running' : `done-${goal?.status}`}`}>
      {running ? (
        <>
          <div className="goal-line">
            <span className="pulse" />
            <span>
              {mission.kind === 'route' ? `Rota "${mission.label}"` : 'Indo para'} <strong>{name}</strong>
              {step}
            </span>
          </div>
          {goal?.distance_remaining !== undefined && (
            <div className="goal-distance">{goal.distance_remaining.toFixed(1)} m restantes</div>
          )}
          <button className="danger wide" onClick={cancel}>
            Parar o robô
          </button>
        </>
      ) : (
        <div className="goal-line">
          <span>{RESULT_TEXT[goal!.status] ?? goal!.status}</span>
          <strong>{goal!.label}</strong>
        </div>
      )}
    </section>
  );
}
