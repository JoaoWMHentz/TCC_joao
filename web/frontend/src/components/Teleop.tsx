import { useEffect, useRef } from 'react';
import { api } from '../api';

const SPEED = { linear: 0.8, angular: 0.6 };
type Dir = [number, number];

// Setas de condução manual: enquanto o botão está pressionado, envia o comando
// a 5 Hz (o robô para sozinho se os comandos deixarem de chegar).
export default function Teleop({ enabled }: { enabled: boolean }) {
  const timer = useRef<number | null>(null);

  const stop = () => {
    if (timer.current !== null) {
      clearInterval(timer.current);
      timer.current = null;
      api('/teleop', { linear: 0, angular: 0 }).catch(() => {});
    }
  };

  const start = ([lin, ang]: Dir) => {
    stop();
    const send = () => api('/teleop', { linear: lin * SPEED.linear, angular: ang * SPEED.angular }).catch(() => {});
    send();
    timer.current = window.setInterval(send, 200);
  };

  useEffect(() => stop, []);

  const pad: { label: string; name: string; dir: Dir; area: string }[] = [
    { label: '▲', dir: [1, 0], area: 'up', name: 'Frente' },
    { label: '◀', dir: [0, 1], area: 'left', name: 'Girar à esquerda' },
    { label: '▶', dir: [0, -1], area: 'right', name: 'Girar à direita' },
    { label: '▼', dir: [-1, 0], area: 'down', name: 'Ré' },
  ];

  return (
    <section className="panel">
      <h2>Condução manual</h2>
      <div className="teleop">
        {pad.map((b) => (
          <button
            key={b.area}
            className={`teleop-${b.area}`}
            aria-label={b.name}
            title={b.name}
            disabled={!enabled}
            onPointerDown={() => start(b.dir)}
            onPointerUp={stop}
            onPointerLeave={stop}
            onPointerCancel={stop}
          >
            {b.label}
          </button>
        ))}
      </div>
    </section>
  );
}
