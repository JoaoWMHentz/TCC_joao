import { useEffect, useState } from 'react';

// Imagem atual da câmera RGB do robô, recebida como stream MJPEG do backend.
// O backend só converte quadros enquanto alguém assiste, então ocultar o painel
// economiza CPU no robô e banda na rede.
export default function CameraView() {
  const [visible, setVisible] = useState(true);
  const [live, setLive] = useState(false);
  const [attempt, setAttempt] = useState(0);

  // Se o stream cair (ou a câmera ainda não estiver publicando), tenta de novo
  useEffect(() => {
    if (!visible || live) return;
    const timer = window.setTimeout(() => setAttempt((n) => n + 1), 3000);
    return () => clearTimeout(timer);
  }, [visible, live, attempt]);

  return (
    <section className="panel">
      <h2 className="panel-title">
        Câmera
        <button className="ghost" onClick={() => { setVisible((v) => !v); setLive(false); }}>
          {visible ? 'Ocultar' : 'Mostrar'}
        </button>
      </h2>
      {visible && (
        <div className="camera-view">
          <img
            key={attempt}
            src={`/api/camera/stream?t=${attempt}`}
            alt="Imagem ao vivo da câmera do robô"
            onLoad={() => setLive(true)}
            onError={() => setLive(false)}
            style={{ visibility: live ? 'visible' : 'hidden' }}
          />
          {!live && <div className="camera-empty">Aguardando imagem da câmera...</div>}
        </div>
      )}
    </section>
  );
}
