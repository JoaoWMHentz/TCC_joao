"""
app.py — servidor da interface web de operação.

HTTP (Flask) para comandos e consultas; Socket.IO para o estado em tempo real
(pose, mapa, caminho, laser, status da navegação). O nó ROS roda numa thread
própria (ros_bridge) e os eventos dele passam por uma fila que mantém apenas
o valor mais recente dos eventos de alta frequência.

Uso:  python app.py     (com o ROS e o workspace carregados no ambiente)
"""

from __future__ import annotations

import logging
import os
import pathlib
import queue
import signal
import threading
import time

import api
import config
import db
from flask import Flask, request, send_from_directory
from flask.json.provider import DefaultJSONProvider
from flask_cors import CORS
from flask_socketio import SocketIO
from launcher import Launcher
import ros_bridge

logging.basicConfig(level=logging.INFO,
                    format='%(asctime)s [%(levelname)s] %(name)s: %(message)s', datefmt='%H:%M:%S')
logging.getLogger('werkzeug').setLevel(logging.WARNING)
log = logging.getLogger('app')


class JSONProvider(DefaultJSONProvider):
    """Datas do PostgreSQL saem em ISO 8601."""

    @staticmethod
    def default(o):
        if hasattr(o, 'isoformat'):
            return o.isoformat()
        return DefaultJSONProvider.default(o)


# Frontend compilado (npm run build) servido na mesma porta da API
FRONTEND_DIST = pathlib.Path(__file__).resolve().parent.parent / 'frontend' / 'dist'

app = Flask(__name__, static_folder=None)
app.json = JSONProvider(app)
CORS(app)
socketio = SocketIO(app, cors_allowed_origins='*', async_mode='threading')

launcher = Launcher()
api.launcher = launcher
app.register_blueprint(api.api)


@app.get('/')
@app.get('/<path:path>')
def frontend(path: str = 'index.html'):
    if not FRONTEND_DIST.exists():
        return 'Frontend não compilado: rode "npm run build" em web/frontend', 404
    if not (FRONTEND_DIST / path).is_file():
        path = 'index.html'
    return send_from_directory(FRONTEND_DIST, path)


# ── Eventos ROS -> navegador ─────────────────────────────────────────────────
# Eventos de alta frequência guardam só o último valor; os demais vão em ordem.
KEEP_LAST = {'pose_update', 'path_update', 'scan_update', 'map_update', 'hardware_update'}
_latest: dict[str, dict] = {}
_latest_lock = threading.Lock()
_ordered: queue.Queue = queue.Queue(maxsize=500)


def emit(event: str, data: dict):
    if event in KEEP_LAST:
        with _latest_lock:
            _latest[event] = data
    else:
        try:
            _ordered.put_nowait((event, data))
        except queue.Full:
            log.warning('fila de eventos cheia; descartando %s', event)


def _drain_events():
    while True:
        while True:
            try:
                event, data = _ordered.get_nowait()
            except queue.Empty:
                break
            socketio.emit(event, data)
        with _latest_lock:
            pending = list(_latest.items())
            _latest.clear()
        for event, data in pending:
            socketio.emit(event, data)
        time.sleep(0.05)


@socketio.on('connect')
def _on_connect():
    """Cliente novo recebe o estado atual sem esperar a próxima mudança."""
    socketio.emit('status_update', api.status_payload(), to=request.sid)
    node = ros_bridge.node()
    current_map = node.get_map() if node else None
    if current_map:
        meta = {k: v for k, v in current_map.items() if k != 'png_b64'}
        socketio.emit('map_update', meta, to=request.sid)


def _status_loop():
    while True:
        launcher.watch()
        try:
            socketio.emit('status_update', api.status_payload())
        except Exception:
            log.exception('status_loop')
        time.sleep(1.0)


def _init_db():
    for attempt in range(10):
        try:
            db.init()
            log.info('PostgreSQL: esquema ok')
            return
        except Exception as exc:
            log.warning('PostgreSQL indisponível (%s), nova tentativa em 3 s', exc)
            time.sleep(3)
    log.error('PostgreSQL: desisti — destinos, rotas e histórico ficarão indisponíveis')


def _shutdown(signum, _frame):
    """Ctrl+C / SIGTERM: encerra a sessão (o RTAB-Map salva o mapa) e sai."""
    log.info('sinal %s recebido: encerrando a sessão antes de sair', signal.Signals(signum).name)
    node = ros_bridge.node()
    if node:
        node.cancel()
    launcher.stop_session()
    os._exit(0)


def main():
    signal.signal(signal.SIGINT, _shutdown)
    signal.signal(signal.SIGTERM, _shutdown)
    threading.Thread(target=_init_db, daemon=True).start()
    ros_bridge.start(emit)
    threading.Thread(target=_drain_events, daemon=True).start()
    threading.Thread(target=_status_loop, daemon=True).start()
    log.info('modo %s · mapas em %s', 'SIMULAÇÃO' if config.SIM else 'ROBÔ REAL', config.MAPS_DIR)
    socketio.run(app, host=config.HOST, port=config.PORT, allow_unsafe_werkzeug=True)


if __name__ == '__main__':
    main()
