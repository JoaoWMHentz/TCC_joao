"""
launcher.py — sobe e derruba as três partes da pilha ROS como processos.

  base  : sensores e atuadores (robô real: bringup; simulação: sim)
  slam  : RTAB-Map (mapeamento ou localização)
  nav   : Nav2

Uma *sessão* é a combinação das três em um modo:
  mapping     -> cria um mapa novo   (slam mode:=mapping,      nav allow_unknown)
  navigation  -> usa um mapa salvo   (slam mode:=localization, nav normal)

Cada processo roda no seu próprio grupo (setsid). Parar = SIGINT no grupo,
como um Ctrl+C. O RTAB-Map grava o .db inteiro ao receber o SIGINT, então
o slam tem uma espera longa antes de escalar para SIGTERM/SIGKILL.
"""

from __future__ import annotations

import logging
import os
import re
import signal
import subprocess
import threading
import time
from collections import deque

import config

log = logging.getLogger('launcher')

MAP_NAME_RE = re.compile(r'^[A-Za-z0-9_-]{1,40}$')
STOP_GRACE_S = {'slam': 60.0}
DEFAULT_STOP_GRACE_S = 10.0
START_DELAY_S = 5.0  # espera a base publicar odom/TF antes do RTAB-Map


def _commands(mode: str, db_path: str) -> dict[str, str]:
    sim = 'true' if config.SIM else 'false'
    camera = 'true' if config.USE_CAMERA else 'false'
    if config.SIM:
        base = 'ros2 launch sim robot_sim.launch.py start_world:=false'
    else:
        base = f'ros2 launch bringup robot.launch.py use_camera:={camera}'
    slam_mode = 'mapping' if mode == 'mapping' else 'localization'
    allow_unknown = 'true' if mode == 'mapping' else 'false'
    return {
        'base': base,
        'slam': (f'ros2 launch navigation slam.launch.py mode:={slam_mode} '
                 f'db_path:={db_path} use_camera:={camera} use_sim_time:={sim}'),
        'nav': (f'ros2 launch navigation nav2.launch.py '
                f'use_sim_time:={sim} allow_unknown:={allow_unknown}'),
    }


class Launcher:

    ORDER = ('base', 'slam', 'nav')

    def __init__(self):
        self._lock = threading.Lock()
        self._procs: dict[str, subprocess.Popen] = {}
        self._logs: dict[str, deque] = {name: deque(maxlen=300) for name in self.ORDER}
        self.mode = 'idle'        # idle | starting | mapping | navigation | stopping
        self.map_name: str | None = None
        self.error: str | None = None
        config.MAPS_DIR.mkdir(parents=True, exist_ok=True)

    # ── Consulta ─────────────────────────────────────────────────────────────

    def map_path(self, name: str):
        return config.MAPS_DIR / f'{name}.db'

    def list_maps(self) -> list[dict]:
        maps = []
        for path in sorted(config.MAPS_DIR.glob('*.db')):
            st = path.stat()
            maps.append({'name': path.stem, 'size_mb': round(st.st_size / 1e6, 1),
                         'modified': st.st_mtime})
        return maps

    def running(self, name: str) -> bool:
        proc = self._procs.get(name)
        return proc is not None and proc.poll() is None

    def status(self) -> dict:
        return {
            'mode': self.mode,
            'map': self.map_name,
            'sim': config.SIM,
            'error': self.error,
            'processes': {name: self.running(name) for name in self.ORDER},
        }

    def logs(self, name: str) -> list[str]:
        return list(self._logs.get(name, []))

    # ── Sessões ──────────────────────────────────────────────────────────────

    def start_session(self, mode: str, name: str) -> tuple[bool, str]:
        if mode not in ('mapping', 'navigation'):
            return False, 'modo inválido'
        if not MAP_NAME_RE.match(name or ''):
            return False, 'Nome do mapa: use letras, números, "-" ou "_" (até 40)'
        path = self.map_path(name)
        if mode == 'mapping' and path.exists():
            return False, f'Já existe um mapa chamado "{name}"'
        if mode == 'navigation' and not path.exists():
            return False, f'Mapa "{name}" não encontrado'
        with self._lock:
            if self.mode != 'idle':
                return False, 'Já existe uma sessão ativa; pare antes de iniciar outra'
            self.mode, self.map_name, self.error = 'starting', name, None

        threading.Thread(target=self._start_sequence, args=(mode, str(path)),
                         daemon=True).start()
        return True, 'iniciando'

    def _start_sequence(self, mode: str, db_path: str):
        cmds = _commands(mode, db_path)
        try:
            self._spawn('base', cmds['base'])
            time.sleep(START_DELAY_S)
            self._spawn('slam', cmds['slam'])
            self._spawn('nav', cmds['nav'])
        except Exception as exc:  # falha ao criar processo
            log.exception('falha ao iniciar a sessão')
            self.error = str(exc)
            self.stop_session()
            return
        with self._lock:
            if self.mode == 'starting':
                self.mode = mode

    def stop_session(self) -> None:
        with self._lock:
            if self.mode == 'stopping':
                return
            self.mode = 'stopping'
        for name in reversed(self.ORDER):
            self._stop(name)
        with self._lock:
            self.mode, self.map_name = 'idle', None

    def watch(self):
        """Chamado periodicamente: detecta processos que morreram sozinhos."""
        if self.mode in ('mapping', 'navigation'):
            dead = [n for n in self.ORDER if not self.running(n)]
            if dead:
                self.error = f'Processo encerrou inesperadamente: {", ".join(dead)}'
                log.error(self.error)
                threading.Thread(target=self.stop_session, daemon=True).start()

    # ── Processos ────────────────────────────────────────────────────────────

    def _spawn(self, name: str, cmd: str):
        shell = (f'source {config.ROS_SETUP} && source {config.WS_SETUP} && '
                 f'export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST && exec {cmd}')
        # O backend roda num venv: os nós ROS usam o Python do sistema
        env = {k: v for k, v in os.environ.items() if k not in ('VIRTUAL_ENV', 'PYTHONHOME')}
        env['PATH'] = '/usr/local/bin:/usr/bin:/bin'
        log.info('[%s] %s', name, cmd)
        proc = subprocess.Popen(['bash', '-c', shell], stdout=subprocess.PIPE,
                                stderr=subprocess.STDOUT, start_new_session=True, env=env)
        self._procs[name] = proc
        threading.Thread(target=self._pump, args=(name, proc), daemon=True).start()

    def _pump(self, name: str, proc: subprocess.Popen):
        for raw in iter(proc.stdout.readline, b''):
            line = raw.decode(errors='replace').rstrip()
            if line:
                self._logs[name].append(line)
        proc.stdout.close()

    def _stop(self, name: str):
        proc = self._procs.get(name)
        if proc is None or proc.poll() is not None:
            return
        grace = STOP_GRACE_S.get(name, DEFAULT_STOP_GRACE_S)
        for sig, wait in ((signal.SIGINT, grace), (signal.SIGTERM, 5.0), (signal.SIGKILL, 2.0)):
            try:
                os.killpg(proc.pid, sig)
            except ProcessLookupError:
                break
            try:
                proc.wait(timeout=wait)
                break
            except subprocess.TimeoutExpired:
                log.warning('[%s] não encerrou com %s', name, sig.name)
        log.info('[%s] encerrado', name)
