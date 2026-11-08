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

from collections import deque
import logging
import os
import re
import signal
import subprocess
import threading
import time

import config

log = logging.getLogger('launcher')

MAP_NAME_RE = re.compile(r'^[A-Za-z0-9_-]{1,40}$')
STOP_GRACE_S = {'slam': 60.0, 'explore': 5.0}
# Estratégias de exploração (argumento strategy do explore.launch.py)
EXPLORE_STRATEGIES = ('wall_bfs', 'pocket', 'frontier')
DEFAULT_STRATEGY = 'wall_bfs'
DEFAULT_STOP_GRACE_S = 10.0
# O Nav2 pode subir com os processos vivos mas os nós inativos (o
# bt_navigator então recusa todo destino). Isso aparece no log como:
NAV_BRINGUP_FAILED = 'Failed to bring up all requested nodes'
NAV_MAX_RESTARTS = 2
START_DELAY_S = 5.0  # espera a base publicar odom/TF antes do RTAB-Map


def _commands(mode: str, db_path: str, strategy: str = DEFAULT_STRATEGY) -> dict[str, str]:
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
        'explore': (f'ros2 launch navigation explore.launch.py '
                    f'use_sim_time:={sim} strategy:={strategy}'),
    }


# "[INFO] [1712345.678] [frontier_explorer]: mensagem" -> nível, instante, mensagem
# (o instante do log do ROS é sempre o relógio do sistema, mesmo com use_sim_time)
LOG_MSG_RE = re.compile(r'\[([A-Z]+)\] \[([\d.]+)\] \[[^\]]+\]: (.*)$')
EXPLORE_LOG_MAX = 100


class Launcher:

    ORDER = ('base', 'slam', 'nav')

    def __init__(self):
        self._lock = threading.Lock()
        self._procs: dict[str, subprocess.Popen] = {}
        self._logs: dict[str, deque] = {name: deque(maxlen=300)
                                        for name in (*self.ORDER, 'explore')}
        self.mode = 'idle'        # idle | starting | mapping | navigation | stopping
        self.map_name: str | None = None
        self.error: str | None = None
        self.explore_strategy: str | None = None   # a da última exploração iniciada
        self._cmds: dict[str, str] = {}
        self._nav_failed = False       # o Nav2 abortou a subida (ver NAV_BRINGUP_FAILED)
        self._nav_restarts = 0
        self._nav_restarting = False
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

    def delete_map(self, name: str) -> tuple[bool, str]:
        """Apaga o .db de um mapa salvo (não o que está em uso)."""
        if not MAP_NAME_RE.match(name or ''):
            return False, 'Nome de mapa inválido'
        path = self.map_path(name)
        with self._lock:
            if self.mode != 'idle' and self.map_name == name:
                return False, 'Este mapa está em uso; pare a sessão antes de excluí-lo'
            if not path.exists():
                return False, f'Mapa "{name}" não encontrado'
            path.unlink()
        log.info('mapa "%s" excluído', name)
        return True, 'excluído'

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
            'explore': self.explore_status(),
        }

    def logs(self, name: str) -> list[str]:
        return list(self._logs.get(name, []))

    # ── Exploração automática (só durante o mapeamento) ─────────────────────

    def explore_status(self) -> dict:
        log = self.explore_log(limit=1)
        return {'running': self.running('explore'),
                'message': log[0]['text'] if log else None,
                'strategy': self.explore_strategy}

    def explore_log(self, limit: int = EXPLORE_LOG_MAX) -> list[dict]:
        """Decisões do explorador desta sessão, da mais antiga para a mais nova.

        Cada explorador registra em log o motivo de cada mudança de
        comportamento (seguir a parede, ir a uma fronteira, descartar um
        destino, iniciar um passe); aqui as linhas viram {t, level, text}.
        """
        # cópia antes de percorrer: a thread _pump acrescenta linhas ao mesmo tempo
        lines = list(self._logs['explore'])
        entries = []
        for line in lines:
            m = LOG_MSG_RE.search(line)
            if m:
                entries.append({'t': float(m.group(2)), 'level': m.group(1).lower(),
                                'text': m.group(3)})
        return entries[-limit:] if limit > 0 else []

    def start_explore(self, strategy: str = DEFAULT_STRATEGY) -> tuple[bool, str]:
        if strategy not in EXPLORE_STRATEGIES:
            return False, f'Estratégia de exploração inválida: {strategy}'
        with self._lock:
            if self.mode != 'mapping':
                return False, 'A exploração automática só funciona durante um mapeamento'
            if self.running('explore'):
                return False, 'A exploração já está em andamento'
            self._logs['explore'].clear()
            self.explore_strategy = strategy
            self._spawn('explore', _commands('mapping', '', strategy)['explore'])
        return True, 'exploração iniciada'

    def stop_explore(self, wait: bool = False) -> None:
        """Encerra o explorador.

        O SIGINT sai na hora (ele cancela o destino no Nav2 e para de mandar
        outros); a espera/escalada fica em segundo plano.
        """
        proc = self._procs.get('explore')
        if proc is None or proc.poll() is not None:
            return
        try:
            os.killpg(proc.pid, signal.SIGINT)
        except ProcessLookupError:
            return
        if wait:
            self._stop('explore')
        else:
            threading.Thread(target=self._stop, args=('explore',), daemon=True).start()

    # ── Sessões ──────────────────────────────────────────────────────────────

    def start_session(self, mode: str, name: str, explore: bool = False,
                      strategy: str = DEFAULT_STRATEGY) -> tuple[bool, str]:
        """Inicia uma sessão de mapeamento ou navegação.

        explore=True (só no mapeamento): inicia a exploração automática, com a
        estratégia dada, assim que a pilha estiver no ar.
        """
        if mode not in ('mapping', 'navigation'):
            return False, 'modo inválido'
        if strategy not in EXPLORE_STRATEGIES:
            return False, f'Estratégia de exploração inválida: {strategy}'
        if explore and mode != 'mapping':
            return False, 'A exploração automática só funciona durante um mapeamento'
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
            self._logs['explore'].clear()   # decisões da sessão anterior não valem mais

        threading.Thread(target=self._start_sequence, args=(mode, str(path), explore, strategy),
                         daemon=True).start()
        return True, 'iniciando'

    def _start_sequence(self, mode: str, db_path: str, explore: bool = False,
                        strategy: str = DEFAULT_STRATEGY):
        cmds = self._cmds = _commands(mode, db_path, strategy)
        self._nav_failed, self._nav_restarts = False, 0
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
            if self.mode != 'starting':
                return              # sessão parada durante a partida
            self.mode = mode
            if explore:
                # o explorador espera sozinho o mapa, o TF e o Nav2 ficarem prontos
                self._logs['explore'].clear()
                self.explore_strategy = strategy
                self._spawn('explore', cmds['explore'])

    def stop_session(self) -> None:
        with self._lock:
            if self.mode == 'stopping':
                return
            self.mode = 'stopping'
        self.stop_explore(wait=True)
        for name in reversed(self.ORDER):
            self._stop(name)
        with self._lock:
            self.mode, self.map_name = 'idle', None

    def watch(self):
        """Chamado periodicamente: detecta processos que morreram sozinhos."""
        if self.mode in ('mapping', 'navigation'):
            if self._nav_restarting:
                return
            if self._nav_failed:
                self._recover_nav()
                return
            dead = [n for n in self.ORDER if not self.running(n)]
            if dead:
                self.error = f'Processo encerrou inesperadamente: {", ".join(dead)}'
                log.error(self.error)
                threading.Thread(target=self.stop_session, daemon=True).start()

    def _recover_nav(self):
        """O Nav2 não ativou: reinicia só o processo nav (até NAV_MAX_RESTARTS)."""
        self._nav_failed = False
        if self._nav_restarts >= NAV_MAX_RESTARTS:
            self.error = ('O Nav2 não conseguiu iniciar (destinos serão recusados); '
                          'pare e abra o mapa de novo')
            log.error(self.error)
            return
        self._nav_restarts += 1
        self._nav_restarting = True
        log.warning('Nav2 falhou ao ativar; reiniciando (tentativa %d)', self._nav_restarts)

        def restart():
            try:
                self._stop('nav')
                if self.mode in ('mapping', 'navigation'):
                    self._spawn('nav', self._cmds['nav'])
            finally:
                self._nav_restarting = False

        threading.Thread(target=restart, daemon=True).start()

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
                if name == 'nav' and NAV_BRINGUP_FAILED in line and proc is self._procs.get('nav'):
                    self._nav_failed = True
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
