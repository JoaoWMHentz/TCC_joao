"""
serial_link.py — enlace serial orientado a linhas, com reconexão automática.

Uma thread lê a porta e entrega cada linha recebida ao callback `on_line`.
Se a porta cair (cabo, reset do microcontrolador), a thread tenta reabri-la
periodicamente. `seconds_since_rx()` permite ao nó decidir se o enlace está
saudável.
"""

from __future__ import annotations

import os
import threading
import time
from typing import Callable

import serial


class SerialLink:

    def __init__(self, name: str, port: str, baudrate: int,
                 on_line: Callable[[str], None], logger,
                 reopen_period_s: float = 1.0):
        self.name = name
        self._port = port
        self._baud = baudrate
        self._on_line = on_line
        self._log = logger
        self._reopen_period = reopen_period_s

        self._ser: serial.Serial | None = None
        self._write_lock = threading.Lock()
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None
        self._last_rx = None  # time.monotonic() da última linha recebida

    # ── Ciclo de vida ────────────────────────────────────────────────────────

    def start(self):
        self._thread = threading.Thread(target=self._run, name=f'{self.name}-rx', daemon=True)
        self._thread.start()

    def stop(self):
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=1.0)
        self._close()

    # ── Estado ───────────────────────────────────────────────────────────────

    @property
    def is_open(self) -> bool:
        return self._ser is not None

    def seconds_since_rx(self) -> float:
        if self._last_rx is None:
            return float('inf')
        return time.monotonic() - self._last_rx

    # ── Escrita ──────────────────────────────────────────────────────────────

    def write_line(self, text: str) -> bool:
        with self._write_lock:
            ser = self._ser
            if ser is None:
                return False
            try:
                ser.write((text + '\n').encode('ascii'))
                return True
            except (serial.SerialException, OSError) as exc:
                self._log.warn(f'[{self.name}] falha de escrita: {exc}')
                self._close()
                return False

    # ── Leitura (thread) ─────────────────────────────────────────────────────

    def _open(self) -> bool:
        try:
            ser = serial.Serial()
            ser.port = self._port
            ser.baudrate = self._baud
            ser.timeout = 0.2
            ser.write_timeout = 0.05
            # Placas com auto-reset (ESP32 DevKit) usam DTR/RTS para entrar no
            # bootloader: mantê-los baixos evita reset ao abrir a porta.
            # Já o USB CDC do STM32 (ttyACM) só transmite com DTR alto.
            ser.rts = False
            ser.dtr = 'ttyACM' in os.path.realpath(self._port)
            ser.open()
        except (serial.SerialException, OSError):
            return False
        with self._write_lock:
            self._ser = ser
        self._log.info(f'[{self.name}] porta aberta: {self._port} @ {self._baud}')
        return True

    def _close(self):
        with self._write_lock:
            ser, self._ser = self._ser, None
        if ser is not None:
            try:
                ser.close()
            except Exception:
                pass

    def _run(self):
        buffer = b''
        while not self._stop.is_set():
            if self._ser is None:
                if not self._open():
                    self._log.warn(f'[{self.name}] aguardando {self._port}...',
                                   throttle_duration_sec=5.0)
                    self._stop.wait(self._reopen_period)
                    continue
                buffer = b''
            try:
                chunk = self._ser.read(self._ser.in_waiting or 1)
            except (serial.SerialException, OSError, AttributeError) as exc:
                self._log.warn(f'[{self.name}] erro de leitura: {exc}')
                self._close()
                continue
            if not chunk:
                continue
            buffer += chunk
            while b'\n' in buffer:
                raw, buffer = buffer.split(b'\n', 1)
                line = raw.decode('ascii', errors='ignore').strip()
                if not line:
                    continue
                self._last_rx = time.monotonic()
                try:
                    self._on_line(line)
                except Exception as exc:  # callback nunca derruba a thread
                    self._log.error(f'[{self.name}] erro tratando "{line}": {exc}')
            if len(buffer) > 4096:  # lixo sem fim de linha
                buffer = b''
