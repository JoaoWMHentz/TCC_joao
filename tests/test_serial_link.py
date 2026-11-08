"""Testes do enlace serial com reconexão (hardware/hardware/serial_link.py)."""

import math

from hardware import serial_link
from hardware.serial_link import SerialLink
import pytest
from ros_fakes import FakeLogger
import serial


class FakeSerial:
    """serial.Serial falso: `script` é a sequência de leituras (bytes ou exceção).

    Quando o script acaba, chama `on_empty` (os testes param o enlace ali).
    """

    script: list = []
    open_errors = 0          # quantas aberturas seguidas falham
    on_empty = None
    instances: list = []

    def __init__(self):
        self.port = self.baudrate = self.timeout = self.write_timeout = None
        self.rts = self.dtr = None
        self.in_waiting = 0
        self.written = []
        self.closed = False
        self.fail_write = False
        self.fail_close = False
        FakeSerial.instances.append(self)

    def open(self):  # noqa: A003
        if FakeSerial.open_errors > 0:
            FakeSerial.open_errors -= 1
            raise serial.SerialException('sem porta')

    def read(self, n):
        if not FakeSerial.script:
            FakeSerial.on_empty()
            return b''
        item = FakeSerial.script.pop(0)
        if isinstance(item, Exception):
            raise item
        return item

    def write(self, data):
        if self.fail_write:
            raise serial.SerialException('caiu')
        self.written.append(data)

    def close(self):
        self.closed = True
        if self.fail_close:
            raise OSError('já fechada')


@pytest.fixture
def fake_serial(monkeypatch):
    monkeypatch.setattr(serial_link.serial, 'Serial', FakeSerial)
    FakeSerial.script, FakeSerial.open_errors, FakeSerial.instances = [], 0, []
    return FakeSerial


def _link(lines=None, port='/dev/ttyUSB0', on_line=None):
    log = FakeLogger()
    received = [] if lines is None else lines
    link = SerialLink('T', port, 115200, on_line or received.append, log, reopen_period_s=0.0)
    FakeSerial.on_empty = link._stop.set
    return link, log, received


def test_estado_inicial_sem_porta():
    link, _, _ = _link()
    assert not link.is_open
    assert math.isinf(link.seconds_since_rx())
    assert link.write_line('C 0 0') is False


def test_abre_porta_e_escreve(fake_serial):
    link, log, _ = _link()
    assert link._open()
    ser = fake_serial.instances[-1]
    assert link.is_open and ser.baudrate == 115200 and ser.rts is False and ser.dtr is False
    assert link.write_line('C 1 2') is True
    assert ser.written == [b'C 1 2\n']
    assert any('porta aberta' in m for _, m in log.messages)


def test_dtr_alto_em_porta_acm(fake_serial):
    link, _, _ = _link(port='/dev/ttyACM0')
    link._open()
    assert fake_serial.instances[-1].dtr is True


def test_falha_ao_abrir(fake_serial):
    fake_serial.open_errors = 1
    link, _, _ = _link()
    assert link._open() is False
    assert not link.is_open


def test_falha_de_escrita_fecha_a_porta(fake_serial):
    link, log, _ = _link()
    link._open()
    fake_serial.instances[-1].fail_write = True
    assert link.write_line('x') is False
    assert not link.is_open
    assert log.messages[-1][0] == 'warn'


def test_fechar_ignora_erro(fake_serial):
    link, _, _ = _link()
    link._open()
    fake_serial.instances[-1].fail_close = True
    link._close()
    assert not link.is_open


def test_run_entrega_linhas(fake_serial):
    link, _, received = _link()
    fake_serial.script = [b'M 1 2', b' 3 4 R\n\n', b'I 1\r\nresto', b'\n']
    link._run()
    assert received == ['M 1 2 3 4 R', 'I 1', 'resto']
    assert link.seconds_since_rx() < 1.0


def test_run_callback_com_erro_nao_derruba(fake_serial):
    def boom(line):
        raise ValueError('ruim')
    link, log, _ = _link(on_line=boom)
    fake_serial.script = [b'abc\n']
    link._run()
    assert any(level == 'error' and 'abc' in m for level, m in log.messages)


def test_run_descarta_lixo_sem_fim_de_linha(fake_serial):
    link, _, received = _link()
    fake_serial.script = [b'x' * 5000, b'ok\n']
    link._run()
    assert received == ['ok']


def test_run_reabre_apos_erro_de_leitura(fake_serial):
    link, log, received = _link()
    fake_serial.script = [b'a\n', serial.SerialException('caiu'), b'b\n']
    link._run()
    assert received == ['a', 'b']
    assert len(fake_serial.instances) == 2            # reabriu a porta
    assert any('erro de leitura' in m for _, m in log.messages)


def test_run_aguarda_porta(fake_serial):
    link, log, received = _link()
    fake_serial.open_errors = 2
    fake_serial.script = [b'ok\n']
    link._run()
    assert received == ['ok']
    assert sum('aguardando' in m for _, m in log.messages) == 2


def test_start_stop_com_thread(fake_serial):
    link, _, _ = _link()
    fake_serial.on_empty = lambda: None
    link.start()
    link.stop()
    assert not link._thread.is_alive()
    assert not link.is_open
