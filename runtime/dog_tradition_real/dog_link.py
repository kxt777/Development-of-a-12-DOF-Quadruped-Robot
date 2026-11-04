"""One bounded serial writer and one passive feedback reader per application."""
from __future__ import annotations

import threading
import time
from dog_clock import now as clock_now

from dog_protocol import FeedbackParser

GLOBAL_ENABLE = b'DM12\x20\x01'
GLOBAL_DISABLE = b'DM12\x20\x02'


class Cancelled(RuntimeError):
    pass


class MotorLink:
    def __init__(self, config):
        self.config = config
        self.handle = None
        self.port = None
        self.error = None
        self._data_lock = threading.Lock()
        self._write_lock = threading.Lock()
        self._stop = threading.Event()
        self._thread = None
        self._latest = None
        self._timestamp = 0.0
        self._frames = 0

    def open(self):
        import serial
        from serial.tools import list_ports
        expected = self.config.get('usb_serial')
        selected = self.config.get('port')
        ports = [p for p in list_ports.comports()
                 if (p.vid, p.pid) == (0x0483, 0x5740)
                 and (not expected or p.serial_number == expected)
                 and (not selected or p.device.casefold() == selected.casefold())]
        if len(ports) != 1:
            raise RuntimeError('未找到唯一匹配的 STM32；检查端口和 USB 序列号')
        self.port = ports[0].device
        self.handle = serial.Serial(port=None, baudrate=self.config['baudrate'],
                                    timeout=.01, write_timeout=.02,
                                    exclusive=True)
        self.handle.dtr = self.handle.rts = False
        self.handle.port = self.port
        self.handle.open()
        self._thread = threading.Thread(target=self._reader, daemon=True,
                                        name='dog_feedback')
        self._thread.start()

    def _reader(self):
        parser = FeedbackParser()
        try:
            while not self._stop.is_set():
                raw = self.handle.read(min(max(self.handle.in_waiting, 1), 8192))
                for frame in parser.append(raw):
                    with self._data_lock:
                        self._latest = frame
                        self._timestamp = clock_now()
                        self._frames += 1
        except Exception as error:
            self.error = str(error)

    def send(self, packet, cancelled=None):
        # Check the latch INSIDE the lock: a cancelled trajectory cannot enqueue
        # a command behind disable. No flush(), which can block without a bound.
        with self._write_lock:
            if cancelled is not None and cancelled.is_set():
                raise Cancelled('动作已被失能中断')
            if self.handle is None or not self.handle.is_open:
                raise RuntimeError('STM32 未连接')
            count = self.handle.write(packet)
            if count != len(packet):
                raise RuntimeError(f'串口写入不完整：{count}/{len(packet)}')

    def snapshot(self):
        with self._data_lock:
            return (None if self._latest is None else [dict(r) for r in self._latest],
                    self._timestamp, self._frames)

    def close(self):
        self._stop.set()
        if self._thread:
            self._thread.join(timeout=.3)
        if self.handle:
            self.handle.close()


class DryRunLink:
    """No serial imports or I/O. Feedback follows targets for UI/flow checks."""
    def __init__(self, positions):
        self.port = 'DRY RUN / 不连接硬件'
        self.error = None
        self._positions = list(positions)
        self._enabled = False
        self._lock = threading.Lock()
        self.packets = []
        self._count = 0

    def open(self):
        pass

    def send(self, packet, cancelled=None):
        from dog_protocol import RECORD
        with self._lock:
            if cancelled is not None and cancelled.is_set():
                raise Cancelled('动作已被失能中断')
            self.packets.append(packet)
            self._count += 1
            if len(self.packets) > 1000:
                del self.packets[:500]
            if packet == GLOBAL_ENABLE:
                self._enabled = True
            elif packet == GLOBAL_DISABLE:
                self._enabled = False
            elif packet[:5] == b'DM12\x10':
                for index in range(12):
                    valid, _, _, q, _, kp, _, _ = RECORD.unpack_from(packet, 5+index*RECORD.size)
                    if valid and kp > 0 and self._enabled:
                        self._positions[index] = q

    def snapshot(self):
        with self._lock:
            return ([dict(valid=True, id=i+1, state=int(self._enabled),
                          position=q, velocity=0., torque=0., tmos=25., tcoil=25.)
                     for i, q in enumerate(self._positions)], clock_now(),
                    self._count)

    def close(self):
        pass
