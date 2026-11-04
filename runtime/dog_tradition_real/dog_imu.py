# Reused unchanged from dog_rl/dog_nuc/Dog_ROS2/src/dm_imu/dm_imu/protocol.py.
"""DM-IMU-L1 USB protocol, manual V1.3 appendix E and vendor Humble example.

CRC includes the header, as specified by firmware 2.0.3.0. No configuration
commands are sent. Host receipt times do not imply hardware synchronization.
"""

import binascii
import math
import struct
from dataclasses import dataclass
from typing import Dict, List, Optional, Tuple

HEADER = b"\x55\xaa"
ACCEL = 0x01
GYRO = 0x02
EULER = 0x03
QUATERNION = 0x04
FRAME_SIZES = {ACCEL: 19, GYRO: 19, EULER: 19, QUATERNION: 23,
               0x05: 19, 0x07: 19}


def crc16(data: bytes) -> int:
    """CRC-16/CCITT-FALSE, poly=0x1021, init=0xffff, xorout=0."""
    return binascii.crc_hqx(data, 0xFFFF)


@dataclass(frozen=True)
class ImuFrame:
    kind: int
    values: Tuple[float, ...]
    slave_id: int


class ImuParser:
    def __init__(self, slave_id: int = 1):
        if not 0 <= slave_id <= 255:
            raise ValueError("slave_id must fit in one byte")
        self.slave_id = slave_id
        self._buffer = bytearray()
        self.frames_ok = 0
        self.crc_errors = 0
        self.format_errors = 0
        self.other_devices = 0
        self.other_frames = 0

    def append(self, data: bytes) -> List[ImuFrame]:
        self._buffer.extend(data)
        frames = []
        while True:
            start = self._buffer.find(HEADER)
            if start < 0:
                # Preserve a split first header byte, not an unbounded noise stream.
                self._buffer[:] = b"\x55" if self._buffer[-1:] == b"\x55" else b""
                break
            if start:
                del self._buffer[:start]
            if len(self._buffer) < 4:
                break
            kind = self._buffer[3]
            size = FRAME_SIZES.get(kind)
            if size is None:
                self.format_errors += 1
                del self._buffer[0]
                continue
            if len(self._buffer) < size:
                break
            wire = bytes(self._buffer[:size])
            if wire[-1] != 0x0A:
                self.format_errors += 1
                del self._buffer[0]
                continue
            if crc16(wire[:-3]) != struct.unpack_from("<H", wire, size - 3)[0]:
                self.crc_errors += 1
                del self._buffer[0]
                continue
            del self._buffer[:size]
            if wire[2] != self.slave_id:
                self.other_devices += 1
                continue
            if kind not in (ACCEL, GYRO, EULER, QUATERNION):
                self.other_frames += 1
                continue
            count = 4 if kind == QUATERNION else 3
            values = struct.unpack_from("<" + "f" * count, wire, 4)
            if not all(math.isfinite(value) for value in values):
                self.format_errors += 1
                continue
            if kind == QUATERNION and sum(value * value for value in values) < 1e-12:
                self.format_errors += 1
                continue
            frames.append(ImuFrame(kind, values, wire[2]))
            self.frames_ok += 1
        return frames


def euler_to_quaternion(rpy_degrees: Tuple[float, ...]) -> Tuple[float, ...]:
    """Vendor ZYX Euler convention; result is ROS order (x, y, z, w)."""
    roll, pitch, yaw = (math.radians(value) for value in rpy_degrees)
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    return (sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy,
            cr * cp * cy + sr * sp * sy)


@dataclass(frozen=True)
class ImuSample:
    acceleration: Tuple[float, ...]
    angular_velocity: Tuple[float, ...]
    orientation_xyzw: Tuple[float, ...]
    orientation_source: str
    receipt_time: float
    receipt_skew: float


class SampleAssembler:
    """Join new acceleration, gyro and orientation frames within a receipt window.

    No publication until all three are present, fresh, and updated since the last
    publication. This prevents zero-filled channels and replay of cached readings.
    """

    def __init__(self, max_age: float = 0.1, max_skew: float = 0.03):
        if not (math.isfinite(max_age) and math.isfinite(max_skew)
                and 0 < max_skew <= max_age):
            raise ValueError("require 0 < max_skew <= max_age, both finite")
        self.max_age = max_age
        self.max_skew = max_skew
        self.latest: Dict[int, Tuple[ImuFrame, float, int]] = {}
        self._sequence = 0
        self._used_accel = 0
        self._used_gyro = 0
        self._used_orientation = 0

    def add(self, frame: ImuFrame, now: float) -> Optional[ImuSample]:
        self._sequence += 1
        self.latest[frame.kind] = (frame, now, self._sequence)
        return self.sample(now)

    def sample(self, now: float) -> Optional[ImuSample]:
        accel = self.latest.get(ACCEL)
        gyro = self.latest.get(GYRO)
        if accel is None or gyro is None:
            return None
        orientations = [self.latest[kind] for kind in (QUATERNION, EULER)
                        if kind in self.latest]
        # Prefer a fresh native quaternion; fall back to a fresh Euler estimate.
        for orient in orientations:
            frames = (accel, gyro, orient)
            stamps = [entry[1] for entry in frames]
            if any(now - stamp < 0 or now - stamp > self.max_age for stamp in stamps):
                continue
            if max(stamps) - min(stamps) > self.max_skew:
                continue
            if (accel[2] <= self._used_accel or gyro[2] <= self._used_gyro
                    or orient[2] <= self._used_orientation):
                continue
            if orient[0].kind == QUATERNION:
                w, x, y, z = orient[0].values
                norm = math.sqrt(w * w + x * x + y * y + z * z)
                quaternion = (x / norm, y / norm, z / norm, w / norm)
                source = "quaternion"
            else:
                quaternion = euler_to_quaternion(orient[0].values)
                source = "euler"
            self._used_accel, self._used_gyro, self._used_orientation = (
                accel[2], gyro[2], orient[2])
            return ImuSample(accel[0].values, gyro[0].values, quaternion, source,
                             max(stamps), max(stamps) - min(stamps))
        return None
