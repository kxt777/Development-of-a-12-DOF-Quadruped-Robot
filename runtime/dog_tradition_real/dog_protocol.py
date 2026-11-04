"""Existing H723 DM12 USB MIT wire format. Encoding never performs I/O."""
import math
import struct

MAGIC = b'DM12'
RECORD = struct.Struct('<BBBfffff')
PACKET_SIZE = 5 + 12*RECORD.size


def encode_mit(records):
    """Explicit motor-ID keyed (q, dq, kp, kd, tau_ff); absent slots are invalid.

    Firmware valid=0 means no new command, NOT disable. This is not a stop packet.
    Host format has no CRC and cannot prove actual CAN feedback freshness.
    """
    if any(isinstance(i, bool) or not isinstance(i, int) or not 1 <= i <= 12
           for i in records):
        raise ValueError('Motor IDs must be integers 1..12')
    packet = bytearray(MAGIC+b'\x10')
    for motor_id in range(1, 13):
        values = tuple(records.get(motor_id, (0.,)*5))
        if len(values) != 5 or not all(math.isfinite(v) for v in values):
            raise ValueError('Expected five finite MIT fields')
        if not 0 <= values[2] <= 500 or not 0 <= values[3] <= 5:
            raise ValueError('Gain exceeds existing firmware protocol range')
        packet.extend(RECORD.pack(int(motor_id in records), motor_id, 1, *values))
    return bytes(packet)


def model_to_motor(q, dq, tau_ff, calibration):
    """Output-radian, unit-ratio mapping; gains do not change under sign reversal."""
    rows = []
    if not len(q) == len(dq) == len(tau_ff) == len(calibration) == 12:
        raise ValueError('Expected 12 joints and calibration rows')
    for i, row in enumerate(calibration):
        sign, zero = row['direction'], row['zero_rad']
        if sign not in (-1, 1) or zero is None or not math.isfinite(zero):
            raise ValueError('Motor direction/zero calibration is incomplete')
        rows.append((sign*q[i]+zero, sign*dq[i], sign*tau_ff[i]))
    return rows


def encode_model_mit(q, dq, kp, kd, tau_ff, calibration, ranges):
    """Native calibrated MIT fields; no guessed total-torque/velocity gate.

    Reject unrepresentable fields rather than silently letting firmware clamp
    them. These are wire-mapping bounds, not motor continuous/peak ratings.
    """
    for values in (q, dq, kp, kd, tau_ff):
        if len(values) != 12 or not all(math.isfinite(v) for v in values):
            raise ValueError('Expected 12 finite values for each MIT field')
    mapped = model_to_motor(q, dq, tau_ff, calibration)
    records = {}
    for i, (p, v, t) in enumerate(mapped):
        if (abs(p) > ranges['position_rad'] or abs(v) > ranges['velocity_rad_s']
                or abs(t) > ranges['torque_nm']):
            raise ValueError(f'ID {calibration[i]["can_id"]}: MIT field exceeds wire mapping')
        records[calibration[i]['can_id']] = (p, v, kp[i], kd[i], t)
    return encode_mit(records)


class FeedbackParser:
    def __init__(self):
        self.buffer = bytearray()

    def append(self, data):
        self.buffer.extend(data)
        frames = []
        while True:
            start = self.buffer.find(MAGIC)
            if start < 0:
                del self.buffer[:-3]
                return frames
            del self.buffer[:start]
            if len(self.buffer) < 5:
                return frames
            if self.buffer[4] != 0x11:
                del self.buffer[0]
                continue
            if len(self.buffer) < PACKET_SIZE:
                return frames
            rows = [RECORD.unpack_from(self.buffer, 5+i*RECORD.size) for i in range(12)]
            valid = all(r[0] in (0, 1) and r[1] == i+1 and
                        all(math.isfinite(v) for v in r[3:]) for i, r in enumerate(rows))
            if not valid:
                del self.buffer[0]
                continue
            del self.buffer[:PACKET_SIZE]
            frames.append([{'valid': bool(r[0]), 'id': r[1], 'state': r[2],
                            'position': r[3], 'velocity': r[4], 'torque': r[5],
                            'tmos': r[6], 'tcoil': r[7]} for r in rows])
