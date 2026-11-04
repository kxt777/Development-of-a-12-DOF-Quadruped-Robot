"""Passive IMU capture on Windows/Linux without ROS or configuration writes."""

import argparse
from collections import Counter
from dataclasses import asdict
import json
import math
import time

import serial
from serial.tools import list_ports

from .protocol import ACCEL, GYRO, EULER, QUATERNION, ImuParser, SampleAssembler


def main(args=None):
    cli = argparse.ArgumentParser(description=__doc__)
    cli.add_argument("--list-ports", action="store_true")
    cli.add_argument("--port", help="Explicit IMU port, e.g. COM5 or /dev/ttyACM1")
    cli.add_argument("--baud", type=int, default=921600)
    cli.add_argument("--slave-id", type=int, default=1)
    cli.add_argument("--crc-mode", choices=("ccitt", "legacy_v1"), default="ccitt")
    cli.add_argument("--seconds", type=float, default=5.0)
    opts = cli.parse_args(args)
    if opts.list_ports:
        print(json.dumps([{"port": port.device, "description": port.description,
                           "hwid": port.hwid} for port in list_ports.comports()], indent=2))
        return 0
    if not opts.port:
        cli.error("--port is required; use --list-ports to identify the IMU")
    if not math.isfinite(opts.seconds) or opts.seconds <= 0 or opts.baud <= 0:
        cli.error("--seconds and --baud must be positive")
    parser = ImuParser(opts.slave_id, opts.crc_mode)
    assembler = SampleAssembler()
    counts = Counter()
    last_sample = None
    samples = 0
    received = 0
    started = time.monotonic()
    result = {"port": opts.port, "baud": opts.baud, "bytes_sent": 0}
    try:
        with serial.Serial(opts.port, opts.baud, timeout=0.02) as handle:
            handle.reset_input_buffer()
            started = time.monotonic()
            while time.monotonic() - started < opts.seconds:
                data = handle.read(min(handle.in_waiting, 65536) or 1)
                now = time.monotonic()
                received += len(data)
                for frame in parser.append(data):
                    counts[frame.kind] += 1
                    sample = assembler.add(frame, now)
                    if sample is not None:
                        last_sample = sample
                        samples += 1
    except (serial.SerialException, OSError) as exc:
        result["error"] = str(exc)
    elapsed = time.monotonic() - started
    labels = {ACCEL: "acceleration", GYRO: "angular_velocity",
              EULER: "euler", QUATERNION: "quaternion"}
    result.update({
        "duration_s": round(elapsed, 3), "bytes_received": received,
        "frames_by_type": {labels[kind]: counts[kind] for kind in labels},
        "frame_hz_by_type": {labels[kind]: round(counts[kind] / max(elapsed, 1e-9), 2)
                             for kind in labels},
        "crc_errors": parser.crc_errors, "format_errors": parser.format_errors,
        "other_devices": parser.other_devices, "other_frames": parser.other_frames,
        "complete_samples": samples,
        "sample_hz": round(samples / max(elapsed, 1e-9), 2),
        "units": {"acceleration": "m/s^2 (includes gravity)",
                  "angular_velocity": "rad/s", "orientation": "ROS x,y,z,w"},
    })
    fresh = last_sample is not None and time.monotonic() - last_sample.receipt_time <= assembler.max_age
    result["last_sample_fresh"] = fresh
    if last_sample is not None:
        result["last_sample"] = asdict(last_sample)
        result["acceleration_norm_m_s2"] = math.sqrt(sum(value * value for value in last_sample.acceleration))
    else:
        result["hint"] = "Enable USB accel + gyro + Euler or quaternion in the DAMIAO tool; check slave ID and firmware."
    print(json.dumps(result, indent=2))
    return 0 if fresh and "error" not in result else 2


if __name__ == "__main__":
    raise SystemExit(main())
