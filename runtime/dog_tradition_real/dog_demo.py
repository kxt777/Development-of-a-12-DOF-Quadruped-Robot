"""No-ROS deployment preparation: offline MIT preview and explicit passive reads.

This release deliberately has no serial.write or motor-enable path.
"""
import argparse
import csv
from dataclasses import asdict
import importlib.util
import json
import math
from pathlib import Path
import time

import numpy as np

from dog_gait import DogGait
from dog_kinematics import DogKinematics
from dog_protocol import FeedbackParser, PACKET_SIZE, encode_mit

ROOT = Path(__file__).resolve().parent


def load_config(path=ROOT/'dog_config.json'):
    return json.loads(Path(path).read_text(encoding='utf-8'))


def preview(config, mode, seconds, output=None):
    c = config['sim_candidate']
    kin = DogKinematics()
    gait = DogGait(c, kin.forward(kin.stand_q), mode)
    dt = c['control_dt']
    if not math.isfinite(seconds) or not 0 < seconds <= 300:
        raise ValueError('Preview seconds must be in (0,300]')
    count = int(round(seconds/dt))+1
    q = kin.stand_q.copy()
    maximum_error = 0.
    q_history, dq_history, target_history = [], [], []
    for index in range(count):
        targets = gait.targets(index*dt)
        previous = q.copy()
        q, errors = kin.solve(targets, previous)
        maximum_error = max(maximum_error, float(errors.max()))
        dq = (q-previous)/dt
        if errors.max() > 0.001:
            raise ValueError('IK error exceeds 1 mm; no preview should be used')
        if np.abs(dq).max() > c['joint_velocity_target_limit']:
            raise ValueError('Preview exceeds simulation joint target velocity limit')
        # MODEL angles + candidate IDs; NOT calibrated encoder commands.
        records = {i+1: (q[i], dq[i], c['kp'][i%3], c['kd'][i%3], 0.)
                   for i in range(12)}
        if len(encode_mit(records)) != PACKET_SIZE:
            raise AssertionError('Invalid MIT encoding')
        q_history.append(q.copy())
        dq_history.append(dq.copy())
        target_history.append(targets)
    q_history, dq_history = np.array(q_history), np.array(dq_history)
    result = {'mode': mode, 'seconds': seconds, 'steps': count,
              'io': 'OFFLINE ONLY; no serial port opened',
              'frequency_hz': gait.config['frequency'],
              'max_ik_error_m': maximum_error,
              'max_joint_target_speed_rad_s': float(np.abs(dq_history).max()),
              'peak_joint_excursion_rad': float(np.ptp(q_history, axis=0).max()),
              'mit_packet_bytes': PACKET_SIZE, 'tau_ff': 0.,
              'note': 'Kinematic candidate, no simulation balance or real torque estimate'}
    if output:
        path = Path(output)
        if not path.name.startswith('dog_'):
            raise ValueError('Output filename must start with dog_')
        with path.open('x', newline='', encoding='utf-8') as handle:
            writer = csv.writer(handle)
            writer.writerow(['time_s'] + ['model_q_'+n for n in kin.joint_names] +
                            ['model_dq_'+n for n in kin.joint_names])
            for i in range(count):
                writer.writerow([i*dt, *q_history[i], *dq_history[i]])
        result['csv'] = str(path)
    return result


def check(config):
    from dog_runtime import calibration_problems, load_settings
    ports = []
    if importlib.util.find_spec('serial'):
        from serial.tools import list_ports
        ports = [{'device': p.device, 'description': p.description, 'serial_number': p.serial_number}
                 for p in list_ports.comports()]
    kin = DogKinematics()
    problems = calibration_problems(config, load_settings()[1], kin)
    return {'actuation_available': not problems, 'serial_ports_opened': 0,
            'dependencies': {name: bool(importlib.util.find_spec(name))
                             for name in ('numpy', 'serial')},
            'ports': ports, 'configured_motor_port': config['motor']['port'],
            'configured_imu_port': config['imu']['port'],
            'stand_feet_body_m': kin.forward(kin.stand_q).tolist(),
            'pending': problems, 'runtime_entry': 'dogui.py'}


def monitor(config, device, seconds, port=None):
    """Read existing unsolicited streams; no ping, setup, enable or write commands.

    Opening a serial device can toggle control lines. Must be explicitly requested;
    do not use this while another process owns the port or a robot is enabled.
    """
    import serial
    selected = config[device]
    port = port or selected['port']
    if not port:
        raise ValueError('Identify an explicit stable serial-by-id port first')
    if device == 'motor' and port == config['imu']['port']:
        raise ValueError('Motor port equals configured IMU port')
    if not math.isfinite(seconds) or not 0 < seconds <= 60:
        raise ValueError('Monitor seconds must be in (0,60]')
    if device == 'imu':
        from dog_imu import ImuParser, SampleAssembler
        parser, assembler = ImuParser(), SampleAssembler()
    else:
        parser = FeedbackParser()
    frames, byte_count, latest = 0, 0, None
    handle = serial.Serial(port=None, baudrate=selected['baudrate'], timeout=.02,
                           exclusive=True)
    handle.dtr, handle.rts = False, False
    handle.port = port
    with handle:
        start = time.monotonic()
        while time.monotonic()-start < seconds:
            data = handle.read(min(max(handle.in_waiting, 1), 4096))
            byte_count += len(data)
            for frame in parser.append(data):
                frames += 1
                if device == 'imu':
                    sample = assembler.add(frame, time.monotonic())
                    if sample:
                        latest = asdict(sample)
                else:
                    latest = frame
    return {'device': device, 'port': port, 'received_bytes': byte_count,
            'parsed_frames': frames, 'bytes_sent': 0, 'latest': latest,
            'notice': ('USB receipt does not prove per-motor CAN freshness' if device == 'motor'
                       else 'Sensor-frame IMU only; body mount transform not calibrated')}


def main():
    cli = argparse.ArgumentParser(description=__doc__)
    action = cli.add_mutually_exclusive_group()
    action.add_argument('--check', action='store_true')
    action.add_argument('--preview', action='store_true')
    action.add_argument('--monitor', choices=['motor', 'imu'])
    action.add_argument('--run', action='store_true', help='Use dogstand.py/dogwalk.py/dogtrot.py instead')
    cli.add_argument('--mode', choices=['stand', 'trot', 'walk'], default='stand')
    cli.add_argument('--seconds', type=float, default=8.)
    cli.add_argument('--csv', help='New dog_-prefixed offline CSV; never overwrites')
    cli.add_argument('--port', help='Explicit monitor port only')
    args = cli.parse_args()
    config = load_config()
    try:
        if args.run:
            raise ValueError('This file is for offline diagnostics. Start dog'+args.mode+
                             '.py for the desktop controller, or dogui.py for manual selection.')
        if args.preview:
            result = preview(config, args.mode, args.seconds, args.csv)
        elif args.monitor:
            result = monitor(config, args.monitor, args.seconds, args.port)
        else:
            result = check(config)
        print(json.dumps(result, indent=2, ensure_ascii=False))
    except (ValueError, OSError) as error:
        cli.exit(2, str(error)+'\n')


if __name__ == '__main__':
    main()
