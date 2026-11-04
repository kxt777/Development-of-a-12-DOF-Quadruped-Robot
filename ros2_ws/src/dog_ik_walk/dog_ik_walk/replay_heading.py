"""Offline recorded-input replay and MIT encoding check; never opens motor ports."""
import argparse
import csv
import hashlib
import json
import math
from pathlib import Path
import numpy as np
from dog_runtime import load_settings
from dog_link import DryRunLink
from dog_protocol import encode_model_mit
from .controller import WriteGate
from .imu_controller import ImuWalkController


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--deployment',type=Path,required=True)
    parser.add_argument('--recording',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    deployment=json.loads(args.deployment.read_text())
    report=json.loads(args.recording.read_text())
    config,calibration=load_settings(Path(deployment['runtime_root'])/'dog_tradition_real/dog_config.json')
    config['allow_actuation']=False
    raw=DryRunLink([c['zero_rad'] for c in calibration])
    controller=ImuWalkController(config,calibration,deployment,WriteGate(raw,False))
    heading=controller.heading
    heading.activate_command(report['heading_command'])
    rows=[];previous=None;max_dq=0.;max_derivative=0.
    for record in report['commands']:
        old=record.get('heading')
        if not old:continue
        stamp=old['imu_receipt_monotonic']+old['imu_age_s']
        dt=controller.dt if previous is None else stamp-previous
        if dt<=0:raise ValueError('Recording timestamps must increase')
        previous=stamp
        if old['imu_available']:
            heading.latest=dict(yaw=old['heading_actual_rad'],yaw_rate=old['yaw_rate_measured_rad_s'],
                receipt_monotonic=old['imu_receipt_monotonic'])
        rate,derivative=heading.step(dt,stamp)
        elapsed=record['time_monotonic']-report['walk_start_monotonic']
        q,dq=controller.sample_targets(elapsed,rate,derivative)
        encode_model_mit(q,dq,controller.kp,controller.kd,np.zeros(12),calibration,config['motor']['mapping_ranges'])
        max_dq=max(max_dq,float(abs(dq).max()));max_derivative=max(max_derivative,abs(derivative))
        rows.append(dict(t_s=elapsed,error_deg=math.degrees(old['heading_error_rad']),
            gyro_deg_s=math.degrees(old['yaw_rate_measured_rad_s']),
            old_command_deg_s=math.degrees(old['yaw_rate_command_rad_s']),
            new_command_deg_s=math.degrees(rate),
            new_requested_deg_s=math.degrees(heading.detail['yaw_rate_requested_rad_s']),
            filtered_gyro_deg_s=math.degrees(heading.detail['yaw_rate_filtered_rad_s']),
            damping_deg_s=math.degrees(heading.detail['damping_yaw_rate_rad_s'])))
    # Include fast direction reversals, not only the input sequence that happened.
    envelope=deployment['heading']['trajectory_yaw_rate_rad_s']
    tau=min(deployment['heading']['command_filter_seconds'],deployment['heading']['straight_response']['command_filter_seconds'])
    slew=2*envelope/(tau+controller.dt)
    sweep_max=0.
    for t in controller.bank_times:
        for rate in (-envelope,0.,envelope):
            for derivative in (-slew,slew):
                q,dq=controller.sample_targets(t,rate,derivative)
                encode_model_mit(q,dq,controller.kp,controller.kd,np.zeros(12),calibration,config['motor']['mapping_ranges'])
                sweep_max=max(sweep_max,float(abs(dq).max()))
    assert not raw.packets and controller.link.writes==0
    errors=np.array([r['error_deg'] for r in rows]);old=np.array([r['old_command_deg_s'] for r in rows]);new=np.array([r['new_command_deg_s'] for r in rows])
    selected=(abs(errors)>=3)&(abs(errors)<=5)
    summary=dict(recording=str(args.recording),recording_sha256=hashlib.sha256(args.recording.read_bytes()).hexdigest(),
        scope='Counterfactual commands on recorded IMU inputs; no closed-loop hardware performance claim.',
        records=len(rows),examples=[next(r for r in rows if abs(r['error_deg'])>=threshold) for threshold in (3,5,10)],
        mean_abs_command_error3to5_deg_s=dict(old=float(np.mean(abs(old[selected]))),new=float(np.mean(abs(new[selected])))),
        opposite_command_samples_error_above3=dict(old=int(sum((abs(errors)>=3)&(old*errors<0))),new=int(sum((abs(errors)>=3)&(new*errors<0)))),
        replay_max_target_joint_speed_rad_s=max_dq,replay_max_yaw_command_derivative_rad_s2=max_derivative,
        reversal_sweep_max_target_joint_speed_rad_s=sweep_max,
        reference_preserved=heading.reference==report['heading_command']['target_yaw_rad'],
        motor_writes=controller.link.writes,trajectory_preflight=controller.preflight)
    args.output.mkdir(parents=True,exist_ok=True)
    (args.output/'summary.json').write_text(json.dumps(summary,indent=2))
    with (args.output/'commands.csv').open('w',newline='') as f:
        writer=csv.DictWriter(f,fieldnames=list(rows[0]));writer.writeheader();writer.writerows(rows)
    print(json.dumps(summary,indent=2))


if __name__=='__main__':main()
