"""Bounded headless stand test using the calibrated controller and its guards."""
from __future__ import annotations

import argparse
from datetime import datetime
import json
from pathlib import Path
import sys
import time
from zoneinfo import ZoneInfo

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import numpy as np
from dog_clock import now as clock_now

from dog_link import DryRunLink
from dog_runtime import DogController, ROOT, calibration_problems, load_settings, minimum_jerk


class StandTestController(DogController):
    def return_to_start(self, goal, seconds):
        self.move_to_pose(goal, seconds, 'return')

    def move_to_pose(self, goal, seconds, name):
        goal = np.asarray(goal, dtype=float)
        if (goal.shape != (12,) or not np.isfinite(goal).all() or
                np.any(goal < self.kin.limits[:, 0]) or
                np.any(goal > self.kin.limits[:, 1])):
            raise ValueError('Pose target must contain 12 finite angles within model limits')
        # Replace the hold plan; serial writes remain on the guarded worker.
        with self._lock:
            self._active(self._generation, self._cancel)
            if not self._enabled:
                raise RuntimeError('Cannot move after a protection stop')
            q = self._model_positions(self._feedback(require_enabled=True))
            seconds = max(seconds, 1.875*float(np.abs(goal-q).max()) /
                          self.options['max_target_velocity'])
            now = clock_now()
            self._plan = dict(kind='test_pose', q0=q, goal=goal.copy(),
                              start=now, seconds=seconds, name=name)
            self._mode = name
            self._last_tick = now
        self._say(f'{name}: smooth pose transition over {seconds:.1f}s')

    def _tick(self, token, cancel):
        with self._lock:
            plan = self._plan
        if not plan or plan['kind'] != 'test_pose':
            return super()._tick(token, cancel)
        self._active(token, cancel)
        now = clock_now()
        if now-self._last_tick > self.options['max_control_gap']:
            raise RuntimeError('Pose control loop timeout')
        u = (now-plan['start'])/plan['seconds']
        position, speed = minimum_jerk(u)
        delta = plan['goal']-plan['q0']
        q = plan['q0']+delta*position
        self._send_targets(q, delta*speed/plan['seconds'], cancel)
        with self._lock:
            self._active(token, cancel)
            self._q = q.copy()
            self._last_tick = now
            if u >= 1:
                self._plan = dict(kind='hold', q=plan['goal'].copy())
                self._mode = 'hold'
        if u >= 1:
            self._say(plan['name']+': trajectory complete')


def main():
    cli = argparse.ArgumentParser(description=__doc__)
    mode = cli.add_mutually_exclusive_group()
    mode.add_argument('--execute', action='store_true')
    mode.add_argument('--dry-run', action='store_true')
    cli.add_argument('--kp', type=float, default=5.)
    cli.add_argument('--kv', type=float, default=2.)
    cli.add_argument('--stand-seconds', type=float, default=12.)
    cli.add_argument('--hold-seconds', type=float, default=5.)
    cli.add_argument('--gain-seconds', type=float, default=2.)
    cli.add_argument('--max-target-velocity', type=float, default=.3)
    cli.add_argument('--zero-first', action='store_true')
    cli.add_argument('--zero-only', action='store_true',
                     help='Return to the calibrated model zero pose and finish without standing')
    cli.add_argument('--zero-seconds', type=float, default=6.)
    cli.add_argument('--zero-tolerance', type=float, default=.05)
    cli.add_argument('--zero-settle-seconds', type=float, default=4.)
    cli.add_argument('--disable-tracking-error-guard', action='store_true')
    cli.add_argument('--trot-seconds', type=float)
    cli.add_argument('--use-current-stand-pose', action='store_true',
                     help='Start trot from the settled measured pose without the nominal stand-angle gate')
    cli.add_argument('--trot-frequency', type=float, default=1.)
    cli.add_argument('--trot-step-height', type=float, default=.004)
    cli.add_argument('--trot-ramp-down-seconds', type=float, default=1.5)
    cli.add_argument('--stable-seconds', type=float, default=1.)
    cli.add_argument('--settle-timeout', type=float, default=5.)
    args = cli.parse_args()
    if args.zero_only:
        args.zero_first = True
    if args.use_current_stand_pose and args.trot_seconds is None:
        cli.error('--use-current-stand-pose requires --trot-seconds')
    values = (args.kp, args.kv, args.stand_seconds, args.hold_seconds,
              args.gain_seconds, args.max_target_velocity, args.zero_seconds,
              args.zero_tolerance, args.zero_settle_seconds, args.trot_frequency,
              args.trot_step_height, args.trot_ramp_down_seconds,
              args.stable_seconds, args.settle_timeout)
    if args.trot_seconds is not None:
        values += (args.trot_seconds,)
        if args.zero_only:
            cli.error('--trot-seconds cannot be combined with --zero-only')
    if not all(np.isfinite(v) and v > 0 for v in values):
        cli.error('All gains and durations must be positive finite numbers')
    config, rows = load_settings()
    if args.trot_seconds is not None:
        if (args.trot_frequency > config['runtime']['gaits']['trot']['frequency'] or
                args.trot_step_height > config['sim_candidate']['step_height']):
            cli.error('Trial trot frequency and step height must not exceed the configured gait')
        config['runtime']['gaits']['trot']['frequency'] = args.trot_frequency
        config['sim_candidate']['step_height'] = args.trot_step_height
    config['motor']['kp'], config['motor']['kd'] = args.kp, args.kv
    for row in rows:
        row['approved_kp'] = row['approved_kd'] = None
    config['runtime'].update(stand_seconds=args.stand_seconds,
                             gain_ramp_seconds=args.gain_seconds,
                             max_target_velocity=min(args.max_target_velocity,
                                                     config['runtime']['max_target_velocity']))
    controller = StandTestController(config, rows,
                                     tracking_error_guard=not args.disable_tracking_error_guard,
                                     standing_pose_guard=not args.use_current_stand_pose)
    problems = calibration_problems(config, rows, controller.kin)
    plan = dict(kp=args.kp, kv=args.kv, stand_seconds=args.stand_seconds,
                hold_seconds=args.hold_seconds, gain_seconds=args.gain_seconds,
                max_target_velocity=config['runtime']['max_target_velocity'],
                zero_first=args.zero_first, zero_only=args.zero_only, zero_seconds=args.zero_seconds,
                zero_tolerance_rad=args.zero_tolerance,
                zero_settle_seconds=args.zero_settle_seconds,
                tracking_error_guard=controller.tracking_error_guard,
                standing_pose_guard=controller.standing_pose_guard,
                trot_seconds=args.trot_seconds, trot_frequency=args.trot_frequency,
                trot_step_height_m=args.trot_step_height,
                trot_ramp_down_seconds=args.trot_ramp_down_seconds,
                stable_seconds=args.stable_seconds, settle_timeout=args.settle_timeout,
                return_to_start=not args.zero_only, dry_run=args.dry_run, problems=problems)
    print(json.dumps(plan, ensure_ascii=False), flush=True)
    if problems:
        return 2
    if not args.execute and not args.dry_run:
        return 0
    if args.dry_run:
        q = (np.tile([.04, -.04, .04], 4) if args.zero_first else
             controller.kin.stand_q+np.tile([.02, -.02, .02], 4))
        controller.link = DryRunLink([r['direction']*q[i]+r['zero_rad']
                                     for i, r in enumerate(rows)])

    started = clock_now()
    report = dict(plan=plan, started_at=datetime.now(ZoneInfo('Asia/Shanghai')).isoformat(),
                  samples=[], result='incomplete', error=None)
    seen_events = set()
    stage = 'connect'

    def sample():
        status = controller.status()
        for event in status['events']:
            if event not in seen_events:
                print(event, flush=True)
                seen_events.add(event)
        with controller._lock:
            model_target = controller._q.copy()
        report['samples'].append(dict(elapsed=clock_now()-started, stage=stage,
                                     enabled=status['enabled'], mode=status['mode'],
                                     feedback_age=status['feedback_age'],
                                     feedback=status['feedback'],
                                     model_target_q_rad=model_target.tolist()))
        if controller.link.error:
            raise RuntimeError(controller.link.error)
        if status['disabling'] or status['disable_state'] in ('confirmed', 'unconfirmed'):
            raise RuntimeError(status['message'])
        return status

    def wait_for(predicate, timeout):
        deadline = clock_now()+timeout
        while True:
            status = sample()
            if predicate(status):
                return status
            if clock_now() > deadline:
                raise RuntimeError(f'Timeout during {stage}')
            time.sleep(.02)

    try:
        controller.start()
        wait_for(lambda s: s['connected'] and s['feedback'] is not None and
                 s['feedback_age'] < config['runtime']['feedback_timeout'], 3.)
        q_start = controller._model_positions(controller._feedback())
        report['start_model_q_rad'] = q_start.tolist()
        report['stand_model_q_rad'] = controller.kin.stand_q.tolist()
        stage = 'enable'
        controller.request('enable')
        wait_for(lambda s: s['enabled'], args.gain_seconds+4.)
        if args.zero_first:
            stage = 'zero'
            zero = np.zeros(12)
            controller.move_to_pose(zero, args.zero_seconds, 'zero')
            with controller._lock:
                zero_duration = controller._plan['seconds']
            report['actual_zero_seconds'] = zero_duration
            wait_for(lambda s: s['mode'] == 'hold', zero_duration+2.)
            stage = 'zero_settle'
            deadline = clock_now()+args.zero_settle_seconds
            settled_since = None
            while True:
                sample()
                zero_feedback = controller._feedback(require_enabled=True)
                measured_zero = controller._model_positions(zero_feedback)
                zero_error = float(np.abs(measured_zero).max())
                still = max(abs(m['velocity']) for m in zero_feedback) <= .1
                report['zero_model_q_rad'] = measured_zero.tolist()
                report['zero_max_error_rad'] = zero_error
                now = clock_now()
                if zero_error <= args.zero_tolerance and still:
                    if settled_since is None:
                        settled_since = now
                    if now-settled_since >= .5:
                        break
                else:
                    settled_since = None
                if now > deadline:
                    raise RuntimeError(f'Zero pose not confirmed: maximum error {zero_error:.4f} rad; '
                                       f'tolerance {args.zero_tolerance:.4f} rad; stand was not started')
                time.sleep(.02)
            report['zero_confirmed'] = True
            print(f'Zero pose confirmed; max joint error {zero_error:.4f} rad', flush=True)
            # Finish this test at the requested zero pose.
            q_start = zero
        if args.zero_only:
            report['result'] = 'zero_pose_completed'
        else:
            stage = 'stand'
            controller.request('stand')
            wait_for(lambda s: s['mode'] == 'stand', 2.)
            with controller._lock:
                actual_duration = controller._plan['seconds']
            report['actual_stand_seconds'] = actual_duration
            wait_for(lambda s: controller._plan is not None and
                     controller._plan['kind'] == 'hold', actual_duration+2.)
            stage = 'standing_hold'
            held_until = clock_now()+args.hold_seconds
            stable_since = None
            while clock_now() < held_until:
                status = sample()
                now = clock_now()
                if max(abs(m['velocity']) for m in status['feedback']) <= .1:
                    if stable_since is None:
                        stable_since = now
                else:
                    stable_since = None
                time.sleep(.02)
            if args.trot_seconds is not None:
                stage = 'standing_settle'
                settle_deadline = clock_now()+args.settle_timeout
                while stable_since is None or clock_now()-stable_since < args.stable_seconds:
                    status = sample()
                    now = clock_now()
                    if max(abs(m['velocity']) for m in status['feedback']) <= .1:
                        if stable_since is None:
                            stable_since = now
                    else:
                        stable_since = None
                    if now > settle_deadline:
                        raise RuntimeError('Joint feedback did not settle; trot was not started')
                    time.sleep(.02)
                report['joint_velocity_settled'] = True
            actual_q = controller._model_positions(controller._feedback(require_enabled=True))
            error = float(np.abs(actual_q-controller.kin.stand_q).max())
            report['standing_model_q_rad'] = actual_q.tolist()
            report['standing_max_error_rad'] = error
            print(f'Standing hold completed; max joint error {error:.4f} rad', flush=True)
            trot_completed = False
            if args.trot_seconds is not None:
                report['trot_started'] = False
                if (controller.standing_pose_guard and
                        error > config['runtime']['standing_tolerance_rad']):
                    report['trot_skip_reason'] = (
                        f'Stand error {error:.4f} rad exceeds existing trot entry tolerance '
                        f'{config["runtime"]["standing_tolerance_rad"]:.4f} rad')
                    print('Trot not started: '+report['trot_skip_reason'], flush=True)
                else:
                    stage = 'trot'
                    controller.request('trot')
                    wait_for(lambda s: s['mode'] == 'trot', 2.)
                    report['trot_started'] = True
                    trot_started = clock_now()
                    while clock_now()-trot_started < args.trot_seconds:
                        sample()
                        time.sleep(.02)
                    report['actual_trot_seconds'] = clock_now()-trot_started
                    stage = 'trot_ramp_down'
                    with controller._lock:
                        gait = controller._plan['gait']
                    ramp_start = clock_now()
                    while True:
                        fraction = min(1., (clock_now()-ramp_start)/args.trot_ramp_down_seconds)
                        fade, _ = minimum_jerk(fraction)
                        gait.config['step_height'] = args.trot_step_height*(1.-fade)
                        sample()
                        if fraction >= 1:
                            break
                        time.sleep(.02)
                    trot_completed = True
                    print('Trot interval and amplitude ramp-down completed', flush=True)
            stage = 'return'
            controller.return_to_start(q_start, args.stand_seconds)
            with controller._lock:
                return_duration = controller._plan['seconds']
            wait_for(lambda s: s['mode'] == 'hold', return_duration+2.)
            if args.trot_seconds is not None:
                report['result'] = 'trot_completed' if trot_completed else 'trot_not_started_pose'
            else:
                report['result'] = ('stand_hold_completed' if error <= config['runtime']['standing_tolerance_rad']
                                    else 'stand_tracking_incomplete')
        stage = 'gain_down'
        # Stay on the regular guarded hold controller while reducing effort.
        ramp_start = clock_now()
        while clock_now()-ramp_start < args.gain_seconds:
            fraction = max(0., 1.-(clock_now()-ramp_start)/args.gain_seconds)
            with controller._lock:
                controller.kp[:] = args.kp*fraction
                controller.kd[:] = args.kv*fraction
            sample()
            time.sleep(.02)
    except (Exception, KeyboardInterrupt) as error:
        report.update(result='stopped', error=f'{type(error).__name__}: {error}', stopped_stage=stage)
        print(report['error'], flush=True)
    finally:
        controller.disable('Headless stand test finished')
        deadline = clock_now()+config['runtime']['disable_timeout']+1.
        while controller.status()['disabling'] and clock_now() < deadline:
            time.sleep(.02)
        final = controller.status()
        report['final_status'] = final
        report['duration_seconds'] = clock_now()-started
        controller.close()
        folder = ROOT/'test_logs'
        folder.mkdir(exist_ok=True)
        stamp = datetime.now(ZoneInfo('Asia/Shanghai')).strftime('%Y%m%d_%H%M%S')
        prefix = ('zero_pose' if args.zero_only else
                  'stand_trot' if args.trot_seconds is not None else 'slow_stand')
        path = folder/f'{prefix}_{"dry_run" if args.dry_run else "real"}_{stamp}.json'
        path.write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding='utf-8')
        print(json.dumps(dict(result=report['result'], error=report['error'],
                              disable_state=final['disable_state'], log=str(path)),
                         ensure_ascii=False), flush=True)
    return 0 if report['result'] in ('stand_hold_completed', 'zero_pose_completed', 'trot_completed') and final['disable_state'] == 'confirmed' else 1


if __name__ == '__main__':
    raise SystemExit(main())
