"""Open-loop simulation reference replay; actuator feedback is logged, not faked.

The MIT joint PD loop still executes in each motor. The simulation body balance
loop is frozen in the recorded reference. No IMU or MuJoCo runtime is required.
"""
import hashlib
import json
from pathlib import Path
import time
from dog_clock import now as clock_now

import numpy as np

from dog_link import Cancelled
from dog_protocol import encode_model_mit
from dog_runtime import DogController, minimum_jerk
from tools.dog_slow_stand_test import StandTestController


class RecordedTrajectory:
    def __init__(self, path):
        self.path = Path(path).resolve()
        with np.load(self.path, allow_pickle=False) as data:
            self.commands = np.array(data['commands'], dtype=float)
            self.kp = np.array(data['kp'], dtype=float)
            self.kd = np.array(data['kd'], dtype=float)
            self.source_sha256 = str(data['source_sha256'])
            self.summary = json.loads(str(data['summary_json']))
        if (self.commands.ndim != 2 or self.commands.shape[1] != 37
                or len(self.commands) < 2 or not np.isfinite(self.commands).all()
                or self.commands[0, 0] != 0 or np.any(np.diff(self.commands[:, 0]) <= 0)):
            raise ValueError('Reference must contain ordered finite time/q/dq/FF samples from t=0')
        if (self.kp.shape != (12,) or self.kd.shape != (12,)
                or not np.isfinite(self.kp).all() or not np.isfinite(self.kd).all()
                or np.any(self.kp <= 0) or np.any(self.kp > 500)
                or np.any(self.kd <= 0) or np.any(self.kd > 5)):
            raise ValueError('Reference MIT gains are invalid')
        if (self.summary['stop_reason'] != 'completed'
                or self.summary['name'] != 'original_5ms'
                or self.summary['model']['source_sha256'] != self.source_sha256):
            raise ValueError('Replay requires a completed original simulation reference')
        self.duration = float(self.commands[-1, 0])

    def sample(self, elapsed):
        if not np.isfinite(elapsed) or elapsed < 0 or elapsed > self.duration:
            raise ValueError('Requested time is outside recorded reference')
        index = min(max(int(np.searchsorted(self.commands[:, 0], elapsed, side='right'))-1, 0),
                    len(self.commands)-2)
        first, second = self.commands[index:index+2]
        fraction = (elapsed-first[0])/(second[0]-first[0])
        values = first[1:]*(1-fraction) + second[1:]*fraction
        return values[:12].copy(), values[12:24].copy(), values[24:].copy()

    def preflight(self, config, calibration):
        source = Path(__file__).resolve().parents[1]/'rl_training/dog/dog.xml'
        if (hashlib.sha256(source.read_bytes()).hexdigest() != self.source_sha256
                or any(r.get('source_xml_sha256', self.source_sha256) != self.source_sha256
                       for r in calibration)):
            raise ValueError('Reference CAD model changed; regenerate the simulation recording')
        ranges = config['motor']['mapping_ranges']
        for row in self.commands:
            encode_model_mit(row[1:13], row[13:25], self.kp, self.kd, row[25:37], calibration, ranges)


class ReplayController(StandTestController):
    def __init__(self, config, calibration, trajectory, seconds, *, replay_kp=None,
                 replay_kd=None, replay_gain_seconds=2., **kwargs):
        self.trajectory = trajectory
        if not np.isfinite(seconds) or not 0 < seconds <= trajectory.duration:
            raise ValueError('Replay duration is outside recorded reference')
        self.replay_seconds = seconds
        self.last_command = None
        self.command_history = []
        self._command_sequence = 0
        self.replay_kp = trajectory.kp.copy() if replay_kp is None else np.full(12, float(replay_kp))
        self.replay_kd = trajectory.kd.copy() if replay_kd is None else np.full(12, float(replay_kd))
        if (not np.isfinite(self.replay_kp).all() or not np.isfinite(self.replay_kd).all()
                or np.any(self.replay_kp <= 0) or np.any(self.replay_kp > 500)
                or np.any(self.replay_kd <= 0) or np.any(self.replay_kd > 5)
                or not np.isfinite(replay_gain_seconds) or replay_gain_seconds <= 0):
            raise ValueError('Replay gains must fit MIT protocol; gain duration must be positive')
        self.replay_gain_seconds = replay_gain_seconds
        self._replay_gain_exit = None
        super().__init__(config, calibration, tracking_error_guard=False,
                         standing_pose_guard=False, **kwargs)
        if not np.array_equal(self.kp, trajectory.kp) or not np.array_equal(self.kd, trajectory.kd):
            raise ValueError('Motor gains differ from recorded simulation gains')
        trajectory.preflight(config, calibration)

    def _feedback(self, require_enabled=False):
        # Keep communication/data validity and driver error handling. No project
        # torque, speed, temperature or tracking-error numerical stop thresholds.
        if self.link.error:
            raise RuntimeError('Feedback reader failed: '+self.link.error)
        rows, stamp, _ = self.link.snapshot()
        if rows is None or clock_now()-stamp > self.options['feedback_timeout']:
            raise RuntimeError('USB feedback missing/stale')
        if len(rows) != 12 or {r['id'] for r in rows} != set(range(1, 13)):
            raise RuntimeError('Incomplete motor feedback IDs')
        for row in rows:
            if not row['valid'] or row['state'] not in (0, 1):
                raise RuntimeError(f'ID {row["id"]}: driver fault/invalid state {row["state"]}')
            if require_enabled and row['state'] != 1:
                raise RuntimeError(f'ID {row["id"]}: motor no longer enabled')
            if not all(np.isfinite(row[k]) for k in ('position', 'velocity', 'torque', 'tmos', 'tcoil')):
                raise RuntimeError('Nonfinite feedback')
        return rows

    def _send_targets(self, q, dq, cancel, *, gain=1., tau_ff=None, reference_time=None):
        if cancel.is_set():
            raise Cancelled('Replay cancelled')
        rows = self._feedback(require_enabled=True)
        by_id = {r['id']: r for r in rows}
        actual_q = self._model_positions(rows)
        actual_dq = np.array([r['direction']*by_id[r['can_id']]['velocity'] for r in self.calibration])
        actual_torque = np.array([r['direction']*by_id[r['can_id']]['torque'] for r in self.calibration])
        ff = np.zeros(12) if tau_ff is None else np.asarray(tau_ff, dtype=float)
        kp, kd = self.kp*gain, self.kd*gain
        if reference_time is not None:
            fade, _ = minimum_jerk(reference_time/self.replay_gain_seconds)
            kp = kp + fade*(self.replay_kp*gain-kp)
            kd = kd + fade*(self.replay_kd*gain-kd)
        elif self._replay_gain_exit is not None:
            exit_start, exit_kp, exit_kd = self._replay_gain_exit
            elapsed = clock_now()-exit_start
            fade, _ = minimum_jerk(elapsed/self.replay_gain_seconds)
            kp = kp + (1-fade)*(exit_kp-kp)
            kd = kd + (1-fade)*(exit_kd-kd)
            if elapsed >= self.replay_gain_seconds:
                self._replay_gain_exit = None
        packet = encode_model_mit(q, dq, kp, kd, ff, self.calibration,
                                  self.config['motor']['mapping_ranges'])
        p_term, d_term = kp*(q-actual_q), kd*(dq-actual_dq)
        diagnostic = dict(time_monotonic=clock_now(), reference_time_s=reference_time,
                          q_des=np.asarray(q).tolist(),
                          dq_des=np.asarray(dq).tolist(), kp=kp.tolist(), kd=kd.tolist(),
                          tau_ff=ff.tolist(), q_actual=actual_q.tolist(), dq_actual=actual_dq.tolist(),
                          measured_torque_model_nm=actual_torque.tolist(), p_term_nm=p_term.tolist(),
                          d_term_nm=d_term.tolist(), estimated_total_nm=(p_term+d_term+ff).tolist())
        self.link.send(packet, cancel)
        with self._lock:
            self._command_sequence += 1
            diagnostic['command_sequence'] = self._command_sequence
            self.last_command = diagnostic
            self.command_history.append(diagnostic)

    def status(self):
        result = super().status()
        with self._lock:
            result['last_command'] = self.last_command
        result['controller_kind'] = 'recorded simulation reference; no live body-state balance'
        return result

    def _begin_motion(self, action, token, cancel):
        if action != 'trot':
            return super()._begin_motion(action, token, cancel)
        self._active(token, cancel)
        if not self._enabled:
            raise RuntimeError('Enable before replay')
        self._feedback(require_enabled=True)
        now = clock_now()
        with self._lock:
            self._active(token, cancel)
            self._plan = dict(kind='replay', start=now)
            self._replay_gain_exit = None
            self._mode = 'trot'
            self._last_tick = now
        self._say('trot: replaying simulation q/dq/Kp/Kv/FF through calibrated mapping', token)

    def _tick(self, token, cancel):
        with self._lock:
            plan = self._plan
        if not plan or plan['kind'] != 'replay':
            return super()._tick(token, cancel)
        self._active(token, cancel)
        now = clock_now()
        if now-self._last_tick > self.options['max_control_gap']:
            raise RuntimeError('Replay control loop stalled')
        elapsed = min(now-plan['start'], self.replay_seconds)
        q, dq, ff = self.trajectory.sample(elapsed)
        self._send_targets(q, dq, cancel, tau_ff=ff, reference_time=elapsed)
        with self._lock:
            self._active(token, cancel)
            self._q = q.copy()
            self._last_tick = now
            if elapsed >= self.replay_seconds:
                self._replay_gain_exit = (now, np.array(self.last_command['kp']),
                                          np.array(self.last_command['kd']))
                self._plan = dict(kind='hold', q=q.copy())
                self._mode = 'replay_done'
