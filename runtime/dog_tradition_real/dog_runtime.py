"""Shared stand/walk/trot controller; emergency disable bypasses the action queue."""
from __future__ import annotations

from collections import deque
import json
import math
from pathlib import Path
import threading
import time
from dog_clock import now as clock_now

import numpy as np

from dog_gait import DogGait
from dog_kinematics import DogKinematics
from dog_link import Cancelled, DryRunLink, GLOBAL_DISABLE, GLOBAL_ENABLE, MotorLink
from dog_protocol import encode_mit

ROOT = Path(__file__).resolve().parent


def load_settings(path=None):
    path = Path(path or ROOT/'dog_config.json').resolve()
    config = json.loads(path.read_text(encoding='utf-8'))
    calibration_path = path.parent/config['motor']['calibration_file']
    calibration = json.loads(calibration_path.read_text(encoding='utf-8'))['motors']
    return config, calibration


def effective_gains(config, rows):
    """Configured MIT gains; optional per-joint overrides, never IK outputs."""
    def expand(name):
        value = config['motor'].get(name, config['sim_candidate'][name])
        array = np.asarray(value, dtype=float)
        if array.ndim == 0:
            return np.full(12, float(array))
        if array.shape == (3,):
            return np.tile(array, 4)
        if array.shape == (12,):
            return array.copy()
        raise ValueError('MIT '+name+' 必须是一个数、3 个或 12 个关节参数')
    kp, kd = expand('kp'), expand('kd')
    for index, row in enumerate(rows):
        for name, values in [('approved_kp', kp), ('approved_kd', kd)]:
            if row.get(name) is not None:
                values[index] = row[name]
    if not np.isfinite(kp).all() or np.any(kp <= 0) or np.any(kp > 500):
        raise ValueError('MIT Kp 必须是 (0,500] 内的有限数')
    if not np.isfinite(kd).all() or np.any(kd < 0) or np.any(kd > 5):
        raise ValueError('MIT Kd 必须是 [0,5] 内的有限数')
    return kp, kd


def calibration_problems(config, rows, kin):
    problems = []
    if config.get('allow_actuation') is not True:
        problems.append('配置已关闭真机驱动 allow_actuation')
    if len(rows) != 12 or [r.get('joint') for r in rows] != kin.joint_names:
        return ['标定表必须按模型顺序列出 12 个关节']
    ids = [r.get('can_id') for r in rows]
    if any(type(i) is not int for i in ids) or set(ids) != set(range(1, 13)):
        problems.append('12 个 CAN ID 尚未完整确认（必须各为 1..12）')
    try:
        effective_gains(config, rows)
    except (TypeError, ValueError) as error:
        problems.append(str(error))
    enforce_soft_limits = config['motor'].get('enforce_soft_limits', True)
    for index, row in enumerate(rows):
        missing = []
        if type(row.get('direction')) is not int or row['direction'] not in (-1, 1):
            missing.append('direction')
        required = ['zero_rad']
        if enforce_soft_limits:
            required.extend(('soft_min_rad', 'soft_max_rad'))
        for name in required:
            value = row.get(name)
            if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
                missing.append(name)
        if not all(row.get(k) is True for k in
                   ('verified', 'mit_mode_confirmed', 'output_radians_unit_ratio_confirmed')):
            missing.append('实测确认')
        if missing:
            problems.append(row['joint']+': '+', '.join(missing))
            continue
        stand = row['direction']*kin.stand_q[index]+row['zero_rad']
        pmax = config['motor']['mapping_ranges']['position_rad']
        if not -pmax < stand < pmax:
            problems.append(row['joint']+': 站姿超出电机位置映射范围')
        if enforce_soft_limits:
            low, high = row['soft_min_rad'], row['soft_max_rad']
            if not -pmax < low <= stand <= high < pmax or low >= high:
                problems.append(row['joint']+': 软限位无效或站姿超限')
    if config['motor'].get('mapping_ranges_confirmed') is not True:
        problems.append('电机 PMAX/VMAX/TMAX 与 STM32 映射范围尚未确认一致；'
                        '驱动器写入并读回 12.5/10/10 后，设置 '
                        'motor.mapping_ranges_confirmed=true 并重启程序')
    return problems


def demo_calibration(kin):
    # Only in memory. Never save simulated calibration into the real table.
    return [dict(joint=name, can_id=i+1, direction=1, zero_rad=0.,
                 soft_min_rad=float(kin.limits[i, 0]), soft_max_rad=float(kin.limits[i, 1]),
                 approved_kp=20., approved_kd=1., verified=True,
                 mit_mode_confirmed=True, output_radians_unit_ratio_confirmed=True)
            for i, name in enumerate(kin.joint_names)]


def minimum_jerk(u):
    u = min(max(u, 0.), 1.)
    return 10*u**3-15*u**4+6*u**5, 30*u**2-60*u**3+30*u**4


class DogController:
    def __init__(self, config=None, calibration=None, *, dry_run=False,
                 initial_action=None, link=None, tracking_error_guard=True,
                 standing_pose_guard=True):
        if config is None or calibration is None:
            config, calibration = load_settings()
        self.config = config
        self.kin = DogKinematics()
        self.dry_run = dry_run
        if type(tracking_error_guard) is not bool:
            raise ValueError('tracking_error_guard 必须是 true 或 false')
        self.tracking_error_guard = tracking_error_guard
        if type(standing_pose_guard) is not bool:
            raise ValueError('standing_pose_guard 必须是 true 或 false')
        self.standing_pose_guard = standing_pose_guard
        self.calibration = demo_calibration(self.kin) if dry_run else calibration
        self.kp, self.kd = effective_gains(config, self.calibration)
        self.enforce_soft_limits = config['motor'].get('enforce_soft_limits', True)
        if type(self.enforce_soft_limits) is not bool:
            raise ValueError('enforce_soft_limits 必须是 true 或 false')
        self.options = config['runtime']
        for name, value in self.options.items():
            if name != 'gaits' and (isinstance(value, bool) or not isinstance(value, (int, float))
                                   or not math.isfinite(value) or value <= 0):
                raise ValueError('运行参数必须是正有限数：'+name)
        ranges = config['motor']['mapping_ranges']
        for name, expected in [('position_rad', 12.5), ('velocity_rad_s', 10.), ('torque_nm', 10.)]:
            if ranges[name] != expected:
                raise ValueError('当前 STM32 固件映射范围为 12.5/10/10；修改主机配置不会修改固件')
        for name, profile in self.options['gaits'].items():
            if (not all(math.isfinite(profile[k]) for k in ('frequency', 'duty_factor', 'forward_speed'))
                    or not 0 < profile['frequency'] <= 5 or not 0 < profile['duty_factor'] < 1
                    or len(profile['phase_offsets']) != 4
                    or not all(math.isfinite(v) and 0 <= v < 1 for v in profile['phase_offsets'])):
                raise ValueError('步态配置无效：'+name)
        self.dt = float(self.options['control_dt'])
        if not .01 <= self.dt <= .05:
            raise ValueError('真机控制周期必须在 0.01..0.05 秒内')
        self.link = link or (DryRunLink(self.kin.stand_q) if dry_run
                             else MotorLink(config['motor']))
        self.initial_action = initial_action
        self._lock = threading.Lock()
        self._wake = threading.Event()
        self._shutdown = threading.Event()
        self._cancel = threading.Event()
        self._cancel.set()
        self._generation = 0
        self._commands = deque()
        self._events = deque(maxlen=80)
        self._connected = False
        self._enabled = False
        self._disabling = False
        self._disable_state = 'not_requested'
        self._disable_thread = None
        self._message = '正在连接…'
        self._mode = None
        self._plan = None
        self._thread = None
        self._last_tick = None
        self._q = self.kin.stand_q.copy()

    def start(self):
        self._thread = threading.Thread(target=self._worker, daemon=True, name='dog_control')
        self._thread.start()

    def _say(self, message, token=None):
        with self._lock:
            if token is not None and token != self._generation:
                return
            self._message = message
            self._events.append(time.strftime('%H:%M:%S')+' '+message)

    def status(self):
        with self._lock:
            result = dict(connected=self._connected, enabled=self._enabled,
                          disabling=self._disabling, mode=self._mode,
                          disable_state=self._disable_state,
                          message=self._message, events=list(self._events),
                          dry_run=self.dry_run, port=self.link.port,
                          tracking_error_guard=self.tracking_error_guard,
                          standing_pose_guard=self.standing_pose_guard)
        rows, stamp, count = self.link.snapshot()
        result.update(feedback=rows, feedback_age=clock_now()-stamp, frames=count)
        return result

    def request(self, action):
        if action == 'disable':
            self.disable()
            return
        if action not in ('enable', 'stand', 'walk', 'trot'):
            raise ValueError('未知动作')
        with self._lock:
            if not self._connected:
                raise RuntimeError('STM32 尚未连接')
            if self._disabling:
                raise RuntimeError('失能处理中，请等待结果')
            if action == 'enable' and self._disable_state == 'unconfirmed':
                raise RuntimeError('上一次失能未确认；请先重试失能并确认回报')
            if action == 'enable' and self._cancel.is_set():
                self._cancel = threading.Event()
                self._generation += 1
            if self._cancel.is_set():
                raise RuntimeError('请先使能')
            self._commands.append((action, self._generation, self._cancel))
        self._wake.set()

    def disable(self, reason='操作者一键失能'):
        # Set the latch before waiting for any normal action or serial write.
        with self._lock:
            self._cancel.set()
            self._generation += 1
            self._commands.clear()
            self._enabled = False
            self._mode = None
            self._plan = None
            if self._disabling:
                return
            self._disabling = True
            self._disable_state = 'pending'
        self._say(reason+'；正在发送全局失能')
        self._disable_thread = threading.Thread(target=self._disable_worker,
                                               args=(reason,),
                                               daemon=True, name='dog_disable')
        self._disable_thread.start()
        self._wake.set()

    def _disable_worker(self, reason):
        try:
            _, _, count0 = self.link.snapshot()
            start = clock_now()
            # First packet is the actual global disable command. It is never
            # delayed behind a trajectory or a stand transition.
            self.link.send(GLOBAL_DISABLE)
            self.link.send(encode_mit({i: (0., 0., 0., 0., 0.) for i in range(1, 13)}))
            next_repeat = start+.15
            confirmed = False
            while clock_now()-start < self.options['disable_timeout']:
                now = clock_now()
                rows, stamp, count = self.link.snapshot()
                if (now-start >= .2 and count > count0 and stamp > start
                        and rows and all(r['valid'] and r['state'] == 0 for r in rows)):
                    confirmed = True
                    break
                if now >= next_repeat:
                    self.link.send(GLOBAL_DISABLE)
                    next_repeat = now+.15
                time.sleep(.01)
            if confirmed:
                with self._lock:
                    self._disable_state = 'confirmed'
                confirmation = ('失能回报已确认' if self.dry_run else
                                '12 电机 USB 回报均为失能（当前固件回报使用 CAN 缓存）')
                self._say(reason+'；'+confirmation)
            else:
                with self._lock:
                    self._disable_state = 'unconfirmed'
                self._say(reason+'；失能已发送，但未收到完整失能回报；请使用物理断电')
        except Exception as error:
            with self._lock:
                self._disable_state = 'unconfirmed'
            self._say(f'{reason}；失能发送/确认失败：{error}；请使用物理断电')
        finally:
            with self._lock:
                self._disabling = False

    def _active(self, token, cancel):
        if cancel.is_set() or self._shutdown.is_set() or token != self._generation:
            raise Cancelled('动作已取消')

    def _feedback(self, require_enabled=False):
        if self.link.error:
            raise RuntimeError('反馈线程异常：'+self.link.error)
        rows, stamp, _ = self.link.snapshot()
        if rows is None or clock_now()-stamp > self.options['feedback_timeout']:
            raise RuntimeError('USB 反馈缺失/超时')
        for row in rows:
            if not row['valid'] or row['state'] not in (0, 1):
                raise RuntimeError(f'ID {row["id"]} 反馈无效或电机报错：state={row["state"]}')
            if require_enabled and row['state'] != 1:
                raise RuntimeError(f'ID {row["id"]} 没有使能回报')
            if not all(0 < row[k] < self.options['max_temperature'] for k in ('tmos', 'tcoil')):
                raise RuntimeError(f'ID {row["id"]} 温度无效/过温')
            if abs(row['velocity']) > self.options['max_measured_velocity']:
                raise RuntimeError(f'ID {row["id"]} 实测速度保护')
            if abs(row['torque']) > self.options['max_measured_torque']:
                raise RuntimeError(f'ID {row["id"]} 实测力矩保护')
        return rows

    def _model_positions(self, rows):
        by_id = {r['id']: r for r in rows}
        return np.array([r['direction']*(by_id[r['can_id']]['position']-r['zero_rad'])
                         for r in self.calibration])

    def _send_targets(self, q, dq, cancel, *, gain=1.):
        if cancel.is_set():
            raise Cancelled('动作已取消')
        if not np.isfinite(q).all() or not np.isfinite(dq).all():
            raise RuntimeError('目标包含非有限值')
        if np.abs(dq).max() > self.options['max_target_velocity']:
            raise RuntimeError('目标关节速度保护')
        if np.any(q < self.kin.limits[:, 0]) or np.any(q > self.kin.limits[:, 1]):
            raise RuntimeError('目标超过模型关节限位')
        actual = {r['id']: r for r in self._feedback(require_enabled=True)}
        records = {}
        for i, row in enumerate(self.calibration):
            motor_id = row['can_id']
            position = row['direction']*q[i]+row['zero_rad']
            velocity = row['direction']*dq[i]
            kp, kd = self.kp[i]*gain, self.kd[i]*gain
            measured = actual[motor_id]
            if abs(position) >= self.config['motor']['mapping_ranges']['position_rad']:
                raise RuntimeError(f'ID {motor_id} 目标超过 STM32 位置映射范围')
            if self.enforce_soft_limits:
                if not row['soft_min_rad'] <= position <= row['soft_max_rad']:
                    raise RuntimeError(f'ID {motor_id} 目标超过电机软限位')
                if not row['soft_min_rad'] <= measured['position'] <= row['soft_max_rad']:
                    raise RuntimeError(f'ID {motor_id} 实测位置超过软限位')
            error = position-measured['position']
            if self.tracking_error_guard and abs(error) > self.options['max_tracking_error_rad']:
                raise RuntimeError(f'ID {motor_id} 跟踪误差保护')
            estimated = kp*error+kd*(velocity-measured['velocity'])
            if abs(estimated) > self.options['max_estimated_pd_torque']:
                raise RuntimeError(f'ID {motor_id} 估计 PD 力矩保护：{estimated:.2f} Nm，'
                                   f'阈值 {self.options["max_estimated_pd_torque"]:g} Nm')
            if abs(velocity) >= self.config['motor']['mapping_ranges']['velocity_rad_s']:
                raise RuntimeError('目标超过 STM32 速度映射范围')
            records[motor_id] = (position, velocity, kp, kd, 0.)
        self.link.send(encode_mit(records), cancel)

    def _enable(self, token, cancel):
        self._active(token, cancel)
        if self._enabled:
            return
        if not self.dry_run:
            problems = calibration_problems(self.config, self.calibration, self.kin)
            if problems:
                raise ValueError('真机标定未完成：\n'+'\n'.join(problems))
        rows = self._feedback()
        q = self._model_positions(rows)
        if np.any(q < self.kin.limits[:, 0]) or np.any(q > self.kin.limits[:, 1]):
            raise ValueError('实测起始姿态超过模型限位，未发送使能')
        actual = {r['id']: r for r in rows}
        if self.enforce_soft_limits:
            for row in self.calibration:
                if not row['soft_min_rad'] <= actual[row['can_id']]['position'] <= row['soft_max_rad']:
                    raise ValueError('实测起始姿态超过软限位，未发送使能')
        self._say('正在使能：先保持实测当前位置，再缓升增益', token)
        self.link.send(encode_mit({r['id']: (r['position'], 0., 0., 0., 0.) for r in rows}), cancel)
        self._active(token, cancel)
        self.link.send(GLOBAL_ENABLE, cancel)
        deadline = clock_now()+1.5
        while True:
            if cancel.wait(self.dt):
                raise Cancelled('使能被中断')
            self._active(token, cancel)
            rows = self._feedback()
            if all(r['state'] == 1 for r in rows):
                break
            if clock_now() > deadline:
                raise RuntimeError('未收到全部电机使能回报')
        start = clock_now()
        while True:
            self._active(token, cancel)
            elapsed = clock_now()-start
            self._send_targets(q, np.zeros(12), cancel,
                               gain=min(elapsed/self.options['gain_ramp_seconds'], 1.))
            if elapsed >= self.options['gain_ramp_seconds']:
                break
            if cancel.wait(self.dt):
                raise Cancelled('增益缓升被中断')
        with self._lock:
            if token != self._generation or cancel.is_set():
                raise Cancelled('使能被中断')
            self._enabled = True
            self._mode = 'hold'
            self._plan = dict(kind='hold', q=q.copy())
            self._q = q.copy()
            self._last_tick = clock_now()
        self._say('已使能，保持当前位置；可选择 stand / walk / trot', token)

    def _begin_motion(self, action, token, cancel):
        self._active(token, cancel)
        if not self._enabled:
            raise RuntimeError('请先使能')
        q = self._model_positions(self._feedback(require_enabled=True))
        now = clock_now()
        if action == 'stand':
            duration = max(self.options['stand_seconds'],
                           1.875*float(np.abs(self.kin.stand_q-q).max())/
                           self.options['max_target_velocity'])
            plan = dict(kind='stand', q0=q.copy(), start=now, seconds=duration)
            message = f'stand：从实测姿态平滑站立（{duration:.1f} 秒）'
        else:
            if (self.standing_pose_guard and
                    np.abs(q-self.kin.stand_q).max() > self.options['standing_tolerance_rad']):
                raise RuntimeError(action+' 要求从站姿开始；当前姿态不符，不执行站立流程')
            gait_config = dict(self.config['sim_candidate'])
            gait_config['settle_seconds'] = 0.
            gait_config['ramp_seconds'] = self.options['gait_ramp_seconds']
            gait_config['mode_profiles'] = self.options['gaits']
            gait = DogGait(gait_config, self.kin.forward(q), action)
            plan = dict(kind=action, gait=gait, start=now, q=q.copy())
            message = action+'：从当前站姿开始步态，未调用 stand'
        with self._lock:
            self._active(token, cancel)
            self._plan = plan
            self._mode = action
            self._q = q.copy()
            self._last_tick = now
        self._say(message, token)

    def _tick(self, token, cancel):
        self._active(token, cancel)
        with self._lock:
            plan = self._plan
        if plan is None:
            return
        now = clock_now()
        delta_time = now-self._last_tick
        if delta_time > self.options['max_control_gap']:
            raise RuntimeError('控制循环超时')
        if plan['kind'] == 'hold':
            q, dq = plan['q'], np.zeros(12)
        elif plan['kind'] == 'stand':
            u = (now-plan['start'])/plan['seconds']
            position, speed = minimum_jerk(u)
            delta = self.kin.stand_q-plan['q0']
            q, dq = plan['q0']+delta*position, delta*speed/plan['seconds']
            if u >= 1:
                plan = dict(kind='hold', q=self.kin.stand_q.copy())
                with self._lock:
                    self._active(token, cancel)
                    self._plan = plan
                self._say('stand 完成，持续保持站姿', token)
        else:
            targets = plan['gait'].targets(now-plan['start'])
            q, errors = self.kin.solve(targets, self._q)
            if errors.max() > self.options['max_ik_error_m']:
                raise RuntimeError('步态 IK 误差超限')
            dq = (q-self._q)/max(delta_time, self.dt/2)
        self._send_targets(q, dq, cancel)
        self._q = q.copy()
        self._last_tick = now

    def _worker(self):
        boot_generation = self._generation
        try:
            self.link.open()
            with self._lock:
                self._connected = True
            problems = [] if self.dry_run else calibration_problems(self.config, self.calibration, self.kin)
            self._say('已连接；禁止使能：'+'；'.join(problems) if problems else
                      '已连接，尚未使能')
            if self.initial_action and boot_generation == self._generation:
                if problems:
                    self._say('自动启动未执行：'+problems[0])
                else:
                    self.request('enable')
                    self.request(self.initial_action)
        except Exception as error:
            self._say('连接失败：'+str(error))
            return
        while not self._shutdown.is_set():
            loop_start = clock_now()
            with self._lock:
                command = self._commands.popleft() if self._commands else None
                token, cancel, enabled = self._generation, self._cancel, self._enabled
            try:
                if command:
                    action, action_token, action_cancel = command
                    self._active(action_token, action_cancel)
                    if action == 'enable':
                        self._enable(action_token, action_cancel)
                    else:
                        self._begin_motion(action, action_token, action_cancel)
                elif enabled:
                    self._tick(token, cancel)
            except Cancelled:
                pass
            except ValueError as error:
                # Validation failed before enable. Stay connected and available
                # for disable; no automatic use of old simulation calibration.
                self.disable('配置校验失败：'+str(error))
            except Exception as error:
                if not cancel.is_set():
                    self.disable('保护触发：'+str(error))
            self._wake.wait(max(0., self.dt-(clock_now()-loop_start)))
            self._wake.clear()

    def close(self):
        # The UI waits asynchronously for disable before calling close().
        self._cancel.set()
        self._shutdown.set()
        self._wake.set()
        if self._thread:
            self._thread.join(timeout=.4)
        if self._disable_thread and self._disable_thread.is_alive():
            self._disable_thread.join(timeout=self.options['disable_timeout']+.2)
        self.link.close()
