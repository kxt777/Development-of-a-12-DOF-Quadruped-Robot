"""Use the original simulation command producer with measured/estimated state.

No physics stepping is performed by this adapter. MuJoCo is only the original
CAD kinematics/dynamics calculator; it does not supply imaginary observations.
"""
from dataclasses import dataclass
import json
from pathlib import Path
import sys

import mujoco
import numpy as np

PROJECT = Path(__file__).resolve().parents[1]
if str(PROJECT) not in sys.path:
    sys.path.insert(0, str(PROJECT))

from dog_tradition.dog_controller import DogController, DogMitCommand
from dog_tradition.dog_model import load_model, rotation
from dog_tradition.dog_profiles import resolve_config
from dog_protocol import encode_model_mit


def vector(value, size, name):
    result = np.asarray(value, dtype=float)
    if result.shape != (size,) or not np.isfinite(result).all():
        raise ValueError(f'{name}: expected {size} finite values')
    return result


@dataclass
class BodyState:
    time: float
    position_world: np.ndarray
    quaternion_wxyz: np.ndarray
    velocity_world: np.ndarray
    omega_body: np.ndarray
    q: np.ndarray
    dq: np.ndarray

    def validated(self):
        if not np.isfinite(self.time) or self.time < 0:
            raise ValueError('State time must be finite and nonnegative')
        self.position_world = vector(self.position_world, 3, 'position')
        self.velocity_world = vector(self.velocity_world, 3, 'velocity')
        self.omega_body = vector(self.omega_body, 3, 'omega_body')
        self.q = vector(self.q, 12, 'q')
        self.dq = vector(self.dq, 12, 'dq')
        quat = vector(self.quaternion_wxyz, 4, 'quaternion')
        norm = np.linalg.norm(quat)
        if norm < 1e-12:
            raise ValueError('Orientation quaternion is zero')
        self.quaternion_wxyz = quat / norm
        return self


def feedback_to_model(rows, calibration):
    by_id = {r['id']: r for r in rows}
    q, dq, torque = [], [], []
    for row in calibration:
        measured = by_id[row['can_id']]
        sign = row['direction']
        q.append(sign * (measured['position'] - row['zero_rad']))
        dq.append(sign * measured['velocity'])
        torque.append(sign * measured['torque'])
    return tuple(vector(v, 12, name) for v, name in
                 zip((q, dq, torque), ('feedback q', 'feedback dq', 'feedback torque')))


def native_mit_packet(command, calibration, ranges):
    """Preserve all MIT fields and sign mappings; no estimated-torque stop gate.

    Bounds below are the actual wire mappings, not guessed motor output limits.
    The driver calculates its PD torque; TMAX only bounds the FF wire field.
    """
    fields = [vector(getattr(command, key), 12, key)
              for key in ('q_des', 'dq_des', 'kp', 'kd', 'tau_ff')]
    q, dq, kp, kd, ff = fields
    if np.any(kp < 0) or np.any(kp > 500) or np.any(kd < 0) or np.any(kd > 5):
        raise ValueError('MIT gain exceeds the documented protocol range')
    return encode_model_mit(q, dq, kp, kd, ff, calibration, ranges)


class SimControllerBridge:
    def __init__(self, mode='trot', control_dt=None, torque_limit=None, config_overrides=None):
        source = PROJECT / 'dog_tradition/dog_config.json'
        self.config = resolve_config(json.loads(source.read_text(encoding='utf-8')), mode)
        if config_overrides:
            unknown = set(config_overrides) - set(self.config) - {'phase_offsets'}
            if unknown:
                raise ValueError(f'Unknown controller settings: {sorted(unknown)}')
            self.config.update(config_overrides)
        if control_dt is not None:
            self.config['control_dt'] = float(control_dt)
        if torque_limit is not None:
            self.config['torque_limit'] = [float(torque_limit)]*3
        self.mode = mode
        self.model, self.standing_q, self.metadata = load_model(self.config)
        self.data = mujoco.MjData(self.model)
        self.controller = None
        self.previous_time = None

    def set_state(self, state):
        state.validated()
        if self.previous_time is not None and state.time < self.previous_time:
            raise ValueError('Controller state time moved backwards')
        self.data.time = state.time
        self.data.qpos[:3] = state.position_world
        self.data.qpos[3:7] = state.quaternion_wxyz
        self.data.qvel[:3] = state.velocity_world
        # MuJoCo free-joint angular velocity is in the local body frame.
        self.data.qvel[3:6] = state.omega_body
        from dog_tradition.dog_kinematics import DogKinematics
        if not hasattr(self, 'kin'):
            self.kin = DogKinematics(self.model)
        self.data.qpos[self.kin.q_ids] = state.q
        self.data.qvel[self.kin.v_ids] = state.dq
        mujoco.mj_forward(self.model, self.data)
        self.previous_time = state.time

    def start(self, state):
        self.set_state(state)
        self.controller = DogController(self.model, self.data, self.config, state.q.copy())

    def step(self, state):
        if self.controller is None:
            self.start(state)
        else:
            self.set_state(state)
        self.controller.update(self.mode)
        return self.controller.command

    def telemetry(self):
        c = self.controller
        return dict(phase=c.phase.tolist(), planned_stance=c.planned_stance.tolist(),
                    support_force_model_n=c.forces.tolist(),
                    ik_error_m=c.ik_errors.tolist(), wrench_residual=c.wrench_residual,
                    position_reference=c.position_reference.tolist())


class StanceStateEstimator:
    """IMU orientation/rate + planned-support no-slip leg odometry.

    Support selection is a gait assumption, not a contact measurement. Slip,
    missing ground contact, CAD geometry and motor feedback latency are gaps.
    Accelerometer integration is deliberately not presented as ground truth.
    body_mount_quaternion maps IMU-frame vectors INTO the robot body frame.
    """
    def __init__(self, model, body_mount_quaternion_wxyz):
        if body_mount_quaternion_wxyz is None:
            raise ValueError('IMU-to-body mounting rotation is required; it is not assumed')
        mount = vector(body_mount_quaternion_wxyz, 4, 'IMU mounting quaternion')
        if np.linalg.norm(mount) < 1e-12:
            raise ValueError('IMU mounting quaternion is zero')
        self.body_from_imu = rotation(mount / np.linalg.norm(mount))
        from dog_tradition.dog_kinematics import DogKinematics
        self.kin = DogKinematics(model)
        self.heading_from_world = None
        self.anchors = None
        self.was_stance = np.ones(4, dtype=bool)
        self.position = None

    def update(self, time, q, dq, quaternion_imu_wxyz, omega_imu, planned_stance):
        quat = vector(quaternion_imu_wxyz, 4, 'IMU quaternion')
        if np.linalg.norm(quat) < 1e-12:
            raise ValueError('IMU quaternion is zero')
        world_from_body = rotation(quat / np.linalg.norm(quat)) @ self.body_from_imu.T
        if self.heading_from_world is None:
            yaw = np.arctan2(world_from_body[1, 0], world_from_body[0, 0])
            self.heading_from_world = np.array([[np.cos(yaw), np.sin(yaw), 0],
                                               [-np.sin(yaw), np.cos(yaw), 0], [0, 0, 1.]])
        r = self.heading_from_world @ world_from_body
        omega = self.body_from_imu @ vector(omega_imu, 3, 'IMU angular rate')
        q, dq = vector(q, 12, 'joint q'), vector(dq, 12, 'joint dq')
        stance = np.asarray(planned_stance, dtype=bool)
        if stance.shape != (4,) or not stance.any():
            raise ValueError('Stance odometry requires at least one planned support leg')
        feet = self.kin.forward(q)
        offsets_world = feet @ r.T
        if self.anchors is None:
            self.position = np.array([0., 0., -np.mean(offsets_world[stance, 2])])
            self.anchors = self.position + offsets_world
            self.anchors[:, 2] = 0.
        for leg in np.flatnonzero(stance & ~self.was_stance):
            self.anchors[leg] = self.position + offsets_world[leg]
            self.anchors[leg, 2] = 0.
        self.position = np.mean(self.anchors[stance] - offsets_world[stance], axis=0)
        candidates = []
        for leg in np.flatnonzero(stance):
            joint_velocity = self.kin.jacobian(leg) @ dq[3*leg:3*leg+3]
            candidates.append(-r @ (np.cross(omega, feet[leg]) + joint_velocity))
        velocity = np.mean(candidates, axis=0)
        body_quat = np.empty(4)
        mujoco.mju_mat2Quat(body_quat, r.ravel())
        self.was_stance = stance.copy()
        return BodyState(time, self.position.copy(), body_quat, velocity, omega, q, dq).validated()


def command_telemetry(command, q, dq, measured_torque):
    p = command.kp * (command.q_des - q)
    d = command.kd * (command.dq_des - dq)
    return dict(q_des=command.q_des.tolist(), dq_des=command.dq_des.tolist(),
                kp=command.kp.tolist(), kd=command.kd.tolist(), tau_ff=command.tau_ff.tolist(),
                q_actual=q.tolist(), dq_actual=dq.tolist(),
                measured_torque_model_nm=measured_torque.tolist(),
                p_term_nm=p.tolist(), d_term_nm=d.tolist(),
                estimated_total_nm=(p+d+command.tau_ff).tolist())


def planned_support(config, time, mode):
    if mode == 'stand' or time <= config['settle_seconds']:
        return np.ones(4, dtype=bool)
    phase = ((time-config['settle_seconds']) * config['frequency']
             + np.asarray(config.get('phase_offsets', [0., .5, .5, 0.]))) % 1.
    return phase < config['duty_factor']
