"""MJCF-derived FK and damped IK, using NumPy only and original joint axes."""
import json
from pathlib import Path

import numpy as np


def quaternion_rotation(values):
    w, x, y, z = np.asarray(values, dtype=float) / np.linalg.norm(values)
    return np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                     [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                     [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])


def axis_rotation(axis, angle):
    axis = np.asarray(axis, dtype=float)
    axis /= np.linalg.norm(axis)
    x, y, z = axis
    skew = np.array([[0, -z, y], [z, 0, -x], [-y, x, 0]])
    return np.eye(3) + np.sin(angle)*skew + (1-np.cos(angle))*(skew@skew)


class DogKinematics:
    def __init__(self, path=None):
        path = path or Path(__file__).with_name('dog_kinematics.json')
        self.geometry = json.loads(Path(path).read_text(encoding='utf-8'))
        self.stand_q = np.array(self.geometry['stand_q_rad'])
        self.limits = np.array([j['range'] for leg in self.geometry['legs']
                                for j in leg['chain']])
        self.joint_names = [j['joint'] for leg in self.geometry['legs']
                            for j in leg['chain']]

    def evaluate(self, q):
        q = np.asarray(q, dtype=float)
        if q.shape != (12,) or not np.isfinite(q).all():
            raise ValueError('Expected 12 finite model joint angles')
        feet, jacobians = [], []
        for index, leg in enumerate(self.geometry['legs']):
            rotation, translation = np.eye(3), np.zeros(3)
            pivots, axes = [], []
            for k, joint in enumerate(leg['chain']):
                translation = translation + rotation @ np.array(joint['position'])
                rotation = rotation @ quaternion_rotation(joint['quaternion_wxyz'])
                origin = np.array(joint['joint_origin'])
                axis = np.array(joint['axis'], dtype=float)
                axis /= np.linalg.norm(axis)
                pivots.append(translation + rotation @ origin)
                axes.append(rotation @ axis)
                turn = axis_rotation(axis, q[3*index+k])
                translation += rotation @ (origin - turn @ origin)
                rotation = rotation @ turn
            foot = translation + rotation @ np.array(leg['foot_point'])
            feet.append(foot)
            jacobians.append(np.column_stack([np.cross(a, foot-p)
                                              for a, p in zip(axes, pivots)]))
        return np.array(feet), np.array(jacobians)

    def forward(self, q):
        return self.evaluate(q)[0]

    def solve(self, targets, seed, max_iterations=24, tolerance=2e-5):
        targets = np.asarray(targets, dtype=float)
        if targets.shape != (4, 3) or not np.isfinite(targets).all():
            raise ValueError('Expected four finite XYZ targets in body coordinates')
        q = np.asarray(seed, dtype=float).copy()
        for _ in range(max_iterations):
            positions, jacobians = self.evaluate(q)
            errors = targets - positions
            if np.linalg.norm(errors, axis=1).max() < tolerance:
                break
            for leg, j in enumerate(jacobians):
                delta = j.T @ np.linalg.solve(j@j.T + 0.002**2*np.eye(3), errors[leg])
                q[3*leg:3*leg+3] += delta * min(1., 0.12/max(np.linalg.norm(delta), 1e-12))
            q = np.clip(q, self.limits[:, 0]+0.02, self.limits[:, 1]-0.02)
        return q, np.linalg.norm(targets-self.forward(q), axis=1)
