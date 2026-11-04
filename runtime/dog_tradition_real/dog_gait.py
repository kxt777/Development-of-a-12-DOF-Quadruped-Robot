"""Body-frame kinematic demo; not the simulation's world-state balance loop."""
import numpy as np


def smooth(value):
    value = np.clip(value, 0., 1.)
    return value*value*(3.-2.*value)


class DogGait:
    def __init__(self, config, neutral, mode):
        if mode not in ('stand', 'trot', 'walk'):
            raise ValueError('Unknown gait mode')
        self.config = dict(config)
        self.config.update(config.get('mode_profiles', {}).get(mode, {}))
        self.neutral = np.array(neutral, dtype=float)
        self.mode = mode

    def targets(self, time):
        c = self.config
        elapsed = max(0., time-c['settle_seconds'])
        feet = self.neutral.copy()
        if self.mode == 'stand' or elapsed == 0:
            return feet
        ramp = smooth(elapsed/c['ramp_seconds'])
        phase = (elapsed*c['frequency'] + np.array(
            c.get('phase_offsets', [0., .5, .5, 0.]))) % 1.
        duty = c['duty_factor']
        speed = c['forward_speed'] if self.mode == 'walk' else 0.
        stroke = speed*duty/c['frequency']*ramp
        for leg, p in enumerate(phase):
            if p < duty:
                # Smooth endpoints; body-relative stance is an approximation.
                feet[leg, 0] += stroke*(.5-smooth(p/duty))
            else:
                u = (p-duty)/(1-duty)
                feet[leg, 0] += stroke*(-.5+smooth(u))
                feet[leg, 2] += c['step_height']*ramp*np.sin(np.pi*u)**2
        return feet
