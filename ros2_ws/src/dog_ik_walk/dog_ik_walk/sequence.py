"""User-requested finite zero/stand/walk sequence; successful completion holds stand."""
import time
import numpy as np


def run_sequence(controller, deployment, report, cancelled, clock=time.monotonic, sleep=time.sleep):
    c = controller

    def check():
        if cancelled.is_set():
            raise RuntimeError('Demo interrupted by operator')
        s = c.status()
        if s['disabling'] or s['disable_state'] == 'unconfirmed' or (report['stage'] != 'enable' and s['disable_state'] == 'confirmed' and not s['enabled']):
            raise RuntimeError(s['message'])
        c._feedback(require_enabled=report['stage'] != 'enable')

    def wait(predicate, seconds):
        deadline = clock() + seconds
        while True:
            check()
            if predicate():
                return
            if clock() >= deadline:
                raise RuntimeError('Timeout at ' + report['stage'])
            sleep(.02)

    def hold(seconds):
        deadline = clock() + seconds
        wait(lambda: clock() >= deadline, seconds + 1.)

    c._feedback()
    report['stage'] = 'enable'
    c.request('enable')
    wait(lambda: c.status()['enabled'], c.options['gain_ramp_seconds'] + 4.)
    report['stage'] = 'zero'
    c.move_to_pose(np.zeros(12), deployment['zero_seconds'], 'zero')
    with c._lock:
        duration = c._plan['seconds']
    wait(lambda: c.status()['mode'] == 'hold', duration + 4.)
    report['stage'] = 'zero_hold'
    report['zero_measured_q'] = c._model_positions(c._feedback(require_enabled=True)).tolist()
    hold(deployment['zero_hold_seconds'])
    report['stage'] = 'stand'
    c.request('stand')
    wait(lambda: c.stand_completed, deployment['stand_seconds'] + 4.)
    report['stage'] = 'stand_hold'
    hold(deployment['hold_seconds'])
    report['stage'] = 'walk'
    report['walk_start_monotonic'] = clock()
    if deployment.get('imu_used'):
        c.request('walk',preserve_heading=True)
    else:c.request('walk')
    wait(lambda: c.status()['mode'] == 'walk_done', deployment['walk_seconds'] + 4.)
    report['walk_finished_monotonic'] = clock()
    report.update(state='command_sequence_completed_holding_stand', stage='stand_hold_after_walk')
