"""Offline behavioral checks of real control flow and disable races; no serial I/O."""
import copy
import threading
import time
import unittest

import numpy as np

from dog_gait import DogGait
from dog_link import Cancelled, DryRunLink, GLOBAL_DISABLE, GLOBAL_ENABLE, MotorLink
from dog_protocol import RECORD, encode_mit
from dog_runtime import DogController, calibration_problems, demo_calibration, load_settings


def wait_until(predicate, timeout=3.):
    deadline = time.monotonic()+timeout
    while time.monotonic() < deadline:
        if predicate():
            return
        time.sleep(.005)
    raise AssertionError('Timed out waiting for controller state')


class RuntimeTests(unittest.TestCase):
    def setUp(self):
        self.config, self.rows = load_settings()
        self.config = copy.deepcopy(self.config)
        self.rows = copy.deepcopy(self.rows)
        for row in self.rows:
            row.update(can_id=None, direction=None, zero_rad=None, verified=False)
        self.config['runtime']['gain_ramp_seconds'] = .04
        self.controllers = []

    def make(self, **kwargs):
        controller = DogController(self.config, self.rows, dry_run=True, **kwargs)
        self.controllers.append(controller)
        controller.start()
        wait_until(lambda: controller.status()['connected'])
        return controller

    def tearDown(self):
        for controller in self.controllers:
            controller.disable('测试结束')
            wait_until(lambda: not controller.status()['disabling'])
            controller.close()

    def enable(self, controller):
        controller.request('enable')
        wait_until(lambda: controller.status()['enabled'])

    def test_real_missing_calibration_never_enables(self):
        demo = self.make()
        rows = demo_calibration(demo.kin)
        config = copy.deepcopy(self.config)
        config['motor']['mapping_ranges_confirmed'] = True
        self.assertEqual(calibration_problems(config, rows, demo.kin), [])
        self.assertGreater(len(calibration_problems(self.config, self.rows, demo.kin)), 0)
        link = DryRunLink(demo.kin.stand_q)
        real = DogController(self.config, self.rows, link=link)
        self.controllers.append(real)
        real.start()
        wait_until(lambda: real.status()['connected'])
        real.request('enable')
        wait_until(lambda: GLOBAL_DISABLE in link.packets)
        self.assertNotIn(GLOBAL_ENABLE, link.packets)

    def test_unconfirmed_mapping_keeps_reason_visible_and_never_enables(self):
        config, rows = load_settings()
        config = copy.deepcopy(config)
        config['motor']['mapping_ranges_confirmed'] = False
        link = DryRunLink(np.array([row['zero_rad'] for row in rows]))
        controller = DogController(config, rows, link=link)
        self.controllers.append(controller)
        controller.start()
        wait_until(lambda: 'PMAX/VMAX/TMAX' in controller.status()['message'])
        controller.request('enable')
        wait_until(lambda: controller.status()['disable_state'] == 'confirmed'
                   and not controller.status()['disabling'])
        self.assertNotIn(GLOBAL_ENABLE, link.packets)
        self.assertIn('mapping_ranges_confirmed=true', controller.status()['message'])
        self.assertIn('配置校验失败', controller.status()['message'])

    def test_pd_torque_guard_rejects_target_before_sending_even_below_tmax(self):
        config, rows = load_settings()
        config, rows = copy.deepcopy(config), copy.deepcopy(rows)
        controller = DogController(config, rows, link=DryRunLink(np.zeros(12)))
        q0 = controller.kin.stand_q
        controller.link = DryRunLink(np.array([
            row['direction']*q0[i]+row['zero_rad'] for i, row in enumerate(rows)]))
        controller.link.send(GLOBAL_ENABLE)
        q = q0.copy()
        q[0] += np.deg2rad(10.)
        estimated = controller.kp[0]*(q[0]-q0[0])
        self.assertGreater(estimated, config['runtime']['max_estimated_pd_torque'])
        self.assertLess(estimated, config['motor']['mapping_ranges']['torque_nm'])
        before = len(controller.link.packets)
        with self.assertRaisesRegex(RuntimeError, '估计 PD 力矩保护.*阈值 6 Nm'):
            controller._send_targets(q, np.zeros(12), threading.Event())
        self.assertEqual(len(controller.link.packets), before)

    def test_stand_uses_measured_pose_then_holds(self):
        self.config['runtime']['stand_seconds'] = .15
        base = DogController(self.config, self.rows, dry_run=True)
        initial = base.kin.stand_q+np.tile([.04, -.04, .04], 4)
        link = DryRunLink(initial)
        controller = self.make(link=link)
        self.enable(controller)
        np.testing.assert_allclose(controller._plan['q'], initial, atol=1e-6)
        controller.request('stand')
        wait_until(lambda: controller.status()['mode'] == 'stand')
        np.testing.assert_allclose(controller._plan['q0'], initial, atol=1e-6)
        wait_until(lambda: controller._plan['kind'] == 'hold')
        np.testing.assert_allclose(controller._plan['q'], controller.kin.stand_q)

    def test_disabled_tracking_guard_allows_lag_but_keeps_torque_and_feedback_guards(self):
        config, rows = load_settings()
        config, rows = copy.deepcopy(config), copy.deepcopy(rows)
        config['motor'].update(kp=10., kd=2., enforce_soft_limits=False)
        for row in rows:
            row['approved_kp'] = row['approved_kd'] = None
        initial = [row['zero_rad'] for row in rows]
        q = np.zeros(12)
        q[2] = -.35  # Above the tracking threshold, below the PD torque threshold.
        cancel = threading.Event()
        for guarded in (True, False):
            link = DryRunLink(initial)
            controller = DogController(config, rows, link=link, tracking_error_guard=guarded)
            link.send(GLOBAL_ENABLE)
            if guarded:
                with self.assertRaisesRegex(RuntimeError, '跟踪误差保护'):
                    controller._send_targets(q, np.zeros(12), cancel)
                self.assertEqual(len(link.packets), 1)
                continue
            controller._send_targets(q, np.zeros(12), cancel)
            self.assertEqual(len(link.packets), 2)
            link._positions = list(initial)
            q_torque = q.copy()
            q_torque[2] = -.7
            with self.assertRaisesRegex(RuntimeError, '估计 PD 力矩保护'):
                controller._send_targets(q_torque, np.zeros(12), cancel)
            self.assertEqual(len(link.packets), 2)
            link.error = 'simulated USB failure'
            with self.assertRaisesRegex(RuntimeError, '反馈线程异常'):
                controller._send_targets(q, np.zeros(12), cancel)
            self.assertEqual(len(link.packets), 2)

    def test_walk_trot_start_directly_from_standing_without_stand(self):
        for action in ('walk', 'trot'):
            with self.subTest(action=action):
                controller = self.make(initial_action=action)
                wait_until(lambda: controller.status()['mode'] == action)
                self.assertEqual(controller._plan['kind'], action)
                self.assertEqual(controller._plan['gait'].config['settle_seconds'], 0.)
                self.assertFalse(any('stand：' in e for e in controller.status()['events']))
                initial = controller._plan['gait'].targets(0.)
                np.testing.assert_allclose(initial, controller.kin.forward(controller.kin.stand_q))

    def test_walk_rejects_wrong_pose_without_standing(self):
        base = DogController(self.config, self.rows, dry_run=True)
        link = DryRunLink(base.kin.stand_q+np.tile([0., .2, 0.], 4))
        controller = self.make(link=link)
        self.enable(controller)
        controller.request('walk')
        wait_until(lambda: GLOBAL_DISABLE in link.packets)
        self.assertIsNone(controller._plan)
        self.assertTrue(any('要求从站姿开始' in e for e in controller.status()['events']))
        self.assertFalse(any('stand：' in e for e in controller.status()['events']))

    def test_trot_can_explicitly_start_from_the_measured_pose(self):
        base = DogController(self.config, self.rows, dry_run=True)
        initial = base.kin.stand_q+np.tile([0., .2, 0.], 4)
        controller = self.make(link=DryRunLink(initial), standing_pose_guard=False)
        self.enable(controller)
        controller.request('trot')
        wait_until(lambda: controller.status()['mode'] == 'trot')
        self.assertFalse(controller.status()['standing_pose_guard'])
        np.testing.assert_allclose(controller._plan['gait'].neutral,
                                   controller.kin.forward(initial))
        self.assertFalse(any('stand：' in e for e in controller.status()['events']))

    def test_disable_preempts_gain_ramp_and_cancels_queued_walk(self):
        self.config['runtime']['gain_ramp_seconds'] = 2.
        controller = self.make()
        controller.request('enable')
        controller.request('walk')
        wait_until(lambda: GLOBAL_ENABLE in controller.link.packets)
        before = time.monotonic()
        controller.disable()
        wait_until(lambda: GLOBAL_DISABLE in controller.link.packets, .3)
        self.assertLess(time.monotonic()-before, .3)
        wait_until(lambda: not controller.status()['disabling'])
        cutoff = controller.link.packets.index(GLOBAL_DISABLE)
        for packet in controller.link.packets[cutoff+1:]:
            self.assertNotEqual(packet, GLOBAL_ENABLE)
            if packet[:5] == b'DM12\x10':
                for i in range(12):
                    values = RECORD.unpack_from(packet, 5+i*RECORD.size)
                    self.assertEqual(values[5:], (0., 0., 0.))
        self.assertFalse(controller.status()['enabled'])
        self.assertFalse(any('walk：' in e for e in controller.status()['events']))

    def test_disable_during_gait_then_explicit_reenable(self):
        controller = self.make(initial_action='trot')
        wait_until(lambda: controller.status()['mode'] == 'trot')
        controller.disable()
        wait_until(lambda: not controller.status()['disabling'])
        self.assertFalse(controller.status()['enabled'])
        with self.assertRaises(RuntimeError):
            controller.request('walk')
        self.enable(controller)
        self.assertEqual(controller.status()['mode'], 'hold')

    def test_feedback_loss_and_tracking_error_request_disable(self):
        controller = self.make()
        self.enable(controller)
        controller.link.error = 'simulated USB failure'
        wait_until(lambda: GLOBAL_DISABLE in controller.link.packets)
        self.assertTrue(any('反馈线程异常' in e for e in controller.status()['events']))
        controller.link.error = None
        wait_until(lambda: not controller.status()['disabling'])
        self.enable(controller)
        controller.link._positions[1] += .4
        count = controller.link.packets.count(GLOBAL_DISABLE)
        wait_until(lambda: controller.link.packets.count(GLOBAL_DISABLE) > count)
        self.assertTrue(any('跟踪误差保护' in e for e in controller.status()['events']))

    def test_serial_write_checks_cancel_inside_lock(self):
        class Handle:
            is_open = True
            writes = []
            def write(self, packet):
                self.writes.append(packet)
                return len(packet)
        link = MotorLink(self.config['motor'])
        link.handle = Handle()
        cancel = threading.Event()
        cancelled = threading.Event()
        link._write_lock.acquire()
        def pending_send():
            try:
                link.send(encode_mit({1: (1., 0., 20., 1., 0.)}), cancel)
            except Cancelled:
                cancelled.set()
        thread = threading.Thread(target=pending_send)
        thread.start()
        cancel.set()
        link._write_lock.release()
        thread.join(timeout=.5)
        self.assertTrue(cancelled.is_set())
        self.assertEqual(link.handle.writes, [])

    def test_unconfirmed_disable_does_not_claim_disabled_or_allow_enable(self):
        self.config['runtime']['disable_timeout'] = .25
        controller = self.make()
        self.enable(controller)
        original_send = controller.link.send
        def dropped_disable(packet, cancelled=None):
            if packet != GLOBAL_DISABLE:
                original_send(packet, cancelled)
        controller.link.send = dropped_disable
        controller.disable()
        wait_until(lambda: not controller.status()['disabling'])
        self.assertEqual(controller.status()['disable_state'], 'unconfirmed')
        with self.assertRaises(RuntimeError):
            controller.request('enable')
        controller.link.send = original_send
        controller.disable()
        wait_until(lambda: not controller.status()['disabling'])
        self.assertEqual(controller.status()['disable_state'], 'confirmed')
        self.enable(controller)

    def test_walk_has_one_swing_leg_trot_has_diagonal_pairs(self):
        controller = self.make()
        for mode in ('walk', 'trot'):
            profile = dict(self.config['sim_candidate'])
            profile.update(settle_seconds=0., ramp_seconds=.01,
                           mode_profiles=self.config['runtime']['gaits'])
            neutral = controller.kin.forward(controller.kin.stand_q)
            gait = DogGait(profile, neutral, mode)
            for t in np.linspace(2., 4., 100):
                lift = gait.targets(t)[:, 2]-neutral[:, 2]
                if mode == 'walk':
                    self.assertLessEqual(int(np.count_nonzero(lift > 1e-7)), 1)
                else:
                    np.testing.assert_allclose(lift[[0, 3]], lift[0], atol=1e-12)
                    np.testing.assert_allclose(lift[[1, 2]], lift[1], atol=1e-12)

    def test_reversed_real_mapping_sends_full_mit_with_configured_gains(self):
        config, rows = load_settings()
        config, rows = copy.deepcopy(config), copy.deepcopy(rows)
        config['motor']['enforce_soft_limits'] = False
        config['motor']['mapping_ranges_confirmed'] = True
        for row in rows:
            row.update(can_id=row['candidate_can_id'], zero_rad=.4, verified=True,
                       mit_mode_confirmed=True, output_radians_unit_ratio_confirmed=True,
                       soft_min_rad=None, soft_max_rad=None, approved_kp=None, approved_kd=None)
        controller = DogController(config, rows, link=DryRunLink(np.zeros(12)))
        self.assertEqual(calibration_problems(config, rows, controller.kin), [])
        q0 = controller.kin.stand_q
        controller.link = DryRunLink(.4-q0)
        controller.link.send(GLOBAL_ENABLE)
        q, dq = q0+.001, np.linspace(-.1, .1, 12)
        controller._send_targets(q, dq, threading.Event())
        packet = controller.link.packets[-1]
        for i, row in enumerate(rows):
            fields = RECORD.unpack_from(packet, 5+(row['can_id']-1)*RECORD.size)
            self.assertEqual(fields[:3], (1, row['can_id'], 1))
            np.testing.assert_allclose(fields[3:], (.4-q[i], -dq[i], 50., 1., 0.), atol=1e-6)

    def test_soft_limits_are_optional_but_protocol_position_range_remains(self):
        config = copy.deepcopy(self.config)
        base = DogController(config, self.rows, dry_run=True)
        rows = demo_calibration(base.kin)
        config['motor']['mapping_ranges_confirmed'] = True
        for row in rows:
            row.update(soft_min_rad=None, soft_max_rad=None)
        config['motor']['enforce_soft_limits'] = False
        self.assertEqual(calibration_problems(config, rows, base.kin), [])
        config['motor']['enforce_soft_limits'] = True
        self.assertTrue(any('soft_min_rad' in p for p in calibration_problems(config, rows, base.kin)))
        config['motor']['enforce_soft_limits'] = False
        for row in rows:
            row['zero_rad'] = 13.
        controller = DogController(config, rows, link=DryRunLink(base.kin.stand_q+13.))
        controller.link.send(GLOBAL_ENABLE)
        with self.assertRaisesRegex(RuntimeError, 'STM32'):
            controller._send_targets(base.kin.stand_q, np.zeros(12), threading.Event())


if __name__ == '__main__':
    unittest.main(verbosity=2)
