"""Replay and stop behavior with memory-only feedback; never opens hardware."""
import copy
import hashlib
import json
from pathlib import Path
import tempfile
import threading
import time
from dog_clock import now as clock_now
import unittest

import numpy as np

from dog_link import Cancelled, DryRunLink, GLOBAL_ENABLE
from dog_protocol import RECORD
from dog_replay import RecordedTrajectory, ReplayController
from dog_runtime import load_settings


class ReplayTests(unittest.TestCase):
    def setUp(self):
        self.config, self.rows = load_settings()
        self.config = copy.deepcopy(self.config)
        self.rows = copy.deepcopy(self.rows)
        self.config['motor']['kp'] = [50.]*12
        self.config['motor']['kd'] = [1.]*12
        self.folder = tempfile.TemporaryDirectory()
        self.addCleanup(self.folder.cleanup)
        path = Path(self.folder.name)/'reference.npz'
        source = Path(__file__).resolve().parents[1]/'rl_training/dog/dog.xml'
        digest = hashlib.sha256(source.read_bytes()).hexdigest()
        data = np.zeros((3, 37))
        data[:, 0] = [0., .5, 1.]
        data[:, 1:13] = np.array([0., .1, .2])[:, None]
        data[:, 13:25] = .2
        data[:, 25:] = np.array([.1, .2, .3])[:, None]
        np.savez_compressed(path, commands=data, kp=np.full(12, 50.), kd=np.ones(12),
                            source_sha256=digest, summary_json=json.dumps(dict(
                                name='original_5ms', stop_reason='completed', model=dict(source_sha256=digest))))
        self.reference = RecordedTrajectory(path)

    def controller(self, link=None):
        link = link or DryRunLink([r['zero_rad'] for r in self.rows])
        controller = ReplayController(self.config, self.rows, self.reference, .8, link=link)
        link.send(GLOBAL_ENABLE)
        return controller

    def test_interpolates_all_three_original_fields(self):
        q, dq, ff = self.reference.sample(.25)
        np.testing.assert_allclose(q, .05)
        np.testing.assert_allclose(dq, .2)
        np.testing.assert_allclose(ff, .15)
        for invalid in (-1., 1.01, float('nan')):
            with self.assertRaises(ValueError):
                self.reference.sample(invalid)

    def test_feedback_timestamps_share_the_high_resolution_controller_clock(self):
        from dog_clock import now
        self.assertLessEqual(time.get_clock_info('perf_counter').resolution, .001)
        link = DryRunLink([r['zero_rad'] for r in self.rows])
        before = now()
        _, stamp, _ = link.snapshot()
        after = now()
        self.assertLessEqual(before, stamp)
        self.assertLessEqual(stamp, after)

    def test_no_project_pd_stop_and_feedforward_is_transmitted_with_calibration_sign(self):
        controller = self.controller()
        q, dq, ff = self.reference.sample(.8)
        controller._send_targets(q, dq, threading.Event(), tau_ff=ff, reference_time=.8)
        self.assertGreater(max(controller.last_command['estimated_total_nm']), 6.)
        self.assertEqual(controller.last_command['reference_time_s'], .8)
        for i, row in enumerate(self.rows):
            _, _, _, p, v, kp, kd, tau = RECORD.unpack_from(controller.link.packets[-1],
                                                          5+(row['can_id']-1)*RECORD.size)
            self.assertAlmostEqual(tau, row['direction']*ff[i], places=6)
            self.assertEqual((kp, kd), (50., 1.))

    def test_feedback_project_torque_velocity_temperature_values_do_not_gate_motion(self):
        class Link(DryRunLink):
            def snapshot(self):
                rows, stamp, count = super().snapshot()
                for row in rows:
                    row.update(velocity=9., torque=8., tmos=70., tcoil=70.)
                return rows, stamp, count
        controller = self.controller(Link([r['zero_rad'] for r in self.rows]))
        q, dq, ff = self.reference.sample(.2)
        controller._send_targets(q, dq, threading.Event(), tau_ff=ff)
        self.assertEqual(len(controller.link.packets), 2)

    def test_driver_fault_stops_before_any_new_motion_packet(self):
        class Link(DryRunLink):
            def snapshot(self):
                rows, stamp, count = super().snapshot()
                rows[5]['state'] = 9
                return rows, stamp, count
        controller = self.controller(Link([r['zero_rad'] for r in self.rows]))
        with self.assertRaisesRegex(RuntimeError, 'driver fault'):
            controller._send_targets(*self.reference.sample(.2)[:2], threading.Event())
        self.assertEqual(len(controller.link.packets), 1)

    def test_stale_feedback_and_cancel_stop_before_new_motion_packet(self):
        class Link(DryRunLink):
            def snapshot(self):
                rows, stamp, count = super().snapshot()
                return rows, clock_now()-.3, count
        controller = self.controller(Link([r['zero_rad'] for r in self.rows]))
        with self.assertRaisesRegex(RuntimeError, 'stale'):
            controller._send_targets(*self.reference.sample(.2)[:2], threading.Event())
        cancel = threading.Event()
        cancel.set()
        with self.assertRaises(Cancelled):
            controller._send_targets(*self.reference.sample(.2)[:2], cancel)
        self.assertEqual(len(controller.link.packets), 1)

    def test_wire_overflow_does_not_get_silently_clamped(self):
        controller = self.controller()
        q, dq, ff = self.reference.sample(.2)
        ff[4] = 10.01
        with self.assertRaisesRegex(ValueError, 'wire mapping'):
            controller._send_targets(q, dq, threading.Event(), tau_ff=ff)
        self.assertEqual(len(controller.link.packets), 1)


if __name__ == '__main__':
    unittest.main(verbosity=2)
