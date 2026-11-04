import math,unittest,time,json,os
from types import SimpleNamespace
from unittest.mock import Mock
import numpy as np
from scipy.spatial.transform import Rotation
import rclpy
from geometry_msgs.msg import Twist,TransformStamped,PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from std_srvs.srv import SetBool
from std_msgs.msg import String
from dog_navigation.velocity_bridge import fit_gait,VelocityBridge
from dog_navigation.odometry import base_pose
from dog_navigation.alignment import map_alignment, FixedAlignment
from dog_navigation.discrete_actions import joystick_action


class NavigationTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        assert os.environ.get('ROS_DOMAIN_ID')=='49'
        assert os.environ.get('ROS_LOCALHOST_ONLY')=='1'
        rclpy.init()
    @classmethod
    def tearDownClass(cls):rclpy.shutdown()

    def test_gait_scaling_preserves_curvature_and_sign(self):
        for v,w in [(.2,.7),(-.12,.4),(.08,-.4),(0.,-.3)]:
            a,b,s=fit_gait(v,w,.08,.15)
            self.assertLessEqual(abs(a),.08+1e-12);self.assertLessEqual(abs(b),.15+1e-12)
            self.assertAlmostEqual(a*w,b*v);self.assertGreaterEqual(s,1.)
        with self.assertRaises(ValueError):fit_gait(float('nan'),.1,.08,.15)

    def test_mounting_transform_with_rotation_and_lever_arm(self):
        t=np.array([.22,-.05,.18]);mount=Rotation.from_euler('xyz',[math.pi,0.,math.pi/2])
        expected=Rotation.from_euler('xyz',[.1,-.2,.7]);base=np.array([2.,3.,.1])
        sensor_position=base+expected.apply(t);sensor_rotation=expected*mount
        actual,rot=base_pose(sensor_position,sensor_rotation.as_quat(),t,mount.as_quat())
        np.testing.assert_allclose(actual,base,atol=1e-10)
        np.testing.assert_allclose(rot.as_matrix(),expected.as_matrix(),atol=1e-10)

    def test_initial_alignment_composition_and_motion(self):
        odom_start=np.array([2.,-1.,.4]);map_start=np.array([-1.,3.,0.])
        odom_rot=Rotation.from_euler('z',-.7);map_rot=Rotation.from_euler('z',1.2)
        t,r=map_alignment(map_start,map_rot,odom_start,odom_rot)
        np.testing.assert_allclose(t+r.apply(odom_start),map_start,atol=1e-10)
        np.testing.assert_allclose((r*odom_rot).as_matrix(),map_rot.as_matrix(),atol=1e-10)
        moved=odom_start+odom_rot.apply([1.,0.,0.])
        np.testing.assert_allclose(t+r.apply(moved),map_start+map_rot.apply([1.,0.,0.]),atol=1e-10)

    def test_alignment_locks_and_tracks_sensor_timestamp(self):
        node=FixedAlignment()
        try:
            node.tf=Mock();odom=Odometry();odom.header.frame_id='dog_odom';odom.child_frame_id='dog_base_link'
            odom.pose.pose.orientation.w=1.;odom.header.stamp.sec=100
            node.receive_odom(odom)
            initial=PoseWithCovarianceStamped();initial.header.frame_id='dog_lio_map'
            initial.pose.pose.orientation.w=1.;initial.pose.pose.position.x=2.
            node.align(initial);self.assertIsNone(node.offset)
            node.receive_controller(String(data=json.dumps(dict(mode=None,teleop=dict(active=False)))))
            node.align(initial);self.assertIsNotNone(node.offset)
            saved=node.offset[0].copy()
            initial.pose.pose.position.x=99.;node.align(initial)
            np.testing.assert_allclose(node.offset[0],saved)
            odom.header.stamp.sec=101;odom.pose.pose.position.x=1.;node.receive_odom(odom)
            sent=node.tf.sendTransform.call_args.args[0]
            self.assertEqual(sent.header.stamp.sec,101)
            self.assertEqual(sent.transform.translation.x,2.)
        finally:node.destroy_node()

    def test_preview_cannot_arm_or_publish_motor_topic(self):
        node=VelocityBridge()
        try:
            self.assertIsNone(node.output)
            result=node.arm(SetBool.Request(data=True),SetBool.Response())
            self.assertFalse(result.success);self.assertFalse(node.armed)
        finally:node.destroy_node()

    def test_command_lease_and_feedback_loss_stop_navigation(self):
        node=VelocityBridge()
        try:
            node.allowed=True;node.cfg['mounting_configured']=True
            node.feedback_issue=lambda:None;node.controller_issue=lambda:None
            node.motion_issue=lambda v,w:None
            node.desired_motion=lambda:(.08,-.15)
            sent=[];node.send=lambda v,w,active:sent.append((v,w,active))
            self.assertTrue(node.arm(SetBool.Request(data=True),SetBool.Response()).success)
            command=Twist();command.linear.x=.08;command.angular.z=-.15
            node.receive_command(command);node.tick();self.assertEqual(sent[-1],(*joystick_action(7)[1:],True))
            node.command=(time.monotonic()-1.,.08,-.15);node.tick()
            self.assertFalse(node.armed);self.assertEqual(sent[-1],(0.,0.,False))
            node.arm(SetBool.Request(data=True),SetBool.Response())
            node.receive_command(command);node.feedback_issue=lambda:'odometry_missing_or_stale';node.tick()
            self.assertFalse(node.armed);self.assertEqual(sent[-1],(0.,0.,False))
        finally:node.destroy_node()

    def test_replanning_only_extends_zero_with_a_live_mission(self):
        node=VelocityBridge()
        try:
            node.allowed=True;node.cfg['mounting_configured']=True
            node.feedback_issue=lambda:None;node.controller_issue=lambda:None
            sent=[];node.send=lambda v,w,active:sent.append((v,w,active))
            node.arm(SetBool.Request(data=True),SetBool.Response())
            def heartbeat(active=True):
                node.receive_mission(String(data=json.dumps(dict(mission_id='test-mission',active=active,issued_monotonic=time.monotonic()))))
            heartbeat();node.command=(time.monotonic()-1.,0.,0.);node.tick()
            self.assertTrue(node.armed);self.assertEqual(sent[-1],(0.,0.,True))
            self.assertEqual(node.reason,'replanning_zero_hold')
            node.command=(time.monotonic()-1.,.04,0.);node.tick()
            self.assertFalse(node.armed);self.assertEqual(sent[-1],(0.,0.,False))
            node.arm(SetBool.Request(data=True),SetBool.Response());heartbeat()
            node.command=(time.monotonic()-1.,0.,0.);node.mission=('test-mission',time.monotonic()-1.)
            node.tick();self.assertFalse(node.armed)
            node.arm(SetBool.Request(data=True),SetBool.Response());heartbeat();heartbeat(False)
            self.assertFalse(node.armed);self.assertEqual(node.reason,'mission_finished')
        finally:node.destroy_node()

    def test_goal_switch_stops_old_action_and_keeps_zero_lease(self):
        node=VelocityBridge()
        try:
            node.allowed=True;node.cfg['mounting_configured']=True
            node.feedback_issue=lambda:None;node.controller_issue=lambda:None
            node.motion_issue=lambda v,w:None
            node.desired_motion=lambda:(.08,.15)
            sent=[];node.send=lambda v,w,active:sent.append((v,w,active))
            node.arm(SetBool.Request(data=True),SetBool.Response())
            node.command=(time.monotonic(),.08,0.);node.tick()
            def target(identity):
                node.receive_mission(String(data=json.dumps(dict(mission_id='mission',active=True,
                    issued_monotonic=time.monotonic(),target_id=identity,target_xy=[1.,0.]))))
            target('goal2');node.tick()
            self.assertEqual(sent[-1],(0.,0.,True));self.assertTrue(node.armed)
            self.assertEqual(node.policy.action[0],'stop');self.assertIsNone(node.command)
            node.command=(time.monotonic(),0.,.15);node.tick()
            self.assertEqual(sent[-1],(*joystick_action(1)[1:],True))
            for _ in range(10):
                node.command=(time.monotonic(),0.,-.15);node.tick()
                self.assertEqual(sent[-1],(*joystick_action(1)[1:],True))
            node.motion_issue=lambda v,w:'held_action_obstructed_wait_for_replan';node.tick()
            self.assertEqual(sent[-1],(0.,0.,True));self.assertTrue(node.armed)
        finally:node.destroy_node()

    def test_path_drives_reverse_despite_oscillating_nav_turn_and_stops_at_30cm(self):
        from nav_msgs.msg import Path
        from geometry_msgs.msg import PoseStamped
        node=VelocityBridge()
        try:
            node.feedback_issue=lambda:None;node.motion_issue=lambda v,w:None
            transform=TransformStamped();transform.transform.rotation.w=1.
            node.tf=Mock();node.tf.lookup_transform.return_value=transform
            path=Path();path.header.frame_id='dog_lio_map';path.header.stamp=node.get_clock().now().to_msg()
            for x in (0.,-.5,-1.):
                p=PoseStamped();p.pose.position.x=x;path.poses.append(p)
            node.receive_path(path)
            for w in (.15,-.15,.15):
                node.command=(time.monotonic(),0.,w);node.tick()
                self.assertEqual(node.policy.action[0],'backward')
            node.path=None;node.tick();self.assertEqual(node.policy.action[0],'stop')
            node.goal=[.30,0.];node.motion_issue=VelocityBridge.motion_issue.__get__(node)
            self.assertEqual(node.motion_issue(.08,0.),'target_reached')
            self.assertEqual(node.cfg['goal_radius_m'],.30)
            node.goal=[.31,0.]
            self.assertNotEqual(node.motion_issue(.08,0.),'target_reached')
        finally:node.destroy_node()

    def test_tight_arc_uses_checked_reverse_turn_only_at_decision_boundary(self):
        node=VelocityBridge()
        try:
            node.feedback_issue=lambda:None;node.desired_motion=lambda:(.08,.15)
            node.motion_issue=lambda v,w:'held_action_obstructed_wait_for_replan' if v>0 else None
            node.command=(time.monotonic(),.08,.15);node.tick()
            self.assertEqual(node.policy.action[0],'backward_left')
            node.motion_issue=lambda v,w:None;node.tick()
            self.assertEqual(node.policy.action[0],'backward_left')
            node.motion_issue=lambda v,w:'held_action_obstructed_wait_for_replan';node.tick()
            self.assertEqual(node.policy.action[0],'stop')
        finally:node.destroy_node()


if __name__=='__main__':unittest.main()
