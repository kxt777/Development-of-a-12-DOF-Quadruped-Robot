"""Nav2 Twist -> the existing sole motor writer; idle and preview-only by default."""
import json
import math
import time
import uuid
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry, OccupancyGrid, Path
from std_msgs.msg import String
from std_srvs.srv import SetBool
from tf2_ros import Buffer, TransformListener, TransformException
from .discrete_actions import HeldAction, swept_clear, path_command, quantize


def fit_gait(v, w, vmax, wmax):
    """Preserve requested curvature inside the already evaluated IK bank."""
    if not all(math.isfinite(x) for x in (v,w,vmax,wmax)) or min(vmax,wmax)<=0:
        raise ValueError('Nonfinite velocity or invalid gait envelope')
    scale=max(1.,abs(v)/vmax,abs(w)/wmax)
    return v/scale,w/scale,scale


class VelocityBridge(Node):
    def __init__(self):
        super().__init__('dog_nav_velocity_bridge')
        defaults={'allow_motor_output':False,'mounting_configured':False,
                  'max_forward_m_s':.08,'max_yaw_rad_s':.15,'command_timeout_s':.5,
                  'feedback_timeout_s':.5,'controller_namespace':'/dog_ik_walk_imu',
                  'action_hold_s':4.,'goal_radius_m':.30,'costmap_timeout_s':1.5,
                  'stop_ramp_s':2.,'footprint_json':'[]','path_lookahead_m':.5,'path_timeout_s':3.}
        for k,v in defaults.items():self.declare_parameter(k,v)
        self.cfg={k:self.get_parameter(k).value for k in defaults}
        self.allowed=self.cfg['allow_motor_output']
        self.armed=False;self.gesture='';self.seq=0;self.command=None;self.odom=None
        self.status=None;self.status_time=0.;self.mission=None;self.reason='preview_only' if not self.allowed else 'disarmed'
        self.policy=HeldAction(self.cfg['action_hold_s'],self.cfg['max_forward_m_s'],self.cfg['max_yaw_rad_s'])
        self.costmap=None;self.path=None;self.goal=None;self.target_id=None;self.action_event_revision=-1;self.approach_sign=None
        self.footprint=json.loads(self.cfg['footprint_json'])
        self.tf=Buffer();self.listener=TransformListener(self.tf,self)
        self.preview=self.create_publisher(Twist,'/dog_nav/velocity_preview',1)
        # Preview mode does not even create a publisher on the actuator interface.
        ns=self.cfg['controller_namespace']
        self.output=self.create_publisher(String,ns+'/nav_velocity',1) if self.allowed else None
        self.state_pub=self.create_publisher(String,'/dog_nav/bridge_status',1)
        self.action_pub=self.create_publisher(String,'/dog_nav/discrete_action',10)
        self.create_subscription(Twist,'/dog_nav/cmd_vel',self.receive_command,1)
        self.create_subscription(Odometry,'/dog_nav/odom',self.receive_odom,qos_profile_sensor_data)
        self.create_subscription(String,ns+'/status',self.receive_status,1)
        self.create_subscription(String,'/dog_nav/mission_lease',self.receive_mission,1)
        self.create_subscription(OccupancyGrid,'/dog_nav/dog_local_costmap/costmap',self.receive_costmap,1)
        self.create_subscription(Path,'/dog_nav/plan',self.receive_path,1)
        self.create_service(SetBool,'/dog_nav/arm',self.arm)
        self.create_timer(.05,self.tick)

    def receive_command(self,msg):
        values=[msg.linear.x,msg.linear.y,msg.linear.z,msg.angular.x,msg.angular.y,msg.angular.z]
        if not all(math.isfinite(v) for v in values) or any(abs(values[i])>1e-6 for i in (1,2,3,4)):
            self.command=None;self.disarm('invalid_or_unsupported_twist');return
        self.command=(time.monotonic(),msg.linear.x,msg.angular.z)

    def receive_odom(self,msg):
        p=msg.pose.pose.position;q=msg.pose.pose.orientation
        if not all(math.isfinite(v) for v in (p.x,p.y,p.z,q.x,q.y,q.z,q.w)):
            self.odom=None;self.disarm('invalid_odometry');return
        self.odom=(time.monotonic(),msg)

    def receive_status(self,msg):
        try:self.status=json.loads(msg.data);self.status_time=time.monotonic()
        except (ValueError,TypeError):self.status=None

    def receive_costmap(self,msg):
        if msg.info.resolution<=0 or len(msg.data)!=msg.info.width*msg.info.height:return
        self.costmap=(time.monotonic(),msg)

    def receive_path(self,msg):
        points=[(p.pose.position.x,p.pose.position.y) for p in msg.poses]
        if not msg.header.frame_id or not points or not all(math.isfinite(v) for p in points for v in p):
            self.path=None;return
        # Reject a late plan for the previous waypoint after a goal switch.
        if self.goal is not None and msg.header.frame_id=='dog_lio_map' and math.hypot(points[-1][0]-self.goal[0],points[-1][1]-self.goal[1])>self.cfg['goal_radius_m']:
            return
        self.path=(time.monotonic(),msg.header,points)

    def desired_motion(self):
        if self.path is None or time.monotonic()-self.path[0]>self.cfg['path_timeout_s']:
            raise ValueError('path_missing_or_stale')
        _,header,points=self.path
        age=(self.get_clock().now().nanoseconds-Time.from_msg(header.stamp).nanoseconds)/1e9
        if not 0<=age<=self.cfg['path_timeout_s']:raise ValueError('path_stamp_stale')
        pose=self.tf.lookup_transform(header.frame_id,'dog_base_link',Time()).transform
        desired=path_command(points,(pose.translation.x,pose.translation.y,self.yaw(pose.rotation)),
            self.cfg['path_lookahead_m'],self.cfg['max_forward_m_s'],self.cfg['max_yaw_rad_s'],self.approach_sign)
        if self.approach_sign is None and desired[0]!=0:self.approach_sign=math.copysign(1.,desired[0])
        return desired

    def receive_mission(self,msg):
        try:
            data=json.loads(msg.data);identity=data['mission_id'];stamp=float(data['issued_monotonic'])
            if not isinstance(identity,str) or not identity or type(data['active']) is not bool:return
            if not 0<=time.monotonic()-stamp<=self.cfg['command_timeout_s']:return
            if self.allowed and not self.armed:return
            if self.mission is not None and self.mission[0]!=identity:return
            if not data['active']:
                if self.mission is not None:self.disarm('mission_finished')
                self.command=None;self.path=None;self.mission=None
                return
            self.mission=(identity,time.monotonic())
            target=data.get('target_id');xy=data.get('target_xy')
            if target is not None and target!=self.target_id:
                if not isinstance(xy,list) or len(xy)!=2 or not all(math.isfinite(v) for v in xy):return
                self.target_id=target;self.goal=xy
                self.policy.stop(time.monotonic(),reset=True)
                self.command=None  # Never carry the previous goal's velocity into this goal.
                self.path=None
                self.approach_sign=None
        except (ValueError,TypeError,KeyError):return

    def feedback_issue(self):
        now=time.monotonic();limit=self.cfg['feedback_timeout_s']
        if self.odom is None or now-self.odom[0]>limit:return 'odometry_missing_or_stale'
        msg=self.odom[1]
        age=(self.get_clock().now().nanoseconds-Time.from_msg(msg.header.stamp).nanoseconds)/1e9
        if not 0<=age<=limit:return 'odometry_stamp_stale'
        try:
            transform=self.tf.lookup_transform('dog_lio_map','dog_base_link',Time())
            age=(self.get_clock().now().nanoseconds-Time.from_msg(transform.header.stamp).nanoseconds)/1e9
            if not 0<=age<=limit:return 'map_pose_stale'
        except TransformException:return 'map_pose_missing'
        return None

    def controller_issue(self):
        s=self.status or {}
        if time.monotonic()-self.status_time>1.5:return 'controller_status_stale'
        if not s.get('enabled') or not s.get('stand_ready'):return 'controller_not_enabled_and_standing'
        t=s.get('teleop',{})
        if t.get('velocity_source')!='navigation':return 'navigation_source_not_selected'
        if self.armed and t.get('gesture')==self.gesture and t.get('state') in ('released','disabled','command_timeout'):
            return 'operator_or_controller_stopped'
        return None

    @staticmethod
    def yaw(q):
        return math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))

    def motion_issue(self,v,w):
        try:
            if self.goal is not None:
                pose=self.tf.lookup_transform('dog_lio_map','dog_base_link',Time()).transform.translation
                if math.hypot(pose.x-self.goal[0],pose.y-self.goal[1])<=self.cfg['goal_radius_m']:return 'target_reached'
            if v==0 and w==0:return None
            if self.costmap is None or time.monotonic()-self.costmap[0]>self.cfg['costmap_timeout_s']:return 'costmap_missing_or_stale'
            msg=self.costmap[1]
            age=(self.get_clock().now().nanoseconds-Time.from_msg(msg.header.stamp).nanoseconds)/1e9
            if not 0<=age<=self.cfg['costmap_timeout_s']:return 'costmap_stamp_stale'
            pose=self.tf.lookup_transform(msg.header.frame_id,'dog_base_link',Time()).transform
            origin=msg.info.origin
            # Check the action actually sent to the dog, not Nav2's changing raw
            # Twist. Include the existing joystick's 2 s stopping transition.
            duration=max(0.,self.policy.next_decision-time.monotonic())+self.cfg['stop_ramp_s']
            clear=swept_clear(msg.data,msg.info.width,msg.info.height,msg.info.resolution,
                (origin.position.x,origin.position.y,self.yaw(origin.orientation)),
                (pose.translation.x,pose.translation.y,self.yaw(pose.rotation)),v,w,duration,self.footprint)
            return None if clear else 'held_action_obstructed_wait_for_replan'
        except TransformException:return 'costmap_transform_missing'

    def arm(self,request,response):
        if not request.data:
            self.disarm('operator_disarmed');response.success=True;response.message=self.reason;return response
        issue=None
        if not self.allowed:issue='Preview-only deployment cannot arm'
        elif not self.cfg['mounting_configured']:issue='Lidar-to-dog mounting convention not configured'
        else:issue=self.feedback_issue() or self.controller_issue()
        if issue:response.success=False;response.message=issue;return response
        # Old Nav2 output cannot become a new motion when arming.
        self.command=None;self.path=None;self.mission=None;self.armed=True;self.gesture='nav-'+uuid.uuid4().hex;self.seq=0
        self.target_id=None;self.goal=None;self.approach_sign=None;self.policy.stop(time.monotonic(),reset=True)
        self.reason='armed_waiting_for_new_command'
        response.success=True;response.message=self.reason;return response

    def send(self,v,w,active):
        if self.output is None:return
        self.seq+=1
        packet=dict(active=active,gesture=self.gesture,seq=self.seq,
                    issued_monotonic=time.monotonic(),vx=v,wz=w,
                    action_name=self.policy.action[0],action_id=self.policy.revision)
        self.output.publish(String(data=json.dumps(packet)))

    def disarm(self,reason):
        self.policy.stop(time.monotonic(),reset=True)
        if self.armed:self.send(0.,0.,False)
        self.armed=False;self.reason=reason

    def tick(self):
        issue=self.feedback_issue();v=w=0.;scale=1.;motion_issue=None
        fresh=self.command is not None and time.monotonic()-self.command[0]<=self.cfg['command_timeout_s']
        if not issue and fresh:
            decision_due=time.monotonic()>=self.policy.next_decision
            try:
                # Nav2 zero still stops/replanning pauses. A nonzero command is
                # only an activity lease; direction comes from path geometry.
                desired=self.desired_motion() if abs(self.command[1])+abs(self.command[2])>1e-9 else (0.,0.)
                _,v,w=self.policy.step(time.monotonic(),*desired)
            except (ValueError,TransformException) as error:
                motion_issue=str(error) or 'path_transform_missing'
                _,v,w=self.policy.stop(time.monotonic())
            motion_issue=motion_issue or self.motion_issue(v,w)
            # A fixed-radius arc can cut inside a tight Nav2 detour. At a normal
            # decision boundary only, back up while keeping the requested yaw
            # direction (or move forward if reversing) to make room for the turn.
            # It is another existing joystick action, checked over the same sweep.
            if decision_due and motion_issue=='held_action_obstructed_wait_for_replan' and v!=0. and w!=0.:
                alternative=quantize(-v,w,self.cfg['max_forward_m_s'],self.cfg['max_yaw_rad_s'])
                if self.motion_issue(*alternative[1:]) is None:
                    self.policy._set(alternative);_,v,w=alternative;motion_issue=None
            if motion_issue:_,v,w=self.policy.stop(time.monotonic())
        else:self.policy.stop(time.monotonic())
        preview=Twist();preview.linear.x=v;preview.angular.z=w;self.preview.publish(preview)
        if self.armed:
            problem=issue or self.controller_issue()
            mission_fresh=self.mission is not None and time.monotonic()-self.mission[1]<=self.cfg['command_timeout_s']
            # A live route can briefly pause for a new plan. Only an explicit
            # prior zero command may be extended, never stale nonzero motion.
            zero_hold=(not fresh and mission_fresh and (self.command is None or self.command[1:]==(0.,0.)))
            if self.mission is not None and not mission_fresh:problem=problem or 'mission_lease_expired'
            if not fresh and self.command is not None and not zero_hold:problem=problem or 'navigation_command_timeout'
            if problem:self.disarm(problem)
            elif fresh or zero_hold:
                # A zero Twist remains in the same navigation session; a future
                # nonzero command must not be mistaken for a new heading lock.
                # Repeated packets only renew the existing watchdog lease. They
                # do not choose a new action or reset the gait phase/gesture.
                self.send(v,w,True);self.reason=motion_issue or ('replanning_zero_hold' if zero_hold else 'holding_discrete_action')
        if self.action_event_revision!=self.policy.revision:
            self.action_event_revision=self.policy.revision
            self.action_pub.publish(String(data=json.dumps(dict(action=self.policy.action[0],
                action_id=self.policy.revision,vx=self.policy.action[1],wz=self.policy.action[2],
                hold_s=self.cfg['action_hold_s'],reason=motion_issue or self.reason))))
        self.state_pub.publish(String(data=json.dumps(dict(
            allow_motor_output=self.allowed,armed=self.armed,reason=self.reason,
            feedback_issue=issue,command_fresh=fresh,preview_vx=v,preview_wz=w,
            gait_scale=scale,gesture=self.gesture,action=self.policy.action[0],action_id=self.policy.revision,
            seconds_to_next_decision=max(0.,self.policy.next_decision-time.monotonic()),
            motion_issue=motion_issue,target_id=self.target_id))))


def main():
    rclpy.init();node=VelocityBridge()
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:
        node.disarm('bridge_shutdown');node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
