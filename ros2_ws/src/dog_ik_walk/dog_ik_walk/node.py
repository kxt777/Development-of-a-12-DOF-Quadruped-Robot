"""ROS deployment starts idle and reads serial feedback only."""
from pathlib import Path
from datetime import datetime
import json,sys,time,threading
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState,Imu
from std_msgs.msg import String,Float64MultiArray
from std_srvs.srv import Trigger,SetBool

class IkWalkNode(Node):
    def __init__(self):
        super().__init__('dog_ik_walk')
        self.declare_parameter('deployment_config','')
        self.declare_parameter('allow_actuation',False)
        self.allow=bool(self.get_parameter('allow_actuation').value)
        self.deployment=json.loads(Path(self.get_parameter('deployment_config').value).read_text())
        self.imu_used=bool(self.deployment.get('imu_used',False))
        self.namespace=self.deployment.get('ros_namespace','/dog_ik_walk')
        if not self.namespace.startswith('/dog_'):raise ValueError('Project interfaces require dog_ prefix')
        root=Path(self.deployment['runtime_root']);sys.path.insert(0,str(root/'dog_tradition_real'));sys.path.insert(0,str(root))
        from dog_runtime import load_settings
        from dog_link import MotorLink
        from dog_ik_walk.controller import IkWalkController,WriteGate
        config,calibration=load_settings(root/'dog_tradition_real/dog_config.json')
        if self.imu_used:config['allow_actuation']=self.allow
        config['runtime']['stand_seconds']=self.deployment['stand_seconds']
        config['motor']['port']=str(Path(self.deployment['motor_port']).resolve())
        controller_type=IkWalkController
        if self.imu_used:
            from dog_ik_walk.imu_controller import ImuWalkController
            controller_type=ImuWalkController
            if self.deployment.get('teleop'):
                from dog_ik_walk.teleop import TeleopController
                controller_type=TeleopController
        self.controller=controller_type(config,calibration,self.deployment,WriteGate(MotorLink(config['motor']),self.allow))
        self.demo_thread=None;self.demo_status={'state':'not_requested'};self.demo_cancel=threading.Event()
        self.pub=self.create_publisher(String,self.namespace+'/status',10)
        self.feedback_pub=self.create_publisher(JointState,self.namespace+'/motor_feedback' if self.imu_used else '/dog_motor_feedback',qos_profile_sensor_data)
        self.joint_pub=self.create_publisher(JointState,self.namespace+'/joint_feedback' if self.imu_used else '/dog_joint_feedback',qos_profile_sensor_data)
        self.command_pub=self.create_publisher(Float64MultiArray,self.namespace+'/last_mit_command',10)
        self.imu_error='waiting for IMU'
        if self.imu_used:
            self.imu_sub=self.create_subscription(Imu,self.deployment['imu_topic'],self.on_imu,qos_profile_sensor_data)
            self.body_imu_pub=self.create_publisher(Imu,'/dog_imu/body',qos_profile_sensor_data)
        self.action_services=[]
        for action in ('enable','zero','stand','walk','walk_right','walk_left','stop','teleop_stop','disable','demo','demo_right','demo_left'):
            self.action_services.append(self.create_service(Trigger,self.namespace+'/'+action,lambda req,res,a=action:self.on_action(a,res)))
        if self.deployment.get('teleop'):
            self.teleop_sub=self.create_subscription(String,self.namespace+'/teleop',self.on_teleop,1)
            self.navigation_sub=self.create_subscription(String,self.namespace+'/nav_velocity',lambda m:self.on_teleop(m,'navigation'),1)
            self.navigation_service=self.create_service(SetBool,self.namespace+'/navigation_mode',self.on_navigation_mode)
        folder=Path(self.deployment['log_directory']);folder.mkdir(parents=True,exist_ok=True)
        self.log=(folder/('ros_ik_'+datetime.now().strftime('%Y%m%d_%H%M%S')+'.jsonl')).open('x',buffering=1)
        self.last_sequence=0;self.last_feedback_frames=-1;self.last_status=0.
        self.timer=self.create_timer(.02,self.publish_status)
        # New IMU monitor does not open the motor port or consume the old reader's bytes.
        if self.allow or not self.imu_used:self.controller.start()
        self.get_logger().info(f'IK ROS ready; allow_actuation={self.allow}; no automatic enable/motion')

    def write_log(self,record):
        self.log.write(json.dumps(record)+'\n')
        # Bound the continuous web-controller journal; prior trial logs remain.
        if self.deployment.get('teleop') and self.log.tell()>=50*1024*1024:
            path=Path(self.log.name);self.log.close()
            for index in (2,1):
                old=Path(str(path)+'.'+str(index))
                if old.exists():old.replace(Path(str(path)+'.'+str(index+1)))
            path.replace(Path(str(path)+'.1'));self.log=path.open('w',buffering=1)

    def on_navigation_mode(self,request,response):
        try:
            self.controller.select_velocity_source('navigation' if request.data else 'joystick')
            response.success=True;response.message='Velocity source selected; no motion or enable requested'
        except (ValueError,RuntimeError) as error:response.success=False;response.message=str(error)
        return response

    def on_teleop(self,msg,source='joystick'):
        try:
            if not self.allow:raise RuntimeError('Monitor-only deployment')
            if self.demo_thread and self.demo_thread.is_alive():raise RuntimeError('Demo running')
            self.controller.accept_velocity(json.loads(msg.data),source=source)
        except (ValueError,RuntimeError,KeyError,TypeError) as error:
            self.controller.telemetry['last_rejection']=str(error)

    def on_imu(self,msg):
        try:
            age=(self.get_clock().now().nanoseconds-(msg.header.stamp.sec*1000000000+msg.header.stamp.nanosec))/1e9
            if not 0<=age<=self.deployment['heading']['max_imu_age_s']:raise ValueError('Stale ROS IMU stamp')
            if msg.orientation_covariance[0]<0 or msg.angular_velocity_covariance[0]<0:raise ValueError('IMU channel unavailable')
            q=msg.orientation;g=msg.angular_velocity;a=msg.linear_acceleration
            sample=self.controller.heading.receive([q.x,q.y,q.z,q.w],[g.x,g.y,g.z],[a.x,a.y,a.z])
            body=Imu();body.header.stamp=msg.header.stamp;body.header.frame_id='dog_base_link'
            body.orientation.x,body.orientation.y,body.orientation.z,body.orientation.w=sample['quaternion_xyzw']
            body.angular_velocity.x,body.angular_velocity.y,body.angular_velocity.z=sample['gyro_body']
            body.linear_acceleration.x,body.linear_acceleration.y,body.linear_acceleration.z=sample['accel_body']
            self.body_imu_pub.publish(body);self.imu_error=None
        except (ValueError,RuntimeError) as error:self.imu_error=str(error)

    def on_action(self,action,response):
        command_received=time.monotonic()
        try:
            if not self.allow:raise RuntimeError('Monitor-only deployment: no motor commands allowed')
            if action in ('demo','demo_right','demo_left'):
                if action in ('demo_right','demo_left') and not self.imu_used:raise RuntimeError('Turning requires the IMU trajectory controller')
                if self.demo_thread and self.demo_thread.is_alive():raise RuntimeError('Demo already running')
                if self.imu_used:
                    heading=self.controller.heading
                    heading.activate_command(heading.capture_command(action,command_received))
                self.demo_cancel.clear()
                self.demo_thread=threading.Thread(target=self.run_demo,daemon=True,name='dog_walk_demo');self.demo_thread.start()
            else:
                if action in ('stop','stand','zero','disable') and hasattr(self.controller,'stop_velocity'):
                    self.controller.stop_velocity('released')
                if self.demo_thread and self.demo_thread.is_alive():
                    if action not in ('stop','disable'):raise RuntimeError('Demo running; stop it before requesting another action')
                    self.demo_cancel.set()
                if action=='stop':action='stand'
                if action=='zero':
                    self.controller.stand_completed=False
                    self.controller.move_to_pose(np.zeros(12),self.deployment['zero_seconds'],'zero')
                elif action=='teleop_stop':
                    if not hasattr(self.controller,'stop_velocity'):raise RuntimeError('Teleop is not configured')
                    self.controller.stop_velocity()
                elif action in ('walk_left','walk_right'):
                    if not self.imu_used:raise RuntimeError('Turning requires the IMU trajectory controller')
                    # Commit the direction before the queued walk can read it.
                    with self.controller.heading.lock:
                        self.controller.request('walk',command_received=command_received)
                        self.controller.heading.command['source']=action
                elif self.imu_used and action=='walk':
                    self.controller.request(action,command_received=command_received)
                else:self.controller.request(action)
            response.success=True;response.message='Requested '+action+'; check '+self.namespace+'/status for completion'
        except Exception as error:response.success=False;response.message=str(error)
        return response

    def publish_status(self):
        c=self.controller;s=c.status()
        if self.imu_used and not self.allow and c.heading.reference is not None:
            try:c.heading.step(.02)
            except RuntimeError:pass
        if s['feedback'] and s['frames']!=self.last_feedback_frames:
            msg=JointState();msg.header.stamp=self.get_clock().now().to_msg();msg.header.frame_id='motor_coordinates'
            order=[f'{leg}_{n}' for leg in ('fl','fr','bl','br') for n in (1,2,3)]
            msg.name=[order[r['id']-1] for r in s['feedback']]
            msg.position=[float(r['position']) for r in s['feedback']];msg.velocity=[float(r['velocity']) for r in s['feedback']];msg.effort=[float(r['torque']) for r in s['feedback']]
            self.feedback_pub.publish(msg);self.last_feedback_frames=s['frames']
            from dog_ik_walk.controller import feedback_to_model
            try:usable=c._feedback()
            except RuntimeError:usable=None
            if usable is not None:
                q,dq,tau=feedback_to_model(usable,c.calibration)
                joint=JointState();joint.header.stamp=msg.header.stamp;joint.header.frame_id='base_link'
                joint.name=c.kin.joint_names;joint.position=q.tolist();joint.velocity=dq.tolist();joint.effort=tau.tolist();self.joint_pub.publish(joint)
        if c.last_command and c.last_command['sequence']!=self.last_sequence:
            self.last_sequence=c.last_command['sequence'];record=dict(c.last_command);record.update(kind='sent_command',imu_used=self.imu_used)
            self.write_log(record)
            msg=Float64MultiArray();msg.data=sum([c.last_command[k] for k in ('q_des','dq_des','kp','kd','tau_ff')],[]);self.command_pub.publish(msg)
        stamp=time.monotonic()
        if stamp-self.last_status<.5:return
        self.last_status=stamp
        try:c._feedback();feedback_issue=None
        except RuntimeError as error:feedback_issue=str(error)
        s.update(kind='status',allow_actuation=self.allow,usb_commands_written=c.link.writes,imu_used=self.imu_used,
                 feedback_usable=feedback_issue is None,feedback_issue=feedback_issue,
                 body_height_target_m=self.deployment['body_height_m'],last_command_sequence=self.last_sequence,
                 feedback_note='Motor coordinates; cached CAN snapshot, per-motor freshness unknown',
                 trajectory_preflight=c.preflight,demo=self.demo_status)
        if self.imu_used:s.update(heading=c.heading.status(),imu_error=self.imu_error,
                                motor_port_opened=self.allow,monitor_note='Read-only IMU correction preview; motor port remains closed' if not self.allow else '')
        msg=String();msg.data=json.dumps(s);self.pub.publish(msg);self.write_log(s)

    def run_demo(self):
        c=self.controller;report={'state':'running','started_monotonic':time.monotonic(),'imu_used':self.imu_used,'preflight':c.preflight}
        if self.imu_used:report['heading_command']=c.heading.status()['command']
        self.demo_status=report
        first_sequence=c.sequence
        try:
            from dog_ik_walk.sequence import run_sequence
            run_sequence(c,self.deployment,report,self.demo_cancel)
        except Exception as error:
            report.update(state='stopped',error=str(error))
            c.disable('IK walk demonstration interrupted: '+str(error))
            deadline=time.monotonic()+c.options['disable_timeout']+.5
            while c.status()['disabling'] and time.monotonic()<deadline:time.sleep(.02)
        finally:
            report['disable_state']=c.status()['disable_state'];report['finished_monotonic']=time.monotonic()
            report['motors_enabled_after_sequence']=c.status()['enabled']
            samples=[r for r in list(c.history) if r['sequence']>first_sequence];report['commands']=samples
            if samples:
                report['max_feedback_torque_nm']=float(np.abs([r['measured_torque_model_nm'] for r in samples]).max())
                report['joint_tracking_rms_rad']=float(np.sqrt(np.mean(np.array([r['tracking_error_rad'] for r in samples])**2)))
            path=Path(self.deployment['log_directory'])/('dog_walk_demo_'+datetime.now().strftime('%Y%m%d_%H%M%S')+'.json')
            path.write_text(json.dumps(report,indent=2));self.demo_status={k:v for k,v in report.items() if k!='commands'};self.demo_status['report']=str(path)

    def destroy_node(self):
        self.demo_cancel.set()
        if self.allow and (self.controller.status()['enabled'] or self.controller.link.writes):
            self.controller.disable('ROS controller closing')
            deadline=time.monotonic()+self.controller.options['disable_timeout']+.5
            while self.controller.status()['disabling'] and time.monotonic()<deadline:time.sleep(.01)
        self.controller.close();self.log.close();super().destroy_node()

def main():
    rclpy.init();node=None
    try:node=IkWalkNode();rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:
        if node is not None:node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
