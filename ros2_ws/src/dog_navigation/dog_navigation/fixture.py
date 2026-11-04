"""Kinematic SOFTWARE test only: isolated DDS domain, no serial or motor topics."""
import argparse,json,math,os,time
from pathlib import Path
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped,Twist
from nav_msgs.msg import Odometry,Path as NavPath
from sensor_msgs.msg import LaserScan
from tf2_ros import TransformBroadcaster,StaticTransformBroadcaster


class Fixture(Node):
    def __init__(self,route,output,dynamic_obstacle=False):
        super().__init__('dog_nav_fixture')
        if os.environ.get('ROS_DOMAIN_ID')!='49' or os.environ.get('ROS_LOCALHOST_ONLY')!='1':
            raise RuntimeError('Fixture requires ROS_DOMAIN_ID=49 ROS_LOCALHOST_ONLY=1')
        points=json.loads(Path(route).read_text())['points_xy_m']
        self.x,self.y=points[0];self.yaw=math.atan2(points[1][1]-self.y,points[1][0]-self.x)
        self.v=self.w=0.;self.received=0.;self.started=None;self.last=time.monotonic();self.injected=False
        self.file=Path(output).open('x');self.frames=0
        self.dynamic_obstacle=dynamic_obstacle;self.obstacle_logged=False
        self.scan=self.create_publisher(LaserScan,'/dog_nav/scan',10)
        self.tf=TransformBroadcaster(self);self.static=StaticTransformBroadcaster(self)
        identity=TransformStamped();identity.header.frame_id='dog_lio_map';identity.child_frame_id='dog_odom'
        identity.header.stamp=self.get_clock().now().to_msg();identity.transform.rotation.w=1.;self.static.sendTransform(identity)
        self.odom=self.create_publisher(Odometry,'/dog_nav/odom',10)
        self.create_subscription(Twist,'/dog_nav/velocity_preview',self.command,1)
        self.create_subscription(NavPath,'/dog_nav/plan',self.path,1)
        self.create_timer(.02,self.tick)

    def path(self,msg):
        self.file.write(json.dumps({'kind':'path','xy':[[p.pose.position.x,p.pose.position.y] for p in msg.poses]})+'\n');self.file.flush()

    def command(self,msg):
        self.v=msg.linear.x;self.w=msg.angular.z;self.received=time.monotonic()
        if self.started is None and abs(self.v)+abs(self.w)>1e-6:self.started=self.received

    def tick(self):
        now=time.monotonic();dt=min(now-self.last,.1);self.last=now
        if now-self.received>.5:self.v=self.w=0.
        self.x+=self.v*math.cos(self.yaw)*dt;self.y+=self.v*math.sin(self.yaw)*dt;self.yaw+=self.w*dt
        if self.started and now-self.started>20. and not self.injected:
            self.y+=.10;self.yaw-=math.radians(10);self.injected=True
            self.file.write(json.dumps({'kind':'disturbance','dy_m':.10,'dyaw_deg':-10,'elapsed':now-self.started})+'\n')
        stamp=self.get_clock().now().to_msg();tf=TransformStamped()
        tf.header.stamp=stamp;tf.header.frame_id='dog_odom';tf.child_frame_id='dog_base_link'
        tf.transform.translation.x=self.x;tf.transform.translation.y=self.y
        tf.transform.rotation.z=math.sin(self.yaw/2);tf.transform.rotation.w=math.cos(self.yaw/2);self.tf.sendTransform(tf)
        od=Odometry();od.header=tf.header;od.child_frame_id=tf.child_frame_id
        od.pose.pose.position.x=self.x;od.pose.pose.position.y=self.y;od.pose.pose.orientation=tf.transform.rotation
        od.twist.twist.linear.x=self.v;od.twist.twist.angular.z=self.w;self.odom.publish(od)
        self.frames+=1
        if self.frames%5==0:
            if self.dynamic_obstacle:
                scan=LaserScan();scan.header.stamp=stamp;scan.header.frame_id='dog_base_link'
                scan.angle_min=-math.pi;scan.angle_max=math.pi;scan.angle_increment=math.pi/360
                scan.range_min=.05;scan.range_max=12.;scan.scan_time=.1
                active=self.started is not None and now-self.started>=8.
                if active and not self.obstacle_logged:
                    self.file.write(json.dumps({'kind':'obstacle_appeared','center':[1.25,.38],'radius':.20,'elapsed':now-self.started})+'\n')
                    self.obstacle_logged=True
                ranges=[];cx,cy=1.25-self.x,.38-self.y
                for i in range(721):
                    a=self.yaw-math.pi+i*math.pi/360;dot=cx*math.cos(a)+cy*math.sin(a)
                    discriminant=dot*dot-(cx*cx+cy*cy-.20**2)
                    distance=dot-math.sqrt(discriminant) if active and discriminant>=0 else float('inf')
                    ranges.append(distance if .05<=distance<=12. else float('inf'))
                scan.ranges=ranges;self.scan.publish(scan)
            self.file.write(json.dumps({'kind':'sample','elapsed':0 if self.started is None else now-self.started,
                'x':self.x,'y':self.y,'yaw':self.yaw,'vx':self.v,'wz':self.w})+'\n');self.file.flush()


def main():
    parser=argparse.ArgumentParser();parser.add_argument('--route',required=True);parser.add_argument('--output',required=True)
    parser.add_argument('--dynamic-obstacle',action='store_true')
    args,ros=parser.parse_known_args();rclpy.init(args=ros);node=Fixture(args.route,args.output,args.dynamic_obstacle)
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:node.file.close();node.destroy_node();rclpy.shutdown()
