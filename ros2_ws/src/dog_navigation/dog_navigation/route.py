"""Explicit user invocation sends the saved reference anchors to stock Nav2."""
import argparse,json,math,time,uuid
from pathlib import Path
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.signals import SignalHandlerOptions
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from action_msgs.srv import CancelGoal
from unique_identifier_msgs.msg import UUID


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--route',required=True);parser.add_argument('--report',required=True)
    parser.add_argument('--timeout',type=float,default=600.)
    parser.add_argument('--behavior-tree',default='',help='Optional Nav2 BT file, primarily for isolated integration tests')
    args,ros=parser.parse_known_args()
    target=Path(args.report)
    if target.exists():raise ValueError('Report already exists')
    data=json.loads(Path(args.route).read_text());points=data['points_xy_m']
    indices=data.get('anchor_indices')
    if not isinstance(indices,list) or not indices or any(type(i) is not int or i<0 or i>=len(points) for i in indices):
        raise ValueError('Route must explicitly specify anchor_indices; raw path samples are not navigation goals')
    # Keep ROS alive while Ctrl-C cancels this goal and persists the result.
    rclpy.init(args=ros,signal_handler_options=SignalHandlerOptions.NO);node=Node('dog_nav_route')
    client=ActionClient(node,NavigateToPose,'/dog_nav/navigate_to_pose')
    identity=uuid.uuid4();goal_id=UUID(uuid=list(identity.bytes))
    lease=node.create_publisher(String,'/dog_nav/mission_lease',1)
    active=False;current_target_id=None;current_target_xy=None
    bridge_was_armed=False;bridge_stop=None
    def bridge_status(msg):
        nonlocal bridge_was_armed,bridge_stop
        try:
            state=json.loads(msg.data)
            if not state.get('allow_motor_output'):return
            if state.get('armed'):bridge_was_armed=True;bridge_stop=None
            else:bridge_stop=state.get('reason','bridge_disarmed')
        except (ValueError,TypeError):pass
    node.create_subscription(String,'/dog_nav/bridge_status',bridge_status,10)
    def heartbeat():
        if active:lease.publish(String(data=json.dumps(dict(mission_id=identity.hex,active=True,
            issued_monotonic=time.monotonic(),target_id=current_target_id,target_xy=current_target_xy))))
    timer=node.create_timer(.1,heartbeat)
    report={'route':args.route,'frame_id':data['frame_id'],'anchor_indices':indices,'closed':False,
            'motor_enable_requested':False,'navigation_goal_requested':True,'started_unix':time.time(),
            'mode':'ordered_single_goals','completed_targets':[]}
    handle=None
    try:
        if not client.wait_for_server(timeout_sec=30.):raise RuntimeError('Nav2 action unavailable')
        # Humble/Fast DDS may discover the server before the new client's reply
        # endpoints. Allow discovery to settle before sending a motion goal.
        discovery_deadline=time.monotonic()+2.
        while time.monotonic()<discovery_deadline:rclpy.spin_once(node,timeout_sec=.1)
        if bridge_stop:raise RuntimeError('Navigation output not armed: '+bridge_stop)
        active=True;heartbeat();deadline=time.monotonic()+args.timeout
        for i in indices:
            identity_goal=uuid.uuid4();goal_id=UUID(uuid=list(identity_goal.bytes));handle=None
            goal=NavigateToPose.Goal();goal.behavior_tree=args.behavior_tree
            p=points[i];other=points[i+1] if i+1<len(points) else points[i-1]
            dx,dy=(other[0]-p[0],other[1]-p[1]) if i+1<len(points) else (p[0]-other[0],p[1]-other[1])
            yaw=math.atan2(dy,dx);pose=PoseStamped();pose.header.frame_id=data['frame_id']
            pose.header.stamp=node.get_clock().now().to_msg()
            pose.pose.position.x=float(p[0]);pose.pose.position.y=float(p[1])
            pose.pose.orientation.z=math.sin(yaw/2);pose.pose.orientation.w=math.cos(yaw/2);goal.pose=pose
            report['current_target']=indices.index(i)+1
            current_target_id=identity.hex+':'+str(report['current_target']);current_target_xy=list(map(float,p));heartbeat()
            node.get_logger().info('Starting target '+str(report['current_target'])+'/'+str(len(indices)))
            pending=client.send_goal_async(goal,goal_uuid=goal_id)
            rclpy.spin_until_future_complete(node,pending,timeout_sec=10.)
            if not pending.done():raise RuntimeError('Goal acknowledgement timeout')
            handle=pending.result()
            if not handle.accepted:raise RuntimeError('Nav2 rejected target')
            report['accepted']=True;result=handle.get_result_async()
            while rclpy.ok() and not result.done() and time.monotonic()<deadline:
                rclpy.spin_once(node,timeout_sec=.1)
                if bridge_stop:raise RuntimeError('Navigation output stopped: '+bridge_stop)
            if not result.done():raise RuntimeError('Route timeout')
            report['action_status']=result.result().status
            if report['action_status']!=4:raise RuntimeError('Target did not complete')
            report['completed_targets'].append(dict(target=report['current_target'],route_index=i,unix=time.time()))
        report['success']=True
    except (KeyboardInterrupt,Exception) as error:
        report.update(success=False,error=str(error) or type(error).__name__)
        if handle:
            cancel=handle.cancel_goal_async();rclpy.spin_until_future_complete(node,cancel,timeout_sec=3.)
        elif active:
            # A lost acknowledgement does not prove the server rejected the goal.
            # Cancel only this request UUID, never an unrelated operator's goal.
            cancellation=node.create_client(CancelGoal,'/dog_nav/navigate_to_pose/_action/cancel_goal')
            if cancellation.wait_for_service(timeout_sec=2.):
                request=CancelGoal.Request();request.goal_info.goal_id=goal_id
                pending=cancellation.call_async(request);rclpy.spin_until_future_complete(node,pending,timeout_sec=3.)
    finally:
        active=False;timer.cancel()
        report['finished_unix']=time.time();target.parent.mkdir(parents=True,exist_ok=True)
        target.write_text(json.dumps(report,indent=2));print(json.dumps(report))
        if rclpy.ok():
            for _ in range(3):
                lease.publish(String(data=json.dumps(dict(mission_id=identity.hex,active=False,issued_monotonic=time.monotonic()))))
                rclpy.spin_once(node,timeout_sec=.1)
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()


if __name__=='__main__':main()
