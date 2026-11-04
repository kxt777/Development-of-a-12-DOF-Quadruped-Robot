"""Request and audit an actual Nav2 path. Contains no planning algorithm."""
import argparse
import json
import math
from pathlib import Path
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid
from nav2_msgs.action import ComputePathToPose


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--start', nargs=2, type=float, default=[0.6, 0.0])
    parser.add_argument('--goal', nargs=2, type=float, default=[1.0, 2.7])
    parser.add_argument('--output', required=True)
    args = parser.parse_args()
    output = Path(args.output)
    if output.exists():
        raise FileExistsError(output)
    rclpy.init()
    node = rclpy.create_node('dog_plan_probe', namespace='/dog_nav')
    maps = []
    qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                     reliability=ReliabilityPolicy.RELIABLE)
    sub = node.create_subscription(OccupancyGrid, '/dog_nav/map', maps.append, qos)
    client = ActionClient(node, ComputePathToPose, '/dog_nav/compute_path_to_pose')
    try:
        deadline = time.monotonic() + 30
        while not maps and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.2)
        if not maps or not client.wait_for_server(timeout_sec=30):
            raise RuntimeError('Map or Nav2 action unavailable')
        grid = maps[-1]
        def cell(x, y):
            ix = math.floor((x-grid.info.origin.position.x)/grid.info.resolution)
            iy = math.floor((y-grid.info.origin.position.y)/grid.info.resolution)
            if not (0 <= ix < grid.info.width and 0 <= iy < grid.info.height):
                return None
            return int(grid.data[iy*grid.info.width+ix])
        assert cell(*args.start) == 0 and cell(*args.goal) == 0, 'Endpoints must be observed free cells'
        def pose(xy):
            p = PoseStamped()
            p.header.frame_id = grid.header.frame_id
            p.header.stamp = node.get_clock().now().to_msg()
            p.pose.position.x, p.pose.position.y = xy
            p.pose.orientation.w = 1.0
            return p
        goal = ComputePathToPose.Goal()
        goal.start, goal.goal = pose(args.start), pose(args.goal)
        goal.use_start, goal.planner_id = True, 'GridBased'
        future = client.send_goal_async(goal)
        rclpy.spin_until_future_complete(node, future, timeout_sec=30)
        if not future.done() or not future.result().accepted:
            raise RuntimeError('Nav2 did not accept goal')
        result_future = future.result().get_result_async()
        rclpy.spin_until_future_complete(node, result_future, timeout_sec=30)
        if not result_future.done():
            raise RuntimeError('Nav2 result timed out')
        wrapped = result_future.result()
        result = wrapped.result
        points = [[p.pose.position.x, p.pose.position.y] for p in result.path.poses]
        def sample_segments(poly):
            values = []
            for a, b in zip(poly, poly[1:]):
                n = max(1, math.ceil(math.dist(a, b)/(grid.info.resolution/10)))
                values.extend(cell(a[0]+(b[0]-a[0])*i/n, a[1]+(b[1]-a[1])*i/n) for i in range(n+1))
            return values
        samples = sample_segments(points)
        direct = sample_segments([args.start, args.goal])
        data = dict(action='/dog_nav/compute_path_to_pose', status=int(wrapped.status),
                    planner='nav2_navfn_planner/NavfnPlanner', frame=result.path.header.frame_id,
                    start=args.start, goal=args.goal, points=points,
                    path_length_m=sum(math.dist(a,b) for a,b in zip(points,points[1:])),
                    planning_time_s=result.planning_time.sec+result.planning_time.nanosec/1e9,
                    sampled_path_cells=len(samples),
                    nonfree_samples=sum(v != 0 for v in samples),
                    direct_line_occupied_samples=sum(v == 100 for v in direct),
                    endpoint_error_m=math.dist(points[-1],args.goal) if points else None,
                    map=dict(width=grid.info.width,height=grid.info.height,resolution=grid.info.resolution),
                    mode='offline explicit start; point robot; no inflation; no motor commands')
        output.parent.mkdir(parents=True, exist_ok=True)
        output.write_text(json.dumps(data, indent=2)+'\n')
        print(json.dumps({k:v for k,v in data.items() if k != 'points'}, indent=2))
        if wrapped.status != 4 or len(points)<2 or any(v != 0 for v in samples):
            raise RuntimeError('Nav2 path did not pass free-space audit; result retained')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
