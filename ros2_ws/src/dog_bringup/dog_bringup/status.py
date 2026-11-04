"""Read-only system status; no action clients, services or command publishers."""
import json
import subprocess
import time
from .configuration import project_root, read_site


def main():
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import String
    root = project_root()
    site = read_site(root)
    print('Release:', site['release'])
    print('Project:', root)
    print('ROS_DOMAIN_ID:', __import__('os').environ.get('ROS_DOMAIN_ID', '0'))
    rclpy.init()
    node = Node('dog_final_status')
    data = {}
    def receive(key, msg):
        try:
            data[key] = json.loads(msg.data)
        except ValueError:
            data[key] = {'message': msg.data[:200]}
    subscriptions = [node.create_subscription(String, topic, lambda m, key=key: receive(key, m), 1)
                     for key, topic in [('controller', '/dog_ik_walk_imu/status'),
                                         ('navigation', '/dog_nav/bridge_status')]]
    try:
        until = time.monotonic() + 2.5
        while time.monotonic() < until:
            rclpy.spin_once(node, timeout_sec=0.1)
        print('Nodes:', ', '.join(sorted(n for n in node.get_node_names() if n.startswith('dog_'))))
        for key, value in data.items():
            print(key + ':', json.dumps({k: value[k] for k in value if k in
                  ('allow_actuation', 'allowed', 'armed', 'enabled', 'mode', 'state', 'reason', 'message', 'connected')},
                  ensure_ascii=False))
        if not data:
            print('No controller/bridge status received in 2.5 s.')
    finally:
        node.destroy_node()
        rclpy.shutdown()
    result = subprocess.run(['systemctl', '--user', 'is-active', 'dog_ik_controller.service',
                             'dog_imu.service', 'dog_web.service'], text=True, capture_output=True)
    print('Legacy services (controller / IMU / web):', result.stdout.strip().replace('\n', ' / '))
    print('New vision viewer:', 'http://192.168.30.176:' + str(site['vision_port']))
