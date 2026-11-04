"""Generate the SDK network configuration; never modifies a NIC or contacts hardware."""
import argparse
import ipaddress
import json
from pathlib import Path


def make_config(host_ip, lidar_ip, model='mid360s'):
    host = ipaddress.IPv4Address(host_ip)
    lidar = ipaddress.IPv4Address(lidar_ip)
    for address in (host, lidar):
        if address.is_multicast or address.is_unspecified or address.is_loopback:
            raise ValueError('Use real unicast IPv4 addresses for host and lidar')
    if host == lidar:
        raise ValueError('Host and lidar must have distinct addresses')
    host_info = {}
    ports = {'cmd_data': 56100, 'push_msg': 56200, 'point_data': 56300,
             'imu_data': 56400, 'log_data': 56500}
    for name, port in ports.items():
        host_info[name + '_ip'] = str(host) if name != 'log_data' else ''
        host_info[name + '_port'] = port + 1
    if model not in ('mid360', 'mid360s'):
        raise ValueError('Supported models: mid360, mid360s')
    model_key = 'Mid360s' if model == 'mid360s' else 'MID360'
    if model == 'mid360s':
        host_info = [dict(host_ip=str(host), **{name + '_port': port + 1 for name, port in ports.items()})]
    return {
        'lidar_summary_info': {'lidar_type': 8},
        model_key: {
            'lidar_net_info': {name + '_port': port for name, port in ports.items()},
            'host_net_info': host_info,
        },
        'lidar_configs': [{
            'ip': str(lidar), 'pcl_data_type': 1, 'pattern_mode': 0,
            # Keep raw sensor coordinates. This is NOT a calibrated base-to-lidar TF.
            'extrinsic_parameter': dict(roll=0.0, pitch=0.0, yaw=0.0, x=0, y=0, z=0),
        }],
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--host-ip', required=True)
    parser.add_argument('--lidar-ip', required=True)
    parser.add_argument('--model', choices=('mid360s', 'mid360'), default='mid360s')
    parser.add_argument('--output', required=True, type=Path)
    args = parser.parse_args()
    config = make_config(args.host_ip, args.lidar_ip, args.model)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    with args.output.open('x', encoding='utf-8') as stream:
        json.dump(config, stream, indent=2)
    print(args.output)


if __name__ == '__main__':
    main()
