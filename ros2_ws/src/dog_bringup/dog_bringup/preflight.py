"""Reject duplicate device owners without stopping any existing process."""
from pathlib import Path
import os
import socket

_locks = []


def acquire_locks(root, wanted):
    import fcntl
    folder = Path(root) / 'state/locks'
    folder.mkdir(parents=True, exist_ok=True)
    new = []
    try:
        # The live and offline Nav2 stacks share a single resource lock.
        resources = {'nav2' if name in {'navigation', 'plan'} else name for name in wanted}
        for name in sorted(resources):
            handle = (folder / (name + '.lock')).open('a+')
            try:
                fcntl.flock(handle, fcntl.LOCK_EX | fcntl.LOCK_NB)
            except BlockingIOError:
                handle.close()
                raise RuntimeError(f'Another dog_final launch owns {name}; stop its foreground launch first.')
            new.append(handle)
    except Exception:
        for handle in new:
            handle.close()
        raise
    _locks.extend(new)


def process_commands():
    result = []
    for entry in Path('/proc').glob('[0-9]*'):
        if int(entry.name) == os.getpid():
            continue
        try:
            argv = (entry / 'cmdline').read_bytes().replace(b'\0', b' ').decode(errors='replace')
        except (OSError, PermissionError):
            continue
        if argv:
            result.append((int(entry.name), argv))
    return result


def conflicts(wanted, processes):
    patterns = {
        'controller': ('dog_ik_walk.node', 'dog_ik_walk_node', 'ik_walk_imu.launch.py'),
        'imu': ('dm_imu_node', 'ik_walk_imu.launch.py'),
        'lidar': ('livox_ros_driver2_node', 'dog_lidar.launch.py'),
        'mapping': ('fastlio_mapping', 'dog_mapping.launch.py'),
        'vision': ('dog_custom_yolo.py', '/dog_yolo ', 'depthai_ros_driver_v3/driver.launch.py'),
        'plan': ('/planner_server', 'dog_planning_offline.launch.py', 'dog_navigation_live.launch.py'),
        'navigation': ('/planner_server', '/controller_server', 'dog_navigation_live.launch.py'),
    }
    found = [(name, pid, command) for name in sorted(wanted) for pid, command in processes
             if any(pattern in command for pattern in patterns.get(name, ()))]
    if 'vision' in wanted:
        for pid, command in processes:
            if (('depthai_ros_driver_v3' in command and 'driver.launch.py' in command) or
                    ('component_container' in command and 'dog_oak' in command)):
                found.append(('vision', pid, command))
    return sorted(set(found))


def check(wanted, vision_port=None):
    found = conflicts(wanted, process_commands())
    if found:
        lines = [f'{name}: PID {pid}: {command[:220]}' for name, pid, command in found]
        raise RuntimeError('Existing processes own required devices/nodes. Nothing was stopped.\n'
                           + '\n'.join(lines) + '\nUse ./dog status; coordinate migration separately.')
    if 'vision' in wanted:
        with socket.socket() as sock:
            try:
                sock.bind(('0.0.0.0', int(vision_port)))
            except OSError as exc:
                raise RuntimeError(f'Vision port {vision_port} is already occupied') from exc
