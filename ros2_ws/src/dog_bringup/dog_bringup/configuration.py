"""File/configuration handling; importing this module never opens hardware."""
from datetime import datetime, timezone, timedelta
import json
import os
from pathlib import Path

PROFILES = {
    'monitor': frozenset({'imu', 'controller'}),
    'vision': frozenset({'vision'}),
    'mapping': frozenset({'lidar', 'mapping', 'grid'}),
    'survey': frozenset({'lidar', 'mapping', 'grid', 'vision'}),
    'plan': frozenset({'plan'}),
    'nav-monitor': frozenset({'lidar', 'mapping', 'navigation', 'vision'}),
    'model': frozenset({'model'}),
}


def project_root(value=None):
    if value or os.environ.get('DOG_FINAL_ROOT'):
        candidates = [Path(value or os.environ['DOG_FINAL_ROOT']).expanduser().resolve()]
    else:
        candidates = list(Path(__file__).resolve().parents)
    for root in candidates:
        if (root / 'config/site.json').is_file() and (root / 'runtime').is_dir():
            return root
    raise ValueError('Cannot locate dog_final; use ./dog or set DOG_FINAL_ROOT.')


def read_site(root):
    site = json.loads((Path(root) / 'config/site.json').read_text(encoding='utf-8'))
    if not 1024 <= int(site['vision_port']) <= 65535:
        raise ValueError('vision_port must be between 1024 and 65535')
    return site


def resolve(root, value):
    path = Path(value).expanduser()
    return path if path.is_absolute() else Path(root) / path


def create_run(root, mode, requested=''):
    if mode not in PROFILES:
        raise ValueError('Unknown mode: ' + mode)
    stamp = datetime.now(timezone(timedelta(hours=8))).strftime('%Y%m%d_%H%M%S_%f')
    run = resolve(root, requested) if requested else Path(root) / 'runs' / (stamp + '_' + mode)
    run.mkdir(parents=True, exist_ok=False)
    (run / 'vision').mkdir()
    (run / 'grid').mkdir()
    (run / 'run.json').write_text(json.dumps({
        'release': 'dog_final_20261030', 'mode': mode, 'pid': os.getpid(),
        'directory': str(run), 'allow_actuation': False, 'allow_motor_output': False,
        'start_ticks': Path('/proc/self/stat').read_text().split(') ', 1)[1].split()[19]
                       if Path('/proc/self/stat').exists() else None,
        'started_at': datetime.now(timezone(timedelta(hours=8))).isoformat()
    }, indent=2), encoding='utf-8')
    state = Path(root) / 'state'
    state.mkdir(exist_ok=True)
    (state / 'latest_run.json').write_text((run / 'run.json').read_text(), encoding='utf-8')
    if 'grid' in PROFILES[mode]:
        (state / 'mapping_run.json').write_text((run / 'run.json').read_text(), encoding='utf-8')
    return run


def prepare_deployment(root, run, site):
    root, run = Path(root), Path(run)
    cfg = json.loads((root / 'config/source/deployment.json').read_text(encoding='utf-8'))
    cfg.update(runtime_root=str(root / 'runtime'), motor_port=site['motor_port'],
               log_directory=str(run / 'controller'), standpose_file=str(root / 'config/source/standpose.json'),
               trajectory_cache=str(root / 'config/source/heading_trajectory.npz'),
               trial='dog_final_monitor_only', allow_actuation=False,
               authorization='Monitor-only organization release; no hardware motion authorized.')
    if cfg.get('teleop'):
        cfg['teleop']['trajectory_cache'] = str(root / 'config/source/teleop_table.npz')
    (run / 'controller').mkdir()
    path = run / 'deployment.json'
    path.write_text(json.dumps(cfg, indent=2, ensure_ascii=False), encoding='utf-8')
    return path
