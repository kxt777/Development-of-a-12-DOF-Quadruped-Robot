#!/usr/bin/env python3
"""Static and pure-Python bringup checks. Never import a motor or sensor node."""
import ast
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
import unittest
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'ros2_ws/src/dog_bringup'))
from dog_bringup.configuration import PROFILES, create_run, prepare_deployment, read_site, resolve
from dog_bringup.preflight import conflicts


class BringupContract(unittest.TestCase):
    def test_all_profiles_have_no_automatic_motion_stage(self):
        self.assertEqual(set(PROFILES), {'monitor', 'mapping', 'vision', 'survey', 'plan', 'nav-monitor', 'model'})
        self.assertEqual([k for k, v in PROFILES.items() if 'controller' in v], ['monitor'])

    def test_relocated_monitor_config_does_not_inherit_historical_authorization(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            (root/'config/source').mkdir(parents=True)
            shutil.copy2(ROOT/'config/source/deployment.json', root/'config/source/deployment.json')
            run = create_run(root, 'monitor')
            cfg = json.loads(prepare_deployment(root, run, read_site(ROOT)).read_text())
            self.assertIs(cfg['allow_actuation'], False)
            self.assertEqual(cfg['runtime_root'], str(root/'runtime'))
            self.assertIn('no hardware motion', cfg['authorization'])
            self.assertNotIn('/home/robot/dog_ik_walk_imu_ws', json.dumps(cfg))

    def test_existing_run_is_never_reused(self):
        with tempfile.TemporaryDirectory() as directory:
            run = create_run(directory, 'mapping', 'new')
            with self.assertRaises(FileExistsError):
                create_run(directory, 'mapping', str(run))

    def test_nav_monitor_does_not_replace_savable_mapping_run(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            first = create_run(root, 'mapping')
            create_run(root, 'nav-monitor')
            self.assertEqual(json.loads((root/'state/mapping_run.json').read_text())['directory'], str(first))

    def test_active_legacy_controller_blocks_only_controller_profile(self):
        processes = [(1004, '/usr/bin/python3 -c from dog_ik_walk.node import main; main()')]
        self.assertTrue(conflicts(PROFILES['monitor'], processes))
        self.assertFalse(conflicts(PROFILES['survey'], processes))

    def test_camera_conflict_detects_driver_and_component_forms(self):
        for command in ['ros2 launch depthai_ros_driver_v3 driver.launch.py name:=dog_oak',
                        '/opt/ros/humble/lib/rclcpp_components/component_container --ros-args -r __node:=dog_oak_container']:
            self.assertTrue(conflicts(PROFILES['vision'], [(123, command)]))

    def test_required_assets_and_model_hash(self):
        import hashlib
        site = read_site(ROOT)
        for key in ['lidar_config', 'camera_config', 'mounting_config', 'map_yaml', 'model']:
            self.assertTrue(resolve(ROOT, site[key]).is_file(), key)
        self.assertEqual(hashlib.sha256(resolve(ROOT, site['model']).read_bytes()).hexdigest(),
                         'dcd1544872485062ee3bd7619914442992dc4c9ee89db4f55c325791b5f28cb9')

    def test_package_and_node_names(self):
        for path in (ROOT/'ros2_ws/src').glob('*/package.xml'):
            self.assertTrue(ET.parse(path).getroot().findtext('name'))
        for path in (ROOT/'ros2_ws/src').rglob('*.py'):
            tree = ast.parse(path.read_text(encoding='utf-8-sig'), filename=str(path))
            for call in ast.walk(tree):
                if not isinstance(call, ast.Call):
                    continue
                if isinstance(call.func, ast.Name) and call.func.id == 'Node':
                    for item in call.keywords:
                        if item.arg == 'name' and isinstance(item.value, ast.Constant):
                            self.assertTrue(item.value.value.startswith('dog_'), str(path))


if __name__ == '__main__':
    for path in [ROOT/'dog', *(ROOT/'scripts').glob('*.sh')]:
        if os.name == 'posix' and shutil.which('bash'):
            subprocess.run(['bash', '-n', str(path)], check=True)
    unittest.main(verbosity=2)
