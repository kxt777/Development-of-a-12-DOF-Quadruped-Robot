from setuptools import setup
from glob import glob
setup(name='dog_ik_walk',version='0.3.1',packages=['dog_ik_walk'],
      data_files=[('share/ament_index/resource_index/packages',['resource/dog_ik_walk']),
                  ('share/dog_ik_walk',['package.xml']),
                  ('share/dog_ik_walk/launch',glob('launch/*.launch.py'))],
      install_requires=['setuptools'],zip_safe=True,
      package_data={'dog_ik_walk':['web/*']},
      entry_points={'console_scripts':['dog_ik_walk_node = dog_ik_walk.node:main','dog_web_bridge = dog_ik_walk.web_server:main']})
