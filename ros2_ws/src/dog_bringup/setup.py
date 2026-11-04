from glob import glob
from setuptools import setup

setup(name='dog_bringup', version='1.0.0', packages=['dog_bringup'],
      data_files=[('share/ament_index/resource_index/packages', ['resource/dog_bringup']),
                  ('share/dog_bringup', ['package.xml']),
                  ('share/dog_bringup/launch', glob('launch/*.launch.py'))],
      install_requires=['setuptools'], zip_safe=True,
      entry_points={'console_scripts': ['dog_status = dog_bringup.status:main']})
