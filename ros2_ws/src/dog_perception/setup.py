from setuptools import setup

setup(name='dog_perception', version='1.0.0', packages=['dog_perception'],
      data_files=[('share/ament_index/resource_index/packages', ['resource/dog_perception']),
                  ('share/dog_perception', ['package.xml'])],
      install_requires=['setuptools'], zip_safe=True,
      entry_points={'console_scripts': ['dog_yolo = dog_perception.viewer:main']})
