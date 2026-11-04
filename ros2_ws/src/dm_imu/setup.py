from setuptools import setup

setup(
    name="dm_imu",
    version="0.2.0",
    packages=["dm_imu"],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/dm_imu"]),
        ("share/dm_imu", ["package.xml", "LICENSE", "README.md"]),
    ],
    install_requires=["setuptools", "pyserial"],
    zip_safe=True,
    maintainer="user",
    maintainer_email="user@example.com",
    description="DM-IMU-L1 passive USB driver adapted from DAMIAO's Humble example",
    license="MIT",
    entry_points={"console_scripts": [
        "dm_imu_node = dm_imu.node:main",
        "dm_imu_probe = dm_imu.probe:main",
    ]},
)
