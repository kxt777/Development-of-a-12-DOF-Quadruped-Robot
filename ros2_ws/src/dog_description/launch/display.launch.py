from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import os


def generate_launch_description():
    urdf_file = os.path.join(
        get_package_share_directory("dog_description"),
        "urdf",
        "dog.urdf"
    )
    rviz_config = os.path.join(
        get_package_share_directory("dog_description"),
        "config",
        "dog_display.rviz"
    )

    with open(urdf_file, "r", encoding="utf-8") as file:
        robot_description = file.read()

    return LaunchDescription([
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="dog_robot_state_publisher",
            parameters=[{"robot_description": robot_description}],
            output="screen"
        ),

        Node(
            package="rviz2",
            executable="rviz2",
            name="dog_rviz2",
            arguments=["-d", rviz_config],
            output="screen"
        ),
    ])
