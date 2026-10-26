"""Launch the QUTMS x EUFS test and control GUI."""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="qutms_test_gui",
                executable="qutms_test_gui",
                name="qutms_test_gui",
                output="screen",
            )
        ]
    )
