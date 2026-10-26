"""Launch only the QUTMS Twist-to-Ackermann controller for isolated testing."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            Node(
                package="controllers",
                executable="vel_to_ackermann_node",
                name="vel_to_ackermann_node",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": use_sim_time,
                        # This isolated controller test must not require a path or TF tree.
                        "distance_ctrl": False,
                        # Match eufs_config/configDry.yaml wheelbase.
                        "wheelbase": 1.525,
                        # Conservative initial conversion gain for low-speed tests.
                        "Kp": 1.0,
                    }
                ],
            ),
        ]
    )
