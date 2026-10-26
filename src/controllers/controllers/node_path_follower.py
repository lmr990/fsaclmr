
import math

import numpy as np
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from transforms3d.euler import quat2euler

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from nav_msgs.msg import Path


class PathFollower(Node):
    def __init__(self):
        super().__init__("path_follower_node")

        self.declare_parameter("target_speed", 0.6)
        self.declare_parameter("lookahead_distance", 3.0)
        self.declare_parameter("steering_gain", 1.5)
        self.declare_parameter("map_frame", "track")
        self.declare_parameter("base_frame", "base_link")

        self.target_speed = self.get_parameter("target_speed").value
        self.lookahead_distance = self.get_parameter("lookahead_distance").value
        self.steering_gain = self.get_parameter("steering_gain").value
        self.map_frame = self.get_parameter("map_frame").value
        self.base_frame = self.get_parameter("base_frame").value

        self.path = None

        self.create_subscription(Path, "/planning/midline_path", self.path_callback, 1)
        self.cmd_vel_pub = self.create_publisher(Twist, "/control/nav_cmd_vel", 1)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.create_timer(1 / 20.0, self.control_loop)

        self.get_logger().info("---Path follower node initialised---")

    def path_callback(self, msg: Path):
        self.path = msg

    def control_loop(self):
        if self.path is None or len(self.path.poses) == 0:
            self.get_logger().warn("No path available", throttle_duration_sec=2)
            return

        try:
            map_to_base = self.tf_buffer.lookup_transform(
                self.map_frame, self.base_frame, rclpy.time.Time()
            )
        except TransformException as e:
            self.get_logger().warn("TF exception: " + str(e), throttle_duration_sec=2)
            return

        car_x = map_to_base.transform.translation.x
        car_y = map_to_base.transform.translation.y
        car_pos = np.array([car_x, car_y])

        quat = map_to_base.transform.rotation
        car_yaw = quat2euler([quat.w, quat.x, quat.y, quat.z])[2]

        points = np.array(
            [[p.pose.position.x, p.pose.position.y] for p in self.path.poses]
        )

        distances = np.linalg.norm(points - car_pos, axis=1)
        nearest_idx = int(np.argmin(distances))

        lookahead_idx = nearest_idx
        accumulated = 0.0
        for i in range(nearest_idx, len(points) - 1):
            seg = np.linalg.norm(points[i + 1] - points[i])
            accumulated += seg
            if accumulated >= self.lookahead_distance:
                lookahead_idx = i + 1
                break
        else:
            lookahead_idx = len(points) - 1

        target = points[lookahead_idx]

        dx = target[0] - car_x
        dy = target[1] - car_y
        target_angle = math.atan2(dy, dx)

        heading_error = target_angle - car_yaw
        heading_error = math.atan2(math.sin(heading_error), math.cos(heading_error))

        angular_z = self.steering_gain * heading_error

        twist = Twist()
        twist.linear.x = self.target_speed
        twist.angular.z = angular_z
        self.cmd_vel_pub.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    node = PathFollower()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()