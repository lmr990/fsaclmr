"""ROS2 node that assigns stable cone ids for the urinay planner."""

import rclpy
from rclpy.node import Node

from planners.cone_id_mapper import ConeIdMapper


class ConeIdMapperNode(Node, ConeIdMapper):
    def __init__(self):
        super().__init__("cone_id_mapper_node")
        self.declare_parameter("input_topic", "/mapping/cone_map")
        self.declare_parameter("output_topic", "/urinay/cone_map")
        self.declare_parameter("assoc_dist", 1.0)
        self.init_id_mapper(
            self,
            self.get_parameter("input_topic").value,
            self.get_parameter("output_topic").value,
            float(self.get_parameter("assoc_dist").value),
        )
        self.get_logger().info("---ConeIdMapper node initialised---")


def main(args=None):
    rclpy.init(args=args)
    node = ConeIdMapperNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
