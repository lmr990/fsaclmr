"""Publishes a constant 0 m/s on /vehicle/velocity so ft_planner's speed
seeding has an input in the static-camera demo."""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32


class MockVelocity(Node):
    def __init__(self) -> None:
        super().__init__("mock_velocity")
        self.pub = self.create_publisher(Float32, "/vehicle/velocity", 10)
        self.create_timer(0.2, self._tick)
        self._v = 0.0

    def _tick(self) -> None:
        m = Float32()
        m.data = self._v
        self.pub.publish(m)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MockVelocity()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
