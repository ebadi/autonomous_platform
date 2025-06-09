import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


class LidarNode(Node):
    def __init__(self):
        super().__init__("lidar_node")
        self.subscriber_ = self.create_subscription(
            LaserScan, "scan", self.callback_subscriber, 10
        )
        self.get_logger().info("Lidar node started...")

    def callback_subscriber(self, msg):
        # Iterate over the ranges array to find any value less than 3 meters
        for i, distance in enumerate(msg.ranges):
            if distance < 3.0 and distance > 0:  # Ignore values that are 0 (invalid)
                self.get_logger().info(
                    f"Object detected at {distance} meters (Index {i})"
                )


def main(args=None):
    rclpy.init(args=args)
    node = LidarNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
