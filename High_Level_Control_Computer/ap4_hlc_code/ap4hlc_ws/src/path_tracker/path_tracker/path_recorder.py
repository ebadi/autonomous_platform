import rclpy
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Path
import csv
import os
import math
from datetime import datetime


class PathTracker(Node):
    def __init__(self):
        super().__init__("path_tracker")

        # Initialize TF listener for robot's position
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Initialize planned path list
        self.planned_path = []
        self.actual_path = []
        self.closest_point_list = []

        # Create logs directory in current working directory
        logs_dir = os.path.expanduser("~/ap4_hlc_docker_dir/ap4hlc_ws/logs/paths_csv")
        os.makedirs(logs_dir, exist_ok=True)

        # Timestamped file name for saving paths
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.actual_file_path = os.path.join(logs_dir, f"actual_path_{timestamp}.csv")
        self.planned_file_path = os.path.join(logs_dir, f"planned_path_{timestamp}.csv")

        # Write headers for the CSV files
        self.write_csv_header(self.actual_file_path, ["timestamp", "x", "y", "z"])
        self.write_csv_header(self.planned_file_path, ["timestamp", "x", "y", "z"])

        # Subscribe to planned path topic
        self.plan_subscription = self.create_subscription(
            Path, "/plan", self.plan_callback, 10
        )

        # Timer to record robot pose every second
        self.timer = self.create_timer(1.0, self.record_pose)

        self.get_logger().info(f"Recording actual path to: {self.actual_file_path}")
        self.get_logger().info(f"Recording planned path to: {self.planned_file_path}")

    def write_csv_header(self, file_path, header):
        """Write the header to CSV file"""
        with open(file_path, mode="w", newline="") as file:
            writer = csv.writer(file)
            writer.writerow(header)

    def calculate_distance(self, p1, p2):
        """Calculate Euclidean distance between two 3D points"""
        return math.sqrt((p2[0] - p1[0]) ** 2 + (p2[1] - p1[1]) ** 2)

    def find_closest_point(self, robot_position):
        """Find the closest point from the planned path to the robot's position"""
        min_distance = float("inf")
        closest_point = None

        # Iterate over the planned path to find the closest point
        i = 0
        for point in self.planned_path:
            i = i + 1
            distance = self.calculate_distance(robot_position, point)
            if distance < min_distance:
                min_distance = distance
                closest_point = point
            if i >= 20:
                break
        return closest_point

    def record_pose(self):
        """Record the robot's current position and closest planned path point"""

        try:
            transform: TransformStamped = self.tf_buffer.lookup_transform(
                "map", "base_link", rclpy.time.Time()
            )

            pos = transform.transform.translation

            # Get timestamp
            timestamp = (
                self.get_clock().now().to_msg().sec
            )  # + self.get_clock().now().to_msg().nanosec * 1e-9

            # Save robot position to CSV
            self.actual_path.append([timestamp, pos.x, pos.y, pos.z])
            # with open(self.actual_file_path, mode='a', newline='') as file:
            #     writer = csv.writer(file)
            #     writer.writerow([timestamp, pos.x, pos.y, pos.z])

            # Find the closest point in the planned path
            closest_point = self.find_closest_point((pos.x, pos.y, pos.z))
            if closest_point:
                self.closest_point_list.append([timestamp, *closest_point])
            # Save the closest planned path point to CSV (only if we found a valid point)
            # if closest_point:
            #     with open(self.planned_file_path, mode='a', newline='') as file:
            #         writer = csv.writer(file)
            #         writer.writerow([timestamp, *closest_point])

        except Exception as e:
            self.get_logger().warn(f"Transform not available: {e}")

    def plan_callback(self, msg):

        self.planned_path = []
        self.get_logger().info("New planned path sent:")
        # Update the planned path list with the new poses
        for pose in msg.poses:
            coord = (
                round(pose.pose.position.x, 4),
                round(pose.pose.position.y, 4),
                round(pose.pose.position.z, 4),
            )
            self.planned_path.append(coord)


def main(args=None):
    rclpy.init(args=args)
    node = PathTracker()
    try:
        rclpy.spin(node)
    finally:
        with open(node.actual_file_path, mode="a", newline="") as file:
            writer = csv.writer(file)
            writer.writerows(node.actual_path)
        with open(node.planned_file_path, mode="a", newline="") as file:
            writer = csv.writer(file)
            writer.writerows(node.closest_point_list)

    rclpy.shutdown()


if __name__ == "__main__":
    main()
