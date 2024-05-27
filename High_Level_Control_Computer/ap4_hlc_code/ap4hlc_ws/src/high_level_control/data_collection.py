#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from geometry_msgs.msg import TwistStamped  # /cmd_vel topic
from std_msgs.msg import UInt16
from std_msgs.msg import Int8
from sensor_msgs.msg import Image
from sensor_msgs.msg import Imu
import message_filters
import pandas as pd
import numpy as np
import time
import cv2
import os
import pickle
import signal  # Import the signal module
import sys
import gc
import argparse


class MinimalPublisher(Node):

    def __init__(self):
        print("init")
        super().__init__("data_collection")
        # Listen to /cmd_vel topic, once cmd_vel message recieved, perform controller action
        # self.subscriber_cmd_vel_ = self.create_subscription(Twist, '/cmd_vel', self.Callback_cmd_vel, 10)
        # self.subscriber_color_image_ = self.create_subscription(Image, '/color/image', self.Callback_color_image, 10)

        self.parser = argparse.ArgumentParser(
            description="Start autonomous drive and data collection with different inputs."
        )

        # Add an argument
        self.parser.add_argument(
            "--param", type=str, help="Parameter to modify in bash files."
        )

        # Parse the argument
        self.args = self.parser.parse_args()

        # Example parameter to modify in bash files
        self.param_value = self.args.param if self.args.param else "default"

        self.subscriber_cmd_vel = message_filters.Subscriber(
            self, TwistStamped, "/cmd_vel_stamped"
        )
        self.subscriber_color_image = message_filters.Subscriber(
            self, Image, "/color/image"
        )
        self.subscriber_depth = message_filters.Subscriber(self, Image, "/stereo/depth")
        self.subscriber_imu = message_filters.Subscriber(self, Imu, "/imu")
        self.ts = message_filters.ApproximateTimeSynchronizer(
            [
                self.subscriber_color_image,
                self.subscriber_cmd_vel,
                self.subscriber_depth,
                self.subscriber_imu,
            ],
            30,
            0.05,
        )
        # self.ts = message_filters.TimeSynchronizer([self.subscriber_color_image], 10)
        self.ts.registerCallback(self.callback)

        self.image_size = (208, 156)
        self.output_directory = "image_output"
        self.columns = ["Timestamp", "Image", "Steering_angle", "Speed"]
        self.image_df = pd.DataFrame(columns=self.columns)

        self.timestamps = []
        self.image_paths = []
        self.depth = []

        self.speed = []
        self.steering_angle = []
        self.time = []
        self.imu_measurments = []
        self.start_time = time.time()
        self.memory_counter = 0

        signal.signal(signal.SIGINT, self.signal_handler)

    def callback(self, image, cmd_vel, depth, imu):
        # print('entered callback')
        timestamp = image.header.stamp.sec + image.header.stamp.nanosec * 1e-9
        self.timestamps.append(timestamp)

        # Handling color image
        image_data = np.frombuffer(image.data, dtype=np.uint8)
        image_shape = (image.height, image.width, 3)
        image_data = image_data.reshape(image_shape)
        image_data = cv2.resize(image_data, self.image_size)
        self.image_paths.append(image_data)

        # Handling depth image
        depth_data = np.frombuffer(
            depth.data, dtype=np.uint16
        )  # Assuming depth data is 16-bit unsigned integer
        depth_shape = (depth.height, depth.width)
        depth_data = depth_data.reshape(depth_shape)
        self.depth.append(cv2.resize(depth_data, self.image_size))

        # cv2.imwrite(image_path, image)
        self.speed.append(cmd_vel.twist.linear.x)
        self.steering_angle.append(cmd_vel.twist.angular.z)

        angular_velocity = np.array(
            [imu.angular_velocity.x, imu.angular_velocity.y, imu.angular_velocity.z]
        )
        linear_acceleration = np.array(
            [
                imu.linear_acceleration.x,
                imu.linear_acceleration.y,
                imu.linear_acceleration.z,
            ]
        )

        self.imu_measurments.append(
            np.concatenate((angular_velocity, linear_acceleration))
        )

        print(len(self.image_paths))

        if len(self.image_paths) >= 3000:
            self.free_memory()
        # if (time.time() - self.start_time) > 60*1:
        #     self.free_memory()
        #     self.start_time = time.time()

    def free_memory(self):
        self.memory_counter += 1
        print("Saving to clear memory part " + str(self.memory_counter))
        self.save_to_csv(self.memory_counter)

    def signal_handler(self, signal, frame):
        print("\nReceived Ctrl+C. Printing DataFrame:")
        if self.memory_counter > 0:
            self.memory_counter += 1
            self.save_to_csv(self.memory_counter)
            sys.exit(0)
        else:
            self.save_to_csv()
            sys.exit(0)

    def save_to_csv(self, counter=""):

        if counter != "":
            counter = "part_" + str(counter)

        df = pd.DataFrame(
            {
                "Timestamp": self.timestamps,
                "Image": self.image_paths,
                "Steering_angle": self.steering_angle,
                "Speed": self.speed,
                "Depth": self.depth,
                "IMU": self.imu_measurments,
            }
        )
        del (
            self.timestamps,
            self.image_paths,
            self.depth,
            self.steering_angle,
            self.speed,
            self.imu_measurments,
        )
        self.timestamps = []
        self.image_paths = []
        self.depth = []
        self.speed = []
        self.steering_angle = []
        self.time = []
        self.imu_measurments = []
        print("Saving DF")
        # print(df)
        data_name = time.time()

        if self.param_value == "validation":
            dir_path = os.path.join(base_dir, "validation")
        else:
            dir_path = os.path.join(base_dir, "training", self.param_value)

        # Construct the full file path
        file_path = os.path.join(
            dir_path, "image_data_" + str(counter) + "_" + str(data_name) + ".pkl"
        )

        # Save the DataFrame to a pickle file
        df.to_pickle(file_path, protocol=4)
        
        del df
        gc.collect()
        print("saving took: " + str(time.time() - data_name))
        print(
            "DF was saved to: "
            + file_path
        )


def main(args=None):
    rclpy.init(args=args)
    orb_dir = os.path.join(base_dir, "training/orb")
    default_dir = os.path.join(base_dir, "training/default")
    depth_dir = os.path.join(base_dir, "training/depth")
    color_dir = os.path.join(base_dir, "training/color")
    validation_dir = os.path.join(base_dir, "validation")

    rosbag_orb_dir = default_dir = os.path.join(base_dir + "/training/orb", "rosbag")
    rosbag_default_dir = default_dir = os.path.join(base_dir + "/training/default", "rosbag")
    rosbag_depth_dir = default_dir = os.path.join(base_dir + "/training/depth", "rosbag")
    rosbag_color_dir = default_dir = os.path.join(base_dir + "/training/color", "rosbag")
    rosbag_validation_dir = default_dir = os.path.join(base_dir + "/validation", "rosbag")

    # Check if directories exist, create them if they don't
    for directory in [
        base_dir,
        orb_dir,
        default_dir,
        depth_dir,
        color_dir,
        validation_dir,
        rosbag_orb_dir,
        rosbag_default_dir,
        rosbag_depth_dir,
        rosbag_color_dir,
        rosbag_validation_dir,
    ]:
        if not os.path.exists(directory):
            os.makedirs(directory)
    minimal_publisher = MinimalPublisher()
    rclpy.spin(minimal_publisher)
    minimal_publisher.destroy_node()
    rclpy.shutdown()

base_dir = "/root/ap4_hlc_docker_dir/recorded_data/"

if __name__ == "__main__":
    main()
