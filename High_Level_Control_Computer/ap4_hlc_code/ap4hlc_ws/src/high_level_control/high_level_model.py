#!/usr/bin/env python3

import torch as th
import numpy as np
import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from geometry_msgs.msg import TwistStamped  # /cmd_vel topic
from std_msgs.msg import UInt16
from std_msgs.msg import Int8
from sensor_msgs.msg import Image
import message_filters
import pandas as pd
import numpy as np
import time
import cv2
import os
import pickle
import signal  # Import the signal module
import sys
from sensor_msgs.msg import Imu
import argparse
from utils import *


class MinimalPublisher(Node):

    def __init__(self):
        print("init")
        super().__init__("high_level_control")

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
        self.param_value = self.args.param if self.args.param else "color"

        self.image_size = (160, 120)

        self.orb = cv2.ORB_create(
            nfeatures=1000, scaleFactor=1.2, nlevels=8, edgeThreshold=15
        )
        self.colormap = cv2.COLORMAP_JET
        #Subscriptions
        self.subscriber_color_image = message_filters.Subscriber(
            self, Image, "/color/image"
        )
        self.subscriber_imu = message_filters.Subscriber(self, Imu, "/imu")

        self.subscriber_depth = message_filters.Subscriber(self, Image, "/stereo/depth")
            
        #syncronization
        self.ts = synchronize_messages(self)



        self.ts.registerCallback(self.predict_actions)
        self.publisher_camera_cmd_vel = self.create_publisher(
            Twist, "/camera_cmd_vel", 10
        )

        self.policy = load_policy(self)

        
        self.twist_msg = Twist()
        self.twist_msg.linear.x = 0.0
        self.twist_msg.linear.y = 0.0
        self.twist_msg.linear.z = 0.0
        self.twist_msg.angular.x = 0.0
        self.twist_msg.angular.y = 0.0
        self.twist_msg.angular.z = 0.0

    def predict_actions(self, image, imu, depth=None):
        # print('entered prediction')
        print("predict actions")
        observation = get_obs(self, image, imu, depth)
        print("afdter obs")

        action, _states = self.policy.predict(observation, deterministic=False)
        self.twist_msg.linear.x = 0.5 * float(action[-1])
        self.twist_msg.linear.y = 0.0
        self.twist_msg.linear.z = 0.0
        self.twist_msg.angular.x = 0.0
        self.twist_msg.angular.y = 0.0
        self.twist_msg.angular.z = 0.5 * float(action[0])
        print("innan publish")
        self.publisher_camera_cmd_vel.publish(self.twist_msg)

 
def main(args=None):
    rclpy.init(args=args)
    minimal_publisher = MinimalPublisher()
    rclpy.spin(minimal_publisher)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
