import os
import numpy as np
import gymnasium as gym
import pandas as pd
import cv2
import numpy as np
from imitation.data.types import DictObs, Transitions
import os
import json
import re
import message_filters
import torch as th

def find_highest_numbered_model(base_path, base_pattern):
    """
    Finds the highest numbered model in a directory based on a naming pattern.
    For example, if base_pattern is 'model_color', it will look for 'model_color_1.pt',
    'model_color_2.pt', ..., and returns the path to the model with the highest number.
    """
    max_number = -1
    highest_numbered_model = None
    pattern = re.compile(re.escape(base_pattern) + r"_([0-9]+)\.pt$")

    for filename in os.listdir(base_path):
        match = pattern.match(filename)
        if match:
            number = int(match.group(1))
            if number > max_number:
                max_number = number
                highest_numbered_model = os.path.join(base_path, filename)

    return highest_numbered_model

def synchronize_messages(self):
    if self.param_value == "depth":
        ts = message_filters.ApproximateTimeSynchronizer(
            [self.subscriber_color_image, self.subscriber_imu, self.subscriber_depth],
            30,
            0.05,
        )
    else:
        ts = message_filters.ApproximateTimeSynchronizer(
            [self.subscriber_color_image, self.subscriber_imu], 30, 0.05
        )
    return ts

def load_policy(self):
    model_path = "/root/ap4_hlc_docker_dir/ap4hlc_ws/src/imitation_learning/models/"
    if self.param_value == "depth":
        model_name = "model_depth"
    elif self.param_value == "orb":
        model_name = "model_orb"
    else:
        model_name = "model_color"
    model_name = find_highest_numbered_model(model_path,model_name)
    print(f"Loaded model is: {model_name}")
    return th.load(model_name)

def get_obs(self, image, imu, depth):
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

    imu = np.concatenate((angular_velocity, linear_acceleration))

    image_data = np.frombuffer(image.data, dtype=np.uint8)
    image_shape = (image.height, image.width, 3)
    image_data = image_data.reshape(image_shape)
    image_data = cv2.resize(image_data, self.image_size)

    if self.param_value == "depth":
        depth_data = np.frombuffer(depth.data, dtype=np.uint16)
        depth_shape = (depth.height, depth.width, 1)
        depth_data = depth_data.reshape(depth_shape)
        depth_data = cv2.resize(depth_data, self.image_size)
        depth_data = cv2.normalize(depth_data, None, 0, 255, cv2.NORM_MINMAX)
        depth_data = cv2.applyColorMap(depth_data.astype(np.uint8), self.colormap)

        observation = {
            "color_image": np.array(image_data),
            "depth_image": np.array(depth_data),
            "imu": np.array(imu),
        }
    elif self.param_value == "orb":
        kp = self.orb.detect(image_data, None)

        # Create a binary matrix to represent keypoints
        orbs = np.zeros(
            tuple(np.array([self.image_size[1], self.image_size[0]])), dtype=np.uint8
        )

        # Mark keypoints with 1s in the matrix
        for point in kp:
            x, y = map(int, point.pt)
            orbs[y, x] = (
                1  # Note the y, x order because of how numpy arrays are indexed
            )

        observation = {
            "color_image": np.array(image_data),
            "orbs": orbs,
            "imu": np.array(imu),
        }
    else:
        observation = {"color_image": np.array(image_data), "imu": np.array(imu)}
    return observation

    
    
