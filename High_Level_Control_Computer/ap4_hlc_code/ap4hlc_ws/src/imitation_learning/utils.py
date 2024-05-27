import os
import numpy as np
import gymnasium as gym
import pandas as pd
import cv2
import numpy as np
from imitation.data.types import DictObs, Transitions
import os
import json

def select_input(
    expert_data,
    image=True,
    depth=False,
    orbs=False,
    imu=False,
    simulation=False,
    image_size=np.array([160, 120]),
):
    demonstrations, _ = load_human_expert_data(
        expert_data,
        image_size,
        simulation=simulation,
        color_true=image,
        depth_true=depth,
        orbs_true=orbs,
        imu_true=imu,
    )

    observation_space_dict = {
        "color_image": (
            gym.spaces.Box(low=0, high=255, shape=(3, 120, 160), dtype=np.uint8)
            if image
            else None
        ),
        "depth_image": (
            gym.spaces.Box(low=0, high=255, shape=(3, 120, 160), dtype=np.uint8)
            if depth
            else None
        ),
        "orbs": (
            gym.spaces.Box(low=0, high=1, shape=(120, 160), dtype=np.uint8)
            if orbs
            else None
        ),
        "imu": (
            gym.spaces.Box(low=-30, high=30, shape=(6,), dtype=np.float32)
            if imu
            else None
        ),
    }

    # Filter out None values (in case any condition evaluated to False)
    print(observation_space_dict)
    observation_space_dict = {
        key: value for key, value in observation_space_dict.items() if value is not None
    }
    print(observation_space_dict)
    # Create observation space
    observation_space = gym.spaces.Dict(observation_space_dict)
    action_space = gym.spaces.Box(low=-1, high=1, shape=(2,), dtype=np.float32)

    return demonstrations, observation_space, action_space

def load_human_expert_data(
    csv_file_path,
    size,
    simulation=False,
    color_true=True,
    depth_true=False,
    orbs_true=False,
    imu_true=False,
):
    colormap = cv2.COLORMAP_JET
    if simulation:

        expert_data = []
        transitions = []
        inverse_transitions = []
        for filename in os.listdir(csv_file_path):
            # Check if the file has the .catalog extension
            if filename.endswith(".catalog"):
                expert_data.append(filename)

        for data in expert_data:
            with open(csv_file_path + data, "r") as file:
                for line in file:
                    # Parse the JSON object
                    data = json.loads(line)

                    # Access the fields within the JSON object
                    _index = data["_index"]
                    _session_id = data["_session_id"]
                    timestamp = data["_timestamp_ms"]
                    image_path = data["cam/image_array"]
                    steering_angle = data["user/angle"]
                    user_mode = data["user/mode"]
                    speed = data["user/throttle"]

                    image = cv2.imread(csv_file_path + "images/" + image_path)
                    image = cv2.resize(image, size)
                    inverse_image = cv2.flip(image, 1)
                    image = np.moveaxis(image, -1, 0)

                    steering_angle = float(steering_angle)
                    speed = 2 * float(speed)
                    inverse_image = np.moveaxis(inverse_image, -1, 0)
                    inverse_steering_angle = -steering_angle

                    # Create the dictionary

                    # image = np.moveaxis(image, -1, 0)
                    transition = {
                        "obs": image,
                        "acts": np.array([steering_angle, speed]),
                        "infos": None,  # You can add info if needed
                        "dones": False,
                    }

                    inverse_transition = {
                        "obs": inverse_image,
                        "acts": np.array([inverse_steering_angle, speed]),
                        "infos": None,  # You can add info if needed
                        "dones": False,
                    }

                    transitions.append(transition)
                    inverse_transitions.append(inverse_transition)

        transitions.extend(inverse_transitions)

        for i in range(len(transitions) - 1):
            transitions[i]["next_obs"] = transitions[i + 1]["obs"]

        # For the last transition, set 'next_obs' to the last 'obs' for consistency
        transitions[-1]["next_obs"] = transitions[-1]["obs"]

        # Convert the list of transition dictionaries to TransitionsMinimal

        observation_dict = DictObs
        obs_list = np.array([t["obs"] for t in transitions])
        next_obs_list = np.array([t["next_obs"] for t in transitions])
        acts_array = np.array([t["acts"] for t in transitions])
        infos_array = np.array([t["infos"] for t in transitions])
        dones_array = np.array([t["dones"] for t in transitions])
        dones_array[-1] = True

        # observations = observation_dict.from_obs_list(obs_list=obs_list)
        # next_observations = observation_dict.from_obs_list(obs_list=next_obs_list)

        # print(observation_dict)
        return (
            Transitions(
                obs=obs_list,
                acts=acts_array,
                infos=infos_array,
                next_obs=next_obs_list,
                dones=dones_array,
            ),
            obs_list,
        )

    # Extract values from the catalog data

    else:
        orb = cv2.ORB_create(
            nfeatures=1000, scaleFactor=1.2, nlevels=8, edgeThreshold=15
        )
        expert_data = []
        transitions = []
        inverse_transitions = []
        if isinstance(csv_file_path, list):
            for filepath in csv_file_path:
                for filename in os.listdir(filepath):
                    # Check if the file has the .catalog extension
                    if filename.endswith(".pkl"):
                        expert_data.append(filepath + "/" + filename)

        else:
            for filename in os.listdir(csv_file_path):
                # Check if the file has the .catalog extension
                if filename.endswith(".pkl"):
                    expert_data.append(csv_file_path + "/" + filename)

        for data in expert_data:
            expert_df = pd.read_pickle(data)
            for index, row in expert_df.iterrows():
                timestamp = row["Timestamp"]
                done = False
                if index < len(expert_df) - 1:
                    timestamp_next = expert_df["Timestamp"][index + 1]
                    if timestamp_next - timestamp > 1.0:
                        done = True
                else:
                    done = True

                image = row["Image"]
                speed = row["Speed"]
                steering_angle = row["Steering_angle"]
                depth_image = row["Depth"]
                imu = row["IMU"]

                imu = np.clip(imu, -30, 30)

                image_raw = cv2.resize(image, size)
                image = np.moveaxis(image_raw, -1, 0)

                kp = orb.detect(image_raw, None)

                # Create a binary matrix to represent keypoints
                orbs = np.zeros(tuple(np.array([size[1], size[0]])), dtype=np.uint8)

                # Mark keypoints with 1s in the matrix
                for point in kp:
                    x, y = map(int, point.pt)
                    orbs[y, x] = (
                        1  # Note the y, x order because of how numpy arrays are indexed
                    )

                depth_image = cv2.resize(depth_image, size)
                depth_image = cv2.normalize(depth_image, None, 0, 255, cv2.NORM_MINMAX)
                depth_image = cv2.applyColorMap(depth_image.astype(np.uint8), colormap)
                depth_image = np.moveaxis(depth_image, -1, 0)

                steering_angle = 2 * float(steering_angle)
                speed = 2 * float(speed)
                inverse_steering_angle = -steering_angle

                # Example ORB keypoints (empty list)

                # Input dictionary
                observation = {
                    "color_image": np.array(image) if color_true else None,
                    "depth_image": np.array(depth_image) if depth_true else None,
                    "orbs": orbs if orbs_true else None,
                    "imu": np.array(imu) if imu_true else None,
                }

                observation = {
                    key: value
                    for key, value in observation.items()
                    if value is not None
                }

                # image = np.moveaxis(image, -1, 0)
                transition = {
                    "obs": observation,
                    "acts": np.array([steering_angle, speed]),
                    "infos": None,  # You can add info if needed
                    "dones": done,
                }

                transitions.append(transition)

        # transitions.extend(inverse_transitions)

        for i in range(len(transitions)):
            if transitions[i]["dones"]:
                transitions[i]["next_obs"] = transitions[i]["obs"]
            else:
                transitions[i]["next_obs"] = transitions[i + 1]["obs"]

        # Convert the list of transition dictionaries to TransitionsMinimal

        obs_list = [t["obs"] for t in transitions]
        obs = DictObs.from_obs_list(obs_list)

        next_obs_list = [t["next_obs"] for t in transitions]
        next_obs = DictObs.from_obs_list(next_obs_list)
        acts_array = np.array([t["acts"] for t in transitions])
        infos_array = np.array([t["infos"] for t in transitions])
        dones_array = np.array([t["dones"] for t in transitions])
        dones_array[-1] = True

        # observations = observation_dict.from_obs_list(obs_list=obs_list)
        # next_observations = observation_dict.from_obs_list(obs_list=next_obs_list)

        # print(observation_dict)
        return (
            Transitions(
                obs=obs,
                acts=acts_array,
                infos=infos_array,
                next_obs=next_obs,
                dones=dones_array,
            ),
            obs_list,
        )
    
def generate_unique_model_name(base_path, base_name):
    """
    Generates a unique model name to avoid overwriting existing models.
    If model_color.pt exists, it will try model_color_1.pt, model_color_2.pt, etc.
    """
    counter = 1
    model_name = base_name
    while os.path.exists(os.path.join(base_path, model_name)):
        model_name = f"{base_name.rsplit('.', 1)[0]}_{counter}.pt"
        counter += 1
    return model_name