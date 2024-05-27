import tempfile
import pdb
import numpy as np
import gymnasium as gym
from stable_baselines3.common.evaluation import evaluate_policy

import torch as th

from imitation.algorithms import bc
from imitation.algorithms.dagger import DAggerTrainer, SimpleDAggerTrainer
from imitation.algorithms.base import DemonstrationAlgorithm
from imitation.util.util import make_vec_env
from utils import load_human_expert_data
from utils import select_input
from utils import generate_unique_model_name
import os
from gymnasium import spaces
from stable_baselines3.common import policies, torch_layers, utils, vec_env
from stable_baselines3.common.env_checker import check_env
from stable_baselines3.common import env_util
from stable_baselines3 import PPO
from stable_baselines3.common.vec_env import DummyVecEnv
from stable_baselines3.common.monitor import Monitor
from stable_baselines3.common.policies import ActorCriticCnnPolicy, ActorCriticPolicy
from stable_baselines3.ppo import MlpPolicy, CnnPolicy
import argparse
import uuid
from imitation.policies.base import FeedForward32Policy
from imitation.algorithms.base import DemonstrationAlgorithm

import torch as th
import sys

import tqdm
from stable_baselines3.common import policies, torch_layers, utils, vec_env

from imitation.algorithms import base as algo_base
from imitation.data import rollout, types
from imitation.policies import base as policy_base
from imitation.util import logger as imit_logger
from imitation.util import util

import matplotlib.pyplot as plt

parser = argparse.ArgumentParser(
    description="Start autonomous drive and data collection with different inputs."
)

# Add an argument
parser.add_argument("--param", type=str, help="Parameter to modify in bash files.")

# Parse the argument
args = parser.parse_args()

# Example parameter to modify in bash files
param_value = args.param if args.param else "color"

image = True
depth = False
orbs = False
imu = True
simulation = False

if param_value == "depth":
    print("The training will be done with depth camera\n")
    depth = True
elif param_value == "orb":
    print("The training will be done with ORB-features\n")
    orbs = True
elif param_value == "sim":
    simulation = True
else:
    print("The training is set to default using color camera and IMU\n")


print("Python version:", sys.version)
print("PyTorch version:", th.__version__)
print("Make sure that the version of Pytorch is the same as in the HLC Docker!\n")

save_real_gokart_model = True
save_sim_model = False

model_dir = "/root/ap4_hlc_docker_dir/ap4hlc_ws/src/imitation_learning/models"
dir = "/root/ap4_hlc_docker_dir"
expert_data_filepath = dir + "/recorded_data/training/default"
validataion_data_filepath = dir + "/recorded_data/validation"
donkey_test_dir = (
    dir
    + "/src/imitation_learning/simulation_donkeycar/data"
)

base_dir = dir + "/recorded_data/training"
orb_dir = os.path.join(base_dir, "orb")
default_dir = os.path.join(base_dir, "default")
depth_dir = os.path.join(base_dir, "depth")
color_dir = os.path.join(base_dir, "color")

if not os.path.exists(model_dir):
    os.makedirs(model_dir)
# Check if directories exist
for directory in [
    validataion_data_filepath,
    base_dir,
    orb_dir,
    default_dir,
    depth_dir,
    color_dir,
]:
    if not os.path.exists(directory):
        print(f"Directory not found {directory}, add recorded data before training.")

        

rng = np.random.default_rng(0)
image_size = np.array([160, 120])

model_name_sim = "model.pt"
if image and depth and imu:
    model_name_gokart = "model_depth.pt"
    expert_data_filepath_dagger = depth_dir
elif image and orbs and imu:
    model_name_gokart = "model_orb.pt"
    expert_data_filepath_dagger = orb_dir
else:
    model_name_gokart = "model_color.pt"
    expert_data_filepath_dagger = color_dir

expert_data_list = [expert_data_filepath, expert_data_filepath_dagger]

validataion_demonstrations, _ = load_human_expert_data(
    validataion_data_filepath,
    image_size,
    color_true=image,
    depth_true=depth,
    imu_true=imu,
    orbs_true=orbs,
)

demonstrations, observation_space, action_space = select_input(
    expert_data=expert_data_list,
    image=image,
    depth=depth,
    orbs=orbs,
    imu=imu,
    simulation=simulation,
)
# saved model name


kwargs = {"lr": 0.001}
bc_trainer = bc.BC(
    observation_space=observation_space,
    action_space=action_space,
    rng=rng,
    demonstrations=demonstrations,
    batch_size=64,
    device="cpu",
    l2_weight=1e-3,
    ent_weight=1e-3,
    optimizer_kwargs=kwargs,
)
print("BC created")
loss_calculator = bc.BehaviorCloningLossCalculator(ent_weight=1e-3, l2_weight=1e-3)

l2_loss = []
loss = []
ent_loss = []
prob_true_acts = []
print("starting training")
for epoch in range(1, 2):
    print("starting epoch ", epoch)
    bc_trainer.train(n_epochs=1)
    print("epoch " + str(epoch) + " finished")

    training_metrics = loss_calculator(
        bc_trainer.policy,
        validataion_demonstrations.obs,
        validataion_demonstrations.acts,
    )
    print(
        "------------------------------------------------\n"
        "validation loss\n"
        "------------------------------------------------"
    )
    print("l2 loss: ", training_metrics.l2_loss.item())
    l2_loss.append(training_metrics.l2_loss.item())

    print("loss: ", training_metrics.loss.item())
    loss.append(training_metrics.loss.item())

    print("entropy loss: ", training_metrics.ent_loss.item())
    ent_loss.append(training_metrics.ent_loss.item())

    print("probability true act: ", training_metrics.prob_true_act.item())
    prob_true_acts.append(training_metrics.prob_true_act.item())

    print("neglogp: ", training_metrics.neglogp.item())

    if epoch == 1:
        best_model = bc_trainer
    elif prob_true_acts[epoch - 1] >= max(prob_true_acts):
        best_model = bc_trainer
        print("NEW BEST MODEL")

    print("------------------------------------------------")
print("training finished")
# Plotting the losses in separate subfigures
plt.figure(figsize=(15, 5))

# Plot L2 Loss
plt.subplot(2, 2, 1)
plt.plot(range(1, len(l2_loss) + 1), l2_loss, label="L2 Loss")
plt.xlabel("Epochs")
plt.ylabel("L2 Loss")
plt.title("L2 Loss over Epochs")

# Plot Total Loss
plt.subplot(2, 2, 2)
plt.plot(range(1, len(loss) + 1), loss, label="Total Loss")
plt.xlabel("Epochs")
plt.ylabel("Total Loss")
plt.title("Total Loss over Epochs")

# Plot Entropy Loss
plt.subplot(2, 2, 3)
plt.plot(range(1, len(ent_loss) + 1), ent_loss, label="Entropy Loss")
plt.xlabel("Epochs")
plt.ylabel("Entropy Loss")
plt.title("Entropy Loss over Epochs")

plt.subplot(2, 2, 4)
plt.plot(range(1, len(prob_true_acts) + 1), prob_true_acts, label="True act")
plt.xlabel("Epochs")
plt.ylabel("Probability of correct action")
plt.title("Correct prediction over Epochs")

plt.tight_layout()
# plt.show()

for demonstration in validataion_demonstrations[1100:1200]:
    pred_action = best_model.policy.predict(demonstration["obs"]._d)
    print("actual actions:               " + str(demonstration["acts"]))
    print("BC predicted actions:         " + str(pred_action[0]) + "\n")


BC_path_sim = (
    dir
    + "/ap4hlc_ws/src/imitation_learning/simulation_donkeycar"
)
BC_path_gokart = (model_dir
)

if save_sim_model:
    th.save(best_model.policy, f"{BC_path_sim}/{model_name_sim}")
    print(f"Model for simulation has been saved to: {BC_path_sim}/{model_name_sim}")
else:
    print(
        "Model for simulation has not been saved. Set save_sim_model=True if you wish to save the model for simulation."
    )

if save_real_gokart_model:
    unique_model_name = generate_unique_model_name(BC_path_gokart, model_name_gokart)
    th.save(best_model.policy, os.path.join(BC_path_gokart, unique_model_name))
    print(f"Model for real world testing has been saved to: {BC_path_gokart}/{unique_model_name}")
else:
    print(
        "Model for real world testing has not been saved. Set save_real_gokart_model=True if you wish to save the model for real world testing."
    )
 	