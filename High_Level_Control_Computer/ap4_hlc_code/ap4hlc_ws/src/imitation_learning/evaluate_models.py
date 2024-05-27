import torch as th
from utils import load_human_expert_data
import numpy as np
from imitation.algorithms import bc
import os
from tabulate import tabulate

dir = "/root/ap4_hlc_docker_dir/ap4hlc_ws"
image_size = np.array([160, 120])

validation_data_filepath = dir + "/recorded_data/validation"
models_filepath = dir + "/src/imitation_learning/models"

loss_calculator = bc.BehaviorCloningLossCalculator(ent_weight=1e-3, l2_weight=1e-3)

model_directories = {"Depth": "/Depth", "Color": "/Color", "Orbs": "/Orbs"}

results = []

for model_type, model_directory in model_directories.items():
    model_path = models_filepath + model_directory
    validation_demonstrations = None

    if model_type == "Depth":
        image = True
        depth = True
        orbs = False
        imu = True
    elif model_type == "Orbs":
        image = True
        depth = False
        orbs = True
        imu = True
    elif model_type == "Color":
        image = True
        depth = False
        orbs = False
        imu = True
    else:
        # Skip this model type if conditions don't match
        continue

    validation_demonstrations, _ = load_human_expert_data(
        validation_data_filepath,
        image_size,
        color_true=image,
        depth_true=depth,
        imu_true=imu,
        orbs_true=orbs,
    )
    model_lengths = []

    for model_file in os.listdir(model_path):
        policy = th.load(os.path.join(model_path, model_file))

        if validation_demonstrations is not None:
            training_metrics = loss_calculator(
                policy, validation_demonstrations.obs, validation_demonstrations.acts
            )
            results.append(
                {
                    "Model Type": model_type,
                    "Model": model_file,
                    "L2 Loss": training_metrics.l2_loss.item(),
                    "Loss": training_metrics.loss.item(),
                    "Entropy Loss": training_metrics.ent_loss.item(),
                    "Probability True Act": training_metrics.prob_true_act.item(),
                    "neglogp": training_metrics.neglogp.item(),
                }
            )
            print("model added")
            model_lengths.append(len(model_file))

    max_model_length = (
        max(model_lengths) if model_lengths else 10
    )  # Set a default value if there are no models for this type

    # Add a separator line after each model type
    results.append(
        {
            "Model Type": "-" * 11,
            "Model": "-" * max_model_length,
            "L2 Loss": "-" * 21,
            "Loss": "-" * 21,
            "Entropy Loss": "-" * 21,
            "Probability True Act": "-" * 21,
            "neglogp": "-" * 21,
        }
    )

if results:
    results.pop()

print(tabulate(results, headers="keys", tablefmt="pretty"))
