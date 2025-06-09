import matplotlib.pyplot as plt
import pandas as pd
import numpy as np
import math


def load_csv(file_path):
    """Load the CSV file with timestamp, x, y, z data into a pandas DataFrame."""
    return pd.read_csv(file_path)


def calculate_error(actual_path, planned_path):
    """Calculate the Euclidean error (distance) between the actual and planned paths."""
    errors = []
    for i in range(len(actual_path)):
        distance = math.sqrt(
            (actual_path.iloc[i]["x"] - planned_path.iloc[i]["x"]) ** 2
            + (actual_path.iloc[i]["y"] - planned_path.iloc[i]["y"]) ** 2
        )
        errors.append(distance)
    return errors


def rotate_path(df, angle_degrees, center=None):
    """Rotate the (x, y) coordinates around a given center point."""
    angle_rad = math.radians(angle_degrees)
    cos_theta = math.cos(angle_rad)
    sin_theta = math.sin(angle_rad)

    # Use provided center or compute center of the dataframe
    if center is None:
        center_x = df["x"].mean()
        center_y = df["y"].mean()
    else:
        center_x, center_y = center

    # Shift to origin
    x_shifted = df["x"] - center_x
    y_shifted = df["y"] - center_y

    # Apply rotation
    x_rotated = x_shifted * cos_theta - y_shifted * sin_theta
    y_rotated = x_shifted * sin_theta + y_shifted * cos_theta

    # Shift back
    rotated_df = df.copy()
    rotated_df["x"] = x_rotated + center_x
    rotated_df["y"] = y_rotated + center_y

    return rotated_df


def compute_moving_average(errors, window_size=10):
    """Compute the average error every `window_size` elements."""
    moving_averages = []
    indices = []
    for i in range(0, len(errors)):
        window = errors[i : i + window_size]
        if len(window) > 0:
            avg = sum(window) / len(window)
            moving_averages.append(avg)
            indices.append(i + len(window) // 2)  # center point of window
            # print(f"Average error for points {i} to {i+len(window)-1}: {avg:.4f}")
    return indices, moving_averages


def interpolate_planned_to_actual_timestamps(actual_path, planned_path):
    # Interpolate x and y of planned_path to match timestamps in actual_path
    interpolated_x = np.interp(
        actual_path["timestamp"], planned_path["timestamp"], planned_path["x"]
    )
    interpolated_y = np.interp(
        actual_path["timestamp"], planned_path["timestamp"], planned_path["y"]
    )

    planned_interpolated = pd.DataFrame(
        {
            "timestamp": actual_path["timestamp"],
            "x": interpolated_x,
            "y": interpolated_y,
        }
    )

    return planned_interpolated


def plot_paths_and_error(actual_path_file, planned_path_file, rotation_angle=170):
    # Load actual and planned path CSVs
    actual_path = load_csv(actual_path_file)
    planned_path = load_csv(planned_path_file)

    # Interpolate planned path to match actual path timestamps
    planned_path_interp = interpolate_planned_to_actual_timestamps(
        actual_path, planned_path
    )

    shared_center = (actual_path["x"].mean(), actual_path["y"].mean())

    # Rotate both using the same center
    actual_path = rotate_path(actual_path, rotation_angle, center=shared_center)
    planned_path_interp = rotate_path(
        planned_path_interp, rotation_angle, center=shared_center
    )

    # Calculate error
    errors = calculate_error(actual_path, planned_path_interp)
    errors = pd.Series(errors)  # Ensure it's a pandas Series for rolling()
    root_mean_error = sum(errors) / len(errors)
    print(root_mean_error)
    mean_indices, mean_errors = compute_moving_average(errors, window_size=10)

    # Plot the planned vs actual path
    plt.figure(figsize=(12, 6))

    plt.subplot(1, 2, 1)
    plt.plot(
        planned_path_interp["x"],
        planned_path_interp["y"],
        label="Planned Path",
        color="blue",
    )
    plt.scatter(
        planned_path_interp["x"],
        planned_path_interp["y"],
        color="blue",
        s=10,
        label="Planned Path Measuring Point",
    )

    plt.plot(actual_path["x"], actual_path["y"], label="Actual Path", color="red")
    plt.scatter(
        actual_path["x"],
        actual_path["y"],
        color="red",
        s=5,
        label="Actual Path Measuring Point",
    )

    plt.title("Planned vs Actual Path")
    plt.xlabel("X Position")
    plt.ylabel("Y Position")
    plt.legend()
    plt.grid(True)

    # Plot error and moving average
    plt.subplot(1, 2, 2)
    timestamps = actual_path["timestamp"]
    plt.plot(timestamps, errors, label="Error (Distance)", color="green", alpha=0.75)

    mean_errors_series = pd.Series(mean_errors)
    smoothed_errors = mean_errors_series.rolling(window=5, center=True).mean()
    if len(mean_indices) > 0:
        plt.plot(
            timestamps.iloc[mean_indices],
            smoothed_errors,
            label="Avg Error (every 10 pts)",
            color="red",
            linewidth=2,
        )

    plt.title("Error between Actual and Planned Paths")
    plt.xlabel("Timestamp")
    plt.ylabel("Error [m]")
    plt.legend()
    plt.grid(True)

    plt.tight_layout()
    plt.show()


if __name__ == "__main__":
    actual_path_file = "/home/ap4-dev-laptop/Desktop/autonomous_platform/High_Level_Control_Computer/ap4_hlc_code/ap4hlc_ws/logs/paths_csv/palnned_path_imu.csv"
    planned_path_file = "/home/ap4-dev-laptop/Desktop/autonomous_platform/High_Level_Control_Computer/ap4_hlc_code/ap4hlc_ws/logs/paths_csv/actual_path_imu.csv"

    plot_paths_and_error(actual_path_file, planned_path_file)
