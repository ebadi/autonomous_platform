#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import depthai as dai
import time
import numpy as np
from std_msgs.msg import Float32MultiArray, String


class ImuPublisher(Node):

    def __init__(self):
        super().__init__("imu_publisher")

        # Create a publisher for IMU data
        self.imu_publisher = self.create_publisher(Imu, "imu", 10)

        # Create a publisher for the mean and variance data
        self.stats_publisher = self.create_publisher(Float32MultiArray, "imu_stats", 10)

        # Create pipeline
        self.pipeline = dai.Pipeline()

        # Define sources and outputs
        self.imu = self.pipeline.create(dai.node.IMU)
        xlinkOut = self.pipeline.create(dai.node.XLinkOut)
        xlinkOut.setStreamName("imu")
        self.all_gyro = []

        # Enable IMU sensors
        self.imu.enableIMUSensor(dai.IMUSensor.ACCELEROMETER_RAW, 500)
        self.imu.enableIMUSensor(dai.IMUSensor.GYROSCOPE_RAW, 400)
        self.imu.setBatchReportThreshold(1)
        self.imu.setMaxBatchReports(10)

        # Link plugins IMU -> XLINK
        self.imu.out.link(xlinkOut.input)

        # Connect to the device
        self.device = dai.Device(self.pipeline)

        # Output queue for imu bulk packets
        self.imuQueue = self.device.getOutputQueue(
            name="imu", maxSize=50, blocking=False
        )

        self.get_logger().info("IMU publisher node has been started.")

    def timeDeltaToMilliS(self, delta) -> float:
        return delta.total_seconds() * 1000  # Convert to milliseconds

    def publish_imu_stats(self, mean_accelero, variance_accelero):
        # Create and publish a Float32MultiArray for mean and variance
        imu_stats_msg = Float32MultiArray()

        # Adding the names and values as a list to the data field
        imu_stats_msg.data = [
            mean_accelero[0],  # mean_x
            mean_accelero[1],  # mean_y
            mean_accelero[2],  # mean_z
            variance_accelero[0],  # var_x
            variance_accelero[1],  # var_y
            variance_accelero[2],  # var_z
        ]

        # Publish the message
        self.stats_publisher.publish(imu_stats_msg)

    def publish_imu_data(self):
        baseTs = None
        accelero_values_buffer = (
            []
        )  # To store accelerometer readings for the first 5 seconds
        start_time = time.time()  # Track when the 5-second window starts

        # Collect data for 5 seconds
        while time.time() - start_time < 5.0 and rclpy.ok():
            self.get_logger().info("calculating mean and variance")
            imuData = (
                self.imuQueue.get()
            )  # blocking call, will wait until new data has arrived

            imuPackets = imuData.packets
            for imuPacket in imuPackets:
                acceleroValues = imuPacket.acceleroMeter
                # Store accelerometer data for the first 5 seconds
                accelero_values_buffer.append(
                    [acceleroValues.x, acceleroValues.y, acceleroValues.z]
                )

        # Calculate the mean and variance of accelerometer values over the first 5 seconds
        accelero_values_buffer = np.array(accelero_values_buffer)
        mean_accelero = np.mean(accelero_values_buffer, axis=0)
        variance_accelero = np.var(accelero_values_buffer, axis=0)

        # Create a Float32MultiArray message for publishing the mean and variance
        self.publish_imu_stats(mean_accelero, variance_accelero)

        # Now compensate with the calculated mean and variance for the rest of the program
        while rclpy.ok():
            imuData = (
                self.imuQueue.get()
            )  # blocking call, will wait until new data has arrived

            imuPackets = imuData.packets
            for imuPacket in imuPackets:
                acceleroValues = imuPacket.acceleroMeter
                gyroValues = imuPacket.gyroscope

                # Create IMU message to publish
                imu_msg = Imu()

                # Compensate accelerometer data by subtracting the mean
                imu_msg.linear_acceleration.x = acceleroValues.z - mean_accelero[2]
                imu_msg.linear_acceleration.y = acceleroValues.y - mean_accelero[1]
                imu_msg.linear_acceleration.z = acceleroValues.x - mean_accelero[0]

                # Fill the gyroscope data
                imu_msg.angular_velocity.x = gyroValues.z
                imu_msg.angular_velocity.y = gyroValues.y
                imu_msg.angular_velocity.z = -gyroValues.x

                self.all_gyro.append([gyroValues.x, gyroValues.y, gyroValues.z])

                # Set the timestamp (convert to ROS time)
                acceleroTs = acceleroValues.getTimestampDevice()
                gyroTs = gyroValues.getTimestampDevice()
                if baseTs is None:
                    baseTs = acceleroTs if acceleroTs < gyroTs else gyroTs
                imu_msg.header.stamp = self.get_clock().now().to_msg()

                # Publish the IMU message
                self.imu_publisher.publish(imu_msg)

                # Exit condition for the loop (e.g., after 7 seconds)
                if acceleroTs.total_seconds() * 1000 >= 7000.0:
                    break


def main(args=None):
    rclpy.init(args=args)

    imu_publisher = ImuPublisher()

    # Run the publisher loop
    imu_publisher.publish_imu_data()

    # Shutdown ROS 2
    imu_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
