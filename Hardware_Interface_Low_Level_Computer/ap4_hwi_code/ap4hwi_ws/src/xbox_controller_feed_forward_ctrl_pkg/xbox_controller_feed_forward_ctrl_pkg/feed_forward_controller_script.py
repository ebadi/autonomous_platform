import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from geometry_msgs.msg import Twist  # /cmd_vel topic
from std_msgs.msg import UInt16
from std_msgs.msg import Int8

import numpy as np


class MinimalPublisher(Node):

    def __init__(self):
        super().__init__("feed_forward_ctrl_node")

        # Create publishers with topics out from feed forward controller
        self.publisher_throttle_ = self.create_publisher(
            UInt16, "/SET_0x3e8_Act_ThrottleVoltage", 10
        )
        self.publisher_break_ = self.create_publisher(
            UInt16, "/SET_0x3e8_Act_BreakVoltage", 10
        )
        self.publisher_steering_position_ = self.create_publisher(
            Int8, "/SET_0x3e8_Act_SteeringPosition", 10
        )

        # create timed publish rate of commands
        timer_period = 1 / 20
        self.timer_ = self.create_timer(timer_period, self.TimerCallback)

        # constants, maximum and minimum millivolts
        self.max = 4350
        self.min = 850

        # internal storage of variables to publish
        self.saved_throttlevoltage_ = self.min
        self.saved_brakevoltage_ = self.min
        self.saved_steeringangle_ = 0

        # create subscribers
        # Listen to /cmd_vel topic, once cmd_vel message recieved, perform controller action
        self.subscriber_cmd_vel_ = self.create_subscription(
            Twist, "/cmd_vel", self.Callback_cmd_vel, 10
        )

        # TODO listen to wheel speed sensor messages to perform feedback controll
        # self.subscriber_vehicle_speed_ = ...

    def ThrottleVoltageFeedForward(self, msg):
        x_dot = msg.linear.x
        kp = self.max / 0.7
        x1_dot = x_dot * kp

        # limit range of output voltage between 850 and 4350 mV
        if x1_dot < self.min:
            x1_dot = self.min
        elif x1_dot > self.max:
            x1_dot = self.max

        self.saved_throttlevoltage_ = x1_dot

    def BreakVoltageFeedForward(self, msg):
        x_dot = msg.linear.x
        kp = -self.max / 0.7
        x1_dot = x_dot * kp

        # limit range of output voltage between 850 and 4350 mV
        if x1_dot < self.min:
            x1_dot = self.min
        elif x1_dot > self.max:
            x1_dot = self.max

        self.saved_brakevoltage_ = x1_dot

    def SteeringFeedForward(self, msg):
        max = 40.0
        min = -40.0

        rz = msg.angular.z
        kp = -90

        y = rz * kp

        # constrain steering value between min and max
        if y > max:
            y = max
        elif y < min:
            y = min

        # remove any rounding errors close to zero
        dead_space = 3  # degrees
        if y > 0 and y < dead_space:
            y = 0
        elif y < 0 and y > -dead_space:
            y = 0

        self.saved_steeringangle_ = y
        # print("y is:", y)

    # *
    # Callback function
    # msg is of type geometry_msgs/msg/Twist
    # *#
    def Callback_cmd_vel(self, msg):

        self.ThrottleVoltageFeedForward(msg)
        self.BreakVoltageFeedForward(msg)
        self.SteeringFeedForward(msg)

    def TimerCallback(self):
        # throttle
        out_msg = UInt16()
        out_msg.data = int(self.saved_throttlevoltage_)
        self.publisher_throttle_.publish(out_msg)

        # break
        out_msg = UInt16()
        out_msg.data = int(self.saved_brakevoltage_)
        self.publisher_break_.publish(out_msg)

        # steering
        out_msg = Int8()
        out_msg.data = int(self.saved_steeringangle_)
        self.publisher_steering_position_.publish(out_msg)


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
