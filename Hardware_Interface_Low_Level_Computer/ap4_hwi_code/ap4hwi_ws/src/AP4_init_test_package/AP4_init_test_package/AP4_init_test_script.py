import time
import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from std_msgs.msg import Int16
from geometry_msgs.msg import Twist


class MinimalPublisher(Node):

    def __init__(self):
        super().__init__("minimal_publisher")
        self.publisher_ = self.create_publisher(String, "topic", 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

        self.set_steering_angle_pub_ = self.create_publisher(Twist, "/cmd_vel", 10)
        self.steering_angle_sub_ = self.create_subscription(
            Int16, "/GET_0x7d0_Get_SteeringAngle", self.steering_angle_callback, 10
        )
        self.last_steering = 0

        print("Steering oK? ", self.check_steering_ok())

        for i in range(5):
            print("itarion ", i)

            print(self.check_ros_node_is_up())
            print(self.check_ros_topics_availible())

    def timer_callback(self):
        msg = String()
        msg.data = "Hello World: %d" % self.i
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1

    # *
    # Method which checks that important nodes are started
    # List of nodes that should be up has to be changed by a developer
    # *#
    def check_ros_node_is_up(self):
        node_up_list = super().get_node_names()
        list_should_be_up = [
            "can_ros2_interface_node",
            "socket_can_receiver",
            "socket_can_sender",
        ]

        print("node up list:", node_up_list)

        # returns true if all nodes that should be up are a subset of the list of nodes that should be up
        nodes_up_ok = all(node in list_should_be_up for node in node_up_list)

        # Get a list of nodes that are not started, which should be started
        list_nodes_not_started = []
        if not nodes_up_ok:
            for node_not_up in list_should_be_up:
                if node_not_up not in node_up_list:
                    list_nodes_not_started.append(node_not_up)

        return (nodes_up_ok, list_nodes_not_started)

    def check_ros_topics_availible(self):
        active_topics = super().get_topic_names_and_types()
        topic_names = []
        for topic in active_topics:
            topic_names.append(topic[0])

        return topic_names

    # *
    # Sets a desired steering angle
    # waits for at most 5 seconds
    # reads steering angle
    # checks if desired steering angle could be reached
    # *#
    def check_steering_ok(self):
        steering_cmd_left = Twist()
        steering_cmd_left.angular.z = -1.0
        self.set_steering_angle_pub_.publish(steering_cmd_left)
        # wait 3 seconds
        start_time = self.get_clock().now()
        max_wait_time = rclpy.duration.Duration(seconds=5, nanoseconds=0)

        while (
            self.get_clock().now() - start_time
        ) < max_wait_time and self.last_steering > -30:
            self.set_steering_angle_pub_.publish(steering_cmd_left)
        wait_time = (self.get_clock().now() - start_time).nanoseconds * 1e-9
        print("waited: ", wait_time)

        # time.sleep(10)
        # rclpy.sleep_for(3)
        # if(self.last_steering > -30):
        # steering should be turned to the left, i.e have a negative steering angle
        # return False

        steering_cmd_right = Twist()
        steering_cmd_right.angular.z = 0.1
        self.set_steering_angle_pub_.publish(steering_cmd_right)
        # wait 3 seconds
        # rclpy.sleep_for(3)
        if self.last_steering < 30:
            # steering should be turned to the left, i.e have a negative steering angle
            return False

        # return steering to faceing forwards
        steering_cmd_center = Twist()
        steering_cmd_left.angular.z = 0
        self.set_steering_angle_pub_.publish(steering_cmd_center)
        # wait 3 seconds
        # rclpy.sleep_for(3)
        if self.last_steering > 5 or self.last_steering < -5:
            # steering should be centered, i.e steering angle = 0
            return False

        return True

    def steering_angle_callback(self, msg):
        self.last_steering = msg.data
        print("RECIEVED CALLBACK!!!!!!!")


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
