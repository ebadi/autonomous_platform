import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys
import tty
import termios
import signal
from select import select


class TeleopRobot(Node):
    def __init__(self):
        super().__init__("teleop_robot")

        # Publisher for cmd_vel topic
        self.publisher_ = self.create_publisher(Twist, "cmd_vel", 10)

        # Initial values for linear and angular speeds
        self.linear_speed = 0.0
        self.angular_speed = 0.0
        self.l_scale = 15.0  # linear scale factor
        self.a_scale = 15.0  # angular scale factor
        self.last_linear = 0.0
        self.last_angular = 0.0

        self.get_logger().info(
            "Keyboard control node started. Press 'q', 'w', 'e', 'a', 's', 'd' to control the robot. Press 'r' or 'f' to increase or lower max speed. Press 'x' to exit"
        )

        self.timer_ = self.create_timer(0.01, self.keyLoop)

    def keyLoop(self):
        # Setting up terminal in raw mode
        kfd = sys.stdin.fileno()
        cooked = termios.tcgetattr(kfd)
        raw = cooked[:]
        raw[3] = raw[3] & ~(
            termios.ICANON | termios.ECHO
        )  # Disable buffering and echoing
        termios.tcsetattr(kfd, termios.TCSANOW, raw)

        # Define the keycodes for 'wasd'
        key_map = {
            "w": (1.0, 0.0),  # Forward
            "q": (1.0, 1.0),  # Forward Left
            "e": (1.0, -1.0),  # Forward Right
            "s": (-1.0, 0.0),  # Backward
            "a": (-1.0, -1.0),  # Backward Left
            "d": (-1.0, 1.0),  # Backward Right
        }

        try:
            while rclpy.ok():
                key = self.get_key()  # Get the pressed key

                if key in key_map:
                    # If key is in the map, get linear and angular velocities
                    linear, angular = key_map[key]
                elif key == "x":  # Quit the program if 'x' is pressed
                    break

                elif key == "r":
                    self.l_scale += 1
                    self.a_scale += 1
                    self.get_logger().info(
                        f"max speed is linear:{self.l_scale} angular{self.a_scale}"
                    )
                elif key == "f":
                    self.l_scale -= 1
                    self.a_scale -= 1
                    self.get_logger().info(
                        f"max speed is linear:{self.l_scale} angular{self.a_scale}"
                    )
                else:
                    linear, angular = 0.0, 0.0  # Stop if an unrecognized key is pressed

                # Create a Twist message and set velocities
                twist = Twist()
                twist.linear.x = self.l_scale * linear
                twist.angular.z = self.a_scale * angular

                # Publish the twist message
                self.publisher_.publish(twist)
                if linear != self.last_linear or angular != self.last_angular:
                    self.get_logger().info(
                        f"Moving: linear.x={twist.linear.x}, angular.z={twist.angular.z}"
                    )
                self.last_linear = linear
                self.last_angular = angular

        finally:
            # Reset terminal settings when quitting
            termios.tcsetattr(kfd, termios.TCSANOW, cooked)

    def get_key(self):
        """Reads a single key press from the terminal."""
        tty.setraw(sys.stdin.fileno())

        # Wait for input with a short timeout, using select to prevent blocking
        rlist, _, _ = select([sys.stdin], [], [], 0.5)  # 0.1 seconds timeout

        if rlist:  # If input is available, read the key
            key = sys.stdin.read(1)
        else:
            key = ""  # If no input, set key to empty string

        # Reset terminal settings
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, termios.tcgetattr(sys.stdin))

        return key


def main(args=None):
    rclpy.init(args=args)
    node = TeleopRobot()

    # Start the key loop
    node.keyLoop()

    # Shutdown after key loop ends
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
