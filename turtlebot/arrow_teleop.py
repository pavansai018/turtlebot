#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys
import termios
import tty
import select

LINEAR_SPEED = 0.2
ANGULAR_SPEED = 1.0


class ArrowTeleop(Node):

    def __init__(self):
        super().__init__('turtlebot_arrow_teleop')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.settings = termios.tcgetattr(sys.stdin)
        self.twist = Twist()

    def get_key(self):
        tty.setraw(sys.stdin.fileno())
        select.select([sys.stdin], [], [], 0.1)
        key = sys.stdin.read(1)

        # Handle arrow keys (3-character sequence)
        if key == '\x1b':
            key += sys.stdin.read(2)

        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key

    def run(self):
        print("Arrow Teleop Started")
        print("Arrows --> Move")
        print("Space --> Stop")
        print("s --> Exit")

        while True:

            key = self.get_key()

            if key == '\x1b[A':          # Up
                self.twist.linear.x = LINEAR_SPEED
                self.twist.angular.z = 0.0

            elif key == '\x1b[B':        # Down
                self.twist.linear.x = -LINEAR_SPEED
                self.twist.angular.z = 0.0

            elif key == '\x1b[D':        # Left
                self.twist.linear.x = 0.0
                self.twist.angular.z = ANGULAR_SPEED

            elif key == '\x1b[C':        # Right
                self.twist.linear.x = 0.0
                self.twist.angular.z = -ANGULAR_SPEED

            elif key == ' ':
                self.twist.linear.x = 0.0
                self.twist.angular.z = 0.0

            elif key == 's':
                print("Exiting...")
                break

            self.publisher_.publish(self.twist)

        # Stop robot before quitting
        self.twist.linear.x = 0.0
        self.twist.angular.z = 0.0
        self.publisher_.publish(self.twist)


def main(args=None):
    rclpy.init(args=args)
    node = ArrowTeleop()
    node.run()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
