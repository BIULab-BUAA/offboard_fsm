#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import sys
import select
import tty
import termios
import os

class KeyboardPublisher(Node):
    def __init__(self):
        super().__init__('keyboard_publisher')
        self.publisher_ = self.create_publisher(String, '/keys', 10)
        if os.isatty(sys.stdin.fileno()):
            print("file no")
            self.old_settings = termios.tcgetattr(sys.stdin)
            tty.setcbreak(sys.stdin.fileno())
            self.stdin_is_terminal = True
        else:
            self.stdin_is_terminal = False

    def publish_key(self):
        while rclpy.ok():
            if select.select([sys.stdin], [], [], 0.1)[0]:
                key = sys.stdin.read(1)
                msg = String()
                msg.data = key
                self.publisher_.publish(msg)
                self.get_logger().info('Publishing: "%s"' % msg.data)
        
        # if not self.stdin_is_terminal:
        #     self.get_logger().warn("Standard input is not a terminal. Keyboard listening is disabled.")
        #     return
        # Your existing keyboard listening logic...

    def __del__(self):
        if hasattr(self, 'old_settings') and self.old_settings and self.stdin_is_terminal:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.old_settings)

def main(args=None):
    rclpy.init(args=args)
    keyboard_publisher = KeyboardPublisher()
    try:
        keyboard_publisher.publish_key()
    finally:
        # Explicit cleanup in try/finally block
        if keyboard_publisher.stdin_is_terminal:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, keyboard_publisher.old_settings)
        keyboard_publisher.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()