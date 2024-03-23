#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import sys
import select
import tty
import termios

class KeyboardListener(Node):
    def __init__(self):
        super().__init__('keyboard_listener')
        self.publisher_ = self.create_publisher(String, 'keys', 10)
        self.old_settings = termios.tcgetattr(sys.stdin)
        tty.setcbreak(sys.stdin.fileno())

    def listen_keys(self):
        while rclpy.ok():
            if select.select([sys.stdin], [], [], 0)[0] == [sys.stdin]:
                key = sys.stdin.read(1)
                self.publisher_.publish(String(data=key))
                self.get_logger().info(f'Publishing: "{key}"')
            rclpy.spin_once(self, timeout_sec=0.01)
    def dest(self):
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.old_settings)
        

def main(args=None):
    rclpy.init(args=args)
    keyboard_listener = KeyboardListener()
    try:
        keyboard_listener.listen_keys()
    except KeyboardInterrupt:
        keyboard_listener.dest()
        pass
    finally:
        keyboard_listener.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()