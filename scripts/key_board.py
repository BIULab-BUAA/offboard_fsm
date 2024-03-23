#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import sys
import select
import tty
import termios
import os

def cleanup(node):
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_attr)
def getKey():
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def main(args=None):
    global old_attr
    rclpy.init(args=args)
    node = Node("simple_publisher")
    key_pub = node.create_publisher(String, '/keys', 10)
    old_attr = termios.tcgetattr(sys.stdin)
    tty.setcbreak(sys.stdin.fileno())
    print("Publishing keystrokes. Press Ctrl-C to exit...")
         
    # node.add_on_shutdown(lambda: cleanup(node))
    timer_period = 0.01
    while rclpy.ok():
        if select.select([sys.stdin], [], [], 0)[0] == [sys.stdin]:
            key_pub.publish(sys.stdin.read(1))
            rclpy.spin_once(node, timeout_sec=timer_period)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_attr)  
    node.get_logger().info('Node is shutting down, performing cleanup...')

    node.destroy_node()
    rclpy.shutdown()  

if __name__ == '__main__':
    main()