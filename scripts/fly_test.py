'''
Author: xindong324
Date: 2022-11-29 14:10:34
LastEditors: xindong324
LastEditTime: 2022-11-30 14:09:26
Description: file content
'''
#!/usr/bin/env python
# -*- coding: utf-8 -*-

from turtle import forward
import rospy

import sys, select, tty, termios
from std_msgs.msg import String
from geometry_msgs.msg import TwistStamped

takeoff_land_cmd=String()

state = None
cmd = 0
vel_pub = None
takeoff_land_pub = None
state_sub = None
time_start = None

forward_x = 0.2 #m/s
rot_z = 0.2  #rad/s

cnt = 0

def state_cb(msg):
    global state,cmd, cnt
    state = msg.data
    print(state)
    if cmd == 1:
        pub_takeoff()
    
    if cmd == 2: 
        pub_land()
    
    if cmd == 3:
        pub_forward()
        
    if cmd == 4:
        pub_backward()
    
    if cmd == 5:
        pub_rotleft()
        
    if cmd == 6:
        pub_rotright()
    
def pub_takeoff():
    #cmd = True
    if state == "INIT" and state != "TAKEOFF":
        takeoff_land_cmd.data = "a"
        takeoff_land_pub.publish(takeoff_land_cmd)
        print("pub_takeoff")

def pub_land():
    global cmd
    if state != "LAND" and state != "INIT":
        takeoff_land_cmd.data = "d"
        takeoff_land_pub.publish(takeoff_land_cmd)
        print("pub_land")

def pub_forward():
    vel_cmd = TwistStamped()
    vel_cmd.header.stamp = rospy.Time.now()
    vel_cmd.twist.linear.x = forward_x
    vel_cmd.twist.angular.z = 0
    vel_pub.publish(vel_cmd)
    #print("pub_vel")

def pub_backward():
    vel_cmd = TwistStamped()
    vel_cmd.header.stamp = rospy.Time.now()
    vel_cmd.twist.linear.x = -forward_x
    vel_cmd.twist.angular.z = 0
    vel_pub.publish(vel_cmd)

def pub_rotleft():
    vel_cmd = TwistStamped()
    vel_cmd.header.stamp = rospy.Time.now()
    vel_cmd.twist.linear.x = 0
    vel_cmd.twist.angular.z = rot_z
    vel_pub.publish(vel_cmd)

def pub_rotright():
    vel_cmd = TwistStamped()
    vel_cmd.header.stamp = rospy.Time.now()
    vel_cmd.twist.linear.x = 0
    vel_cmd.twist.angular.z = -rot_z
    vel_pub.publish(vel_cmd)

def fsm_cb(event):
    global cmd,cnt
    if(cmd == 0):
        cmd = 1
        
    if(cmd == 1):
        cnt += 1
        if(cnt > 10):
            cmd=3
            cnt = 0
    
    if(cmd == 3):
        cnt += 1
        if(cnt > 30):
            cmd=4
            cnt = 0
    
    if(cmd == 4):
        cnt += 1
        if(cnt > 30):
            cmd=5
            cnt = 0

    if(cmd == 5):
        cnt += 1
        if(cnt > 30):
            cmd=6
            cnt = 0
    
    if(cmd == 6):
        cnt += 1
        if(cnt > 30):
            cmd=2
            cnt = 0
    
    if(cmd == 2):
        cmd = 2



if __name__ == '__main__':
    takeoff_land_pub = rospy.Publisher('keys', String, queue_size=1)

    rospy.init_node("keyboard_driver")
    exec_timer_ = rospy.Timer(rospy.Duration(0.1), fsm_cb)
    vel_pub = rospy.Publisher("/vel_cmd", TwistStamped, queue_size=1)
    
    state_sub = rospy.Subscriber("/state_uav",String, state_cb)
    
    rospy.spin()
    

