#!/usr/bin/env python
# -*- coding: utf-8 -*-
'''
Author: xindong324
Date: 2022-09-19 19:50:43
LastEditors: xindong324 xindong324@163.com
LastEditTime: 2023-05-17 11:34:29
Description: file content
'''
import rospy
import tf
import math
import copy
from datetime import datetime
import time
from geometry_msgs.msg import *
from mavros_msgs.msg import *
from mavros_msgs.srv import *
from std_msgs.msg import *
from quadrotor_msgs.msg import *

PI = 3.1415926
YAW_DOT_MAX_PER_SEC = PI
last_yaw_ = 0
last_yaw_dot_ = 0
time_forward_ = 1.0

acc_radius = 4.5
time_const = 2

home_pose = PoseStamped()
flag_has_odom_ = False
flag_set_circle = False

def positionCallback( msg):
    global flag_has_odom_, home_pose,time_odom_
    home_pose = msg

    flag_has_odom_ = True
    time_odom_ = rospy.Time.now()
        

def calculate_yaw(yaw_temp, time_now, time_last):
    global last_yaw_, last_yaw_dot_

    yaw_yawdot = [0,0]
    yaw = 0;
    yawdot = 0;

    #dir = traj_[0].evaluateDeBoorT(t_cur + time_forward_) - pos  if t_cur + time_forward_ <= traj_duration_  else traj_[0].evaluateDeBoorT(traj_duration_) - pos;
    #yaw_temp = dir.norm() > 0.1 ? atan2(dir(1), dir(0)) : last_yaw_;
    dt = (time_now - time_last)
    max_yaw_change = YAW_DOT_MAX_PER_SEC * (time_now - time_last);

    yaw = yaw_temp
    if(yaw_temp > PI):
        yaw = yaw_temp - 2*PI
    if (yaw_temp < -PI):
        yaw = yaw_temp + 2 * PI

    delta_yaw = yaw - last_yaw_
    # case 1" form -180 to 180
    if(delta_yaw > PI and yaw_temp > PI/2):
        yawdot = (delta_yaw - 2*PI) / dt
    elif(delta_yaw < -PI and yaw_temp < -PI/2):
        # from -pi 2 pi
        yawdot = (delta_yaw + 2*PI) / dt
    else:
        yawdot = delta_yaw / dt
    
    if(yawdot > YAW_DOT_MAX_PER_SEC):
        yawdot = YAW_DOT_MAX_PER_SEC

    if (math.fabs(yaw - last_yaw_) <= max_yaw_change):
        yaw = 0.5 * last_yaw_ + 0.5 * yaw; 
    yawdot = 0.5 * last_yaw_dot_ + 0.5 * yawdot;
    last_yaw_ = yaw;
    last_yaw_dot_ = yawdot;
    return [yaw, yawdot];


if __name__ == '__main__':
    traj_pub = rospy.Publisher('/position_cmd', PositionCommand, queue_size=1)
    
    rospy.init_node("keyboard_driver")
    rate = rospy.Rate(100)
    home_pose_sub = rospy.Subscriber("/mavros/local_position/pose",PoseStamped, callback=positionCallback, queue_size=10)

    traj = PositionCommand()

    print("Publishing keystrokes. Press Ctrl-C to exit...")
    time_last_ = time.time()
    start_time = time.time()
    last_t = 0
    start_pose = PoseStamped()


    print("get_start")
    trja_radius = acc_radius/(time_const * time_const)
    
    

    while not rospy.is_shutdown():
        if not flag_has_odom_:
            print("no odom")
            continue

        if(flag_has_odom_ and not flag_set_circle):
            start_pose = home_pose
            circle_center_x = trja_radius + start_pose.pose.position.x
            circle_center_y = start_pose.pose.position.y
            flag_set_circle = True
        


        print("start: ", start_pose.pose.position.z)
        now = time.time()
        t = (now - start_time)
        if(t < 1e-6):
            continue
        
        traj.position.z = start_pose.pose.position.z;
        traj.velocity.z = 0
        traj.acceleration.z = 0

        time_const_2 = time_const * time_const


        traj.position.x = acc_radius/(time_const_2) * math.cos(time_const * t) + circle_center_x
        traj.position.y = acc_radius/time_const_2 * math.sin(time_const *t) + circle_center_y
        traj.velocity.x = -acc_radius/time_const * math.sin(time_const *t)
        traj.velocity.y = acc_radius/time_const * math.cos(time_const *t)
        traj.acceleration.x = -acc_radius * math.cos(time_const *t)
        traj.acceleration.y = -acc_radius * math.sin(time_const *t)


        dx = 0.8*math.sin(2*0.1*(t+1)) - 0.8*math.sin(2*0.1*t)
        dy =  0.8*math.sin(0.1*(t+1)) -  0.8*math.sin(0.1*t)

        # yaw_temp = math.atan2(dy,dx)
        yaw_temp = math.atan2(traj.velocity.y,traj.velocity.x)
        
        cur_yaw, cur_yawrate = calculate_yaw(yaw_temp, t, last_t)

        last_t =  t
        # traj.yaw = cur_yaw
        # traj.yaw_dot = cur_yawrate
        # print("yaw: ", cur_yaw, "yaw_rate: ",cur_yawrate)

        traj_pub.publish(traj)
        
        rate.sleep()
    rospy.spin()