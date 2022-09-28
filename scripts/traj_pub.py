#!/usr/bin/env python
# -*- coding: utf-8 -*-
'''
Author: xindong324
Date: 2022-09-19 19:50:43
LastEditors: xindong324
LastEditTime: 2022-09-27 15:16:41
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

def calculate_yaw(yaw_temp, time_now, time_last):
    global last_yaw_, last_yaw_dot_

    yaw_yawdot = [0,0]
    yaw = 0;
    yawdot = 0;

    #dir = traj_[0].evaluateDeBoorT(t_cur + time_forward_) - pos  if t_cur + time_forward_ <= traj_duration_  else traj_[0].evaluateDeBoorT(traj_duration_) - pos;
    #yaw_temp = dir.norm() > 0.1 ? atan2(dir(1), dir(0)) : last_yaw_;
    dt = (time_now - time_last)
    print(dt)
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

    traj = PositionCommand()

    print("Publishing keystrokes. Press Ctrl-C to exit...")
    time_last_ = time.time()
    start_time = time.time()
    last_t = 0
    while not rospy.is_shutdown():
        now = time.time()
        t = (now - start_time)
        if(t < 1e-6):
            continue
        # 8 zi
        traj.position.x = 0.8 * math.sin(2*0.1*t)
        traj.velocity.x = 0.16 * math.cos(2*0.1*t)
        traj.acceleration.x = -0.032 * math.sin(2*0.1*t)
        traj.position.y = 0.8 *math.sin(0.1*t)
        traj.velocity.y = 0.08 *math.cos(0.1*t)
        traj.acceleration.y = -0.008*math.sin(0.1*t)
        traj.position.z = 0.5 * math.sin(0.1*t) + 1
        traj.velocity.z = 0.05 * math.cos(0.1*t)
        traj.acceleration.z = -0.005 * math.sin(0.1*t)

        dx = 0.8*math.sin(2*0.1*(t+1)) - 0.8*math.sin(2*0.1*t)
        dy =  0.8*math.sin(0.1*(t+1)) -  0.8*math.sin(0.1*t)

        # yaw_temp = math.atan2(dy,dx)
        yaw_temp = math.atan2(traj.velocity.y,traj.velocity.x)
        
        cur_yaw, cur_yawrate = calculate_yaw(yaw_temp, t, last_t)

        last_t =  t
        traj.yaw = cur_yaw
        traj.yaw_dot = cur_yawrate
        print("yaw: ", cur_yaw, "yaw_rate: ",cur_yawrate)

        traj_pub.publish(traj)
        
        rate.sleep()