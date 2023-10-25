#!/usr/bin/env python
# -*- coding: utf-8 -*-
'''
Author: xindong324
Date: 2022-09-19 19:50:43
LastEditors: xindong324
LastEditTime: 2022-10-24 21:29:05
Description: file content
'''
# from asyncio import futures
import rospy
import tf
import math
import copy
import numpy as np
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

duration  = 5

gravity_acc = 9.81
pit_ang_end = -PI/4
roll_ang_end = PI/4

acc_x = gravity_acc * math.tan(pit_ang_end)
acc_y = -gravity_acc * math.tan(roll_ang_end)


start_state = np.array([0,0,1.0])
end_state = np.array([5,0.15,2.5])
end2_state = np.array([10,0,1.0])

sv_state = np.array([0, 0, 0])

# ev_state = np.array([2, 0, 0])
ev2_state = np.array([0, 0, 0])

sa_state = [0, 0, 0]
ea_state = [0, acc_y, 0]
ea2_state = [0, 0, 0]



# start_state = np.array([0,0,1.0])
# end_state = np.array([2,0,0.5])

# pos_min_constrain = np.array([-1, -1, 0])
# pos_max_constrain = np.array([3, 2, 2.5])

# sv_state = np.array([0, 0, 0])

# ev_state = np.array([0, 0, 0])

# sa_state = [0, 0, 0]
# ea_state = [acc_x, 0, 0]

# coeff_x = [-0.0065, 0.0844, -0.2588, 0, 0, 0]
# coeff_y = [0.0021, -0.0375, 0.1749, 0, 0, 0]
# coeff_z = [-0.00089, 0.0156, -0.0729, 0, 0, 3]


A = np.array([[  0,   0,   0,   0, 0, 1],
            [duration**5, duration**4, duration**3, duration**2, duration, 1],
            [0,     0,     0,   0, 1, 0],
            [5*duration**4, 4*duration**3, 3*duration**2, 2**duration, 1, 0],
            [0,     0,     0,   2, 0, 0],
            [20*duration**3, 12*duration**2, 6*duration, 2, 0, 0]])

bx =[start_state[0], end_state[0], sv_state[0], ev_state[0], sa_state[0], ea_state[0]]
by =[start_state[1], end_state[1], sv_state[1], ev_state[1], sa_state[1], ea_state[1]]
bz =[start_state[2], end_state[2], sv_state[2], ev_state[2], sa_state[2], ea_state[2]]

coeff_x = np.linalg.solve(A,bx)
coeff_y = np.linalg.solve(A,by)
coeff_z = np.linalg.solve(A,bz)
coeff = np.array([coeff_x, coeff_y, coeff_z])
print("coeff_sz" , coeff.shape)
print(coeff)

def get_minmax(start_t,end_t,coeffc):
    der = np.polyder(coeffc)
    se_time = np.array([start_t, end_t])
    print("coeff",coeffc)
    print("derive coeff",der)
    root_val = np.roots(der).real
    print("root_t1", root_val)
    root_val = root_val[root_val>start_t]
    tmp_time = root_val[root_val<end_t]
    print("root_t", tmp_time)
    if(len(tmp_time ) > 0):
        tmp_time = np.hstack((se_time, tmp_time))
    else:
        tmp_time = se_time
    vals = np.polyval(coeffc, tmp_time)
    print("vals",vals)
    # tmp_vals = vals[vals[[vals>=start_t]<=end_t]]
    
    
    min_val = min(vals)
    max_val = max(vals)
    print("min_val: ", min_val, " max_val ", max_val)
    return min_val, max_val

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
        yawdot = YAW_DOT_MAX_PER_SEC,
    if (math.fabs(yaw - last_yaw_) <= max_yaw_change):
        yaw = 0.5 * last_yaw_ + 0.5 * yaw; 
    yawdot = 0.5 * last_yaw_dot_ + 0.5 * yawdot;
    last_yaw_ = yaw;
    last_yaw_dot_ = yawdot;
    return [yaw, yawdot]

if __name__ == '__main__':
    
    rospy.init_node("keyboard_driver")
    rate = rospy.Rate(100)

    traj_pub = rospy.Publisher('/position_cmd', PositionCommand, queue_size=1)
    emergency_pub = rospy.Publisher('/emergency_landing', Bool, queue_size=1)

    last_yaw = 0

    traj = PositionCommand()

    min_valx,max_valx = get_minmax(0,duration,coeff_x)
    min_valy,max_valy = get_minmax(0,duration,coeff_y)
    min_valz,max_valz = get_minmax(0,duration,coeff_z)

    print("min_x, max_x", min_valx,max_valx)
    print("min_y, max_y", min_valy,max_valy)
    print("min_z, max_z", min_valz,max_valz)

    if (min_valx < pos_min_constrain[0] or min_valy < pos_min_constrain[1] or min_valz < pos_min_constrain[2] or max_valx > pos_max_constrain[0] or max_valy > pos_max_constrain[1] or max_valz > pos_max_constrain[2]):
        rospy.logwarn("out of bound")

    print("Publishing keystrokes. Press Ctrl-C to exit...")
    time_last_ = time.time()
    start_time = time.time()
    last_t = 0
    while not rospy.is_shutdown():
        now = time.time()
        t = (now - start_time)
        if(t < 1e-6):
            continue
        if(t >= duration):
            flag_em = Bool()
            flag_em.data = True
            emergency_pub.publish(flag_em)
            break;

        t_vec = np.array([[t**5, t**4, t**3, t**2, t, 1],
                        [5*t**4, 4*t**3, 3*t**2, 2*t, 1, 0],
                        [20*t**3, 12*t**2, 6*t, 2, 0, 0]]
                        ).T
        t_future = t+0.5;
        t_fu_vec = np.array([[t_future**5, t_future**4, t_future**3, t_future**2, t_future, 1]])

        pos_vel_acc = np.dot(coeff , t_vec)
        future_pos = np.dot(coeff, t_fu_vec.T)

        pos = pos_vel_acc[:,0]

        vel = pos_vel_acc[:,1]
        acc = pos_vel_acc[:,2]

        # 8 zi
        traj.position.x = pos[0]
        traj.velocity.x = vel[0]
        traj.acceleration.x = acc[0]
        traj.position.y = pos[1]
        traj.velocity.y = vel[1]
        traj.acceleration.y = acc[1]
        traj.position.z = pos[2]
        traj.velocity.z = vel[2]
        traj.acceleration.z = acc[2]

        dx = future_pos[0] - pos[0]
        dy =  future_pos[1] -  pos[1]

        # yaw_temp = math.atan2(dy,dx)
        yaw_temp = math.atan2(traj.velocity.y,traj.velocity.x)
        
        #cur_yaw, cur_yawrate = calculate_yaw(yaw_temp, t, last_t) 

        # d_yaw = cur_yaw - last_yaw
        # d_yaw = d_yaw - 2 * PI if d_yaw >= PI else d_yaw
        # d_yaw = d_yaw + 2 * PI if d_yaw <= -PI else d_yaw
        # d_yaw_abs = abs(d_yaw)
        # if (d_yaw_abs >= 0.02):
        #     cur_yaw = last_yaw + d_yaw / d_yaw_abs * 0.02
        # cur_yawrate = (cur_yaw - last_yaw)/(t - last_t)
        
        last_t =  t
        # last_yaw = cur_yaw
        traj.yaw = 0
        #traj.yaw_dot = cur_yawrate
        #print("yaw: ", cur_yaw, "yaw_rate: ",cur_yawrate)
        

        traj_pub.publish(traj)
        
        rate.sleep()

    while not rospy.is_shutdown():
        flag_em = Bool()
        flag_em.data = True
        emergency_pub.publish(flag_em);