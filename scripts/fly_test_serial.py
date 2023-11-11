'''
Author: xindong324
Date: 2022-11-29 14:10:34
LastEditors: xindong324 xindong324@163.com
LastEditTime: 2023-11-09 12:00:32
Description: file content
'''
#!/usr/bin/env python
# -*- coding: utf-8 -*-

from turtle import forward
import rospy

import sys, select, tty, termios
from std_msgs.msg import String
from geometry_msgs.msg import TwistStamped

import socket

################### udp processing
import serial
running = True
order = 0       ## 指令
## 本地地址，可修改获取方式 @TODO
ser = None


#ser_name = "/dev/ttyUSB0"
ser_name = "/dev/ttyUSB1"
baud = 115200

ser = serial.Serial(ser_name, baud)
ser.flushInput()  # 清空缓冲区

def ser_cb(event):
    global order, ser
    
    
    data = ''
    data = data.encode('utf-8')
    
    n = ser.inWaiting()
    
    if n!=0:
        # print("data len: ",n) 
        buffer = ser.read(ser.in_waiting)
        # print("[",time.time(),"]recv->", buffer.hex(), " len: ",len(buffer))  # 打印一下子
        ## 判断帧头和数据长度
        if buffer[0] == 0x55 and buffer[1] == 0xEE and buffer[2] == 0x05 and len(buffer) >= 8:
            checkSum = 0
            ## 计算校验和
            for iNum in range(7):
                checkSum += int(buffer[iNum])

            ## 检查校验和
            if checkSum & 0xff == buffer[7]:
                ##### 命令接收完成
                command_trigger = True
                ##获取并判断指令
                # buffer[6]
                data = buffer[4:7]
                if data[2] == 0x01:
                    order = 1
                    pub_takeoff()
                elif data[2] == 0x02:
                    order = 2
                    pub_land()
                elif data[2] == 0x04:
                    order = 3
                elif data[2] == 0x08:
                    order = 4
                elif data[2] == 0x10:
                    order = 5
                elif data[2] == 0x20:
                    order = 6
                ## 处理指令 @TODO
                ## only for test
                print(order)
        #time.sleep(duration_command)  # 延时，免得CPU出问题(线程占满时间片)  



##################################


takeoff_land_cmd=String()

state = None
vel_pub = None
takeoff_land_pub = None
state_sub = None
time_start = None

forward_x = 0.2 #m/s
rot_z = 0.2  #rad/s

cnt = 0



def state_cb(msg):
    global state,order, cnt
    state = msg.data
    print(state)
    # if order == 1:
    #     pub_takeoff()
    
    # if order == 2: 
    #     pub_land()
    
    # if order == 3:
    #     pub_forward()
        
    # if order == 4:
    #     pub_backward()
    
    # if order == 5:
    #     pub_rotleft()
        
    # if order == 6:
    #     pub_rotright()
    
def pub_takeoff():
    #cmd = True
    if state == "INIT" and state != "TAKEOFF":
        takeoff_land_cmd.data = "a"
        takeoff_land_pub.publish(takeoff_land_cmd)
        print("pub_takeoff")

def pub_land():
    global order
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



if __name__ == '__main__':
    takeoff_land_pub = rospy.Publisher('keys', String, queue_size=1)

    rospy.init_node("keyboard_driver")
    print("start rcv")
    exec_timer_ = rospy.Timer(rospy.Duration(0.1), ser_cb)
    vel_pub = rospy.Publisher("/vel_cmd", TwistStamped, queue_size=1)
    
    state_sub = rospy.Subscriber("/state_uav",String, state_cb)
    
    rospy.spin()
    

