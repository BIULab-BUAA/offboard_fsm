'''
Author: xindong324
Date: 2022-11-29 14:10:34
LastEditors: xindong324
LastEditTime: 2022-12-05 16:16:26
Description: file content
'''
#!/usr/bin/env python
# -*- coding: utf-8 -*-

import imp
from time import sleep
from turtle import forward
import rospy

import sys, select, tty, termios
from std_msgs.msg import String
from geometry_msgs.msg import TwistStamped

import time

################### udp processing
import socket

running = True
ident = 0
order = [0x01,0x04,0x08,0x10, 0x20,0x02]       ## 指令
## 本地地址，可修改获取方式 @TODO
addr = ('127.0.0.1', 6665)

## 创建socket
udpSocket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
udpSocket.bind(addr)

## socket 接收 函数
def fsm_cb():
    global order, udpSocket,ident
    buffer = [0x55,0xEE,0x05,0,0,0,0,0,0]
    while True:
        if(ident > 5):
            break
        print("order: ", order[ident])
        buffer[7] = order[ident] &0xFF
        checkSum = 0
        for i in range(8):
            checkSum += int(buffer[i])
        buffer[8] = checkSum & 0xFF
        udpSocket.sendto(bytes(buffer),("127.0.0.1",6666))
        ident += 1
        time.sleep(5)
    print("send done")    
    udpSocket.close()
    

if __name__ == '__main__':
    fsm_cb()
    

