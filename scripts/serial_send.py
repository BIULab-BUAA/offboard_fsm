'''
Author: xindong324
Date: 2022-11-29 14:10:34
LastEditors: xindong324 xindong324@163.com
LastEditTime: 2023-11-09 11:50:45
Description: file content
'''
#!/usr/bin/env python
# -*- coding: utf-8 -*-

import time
from time import sleep


import serial

################### serial

# header 0x55 0xee  0x05


running = True
ident = 0
order = [0x01,0x02, 0x08,0x10, 0x20,0x40,0x80, 0x04]       ## 指令
## 本地地址，可修改获取方式 @TODO
addr = "/dev/ttyUSB0"
baud = 115200


## socket 接收 函数
def fsm_cb():
    global order, ser,ident
    ser = serial.Serial(addr,baud) #启动串口
    data_len = len(order)
    print('serial test start ...')
    buffer = [0x55,0xEE,0x05,0,0,0,0,0]
    if True:
        # if(ident > data_len - 1):
        #     break
        print("order: ", order[ident])
        buffer[6] = order[ident] &0xFF
        checkSum = 0
        for i in range(7):
            checkSum += int(buffer[i])
        buffer[7] = checkSum & 0xFF
        ser.write(buffer)
        ident += 1
        time.sleep(8)
    print("send done")    
    if ser != None:
        ser.close()
    

if __name__ == '__main__':
    fsm_cb()
    

