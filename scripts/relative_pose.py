'''
Author: xindong324
Date: 2022-11-01 19:20:27
LastEditors: xindong324
LastEditTime: 2022-11-01 19:34:00
Description: file content
'''
#!/usr/bin/env python

from gazebo_msgs.msg import *

import copy
import datetime
import imp
import math
import queue
import time
from turtle import pos

from matplotlib.markers import MarkerStyle
from matplotlib.pyplot import flag
from numpy import take
import rospy
import tf
from geometry_msgs.msg import *
#from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Float32, String
from tf.transformations import euler_from_quaternion
import threading
import numpy as np

quad_pose = Pose()
husky_pose = Pose()

quad_name = "iris"
husky_name = "husky"

def gazebo_cb(msg):
    num_model = len(msg.model_name)
    for i in range(num_model):
        if(msg.model_name[i] == quad_name):
            quad_pose = msg.pose[i]
        elif(msg.model_name[i] == husky_name):
            husky_pose = msg.pose[i]

if __name__ == "__main__":
    # may stay out
    rospy.init_node("offb_nodepy", anonymous=True)
    self.local_position_sub_ = rospy.Subscriber("/mavros/local_position/pose",PoseStamped, callback=self.positionCallback, queue_size=1)
                            ``````