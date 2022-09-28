#!/usr/bin/env python
# -*- coding: utf-8 -*-
'''
Author: xindong324
Date: 2022-09-16 08:45:15
LastEditors: xindong324
LastEditTime: 2022-09-19 20:19:09
Description: file content
'''


# TODO: check if odom is unavilable

#ros python示例程序，待添加状态机
#!/usr/bin/env python
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
from mavros_msgs.msg import *
from mavros_msgs.srv import *
from quadrotor_msgs.msg import *
from visualization_msgs.msg import *
#from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Float32, String
from tf.transformations import euler_from_quaternion
import threading
import numpy as np



class OffboardFSM:
    def __init__(self) :

        # def fsm state
        self.state = {'init_state': 0, 'auto_takeoff': 1, 'auto_hover': 2, 'auto_mission': 3, 'auto_land': 4,
                    'emergency_stop': 5}
        self.state_name = ['init_state', 'auto_takeoff', 'auto_hover', 'auto_mission', 'auto_land',
                            'emergency_stop']
        # fsm cur state pointer
        self.exec_state_ = 0
        # fsm state function cb
        self.state_func_cb = [self.init_state, self.auto_takeoff, self.auto_hover, 
                            self.auto_misssion, self.auto_land, self.emergency_stop]

        # may stay out
        rospy.init_node("offb_nodepy", anonymous=True)

        self.fsm_num = 0
        print("start")
        # need a class to prase param
        self.flag_simulation_ = True
        self.flag_simulation_ = rospy.get_param("offb_fsm/flag_simulation", True)
        self.target_pos_ = Pose()
        self.target_pos_.position.x = rospy.get_param("offb_fsm/target_x", 0.0)
        self.target_pos_.position.y = rospy.get_param("offb_fsm/target_y", 0.0)
        self.target_pos_.position.z = rospy.get_param("offb_fsm/target_z", 2.0)
        self.target_yaw_ = rospy.get_param("offb_fsm/target_yaw", 0)
        self.reach_thres_ = 0.1
        self.reach_thres_ = rospy.get_param("offb_fsm/target_thres", 0.1)
        
        self.continously_called_times_ = 0
        self.trigger_ = False 
        # self.flag_simulation_ = False 
        self.start_mission_ = False
        self.flag_emergency_stop_ = False
        self.has_quad_cmd_ = False
        self.flag_has_odom_ = False

        #/** ctrl data **/
        self.current_state_ = State()
        self.extented_state_ = ExtendedState()
        self.target_pos_ = Pose()
        self.target_yaw_ = 0.0
        self.local_position_ = PoseStamped()
        self.home_pose_ = PoseStamped()
        self.takeoff_pose_ = PoseStamped()
        self.loiter_pos_ = PoseStamped()
        self.local_vel_ = TwistStamped()
        self.offbset_mode_ = SetModeRequest()
        self.arm_cmd_ = CommandBoolRequest()
        self.quad_command_ = PositionCommand()

        # var pub
        self.local_raw_ = PositionTarget()
        self.att_raw_ = AttitudeTarget()
        

        # /*ros utils*/
        # self.exec_timer_ = rospy.Timer()
        self.time_mission_ = rospy.Time.now()
        self.time_quad_cmd_ = rospy.Time.now()
        self.time_odom_ = rospy.Time.now()
        # subscriber
        self.state_sub_=0
        self.local_position_sub_=0
        self.local_velocity_sub_=0 
        self.joy_sub_ = 0
        
        # publisher
        self.local_pos_pub_ = 0 
        self.local_att_pub_ = 0
        self.local_pos_raw_pub_=0 
        self.marker_pub_ = 0
        # client
        self.arming_client_=0
        self.setmode_client_ = 0
        self.landing_client_ = 0

        self.init_ros()
    
    def init_flight(self):
        pass

    def init_ros(self):
        

        print("end")
        # init state
        # self.trigger_ = False 
        # self.start_mission_ = False
        # self.flag_emergency_stop_ = False
        # self.has_quad_cmd_ = False
        # self.flag_has_odom_ = False
        self.exec_state_ = self.state['init_state']

        # pos_controller_.reset( new PosController(nh));
        # process code thread
        self.exec_timer_ = rospy.Timer(rospy.Duration(0.01), self.fsm_cb)

        # subscrib
        self.local_position_sub_ = rospy.Subscriber("/mavros/local_position/pose",PoseStamped, callback=self.positionCallback, queue_size=1)
        self.local_velocity_sub_ = rospy.Subscriber("/mavros/local_position/velocity_local", TwistStamped,  callback=self.localVelocityCallback, queue_size=1)
        self.state_sub_ = rospy.Subscriber("/mavros/state", State, callback=self.stateCallback, queue_size=1)
        self.extent_state_sub_ = rospy.Subscriber("/mavros/extended_state", ExtendedState, callback=self.extendedStateCallback, queue_size=1)
        self.joy_sub_ = rospy.Subscriber("/keys", String, callback=self.joyCallback, queue_size=1)
        self.quad_cmd_sub_ = rospy.Subscriber("pos_cmd", PositionCommand, callback=self.quadCmdCallback, queue_size=1)
        
        # /************ publisher ******************/
        self.marker_pub_ = rospy.Publisher("visualization_marker", visualization_msgs.msg.Marker, queue_size=10)
        self.local_pos_pub_ =  rospy.Publisher("/mavros/setpoint_position/local", PoseStamped, queue_size=10)
        self.local_pos_raw_pub_ = rospy.Publisher("/mavros/setpoint_raw/local", PositionTarget, queue_size=10)
        self.local_att_pub_ = rospy.Publisher("/mavros/setpoint_raw/attitude", AttitudeTarget, queue_size=10)

        self.arming_client_ = rospy.ServiceProxy("/mavros/cmd/arming", CommandBool)
        self.landing_client_ = rospy.ServiceProxy("/mavros/cmd/land", CommandTOL)
        self.setmode_client_ = rospy.ServiceProxy("/mavros/set_mode", SetMode)
        print("Clients Created")


        self.home_pose_ = copy.deepcopy(self.local_position_)
        self.takeoff_pose_ = copy.deepcopy(self.local_position_)
        print("init height on ground: ", self.local_position_.pose.position.z)
        self.takeoff_pose_.pose.position.z = self.local_position_.pose.position.z + 1.0
        self.loiter_pos_ = copy.deepcopy(self.takeoff_pose_)

        self.att_raw_.type_mask = 0b00000111

        rate = rospy.Rate(10)
        # while( not rospy.is_shutdown() and self.current_state_.connected):
        #     rate.sleep();
        #     print("\rconnecting to FCU...")
        
        rospy.sleep(2)
        rospy.spin()
        
        
    ############## main fsm func ######################
    def fsm_cb(self,event):
        # some important state check
        # code here
        self.fsm_num = self.fsm_num + 1
        #print("fsm")

        if(self.fsm_num == 50):
            self.printFSMExecState()
            self.fsm_num = 0

        # no odom
        if((rospy.Time.now() - self.time_odom_).to_sec() > 1):
            self.flag_has_odom_ = False
            return
        
        if(self.flag_emergency_stop_):
            self.changeFSMExecState(self.state["emergency_stop"], "FSM")

        # some basic process exec each epsiod at every state
        # code here
        # calculate
        # pub msg
        # exec fsm
        self.state_func_cb[int(self.exec_state_)]()

    ############# state cb ###################################################
    def init_state(self):
        if(not self.trigger_):
            return
        # print(self.flag_simulation_)
        self.offbset_mode_.custom_mode = "OFFBOARD";
        self.arm_cmd_.value = True;
        if(self.flag_simulation_):
            print("START INIT");
            if( self.current_state_.mode != "OFFBOARD"):
                self.setmode_client_(self.offbset_mode_);
            else:
                print("OFFB ENABLE")
            if(not self.current_state_.armed):
                self.arming_client_(self.arm_cmd_)
            else:
                print("ARMED")
        

        self.local_pos_pub_.publish(self.home_pose_)
        if(self.current_state_.armed and self.current_state_.mode == "OFFBOARD"):
            print("cur_ori", self.local_position_.pose.position.z)
            self.takeoff_pose_ = copy.deepcopy(self.local_position_)
            a = self.local_position_.pose.position.z
            self.takeoff_pose_.pose.position.z = 1.0 + self.local_position_.pose.position.z
            print("tar", self.takeoff_pose_.pose.position.z)
            print("cur", self.local_position_.pose.position.z)
            print("a",a)
            self.home_pose_ = copy.deepcopy(self.local_position_)
            self.changeFSMExecState(self.state['auto_takeoff'], "FSM");
        

    def auto_takeoff(self):
        self.local_pos_pub_.publish(self.takeoff_pose_);
        
        if(self.reachedTargetPosition(self.takeoff_pose_.pose.position, self.local_position_.pose.position, self.reach_thres_)):
        
            self.changeFSMExecState(self.state["auto_hover"], "FSM");
        

    def auto_hover(self):
        ct, _ = self.timesOfConsecutiveStateCalls()
        if(ct == 1):
            self.loiter_pos_ = copy.deepcopy(self.local_position_)
            self.changeFSMExecState(self.state["auto_hover"], "FSM");
        
        # self.local_pos_pub_.publish(self.loiter_pos_);

        self.time_mission_ = rospy.Time.now()
        self.local_pos_pub_.publish(self.loiter_pos_);
        if(self.has_quad_cmd_):
            self.changeFSMExecState(self.state["auto_mission"], "FSM");
        

    def auto_misssion(self):
        self.execMission()
        if(self.flag_emergency_stop_):
            has_quad_cmd_ = False;
            self.changeFSMExecState(self.state["emergency_stop"], "FSM");
        

        if(not self.has_quad_cmd_):
            self.changeFSMExecState(self.state["auto_hover"], "FSM");
        

    def auto_land(self):
        self.offbset_mode_.custom_mode = "AUTO.LAND";
        if( self.setmode_client_(self.offbset_mode_) ):
            pass
        
        if(self.extended_state_.landed_state == ExtendedState.LANDED_STATE_ON_GROUND):
            self.trigger_ = False
            self.changeFSMExecState(self.state["init_state"], "FSM")


    def emergency_stop(self):
        self.att_raw_.thrust = 0
        self.att_raw_.orientation = Quaternion()
        self.local_att_pub_.publish(self.att_raw_)
    ############# end state cb ###################################################  
    # 
    ################### ros call back#########################
    def stateCallback(self, msg):
        self.current_state_ = msg

    def positionCallback(self, msg):
        self.local_position_ = msg

        self.flag_has_odom_ = True
        self.time_odom_ = rospy.Time.now()
        # /****************pub marker*********************/
        marker = visualization_msgs.msg.Marker
        shape = visualization_msgs.msg.Marker.SPHERE
        x_cor = 0;
        # // Set the frame ID and timestamp.  See the TF tutorials for information on these.
        marker = Marker()
        marker.header.frame_id = "/map"
        marker.header.stamp = rospy.Time.now()

        marker.ns = "basic_shapes"
        marker.id = x_cor

        marker.type = shape

        marker.action = visualization_msgs.msg.Marker.ADD

        marker.pose = self.local_position_.pose

        # // Set the scale of the marker -- 1x1x1 here means 1m on a side
        # // %Tag(SCALE)%
        marker.scale.x = 1
        marker.scale.y = 1
        marker.scale.z = 1

        # Set the color -- be sure to set alpha to something non-zero!
        # %Tag(COLOR)%
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 1.0
        # EndTag(COLOR)%

        #   %Tag(LIFETIME)%
        marker.lifetime = rospy.Duration(20.0)
        # rospy.Time.Duration(20.0);
        # %EndTag(LIFETIME)%
        x_cor= x_cor+1
        self.marker_pub_.publish(marker);

    def localVelocityCallback(self, msg):
        self.local_vel_ = msg

    def joyCallback(self, str):
        
        if(str.data == "d" or str.data == "D"):
            self.trigger_ = False
            
            self.changeFSMExecState(self.state["auto_land"], "JOY")
        elif(str.data == "g" or str.data == "G"):
            self.trigger_ = False
            self.changeFSMExecState(self.state['emergency_stop'], "JOY")
        elif(str.data == "a" or str.data == "A"):
            self.trigger_ = True
        
        elif(str.data == "w" or str.data == "W"):
            self.start_mission_ = True
        
        elif(str.data == "j" or str.data == "J"):
            self.image_yaw_state_ = 1
        
        elif(str.data == "k" or str.data == "K"):
            self.image_yaw_state_ = 0
        
        print("str.data: ", str.data)

    # msg: ExtendedState
    def extendedStateCallback(self, msg):
        self.extended_state_ = msg

    # msg: quadrotor_msgs::PositionCommandConstPtr
    def quadCmdCallback(self, msg):
        self.quad_command_ = msg
        self.has_quad_cmd_ = True
        self.time_quad_cmd_ = rospy.Time.now()


    ##################### end ros callback #######################

    ################## functional function #####################
    def sample(self):
        pass

    def changeFSMExecState(self, new_state, pos_call):
        if(new_state == self.exec_state_):
            self.continously_called_times_ = self.continously_called_times_ + 1
        else:
            self.continously_called_times_ = 1;

        pre_s = int(self.exec_state_);
        self.exec_state_ = new_state;
        print("[" + pos_call + "]: from" + self.state_name[pre_s] + "to" 
                + self.state_name[int(new_state)])
        
    def timesOfConsecutiveStateCalls(self):
        return [self.continously_called_times_, self.exec_state_]
    
    def printFSMExecState(self):
        print("[FSM]: state: " + self.state_name[int(self.exec_state_)])
    
    def reachedTargetPosition(self, tar_pos, cur_pos, thres):
        tar = np.array([tar_pos.x, tar_pos.y, tar_pos.z], dtype=np.float32)
        cur = np.array([cur_pos.x, cur_pos.y, cur_pos.z], dtype=np.float32)
        #print("tar: ", tar_pos.x, tar_pos.y,tar_pos.z)
        #print("cur: ", cur_pos.x, cur_pos.y,cur_pos.z)
        #print(np.linalg.norm(tar - cur), thres)
        return (np.linalg.norm(tar - cur) < thres)


    
    # quat 2 eular
    # param: msg: geometry_msgs::PoseStamped.Pose
    # return: roll, pitch, yaw
    def quat2eular(self, msg):
        (r, p, y) = tf.transformations.euler_from_quaternion([msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w])
        return r,p,y


    ##################### end functional function #############

    ####################### publish raw control val ###################
    # hover 1m above
    # @param: pose: type mavros_msgs:PoseStamped
    #         stamp = rospy.Time.now();
    def publishHoverPosition(self, pose, stamp):
        self.local_raw_.header.stamp = stamp
        self.local_raw_.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
        self.local_raw_.type_mask = (PositionTarget.IGNORE_VX | PositionTarget.IGNORE_VY | PositionTarget.IGNORE_VZ | 
                                    PositionTarget.IGNORE_AFX | PositionTarget.IGNORE_AFY | PositionTarget.IGNORE_AFZ |
                                    PositionTarget.FORCE | PositionTarget.IGNORE_YAW_RATE)
        self.local_raw_.position = pose.pose.position
        _,_, self.local_raw_.yaw = self.quat2eular(pose.pose)
        self.local_pos_raw_pub_.publish(self.local_raw_)
    
    # hover 1m above
    # @param: pose: type mavros_msgs:PoseStamped
    #         tar_vel: geometry_msgs/Vector3
    #         tar_acc: geometry_msgs/Vector3
    #         tar_yawrate: double
    #         stamp = rospy.Time.now();
    def publishPositionCtrl(self, tar_position, tar_vel, tar_acc, tar_yawrate, stamp):
        self.local_raw_.header.stamp = stamp
        self.local_raw_.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
        self.local_raw_.type_mask = 0
        self.local_raw_.position = copy.deepcopy(tar_position.pose.position)
        self.local_raw_.velocity = copy.deepcopy(tar_vel)
        self.local_raw_.acceleration_or_force = copy.deepcopy(tar_acc)
        _,_, self.local_raw_.yaw = self.quat2eular(tar_position.pose)
        self.local_raw_.yaw_rate = tar_yawrate
        self.local_pos_raw_pub_.publish(self.local_raw_)
    
    def publishVelocityCtrl(self, tar_yaw, tar_vel, tar_acc, tar_yawrate, stamp):
        self.local_raw_.header = stamp
        self.local_raw_.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
        self.local_raw_.type_mask = PositionTarget.IGNORE_PX | PositionTarget.IGNORE_PY | PositionTarget.IGNORE_PZ
        
        self.local_raw_.velocity = copy.deepcopy(tar_vel)
        self.local_raw_.acceleration_or_force = copy.deepcopy(tar_acc)
        self.local_raw_.yaw = tar_yaw.pose.position
        self.local_raw_.yaw_rate = tar_yawrate
        self.local_pos_raw_pub_.publish(self.local_raw_)


    def publishAttitudeCtrl(self, tar_att_q, tar_thr, stamp):
        self.att_raw_.header.stamp = stamp
        # self.att_raw_.header.frame_id = "FCU"
        

        self.att_raw_.type_mask = (AttitudeTarget.IGNORE_ROLL_RATE |
                        AttitudeTarget.IGNORE_PITCH_RATE |
                        AttitudeTarget.IGNORE_YAW_RATE)

        self.att_raw_.orientation.x = tar_att_q.x
        self.att_raw_.orientation.y = tar_att_q.y
        self.att_raw_.orientation.z = tar_att_q.z
        self.att_raw_.orientation.w = tar_att_q.w

        self.att_raw_.thrust = tar_thr

        self.local_att_pub_.publish(self.att_raw)

    def publishAngularRateCtrl(self, tar_rate, tar_thr, stamp):
        self.att_raw_.header.stamp = stamp
        # self.att_raw_.header.frame_id = "FCU"

        self.att_raw_.type_mask = AttitudeTarget.IGNORE_ATTITUDE

        self.att_raw_.body_rate.x = tar_rate.x
        self.att_raw_.body_rate.y = tar_rate.y
        self.att_raw_.body_rate.z = tar_rate.z

        self.att_raw_.thrust = tar_thr

        self.local_att_pub_.publish(self.att_raw)



########################### exec func in mission ################
    def execMission(self):
        if((rospy.Time.now() - self.time_quad_cmd_).to_sec() > 0.5):
            self.has_quad_cmd_ = False
            return

        # TODO: NEED to add check odom
        if(not self.flag_has_odom_):
            self.flag_emergency_stop_ = True
            return 
        
        self.local_raw_.header.stamp = rospy.Time.now();
        self.local_raw_.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
        self.local_raw_.type_mask = 0
        self.local_raw_.position = self.quad_command_.position
        self.local_raw_.velocity = self.quad_command_.velocity
        self.local_raw_.acceleration_or_force = self.quad_command_.acceleration
        
        self.local_raw_.yaw = self.quad_command_.yaw
        self.local_raw_.yaw_rate = self.quad_command_.yaw_dot
        
        self.local_pos_raw_pub_.publish(self.local_raw_)
        
        
        
if __name__ == "__main__":
    OffboardFSM()
    # rospy.spin()



