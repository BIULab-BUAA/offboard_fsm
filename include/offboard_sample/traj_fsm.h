/*
 * @Author: xindong324
 * @Date: 2022-03-03 21:57:53
 * @LastEditors: xindong324 xindong324@163.com
 * @LastEditTime: 2023-11-09 12:06:45
 * @Description: file content
 */
#ifndef _TRAJ_FSM__H
#define _TRAJ_FSM__H

#include <Eigen/Eigen>
#include <iostream>
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/TwistStamped.h>
#include <sensor_msgs/NavSatFix.h>

#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/CommandTOL.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/ExtendedState.h>
#include <mavros_msgs/AttitudeTarget.h>
#include <mavros_msgs/PositionTarget.h>
#include <std_msgs/String.h>
#include <tf/transform_datatypes.h> //转换函数头文件
#include <tf/tf.h>

#include <visualization_msgs/Marker.h>

// #include <offboard_sample/pos_controller.h>
// #include <offboard_sample/controller.h>
#include <quadrotor_msgs/PositionCommand.h>

using namespace std;

class TrajFSM
{
private:
    enum FSM_EXEC_STATE{
        INIT,
        TAKEOFF,
        LOITER,
        MISSION,
        EMERGENCY_STOP,
        LAND
    };

    FSM_EXEC_STATE exec_state_;
    int continously_called_times_{0};
    int image_yaw_state_{0};
    bool trigger_, flag_simulation_, start_mission_, flag_emergency_stop_, has_quad_cmd_;

    /** ctrl data **/
    mavros_msgs::State current_state_;
    mavros_msgs::ExtendedState extended_state_;
    geometry_msgs::Pose target_pos_;
    double target_yaw_;
    double takeoff_height_;
    geometry_msgs::PoseStamped local_position_, home_pose_, takeoff_pose_, loiter_pos_;
    geometry_msgs::TwistStamped local_vel_;
    mavros_msgs::SetMode offbset_mode_;
    mavros_msgs::CommandBool arm_cmd_;
    mavros_msgs::PositionTarget local_raw_;
    mavros_msgs::AttitudeTarget att_raw_;
    quadrotor_msgs::PositionCommand quad_command_;
    std_msgs::String state_uav_;



    /*ros utils*/
    ros::NodeHandle node_;
    ros::Timer exec_timer_;
    ros::Time time_quad_cmd_;
    ros::Time time_mission_;
    ros::Subscriber state_sub_, extent_state_sub_, local_position_sub_, local_velocity_sub_, joy_sub_, quad_cmd_sub_;
    ros::Publisher local_pos_pub_, local_att_pub_, local_pos_raw_pub_, marker_pub_, local_vel_pub_, state_pub_;
    ros::ServiceClient arming_client_, setmode_client_, landing_client_;

    /*return value : std::pair <times of the same state be continuously called, current continuously called state>*/
    void changeFSMExecState(FSM_EXEC_STATE new_state, string pos_call);
    std::pair<int, TrajFSM::FSM_EXEC_STATE > timesOfConsecutiveStateCalls();
    void printFSMExecState();
    void pubFSMExecState();
    /* ROS FUNCTIONS */
    void execFSMCallback(const ros::TimerEvent &e);

    void stateCallback(const mavros_msgs::StateConstPtr &msg);
    void extendedStateCallback(const mavros_msgs::ExtendedStateConstPtr &msg);
    void positionCallback(const geometry_msgs::PoseStampedConstPtr &msg);
    void localVelocityCallback(const geometry_msgs::TwistStampedConstPtr &msg);
    void joyCallback(const std_msgs::StringConstPtr &str);
    void quadCmdCallback(const quadrotor_msgs::PositionCommandConstPtr &msg);

    bool reached_target_position(geometry_msgs::Point tar_pos, geometry_msgs::Point cur_pos, double thres=0.2) {
            Eigen::Vector3d tar(tar_pos.x, tar_pos.y, tar_pos.z);
            Eigen::Vector3d cur(cur_pos.x, cur_pos.y, cur_pos.z);

            return ((tar - cur).norm() < thres);
        }

    void execMission();

public:
	TrajFSM();
    ~TrajFSM();

    void init(ros::NodeHandle &nh);

};

#endif