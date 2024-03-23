#ifndef _TRAJ_FSM__H
#define _TRAJ_FSM__H

#include <rclcpp/rclcpp.hpp>

#include <Eigen/Eigen>
#include <iostream>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <nav_msgs/msg/odometry.hpp>

// #include <mavros_msgs/msg/command_bool.h>
#include <mavros_msgs/msg/command_code.hpp>
#include <mavros_msgs/msg/state.hpp>
#include <mavros_msgs/srv/command_bool.hpp>
#include <mavros_msgs/srv/set_mode.hpp>
#include <mavros_msgs/srv/command_tol.hpp>
#include <mavros_msgs/msg/extended_state.hpp>
#include <mavros_msgs/msg/attitude_target.hpp>
#include <mavros_msgs/msg/position_target.hpp>
#include <std_msgs/msg/string.hpp>
#include <sensor_msgs/msg/imu.hpp>

#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"

#include <visualization_msgs/msg/marker.hpp>

// #include <offboard_sample/pos_controller.h>
// #include <offboard_sample/controller.h>
// #include <quadrotor_msgs/msg/position_command.h>
#include <quadrotor_msgs/msg/position_command.hpp>


#include <rclcpp/rclcpp.hpp>
#include <stdint.h>

#include <chrono>

using namespace std;

class TrajFSM : public rclcpp::Node
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
    bool use_telem_;

    /** ctrl data **/
    mavros_msgs::msg::State current_state_;
    mavros_msgs::msg::ExtendedState extended_state_;
    geometry_msgs::msg::Pose target_pos_;
    double target_yaw_;
    double takeoff_height_;
    geometry_msgs::msg::PoseStamped local_position_, home_pose_, takeoff_pose_, loiter_pos_;
    nav_msgs::msg::Odometry cur_odom_;
    geometry_msgs::msg::TwistStamped local_vel_;
    mavros_msgs::srv::SetMode_Request offbset_mode_;
    mavros_msgs::srv::CommandBool_Request arm_cmd_;
    mavros_msgs::msg::PositionTarget local_raw_;
    mavros_msgs::msg::AttitudeTarget att_raw_;
    quadrotor_msgs::msg::PositionCommand quad_command_;
    std_msgs::msg::String state_uav_;

    /*ros utils*/
    // ros::NodeHandle node_;
    rclcpp::TimerBase::SharedPtr exec_timer_;
    rclcpp::Time time_quad_cmd_;
    rclcpp::Time time_mission_;
    // ros::Subscriber state_sub_, extent_state_sub_, odom_sub_, local_velocity_sub_, joy_sub_, quad_cmd_sub_,imu_sub_;
    // ros::Publisher local_pos_pub_, local_att_pub_, local_pos_raw_pub_, marker_pub_, local_vel_pub_, state_pub_;
    // ros::ServiceClient arming_client_, setmode_client_;

    rclcpp::Publisher<mavros_msgs::msg::PositionTarget>::SharedPtr local_pos_raw_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr local_pos_pub_;
	rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
	rclcpp::Publisher<mavros_msgs::msg::AttitudeTarget>::SharedPtr local_att_pub_;
	rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_pub_;

	rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr state_sub_;
	rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
	rclcpp::Subscription<mavros_msgs::msg::ExtendedState>::SharedPtr extent_state_sub_;
	rclcpp::Subscription<std_msgs::msg::String>::SharedPtr joy_sub_;
    rclcpp::Subscription<quadrotor_msgs::msg::PositionCommand>::SharedPtr quad_cmd_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;

    rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedPtr arming_client_;
    rclcpp::Client<mavros_msgs::srv::SetMode>::SharedPtr setmode_client_;


    /*return value : std::pair <times of the same state be continuously called, current continuously called state>*/
    void changeFSMExecState(FSM_EXEC_STATE new_state, string pos_call);
    std::pair<int, TrajFSM::FSM_EXEC_STATE > timesOfConsecutiveStateCalls();
    void printFSMExecState();
    void pubFSMExecState();
    /* ROS FUNCTIONS */
    void execFSMCallback();

    void stateCallback(const mavros_msgs::msg::State::UniquePtr msg);
    void extendedStateCallback(const mavros_msgs::msg::ExtendedState::UniquePtr msg);
    void odomCallback(const nav_msgs::msg::Odometry::UniquePtr msg);
    void joyCallback(const std_msgs::msg::String::UniquePtr str);
    void quadCmdCallback(const quadrotor_msgs::msg::PositionCommand::UniquePtr msg);

    bool reached_target_position(geometry_msgs::msg::Point tar_pos, geometry_msgs::msg::Point cur_pos, double thres=0.2) {
            Eigen::Vector3d tar(tar_pos.x, tar_pos.y, tar_pos.z);
            Eigen::Vector3d cur(cur_pos.x, cur_pos.y, cur_pos.z);

            return ((tar - cur).norm() < thres);
        }

    void execMission();

public:
	TrajFSM();
    ~TrajFSM();

    // void init();

};

#endif
