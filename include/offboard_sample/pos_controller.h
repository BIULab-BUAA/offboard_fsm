/*
 * @Author: xindong324
 * @Date: 2022-03-04 09:21:26
 * @LastEditors: xindong324
 * @LastEditTime: 2022-03-04 09:57:15
 * @Description: file content
 */
#ifndef _POS_CONTROLLER__H
#define _POS_CONTROLLER__H

#include <Eigen/Eigen>
#include <iostream>
#include <offboard_sample/pid.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <mavros_msgs/AttitudeTarget.h>
#include <tf/tf.h>
#include <tf/transform_datatypes.h>

#define CONSTANT_ONE_GRAVITY 9.81

class PosController
{
public:
	PosController(ros::NodeHandle &nh);
	~PosController(){};

	void set_target_pos(double tar_x, double tar_y, double tar_z);

	void set_target_pos_yaw(double tar_x, double tar_y, double tar_z, double tar_yaw);

	void pos_xy_controller(double dT);

	void vel_xy_controller(double dT);
	void accel_controller();

	void pos_z_controller(double dT);
	void vel_z_controller(double dT);
    //
	void update_current_pose_(geometry_msgs::Pose cur_pose);
    void update_local_vel_(geometry_msgs::Twist cur_vel);

	mavros_msgs::AttitudeTarget update_control_val(geometry_msgs::Pose cur_pose, geometry_msgs::Twist local_vel, double dT);
    mavros_msgs::AttitudeTarget update_control_val(geometry_msgs::Point tar_pos, double tar_yaw,geometry_msgs::Pose cur_pose, geometry_msgs::Twist local_vel, double dT);
    mavros_msgs::AttitudeTarget update_control_val(geometry_msgs::Point tar_pos, geometry_msgs::Pose cur_pose, geometry_msgs::Twist local_vel, double dT);


    mavros_msgs::AttitudeTarget get_att_target() {return att_cmd_;}

    void set_middle_throttle(double mid_thr);

	void set_min_max_thrust(double min_thr, double max_thr){min_thrust_ = min_thr, max_thrust_ = max_thr;}
    static bool reached_target_position(geometry_msgs::Point tar_pos, geometry_msgs::Point cur_pos, double thres = 0.2);
private: 
	Eigen::Vector3d target_pos_;

	Eigen::Vector3d target_vel_;

    Eigen::Vector3d target_acc_;

	double target_yaw_;

	double middle_throttle_;

	Eigen::Vector3d cur_pos_;
	Eigen::Vector3d cur_vel_;
	Eigen::Vector3d cur_att_eular_;

	geometry_msgs::Quaternion cur_att_quat_;

	double min_thrust_;
	double max_thrust_;
	double max_xy_vel_;
	double max_z_vel_;
	double max_xy_acc_;

	bool flag_control_yaw_;

	mavros_msgs::AttitudeTarget att_cmd_;
	
	PID pid_pos_xy;
	PID pid_vel_xy;
	PID pid_pos_z;
	PID pid_vel_z;

public:
    typedef std::unique_ptr<PosController> Ptr;
};

#endif