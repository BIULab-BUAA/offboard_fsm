/*
 * @Author: xindong324
 * @Date: 2022-03-04 09:21:26
 * @LastEditors: xindong324
 * @LastEditTime: 2022-03-06 18:36:06
 * @Description: file content
 */
#include <offboard_sample/pos_controller.h>
#include <geometry_msgs/Pose.h>

PosController::PosController(ros::NodeHandle &nh)
{
    nh.param("pos_ctrl/pid_pos_xy_p", pid_pos_xy.p_, 0.95);
    nh.param("pos_ctrl/pid_pos_xy_i", pid_pos_xy.i_, 0.0);
    nh.param("pos_ctrl/pid_pos_xy_d", pid_pos_xy.d_, 0.0);
    nh.param("pos_ctrl/pid_pos_xy_ilim", pid_pos_xy.iLim_, 0.0);

    nh.param("pos_ctrl/pid_pos_z_p", pid_pos_z.p_, 1.0);
    nh.param("pos_ctrl/pid_pos_z_i", pid_pos_z.i_, 0.0);
    nh.param("pos_ctrl/pid_pos_z_d", pid_pos_z.d_, 0.0);
    nh.param("pos_ctrl/pid_pos_z_ilim", pid_pos_z.iLim_, 0.0);

    nh.param("pos_ctrl/pid_vel_xy_p", pid_vel_xy.p_, 0.9);
    nh.param("pos_ctrl/pid_vel_xy_i", pid_vel_xy.i_, 0.4);
    nh.param("pos_ctrl/pid_vel_xy_d", pid_vel_xy.d_, 0.3);
    nh.param("pos_ctrl/pid_vel_xy_ilim", pid_vel_xy.iLim_, 100.0);

    nh.param("pos_ctrl/pid_vel_z_p", pid_vel_z.p_, 4.0);
    nh.param("pos_ctrl/pid_vel_z_i", pid_vel_z.i_, 2.0);
    nh.param("pos_ctrl/pid_vel_z_d", pid_vel_z.d_, 0.0);
    nh.param("pos_ctrl/pid_vel_z_ilim", pid_vel_z.iLim_, 100.0);

    nh.param("pos_ctrl/tar_x", target_pos_(0), 0.0);
    nh.param("pos_ctrl/tar_y", target_pos_(1), 0.0);
    nh.param("pos_ctrl/tar_z", target_pos_(2), 2.0);
    nh.param("pos_ctrl/tar_yaw", target_yaw_, 0.0);

    nh.param("pos_ctrl/min_thrust", min_thrust_, 0.1);
    nh.param("pos_ctrl/max_thrust",max_thrust_, 0.9);

    nh.param("pos_ctrl/max_xy_acc", max_xy_acc_, 5.0);
    nh.param("pos_ctrl/max_xy_vel", max_xy_vel_, 5.0);
    nh.param("pos_ctrl/max_z_vel", max_z_vel_, 3.0);

    std::cout << "max_xy_vel: "<< max_xy_vel_ << std::endl;

    pid_pos_xy.pid_zero();
    pid_pos_z.pid_zero();
    pid_vel_xy.pid_zero();
    pid_vel_z.pid_zero();

    flag_control_yaw_ = false;

    middle_throttle_ = 0.5;
}

void PosController::set_middle_throttle(double mid_thr) {
    middle_throttle_ = mid_thr;
}

bool PosController::reached_target_position(geometry_msgs::Point tar_pos, geometry_msgs::Point cur_pos, double thres) {
    Eigen::Vector3d tar(tar_pos.x, tar_pos.y, tar_pos.z);
    Eigen::Vector3d cur(cur_pos.x, cur_pos.y, cur_pos.z);

    return ((tar - cur).norm() < thres);
}

void PosController::set_target_pos(double tar_x, double tar_y, double tar_z)
{
    target_pos_(0) = tar_x;
    target_pos_(1) = tar_y;
    target_pos_(2) = tar_z;
}

void PosController::set_target_pos_yaw(double tar_x, double tar_y, double tar_z, double tar_yaw)
{
    set_target_pos(tar_x, tar_y, tar_z);
    target_yaw_ = tar_yaw;//tf::createQuaternionFromYaw(tar_yaw);
}

void PosController::update_current_pose_(geometry_msgs::Pose cur_pose){
    tf::Quaternion quat;

    cur_pos_(0) = cur_pose.position.x;
    cur_pos_(1) = cur_pose.position.y;
    cur_pos_(2) = cur_pose.position.z;

    cur_att_quat_ = cur_pose.orientation;
    tf::quaternionMsgToTF(cur_att_quat_, quat);
    tf::Matrix3x3(quat).getRPY(cur_att_eular_(0), cur_att_eular_(1), cur_att_eular_(2)); // roll, pitch, yaw
}

void PosController::update_local_vel_(geometry_msgs::Twist cur_vel)
{
    cur_vel_(0) = cur_vel.linear.x;
    cur_vel_(1) = cur_vel.linear.y;
    cur_vel_(2) = cur_vel.linear.z;
}

void PosController::pos_xy_controller(double dT) {
    Eigen::Vector3d err_pos = target_pos_ - cur_pos_;
    target_vel_(0) = pid_pos_xy.pid_apply(err_pos(0), dT);
    target_vel_(1) = pid_pos_xy.pid_apply(err_pos(1), dT);
    //target_vel_(2) = pid_pos_z.pid_apply(err_pos(2), dT);

    vel_xy_controller(dT);
}

void PosController::vel_xy_controller(double dT) {
    Eigen::Vector3d err_vel = target_vel_ - cur_vel_;
    target_acc_(0) = pid_vel_xy.pid_apply(err_vel(0), dT);
    target_acc_(1) = pid_vel_xy.pid_apply(err_vel(1), dT);
    //target_acc_(2) = pid_vel_z(err_vel(2), dT);
    target_acc_(0) = PID::boundf(target_acc_(0), -max_xy_acc_, max_xy_acc_);
    target_acc_(1) = PID::boundf(target_acc_(1), -max_xy_acc_, max_xy_acc_);

    accel_controller();
}

void PosController::accel_controller() {
    double cos_yaw = cos(cur_att_eular_(2));
    double sin_yaw = cos(cur_att_eular_(2));

    double desired_accel_forward = target_acc_(0) * cos_yaw + target_acc_(1) * sin_yaw;
    double desired_accel_left = -target_acc_(0) * sin_yaw + target_acc_(1) * cos_yaw;

    double desired_pitch = atan(desired_accel_forward / CONSTANT_ONE_GRAVITY);
    double desired_roll = -atan(desired_accel_left * cos(desired_pitch) / CONSTANT_ONE_GRAVITY);
    double desired_yaw = 0.0;
    if(flag_control_yaw_)
    {
        desired_yaw = target_yaw_;
    }else 
        desired_yaw = cur_att_eular_(2);
    geometry_msgs::Quaternion desired_att_quat = tf::createQuaternionMsgFromRollPitchYaw(desired_roll,desired_pitch,desired_yaw);
    att_cmd_.orientation = desired_att_quat;
}

void PosController::pos_z_controller(double dT)
{
    double err_z = target_pos_(2) - cur_pos_(2);
    target_vel_(2) = pid_pos_z.pid_apply(err_z, dT);
    if(fabs(target_vel_(2)) > max_z_vel_){
        target_vel_(2) *= target_vel_(2) / fabs(max_z_vel_);
    }

    vel_z_controller(dT);
}

void PosController::vel_z_controller(double dT)
{
    double err_vel = target_vel_(2) - cur_vel_(2);
    target_acc_(2) = pid_vel_z.pid_apply(err_vel, dT);

    /***************** get thrust althold demand ************************/
    double cos_roll = cos(cur_att_eular_(0));
    double cos_pitch = cos(cur_att_eular_(1));

    double alt_thr = middle_throttle_ + target_acc_(2) / (cos_roll * cos_pitch);

    att_cmd_.thrust = PID::boundf(alt_thr, min_thrust_, max_thrust_);
}


mavros_msgs::AttitudeTarget PosController::update_control_val(geometry_msgs::Pose cur_pose, geometry_msgs::Twist local_vel, double dT)
{
    flag_control_yaw_ = false;
    update_current_pose_(cur_pose);
    update_local_vel_(local_vel);
    pos_z_controller(dT);
    pos_xy_controller(dT);
    return get_att_target();
}

mavros_msgs::AttitudeTarget PosController::update_control_val(geometry_msgs::Point tar_pos, geometry_msgs::Pose cur_pose, geometry_msgs::Twist local_vel, double dT)
{
    flag_control_yaw_ = false;
    set_target_pos(tar_pos.x,tar_pos.y,tar_pos.z);
    update_current_pose_(cur_pose);
    update_local_vel_(local_vel);
    pos_z_controller(dT);
    pos_xy_controller(dT);
    return get_att_target();
}


mavros_msgs::AttitudeTarget PosController::update_control_val(geometry_msgs::Point tar_pos, double tar_yaw,geometry_msgs::Pose cur_pose, geometry_msgs::Twist local_vel, double dT)
{
    flag_control_yaw_ = true;
    set_target_pos_yaw(tar_pos.x,tar_pos.y,tar_pos.z,tar_yaw);
    update_current_pose_(cur_pose);
    update_local_vel_(local_vel);
    pos_z_controller(dT);

    pos_xy_controller(dT);
    return get_att_target();
}