/*
 * @Author: xindong324
 * @Date: 2022-03-06 20:27:40
 * @LastEditors: xindong324
 * @LastEditTime: 2022-03-22 23:50:23
 * @Description: file content
 */
#include "offboard_sample/darc_height_controller.h"

DARCHeightController::DARCHeightController(ros::NodeHandle &nh, double dt)
{

    double tau_z = 0.2;
    double tau_z_dot = 0.05;
    nh.param("darc/k_i_z",k_i_z_, 1.0);
    nh.param("darc/pos_target_z", pos_target_z_, 0.0);
    nh.param("darc/tau_z", tau_z, 0.19);
    nh.param("darc/tau_z_dot",tau_z_dot, 0.05);

    nh.param("darc/k_fz", k_fz_, 0.0165);
    nh.param("darc/v_z", v_z_, 2.5115);
    nh.param("darc/mass", mass_, 0.0125);

    nh.param("darc/theta_hat_z2", theta_hat_z_(2), 0.001);
    nh.param("darc/theta_hat_z0_min",theta_hat_z_min_(0), 0.011);
    nh.param("darc/theta_hat_z2_min", theta_hat_z_min_(2), -0.049);
    nh.param("darc/theta_hat_z0_max", theta_hat_z_max_(0), 0.018);
    nh.param("darc/theta_hat_z2_max", theta_hat_z_max_(2),0.049);
    


    

    time_constant_ = dt;
    alpha_z_target_ = dt / (tau_z + dt);
    alpha_z_dot_target_ = dt / (tau_z_dot + dt);



    //nh.param("darc/k_")
}

DARCHeightController::~DARCHeightController()
{
}