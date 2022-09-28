#ifndef __INPUT_H
#define __INPUT_H

#include <ros/ros.h>
#include <Eigen/Dense>
#include <sensor_msgs/Imu.h>
#include <quadrotor_msgs/TakeoffLand.h>
#include <quadrotor_msgs/PositionCommand.h>
#include <mavros_msgs/RCIn.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/ExtendedState.h>
#include <sensor_msgs/BatteryState.h>
#include <nav_msgs/Odometry.h>
#include <offboard_sample/utils.h>
#include "PX4CtrlParam.h"

/**
 * @brief rec data
 * 
 */


class Odom_Data_t
{
private:
    /* data */
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    Eigen::Vector3d p;
    Eigen::Vector3d v;
    Eigen::Quaterniond q;
    Eigen::Vector3d w;

    nav_msgs::Odometry msg;
    ros::Time rcv_stamp;
    bool recv_new_msg;

    Odom_Data_t(/* args */);
    ~Odom_Data_t(){};
    void feed(nav_msgs::OdometryConstPtr pMsg);
};

class Imu_Data_t
{
private:
    /* data */
public:
    Eigen::Quaterniond q;
    Eigen::Vector3d w;
    Eigen::Vector3d a;

    sensor_msgs::Imu msg;
    ros::Time rcv_stamp;

    Imu_Data_t(/* args */);
    ~Imu_Data_t(){};
    void feed(sensor_msgs::ImuConstPtr pMsg);
};

class State_Data_t
{
private:
    /* data */
public:

    mavros_msgs::State current_state;
    mavros_msgs::State state_before_offboard;


    State_Data_t(/* args */);
    ~State_Data_t(){};
    void feed(mavros_msgs::StateConstPtr pMsg);
};

class ExtendedState_Data_t
{
private:
    /* data */
public:
    mavros_msgs::ExtendedState current_extended_state;

    ExtendedState_Data_t(/* args */);
    ~ExtendedState_Data_t(){};
    void feed(mavros_msgs::ExtendedStateConstPtr pMsg);
};

class Command_Data_t
{
private:
    /* data */
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    Eigen::Vector3d p;
    Eigen::Vector3d v;
    Eigen::Vector3d a;
    Eigen::Vector3d j;
    double yaw;
    double yaw_rate;

    quadrotor_msgs::PositionCommand msg;
    ros::Time rcv_stamp;

    Command_Data_t(/* args */);
    ~Command_Data_t(){};
    void feed(quadrotor_msgs::PositionCommandConstPtr pMsg);
};

class Battery_Data_t
{
private:
    /* data */
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    double volt{0.0};
    double percentage{0.0};

    sensor_msgs::BatteryState msg;
    ros::Time rcv_stamp;

    Battery_Data_t(/* args */);
    ~Battery_Data_t(){};
    void feed(sensor_msgs::BatteryStateConstPtr pMsg);
};

class Takeoff_Land_Data_t
{
private:
    /* data */
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    bool triggered{false};
    uint8_t takeoff_land_cmd;

    quadrotor_msgs::TakeoffLand msg;
    ros::Time rcv_stamp;

    Takeoff_Land_Data_t(/* args */);
    ~Takeoff_Land_Data_t(){};
    void feed(quadrotor_msgs::TakeoffLandConstPtr pMsg);
};





#endif
