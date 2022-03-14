/*
 * @Author: xindong324
 * @Date: 2022-03-06 20:27:40
 * @LastEditors: xindong324
 * @LastEditTime: 2022-03-13 23:21:16
 * @Description: file content
 */
#include "offboard_sample/darc_height_controller.h"

DARCHeightController::DARCHeightController(ros::NodeHandle &nh)
{
    nh.param("darc/k_i_z",k_i_z_, 1.0);
    //nh.param("darc/k_")
}

DARCHeightController::~DARCHeightController()
{
}