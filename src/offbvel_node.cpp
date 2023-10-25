/*
 * @Author: xindong324
 * @Date: 2022-03-06 00:18:29
 * @LastEditors: xindong324
 * @LastEditTime: 2022-05-26 12:49:23
 * @Description: file content
 */
#include <offboard_sample/offboard_fsm.h>
#include <ros/ros.h>
#include <offboard_sample/vel_fsm_test.h>
using namespace std;

int main(int argc, char **argv) {
    ros::init(argc, argv, "landing_node");
    ros::NodeHandle nh("~");
    ROS_INFO("start landing node");

    //OffboardFSM fsm;
    VelFSM fsm;
    fsm.init(nh);
    ros::Duration(1.0).sleep();
    ros::spin();
    return 0;
}