/*
 * @Author: xindong324
 * @Date: 2022-03-06 00:18:29
 * @LastEditors: xindong324 xindong324@163.com
 * @LastEditTime: 2023-10-25 09:35:47
 * @Description: file content
 */
#include <ros/ros.h>
#include <offboard_sample/traj_fsm.h>
using namespace std;

int main(int argc, char **argv) {
    ros::init(argc, argv, "landing_node");
    ros::NodeHandle nh("~");
    ROS_INFO("start landing node");

    //OffboardFSM fsm;
    TrajFSM fsm;
    fsm.init(nh);
    ros::Duration(1.0).sleep();
    ros::spin();
    return 0;
}