/*
 * @Author: xindong324
 * @Date: 2022-03-06 00:18:29
 * @LastEditors: xindong324
 * @LastEditTime: 2022-09-20 23:08:55
 * @Description: file content
 */
#include <offboard_sample/offboard_fsm.h>
#include <ros/ros.h>
#include <signal.h>
#include <offboard_sample/PX4CtrlParam.h>

#include <offboard_sample/traj_fsm_att.h>

using namespace std;

void mySigintHandler(int sig)
{
    ROS_INFO("[px4Ctrl] exit...");
    ros::shutdown();
}


int main(int argc, char **argv) {
    ros::init(argc, argv, "offboard_att");
    ros::NodeHandle nh("~");

    signal(SIGINT, mySigintHandler);
    ros::Duration(1.0).sleep();
    cout << "sigint" << endl;

    Parameter_t param;
    param.config_from_ros_handle(nh);

    LinearControl controller(param);
    cout<< "bf" <<endl;
    
    TrajFSMAtt fsm = TrajFSMAtt(param, controller);
    cout<< "construct" <<endl;
    fsm.init(nh);

    ros::Duration(1.0).sleep();
    ros::spin();

    return 0;
}