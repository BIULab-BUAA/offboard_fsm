#include <offboard_sample/offboard_fsm.h>
#include <ros/ros.h>
using namespace std;

int main(int argc, char **argv) {
    ros::init(argc, argv, "landing_node");
    ros::NodeHandle nh;
    ROS_INFO("start landing node");

    OffboardFSM fsm;
    fsm.init(nh);
    ros::Duration(1.0).sleep();
    ros::spin();
    return 0;
}