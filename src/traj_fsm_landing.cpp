/*
 * @Author: xindong324
 * @Date: 2022-03-03 21:57:53
 * @LastEditors: xindong324
 * @LastEditTime: 2022-09-19 20:32:48
 * @Description: file content
 */
#include "offboard_sample/traj_fsm.h"

TrajFSM::TrajFSM()
{

}

TrajFSM::~TrajFSM()
{
}

void TrajFSM::init(ros::NodeHandle &nh) {
    /*fsm param*/
    nh.param("offb_fsm/flag_simulation", flag_simulation_, true);
    nh.param("offb_fsm/target_x", target_pos_.position.x, 2.5);
    nh.param("offb_fsm/target_y", target_pos_.position.y, 2.5);
    nh.param("offb_fsm/target_z", target_pos_.position.z, 2.0);
    nh.param("offb_fsm/target_yaw", target_yaw_, 0.0);
    nh.param("takeoff_height", takeoff_height_, 1.0);
    ROS_INFO("TEST");

    trigger_ = false;
    start_mission_ = false;
    flag_emergency_stop_ = false;
    has_quad_cmd_ = false;
    exec_state_ = INIT;

    /*init*/
    //pos_controller_.reset( new PosController(nh));

    exec_timer_ = nh.createTimer(ros::Duration(0.01),&TrajFSM::execFSMCallback, this); // 50Hz;

    local_position_sub_ = nh.subscribe("/mavros/local_position/pose", 10, &TrajFSM::positionCallback, this);
    local_velocity_sub_ = nh.subscribe("/mavros/local_position/velocity_local", 10, &TrajFSM::localVelocityCallback, this);
    
    state_sub_ = nh.subscribe("/mavros/state", 10, &TrajFSM::stateCallback, this);
    extent_state_sub_ = nh.subscribe("/mavros/extended_state", 10, &TrajFSM::extendedStateCallback, this);
    joy_sub_ = nh.subscribe("/keys", 10, &TrajFSM::joyCallback, this);
    quad_cmd_sub_ = nh.subscribe("pos_cmd", 10, &TrajFSM::quadCmdCallback, this);

    /************ publisher ******************/
    marker_pub_ = nh.advertise<visualization_msgs::Marker>("visualization_marker", 1);
    local_pos_pub_ =  nh.advertise<geometry_msgs::PoseStamped>("/mavros/setpoint_position/local", 10);
    local_pos_raw_pub_ = nh.advertise<mavros_msgs::PositionTarget>("/mavros/setpoint_raw/local", 10);
    local_att_pub_ = nh.advertise<mavros_msgs::AttitudeTarget>("/mavros/setpoint_raw/attitude", 10);

    arming_client_ = nh.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");
    landing_client_ = nh.serviceClient<mavros_msgs::CommandTOL>("/mavros/cmd/land");
    setmode_client_ = nh.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");


    home_pose_ = local_position_;
    takeoff_pose_ = local_position_;
    takeoff_pose_.pose.position.z += 1.0;
    loiter_pos_ = takeoff_pose_;

    att_raw_.type_mask = 0b00000111;

    local_raw_.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
    // local_raw_.type_mask = mavros_msgs::PositionTarget::IGNORE_VX | 
    //                           mavros_msgs::PositionTarget::IGNORE_VY |
    //                           mavros_msgs::PositionTarget::IGNORE_VZ |
    //                           mavros_msgs::PositionTarget::IGNORE_AFX |
    //                           mavros_msgs::PositionTarget::IGNORE_AFY |
    //                           mavros_msgs::PositionTarget::IGNORE_AFZ |
    //                           mavros_msgs::PositionTarget::IGNORE_YAW_RATE;

    ros::Rate rate(10);
    while(ros::ok() && current_state_.connected){
        ros::spinOnce();
        rate.sleep();
        ROS_INFO("\rconnecting to FCU...");
    }

}

void TrajFSM::positionCallback(const geometry_msgs::PoseStampedConstPtr &msg) {
    local_position_ = *msg;

    /****************pub marker*********************/
    visualization_msgs::Marker marker;
    uint32_t shape = visualization_msgs::Marker::SPHERE;
    static int x_cor = 0;
    // Set the frame ID and timestamp.  See the TF tutorials for information on these.
    marker.header.frame_id = "/map";
    marker.header.stamp = ros::Time::now();

    marker.ns = "basic_shapes";
    marker.id = x_cor;

    marker.type = shape;

    marker.action = visualization_msgs::Marker::ADD;

    marker.pose = local_position_.pose;

    // Set the scale of the marker -- 1x1x1 here means 1m on a side
// %Tag(SCALE)%
    marker.scale.x =1;
    marker.scale.y = 1;
    marker.scale.z = 1;
// %EndTag(SCALE)%

    // Set the color -- be sure to set alpha to something non-zero!
// %Tag(COLOR)%
    marker.color.r = 1.0f;
    marker.color.g = 0.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0;
// %EndTag(COLOR)%

// %Tag(LIFETIME)%
    marker.lifetime = ros::Duration(20.0);
// %EndTag(LIFETIME)%
    x_cor++;
    marker_pub_.publish(marker);
}

void TrajFSM::localVelocityCallback(const geometry_msgs::TwistStampedConstPtr &msg) {
    local_vel_ = *msg;
}

void TrajFSM::stateCallback(const mavros_msgs::StateConstPtr &msg) {
    current_state_ = *msg;
}

void TrajFSM::extendedStateCallback(const mavros_msgs::ExtendedStateConstPtr &msg)
{
    extended_state_ = *msg;
}

void TrajFSM::quadCmdCallback(const quadrotor_msgs::PositionCommandConstPtr &msg)
{
    quad_command_ = *msg;
    has_quad_cmd_ = true;

    //static ros::Time time_last = ros::Time::now();
    // 
    
    time_quad_cmd_ = ros::Time::now();
}

// Keyboard control 
// A: start mission, take off
// d: land
// g: emergency stop
// w: start mission


void TrajFSM::joyCallback(const std_msgs::StringConstPtr &str) {
    if(str->data == "d" || str->data == "D")
    {
        trigger_ = false;
        changeFSMExecState(LAND, "JOY");
    }
    else if(str->data == "g" || str->data == "G")
    {// take off
        trigger_ = false;
        changeFSMExecState(EMERGENCY_STOP, "JOY");
        //changeFSMExecState(TAKEOFF, "JOY");
    }
    else if(str->data == "a" || str->data == "A")
    {// take off
        trigger_ = true;
        //changeFSMExecState(TAKEOFF, "JOY");
    }
    else if(str->data == "w" || str->data == "W")
    {
        start_mission_ = true;
    }
    else if(str->data == "j" || str->data == "J")
    {
        image_yaw_state_ = 1;
    }
    else if(str->data == "k" || str->data == "K")
    {
        image_yaw_state_ = 0;
    }

    std::cout<<"str.data : "<< str->data <<std::endl;
}

void TrajFSM::changeFSMExecState(TrajFSM::FSM_EXEC_STATE new_state, string pos_call) {
    if(new_state == exec_state_)
    {
        continously_called_times_++;
    } else continously_called_times_ = 1;

    static string state_str[6] = {"INIT", "TAKEOFF", "LOITER","MISSION", "EMERGENCY_STOP", "LAND"};

    int pre_s = int(exec_state_);
    exec_state_ = new_state;
    cout << "[" + pos_call + "]: from" + state_str[pre_s] + "to" + state_str[int(new_state)] << endl;
}

std::pair<int, TrajFSM::FSM_EXEC_STATE > TrajFSM::timesOfConsecutiveStateCalls() {
    return std::pair<int, TrajFSM::FSM_EXEC_STATE >(continously_called_times_, exec_state_);
}

void TrajFSM::printFSMExecState() {
    static string state_str[6] = {"INIT", "TAKEOFF", "LOITER","MISSION", "EMERGENCY_STOP", "LAND"};
     cout << "[FSM]: state: " << state_str[int(exec_state_)] << endl;
}

void TrajFSM::execFSMCallback(const ros::TimerEvent &e) {
    static  int fsm_num = 0;
    fsm_num++;
    if(fsm_num == 50)
    {
        printFSMExecState();

        fsm_num = 0;
    }

    switch (exec_state_)
    {
        case INIT:
        {
            if(!trigger_){
                return;
            }

            local_pos_pub_.publish(home_pose_);
            //ROS_INFO("START INIT");
             offbset_mode_.request.custom_mode = "OFFBOARD";
             arm_cmd_.request.value = true;
            if(flag_simulation_)
            {
                // ROS_INFO("START INIT");
                if( current_state_.mode != "OFFBOARD")
                    setmode_client_.call(offbset_mode_);
                else ROS_INFO("OFFB ENABLE");
                if(!current_state_.armed)
                    arming_client_.call(arm_cmd_);
                else ROS_INFO("ARMED");
                
            }
            
            if(current_state_.armed && current_state_.mode == "OFFBOARD")
            {
                takeoff_pose_ = local_position_;
                takeoff_pose_.pose.position.z = local_position_.pose.position.z + takeoff_height_;

                home_pose_ = local_position_;
                
                changeFSMExecState(TAKEOFF, "FSM");
                
            }
                
            ros::spinOnce();    

            break;
        }

        case TAKEOFF:
        {

            local_pos_pub_.publish(takeoff_pose_);
            if(PosController::reached_target_position(takeoff_pose_.pose.position, local_position_.pose.position))
            {
                changeFSMExecState(LOITER, "FSM");
            }
            //local_pos_pub_.publish(pose);
            break;
        }

        case LOITER:
        {
            if(timesOfConsecutiveStateCalls().first == 1)
            {
                loiter_pos_ = local_position_;
                changeFSMExecState(LOITER, "FSM");
            }

            // if(image_yaw_state_ == 1)
            // {
            //     loiter_pos_.pose.orientation = tf::createQuaternionMsgFromYaw(5.0*3.1415926/180.0);
            // }else loiter_pos_.pose.orientation = tf::createQuaternionMsgFromYaw(0);

            local_pos_pub_.publish(loiter_pos_);

            time_mission_ = ros::Time::now();

            if(has_quad_cmd_)
            {
                changeFSMExecState(MISSION, "FSM");
                //start_mission_ = false;
            }
            break;
        }

        case MISSION:
        {
            
            execMission();
            if(flag_emergency_stop_){
                has_quad_cmd_ = false;
                changeFSMExecState(EMERGENCY_STOP, "FSM");
            }

            if(!has_quad_cmd_){
                changeFSMExecState(LOITER, "FSM");
            }
            
            break;
        }

        case EMERGENCY_STOP:
        {
            
            att_raw_.thrust = 0;
            att_raw_.orientation = geometry_msgs::Quaternion();
            local_att_pub_.publish(att_raw_);
            break;
        }

        case LAND:
        {
            offbset_mode_.request.custom_mode = "AUTO.LAND";
            
            if( setmode_client_.call(offbset_mode_) && offbset_mode_.response.mode_sent)
            {                    
            }
            if(extended_state_.landed_state == mavros_msgs::ExtendedState::LANDED_STATE_ON_GROUND){
                trigger_ = false;
                changeFSMExecState(INIT, "FSM");
            }
            break;
        }

    }

    
}

void TrajFSM::execMission() {
    // judge if rcv timeout
    if((ros::Time::now() - time_quad_cmd_).toSec() > 0.5)
    {
        has_quad_cmd_ = false;
        return;
    }
    //if(!has_quad_cmd_) return;
    local_raw_.header.stamp = ros::Time::now();
    local_raw_.position = quad_command_.position;
    local_raw_.velocity = quad_command_.velocity;
    local_raw_.acceleration_or_force = quad_command_.acceleration;
    
    local_raw_.yaw = quad_command_.yaw;
    local_raw_.yaw_rate = quad_command_.yaw_dot;
    
    local_pos_raw_pub_.publish(local_raw_);
}

