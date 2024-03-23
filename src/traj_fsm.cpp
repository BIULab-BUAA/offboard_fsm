#include "rclcpp/rclcpp.hpp"

#include "offboard_sample/traj_fsm.h"

TrajFSM::TrajFSM():Node("offb_fsm")
{
     this->declare_parameter<bool>("flag_simulation", true); // 声明参数及其默认值
     this->declare_parameter<double>("takeoff_height", 1.0); // 声明参数及其默认值
     this->declare_parameter<bool>("use_telem", false); // 声明参数及其默认值
     this->declare_parameter<double>("target_yaw", 0.0); // 声明参数及其默认值
    
    this->get_parameter("flag_simulation", flag_simulation_);
    this->get_parameter("takeoff_height", takeoff_height_);
    this->get_parameter("target_yaw", target_yaw_);
    this->get_parameter("use_telem", use_telem_);

    std::cout << "flag sim" << flag_simulation_ << std::endl;
    std::cout << "takeoff_height" << takeoff_height_ << std::endl;
    std::cout << "flag sim" << flag_simulation_ << std::endl;

    RCLCPP_INFO(this->get_logger(), "TEST");


    // init flag
    trigger_ = false;
    start_mission_ = false;
    flag_emergency_stop_ = false;
    has_quad_cmd_ = false;
    exec_state_ = INIT;

     /*init*/
    //pos_controller_.reset( new PosController(nh));

   
    
    //sub
    rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
    auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);
    // rclcpp::QoS qos;
    // qos.reliability_level(rclcpp::ReliabilityLevel::BEST_EFFORT);

    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/mavros/local_position/odom", qos, std::bind(&TrajFSM::odomCallback, this, std::placeholders::_1));

    state_sub_ = this->create_subscription<mavros_msgs::msg::State>(
            "/mavros/state", qos, std::bind(&TrajFSM::stateCallback, this, std::placeholders::_1));
    
    extent_state_sub_ = this->create_subscription<mavros_msgs::msg::ExtendedState>(
            "/mavros/extended_state", qos, std::bind(&TrajFSM::extendedStateCallback, this, std::placeholders::_1));
    
    joy_sub_ = this->create_subscription<std_msgs::msg::String>(
            "/keys", qos, std::bind(&TrajFSM::joyCallback, this, std::placeholders::_1));
    
    quad_cmd_sub_ = this->create_subscription<quadrotor_msgs::msg::PositionCommand>(
            "pos_cmd", qos, std::bind(&TrajFSM::quadCmdCallback, this, std::placeholders::_1));
    
    /************ publisher ******************/
    marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("visualization_marker", 10);
    local_pos_pub_ =  this->create_publisher<geometry_msgs::msg::PoseStamped>("/mavros/setpoint_position/local", 10);
    local_pos_raw_pub_ = this->create_publisher<mavros_msgs::msg::PositionTarget>("/mavros/setpoint_raw/local", 10);
    local_att_pub_ = this->create_publisher<mavros_msgs::msg::AttitudeTarget>("/mavros/setpoint_raw/attitude", 10);
    state_pub_ = this->create_publisher<std_msgs::msg::String>("/state_uav", 10);

    arming_client_ = this->create_client<mavros_msgs::srv::CommandBool>("/mavros/cmd/arming");
    // landing_client_ = nh.serviceClient<mavros_msgs::CommandTOL>("/mavros/cmd/land");
    setmode_client_ = this->create_client<mavros_msgs::srv::SetMode>("/mavros/set_mode");


     exec_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(10), std::bind(&TrajFSM::execFSMCallback, this));

    home_pose_ = local_position_;
    takeoff_pose_ = local_position_;
    takeoff_pose_.pose.position.z += 1.0;
    loiter_pos_ = takeoff_pose_;

    att_raw_.type_mask =  0b00000111;
    local_raw_.coordinate_frame = mavros_msgs::msg::PositionTarget::FRAME_LOCAL_NED;
    rclcpp::WallRate loop_rate(10.0);
    while(rclcpp::ok() && current_state_.connected){
        // rclcpp::spin_some();
        //TODO: test this
        loop_rate.sleep();
        RCLCPP_INFO(this->get_logger(),"\rconnecting to FCU...");
    }
}

void TrajFSM::odomCallback(const nav_msgs::msg::Odometry::UniquePtr msg)
{
    cur_odom_ = *msg;
    local_position_.header = msg->header;
    local_position_.pose = msg->pose.pose;
    local_vel_.header = msg->header;
    local_vel_.twist = msg->twist.twist;

    // visualize 
    visualization_msgs::msg::Marker marker;
    uint32_t shape = visualization_msgs::msg::Marker::SPHERE;
    static int x_cor = 0;
    // Set the frame ID and timestamp.  See the TF tutorials for information on these.
    marker.header.frame_id = "/map";
    marker.header.stamp = this->get_clock()->now();

    marker.ns = "basic_shapes";
    marker.id = x_cor;

    marker.type = shape;

    marker.action = visualization_msgs::msg::Marker::ADD;

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
    marker.lifetime = rclcpp::Duration::from_seconds(20.0);
     x_cor++;
    marker_pub_->publish(marker);
}

void TrajFSM::stateCallback(const mavros_msgs::msg::State::UniquePtr msg)
{
    current_state_ = *msg;
}

void TrajFSM::extendedStateCallback(const mavros_msgs::msg::ExtendedState::UniquePtr msg)
{
    extended_state_  =  *msg;
}

void TrajFSM::joyCallback(const std_msgs::msg::String::UniquePtr str)
{
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

void TrajFSM::quadCmdCallback(const quadrotor_msgs::msg::PositionCommand::UniquePtr msg)
{
     quad_command_ = *msg;
    has_quad_cmd_ = true;
    time_quad_cmd_ = this->get_clock()->now();

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

void TrajFSM::pubFSMExecState() {
    static string state_str[6] = {"INIT", "TAKEOFF", "LOITER","MISSION", "EMERGENCY_STOP", "LAND"};
    cout << "[FSM]: state: " << state_str[int(exec_state_)] << endl;
    state_uav_.data = state_str[int(exec_state_)];
    state_pub_->publish(state_uav_);
}

void TrajFSM::execFSMCallback() 
{
    static  int fsm_num = 0;
    fsm_num++;
    if(fsm_num == 50)
    {
        pubFSMExecState();

        fsm_num = 0;
    }

    switch (exec_state_)
    {
        case INIT:
        {
            if(!trigger_){
                return;
            }

            local_pos_pub_->publish(home_pose_);
            //ROS_INFO("START INIT");

            auto request = std::make_shared<mavros_msgs::srv::SetMode_Request>();
            request->custom_mode= "OFFBOARD";
            auto arm_request = std::make_shared<mavros_msgs::srv::CommandBool_Request>();
            arm_request->value = true;
            if(flag_simulation_)
            {
                // ROS_INFO("START INIT");
                if( current_state_.mode != "OFFBOARD")
                 
                    setmode_client_->async_send_request(request);
                else RCLCPP_INFO(this->get_logger(),"OFFB ENABLE");
                if(!current_state_.armed)
                    arming_client_->async_send_request(arm_request);
                else RCLCPP_INFO(this->get_logger(),"ARMED"); 
            }
            
            if(current_state_.armed && current_state_.mode == "OFFBOARD")
            {
                takeoff_pose_ = local_position_;
                takeoff_pose_.pose.position.z = local_position_.pose.position.z + takeoff_height_;

                home_pose_ = local_position_;
                
                changeFSMExecState(TAKEOFF, "FSM");
                
            }

            break;
        }

        case TAKEOFF:
        {

            local_pos_pub_->publish(takeoff_pose_);
            if(reached_target_position(takeoff_pose_.pose.position, local_position_.pose.position))
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

            local_pos_pub_->publish(loiter_pos_);

            time_mission_ = this->get_clock()->now();

            if(has_quad_cmd_)
            {
                changeFSMExecState(MISSION, "FSM");
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
            att_raw_.orientation = geometry_msgs::msg::Quaternion();
            local_att_pub_->publish(att_raw_);
            break;
        }

        case LAND:
        {
            auto request = std::make_shared<mavros_msgs::srv::SetMode_Request>();
            request->custom_mode= "AUTO.LAND";
            // offbset_mode_.request.custom_mode = "AUTO.LAND";
            setmode_client_->async_send_request(request) ;
            
            if(extended_state_.landed_state == mavros_msgs::msg::ExtendedState::LANDED_STATE_ON_GROUND){
                trigger_ = false;
                changeFSMExecState(INIT, "FSM");
            }
            break;
        }

    }

}

void TrajFSM::execMission() {
    // judge if rcv timeout
    if((this->get_clock()->now() - time_quad_cmd_).seconds() > 0.5)
    {
        has_quad_cmd_ = false;
        return;
    }
    //if(!has_quad_cmd_) return;
    local_raw_.header.stamp = this->get_clock()->now();
    local_raw_.position = quad_command_.position;
    local_raw_.velocity = quad_command_.velocity;
    local_raw_.acceleration_or_force = quad_command_.acceleration;
    
    local_raw_.yaw = quad_command_.yaw;
    local_raw_.yaw_rate = quad_command_.yaw_dot;
    
    local_pos_raw_pub_->publish(local_raw_);
}

TrajFSM::~TrajFSM()
{
}



int main(int argc, char * argv[])                               
{
    rclcpp::init(argc, argv);                        
    rclcpp::spin(std::make_shared<TrajFSM>()); 
    rclcpp::shutdown();                               
    return 0;
}
