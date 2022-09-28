/*
 * @Author: xindong324
 * @Date: 2022-03-03 21:57:53
 * @LastEditors: xindong324
 * @LastEditTime: 2022-09-24 09:04:27
 * @Description: file content
 */
#include "offboard_sample/traj_fsm_att.h"

TrajFSMAtt::TrajFSMAtt(Parameter_t &param,LinearControl &contorller): param_(param), controller_(controller_)
{
}

void TrajFSMAtt::init(ros::NodeHandle &nh){
    /*fsm param*/
    nh.param("offb_fsm/flag_simulation", flag_simulation_, true);
    // nh.param("offb_fsm/target_x", target_pos_.position.x, 2.5);
    // nh.param("offb_fsm/target_y", target_pos_.position.y, 2.5);
    // nh.param("offb_fsm/target_z", target_pos_.position.z, 2.0);
    // nh.param("offb_fsm/target_yaw", target_yaw_, 0.0);
    nh.param("takeoff_height", takeoff_height_, 1.0);
    ROS_INFO("TEST");
    ROS_INFO("takeoff height: %lf", takeoff_height_);

    trigger_ = false;
    start_mission_ = false;
    flag_emergency_stop_ = false;
    has_quad_cmd_ = false;
    exec_state_ = INIT;

    /*init*/
    //pos_controller_.reset( new PosController(nh));

    exec_timer_ = nh.createTimer(ros::Duration(0.01),&TrajFSMAtt::execFSMCallback, this); // 50Hz;

    local_position_sub_ = nh.subscribe("/mavros/local_position/pose", 10, &TrajFSMAtt::positionCallback, this);
    local_velocity_sub_ = nh.subscribe("/mavros/local_position/velocity_local", 10, &TrajFSMAtt::localVelocityCallback, this);
    
    state_sub_ = nh.subscribe("/mavros/state", 10, &TrajFSMAtt::stateCallback, this);
    extent_state_sub_ = nh.subscribe("/mavros/extended_state", 10, &TrajFSMAtt::extendedStateCallback, this);
    joy_sub_ = nh.subscribe("/keys", 10, &TrajFSMAtt::joyCallback, this);
    quad_cmd_sub_ = nh.subscribe("pos_cmd", 10, &TrajFSMAtt::quadCmdCallback, this);

    odom_sub_ = nh.subscribe("/mavros/local_position/odom", 10, &TrajFSMAtt::odomCallback, this);
    imu_sub_ = nh.subscribe("/mavros/imu/data", 10, &TrajFSMAtt::imuCallback, this);


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

void TrajFSMAtt::positionCallback(const geometry_msgs::PoseStampedConstPtr &msg) {
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

void TrajFSMAtt::localVelocityCallback(const geometry_msgs::TwistStampedConstPtr &msg) {
    local_vel_ = *msg;
}

void TrajFSMAtt::stateCallback(const mavros_msgs::StateConstPtr &msg) {
    current_state_ = *msg;
}

void TrajFSMAtt::imuCallback(const sensor_msgs::ImuConstPtr &msg){
    imu_data_.msg = *msg;
    ros::Time now = ros::Time::now();
    imu_data_.rcv_stamp = now;

    imu_data_.w(0) = msg->angular_velocity.x;
    imu_data_.w(1) = msg->angular_velocity.y;
    imu_data_.w(2) = msg->angular_velocity.z;

    imu_data_.a(0) = msg->linear_acceleration.x;
    imu_data_.a(1) = msg->linear_acceleration.y;
    imu_data_.a(2) = msg->linear_acceleration.z;

    imu_data_.q.x() = msg->orientation.x;
    imu_data_.q.y() = msg->orientation.y;
    imu_data_.q.z() = msg->orientation.z;
    imu_data_.q.w() = msg->orientation.w;

    // check the frequency
    static int one_min_count = 9999;
    static ros::Time last_clear_count_time = ros::Time(0.0);
    
    if ( (now - last_clear_count_time).toSec() > 1.0 )
    {
        if ( one_min_count < 100 )
        {
            ROS_WARN("IMU frequency seems lower than 100Hz, which is too low!");
        }
        one_min_count = 0;
        last_clear_count_time = now;
    }
    one_min_count ++;
}

void TrajFSMAtt::odomCallback(const nav_msgs::OdometryConstPtr &msg)
{
    time_odom_ = ros::Time::now();
    has_odom_ = true;
    odom_data_.msg = *msg;
    odom_data_.p(0) = msg->pose.pose.position.x;
    odom_data_.p(1) = msg->pose.pose.position.y;
    odom_data_.p(2) = msg->pose.pose.position.z;

    odom_data_.v(0) = msg->twist.twist.linear.x;
    odom_data_.v(1) = msg->twist.twist.linear.y;
    odom_data_.v(2) = msg->twist.twist.linear.z;

    odom_data_.q.w() = msg->pose.pose.orientation.w;
    odom_data_.q.x() = msg->pose.pose.orientation.x;
    odom_data_.q.y() = msg->pose.pose.orientation.y;
    odom_data_.q.z() = msg->pose.pose.orientation.z;

    odom_data_.w(0) = msg->twist.twist.angular.x;
    odom_data_.w(1) = msg->twist.twist.angular.y;
    odom_data_.w(2) = msg->twist.twist.angular.z;

}

void TrajFSMAtt::extendedStateCallback(const mavros_msgs::ExtendedStateConstPtr &msg)
{
    extended_state_ = *msg;
}

void TrajFSMAtt::quadCmdCallback(const quadrotor_msgs::PositionCommandConstPtr &msg)
{
    quad_command_ = *msg;
    has_quad_cmd_ = true;

    cmd_data_.rcv_stamp = ros::Time::now();
    cmd_data_.msg = *msg;
    cmd_data_.p(0) = msg->position.x;
    cmd_data_.p(1) = msg->position.y;
    cmd_data_.p(2) = msg->position.z;

    cmd_data_.v(0) = msg->velocity.x;
    cmd_data_.v(1) = msg->velocity.y;
    cmd_data_.v(2) = msg->velocity.z;

    cmd_data_.a(0) = msg->acceleration.x;
    cmd_data_.a(1) = msg->acceleration.y;
    cmd_data_.a(2) = msg->acceleration.z;

    cmd_data_.j(0) = msg->jerk.x;
    cmd_data_.j(1) = msg->jerk.y;
    cmd_data_.j(2) = msg->jerk.z;

    // std::cout << "j1=" << j.transpose() << std::endl;

    cmd_data_.yaw = normalize_angle(msg->yaw);
    cmd_data_.yaw_rate = msg->yaw_dot;

    //static ros::Time time_last = ros::Time::now();
    
    time_quad_cmd_ = ros::Time::now();
}



// Keyboard control 
// A: start mission, take off
// d: land
// g: emergency stop
// w: start mission


void TrajFSMAtt::joyCallback(const std_msgs::StringConstPtr &str) {
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

void TrajFSMAtt::changeFSMExecState(TrajFSMAtt::FSM_EXEC_STATE new_state, string pos_call) {
    if(new_state == exec_state_)
    {
        continously_called_times_++;
    } else continously_called_times_ = 1;

    static string state_str[6] = {"INIT", "TAKEOFF", "LOITER","MISSION", "EMERGENCY_STOP", "LAND"};

    int pre_s = int(exec_state_);
    exec_state_ = new_state;
    cout << "[" + pos_call + "]: from" + state_str[pre_s] + "to" + state_str[int(new_state)] << endl;
}

std::pair<int, TrajFSMAtt::FSM_EXEC_STATE > TrajFSMAtt::timesOfConsecutiveStateCalls() {
    return std::pair<int, TrajFSMAtt::FSM_EXEC_STATE >(continously_called_times_, exec_state_);
}

void TrajFSMAtt::printFSMExecState() {
    static string state_str[6] = {"INIT", "TAKEOFF", "LOITER","MISSION", "EMERGENCY_STOP", "LAND"};
     cout << "[FSM]: state: " << state_str[int(exec_state_)] << endl;
}

void TrajFSMAtt::execFSMCallback(const ros::TimerEvent &e) {
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
            controller_.resetThrustMapping();
            local_pos_pub_.publish(home_pose_);
            // ROS_INFO("START INIT");
             offbset_mode_.request.custom_mode = "OFFBOARD";
             arm_cmd_.request.value = true;
            if(flag_simulation_)
            {
                // ROS_INFO("START INIT");
                if( current_state_.mode != "OFFBOARD")
                    setmode_client_.call(offbset_mode_);
                else ROS_INFO("OFFB ENABLE");
                for (int i = 0; i < 10 && ros::ok(); ++i) // wait for 0.1 seconds to allow mode change by FMU // mark
                {
                    ros::Duration(0.01).sleep();
                    ros::spinOnce();
                }
                if(!current_state_.armed)
                    arming_client_.call(arm_cmd_);
                else ROS_INFO("ARMED");
                
            }
            
            if(current_state_.armed && current_state_.mode == "OFFBOARD")
            { 
                takeoff_pose_ = local_position_;
                takeoff_pose_.pose.position.z = local_position_.pose.position.z + takeoff_height_;

                home_pose_ = local_position_;
                cout<< "percentage: "<< param_.thr_map.hover_percentage<< endl; 
                controller_.resetThrustMapping();
                changeFSMExecState(TAKEOFF, "FSM");
                
            }
                
            ros::spinOnce();    

            break;
        }

        case TAKEOFF:
        {

            local_pos_pub_.publish(takeoff_pose_);
            
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

void TrajFSMAtt::execMission() {
    // judge if rcv timeout
    if((ros::Time::now() - time_quad_cmd_).toSec() > 0.5)
    {
        has_quad_cmd_ = false;
        return;
    }
    //if(!has_quad_cmd_) return;
    ros::Time now_time = ros::Time::now();
    Controller_Output_t u;
    Desired_State_t des(odom_data_);

    controller_.estimateThrustModel(imu_data_.a, param_);
    des = get_cmd_des();
    controller_.calculateControl(des, odom_data_, imu_data_, u);

    // step4: publish control commands to mavros
    if(param_.use_bodyrate_ctrl)
    {
        publish_bodyrate_ctrl(u, now_time);
    }
    else{
        publish_attitude_ctrl(u, now_time);
    }
    
}

Desired_State_t TrajFSMAtt::get_cmd_des()
{
	Desired_State_t des;

	des.p = cmd_data_.p;
	des.v = cmd_data_.v;
	des.a = cmd_data_.a;
	des.j = cmd_data_.j;
	des.yaw = cmd_data_.yaw;
	des.yaw_rate = cmd_data_.yaw_rate;

	return des;
}


void TrajFSMAtt::publish_bodyrate_ctrl(const Controller_Output_t &u, const ros::Time &stamp)
{
	mavros_msgs::AttitudeTarget msg;

	msg.header.stamp = stamp;
	msg.header.frame_id = std::string("FCU");

	msg.type_mask = mavros_msgs::AttitudeTarget::IGNORE_ATTITUDE;

	msg.body_rate.x = u.bodyrates.x();
	msg.body_rate.y = u.bodyrates.y();
	msg.body_rate.z = u.bodyrates.z();

	msg.thrust = u.thrust;
    
    
	local_att_pub_.publish(msg);
}

void TrajFSMAtt::publish_attitude_ctrl(const Controller_Output_t &u, const ros::Time &stamp)
{
	mavros_msgs::AttitudeTarget msg;

	msg.header.stamp = stamp;
	msg.header.frame_id = std::string("FCU");

	msg.type_mask = mavros_msgs::AttitudeTarget::IGNORE_ROLL_RATE |
					mavros_msgs::AttitudeTarget::IGNORE_PITCH_RATE |
					mavros_msgs::AttitudeTarget::IGNORE_YAW_RATE;

	msg.orientation.x = u.q.x();
	msg.orientation.y = u.q.y();
	msg.orientation.z = u.q.z();
	msg.orientation.w = u.q.w();

	msg.thrust = u.thrust;

	local_att_pub_.publish(msg);
}


double TrajFSMAtt::normalize_angle(double a) {
    int cnt = 0;
    while (true) {
        cnt++;

        if (a < -M_PI) {
            a += M_PI * 2.0;
        } else if (a > M_PI) {
            a -= M_PI * 2.0;
        }

        if (-M_PI <= a && a <= M_PI) {
            break;
        };

        assert(cnt < 10 && "[uav_utils/geometry_msgs] INVALID INPUT ANGLE");
    }

    return a;
}

bool TrajFSMAtt::reached_target_position(geometry_msgs::Point tar_pos, geometry_msgs::Point cur_pos, double thres) {
    Eigen::Vector3d tar(tar_pos.x, tar_pos.y, tar_pos.z);
    Eigen::Vector3d cur(cur_pos.x, cur_pos.y, cur_pos.z);

    return ((tar - cur).norm() < thres);
}