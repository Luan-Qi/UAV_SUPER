#include "subscribe.h"


RC_Data_t::RC_Data_t()
{
    rcv_stamp = ros::Time(0);

    last_mode = -1.0;
    last_gear = -1.0;

    // Parameter initilation is very important in RC-Free usage!
    is_armed = false;
    is_offboard_mode = true;
    enter_hover_mode = false;
    is_command_mode = true;
    enter_command_mode = false;
    for (int i = 0; i < 4; ++i)
    {
        ch[i] = 0.0;
    }
}

void RC_Data_t::feed(mavros_msgs::RCInConstPtr pMsg)
{
    msg = *pMsg;
    rcv_stamp = ros::Time::now();

    for (int i = 0; i < 4; i++)
    {
        ch[i] = ((double)msg.channels[i] - 1500.0) / 500.0;
        if (ch[i] > DEAD_ZONE)
            ch[i] = (ch[i] - DEAD_ZONE) / (1 - DEAD_ZONE);
        else if (ch[i] < -DEAD_ZONE)
            ch[i] = (ch[i] + DEAD_ZONE) / (1 - DEAD_ZONE);
        else
            ch[i] = 0.0;
    }

    arm = (int)msg.channels[4];
    mode = (int)msg.channels[5];
    gear = (int)msg.channels[6];

    check_validity();

    if (!have_init_last_mode)
    {
        have_init_last_mode = true;
        last_mode = mode;
    }
    if (!have_init_last_gear)
    {
        have_init_last_gear = true;
        last_gear = gear;
    }

    if (last_arm < ARM_THRESHOLD_VALUE && arm > ARM_THRESHOLD_VALUE)
        ROS_INFO("[PX4CTRL] Armed.");
    else if(last_arm > ARM_THRESHOLD_VALUE && arm < ARM_THRESHOLD_VALUE)
        ROS_INFO("[PX4CTRL] Disarmed.");

    if (arm > ARM_THRESHOLD_VALUE)
        is_armed = true;
    else
        is_armed = false;

    // 1
    if (last_mode < API_MODE_THRESHOLD_VALUE && mode > API_MODE_THRESHOLD_VALUE){
        enter_hover_mode = true;
        ROS_INFO("[PX4CTRL] Enter offboard mode.");}
    else if(last_mode > API_MODE_THRESHOLD_VALUE && mode < API_MODE_THRESHOLD_VALUE){
        enter_hover_mode = false;
        if(is_command_mode) ROS_INFO("[PX4CTRL] Stop command send.");
        ROS_INFO("[PX4CTRL] Exit offboard mode.");}

    if (mode > API_MODE_THRESHOLD_VALUE)
        is_offboard_mode = true;
    else
        is_offboard_mode = false;

    // 2
    if (is_offboard_mode)
    {
        if (last_gear < GEAR_SHIFT_UP_THRESHOLD && gear > GEAR_SHIFT_UP_THRESHOLD){
            enter_command_mode = true;
            ROS_INFO("[PX4CTRL] Start command send");}
        else if (last_gear > GEAR_SHIFT_UP_THRESHOLD && gear < GEAR_SHIFT_UP_THRESHOLD){
            enter_command_mode = false;
            ROS_INFO("[PX4CTRL] Stop command send");}

        if (gear > GEAR_SHIFT_UP_THRESHOLD){
            is_command_mode = true;
            is_hold_mode = false;}
        else if (gear > GEAR_SHIFT_DOWN_THRESHOLD && gear < GEAR_SHIFT_UP_THRESHOLD){
            is_command_mode = false;
            is_hold_mode = true;}
        else{
            is_command_mode = false;
            is_hold_mode = false;}
    }
    else
    {
        is_command_mode = false;
    }

    last_arm = arm;
    last_mode = mode;
    last_gear = gear;
}

void RC_Data_t::check_validity()
{
    if (mode >= 900 && mode <= 2100 && gear >= 900 && gear <= 2100)
    {
        // pass
    }
    else
    {
        ROS_ERROR("RC data validity check fail. mode=%d, gear=%d", mode, gear);
    }
}

bool RC_Data_t::check_centered()
{
    bool centered = abs(ch[0]) < 1e-5 && abs(ch[0]) < 1e-5 && abs(ch[0]) < 1e-5 && abs(ch[0]) < 1e-5;
    return centered;
}

bool RC_Data_t::is_received(const ros::Time &now_time)
{
	return (now_time - this->rcv_stamp).toSec() < 0.5;
}


State_Data_t::State_Data_t()
{
    rcv_stamp = ros::Time(0);
}

void State_Data_t::feed(mavros_msgs::StateConstPtr pMsg)
{
    current_state = *pMsg;
    rcv_stamp = ros::Time::now();
}

bool State_Data_t::is_received(const ros::Time &now_time)
{
	return (now_time - this->rcv_stamp).toSec() < 0.5;
}


Battery_Data_t::Battery_Data_t()
{
    rcv_stamp = ros::Time(0);
}

void Battery_Data_t::feed(mavros_msgs::BatteryStatusConstPtr pMsg)
{
    battery = *pMsg;
    rcv_stamp = ros::Time::now();
}

bool Battery_Data_t::is_received(const ros::Time &now_time)
{
    return (now_time - this->rcv_stamp).toSec() < 0.5;
}


Command_Data_t::Command_Data_t()
{
    rcv_stamp = ros::Time(0);
}

void Command_Data_t::feed(quadrotor_msgs::PositionCommandConstPtr pMsg)
{

    msg = *pMsg;
    rcv_stamp = ros::Time::now();

    p(0) = msg.position.x;
    p(1) = msg.position.y;
    p(2) = msg.position.z;

    v(0) = msg.velocity.x;
    v(1) = msg.velocity.y;
    v(2) = msg.velocity.z;

    a(0) = msg.acceleration.x;
    a(1) = msg.acceleration.y;
    a(2) = msg.acceleration.z;

    j(0) = msg.jerk.x;
    j(1) = msg.jerk.y;
    j(2) = msg.jerk.z;

    // std::cout << "j1=" << j.transpose() << std::endl;

    yaw = msg.yaw;
    yaw_rate = msg.yaw_dot;
}

bool Command_Data_t::is_received(const ros::Time &now_time)
{
	return (now_time - this->rcv_stamp).toSec() < 0.5;
}


Odom_Data_t::Odom_Data_t()
{
    rcv_stamp = ros::Time(0);
    q.setIdentity();
    recv_new_msg = false;
};

void Odom_Data_t::feed(nav_msgs::OdometryConstPtr pMsg)
{
    ros::Time now = ros::Time::now();

    msg = *pMsg;
    rcv_stamp = now;
    recv_new_msg = true;

    p(0) = msg.pose.pose.position.x;
    p(1) = msg.pose.pose.position.y;
    p(2) = msg.pose.pose.position.z;

    v(0) = msg.twist.twist.linear.x;
    v(1) = msg.twist.twist.linear.y;
    v(2) = msg.twist.twist.linear.z;

    q.w() = msg.pose.pose.orientation.w;
    q.x() = msg.pose.pose.orientation.x;
    q.y() = msg.pose.pose.orientation.y;
    q.z() = msg.pose.pose.orientation.z;
}

bool Odom_Data_t::is_received(const ros::Time &now_time)
{
	return (now_time - this->rcv_stamp).toSec() < 0.5;
}

