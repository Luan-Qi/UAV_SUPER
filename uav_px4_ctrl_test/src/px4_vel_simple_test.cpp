#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <mavros_msgs/RCIn.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/BatteryStatus.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/CommandLong.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/ExtendedState.h>
#include <sensor_msgs/BatteryState.h>
#include <nav_msgs/Odometry.h>
#include <quadrotor_msgs/PositionCommand.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <signal.h>
#include "subscribe.h"


void uav_mavros_pose_fix(geometry_msgs::PoseStamped * pose)
{
    double temp_x = pose->pose.position.x;
    double temp_y = pose->pose.position.y;

    pose->pose.position.x = -temp_y;
    pose->pose.position.y = temp_x;

    tf2::Quaternion q_orig, q_rot, q_new;
    tf2::fromMsg(pose->pose.orientation, q_orig);
    q_rot.setRPY(0, 0, M_PI/2);  // 绕Z轴旋转180度
    q_new = q_rot * q_orig;    // 旋转叠加
    q_new.normalize();
    pose->pose.orientation = tf2::toMsg(q_new);
}

void uav_mavros_cmd_vel_fix(geometry_msgs::TwistStamped * vel)
{
    double temp_x = vel->twist.linear.x;
    double temp_y = vel->twist.linear.y;

    vel->twist.linear.x = -temp_y;
    vel->twist.linear.y = temp_x;
}

geometry_msgs::Quaternion yawToQuaternion(double yaw_rad) {
    tf2::Quaternion q;
    q.setRPY(0, 0, yaw_rad);
    return tf2::toMsg(q);
}

class DroneCtrl
{
public:
    ros::NodeHandle nh;

    ros::Subscriber state_sub;
    ros::Subscriber battery_sub;
    ros::Subscriber cmd_sub;
    ros::Subscriber rc_sub;
    ros::Subscriber odom_sub;

    ros::Publisher pose_pub;
    ros::Publisher vel_pub;
    ros::Publisher local_pos_pub;
    ros::Publisher traj_start_trigger_pub;

    ros::ServiceClient set_FCU_mode_srv;
    ros::ServiceClient arming_client_srv;
    ros::ServiceClient reboot_FCU_srv;
    ros::ServiceClient takeoff_client_src;

    RC_Data_t rc_data;
    State_Data_t state_data;
    Battery_Data_t battery_data;
    Command_Data_t cmd_data;
    Odom_Data_t odom_data;

    DroneCtrl() : nh("~"){
        state_sub = nh.subscribe<mavros_msgs::State>(
            "/mavros/state", 10, boost::bind(&State_Data_t::feed, &state_data, _1));
        
        battery_sub = nh.subscribe<sensor_msgs::BatteryState>(
            "/mavros/battery", 10, boost::bind(&Battery_Data_t::feed, &battery_data, _1));

        cmd_sub = nh.subscribe<quadrotor_msgs::PositionCommand>(
            "/position_cmd", 100, boost::bind(&Command_Data_t::feed, &cmd_data, _1),
            ros::VoidConstPtr(), ros::TransportHints().tcpNoDelay());

        rc_sub = nh.subscribe<mavros_msgs::RCIn>(
            "/mavros/rc/in", 10, boost::bind(&RC_Data_t::feed, &rc_data, _1));

        odom_sub = nh.subscribe("/Odometry", 10, &DroneCtrl::odometryCallback, this);

        pose_pub = nh.advertise<geometry_msgs::PoseStamped>("/mavros/vision_pose/pose", 10);
        vel_pub = nh.advertise<geometry_msgs::TwistStamped>("/mavros/setpoint_velocity/cmd_vel", 10);
        local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>("/mavros/setpoint_position/local", 10);
        traj_start_trigger_pub = nh.advertise<geometry_msgs::PoseStamped>("/traj_start_trigger", 10);

        set_FCU_mode_srv = nh.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");
        arming_client_srv = nh.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");
        reboot_FCU_srv = nh.serviceClient<mavros_msgs::CommandLong>("/mavros/cmd/command");
    }

    bool rc_is_received(const ros::Time &now_time){return rc_data.is_received(now_time);}
    bool state_is_received(const ros::Time &now_time){return state_data.is_received(now_time);}
    bool battery_is_received(const ros::Time &now_time){return battery_data.is_received(now_time);}
    bool odom_is_received(const ros::Time &now_time){return odom_data.is_received(now_time);}
    bool cmd_is_received(const ros::Time &now_time){return cmd_data.is_received(now_time);}

    void init_target_takeoff();
    void update_target(const nav_msgs::Odometry& target);
    void update_cmd_vel(const quadrotor_msgs::PositionCommand& msg);
    void process();
    void publish_target();
    void publish_cmd_vel();

    void odometryCallback(const nav_msgs::Odometry::ConstPtr& msg);

private:
    bool is_armed = false;
    bool is_takeoff = false;
    bool is_land = true;
    bool is_landing = false;
    bool enter_hold = false;
    bool enter_land = false;
    bool check_battery = false;
    bool check_rc_stick = true;

    ros::Time last_arm_request;

    geometry_msgs::PoseStamped desired_target;
    geometry_msgs::TwistStamped desired_cmd_vel;

    int control_status = 0; // 0: 位置控制, 1: 速度控制, 2: 姿态控制
    std::string control_mode;
    double takeoff_height = 0.5;
    double battery_limit = 19.8;

    double position_max_x = 2.0;
    double position_min_x = -2.0;
    double position_max_y = 2.0;
    double position_min_y = -2.0;
    double position_max_z = 1.5;
    double position_min_z = 0.0;

    double cmd_vel_max_x = 1.0;
    double cmd_vel_max_y = 1.0;
    double cmd_vel_max_z = 0.5;

    int cmd_stage = 0;        // 当前步骤（0~6）
    ros::Time cmd_start_time; // 当前步骤开始时间
    bool cmd_running = false; // 是否正在执行command sequence
    bool cmd_finish = false; // 是否完成 command sequence
};

void DroneCtrl::odometryCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
    odom_data.feed(msg);
    // 发布位姿
    geometry_msgs::PoseStamped pose_msg;
    pose_msg.header = msg->header;      // 保留时间戳和 frame_id
    pose_msg.header.frame_id = "odom";
    pose_msg.pose = msg->pose.pose;     // 直接复制位姿

    uav_mavros_pose_fix(&pose_msg); // 矫正坐标系

    pose_pub.publish(pose_msg);
}

void DroneCtrl::init_target_takeoff()
{
    desired_target.pose.position.z = 0.5;
}

void DroneCtrl::update_target(const nav_msgs::Odometry& target)
{
    desired_target.pose.position.x = target.pose.pose.position.x;
    desired_target.pose.position.y = target.pose.pose.position.y;
    desired_target.pose.position.z = target.pose.pose.position.z;
    desired_target.pose.orientation.x = target.pose.pose.orientation.x;
    desired_target.pose.orientation.y = target.pose.pose.orientation.y;
    desired_target.pose.orientation.z = target.pose.pose.orientation.z;
    desired_target.pose.orientation.w = target.pose.pose.orientation.w;
    uav_mavros_pose_fix(&desired_target);
}

void DroneCtrl::update_cmd_vel(const quadrotor_msgs::PositionCommand& msg)
{
    desired_cmd_vel.twist.linear.x = msg.velocity.x;
    desired_cmd_vel.twist.linear.y = msg.velocity.y;
    desired_cmd_vel.twist.linear.z = msg.velocity.z;
    //desired_cmd_vel.twist.angular.x = 0.0;
    //desired_cmd_vel.twist.angular.y = 0.0;
    //desired_cmd_vel.twist.angular.z = msg->yaw_dot;
    uav_mavros_cmd_vel_fix(&desired_cmd_vel);
}

void DroneCtrl::process()
{
    geometry_msgs::PoseStamped target;
    static bool have_hold_set = false;

    if(rc_data.is_armed && rc_data.is_offboard_mode &&!is_armed && check_battery && check_rc_stick)
    {
        if((ros::Time::now() - last_arm_request).toSec() > 3.0)
        {
            mavros_msgs::CommandBool arm_cmd;
            arm_cmd.request.value = true;

            if (arming_client_srv.call(arm_cmd) &&
                arm_cmd.response.success)
            {
                is_armed = true;
            }
            else
            {
                ROS_ERROR("[PX4CTRL] Vehicle while failed armimg, please check your system and restart!");
            }
            last_arm_request = ros::Time::now();
        }
    }
    else
    {
        if(!state_data.current_state.armed)
        {
            is_armed = false;
            is_landing = false;
        }
    }

    if(rc_data.is_offboard_mode && !is_landing)
    {
        if(rc_data.is_hold_mode || (rc_data.is_command_mode && !is_takeoff) || (rc_data.is_command_mode && enter_hold))
        {
            if(!have_hold_set)
            {
                update_target(odom_data.msg);
                have_hold_set = true;
            }

            if(is_land && !is_takeoff)
            {
                init_target_takeoff();
                is_land = false;
            }

            if(odom_data.msg.pose.pose.position.z > takeoff_height - 0.5 && !is_takeoff) 
            {
                is_takeoff = true;
            }
            publish_target();
        }
        else
        {
            if(rc_data.is_command_mode)
            {
                if(!cmd_finish)
                {
                    // 第一次进入 command_mode，启动任务
                    if(!cmd_running)
                    {
                        cmd_running = true;
                        cmd_stage = 1;
                        cmd_start_time = ros::Time::now();
                    }

                    // 每个阶段执行 1 秒
                    double dt = (ros::Time::now() - cmd_start_time).toSec();

                    if(cmd_stage == 1) {
                        // X 正
                        if(dt < 3.0) {
                            quadrotor_msgs::PositionCommand cmd;
                            cmd.velocity.x = 0.5;
                            update_cmd_vel(cmd);
                            publish_cmd_vel();
                        } else {
                            cmd_stage = 2;
                            cmd_start_time = ros::Time::now();
                        }
                    }
                    else if(cmd_stage == 2) {
                        // X 负
                        if(dt < 3.0) {
                            quadrotor_msgs::PositionCommand cmd;
                            cmd.velocity.x = -0.5;
                            update_cmd_vel(cmd);
                            publish_cmd_vel();
                        } else {
                            cmd_stage = 3;
                            cmd_start_time = ros::Time::now();
                        }
                    }
                    else if(cmd_stage == 3) {
                        // Y 正
                        if(dt < 3.0) {
                            quadrotor_msgs::PositionCommand cmd;
                            cmd.velocity.y = -0.5;
                            update_cmd_vel(cmd);
                            publish_cmd_vel();
                        } else {
                            cmd_stage = 4;
                            cmd_start_time = ros::Time::now();
                        }
                    }
                    else if(cmd_stage == 4) {
                        // Y 负
                        if(dt < 3.0) {
                            quadrotor_msgs::PositionCommand cmd;
                            cmd.velocity.y = 0.5;
                            update_cmd_vel(cmd);
                            publish_cmd_vel();
                        } else {
                            cmd_stage = 5;
                            cmd_start_time = ros::Time::now();
                        }
                    }
                    else if(cmd_stage == 5) {
                        // Z 正
                        if(dt < 3.0) {
                            quadrotor_msgs::PositionCommand cmd;
                            cmd.velocity.z = 0.1;
                            update_cmd_vel(cmd);
                            publish_cmd_vel();
                        } else {
                            cmd_stage = 6;
                            cmd_start_time = ros::Time::now();
                        }
                    }
                    else if(cmd_stage == 6) {
                        // Z 负
                        if(dt < 3.0) {
                            quadrotor_msgs::PositionCommand cmd;
                            cmd.velocity.z = -0.1;
                            update_cmd_vel(cmd);
                            publish_cmd_vel();
                        } else {
                            // 所有阶段结束
                            cmd_finish = true;
                            cmd_running = false;
                        }
                    }
                }
            }
            else
            {
                if(!is_landing) is_land = true;
                if(!is_land && !is_takeoff) is_landing = true;
                is_takeoff = false;
                enter_hold = false;
                cmd_finish = false;
            }
            have_hold_set = false;
        }
    }
    else
    {
        is_takeoff = false;
        is_land = true;
        enter_hold = false;
        enter_land = false;
        have_hold_set = false;
    }

    if(battery_data.battery.voltage < battery_limit)
    {
        check_battery = false;
        ROS_WARN_THROTTLE(5.0, "[PX4CTRL] Battery voltage low: %.2f V < %.2f V", battery_data.battery.voltage, battery_limit);
    }
    else
    {
        check_battery = true;
    }
}

void DroneCtrl::publish_target()
{
    geometry_msgs::PoseStamped target = desired_target;

    auto clamp = [](double value, double min_val, double max_val) {
        if (value < min_val)
            return min_val;
        else if (value > max_val)
            return max_val;
        else
            return value;
    };

    target.pose.position.x = clamp(target.pose.position.x, -position_max_y, -position_min_y);
    target.pose.position.y = clamp(target.pose.position.y,  position_min_x,  position_max_x);
    target.pose.position.z = clamp(target.pose.position.z,  position_min_z,  position_max_z);
    local_pos_pub.publish(target);
}

void DroneCtrl::publish_cmd_vel()
{
    geometry_msgs::TwistStamped cmd_vel = desired_cmd_vel;

    auto clamp = [](double value, double max_val) {
        if (value > max_val){
            return max_val;
            ROS_WARN_THROTTLE(5.0, "[PX4CTRL] Velocity limit exceeded: %.2f > %.2f", value, max_val);}
        else{
            return value;}
    };

    cmd_vel.twist.linear.x = clamp(cmd_vel.twist.linear.x,  cmd_vel_max_y);
    cmd_vel.twist.linear.y = clamp(cmd_vel.twist.linear.y,  cmd_vel_max_x);
    cmd_vel.twist.linear.z = clamp(cmd_vel.twist.linear.z,  cmd_vel_max_z);
    vel_pub.publish(cmd_vel);
}



void mySigintHandler(int sig)
{
    ROS_INFO("[PX4Ctrl] exit...");
    ros::shutdown();
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "px4ctrl");
    signal(SIGINT, mySigintHandler);

    DroneCtrl node;

    ros::Duration(1).sleep();

    int trials = 0;
    while (ros::ok() && !node.state_data.current_state.connected)
    {
        ros::spinOnce();
        ros::Duration(1.0).sleep();
        if (trials++ > 5)
            ROS_ERROR_THROTTLE(2.0, "[PX4CTRL] Unable to connnect to PX4!!!");
    }
    ROS_INFO("[PX4CTRL][1/3] PX4 connected.");

    while (ros::ok())
    {
        ros::spinOnce();
        if (node.state_data.current_state.manual_input)
        {
            ROS_INFO("[PX4CTRL][2/3] RC received.");
            node.rc_data.check_validity();
            break;
        }
        ros::Duration(0.1).sleep();
    }
    while (ros::ok())
    {
        ros::spinOnce();
        if (node.odom_is_received(ros::Time::now()))
        {
            ROS_INFO("[PX4CTRL][3/3] Odometry received and publishing.");
            break;
        }
        ros::Duration(0.1).sleep();
    }

    ROS_INFO("[PX4CTRL] Mode: %s, Battery: %.1f V, Current: %.2f A", 
        node.state_data.current_state.mode.c_str(), node.battery_data.battery.voltage, node.battery_data.battery.current);
    ROS_INFO("[PX4CTRL] Please manually switch to offboard mode and manually arm it to automatically takeoff.");
    ros::Rate r(30.0);
    while (ros::ok())
    {
        r.sleep();
        ros::spinOnce();
        node.process();
    }

    return 0;
}
