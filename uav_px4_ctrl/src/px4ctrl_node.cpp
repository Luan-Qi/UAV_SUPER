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
#include <nav_msgs/Odometry.h>
#include <quadrotor_msgs/PositionCommand.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <signal.h>
#include "subscribe.h"
#include "uav_px4_ctrl/TakeoffNotify.h"


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
        nh.param<double>("/takeoff_height", takeoff_height, takeoff_height);
        nh.param<std::string>("control_mode", control_mode, "position");
        nh.param<double>("battery_limit", battery_limit, battery_limit);
        nh.param<double>("position_max_x", position_max_x, position_max_x);
        nh.param<double>("position_min_x", position_min_x, position_min_x);
        nh.param<double>("position_max_y", position_max_y, position_max_y);
        nh.param<double>("position_min_y", position_min_y, position_min_y);
        nh.param<double>("position_max_z", position_max_z, position_max_z);
        nh.param<double>("position_min_z", position_min_z, position_min_z);
        nh.param<double>("position_max_vel", position_max_vel, position_max_vel);
        nh.param<double>("position_max_acc", position_max_acc, position_max_acc);
        nh.param<double>("cmd_vel_max_x", cmd_vel_max_x, cmd_vel_max_x);
        nh.param<double>("cmd_vel_max_y", cmd_vel_max_y, cmd_vel_max_y);
        nh.param<double>("cmd_vel_max_z", cmd_vel_max_z, cmd_vel_max_z);
        nh.param<double>("cmd_vel_max_acc", cmd_vel_max_acc, cmd_vel_max_acc);
        nh.param<double>("cmd_vel_max_jerk", cmd_vel_max_jerk, cmd_vel_max_jerk);

        state_sub = nh.subscribe<mavros_msgs::State>(
            "/mavros/state", 10, boost::bind(&State_Data_t::feed, &state_data, _1));
        
        battery_sub = nh.subscribe<mavros_msgs::BatteryStatus>(
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
        takeoff_client_src = nh.serviceClient<uav_px4_ctrl::TakeoffNotify>("/takeoff_notify");

        if(control_mode == "position")
            control_status = 0;
        else if(control_mode == "velocity")
            control_status = 1;
        else if(control_mode == "attitude")
            control_status = 2;
        else
            ROS_ERROR("[PX4CTRL] Invalid control mode, default to position control mode.");
    }

    bool rc_is_received(const ros::Time &now_time){return rc_data.is_received(now_time);}
    bool state_is_received(const ros::Time &now_time){return state_data.is_received(now_time);}
    bool battery_is_received(const ros::Time &now_time){return battery_data.is_received(now_time);}
    bool odom_is_received(const ros::Time &now_time){return odom_data.is_received(now_time);}
    bool cmd_is_received(const ros::Time &now_time){return cmd_data.is_received(now_time);}

    void init_target_takeoff();
    void update_target(const geometry_msgs::PoseStamped& target);
    void update_target(const nav_msgs::Odometry& target);
    void update_target(const quadrotor_msgs::PositionCommand& target);
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
    ros::Time last_target_pub;
    ros::Time last_cmd_vel_pub;

    geometry_msgs::PoseStamped desired_target;
    geometry_msgs::PoseStamped last_target;
    geometry_msgs::TwistStamped desired_cmd_vel;
    geometry_msgs::TwistStamped last_cmd_vel;

    int control_status = 0; // 0: 位置控制, 1: 速度控制, 2: 姿态控制
    std::string control_mode;
    double takeoff_height = 0.5;
    double battery_limit = 19.8;

    double position_max_x = 0.0;
    double position_min_x = 0.0;
    double position_max_y = 0.0;
    double position_min_y = 0.0;
    double position_max_z = 0.0;
    double position_min_z = 0.0;
    double position_max_vel = 0.0;
    double position_max_acc = 0.0;

    double cmd_vel_max_x = 0.0;
    double cmd_vel_max_y = 0.0;
    double cmd_vel_max_z = 0.0;
    double cmd_vel_max_acc = 0.0;
    double cmd_vel_max_jerk = 0.0;
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
    desired_target.pose.position.z = takeoff_height;
}

void DroneCtrl::update_target(const geometry_msgs::PoseStamped& target)
{
    desired_target.pose.position.x = target.pose.position.x;
    desired_target.pose.position.y = target.pose.position.y;
    desired_target.pose.position.z = target.pose.position.z;
    desired_target.pose.orientation.x = target.pose.orientation.x;
    desired_target.pose.orientation.y = target.pose.orientation.y;
    desired_target.pose.orientation.z = target.pose.orientation.z;
    desired_target.pose.orientation.w = target.pose.orientation.w;
    uav_mavros_pose_fix(&desired_target);
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

void DroneCtrl::update_target(const quadrotor_msgs::PositionCommand& target)
{
    desired_target.pose.position.x = target.position.x;
    desired_target.pose.position.y = target.position.y;
    desired_target.pose.position.z = target.position.z;
    desired_target.pose.orientation = yawToQuaternion((double)target.yaw);
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
                uav_px4_ctrl::TakeoffNotify srv;
                srv.request.takeoff_done = true;

                if(!takeoff_client_src.call(srv))
                    enter_hold = true;
                is_takeoff = true;
            }
            publish_target();
        }
        else
        {
            if(rc_data.is_command_mode)
            {
                switch(control_status)
                {
                    case 0:
                        if(cmd_is_received(ros::Time::now())) update_target(cmd_data.msg);
                        publish_target();
                        break;
                    case 1:
                        if(cmd_is_received(ros::Time::now())) update_cmd_vel(cmd_data.msg);
                        publish_cmd_vel();
                        break;
                    case 2:
                        ROS_ERROR("[PX4CTRL] Attitude control not implemented yet!");
                        break;
                    default:
                        ROS_ERROR("[PX4CTRL] Invalid control mode, please check your code !!!!!!!!!!!!.");
                        break;
                }
            }
            else
            {
                if(!is_landing) is_land = true;
                if(!is_land && !is_takeoff) is_landing = true;
                is_takeoff = false;
                enter_hold = false;
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

    if(is_takeoff)
    {
        ros::Time now = ros::Time::now();
        double dt = (now - last_target_pub).toSec();

        if (dt > 0.001) // 避免除零
        {
            double cur_x = last_target.pose.position.x;
            double cur_y = last_target.pose.position.y;
            double cur_z = last_target.pose.position.z;

            double dx = target.pose.position.x - cur_x;
            double dy = target.pose.position.y - cur_y;
            double dz = target.pose.position.z - cur_z;

            double vel = sqrt(dx * dx + dy * dy + dz * dz) / dt;

            if (vel > position_max_vel)
            {
                ROS_WARN("[PX4CTRL] Velocity limit exceeded: %.2f m/s > %.2f m/s", vel, position_max_vel);
                enter_hold = true;
                return;
            }
        }
        else
        {
            ROS_WARN("[PX4CTRL] Invalid time interval dt=%.6f", dt);
            return;
        }
    }

    last_target = target;
    last_target_pub = ros::Time::now();
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

    cmd_vel.twist.linear.x = clamp(cmd_vel.twist.linear.x, -cmd_vel_max_y);
    cmd_vel.twist.linear.y = clamp(cmd_vel.twist.linear.y,  cmd_vel_max_x);
    cmd_vel.twist.linear.z = clamp(cmd_vel.twist.linear.z,  cmd_vel_max_z);

    if(is_takeoff)
    {
        ros::Time now = ros::Time::now();
        double dt = (now - last_cmd_vel_pub).toSec();

        if (dt > 0.001)  // 避免除零
        {
            double vx = cmd_vel.twist.linear.x;
            double vy = cmd_vel.twist.linear.y;
            double vz = cmd_vel.twist.linear.z;

            double last_vx = last_cmd_vel.twist.linear.x;
            double last_vy = last_cmd_vel.twist.linear.y;
            double last_vz = last_cmd_vel.twist.linear.z;

            double ax = (vx - last_vx) / dt;
            double ay = (vy - last_vy) / dt;
            double az = (vz - last_vz) / dt;

            // 加速度模长
            double acc = sqrt(ax * ax + ay * ay + az * az);

            if (acc > cmd_vel_max_acc)
            {
                ROS_WARN("[PX4CTRL] Acc limit exceeded: %.2f m/s^2 > %.2f m/s^2", 
                        acc, cmd_vel_max_acc);
                enter_hold = true;
                return;
            }
        }
        else
        {
            ROS_WARN("[PX4CTRL] Invalid time interval dt=%.6f", dt);
            return;
        }
    }

    last_cmd_vel = cmd_vel;
    last_cmd_vel_pub = ros::Time::now();
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
