#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <mavros_msgs/RCIn.h>
#include <mavros_msgs/State.h>
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
    ros::Subscriber cmd_sub;
    ros::Subscriber rc_sub;
    ros::Subscriber odom_sub;

    ros::Publisher pose_pub;
    ros::Publisher local_pos_pub;
    ros::Publisher traj_start_trigger_pub;

    ros::ServiceClient set_FCU_mode_srv;
    ros::ServiceClient arming_client_srv;
    ros::ServiceClient reboot_FCU_srv;

    RC_Data_t rc_data;
    State_Data_t state_data;
    Command_Data_t cmd_data;
    Odom_Data_t odom_data;

    bool is_armed = false;
    bool is_takeoff = false;
    bool is_land = true;

    DroneCtrl() {
        nh.param<double>("/takeoff_height", takeoff_height, takeoff_height);

        state_sub = nh.subscribe<mavros_msgs::State>(
            "/mavros/state", 10, boost::bind(&State_Data_t::feed, &state_data, _1));

        cmd_sub = nh.subscribe<quadrotor_msgs::PositionCommand>(
            "/position_cmd", 100, boost::bind(&Command_Data_t::feed, &cmd_data, _1),
            ros::VoidConstPtr(), ros::TransportHints().tcpNoDelay());

        rc_sub = nh.subscribe<mavros_msgs::RCIn>(
            "/mavros/rc/in", 10, boost::bind(&RC_Data_t::feed, &rc_data, _1));

        odom_sub = nh.subscribe("/Odometry", 10, &DroneCtrl::odometryCallback, this);

        pose_pub = nh.advertise<geometry_msgs::PoseStamped>("/mavros/vision_pose/pose", 10);
        local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 10);
        traj_start_trigger_pub = nh.advertise<geometry_msgs::PoseStamped>("/traj_start_trigger", 10);

        set_FCU_mode_srv = nh.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");
        arming_client_srv = nh.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");
        reboot_FCU_srv = nh.serviceClient<mavros_msgs::CommandLong>("/mavros/cmd/command");
    }

    bool rc_is_received(const ros::Time &now_time){return rc_data.is_received(now_time);}
    bool odom_is_received(const ros::Time &now_time){return odom_data.is_received(now_time);}
    bool cmd_is_received(const ros::Time &now_time){return cmd_data.is_received(now_time);}

    void init_target_takeoff();
    void update_target(const geometry_msgs::PoseStamped& target);
    void update_target(const nav_msgs::Odometry& target);
    void update_target(const quadrotor_msgs::PositionCommand& target);
    void process();
    void publish_target();

    void odometryCallback(const nav_msgs::Odometry::ConstPtr& msg);

private:
    geometry_msgs::PoseStamped desired_target;
    double takeoff_height = 0.5;
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

    //pose_pub.publish(pose_msg);
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

void DroneCtrl::process()
{
    geometry_msgs::PoseStamped target;
    static bool have_hold_set = false;

    if(rc_data.is_armed && !is_armed)
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
            ROS_ERROR("Vehicle while failed armimg, please check your system and restart!");
        }
    }

    if(rc_data.is_hover_mode)
    {
        if(rc_data.is_hold_mode)
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
                if(odom_data.msg.pose.pose.position.z > 0.45) is_takeoff = true;
            }
            publish_target();
        }
        else
        {
            if(rc_data.is_command_mode)
            {
                if(cmd_is_received(ros::Time::now())) update_target(cmd_data.msg);
                publish_target();
            }
            else
            {
                is_takeoff = false;
                is_land = true;
            }
            have_hold_set = false;
        }
    }
    else
    {
        is_takeoff = false;
        is_land = true;
        have_hold_set = false;
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

    target.pose.position.x = clamp(target.pose.position.x, -2.0, 2.0);
    target.pose.position.y = clamp(target.pose.position.y, -1.0, 5.0);
    target.pose.position.z = clamp(target.pose.position.z, 0.0, 1.5);

    local_pos_pub.publish(target);
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
    while (ros::ok())
    {
        ros::spinOnce();
        if (node.rc_is_received(ros::Time::now()))
        {
            ROS_INFO("[PX4CTRL] Odometry received and publishing.");
            break;
        }
        ros::Duration(0.1).sleep();
    }

    int trials = 0;
    while (ros::ok() && !node.state_data.current_state.connected)
    {
        ros::spinOnce();
        ros::Duration(1.0).sleep();
        if (trials++ > 5)
            ROS_ERROR("[PX4CTRL] Unable to connnect to PX4!!!");
    }

    ROS_INFO("[PX4CTRL] PX4 connected.");
    while (ros::ok())
    {
        ros::spinOnce();
        if (node.rc_is_received(ros::Time::now()))
        {
            ROS_INFO("[PX4CTRL] RC received.");
            node.rc_data.check_validity();
            break;
        }
        ros::Duration(0.1).sleep();
    }

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
