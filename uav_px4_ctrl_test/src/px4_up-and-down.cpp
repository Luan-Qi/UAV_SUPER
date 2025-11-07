#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <signal.h>
#include <cmath>

// 保存当前飞控状态
mavros_msgs::State current_state;

void mySigintHandler(int sig)
{
    ROS_INFO("controling exit...");
    ros::shutdown();
}

// 坐标系修正
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

// 当前飞控状态回调
void state_cb(const mavros_msgs::State::ConstPtr& msg)
{
    current_state = *msg;
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "offboard_position_control");
    ros::NodeHandle nh;

    signal(SIGINT, mySigintHandler);

    // 订阅无人机状态
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("mavros/state", 10, state_cb);
    // 发布目标位置
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 10);

    // 服务客户端：解锁 + 模式切换
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");

    ros::Rate rate(20.0);  // PX4 要求 ≥ 2Hz，这里设为 20Hz

    // 等待连接
    while (ros::ok() && !current_state.connected)
    {
        ros::spinOnce();
        rate.sleep();
    }
    ROS_INFO("Vehicle connected");

    std::vector<geometry_msgs::PoseStamped> waypoints(2);

    waypoints[0].pose.position.x = 0;
    waypoints[0].pose.position.y = 0;
    waypoints[0].pose.position.z = 0.5;
    waypoints[1].pose.position.x = 0;
    waypoints[1].pose.position.y = 0;
    waypoints[1].pose.position.z = 1.0;

    for (auto &wp : waypoints)
    {
        wp.pose.orientation.x = 0;
        wp.pose.orientation.y = 0;
        wp.pose.orientation.z = 0;
        wp.pose.orientation.w = 1;
    }

    // 矫正mavros坐标，交换x和y，并-x，四元数绕z轴旋转90度
    for (auto &wp : waypoints)
        uav_mavros_pose_fix(&wp);

    // 在进入 Offboard 模式前，先发送一些 setpoint，否则切换会失败
    for (int i = 20; ros::ok() && i > 0; --i)
    {
        local_pos_pub.publish(waypoints[0]);
        ros::spinOnce();
        rate.sleep();
    }

    // 请求切换Offboard模式
    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    if (set_mode_client.call(offb_set_mode) &&
        offb_set_mode.response.mode_sent)
    {
        ROS_INFO("Offboard enabled");
    }
    else
    {
        ROS_ERROR("Vehicle while failed setting to offboard mode, please check your system and restart!");
        while(ros::ok());
    }

    // 切换模式后继续发送 setpoint，否则切换会失败
    for (int i = 10; ros::ok() && i > 0; --i)
    {
        local_pos_pub.publish(waypoints[0]);
        ros::spinOnce();
        rate.sleep();
    }

    // 自动解锁
    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    if (arming_client.call(arm_cmd) &&
        arm_cmd.response.success)
    {
        ROS_INFO("Vehicle armed");
    }
    else
    {
        ROS_ERROR("Vehicle while failed armimg, please check your system and restart!");
        while(ros::ok());
    }

    int target_idx = 0;
    ros::Time last_time = ros::Time::now();
    ROS_INFO("Starting up and down flight...");

    while (ros::ok())
    {
        geometry_msgs::PoseStamped target = waypoints[target_idx];
        local_pos_pub.publish(target);

        if((ros::Time::now() - last_time).toSec() > 5)
        {
            last_time = ros::Time::now();
            target_idx = 1 - target_idx;
            ROS_INFO("moving to %d", target_idx);
        }

        if(current_state.mode != "OFFBOARD")
        {
            ROS_WARN("Manual interrupt, controlling exit!");
            break;
        }

        ros::spinOnce();
        rate.sleep();
    }

    ros::shutdown();
    return 0;
}
