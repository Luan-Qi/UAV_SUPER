/**
 * @file px4_pos_up-and-down.cpp
 * @brief PX4 Offboard 位置控制 — 定高往复升降测试节点。
 *
 * @details
 * 本节点控制无人机在两个高度之间往复升降（0.5 m ↔ 1.0 m），每 5 秒切换一次。
 * 核心功能：
 *   1. 双航点定义 — 低点 (z=0.5m) 和 高点 (z=1.0m)
 *   2. 定时切换 — 每 5 秒切换一次目标高度
 *   3. 坐标修正 — ENU → NED (同 px4_pos_takeoff)
 *   4. Offboard 模式管理 — 预发 setpoint → 切换 → 解锁
 *
 * 依赖：
 *   - mavros_msgs (State, CommandBool, SetMode)
 *   - geometry_msgs/PoseStamped
 *   - tf2 (坐标旋转)
 *
 * 使用示例：
 *   rosrun uav_px4_ctrl_test px4_up-and-down
 */

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <signal.h>
#include <cmath>

// ============================================================================
// 全局状态变量
// ============================================================================
mavros_msgs::State current_state;  ///< 飞控当前状态

// ============================================================================
// SIGINT 信号处理
// ============================================================================
/**
 * @brief SIGINT (Ctrl-C) 信号处理器 — 优雅退出。
 */
void mySigintHandler(int sig)
{
    ROS_INFO("controling exit...");
    ros::shutdown();
}

// ============================================================================
// 坐标系修正
// ============================================================================
/**
 * @brief 坐标系修正：ENU → NED 等效变换。
 * @param[in,out] pose 待修正的位姿消息
 *
 * 位置：x' = -y, y' = x
 * 姿态：绕 Z 轴旋转 +90° (π/2)
 */
void uav_mavros_pose_fix(geometry_msgs::PoseStamped * pose)
{
    double temp_x = pose->pose.position.x;
    double temp_y = pose->pose.position.y;

    pose->pose.position.x = -temp_y;
    pose->pose.position.y = temp_x;

    tf2::Quaternion q_orig, q_rot, q_new;
    tf2::fromMsg(pose->pose.orientation, q_orig);
    q_rot.setRPY(0, 0, M_PI/2);  // 绕 Z 轴旋转 +90°
    q_new = q_rot * q_orig;      // 旋转叠加
    q_new.normalize();
    pose->pose.orientation = tf2::toMsg(q_new);
}

// ============================================================================
// MAVROS 话题回调
// ============================================================================
/**
 * @brief 飞控状态回调 — 缓存当前 PX4 状态。
 */
void state_cb(const mavros_msgs::State::ConstPtr& msg)
{
    current_state = *msg;
}

// ============================================================================
// main — 往复升降控制流水线
// ============================================================================
int main(int argc, char **argv)
{
    ros::init(argc, argv, "offboard_position_control");
    ros::NodeHandle nh;

    signal(SIGINT, mySigintHandler);

    // ---- 通信初始化 ----
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>(
        "mavros/state", 10, state_cb);                              ///< 飞控状态
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>(
        "mavros/setpoint_position/local", 10);                      ///< 位置指令

    ros::ServiceClient arming_client =
        nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");   ///< 解锁服务
    ros::ServiceClient set_mode_client =
        nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");         ///< 模式切换服务

    ros::Rate rate(20.0);  // PX4 要求 ≥ 2 Hz

    // ---- 步骤 1：等待 PX4 连接 ----
    while (ros::ok() && !current_state.connected)
    {
        ros::spinOnce();
        rate.sleep();
    }
    ROS_INFO("Vehicle connected");

    // ---- 步骤 2：定义两个高度航点 ----
    // waypoint[0] = z_low (0.5 m), waypoint[1] = z_high (1.0 m)
    std::vector<geometry_msgs::PoseStamped> waypoints(2);

    waypoints[0].pose.position.x = 0;
    waypoints[0].pose.position.y = 0;
    waypoints[0].pose.position.z = 0.5;  // 低点高度 [m]
    waypoints[1].pose.position.x = 0;
    waypoints[1].pose.position.y = 0;
    waypoints[1].pose.position.z = 1.0;  // 高点高度 [m]

    // 单位四元数
    for (auto &wp : waypoints)
    {
        wp.pose.orientation.x = 0;
        wp.pose.orientation.y = 0;
        wp.pose.orientation.z = 0;
        wp.pose.orientation.w = 1;
    }

    // 坐标修正：ENU → NED
    for (auto &wp : waypoints)
        uav_mavros_pose_fix(&wp);

    // ---- 步骤 3：预发送 setpoint ----
    for (int i = 20; ros::ok() && i > 0; --i)
    {
        local_pos_pub.publish(waypoints[0]);
        ros::spinOnce();
        rate.sleep();
    }

    // ---- 步骤 4：切换到 Offboard 模式 ----
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

    // ---- 步骤 5：切换后继续发送 setpoint ----
    for (int i = 10; ros::ok() && i > 0; --i)
    {
        local_pos_pub.publish(waypoints[0]);
        ros::spinOnce();
        rate.sleep();
    }

    // ---- 步骤 6：自动解锁 ----
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

    // ---- 步骤 7：定时升降循环 ----
    int target_idx = 0;                           // 当前目标高度索引 (0 ↔ 1)
    ros::Time last_time = ros::Time::now();       // 上次切换时间
    ROS_INFO("Starting up and down flight...");

    while (ros::ok())
    {
        geometry_msgs::PoseStamped target = waypoints[target_idx];
        local_pos_pub.publish(target);

        // 每 5 秒切换一次目标高度
        if((ros::Time::now() - last_time).toSec() > 5)
        {
            last_time = ros::Time::now();
            target_idx = 1 - target_idx;  // 0→1, 1→0 (toggle)
            ROS_INFO("moving to %d", target_idx);
        }

        // 模式监控：非 Offboard 退出
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
