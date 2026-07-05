/**
 * @file px4_pos_takeoff.cpp
 * @brief PX4 Offboard 位置控制 — 定高起飞测试节点。
 *
 * @details
 * 本节点实现基本的 PX4 Offboard 位置控制流程，使无人机从地面起飞到指定高度并悬停。
 * 核心功能：
 *   1. 飞控连接等待 — 阻塞等待 MAVROS 连接 PX4
 *   2. 坐标修正 — ENU → NED 位置/姿态变换
 *   3. Offboard 模式切换 — 预发送 setpoint → 切换模式 → 继续发送
 *   4. 自动解锁 — 调用 /mavros/cmd/arming 服务
 *   5. 定高悬停 — 持续发送目标位置以维持飞行
 *   6. SIGINT 安全处理 — Ctrl-C 优雅退出
 *
 * PX4 Offboard 模式要求：
 *   - 进入 Offboard 前需持续发送 setpoint（至少 2 Hz）
 *   - 解锁前需先切换到 Offboard 模式
 *   - setpoint 发布频率需 ≥ 2 Hz（本节点采用 20 Hz）
 *
 * 坐标系变换 (uav_mavros_pose_fix)：
 *   输入坐标系 → 输出坐标系：绕 Z 轴 +90° (π/2)，x' = -y, y' = x
 *   用途：将 Fast-LIO ENU 系位姿转换为 PX4 NED 系位姿
 *
 * 依赖：
 *   - mavros_msgs (State, CommandBool, SetMode)
 *   - geometry_msgs/PoseStamped
 *   - tf2 (坐标旋转)
 *
 * 使用示例：
 *   rosrun uav_px4_ctrl_test px4_takeoff
 */

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <signal.h>

// ============================================================================
// 全局状态变量
// ============================================================================
mavros_msgs::State current_state;  ///< 飞控当前状态 (connected/armed/mode)

// ============================================================================
// SIGINT 信号处理
// ============================================================================
/**
 * @brief SIGINT (Ctrl-C) 信号处理器 — 优雅退出。
 * @param sig 信号编号 (未使用)
 */
void mySigintHandler(int sig)
{
    ROS_INFO("controling exit...");
    ros::shutdown();
}

// ============================================================================
// 坐标变换工具函数
// ============================================================================
/**
 * @brief 坐标系修正：ENU → NED 等效变换。
 * @param[in,out] pose 待修正的位姿消息
 *
 * 位置：x' = -y, y' = x (交换并取反)
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
 * @param msg 飞控状态消息
 */
void state_cb(const mavros_msgs::State::ConstPtr& msg)
{
    current_state = *msg;
}

// ============================================================================
// main — 起飞控制流水线
// ============================================================================
int main(int argc, char **argv)
{
    ros::init(argc, argv, "offboard_position_control");
    ros::NodeHandle nh;

    signal(SIGINT, mySigintHandler);

    // ---- 通信初始化 ----
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>(
        "mavros/state", 10, state_cb);                              ///< 订阅飞控状态

    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>(
        "mavros/setpoint_position/local", 10);                      ///< 发布 offboard 位置指令

    ros::ServiceClient arming_client =
        nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");   ///< 解锁服务
    ros::ServiceClient set_mode_client =
        nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");         ///< 模式切换服务

    ros::Rate rate(20.0);  // PX4 要求 ≥ 2 Hz，这里设为 20 Hz

    // ---- 步骤 1：等待 PX4 连接 ----
    while (ros::ok() && !current_state.connected)
    {
        ros::spinOnce();
        rate.sleep();
    }
    ROS_INFO("Vehicle connected");

    // ---- 步骤 2：构造目标位姿 (起飞到 1 m 高度) ----
    geometry_msgs::PoseStamped pose;

    pose.pose.position.x = 0;
    pose.pose.position.y = 0;
    pose.pose.position.z = 1;  // 目标高度 [m]

    pose.pose.orientation.x = 0;
    pose.pose.orientation.y = 0;
    pose.pose.orientation.z = 0;
    pose.pose.orientation.w = 1; // 单位四元数 (无旋转)

    // 坐标系修正 (ENU → NED)
    uav_mavros_pose_fix(&pose);

    // ---- 步骤 3：预发送 setpoint (Offboard 切换前提条件) ----
    for (int i = 20; ros::ok() && i > 0; --i)
    {
        local_pos_pub.publish(pose);
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

    // ---- 步骤 5：切换后继续发送 setpoint (防止模式回退) ----
    for (int i = 10; ros::ok() && i > 0; --i)
    {
        local_pos_pub.publish(pose);
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

    // ---- 步骤 7：持续发送目标位置 (维持飞行) ----
    while (ros::ok())
    {
        local_pos_pub.publish(pose);

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
