/**
 * @file px4_pos_rectangle.cpp
 * @brief PX4 Offboard 位置控制 — 矩形航线飞行测试节点。
 *
 * @details
 * 本节点控制无人机以位置模式 (setpoint_position/local) 依次飞越矩形四个顶点，
 * 每个航点到达后悬停 3 秒再前往下一个，循环执行。
 * 核心功能：
 *   1. 航点定义 — 四个顶点构成正方形（边长 1 m，高度 1 m）
 *   2. 坐标修正 — ENU → NED (x'=-y, y'=x, yaw+=90°)
 *   3. 到达判定 — 欧氏距离 < 0.1 m 视为到达
 *   4. 悬停等待 — 到达后悬停 3 秒
 *   5. 位置反馈 — 订阅 /mavros/local_position/pose 获取当前位置
 *   6. 模式监控 — 非 Offboard 模式自动退出
 *
 * 坐标系约定：
 *   - 航点定义在 ENU 系 (X=东 Y=北 Z=上)，经 uav_mavros_pose_fix() 转换为 NED
 *   - 位置反馈来自 PX4 本地位置估计 (NED 系)
 *
 * 依赖：
 *   - mavros_msgs (State, CommandBool, SetMode)
 *   - geometry_msgs/PoseStamped
 *   - tf2 (坐标旋转)
 *
 * 使用示例：
 *   rosrun uav_px4_ctrl_test px4_rectangle
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
mavros_msgs::State current_state;           ///< 飞控当前状态
geometry_msgs::PoseStamped current_pose;    ///< 无人机当前位置 (PX4 本地位置估计)

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
// 工具函数
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

/**
 * @brief 计算两点间三维欧氏距离。
 * @param p1 点 1 [m]
 * @param p2 点 2 [m]
 * @return 三维距离 [m]
 */
double distance3D(const geometry_msgs::PoseStamped &p1, const geometry_msgs::PoseStamped &p2)
{
    double dx = p1.pose.position.x - p2.pose.position.x;
    double dy = p1.pose.position.y - p2.pose.position.y;
    double dz = p1.pose.position.z - p2.pose.position.z;
    return std::sqrt(dx * dx + dy * dy + dz * dz);
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

/**
 * @brief 位置回调 — 缓存无人机 PX4 本地位置估计。
 */
void pose_cb(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    current_pose = *msg;
}

// ============================================================================
// main — 矩形航线控制流水线
// ============================================================================
int main(int argc, char **argv)
{
    ros::init(argc, argv, "offboard_position_control");
    ros::NodeHandle nh;

    signal(SIGINT, mySigintHandler);

    // ---- 通信初始化 ----
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>(
        "mavros/state", 10, state_cb);                                  ///< 飞控状态
    ros::Subscriber pose_sub = nh.subscribe<geometry_msgs::PoseStamped>(
        "mavros/local_position/pose", 10, pose_cb);                     ///< 当前位置 (反馈)
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>(
        "mavros/setpoint_position/local", 10);                          ///< 位置指令

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

    // ---- 步骤 2：定义四个矩形航点 (ENU 系) ----
    // 航线： (0,0) → (s,0) → (s,s) → (0,s) → (0,0) → ...
    std::vector<geometry_msgs::PoseStamped> waypoints(4);
    double z_height = 1.0;   // 飞行高度 [m]
    double side = 1.0;       // 正方形边长 [m]
    waypoints[0].pose.position.x = 0;
    waypoints[0].pose.position.y = 0;
    waypoints[0].pose.position.z = z_height;
    waypoints[1].pose.position.x = side;
    waypoints[1].pose.position.y = 0;
    waypoints[1].pose.position.z = z_height;
    waypoints[2].pose.position.x = side;
    waypoints[2].pose.position.y = side;
    waypoints[2].pose.position.z = z_height;
    waypoints[3].pose.position.x = 0;
    waypoints[3].pose.position.y = side;
    waypoints[3].pose.position.z = z_height;

    // 所有航点使用单位四元数 (无偏航旋转)
    for (auto &wp : waypoints)
    {
        wp.pose.orientation.x = 0;
        wp.pose.orientation.y = 0;
        wp.pose.orientation.z = 0;
        wp.pose.orientation.w = 1;
    }

    // 坐标修正：ENU → NED (交换 x↔y，绕 Z 旋转 +90°)
    for (auto &wp : waypoints)
        uav_mavros_pose_fix(&wp);

    // ---- 步骤 3：预发送 setpoint（Offboard 切换前提） ----
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

    // ---- 步骤 5：切换后继续发送 setpoint（防止模式回退） ----
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

    // ---- 步骤 7：矩形航线循环 ----
    int target_idx = 0;  // 当前目标航点索引
    ROS_INFO("Starting rectangle flight...");

    while (ros::ok())
    {
        geometry_msgs::PoseStamped target = waypoints[target_idx];
        local_pos_pub.publish(target);

        // 到达判定：3D 距离 < 0.1 m
        if (distance3D(target, current_pose) < 0.1)
        {
            ROS_INFO("Reached waypoint %d, holding position for 5s", target_idx);
            ros::Time hold_start = ros::Time::now();

            // 悬停等待 3 秒
            while (ros::ok() && (ros::Time::now() - hold_start).toSec() < 3.0)
            {
                local_pos_pub.publish(target);  // 持续发送当前位置目标
                ros::spinOnce();
                rate.sleep();

                if (current_state.mode != "OFFBOARD")
                {
                    ROS_WARN("Manual interrupt, exiting hold!");
                    break;
                }
            }

            // 切换到下一个航点（循环）
            target_idx = (target_idx + 1) % waypoints.size();
            ROS_INFO("Moving to next waypoint %d", target_idx);
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
