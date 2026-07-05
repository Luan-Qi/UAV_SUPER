/**
 * @file    fast_path_follower.cpp
 * @brief   快速路径跟随器 — 基于距离阈值的简单路径跟踪
 *
 * 功能概述:
 *   接收全局路径和无人机里程计位姿, 将路径点逐个发布为飞控目标点:
 *   1. 沿路径按 path_step 步长跳跃前进
 *   2. 当无人机距当前目标点 < goal_distance_threshold 时, 发布下一个目标
 *   3. 将目标从 map 坐标系转换到 odom 坐标系 (通过 /map_to_odom 变换)
 *   4. 到达路径终点后自动重置, 等待新路径
 *
 * 与 fforward_path_follower 的区别:
 *   - fast_path_follower: 简单欧氏距离判断, 无前瞻 (lookahead)
 *   - fforward_path_follower: 球体到达判断 + lookahead 前瞻距离
 *
 * 订阅:
 *   path_topic          (nav_msgs::Path)        — 全局路径
 *   odom_topic          (nav_msgs::Odometry)    — 无人机当前里程计
 *   map_to_odom_topic   (nav_msgs::Odometry)    — map->odom 坐标变换
 *
 * 发布:
 *   goal_topic          (geometry_msgs::PoseStamped) — 当前目标点 (odom 系)
 *   /path_follower_debug_array (MarkerArray)         — 调试可视化
 *
 * 参数:
 *   goal_distance_threshold  — 到达判定距离 (默认 0.5m)
 *   path_step                — 路径点跳跃步长 (默认 1, 每个点都发)
 *   publish_rate             — 控制循环频率 (默认 5 Hz)
 *   repub_distance_threshold — 变换更新时重发目标的距离阈值 (默认 2.0m, 已注释)
 */

#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <visualization_msgs/MarkerArray.h>
#include <Eigen/Dense>
#include <cmath>
#include "visual_path_follower.h"

// ============================================================================
// 全局状态变量
// ============================================================================

nav_msgs::Path local_path;               ///< 缓存的最新全局路径
nav_msgs::Odometry cur_map_to_odom;      ///< map->odom 变换 (pose = T_map_odom)
geometry_msgs::PoseStamped current_goal; ///< 当前发布的目标点
geometry_msgs::PoseStamped current_pose; ///< 无人机当前位姿
Eigen::Matrix4d T_odom_to_map;           ///< odom->map 变换矩阵

bool has_path        = false;  ///< 是否已收到路径
bool has_pose        = false;  ///< 是否已收到里程计
bool has_map_to_odom = false;  ///< 是否已收到坐标变换
bool new_map_to_odom = false;  ///< 是否有新的变换更新
bool need_finished   = false;  ///< 是否需要在下一次发布后结束
bool has_finished    = false;  ///< 路径是否已走完
int  current_index   = 0;      ///< 当前路径点索引
bool first_start     = false;  ///< 是否已开始跟踪

double goal_distance_threshold  = 0.5;   ///< 到达目标的判定距离 (m)
double repub_distance_threshold = 2.0;   ///< 变换更新时重发目标的判定距离 (m)
int    path_step    = 5;                 ///< 每隔几个点发送一次目标
double publish_rate = 5.0;              ///< 控制循环频率 (Hz)

std::string path_topic, odom_topic, goal_topic, map_to_odom_topic;

visualization_msgs::MarkerArray marker_array;  ///< 调试可视化标记
int id = 0;

// ============================================================================
// 坐标变换工具函数
// ============================================================================

/// Pose (geometry_msgs) -> Eigen::Matrix4d
Eigen::Matrix4d poseToMatrix(const geometry_msgs::Pose &pose)
{
    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
    Eigen::Quaterniond q(pose.orientation.w, pose.orientation.x,
                         pose.orientation.y, pose.orientation.z);
    Eigen::Matrix3d R = q.toRotationMatrix();
    T.block<3, 3>(0, 0) = R;
    T(0, 3) = pose.position.x;
    T(1, 3) = pose.position.y;
    T(2, 3) = pose.position.z;
    return T;
}

/// Eigen::Matrix4d -> Pose (geometry_msgs)
geometry_msgs::Pose matrixToPose(const Eigen::Matrix4d &T)
{
    geometry_msgs::Pose pose;
    Eigen::Matrix3d R = T.block<3, 3>(0, 0);
    Eigen::Quaterniond q(R);
    pose.position.x    = T(0, 3);
    pose.position.y    = T(1, 3);
    pose.position.z    = T(2, 3);
    pose.orientation.x = q.x();
    pose.orientation.y = q.y();
    pose.orientation.z = q.z();
    pose.orientation.w = q.w();
    return pose;
}

// ============================================================================
// 回调函数
// ============================================================================

/// 路径回调 — 接收新路径, 重置跟踪状态
void cbPath(const nav_msgs::Path::ConstPtr& msg)
{
    if (msg->poses.empty()) {
        ROS_WARN("Received empty path!");
        return;
    }
    local_path   = *msg;
    has_path     = true;
    has_finished = false;
    current_index = path_step;

    ROS_INFO("[path_follower_3d] Received new path with %lu points.",
             msg->poses.size());

    if (!has_pose || !has_map_to_odom)
        ROS_INFO("[path_follower_3d] But not get %s%s yet.",
                 has_pose ? "" : "pose,",
                 has_map_to_odom ? "" : "MapToOdom");
}

/// 里程计回调 — 缓存无人机当前位姿
void cbOdom(const nav_msgs::Odometry::ConstPtr& msg)
{
    current_pose.pose   = msg->pose.pose;
    current_pose.header = msg->header;
    has_pose = true;
}

/// 坐标变换回调 — 更新 map->odom 变换矩阵
void cbMapToOdom(const nav_msgs::Odometry::ConstPtr &msg)
{
    cur_map_to_odom = *msg;
    // T_odom_to_map = T_map_to_odom^{-1}
    T_odom_to_map    = poseToMatrix(cur_map_to_odom.pose.pose).inverse();
    has_map_to_odom  = true;
    if (!new_map_to_odom) new_map_to_odom = true;
}

// ============================================================================
// 辅助函数
// ============================================================================

/// 计算两个 PoseStamped 之间的 3D 欧氏距离
double distance3D(const geometry_msgs::PoseStamped& a,
                  const geometry_msgs::PoseStamped& b)
{
    double dx = a.pose.position.x - b.pose.position.x;
    double dy = a.pose.position.y - b.pose.position.y;
    double dz = a.pose.position.z - b.pose.position.z;
    return std::sqrt(dx * dx + dy * dy + dz * dz);
}

/**
 * @brief 发布目标点 — 将 map 系目标转换到 odom 系后发布
 *
 * @param publisher 目标点发布器
 * @param goal      map 系下的目标点
 * @param trans     T_odom_to_map 变换矩阵
 */
void publishGoalWithTransform(ros::Publisher& publisher,
                               const geometry_msgs::PoseStamped& goal,
                               const Eigen::Matrix4d& trans)
{
    // 添加调试可视化球体
    Eigen::Vector3d center(goal.pose.position.x,
                           goal.pose.position.y,
                           goal.pose.position.z);
    viz_utils::addSphere(marker_array, center, goal_distance_threshold, id++);

    // 坐标变换: map -> odom
    Eigen::Matrix4d T_global = poseToMatrix(goal.pose);
    Eigen::Matrix4d T_local  = trans * T_global;

    geometry_msgs::PoseStamped current_goal_local;
    current_goal_local.header = current_goal.header;
    current_goal_local.pose   = matrixToPose(T_local);

    publisher.publish(current_goal_local);

    ROS_INFO("[path_follower_3d] New goal #%d: (x=%.1f, y=%.1f, z=%.1f)",
             current_index,
             current_goal_local.pose.position.x,
             current_goal_local.pose.position.y,
             current_goal_local.pose.position.z);
}

// ============================================================================
// 主函数
// ============================================================================

int main(int argc, char** argv)
{
    ros::init(argc, argv, "path_follower");
    ros::NodeHandle nh("~");

    // -------- 读取参数 --------
    nh.param("goal_distance_threshold",  goal_distance_threshold,  0.5);
    nh.param("path_step",                path_step,                1);
    nh.param("publish_rate",             publish_rate,             5.0);
    nh.param<std::string>("path_topic",          path_topic,          "/global_path");
    nh.param<std::string>("odom_topic",          odom_topic,          "/localization");
    nh.param<std::string>("goal_topic",          goal_topic,          "/move_base_simple/goal");
    nh.param<std::string>("map_to_odom_topic",   map_to_odom_topic,   "/map_to_odom");

    // -------- 初始化通信 --------
    ros::Subscriber sub_path         = nh.subscribe(path_topic,        1, cbPath);
    ros::Subscriber sub_odom         = nh.subscribe(odom_topic,        1, cbOdom);
    ros::Subscriber sub_map_to_odom  = nh.subscribe(map_to_odom_topic, 3, cbMapToOdom);
    ros::Publisher  pub_goal         = nh.advertise<geometry_msgs::PoseStamped>(goal_topic, 1);
    ros::Publisher  pub_marker_array = nh.advertise<visualization_msgs::MarkerArray>(
        "/path_follower_debug_array", 10);

    ros::Rate rate(publish_rate);

    ROS_INFO("[path_follower_3d] Node started. Waiting for /path, /odom, /T...");
    current_index = path_step;

    // -------- 主循环 --------
    while (ros::ok())
    {
        ros::spinOnce();

        // 等待所有输入就绪
        if (has_path && has_pose && has_map_to_odom)
        {
            if (!has_finished)
            {
                // 距离判断: 到达当前目标点 或 首次启动
                double dist = distance3D(current_pose, current_goal);
                if (dist < goal_distance_threshold || !first_start)
                {
                    first_start = true;

                    // 取路径上的下一个目标点
                    current_goal = local_path.poses[current_index];
                    publishGoalWithTransform(pub_goal, current_goal,
                                              T_odom_to_map);

                    // 处理终点到达逻辑
                    if (need_finished)
                    {
                        has_finished  = true;
                        need_finished = false;
                        continue;
                    }

                    // 前进到下一个目标索引
                    current_index += path_step;
                    if (current_index >= (int)local_path.poses.size())
                    {
                        current_index = local_path.poses.size() - 1;  // 钳位到终点
                        need_finished = true;  // 标记: 下一次发布后结束
                    }
                }

                pub_marker_array.publish(marker_array);
            }
            else
            {
                // 路径已完成 → 重置状态, 等待新路径
                ROS_INFO("[path_follower_3d] Path finished.");
                has_path      = false;
                current_index = path_step;
                marker_array.markers.clear();
                id = 0;
                first_start = false;
            }
        }

        rate.sleep();
    }

    return 0;
}
