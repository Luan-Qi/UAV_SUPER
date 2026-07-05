/**
 * @file    fforward_path_follower.cpp
 * @brief   前馈路径跟随器 — 带前瞻 (lookahead) 和球体到达判断的路径跟踪
 *
 * 功能概述:
 *   与 fast_path_follower 类似, 但增加了两个关键增强:
 *
 *   1. Lookahead (前瞻):
 *      不直接发布路径上的原始点, 而是沿路径方向向前投影 lookahead_dist 距离,
 *      发布投影后的目标点。这使无人机能提前转向, 平滑跟踪曲线路径。
 *
 *   2. Sphere-based Arrival Check (球体到达判断):
 *      不使用简单的"距目标点 < 阈值"判断, 而是构建一个动态球体:
 *        - 球心: 沿当前方向向前延伸 remaining_dist
 *        - 半径: remaining_dist + goal_distance_threshold
 *      当无人机进入球体即判定到达。这比固定距离判断更鲁棒,
 *      尤其在大步长跳跃时避免了"穿过但未触发"的问题。
 *
 * 订阅/发布: 与 fast_path_follower 相同
 *
 * 参数:
 *   goal_distance_threshold  — 到达判定基础距离 (默认 0.5m)
 *   path_step                — 路径点跳跃步长 (默认 5)
 *   lookahead_distance       — 前瞻距离 (默认 1.2m)
 *   publish_rate             — 控制循环频率 (默认 5 Hz)
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

nav_msgs::Path local_path;
nav_msgs::Odometry cur_map_to_odom;
geometry_msgs::PoseStamped current_goal;
geometry_msgs::PoseStamped current_pose;
geometry_msgs::PoseStamped prev_goal;      ///< 上一个目标点 (用于计算方向)
Eigen::Matrix4d T_odom_to_map;

bool has_path        = false;
bool has_pose        = false;
bool has_map_to_odom = false;
bool need_finished   = false;
bool has_finished    = false;
int  current_index   = 0;
bool first_start     = false;

double goal_distance_threshold = 0.5;   ///< 到达判定基础距离 (m)
int    path_step       = 5;              ///< 每隔几个点发送一次目标
double publish_rate    = 5.0;            ///< 控制循环频率 (Hz)
double lookahead_dist  = 1.2;            ///< 前瞻距离 (m)

std::string path_topic, odom_topic, goal_topic, map_to_odom_topic;

visualization_msgs::MarkerArray marker_array;
int id = 0;

// ============================================================================
// 坐标变换工具函数
// ============================================================================

/// Pose -> Eigen::Matrix4d
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

/// Eigen::Matrix4d -> Pose
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
    local_path    = *msg;
    has_path      = true;
    has_finished  = false;
    current_index = path_step;
    ROS_INFO("[path_follower_3d] Received new path with %lu points.", msg->poses.size());
}

/// 里程计回调
void cbOdom(const nav_msgs::Odometry::ConstPtr& msg)
{
    current_pose.pose   = msg->pose.pose;
    current_pose.header = msg->header;
    has_pose = true;
}

/// map->odom 变换回调
void cbMapToOdom(const nav_msgs::Odometry::ConstPtr &msg)
{
    cur_map_to_odom  = *msg;
    T_odom_to_map    = poseToMatrix(cur_map_to_odom.pose.pose).inverse();
    has_map_to_odom  = true;
}

/// 3D 欧氏距离
double distance3D(const geometry_msgs::PoseStamped& a,
                  const geometry_msgs::PoseStamped& b)
{
    double dx = a.pose.position.x - b.pose.position.x;
    double dy = a.pose.position.y - b.pose.position.y;
    double dz = a.pose.position.z - b.pose.position.z;
    return std::sqrt(dx * dx + dy * dy + dz * dz);
}

/// 向量归一化
Eigen::Vector3d normalize(const Eigen::Vector3d &v)
{
    if (v.norm() < 1e-6) return Eigen::Vector3d(0, 0, 0);
    return v.normalized();
}

// ============================================================================
// 球体到达判断 (核心增强)
// ============================================================================

/**
 * @brief 球体到达判断 — 比固定距离阈值更鲁棒的到达检测
 *
 * 构建球体:
 *   - 方向 dir = normalize(cur_goal - prev_goal)
 *   - 剩余距离 remain_dist = |path_end - cur_goal|
 *   - 球心 center = cur_goal + dir * remain_dist
 *   - 球半径 radius = remain_dist + goal_distance_threshold
 *
 * 当无人机进入球体: |drone_pos - center| < radius → 判定到达
 *
 * 对于前期 (index < path_step*2): 回退到传统距离判断
 *
 * @return true=到达当前目标点, 应发布下一个
 */
bool is_in_sphere()
{
    bool reached_goal = false;

    if (current_index >= path_step * 2)
    {
        // 成熟阶段: 使用球体判断
        geometry_msgs::PoseStamped prev_goal =
            local_path.poses[current_index - path_step * 2];

        Eigen::Vector3d prev(prev_goal.pose.position.x,
                             prev_goal.pose.position.y,
                             prev_goal.pose.position.z);
        Eigen::Vector3d cur(current_goal.pose.position.x,
                            current_goal.pose.position.y,
                            current_goal.pose.position.z);
        Eigen::Vector3d last(local_path.poses.back().pose.position.x,
                             local_path.poses.back().pose.position.y,
                             local_path.poses.back().pose.position.z);

        Eigen::Vector3d dir = normalize(cur - prev);

        // 当前目标到最终点的剩余距离
        double remain_dist = (last - cur).norm();

        // 球心: 沿当前方向延伸 remain_dist
        Eigen::Vector3d center = cur + dir * remain_dist;

        // 球半径
        double radius = remain_dist + goal_distance_threshold;

        // 无人机当前位置
        Eigen::Vector3d cur_pos(current_pose.pose.position.x,
                                current_pose.pose.position.y,
                                current_pose.pose.position.z);

        double dist_to_center = (cur_pos - center).norm();
        if (dist_to_center < radius)
            reached_goal = true;

        // 调试可视化
        viz_utils::addSphere(marker_array, center, radius, id++);
        ROS_INFO_THROTTLE(0.5,
            "[path_follower_3d] sphere: remain=%.2f radius=%.2f dist_center=%.2f -> %s",
            remain_dist, radius, dist_to_center,
            reached_goal ? "ENTER" : "OUT");
    }
    else
    {
        // 初期阶段: 使用传统距离判断
        Eigen::Vector3d center(current_goal.pose.position.x,
                               current_goal.pose.position.y,
                               current_goal.pose.position.z);
        viz_utils::addSphere(marker_array, center, goal_distance_threshold, id++);

        if (distance3D(current_pose, current_goal) < goal_distance_threshold)
            reached_goal = true;
    }

    return reached_goal;
}

// ============================================================================
// 主函数
// ============================================================================

int main(int argc, char** argv)
{
    ros::init(argc, argv, "path_follower");
    ros::NodeHandle nh("~");

    // -------- 读取参数 --------
    nh.param("goal_distance_threshold", goal_distance_threshold, 0.5);
    nh.param("path_step",               path_step,               5);
    nh.param("publish_rate",            publish_rate,            5.0);
    nh.param("lookahead_distance",      lookahead_dist,          1.2);
    nh.param<std::string>("path_topic",        path_topic,        "/global_path");
    nh.param<std::string>("odom_topic",        odom_topic,        "/localization");
    nh.param<std::string>("goal_topic",        goal_topic,        "/move_base_simple/goal");
    nh.param<std::string>("map_to_odom_topic", map_to_odom_topic, "/map_to_odom");

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

        if (has_path && has_pose && has_map_to_odom)
        {
            if (!has_finished)
            {
                // 到达判断: 球体检测 或 首次启动
                if (is_in_sphere() || !first_start)
                {
                    first_start = true;

                    // 取路径上的下一个原始目标点
                    current_goal = local_path.poses[current_index];

                    // 计算上一个目标点 (用于方向计算)
                    if (current_index >= path_step)
                        prev_goal = local_path.poses[current_index - path_step];
                    else
                        prev_goal = current_goal;

                    // -------- Lookahead 前瞻计算 --------
                    Eigen::Vector3d P_prev(prev_goal.pose.position.x,
                                           prev_goal.pose.position.y,
                                           prev_goal.pose.position.z);
                    Eigen::Vector3d P_cur(current_goal.pose.position.x,
                                          current_goal.pose.position.y,
                                          current_goal.pose.position.z);
                    Eigen::Vector3d P_end(local_path.poses.back().pose.position.x,
                                          local_path.poses.back().pose.position.y,
                                          local_path.poses.back().pose.position.z);

                    // 方向向量
                    Eigen::Vector3d dir = P_cur - P_prev;
                    if (dir.norm() > 1e-3)
                        dir.normalize();
                    else
                        dir = Eigen::Vector3d(0, 0, 0);

                    // 前瞻点: 沿方向向前投影, 但不超过路径终点
                    Eigen::Vector3d P_lookahead;
                    double dist_to_end = (P_end - P_cur).norm();
                    if (dist_to_end > lookahead_dist)
                        P_lookahead = P_cur + dir * lookahead_dist;
                    else
                        P_lookahead = P_end;  // 接近终点时直接指向终点

                    // 构建前瞻目标点
                    geometry_msgs::PoseStamped lookahead_goal = current_goal;
                    lookahead_goal.pose.position.x = P_lookahead.x();
                    lookahead_goal.pose.position.y = P_lookahead.y();
                    lookahead_goal.pose.position.z = P_lookahead.z();

                    // 坐标变换: map -> odom
                    Eigen::Matrix4d T_global = poseToMatrix(lookahead_goal.pose);
                    Eigen::Matrix4d T_local  = T_odom_to_map * T_global;

                    geometry_msgs::PoseStamped current_goal_local;
                    current_goal_local.header = current_goal.header;
                    current_goal_local.pose   = matrixToPose(T_local);

                    pub_goal.publish(current_goal_local);

                    ROS_INFO("[path_follower_3d] New goal #%d: (x=%.1f, y=%.1f, z=%.1f)",
                             current_index,
                             current_goal_local.pose.position.x,
                             current_goal_local.pose.position.y,
                             current_goal_local.pose.position.z);

                    // 终点处理
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
                        current_index = local_path.poses.size() - 1;
                        need_finished = true;
                    }
                }

                // 发布调试可视化
                pub_marker_array.publish(marker_array);
                marker_array.markers.clear();
                id = 0;
            }
            else
            {
                // 路径已完成 → 重置
                ROS_INFO("[path_follower_3d] Path finished.");
                has_path      = false;
                current_index = path_step;
                first_start   = false;
            }
        }

        rate.sleep();
    }

    return 0;
}
