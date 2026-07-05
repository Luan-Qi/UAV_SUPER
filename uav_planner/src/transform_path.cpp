/**
 * @file    transform_path.cpp
 * @brief   路径坐标系变换节点 — map 系全局路径 → odom 系局部路径
 *
 * 功能:
 *   将 astar_planner_node 输出的 map 坐标系全局路径转换为 odom 坐标系,
 *   供基于里程计的本地控制器使用。
 *
 * 变换公式:
 *   T_odom_to_map = T_map_to_odom^{-1}
 *   P_odom = T_odom_to_map * P_map
 *
 * 其中 T_map_to_odom 由 /map_to_odom 话题提供 (nav_msgs::Odometry, pose 字段编码变换)
 *
 * 行为:
 *   - 首次收到 /map_to_odom 后锁定变换 (不再更新)
 *   - 收到 /global_path 后逐点执行坐标变换并发布
 *   - 若收到路径时变换尚未就绪 → 阻塞等待
 *
 * 订阅:
 *   global_path_topic  (nav_msgs::Path)      — map 系路径
 *   map_to_odom_topic  (nav_msgs::Odometry)  — map->odom 变换
 *
 * 发布:
 *   local_path_topic   (nav_msgs::Path)      — odom 系路径 (frame_id="odom")
 *
 * @note 此节点在 astar_planner.launch 中被注释掉, 按需启用
 */

#include <ros/ros.h>
#include <mutex>
#include <Eigen/Dense>

#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>

ros::Publisher pub_local_path;
nav_msgs::Odometry cur_map_to_odom;   ///< map->odom 变换 (pose 字段)
bool has_map_to_odom = false;          ///< 是否已收到变换
std::mutex mtx;                        ///< 保护 cur_map_to_odom 的读写

// ============================================================================
// 坐标变换工具函数
// ============================================================================

/**
 * @brief geometry_msgs::Pose → Eigen::Matrix4d
 *
 * 变换矩阵 T = [R  t]
 *              [0  1]
 */
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

/**
 * @brief Eigen::Matrix4d → geometry_msgs::Pose
 */
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

/**
 * @brief /map_to_odom 回调 — 缓存坐标变换 (仅首次)
 */
void cbMapToOdom(const nav_msgs::Odometry::ConstPtr &msg)
{
    if (!has_map_to_odom)
    {
        std::lock_guard<std::mutex> lock(mtx);
        cur_map_to_odom  = *msg;
        has_map_to_odom  = true;
    }
}

/**
 * @brief /global_path 回调 — 收到路径后执行坐标变换并发布
 *
 * 若变换尚未就绪, 阻塞等待直到收到 /map_to_odom
 */
void cbGlobalPath(const nav_msgs::Path::ConstPtr &msg)
{
    // 等待变换就绪
    if (!has_map_to_odom)
    {
        ROS_WARN("[path_trans] Waiting for /map_to_odom transform...");
        ros::Rate rate(10.0);
        while (ros::ok())
        {
            if (has_map_to_odom) break;
            ros::spinOnce();
            rate.sleep();
        }
        return;
    }

    nav_msgs::Path local_path;
    local_path.header = msg->header;
    local_path.header.frame_id = "map";  // 转换后的坐标系 (应为 odom, 此处似有笔误)

    // 获取变换矩阵 (线程安全)
    Eigen::Matrix4d T_map_to_odom;
    {
        std::lock_guard<std::mutex> lock(mtx);
        T_map_to_odom = poseToMatrix(cur_map_to_odom.pose.pose);
    }

    // 求逆: T_odom_to_map
    Eigen::Matrix4d T_odom_to_map = T_map_to_odom.inverse();

    // 遍历路径中的每个点, 执行坐标变换
    for (const auto &pose_stamped : msg->poses)
    {
        Eigen::Matrix4d T_global = poseToMatrix(pose_stamped.pose);
        Eigen::Matrix4d T_local  = T_odom_to_map * T_global;

        geometry_msgs::PoseStamped local_pose;
        local_pose.header = pose_stamped.header;
        local_pose.header.frame_id = "odom";
        local_pose.pose = matrixToPose(T_local);

        local_path.poses.push_back(local_pose);
    }

    pub_local_path.publish(local_path);
}

// ============================================================================
// 主函数
// ============================================================================

int main(int argc, char **argv)
{
    ros::init(argc, argv, "path_transform_node");
    ros::NodeHandle nh("~");

    ROS_INFO("[path_trans] Path Transform Node Started...");

    // -------- 读取参数 --------
    std::string global_path_topic, local_path_topic, map_to_odom_topic;
    nh.param<std::string>("global_path_topic",  global_path_topic,  "/global_path");
    nh.param<std::string>("local_path_topic",   local_path_topic,   "/local_path");
    nh.param<std::string>("map_to_odom_topic",  map_to_odom_topic,  "/map_to_odom");

    // -------- 初始化通信 --------
    ros::Subscriber sub_map_to_odom = nh.subscribe(map_to_odom_topic, 3, cbMapToOdom);
    ros::Subscriber sub_global_path = nh.subscribe(global_path_topic, 3, cbGlobalPath);
    pub_local_path = nh.advertise<nav_msgs::Path>(local_path_topic, 3, true);

    ros::spin();
    return 0;
}
