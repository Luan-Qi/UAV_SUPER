/**
 * @file    astar_planner_node.cpp
 * @brief   A* 3D 全局路径规划器 ROS 节点
 *
 * 功能概述:
 *   1. 订阅 OctoMap 八叉树地图，转换为均匀 3D 栅格占据地图（带障碍物膨胀）
 *   2. 根据 planning_mode 获取起点和终点:
 *      - 模式0 (默认): 每次分别接收 /start_pose 和 /goal_pose
 *      - 模式1 (连续):  终点自动成为下一次规划的起点
 *      - 模式2 (里程计): 收到终点时以当前里程计位姿作为起点
 *   3. 执行 A* 搜索，发布 nav_msgs::Path 全局路径
 *   4. 规划失败时调用 /planner_fail_notify 服务通知飞控
 *
 * 订阅话题:
 *   octomap_topic    (默认 /octomap_full)           — 八叉树占据地图
 *   start_pose_topic (默认 /start_pose)             — 起点 (geometry_msgs::PoseStamped)
 *   goal_pose_topic  (默认 /goal_pose)              — 终点 (geometry_msgs::PoseStamped)
 *   odometry_topic   (默认 /odometry, 仅模式2)       — 里程计位姿 (nav_msgs::Odometry)
 *
 * 发布话题:
 *   global_path_topic (默认 /planned_path)           — 规划结果路径 (nav_msgs::Path, latched)
 *   obstacle_cloud    (可选, 需 enable_obstacle_show)  — 障碍物点云 (sensor_msgs::PointCloud2)
 *   obstacle_markers  (可选, 需 enable_obstacle_show)  — 障碍物可视化 (MarkerArray)
 *
 * 服务客户端:
 *   /planner_fail_notify (uav_px4_ctrl::TakeoffNotify) — 规划失败时调用
 */

#include "astar_searcher.h"
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <octomap_msgs/Octomap.h>
#include <nav_msgs/Path.h>
#include <octomap/OcTree.h>
#include <octomap_msgs/conversions.h>
#include "astar_show_obs.h"
#include "uav_px4_ctrl/TakeoffNotify.h"

// ============================================================================
// 全局变量
// ============================================================================

AstarPathFinder astar;               ///< A* 搜索器实例（全局单例）

ros::Publisher path_pub;             ///< 路径发布器
ros::Publisher obs_marker_pub;       ///< 障碍物 MarkerArray 发布器
ros::Publisher obs_cloud_pub;        ///< 障碍物 PointCloud2 发布器

int  inflater_size            = 2;   ///< 障碍物膨胀格数（OctoMap 叶子节点周围扩展）
int  planning_mode            = 0;   ///< 规划模式: 0=每次输入起终点 1=连续规划 2=里程计起点
bool have_start  = false;           ///< 是否已收到起点
bool have_goal   = false;           ///< 是否已收到终点
bool have_octomap = false;          ///< 是否已加载 OctoMap（仅首次加载）
bool publish_obstacle_show = false; ///< 是否发布障碍物可视化

geometry_msgs::PoseStamped start_pose;  ///< 当前起点
geometry_msgs::PoseStamped goal_pose;   ///< 当前终点

// 模式2 专用: 里程计位姿缓存
geometry_msgs::PoseStamped current_odom_pose;  ///< 最新里程计位姿
bool have_odom = false;                         ///< 是否已收到里程计数据

// ============================================================================
// 回调函数
// ============================================================================

/**
 * @brief OctoMap 回调 — 仅首次收到时执行，将八叉树转为栅格占据地图
 *
 * 处理流程:
 *   1. 将 octomap_msgs::Octomap 反序列化为 octomap::OcTree
 *   2. 根据地图边界和分辨率初始化 A* 栅格地图
 *   3. 遍历所有叶子节点，将占用节点标记为障碍物
 *   4. 对每个占用叶子做 inflater_size 格膨胀
 *   5. (可选) 发布障碍物可视化
 *
 * @note 由于 have_octomap 守卫，地图仅在节点运行期间加载一次
 */
void octomapCallback(const octomap_msgs::OctomapConstPtr &msg)
{
    if (have_octomap)
        return;

    ROS_INFO_ONCE("[astar] Received octomap, converting...");
    octomap::OcTree *octree = dynamic_cast<octomap::OcTree*>(octomap_msgs::fullMsgToMap(*msg));

    if (!octree)
    {
        ROS_ERROR("[astar] Octomap conversion failed!");
        return;
    }

    double res = octree->getResolution();

    // 获取地图边界 (metric min/max)
    double x_min, y_min, z_min, x_max, y_max, z_max;
    octree->getMetricMin(x_min, y_min, z_min);
    octree->getMetricMax(x_max, y_max, z_max);

    // 初始化 A* 栅格地图: 分辨率、世界坐标边界、栅格数量
    astar.initGridMap(res,
                      Eigen::Vector3d(x_min, y_min, z_min),
                      Eigen::Vector3d(x_max, y_max, z_max),
                      (x_max - x_min) / res,
                      (y_max - y_min) / res,
                      (z_max - z_min) / res);

    // 遍历所有叶子节点: 将占据节点标记为障碍物, 并做膨胀处理
    for (octomap::OcTree::leaf_iterator it = octree->begin_leafs();
         it != octree->end_leafs(); ++it)
    {
        if (octree->isNodeOccupied(*it))
        {
            double size = it.getSize();       // 当前八叉树节点边长
            auto coord  = it.getCoordinate(); // 节点中心世界坐标

            int steps   = std::ceil(size / res);  // 该节点覆盖的栅格步数
            int inflate = inflater_size;           // 膨胀格数

            // 遍历节点对应栅格 + 膨胀范围，全部标记为障碍
            for (int dx = -steps / 2 - inflate; dx <= steps / 2 + inflate; ++dx)
                for (int dy = -steps / 2 - inflate; dy <= steps / 2 + inflate; ++dy)
                    for (int dz = -steps / 2 - inflate; dz <= steps / 2 + inflate; ++dz)
                    {
                        double x = coord.x() + dx * res;
                        double y = coord.y() + dy * res;
                        double z = coord.z() + dz * res;
                        astar.setObs(x, y, z);
                    }
        }
    }

    // (可选) 发布障碍物可视化: PointCloud2
    if (publish_obstacle_show)
        publishObstacleCloud(&obs_cloud_pub, res,
                             x_min, y_min, z_min,
                             (x_max - x_min) / res,
                             (y_max - y_min) / res,
                             (z_max - z_min) / res,
                             [](int x, int y, int z) { return astar.getObs(x, y, z); });

    ROS_INFO("[astar] Octomap loaded to grid map.");
    delete octree;
    have_octomap = true;
}

/**
 * @brief 里程计回调 — 持续缓存最新里程计位姿（供模式2使用）
 * @param msg nav_msgs::Odometry 里程计消息
 */
void odomCallback(const nav_msgs::Odometry::ConstPtr &msg)
{
    current_odom_pose.pose   = msg->pose.pose;
    current_odom_pose.header = msg->header;
    have_odom = true;
}

/**
 * @brief 起点回调 — 接收路径起点 (仅在尚未收到起点时生效)
 * @param msg geometry_msgs::PoseStamped 起点位姿
 *
 * @note 起点一旦设置即被锁定,直到规划完成后由主循环根据模式重置
 */
void startCallback(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    if (!have_start)
    {
        start_pose = *msg;
        have_start = true;
        ROS_INFO("[astar] Received start pose [%.1f,%.1f,%.1f]",
                 start_pose.pose.position.x,
                 start_pose.pose.position.y,
                 start_pose.pose.position.z);
    }
}

/**
 * @brief 终点回调 — 接收路径终点
 *
 * 标准行为 (模式0/1): 仅设置 goal_pose
 * 模式2: 同时将当前里程计位姿设为起点（实现"当前位置→目标"的实时规划）
 *
 * @param msg geometry_msgs::PoseStamped 终点位姿
 */
void goalCallback(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    if (!have_goal)
    {
        goal_pose = *msg;
        have_goal = true;
        ROS_INFO("[astar] Received goal pose [%.1f,%.1f,%.1f]",
                 goal_pose.pose.position.x,
                 goal_pose.pose.position.y,
                 goal_pose.pose.position.z);

        // 模式2: 收到目标点时, 使用当前里程计位姿作为起点
        if (planning_mode == 2)
        {
            if (have_odom)
            {
                start_pose = current_odom_pose;
                have_start = true;
                ROS_INFO("[astar] Mode 2: use odometry as start [%.1f,%.1f,%.1f]",
                         start_pose.pose.position.x,
                         start_pose.pose.position.y,
                         start_pose.pose.position.z);
            }
            else
            {
                ROS_WARN("[astar] Mode 2: no odometry received yet, cannot set start pose");
            }
        }
    }
}

// ============================================================================
// 主函数
// ============================================================================

int main(int argc, char **argv)
{
    ros::init(argc, argv, "astar_planner_node");
    ros::NodeHandle nh("~");  // 私有命名空间, 参数通过 ~param_name 读取

    // -------- 读取 ROS 参数 --------
    std::string octomap_topic;
    std::string start_pose_topic;
    std::string goal_pose_topic;
    std::string planned_path_topic;
    std::string odometry_topic;
    int max_search_iter;
    double max_search_time;

    nh.param<std::string>("octomap_topic",       octomap_topic,       "/octomap_full");
    nh.param<std::string>("start_pose_topic",    start_pose_topic,    "/start_pose");
    nh.param<std::string>("goal_pose_topic",     goal_pose_topic,     "/goal_pose");
    nh.param<std::string>("global_path_topic",   planned_path_topic,  "/planned_path");
    nh.param<bool>       ("enable_obstacle_show", publish_obstacle_show, false);
    nh.param<int>        ("inflater_size",        inflater_size,       2);
    nh.param<int>        ("planning_mode",        planning_mode,       0);
    nh.param<std::string>("odometry_topic",       odometry_topic,      "/odometry");
    nh.param<int>        ("max_search_iterations", max_search_iter,   100000);
    nh.param<double>     ("max_search_time",       max_search_time,   0.5);

    // 设置搜索保护参数到 A* 搜索器
    astar.setMaxSearchIterations(max_search_iter);
    astar.setMaxSearchTime(max_search_time);

    // -------- 初始化订阅 --------
    ros::Subscriber octo_sub  = nh.subscribe(octomap_topic,    10, octomapCallback);
    ros::Subscriber start_sub = nh.subscribe(start_pose_topic, 10, startCallback);
    ros::Subscriber goal_sub  = nh.subscribe(goal_pose_topic,  10, goalCallback);

    // -------- 打印当前规划模式 --------
    switch (planning_mode)
    {
        case 0:
            ROS_INFO("[astar] Planning mode: 0 (standard) - each plan requires new start + goal");
            break;
        case 1:
            ROS_INFO("[astar] Planning mode: 1 (continuous) - goal becomes next start");
            break;
        case 2:
            ROS_INFO("[astar] Planning mode: 2 (odometry) - current odometry used as start");
            break;
        default:
            ROS_WARN("[astar] Unknown planning_mode=%d, falling back to mode 0", planning_mode);
            planning_mode = 0;
            break;
    }

    // 模式2: 订阅里程计话题以持续获取当前位姿
    ros::Subscriber odom_sub;
    if (planning_mode == 2)
    {
        odom_sub = nh.subscribe(odometry_topic, 10, odomCallback);
        ROS_INFO("[astar] Mode 2: subscribing to odometry topic [%s]", odometry_topic.c_str());
    }

    // -------- 服务客户端: 规划失败通知 --------
    ros::ServiceClient planner_failed_client =
        nh.serviceClient<uav_px4_ctrl::TakeoffNotify>("/planner_fail_notify");

    // -------- 初始化发布: 路径 (latched=true, 新订阅者立即收到最后一条) --------
    path_pub = nh.advertise<nav_msgs::Path>(planned_path_topic, 10, true);

    // 障碍物可视化发布器 (仅在 enable_obstacle_show=true 时初始化)
    if (publish_obstacle_show)
        obs_marker_pub = nh.advertise<visualization_msgs::MarkerArray>("obstacle_markers", 10, true);
        obs_cloud_pub  = nh.advertise<sensor_msgs::PointCloud2>("obstacle_cloud", 10, true);

    ROS_INFO("[astar] Waiting for %s%s%s",
             have_start  ? "" : "start_pose,",
             have_goal   ? "" : "goal_pose,",
             have_octomap ? "" : "octomap");

    // -------- 主循环: 等待触发条件 → 规划 → 发布 --------
    ros::Rate rate(10.0);  // 10 Hz
    while (ros::ok())
    {
        ros::spinOnce();  // 处理所有回调队列
        
        if(have_start && have_goal && !have_octomap) ROS_WARN_THROTTLE(1.0, "[astar] Check your octomap!");

        // 触发条件: 起点 + 终点 + 地图 三者齐全
        if (have_start && have_goal && have_octomap)
        {
            ROS_INFO("[astar] start pose [%.1f,%.1f,%.1f], goal pose [%.1f,%.1f,%.1f]",
                     start_pose.pose.position.x, start_pose.pose.position.y, start_pose.pose.position.z,
                     goal_pose.pose.position.x,  goal_pose.pose.position.y,  goal_pose.pose.position.z);
            ROS_INFO("[astar] Start planning...");

            // 提取起点/终点的 Eigen 向量
            Eigen::Vector3d start(start_pose.pose.position.x,
                                  start_pose.pose.position.y,
                                  start_pose.pose.position.z);
            Eigen::Vector3d goal(goal_pose.pose.position.x,
                                 goal_pose.pose.position.y,
                                 goal_pose.pose.position.z);

            // -------- 执行 A* 搜索 --------
            astar.AstarGraphSearch(start, goal);
            std::vector<Eigen::Vector3d> path_pts = astar.getPath();

            // -------- 规划失败: path_pts.size() <= 1 表示无有效路径 --------
            if (path_pts.size() <= 1)
            {
                ROS_ERROR("[astar] A* failed to find a path!");
                have_start = false;  // 重置起点, 等待重新输入

                // 通知飞控规划失败
                uav_px4_ctrl::TakeoffNotify srv;
                srv.request.takeoff_done = true;
                if (!planner_failed_client.call(srv))
                    have_octomap = false;  // 服务调用失败时允许重新加载地图
            }
            // -------- 规划成功 --------
            else
            {
                // 构建 nav_msgs::Path 消息
                nav_msgs::Path path_msg;
                path_msg.header.stamp    = ros::Time::now();
                path_msg.header.frame_id = "map";  // 固定地图坐标系

                for (auto &pt : path_pts)
                {
                    geometry_msgs::PoseStamped p;
                    p.pose.position.x    = pt(0);
                    p.pose.position.y    = pt(1);
                    p.pose.position.z    = pt(2);
                    p.pose.orientation.w = 1.0;  // 单位四元数 (无旋转)
                    path_msg.poses.push_back(p);
                }

                path_pub.publish(path_msg);
                ROS_INFO("[astar] Published path with %zu points.", path_msg.poses.size());

                // -------- 根据模式处理起点状态 --------
                if (planning_mode == 1)
                {
                    // 模式1 (连续规划): 终点自动成为下一次的起点
                    start_pose = goal_pose;
                    // have_start 保持 true
                }
                else
                {
                    // 模式0 (标准) / 模式2 (里程计): 每次都需要新的起点
                    have_start = false;
                }
            }

            // 规划结束: 重置终点标志, 清理已使用的栅格节点状态
            have_goal = false;
            astar.resetUsedGrids();

            ROS_INFO(" ");
            ROS_INFO("[astar] Waiting for next %s%s%s",
                     have_start  ? "" : "start_pose,",
                     have_goal   ? "" : "goal_pose,",
                     have_octomap ? "" : "octomap");
        }

        rate.sleep();
    }

    return 0;
}

