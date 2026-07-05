/**
 * @file    greedy_path_simplify.cpp
 * @brief   贪心碰撞感知路径简化节点
 *
 * 算法原理:
 *   从起点开始, 贪心地向前搜索能直达的最远路径点:
 *   1. 对路径上的每个当前点, 从最远的后续点开始向前扫描
 *   2. 检查当前点与候选点之间的直线段是否碰撞 (KD-Tree 半径搜索)
 *   3. 若不碰撞 → 连接之, 跳过中间所有冗余点
 *   4. 若碰撞 → 回退到上一个安全点
 *
 * 与 DP 简化的区别: 本算法考虑了障碍物信息, 确保简化后的路径仍然安全
 *
 * 碰撞检测:
 *   沿直线段以 check_step_size 为步长采样, 对每个采样点在 obstacle cloud 中
 *   做半径为 collision_radius 的最近邻搜索 — 若找到障碍点则判定为碰撞
 *
 * 订阅:
 *   input_topic  (nav_msgs::Path)             — 待简化的原始路径
 *   map_topic    (sensor_msgs::PointCloud2)   — 障碍物点云 (用于碰撞检测)
 *
 * 发布:
 *   output_topic (nav_msgs::Path)             — 简化后的路径
 *
 * 参数:
 *   check_step_size  — 碰撞检测采样步长 (默认 0.1m, 建议设为地图分辨率的一半)
 *   collision_radius — 碰撞判定半径 (默认 0.2m, 建议略大于地图分辨率)
 */

#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseStamped.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>

#include <vector>
#include <cmath>
#include <string>
#include <iomanip>

class PathSmoother3D
{
private:
    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;
    ros::Subscriber path_sub_;
    ros::Subscriber map_sub_;
    ros::Publisher  path_pub_;

    std::string sub_path_topic_;
    std::string sub_map_topic_;
    std::string pub_topic_name_;

    double check_step_size_;   ///< 碰撞检测沿直线的采样步长 (m)
    double collision_radius_;  ///< 碰撞判定的搜索半径 (m)

    pcl::PointCloud<pcl::PointXYZ>::Ptr map_cloud_;  ///< 障碍物点云
    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree_;          ///< KD-Tree 用于快速最近邻搜索

    bool map_received_;   ///< 是否已收到地图
    bool path_received_;  ///< 是否已收到路径

    nav_msgs::Path path_msg_;  ///< 缓存的最新路径

public:
    PathSmoother3D()
        : pnh_("~")
        , map_received_(false)
        , map_cloud_(new pcl::PointCloud<pcl::PointXYZ>)
    {
        // -------- 读取参数 --------
        pnh_.param<std::string>("input_topic",      sub_path_topic_,  "/a_star_path");
        pnh_.param<std::string>("map_topic",        sub_map_topic_,   "/grid_map_vis/point_cloud");
        pnh_.param<std::string>("output_topic",     pub_topic_name_,  "/smoothed_path");
        pnh_.param<double>     ("check_step_size",  check_step_size_,  0.1);
        pnh_.param<double>     ("collision_radius", collision_radius_, 0.2);

        // -------- 初始化通信 --------
        path_sub_ = nh_.subscribe(sub_path_topic_, 1,
                                  &PathSmoother3D::pathCallback, this);
        map_sub_  = nh_.subscribe(sub_map_topic_,  1,
                                  &PathSmoother3D::mapCallback,  this);
        path_pub_ = nh_.advertise<nav_msgs::Path>(pub_topic_name_, 1, true);

        ROS_INFO("[PathSimplify] Greedy Path Smoother Started.");
        ROS_INFO("[PathSimplify] Waiting for map on: %s", sub_map_topic_.c_str());
    }

    /**
     * @brief 地图回调 — 接收障碍物点云, 构建 KD-Tree
     *
     * 收到地图后, 若有缓存的路径则立即执行简化
     */
    void mapCallback(const sensor_msgs::PointCloud2::ConstPtr& msg)
    {
        if (msg->data.empty()) return;

        // ROS 消息 → PCL 点云
        pcl::fromROSMsg(*msg, *map_cloud_);

        // 构建 KD-Tree 加速碰撞查询
        kdtree_.setInputCloud(map_cloud_);

        if (!map_received_)
        {
            ROS_INFO("[PathSimplify] Map received (%lu points). Ready to smooth.",
                     map_cloud_->size());
            map_received_ = true;
        }

        // 若路径已先到达, 立即处理
        if (map_received_ && path_received_)
        {
            ROS_INFO("[PathSimplify] Smoothing path.");
            nav_msgs::Path optimized_path = greedySmoothing(path_msg_);
            path_pub_.publish(optimized_path);
        }
    }

    /**
     * @brief 路径回调 — 接收待简化的原始路径
     *
     * 若地图已就绪则立即处理; 否则缓存路径等待地图
     */
    void pathCallback(const nav_msgs::Path::ConstPtr& msg)
    {
        if (msg->poses.size() < 3)
        {
            path_pub_.publish(msg);  // 太短, 无需简化
            ROS_INFO("[PathSimplify] path too short, skip.");
            return;
        }

        if (!path_received_)
            path_received_ = true;

        path_msg_ = *msg;  // 缓存

        if (map_received_ && path_received_)
        {
            ROS_INFO("[PathSimplify] Smoothing path.");
            nav_msgs::Path optimized_path = greedySmoothing(path_msg_);
            path_pub_.publish(optimized_path);
        }
    }

    /**
     * @brief 贪心简化核心算法
     *
     * 对于路径上的每个点 current_idx:
     *   从最远的后续点向前扫描, 找到第一个可直达 (无碰撞) 的最远点
     *   将该点加入简化路径, 并从该点继续搜索
     *
     * @param raw_path 原始 A* 路径 (稠密)
     * @return 简化后的路径 (稀疏, 但保证安全)
     */
    nav_msgs::Path greedySmoothing(const nav_msgs::Path raw_path)
    {
        ros::WallTime start_time = ros::WallTime::now();

        nav_msgs::Path smooth_path;
        smooth_path.header = raw_path.header;

        const std::vector<geometry_msgs::PoseStamped>& raw_poses = raw_path.poses;
        smooth_path.poses.push_back(raw_poses[0]);  // 起点始终保留

        int current_idx  = 0;
        int total_points = raw_poses.size();

        while (current_idx < total_points - 1)
        {
            int next_idx = current_idx + 1;

            // 从最远点向前扫描, 找可直达的最远点
            for (int i = current_idx + 2; i < total_points; ++i)
            {
                bool collision = hasLineCollision(
                    raw_poses[current_idx].pose.position,
                    raw_poses[i].pose.position);

                if (collision)
                {
                    // 碰撞 → 上一个点 (i-1) 是安全极限
                    next_idx = i - 1;
                    break;
                }
                else
                {
                    // 安全 → 更新最远可达点
                    next_idx = i;
                }
            }

            smooth_path.poses.push_back(raw_poses[next_idx]);
            current_idx = next_idx;
        }

        // -------- 统计输出 --------
        size_t input_size  = raw_path.poses.size();
        size_t output_size = smooth_path.poses.size();
        ros::WallTime end_time = ros::WallTime::now();
        double execution_time_ms = (end_time - start_time).toSec() * 1000.0;
        double reduction_percent = 0.0;
        if (input_size > 0)
            reduction_percent = (1.0 - (static_cast<double>(output_size) / input_size)) * 100.0;

        ROS_INFO_STREAM("[PathSimplify] In: " << input_size
                        << " -> Out: " << output_size
                        << " (" << std::fixed << std::setprecision(2)
                        << reduction_percent << "%)"
                        << " | Time: " << std::setprecision(3)
                        << execution_time_ms << " ms");

        return smooth_path;
    }

    /**
     * @brief 3D 直线段碰撞检测 (基于 KD-Tree 半径搜索)
     *
     * 沿 start→end 线段以 check_step_size_ 步长采样,
     * 对每个采样点做半径为 collision_radius_ 的最近邻搜索
     *
     * @return true=碰撞 (线段穿过障碍物), false=安全
     *
     * @note 起点和终点本身不检测 (它们在原始 A* 路径上, 默认安全)
     */
    bool hasLineCollision(const geometry_msgs::Point& start,
                          const geometry_msgs::Point& end)
    {
        double dx   = end.x - start.x;
        double dy   = end.y - start.y;
        double dz   = end.z - start.z;
        double dist = std::sqrt(dx * dx + dy * dy + dz * dz);

        int steps    = std::ceil(dist / check_step_size_);
        double x_step = dx / steps;
        double y_step = dy / steps;
        double z_step = dz / steps;

        double cur_x = start.x;
        double cur_y = start.y;
        double cur_z = start.z;

        std::vector<int>   pointIdxRadiusSearch;
        std::vector<float> pointRadiusSquaredDistance;

        // 从 i=1 到 steps-1: 跳过起点和终点
        for (int i = 1; i < steps; ++i)
        {
            cur_x += x_step;
            cur_y += y_step;
            cur_z += z_step;

            pcl::PointXYZ searchPoint;
            searchPoint.x = cur_x;
            searchPoint.y = cur_y;
            searchPoint.z = cur_z;

            // 核心查询: 搜索半径 R 内是否有障碍点
            if (kdtree_.radiusSearch(searchPoint, collision_radius_,
                                     pointIdxRadiusSearch,
                                     pointRadiusSquaredDistance) > 0)
            {
                return true;  // 发现障碍物 → 碰撞
            }
        }
        return false;  // 安全
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "greedy_path_smoother");
    PathSmoother3D smoother;
    ros::spin();
    return 0;
}
