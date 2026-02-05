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

class PathSmoother3D
{
private:
    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;
    ros::Subscriber path_sub_;
    ros::Subscriber map_sub_;
    ros::Publisher path_pub_;

    std::string sub_path_topic_;
    std::string sub_map_topic_;
    std::string pub_topic_name_;
    double check_step_size_; // 连线上采样步长
    double collision_radius_; // 判定碰撞的搜索半径

    pcl::PointCloud<pcl::PointXYZ>::Ptr map_cloud_;
    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree_;
    bool map_received_;
    bool path_received_;
    nav_msgs::Path path_msg_;

public:
    PathSmoother3D() : pnh_("~"), map_received_(false), map_cloud_(new pcl::PointCloud<pcl::PointXYZ>)
    {
        pnh_.param<std::string>("input_topic", sub_path_topic_, "/a_star_path");
        pnh_.param<std::string>("map_topic", sub_map_topic_, "/grid_map_vis/point_cloud");
        pnh_.param<std::string>("output_topic", pub_topic_name_, "/smoothed_path");
        
        // 步长建议设为地图分辨率的一半 (0.1m)
        pnh_.param<double>("check_step_size", check_step_size_, 0.1); 
        
        // 碰撞半径：如果搜索到在这个半径内有点，则认为撞了。
        // 你的分辨率是0.2m，建议设为 0.15m ~ 0.2m 之间，确保不会漏掉体素中心
        pnh_.param<double>("collision_radius", collision_radius_, 0.2);

        // 2. 初始化
        path_sub_ = nh_.subscribe(sub_path_topic_, 1, &PathSmoother3D::pathCallback, this);
        map_sub_ = nh_.subscribe(sub_map_topic_, 1, &PathSmoother3D::mapCallback, this);
        path_pub_ = nh_.advertise<nav_msgs::Path>(pub_topic_name_, 1, true);

        ROS_INFO("[PathSimplify] Greedy Path Smoother Started.");
        ROS_INFO("[PathSimplify] Waiting for map on: %s", sub_map_topic_.c_str());
    }

    // 地图回调：构建 KD-Tree
    void mapCallback(const sensor_msgs::PointCloud2::ConstPtr& msg)
    {
        if (msg->data.empty()) return;

        pcl::fromROSMsg(*msg, *map_cloud_);
        
        // 构建 KD-Tree 用于快速最近邻搜索
        kdtree_.setInputCloud(map_cloud_);
        
        if (!map_received_)
        {
            ROS_INFO("[PathSimplify] Map received (%lu points). Ready to smooth.", map_cloud_->size());
            map_received_ = true;
        }
        
        if (map_received_ && path_received_)
        {
            ROS_INFO("[PathSimplify] Smoothing path.");
            nav_msgs::Path optimized_path = greedySmoothing(path_msg_);
            path_pub_.publish(optimized_path);
        }
    }

    // 路径回调
    void pathCallback(const nav_msgs::Path::ConstPtr& msg)
    {
        if (msg->poses.size() < 3)
        {
            path_pub_.publish(msg);
            ROS_INFO("[PathSimplify] path finish low.");
            return;
        }
        if (!path_received_)
        {
            path_received_ = true;
        }

        path_msg_ = *msg;
        if (map_received_ && path_received_)
        {
            ROS_INFO("[PathSimplify] Smoothing path.");
            nav_msgs::Path optimized_path = greedySmoothing(path_msg_);
            path_pub_.publish(optimized_path);
        }
    }

    // 贪心算法实现
    nav_msgs::Path greedySmoothing(const nav_msgs::Path raw_path)
    {
        ros::WallTime start_time = ros::WallTime::now();
        nav_msgs::Path smooth_path;
        smooth_path.header = raw_path.header;

        const std::vector<geometry_msgs::PoseStamped>& raw_poses = raw_path.poses;
        smooth_path.poses.push_back(raw_poses[0]);

        int current_idx = 0;
        int total_points = raw_poses.size();

        while (current_idx < total_points - 1)
        {
            int next_idx = current_idx + 1; 
            // 向前贪心搜索：尝试跳过中间点
            // 考虑到效率，我们不一定非要搜索到尽头，可以设置一个最大前瞻窗口(比如50个点)，
            // 但对于全剧规划，全部搜索也是毫秒级的（基于KDTree）
            for (int i = current_idx + 2; i < total_points; ++i)
            {
                // 检查 P_current 到 P_i 是否连通
                bool collision = hasLineCollision(
                    raw_poses[current_idx].pose.position, 
                    raw_poses[i].pose.position
                );

                if (collision)
                {
                    // 遇到障碍，前一个点 i-1 是安全极限
                    next_idx = i - 1;
                    break; 
                }
                else
                {
                    // 安全，更新目标为当前点 i
                    next_idx = i;
                }
            }

            // 如果 raw_poses[next_idx] 与 smooth_path.back() 非常近（比如重复点），可以忽略
            smooth_path.poses.push_back(raw_poses[next_idx]);
            current_idx = next_idx;
        }

        size_t input_size = raw_path.poses.size();
        size_t output_size = smooth_path.poses.size();
        ros::WallTime end_time = ros::WallTime::now();
        double execution_time_ms = (end_time - start_time).toSec() * 1000.0; // 转换为毫秒
        double reduction_percent = 0.0;
        if (input_size > 0) {
            reduction_percent = (1.0 - (static_cast<double>(output_size) / input_size)) * 100.0;
        }
        ROS_INFO_STREAM("[PathSimplify] In: " << input_size 
                        << " -> Out: " << output_size 
                        << " (" << std::fixed << std::setprecision(2) << reduction_percent << "%)"
                        << " | Time: " << std::setprecision(3) << execution_time_ms << " ms");

        return smooth_path;
    }

    // 使用 KD-Tree 进行 3D 连线碰撞检测
    bool hasLineCollision(const geometry_msgs::Point& start, const geometry_msgs::Point& end)
    {
        double dx = end.x - start.x;
        double dy = end.y - start.y;
        double dz = end.z - start.z;
        double dist = std::sqrt(dx*dx + dy*dy + dz*dz);

        // 步数 = 距离 / 检测步长
        int steps = std::ceil(dist / check_step_size_);
        
        double x_step = dx / steps;
        double y_step = dy / steps;
        double z_step = dz / steps;

        double cur_x = start.x;
        double cur_y = start.y;
        double cur_z = start.z;

        std::vector<int> pointIdxRadiusSearch;
        std::vector<float> pointRadiusSquaredDistance;

        // 遍历连线上的采样点
        // i 从 1 开始，到 steps-1 结束。起点和终点通常默认安全(因为在A*路径上)
        // 如果担心终点正好卡在障碍边缘，可以 loop 到 steps
        for (int i = 1; i < steps; ++i)
        { 
            cur_x += x_step;
            cur_y += y_step;
            cur_z += z_step;

            pcl::PointXYZ searchPoint;
            searchPoint.x = cur_x;
            searchPoint.y = cur_y;
            searchPoint.z = cur_z;

            // 核心查询：搜索当前点半径 R 内是否有地图点
            if (kdtree_.radiusSearch(searchPoint, collision_radius_, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0)
            {
                return true;
            }
        }
        return false;
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "greedy_path_smoother");
    PathSmoother3D smoother;
    ros::spin();
    return 0;
}