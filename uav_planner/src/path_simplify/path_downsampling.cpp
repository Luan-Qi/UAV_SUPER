/**
 * @file    path_downsampling.cpp
 * @brief   路径降采样节点 — 两种模式: 等步长抽取 / 最小间距过滤
 *
 * 模式说明:
 *   - "step"     (等步长):  每隔 step 个点取一个点, 保证终点保留
 *   - "distance" (最小间距): 仅保留与上一个保留点距离 > distance_thresh 的点
 *
 * 适用场景:
 *   - A* 输出的稠密路径 (逐栅格) 在发布给控制器前进行初步稀疏化
 *   - 与 DP/Greedy 简化的区别: 本节点更轻量, 不做几何分析
 *
 * 订阅: input_topic  (nav_msgs::Path)
 * 发布: output_topic (nav_msgs::Path)
 *
 * 参数:
 *   mode            — "step" 或 "distance" (默认 "step")
 *   step            — 步长模式的间隔点数 (默认 5)
 *   distance_thresh — 距离模式的最小间距 (默认 0.5m)
 */

#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <cmath>

class PathDownsampler
{
private:
    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;

    ros::Subscriber path_sub_;
    ros::Publisher  path_pub_;

    int    step_;             ///< 步长模式: 每隔 N 个点取一个
    double distance_thresh_;  ///< 距离模式: 两点最小间距 (m)
    std::string mode_;        ///< 降采样模式: "step" | "distance"

public:
    PathDownsampler() : pnh_("~")
    {
        // -------- 读取参数 --------
        pnh_.param("step",             step_,             5);
        pnh_.param("distance_thresh",  distance_thresh_,  0.5);
        pnh_.param<std::string>("mode", mode_, "step");

        std::string input_topic, output_topic;
        pnh_.param<std::string>("input_topic",  input_topic,  "/input_path");
        pnh_.param<std::string>("output_topic", output_topic, "/downsampled_path");

        // -------- 初始化通信 --------
        path_sub_ = nh_.subscribe(input_topic, 1,
                                  &PathDownsampler::pathCallback, this);
        path_pub_ = nh_.advertise<nav_msgs::Path>(output_topic, 1, true);

        ROS_INFO("[PathSimplify] Path Downsampler initialized (mode=%s)", mode_.c_str());
    }

    /// 计算两个 PoseStamped 之间的 3D 欧氏距离
    double distance(const geometry_msgs::PoseStamped& a,
                    const geometry_msgs::PoseStamped& b)
    {
        double dx = a.pose.position.x - b.pose.position.x;
        double dy = a.pose.position.y - b.pose.position.y;
        double dz = a.pose.position.z - b.pose.position.z;
        return sqrt(dx * dx + dy * dy + dz * dz);
    }

    /**
     * @brief 步长模式降采样 — 每隔 step_ 个点取一个, 终点强制保留
     */
    nav_msgs::Path downsampleStep(const nav_msgs::Path& path)
    {
        nav_msgs::Path result;
        result.header = path.header;

        for (size_t i = 0; i < path.poses.size() - 1; i += step_)
            result.poses.push_back(path.poses[i]);

        // 确保终点不丢失
        if (!path.poses.empty())
            result.poses.push_back(path.poses.back());

        return result;
    }

    /**
     * @brief 距离模式降采样 — 仅保留与上一个保留点距离 > distance_thresh_ 的点
     */
    nav_msgs::Path downsampleDistance(const nav_msgs::Path& path)
    {
        nav_msgs::Path result;
        result.header = path.header;

        if (path.poses.empty())
            return result;

        // 起点始终保留
        result.poses.push_back(path.poses.front());

        for (size_t i = 1; i < path.poses.size(); i++)
        {
            if (distance(path.poses[i], result.poses.back()) > distance_thresh_)
                result.poses.push_back(path.poses[i]);
        }

        return result;
    }

    /**
     * @brief 路径回调 — 根据 mode_ 选择降采样策略
     */
    void pathCallback(const nav_msgs::PathConstPtr& msg)
    {
        nav_msgs::Path output;

        if (mode_ == "step")
        {
            output = downsampleStep(*msg);
            ROS_INFO("[PathSimplify] Downsampled (step): %zu -> %zu",
                     msg->poses.size(), output.poses.size());
        }
        else if (mode_ == "distance")
        {
            output = downsampleDistance(*msg);
            ROS_INFO("[PathSimplify] Downsampled (distance): %zu -> %zu",
                     msg->poses.size(), output.poses.size());
        }
        else
        {
            ROS_WARN("[PathSimplify] Unknown mode '%s', falling back to step", mode_.c_str());
            output = downsampleStep(*msg);
        }

        path_pub_.publish(output);
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "path_downsampler");
    PathDownsampler node;
    ros::spin();
    return 0;
}
