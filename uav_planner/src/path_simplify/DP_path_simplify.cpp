/**
 * @file    DP_path_simplify.cpp
 * @brief   Douglas-Peucker 3D 路径简化节点
 *
 * 算法原理:
 *   Douglas-Peucker (道格拉斯-普克) 是一种递归的曲线简化算法:
 *   1. 连接首尾两点 A→B 作为基准线
 *   2. 在 A 和 B 之间找到离基准线最远的点 P
 *   3. 若 P 的距离 > 阈值 ε: 以 P 为分割点递归简化 A→P 和 P→B
 *   4. 若 P 的距离 ≤ 阈值 ε: 直接保留 A 和 B, 丢弃中间所有点
 *
 * 本实现为 3D 版本, 使用叉积计算点到空间直线的垂直距离
 *
 * 后处理:
 *   - 合并首尾确保完整性
 *   - mergeShortSegments(): 合并过短的线段 (距离 < min_segment_length)
 *
 * 订阅: input_topic  (nav_msgs::Path)
 * 发布: output_topic (nav_msgs::Path)
 *
 * 参数:
 *   simplification_threshold — DP 算法阈值 ε (默认 0.15m)
 *   min_segment_length       — 最短线段长度 (默认 0.5m)
 */

#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <vector>
#include <cmath>
#include <algorithm>
#include <iomanip>

class PathSimplifier
{
public:
    PathSimplifier()
    {
        ros::NodeHandle nh("~");
        nh.param<std::string>("input_topic",              input_topic_,  "/input_topic");
        nh.param<std::string>("output_topic",             output_topic_, "/output_topic");
        nh.param<double>     ("simplification_threshold", threshold_,    0.15);
        nh.param<double>     ("min_segment_length",       min_seg_len_,  0.5);

        sub_ = nh.subscribe(input_topic_,  1, &PathSimplifier::pathCallback, this);
        pub_ = nh.advertise<nav_msgs::Path>(output_topic_, 1, true);

        ROS_INFO("[PathSimplify] DP PathSimplifier initialized.");
    }

private:
    /**
     * @brief 路径回调 — 收到路径后执行 DP 简化并发布
     */
    void pathCallback(const nav_msgs::Path::ConstPtr& msg)
    {
        if (msg->poses.empty())
        {
            pub_.publish(*msg);
            return;
        }

        int max_recursion_depth = 0;
        ros::WallTime start_time = ros::WallTime::now();

        size_t input_size = msg->poses.size();

        // 提取所有点的位置
        std::vector<geometry_msgs::Point> original_points;
        original_points.reserve(input_size);
        for (const auto& pose : msg->poses)
            original_points.push_back(pose.pose.position);

        // -------- 执行 DP 简化 --------
        std::vector<geometry_msgs::Point> simplified;
        douglasPeucker(original_points, 0, original_points.size() - 1,
                       threshold_, simplified, 0, max_recursion_depth);

        // 确保起点和终点始终保留
        if (simplified.empty())
        {
            simplified.push_back(original_points.front());
            simplified.push_back(original_points.back());
        }
        else
        {
            if (simplified.front() != original_points.front())
                simplified.insert(simplified.begin(), original_points.front());
            if (simplified.back() != original_points.back())
                simplified.push_back(original_points.back());
        }

        // 合并过短线段
        simplified = mergeShortSegments(simplified, min_seg_len_);
        size_t output_size = simplified.size();

        // -------- 构建输出路径 --------
        nav_msgs::Path out_path = *msg;
        out_path.poses.clear();
        for (const auto& pt : simplified)
        {
            geometry_msgs::PoseStamped ps;
            ps.header = msg->header;
            ps.pose.position    = pt;
            ps.pose.orientation.w = 1.0;  // 单位四元数
            out_path.poses.push_back(ps);
        }

        pub_.publish(out_path);

        // 统计信息
        ros::WallTime end_time = ros::WallTime::now();
        double execution_time_ms = (end_time - start_time).toSec() * 1000.0;
        double reduction_percent = 0.0;
        if (input_size > 0)
            reduction_percent = (1.0 - (static_cast<double>(output_size) / input_size)) * 100.0;

        ROS_INFO_STREAM("[PathSimplify] Max Depth: " << max_recursion_depth);
        ROS_INFO_STREAM("[PathSimplify] In: " << input_size
                        << " -> Out: " << output_size
                        << " (" << std::fixed << std::setprecision(2)
                        << reduction_percent << "%)"
                        << " | Time: " << std::setprecision(3)
                        << execution_time_ms << " ms");
    }

    /**
     * @brief Douglas-Peucker 3D 递归简化
     *
     * @param points           原始点序列
     * @param start            当前段起始索引
     * @param end              当前段结束索引
     * @param epsilon          距离阈值 (超过此值的点被保留)
     * @param simplified       输出简化结果 (追加)
     * @param current_depth    当前递归深度 (调试用)
     * @param max_depth_tracker 追踪最大递归深度
     */
    void douglasPeucker(const std::vector<geometry_msgs::Point>& points,
                        size_t start, size_t end, double epsilon,
                        std::vector<geometry_msgs::Point>& simplified,
                        int current_depth, int& max_depth_tracker)
    {
        if (current_depth > max_depth_tracker)
            max_depth_tracker = current_depth;

        if (end - start < 1) return;  // 少于2个点, 无法分割

        // 找离基准线最远的点
        double dmax  = 0.0;
        size_t index = 0;

        geometry_msgs::Point A = points[start];
        geometry_msgs::Point B = points[end];

        for (size_t i = start + 1; i < end; ++i)
        {
            double dist = pointToLineDistance(points[i], A, B);
            if (dist > dmax)
            {
                dmax  = dist;
                index = i;
            }
        }

        // 若最远点距离 > 阈值: 在该点处分割, 递归处理两段
        if (dmax > epsilon)
        {
            std::vector<geometry_msgs::Point> rec_results1, rec_results2;
            douglasPeucker(points, start, index, epsilon,
                           rec_results1, current_depth + 1, max_depth_tracker);
            douglasPeucker(points, index, end, epsilon,
                           rec_results2, current_depth + 1, max_depth_tracker);

            // 合并结果 (避免重复分割点)
            simplified.insert(simplified.end(),
                              rec_results1.begin(), rec_results1.end());
            if (!rec_results2.empty())
                simplified.insert(simplified.end(),
                                  rec_results2.begin() + 1, rec_results2.end());
        }
        else
        {
            // 所有中间点都在阈值内 → 仅保留端点
            simplified.push_back(points[start]);
            simplified.push_back(points[end]);
        }
    }

    /**
     * @brief 计算点 P 到空间直线 AB 的垂直距离 (3D)
     * @return 垂直距离 = |(P-A) × (B-A)| / |B-A|
     */
    double pointToLineDistance(const geometry_msgs::Point& P,
                               const geometry_msgs::Point& A,
                               const geometry_msgs::Point& B)
    {
        double ax = B.x - A.x, ay = B.y - A.y, az = B.z - A.z;
        double px = P.x - A.x, py = P.y - A.y, pz = P.z - A.z;

        // 叉积: (P-A) × (B-A)
        double cross_x = ay * pz - az * py;
        double cross_y = az * px - ax * pz;
        double cross_z = ax * py - ay * px;

        double cross_norm = std::sqrt(cross_x * cross_x +
                                       cross_y * cross_y +
                                       cross_z * cross_z);
        double ab_norm = std::sqrt(ax * ax + ay * ay + az * az);

        if (ab_norm < 1e-8) return 0.0;  // A 和 B 重合

        return cross_norm / ab_norm;
    }

    /**
     * @brief 合并过短线段 — 若相邻点距离 < min_len 则跳过中间点
     * @note  始终保留首尾两点
     */
    std::vector<geometry_msgs::Point> mergeShortSegments(
        const std::vector<geometry_msgs::Point>& pts, double min_len)
    {
        if (pts.size() <= 2) return pts;

        std::vector<geometry_msgs::Point> result;
        result.push_back(pts[0]);

        for (size_t i = 1; i < pts.size() - 1; ++i)
        {
            double dx = pts[i].x - result.back().x;
            double dy = pts[i].y - result.back().y;
            double dz = pts[i].z - result.back().z;
            double dist = std::sqrt(dx * dx + dy * dy + dz * dz);

            if (dist >= min_len)
                result.push_back(pts[i]);
            // else: 跳过该点 (合并到下一段)
        }

        // 确保终点保留
        if (result.back() != pts.back())
            result.push_back(pts.back());

        return result;
    }

    // -------- 成员变量 --------
    ros::Subscriber sub_;
    ros::Publisher  pub_;
    std::string input_topic_;
    std::string output_topic_;
    double threshold_;    ///< DP 简化阈值 (m)
    double min_seg_len_;  ///< 最短线段长度 (m)
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "path_simplifier");
    PathSimplifier simplifier;
    ros::spin();
    return 0;
}
