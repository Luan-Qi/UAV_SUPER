/**
 * @file    simple_path_simplify.cpp
 * @brief   简单路径简化节点 — 基于方向容差的自适应直线拟合
 *
 * 算法原理:
 *   从起点开始, 沿路径前进方向建立一条基准直线:
 *   1. 对每个后续点, 检查其到基准线的横向偏差
 *   2. 容差随距离自适应缩放: allowed = max_deviation / (1 + dist_scale * distance)
 *      — 近处严格 (允许偏差小), 远处宽松 (允许偏差大)
 *   3. 若所有中间点都在容差内 → 延长当前直线段
 *   4. 若某点超出容差 → 在前一个安全点处断开, 开始新的直线段
 *
 * 特点:
 *   - 不依赖障碍物信息, 纯几何简化
 *   - 容差自适应: 近处保真度高, 远处可大幅简化
 *   - 支持定时器周期性重发布 (publish_rate > 0 时启用)
 *
 * 订阅: input_topic  (nav_msgs::Path)
 * 发布: output_topic (nav_msgs::Path)
 *
 * 参数:
 *   max_deviation       — 最大允许横向偏差 (默认 0.2m)
 *   dist_scale          — 距离缩放因子 (默认 0.5, 越大远处越宽松)
 *   min_segment_length  — 最短线段长度 (默认 0.05m)
 *   publish_rate        — 定时重发布频率 (默认 0=禁用)
 *
 * @note 此节点在 CMakeLists.txt 中未编译, 保留作为备选简化方案
 */

#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <vector>
#include <cmath>

// ============================================================================
// 轻量 3D 向量运算 (避免引入 Eigen 依赖)
// ============================================================================

struct Vec3 {
    double x, y, z;
    Vec3() : x(0), y(0), z(0) {}
    Vec3(double _x, double _y, double _z) : x(_x), y(_y), z(_z) {}
};

inline Vec3 toVec3(const geometry_msgs::Point &p)
    { return Vec3(p.x, p.y, p.z); }
inline Vec3 toVec3(const geometry_msgs::PoseStamped &ps)
    { return Vec3(ps.pose.position.x, ps.pose.position.y, ps.pose.position.z); }

inline Vec3 operator-(const Vec3 &a, const Vec3 &b)
    { return Vec3(a.x-b.x, a.y-b.y, a.z-b.z); }
inline Vec3 operator+(const Vec3 &a, const Vec3 &b)
    { return Vec3(a.x+b.x, a.y+b.y, a.z+b.z); }
inline Vec3 operator*(const Vec3 &a, double s)
    { return Vec3(a.x*s, a.y*s, a.z*s); }

inline double norm(const Vec3 &a)
    { return std::sqrt(a.x*a.x + a.y*a.y + a.z*a.z); }
inline double dot(const Vec3 &a, const Vec3 &b)
    { return a.x*b.x + a.y*b.y + a.z*b.z; }

/// 叉积 (3D)
inline Vec3 crossv(const Vec3 &a, const Vec3 &b)
{
    return Vec3(a.y*b.z - a.z*b.y,
                a.z*b.x - a.x*b.z,
                a.x*b.y - a.y*b.x);
}

/// 向量归一化
inline Vec3 normalize(const Vec3 &a)
{
    double n = norm(a);
    if (n <= 1e-12) return Vec3(0,0,0);
    return Vec3(a.x/n, a.y/n, a.z/n);
}

/**
 * @brief 计算点 p 到直线 (过 p0, 方向 dir) 的横向距离
 * @param p0  直线上一点
 * @param dir 直线方向 (必须已归一化)
 * @param p   待测点
 * @return 垂直距离 = |(p-p0) × dir| (dir 已归一化, 除法省略)
 */
inline double lateralDistanceToLine(const Vec3 &p0, const Vec3 &dir,
                                     const Vec3 &p)
{
    Vec3 v = p - p0;
    Vec3 c = crossv(v, dir);
    return norm(c);  // |dir| == 1, 故直接返回叉积模长
}

// ============================================================================
// PathSimplifier 类
// ============================================================================

class PathSimplifier
{
public:
    PathSimplifier(ros::NodeHandle &nh): nh_(nh)
    {
        nh_.param<std::string>("input_topic",             input_topic_,             "/global_path");
        nh_.param<std::string>("output_topic",            output_topic_,            "/global_path_simplified");
        nh_.param<int>        ("queue_size",              queue_size_,              1);
        nh_.param<double>     ("max_deviation",           max_deviation_,           0.2);
        nh_.param<double>     ("dist_scale",              dist_scale_,              0.5);
        nh_.param<double>     ("min_segment_length",      min_segment_length_,      0.05);
        nh_.param<bool>       ("publish_header_frame_id", publish_header_frame_id_, true);
        nh_.param<double>     ("publish_rate",            publish_rate_,            0.0);

        sub_ = nh_.subscribe(input_topic_, queue_size_,
                             &PathSimplifier::pathCallback, this);
        pub_ = nh_.advertise<nav_msgs::Path>(output_topic_, 1);

        // 若设置了重发布频率, 创建定时器周期性发布最新简化结果
        if (publish_rate_ > 0.0)
        {
            timer_ = nh_.createTimer(ros::Duration(1.0 / publish_rate_),
                                     &PathSimplifier::timerCallback, this);
        }
    }

    /**
     * @brief 路径回调 — 收到新路径立即简化并发布
     */
    void pathCallback(const nav_msgs::Path::ConstPtr &msg)
    {
        if (msg->poses.empty())
        {
            ROS_WARN("Received empty path");
            return;
        }
        last_input_ = *msg;  // 缓存原始路径

        nav_msgs::Path simplified;
        simplifyPath(last_input_, simplified);

        if (publish_header_frame_id_)
            simplified.header.frame_id = last_input_.header.frame_id;
        simplified.header.stamp = ros::Time::now();
        pub_.publish(simplified);
    }

    /**
     * @brief 定时器回调 — 按固定频率重发布最新简化路径
     */
    void timerCallback(const ros::TimerEvent &)
    {
        if (publish_rate_ <= 0.0) return;
        if (last_input_.poses.empty()) return;

        nav_msgs::Path simplified;
        simplifyPath(last_input_, simplified);

        if (publish_header_frame_id_)
            simplified.header.frame_id = last_input_.header.frame_id;
        simplified.header.stamp = ros::Time::now();
        pub_.publish(simplified);
    }

private:
    ros::NodeHandle nh_;
    ros::Subscriber sub_;
    ros::Publisher  pub_;
    ros::Timer      timer_;

    // 参数
    std::string input_topic_, output_topic_;
    int    queue_size_;
    double max_deviation_;       ///< 最大允许横向偏差 (m)
    double dist_scale_;          ///< 距离缩放因子
    double min_segment_length_;  ///< 最短线段长度 (m)
    bool   publish_header_frame_id_;
    double publish_rate_;        ///< 定时发布频率 (Hz, 0=禁用)

    nav_msgs::Path last_input_;  ///< 缓存的最新原始路径

    /**
     * @brief 核心简化算法 — 自适应方向容差直线拟合
     *
     * 对于路径上的每个起点 s:
     *   1. 取 s+1 作为初始方向
     *   2. 向前扫描 j = s+2, s+3, ...:
     *      a) 计算 j 到当前直线的横向偏差
     *      b) 比较偏差与自适应容差: allowed = max_dev / (1 + dist_scale * distance)
     *      c) 若在容差内 → 延长当前段; 否则断开
     *   3. 将段终点加入输出, 从该点继续
     *
     * @param[in]  in  原始路径
     * @param[out] out 简化后的路径
     */
    void simplifyPath(const nav_msgs::Path &in, nav_msgs::Path &out)
    {
        out.poses.clear();
        const auto &pts = in.poses;
        if (pts.empty()) return;
        size_t N = pts.size();

        // 起点始终保留
        out.poses.push_back(pts.front());

        size_t s = 0;  // 当前段起点索引
        while (s < N - 1)
        {
            size_t next  = s + 1;
            Vec3   p_s   = toVec3(pts[s]);
            Vec3   p_next = toVec3(pts[next]);

            // 计算初始方向
            Vec3 dir = p_next - p_s;
            double dir_norm = norm(dir);
            if (dir_norm < 1e-12)
            {
                s = next;  // 连续重复点, 跳过
                continue;
            }
            dir = normalize(dir);

            size_t j         = next + 1;
            size_t last_good = next;  // 最后一个在容差内的点

            // 向前扫描, 寻找最远的可拟合点
            for (; j < N; ++j)
            {
                Vec3 p_j       = toVec3(pts[j]);
                double distance = norm(p_j - p_s);
                // 自适应容差: 远处允许更大偏差
                double allowed  = max_deviation_ / (1.0 + dist_scale_ * distance);
                double lateral  = lateralDistanceToLine(p_s, dir, p_j);

                if (lateral <= allowed)
                {
                    last_good = j;  // 仍然在容差内
                    continue;
                }
                else
                {
                    // 增强逻辑: 若紧邻的下一个点就超差, 尝试调整方向
                    if (j == next + 1)
                    {
                        Vec3 new_dir = p_j - p_s;
                        if (norm(new_dir) > 1e-12)
                        {
                            dir = normalize(new_dir);
                            lateral = lateralDistanceToLine(p_s, dir, p_j);
                            if (lateral <= allowed)
                            {
                                last_good = j;
                                continue;
                            }
                        }
                    }
                    break;  // 超差, 在当前点之前断开
                }
            }

            size_t seg_end = last_good;

            // 确保线段足够长, 避免退化的极短线段
            double seg_len = norm(toVec3(pts[seg_end]) - p_s);
            if (seg_len < min_segment_length_ && seg_end + 1 < N)
            {
                seg_end = std::min(seg_end + 1, N - 1);
            }

            // 添加段终点 (去重)
            if (!out.poses.empty())
            {
                const auto &last_app  = out.poses.back();
                const auto &candidate = pts[seg_end];
                if (!posesEqual(last_app, candidate))
                    out.poses.push_back(candidate);
            }
            else
            {
                out.poses.push_back(pts[seg_end]);
            }

            // 前进到段终点
            if (seg_end == s)
                s = s + 1;  // 防止死循环
            else
                s = seg_end;
        }

        // 确保终点保留
        if (!out.poses.empty())
        {
            const auto &last_app = out.poses.back();
            const auto &final_pt = pts.back();
            if (!posesEqual(last_app, final_pt))
                out.poses.push_back(final_pt);
        }
    }

    /// 判断两个 PoseStamped 是否位置相同 (容差 1e-6)
    static bool posesEqual(const geometry_msgs::PoseStamped &a,
                           const geometry_msgs::PoseStamped &b)
    {
        const double eps = 1e-6;
        return (std::fabs(a.pose.position.x - b.pose.position.x) < eps &&
                std::fabs(a.pose.position.y - b.pose.position.y) < eps &&
                std::fabs(a.pose.position.z - b.pose.position.z) < eps);
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "simplify_path_node");
    ros::NodeHandle nh("~");
    PathSimplifier ps(nh);
    ros::spin();
    return 0;
}
