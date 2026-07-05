/**
 * @file    bezier_path_smoothify.cpp
 * @brief   贝塞尔曲线路径平滑节点 — 支持二次 (Quadratic) 和三次 (Cubic) 贝塞尔插值
 *
 * 算法对比:
 *   特性              二次贝塞尔 (Order 2)              三次贝塞尔 (Order 3)
 *   ────────────────  ──────────────────────────────  ───────────────────────────────
 *   计算逻辑           中点连接法 (Corner Cutting)       B样条 -> Bezier 控制点转换
 *   输入点窗口         3个点 (bezier_radius 间隔)       4个点 (bezier_radius 间隔)
 *   连续性             C1 (速度连续, 加速度突变)          C2 (加速度连续, 极度平滑)
 *   贴合度             较好, 仅切掉尖角                   较松散, 路径整体向内收缩
 *   适用场景           室内窄道, 需紧贴障碍物绕行          空间开阔, 三维飞行, 追求平稳
 *
 * 二次贝塞尔公式:
 *   B(t) = (1-t)^2 * P0 + 2(1-t)t * P1 + t^2 * P2
 *
 * 三次贝塞尔 (B样条->Bezier 转换):
 *   S  = (P0 + 4*P1 + P2) / 6          (段起点)
 *   E  = (P1 + 4*P2 + P3) / 6          (段终点)
 *   C1 = (2*P1 + P2) / 3               (控制点1)
 *   C2 = (P1 + 2*P2) / 3               (控制点2)
 *   B(t) = (1-t)^3*S + 3(1-t)^2*t*C1 + 3(1-t)*t^2*C2 + t^3*E
 *
 * 朝向重计算:
 *   沿路径逐点计算 yaw = atan2(dy, dx), pitch = atan2(dz, dist_xy), roll = 0
 *
 * 订阅: input_topic  (nav_msgs::Path)
 * 发布: output_topic (nav_msgs::Path)
 *
 * 参数:
 *   bezier_order           — 贝塞尔阶数: 2=二次 3=三次 (默认 2)
 *   bezier_radius          — 控制点采样步长 (默认 2, 越大越平滑)
 *   step_size              — t 参数插值步长 (默认 0.05, 越小越密)
 *   recaluate_orientation  — 是否重算路径朝向 (默认 true)
 */

#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/Point.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <vector>
#include <cmath>

class BezierSmoother
{
public:
    BezierSmoother()
    {
        ros::NodeHandle nh;
        ros::NodeHandle pnh("~");

        // -------- 读取参数 --------
        pnh.param<std::string>("input_topic",           input_topic_,           "path_raw");
        pnh.param<std::string>("output_topic",          output_topic_,          "path_smoothed");
        pnh.param<double>     ("step_size",             step_size_,             0.05);
        pnh.param<int>        ("bezier_order",          bezier_order_,          2);
        pnh.param<int>        ("bezier_radius",         bezier_radius_,         2);
        pnh.param<bool>       ("recaluate_orientation", recaluate_orientation_, true);

        sub_path_ = nh.subscribe(input_topic_,  1, &BezierSmoother::pathCallback, this);
        pub_path_ = nh.advertise<nav_msgs::Path>(output_topic_, 1, true);

        if (bezier_order_ == 2)
            ROS_INFO("[smoothify] Quadratic Bezier Smoother initialized.");
        else if (bezier_order_ == 3)
            ROS_INFO("[smoothify] Cubic Bezier Smoother initialized.");
        else
        {
            ROS_ERROR("[smoothify] Unsupported Bezier order: %d", bezier_order_);
            bezier_order_ = 2;  // 回退到二次
        }
    }

private:
    std::string input_topic_;
    std::string output_topic_;
    double step_size_;               ///< t 参数步长 (0~1 之间插值密度)
    int    bezier_order_;            ///< 贝塞尔阶数 (2 或 3)
    int    bezier_radius_;           ///< 控制点采样间隔 (跳过点数)
    bool   recaluate_orientation_;   ///< 是否重算朝向

    ros::Subscriber sub_path_;
    ros::Publisher  pub_path_;

    /// 3D 点结构体 (轻量, 避免引入 Eigen 依赖)
    struct Point3D
    {
        double x, y, z;
        Point3D(double _x, double _y, double _z) : x(_x), y(_y), z(_z) {}
        Point3D() : x(0), y(0), z(0) {}

        Point3D operator+(const Point3D& other) const
            { return Point3D(x + other.x, y + other.y, z + other.z); }
        Point3D operator-(const Point3D& other) const
            { return Point3D(x - other.x, y - other.y, z - other.z); }
        Point3D operator*(double scalar) const
            { return Point3D(x * scalar, y * scalar, z * scalar); }
        Point3D operator/(double scalar) const
            { return Point3D(x / scalar, y / scalar, z / scalar); }
    };

    Point3D toPoint3D(const geometry_msgs::Pose& pose)
    {
        return Point3D(pose.position.x, pose.position.y, pose.position.z);
    }

    // ========================================================================
    // 贝塞尔曲线计算
    // ========================================================================

    /**
     * @brief 二次贝塞尔公式: B(t) = (1-t)^2*P0 + 2(1-t)t*P1 + t^2*P2
     */
    Point3D calculateQuadraticBezier(float t, Point3D p0, Point3D p1, Point3D p2)
    {
        double u = 1.0 - t;
        return (p0 * (u * u)) + (p1 * (2 * u * t)) + (p2 * (t * t));
    }

    /**
     * @brief 三次贝塞尔公式: B(t) = (1-t)^3*P0 + 3(1-t)^2*t*P1 + 3(1-t)*t^2*P2 + t^3*P3
     */
    Point3D calculateCubicBezier(float t, Point3D p0, Point3D p1,
                                  Point3D p2, Point3D p3)
    {
        double u   = 1.0 - t;
        double tt  = t * t;
        double uu  = u * u;
        double uuu = uu * u;
        double ttt = tt * t;

        return (p0 * uuu) +
               (p1 * (3 * uu * t)) +
               (p2 * (3 * u * tt)) +
               (p3 * ttt);
    }

    // ========================================================================
    // 回调与主处理
    // ========================================================================

    /**
     * @brief 路径回调 — 收到路径后根据阶数选择平滑策略
     */
    void pathCallback(const nav_msgs::PathConstPtr& msg)
    {
        if (msg->poses.size() < 3)
        {
            pub_path_.publish(*msg);  // 太短, 无需平滑
            return;
        }

        nav_msgs::Path smoothed_path;
        smoothed_path.header = msg->header;

        std::vector<geometry_msgs::PoseStamped> raw_poses = msg->poses;

        if (bezier_order_ == 2)
            processQuadratic(raw_poses, smoothed_path);
        else
            processCubic(raw_poses, smoothed_path);

        // 重算路径朝向 (可选)
        if (recaluate_orientation_)
            updateOrientations(smoothed_path);

        ROS_INFO("[smoothify] Got raw path with %ld points.", msg->poses.size());
        ROS_INFO("[smoothify] Smoothed path with %ld points.",
                 smoothed_path.poses.size());

        pub_path_.publish(smoothed_path);
    }

    // ========================================================================
    // 二次贝塞尔处理 — Corner Cutting 中点连接法
    // ========================================================================

    /**
     * @brief 二次贝塞尔平滑 (Corner Cutting)
     *
     * 滑动窗口每次取 3 个点 (间隔 bezier_radius_):
     *   start   = (P_curr + P_next) / 2      (中点)
     *   control = P_next                      (中间点作为控制点)
     *   end     = (P_next + P_next_next) / 2  (下一段中点)
     *
     * 在 start->end 之间拟合二次贝塞尔曲线
     */
    void processQuadratic(const std::vector<geometry_msgs::PoseStamped>& raw_poses,
                          nav_msgs::Path& out_path)
    {
        out_path.poses.push_back(raw_poses[0]);  // 起点

        size_t n_points = raw_poses.size();
        for (size_t i = 0; i < n_points - 2 * bezier_radius_; i += bezier_radius_)
        {
            Point3D p_curr      = toPoint3D(raw_poses[i].pose);
            Point3D p_next      = toPoint3D(raw_poses[i + 1 * bezier_radius_].pose);
            Point3D p_next_next = toPoint3D(raw_poses[i + 2 * bezier_radius_].pose);

            // Corner Cutting: 控制点为相邻中点
            Point3D start_pt   = (p_curr + p_next) * 0.5;
            Point3D control_pt = p_next;
            Point3D end_pt     = (p_next + p_next_next) * 0.5;

            for (double t = 0.0; t <= 1.0; t += step_size_)
            {
                Point3D pt = calculateQuadraticBezier(t, start_pt, control_pt, end_pt);
                addPointToPath(out_path, pt, raw_poses[i + 1].pose.orientation);
            }
        }

        out_path.poses.push_back(raw_poses.back());  // 终点
    }

    // ========================================================================
    // 三次贝塞尔处理 — B-Spline -> Bezier 转换
    // ========================================================================

    /**
     * @brief 三次贝塞尔平滑 (Cubic B-Spline -> Bezier)
     *
     * 使用连续 4 个控制点 P0,P1,P2,P3 生成 P1->P2 之间的平滑段:
     *   - 首尾做 padding (复制首尾点) 确保边界可处理
     *   - B-Spline -> Bezier 转换公式保证 C2 连续性
     */
    void processCubic(const std::vector<geometry_msgs::PoseStamped>& raw_poses,
                      nav_msgs::Path& out_path)
    {
        // 预处理: 首尾 padding
        std::vector<Point3D> points;
        points.push_back(toPoint3D(raw_poses.front().pose));  // 复制起点
        for (const auto& pose : raw_poses)
            points.push_back(toPoint3D(pose.pose));
        points.push_back(toPoint3D(raw_poses.back().pose));   // 复制终点

        // 滑动窗口 [i, i+1, i+2, i+3]
        for (size_t i = 0; i < points.size() - 3 * bezier_radius_; i += bezier_radius_)
        {
            Point3D P0 = points[i];
            Point3D P1 = points[i + 1 * bezier_radius_];
            Point3D P2 = points[i + 2 * bezier_radius_];
            Point3D P3 = points[i + 3 * bezier_radius_];

            // B-Spline -> Bezier 控制点转换 (标准公式)
            Point3D S  = (P0 + P1 * 4.0 + P2) / 6.0;   // 段起点
            Point3D E  = (P1 + P2 * 4.0 + P3) / 6.0;   // 段终点
            Point3D C1 = (P1 * 2.0 + P2) / 3.0;        // 控制点 1
            Point3D C2 = (P1 + P2 * 2.0) / 3.0;        // 控制点 2

            for (double t = 0.0; t <= 1.0; t += step_size_)
            {
                Point3D pt = calculateCubicBezier(t, S, C1, C2, E);
                size_t original_idx = (i < raw_poses.size()) ? i : raw_poses.size() - 1;
                addPointToPath(out_path, pt, raw_poses[original_idx].pose.orientation);
            }
        }

        // 确保终点精确到达
        out_path.poses.push_back(raw_poses.back());
    }

    // ========================================================================
    // 辅助函数
    // ========================================================================

    /// 向 Path 消息追加一个点
    void addPointToPath(nav_msgs::Path& path, Point3D pt,
                        const geometry_msgs::Quaternion& orientation)
    {
        geometry_msgs::PoseStamped new_pose;
        new_pose.header           = path.header;
        new_pose.pose.position.x  = pt.x;
        new_pose.pose.position.y  = pt.y;
        new_pose.pose.position.z  = pt.z;
        new_pose.pose.orientation = orientation;
        path.poses.push_back(new_pose);
    }

    /**
     * @brief 沿路径重算朝向 (yaw + pitch), roll 固定为 0
     *
     * yaw   = atan2(dy, dx)      水平面朝向
     * pitch = atan2(dz, dist_xy)  爬升/下降角
     */
    void updateOrientations(nav_msgs::Path& path)
    {
        if (path.poses.size() < 2) return;

        for (size_t i = 0; i < path.poses.size(); ++i)
        {
            // 最后一个点的朝向与倒数第二个点一致
            if (i == path.poses.size() - 1)
            {
                path.poses[i].pose.orientation = path.poses[i - 1].pose.orientation;
                continue;
            }

            double x_cur  = path.poses[i].pose.position.x;
            double y_cur  = path.poses[i].pose.position.y;
            double z_cur  = path.poses[i].pose.position.z;
            double x_next = path.poses[i + 1].pose.position.x;
            double y_next = path.poses[i + 1].pose.position.y;
            double z_next = path.poses[i + 1].pose.position.z;

            double dx = x_next - x_cur;
            double dy = y_next - y_cur;
            double dz = z_next - z_cur;

            double dist_xy = std::sqrt(dx * dx + dy * dy);

            double yaw   = std::atan2(dy, dx);          // 水平朝向
            double pitch = std::atan2(dz, dist_xy);     // 爬升角
            double roll  = 0.0;                          // 固定为 0

            tf2::Quaternion q;
            q.setRPY(roll, -pitch, yaw);  // 注意 pitch 取反
            q.normalize();
            path.poses[i].pose.orientation = tf2::toMsg(q);
        }
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "bezier_smoother_node");
    BezierSmoother smoother;
    ros::spin();
    return 0;
}
