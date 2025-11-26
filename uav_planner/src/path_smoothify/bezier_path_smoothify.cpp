#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/Point.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <vector>
#include <cmath>

//特性          二次贝塞尔 (Order 2)          三次贝塞尔 (Order 3)
//计算逻辑      中点连接法                    B样条转换法
//输入点窗口    3个点                         4个点
//平滑度        C1 (速度连续，加速度突变)      C2 (加速度连续，极度平滑)
//贴合度        较好，只切掉尖角               较松散，路径整体会向内收缩
//适用场景      室内窄道、需要紧贴障碍物绕行    空间开阔、无人机三维飞行、追求平稳运动

class BezierSmoother
{
public:
    BezierSmoother()
    {
        ros::NodeHandle nh;
        ros::NodeHandle pnh("~");

        // 参数获取
        pnh.param<std::string>("input_topic", input_topic_, "path_raw");
        pnh.param<std::string>("output_topic", output_topic_, "path_smoothed");
        pnh.param<double>("step_size", step_size_, 0.05);
        pnh.param<int>("bezier_order", bezier_order_, 2);
        pnh.param<int>("bezier_radius", bezier_radius_, 2);
        pnh.param<bool>("recaluate_orientation", recaluate_orientation_, true);

        sub_path_ = nh.subscribe(input_topic_, 1, &BezierSmoother::pathCallback, this);
        pub_path_ = nh.advertise<nav_msgs::Path>(output_topic_, 1, true);

        if (bezier_order_ == 2){
            ROS_INFO("[smoothify] Quadratic Bezier Smoother initialized.");}
        else if (bezier_order_ == 3){
            ROS_INFO("[smoothify] Cubic Bezier Smoother initialized.");}
        else{
            ROS_ERROR("[smoothify] Unsupported Bezier order: %d", bezier_order_);
            bezier_order_ = 2;} // 默认为二次贝塞尔
    }

private:
    std::string input_topic_;
    std::string output_topic_;
    double step_size_;
    int bezier_order_;
    int bezier_radius_;
    bool recaluate_orientation_;

    ros::Subscriber sub_path_;
    ros::Publisher pub_path_;

    struct Point3D
    {
        double x, y, z;
        Point3D(double _x, double _y, double _z) : x(_x), y(_y), z(_z) {}
        Point3D() : x(0), y(0), z(0) {}
        
        Point3D operator+(const Point3D& other) const
        {
            return Point3D(x + other.x, y + other.y, z + other.z);
        }
        Point3D operator-(const Point3D& other) const
        {
            return Point3D(x - other.x, y - other.y, z - other.z);
        }
        Point3D operator*(double scalar) const
        {
            return Point3D(x * scalar, y * scalar, z * scalar);
        }
        Point3D operator/(double scalar) const
        {
            return Point3D(x / scalar, y / scalar, z / scalar);
        }
    };

    Point3D toPoint3D(const geometry_msgs::Pose& pose)
    {
        return Point3D(pose.position.x, pose.position.y, pose.position.z);
    }

    // 二次贝塞尔公式 (Quadratic)
    // B(t) = (1-t)^2 P0 + 2(1-t)t P1 + t^2 P2
    Point3D calculateQuadraticBezier(float t, Point3D p0, Point3D p1, Point3D p2)
    {
        double u = 1.0 - t;
        return (p0 * (u * u)) + (p1 * (2 * u * t)) + (p2 * (t * t));
    }

    // 三次贝塞尔公式 (Cubic)
    // B(t) = (1-t)^3 P0 + 3(1-t)^2 t P1 + 3(1-t)t^2 P2 + t^3 P3
    Point3D calculateCubicBezier(float t, Point3D p0, Point3D p1, Point3D p2, Point3D p3)
    {
        double u = 1.0 - t;
        double tt = t * t;
        double uu = u * u;
        double uuu = uu * u;
        double ttt = tt * t;

        Point3D p = (p0 * uuu) + 
                    (p1 * (3 * uu * t)) + 
                    (p2 * (3 * u * tt)) + 
                    (p3 * ttt);
        return p;
    }

    void pathCallback(const nav_msgs::PathConstPtr& msg)
    {
        if (msg->poses.size() < 3)
        {
            pub_path_.publish(*msg);
            return;
        }

        nav_msgs::Path smoothed_path;
        smoothed_path.header = msg->header;

        std::vector<geometry_msgs::PoseStamped> raw_poses = msg->poses;
        
        if (bezier_order_ == 2) 
            processQuadratic(raw_poses, smoothed_path);
        else
            processCubic(raw_poses, smoothed_path);
        
        if(recaluate_orientation_) updateOrientations(smoothed_path);
        ROS_INFO("[smoothify] Got raw path with %ld points.", msg->poses.size());
        ROS_INFO("[smoothify] Smoothed path with %ld points.", smoothed_path.poses.size());

        pub_path_.publish(smoothed_path);
    }

    // --- 二次贝塞尔处理逻辑 (Corner Cutting) ---
    void processQuadratic(const std::vector<geometry_msgs::PoseStamped>& raw_poses, nav_msgs::Path& out_path)
    {
        out_path.poses.push_back(raw_poses[0]); // 起点

        size_t n_points = raw_poses.size();
        for (size_t i = 0; i < n_points - 2 * bezier_radius_; i += bezier_radius_)
        {
            Point3D p_curr = toPoint3D(raw_poses[i].pose);
            Point3D p_next = toPoint3D(raw_poses[i + 1 * bezier_radius_].pose);
            Point3D p_next_next = toPoint3D(raw_poses[i + 2 * bezier_radius_].pose);

            Point3D start_pt = (i == 0) ? (p_curr + p_next) * 0.5 : (p_curr + p_next) * 0.5;
            Point3D control_pt = p_next;
            Point3D end_pt = (p_next + p_next_next) * 0.5;

            for (double t = 0.0; t <= 1.0; t += step_size_)
            {
                Point3D pt = calculateQuadraticBezier(t, start_pt, control_pt, end_pt);
                addPointToPath(out_path, pt, raw_poses[i+1].pose.orientation);
            }
        }
        out_path.poses.push_back(raw_poses.back()); // 终点
    }

    // --- 三次贝塞尔处理逻辑 (B-Spline近似) ---
    // 使用连续4个点 P0, P1, P2, P3 来生成 P1 和 P2 之间的平滑曲线
    void processCubic(const std::vector<geometry_msgs::PoseStamped>& raw_poses, nav_msgs::Path& out_path)
    {
        // 1. 预处理：为了让三次曲线能处理起点和终点，我们需要在首尾padding点
        // 简单的做法是重复起点和终点
        std::vector<Point3D> points;
        points.push_back(toPoint3D(raw_poses.front().pose)); // Duplicate start
        for(const auto& pose : raw_poses) {
            points.push_back(toPoint3D(pose.pose));
        }
        points.push_back(toPoint3D(raw_poses.back().pose)); // Duplicate end

        // 现在 points 的大小是 N + 2
        // 我们需要滑动窗口 [i, i+1, i+2, i+3]
        
        for (size_t i = 0; i < points.size() - 3 * bezier_radius_; i += bezier_radius_)
        {
            Point3D P0 = points[i];
            Point3D P1 = points[i + 1 * bezier_radius_];
            Point3D P2 = points[i + 2 * bezier_radius_];
            Point3D P3 = points[i + 3 * bezier_radius_];

            // 将 B-Spline 控制点转换为 Bezier 控制点 (S, C1, C2, E)
            // 这是一个标准的转换公式，保证 C2 连续性
            Point3D S  = (P0 + P1 * 4.0 + P2) / 6.0;  // Segment Start
            Point3D E  = (P1 + P2 * 4.0 + P3) / 6.0;  // Segment End
            Point3D C1 = (P1 * 2.0 + P2) / 3.0;       // Control Point 1
            Point3D C2 = (P1 + P2 * 2.0) / 3.0;       // Control Point 2

            // 在这一段生成插值点
            for (double t = 0.0; t <= 1.0; t += step_size_)
            {
                Point3D pt = calculateCubicBezier(t, S, C1, C2, E);
                
                // 获取方向：简单地取原始路径中段点的方向
                // 注意：raw_poses 的索引对应关系。points[i+1] 对应 raw_poses[i] (因为开头padding了一次)
                size_t original_idx = (i < raw_poses.size()) ? i : raw_poses.size() - 1;
                addPointToPath(out_path, pt, raw_poses[original_idx].pose.orientation);
            }
        }
        
        // 确保最后一个点精确到达
        out_path.poses.push_back(raw_poses.back());
    }

    void addPointToPath(nav_msgs::Path& path, Point3D pt, const geometry_msgs::Quaternion& orientation)
    {
        geometry_msgs::PoseStamped new_pose;
        new_pose.header = path.header;
        new_pose.pose.position.x = pt.x;
        new_pose.pose.position.y = pt.y;
        new_pose.pose.position.z = pt.z;
        new_pose.pose.orientation = orientation;
        path.poses.push_back(new_pose);
    }

    void updateOrientations(nav_msgs::Path& path)
    {
        if (path.poses.size() < 2) return;

        for (size_t i = 0; i < path.poses.size(); ++i)
        {
            // 如果是最后一个点，方向设为和倒数第二个点一致
            if (i == path.poses.size() - 1)
            {
                path.poses[i].pose.orientation = path.poses[i-1].pose.orientation;
                continue;
            }

            double x_cur = path.poses[i].pose.position.x;
            double y_cur = path.poses[i].pose.position.y;
            double z_cur = path.poses[i].pose.position.z;

            double x_next = path.poses[i+1].pose.position.x;
            double y_next = path.poses[i+1].pose.position.y;
            double z_next = path.poses[i+1].pose.position.z;

            double dx = x_next - x_cur;
            double dy = y_next - y_cur;
            double dz = z_next - z_cur;

            // 计算平面距离 (用于计算 Pitch)
            double dist_xy = std::sqrt(dx * dx + dy * dy);

            // 1. 计算 Yaw (绕 Z 轴旋转)，范围 [-pi, pi]
            double yaw = std::atan2(dy, dx);

            // 2. 计算 Pitch (绕 Y 轴旋转)，范围 [-pi/2, pi/2]
            // 也就是爬升角，dz > 0 为正（抬头），dz < 0 为负（低头）
            double pitch = std::atan2(dz, dist_xy);

            // 3. Roll (绕 X 轴旋转) 设为 0
            double roll = 0.0;

            tf2::Quaternion q;
            q.setRPY(roll, -pitch, yaw); 
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