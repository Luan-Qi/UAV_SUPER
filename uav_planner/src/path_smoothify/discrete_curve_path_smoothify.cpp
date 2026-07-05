/**
 * @file    discrete_curve_path_smoothify.cpp
 * @brief   离散曲线平滑节点 — 基于迭代加权平均的路径平滑
 *
 * 算法原理 (Discrete Curve Smoothing / Dual-Weight Averaging):
 *
 *   核心更新公式 (对每个非端点的路径点 P_i):
 *     P_i := P_i + w_data * (O_i - P_i) + w_smooth * (P_{i-1} + P_{i+1} - 2*P_i)
 *
 *   其中:
 *     - O_i 是原始路径点 (保持对原始形状的忠实度)
 *     - w_data 控制"数据项"权重 (拉向原始点)
 *     - w_smooth 控制"平滑项"权重 (拉向邻居均值)
 *     - 首尾两点固定不动
 *
 *   迭代直到所有点的累计变化量 < tolerance, 或达到最大迭代次数 (1000)
 *
 * 权重建议:
 *   w_data   ∈ [0.1, 0.5]  — 越大越贴合原始路径
 *   w_smooth ∈ [0.1, 0.5]  — 越大越平滑
 *   w_data + w_smooth 通常 < 1.0 以保证收敛
 *
 * 与贝塞尔平滑的区别:
 *   - 贝塞尔: 几何插值, 生成新的插值点, 路径点数增加
 *   - 离散曲线: 迭代优化, 保持原有点数, 每个点位置被调整
 *
 * 订阅: input_topic  (nav_msgs::Path)
 * 发布: output_topic (nav_msgs::Path)
 *
 * 参数:
 *   weight_data            — 数据项权重 (默认 0.45)
 *   weight_smooth          — 平滑项权重 (默认 0.40)
 *   tolerance              — 收敛阈值 (默认 0.00001)
 *   recaluate_orientation  — 是否重算朝向 (默认 true)
 */

#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <vector>
#include <cmath>

class PathSmoother3D
{
public:
    PathSmoother3D()
    {
        ros::NodeHandle nh("~");

        // -------- 读取参数 --------
        nh.param<std::string>("input_topic",           input_topic_,           "astar_path");
        nh.param<std::string>("output_topic",          output_topic_,          "smoothed_path");
        nh.param<double>     ("weight_data",           weight_data_,           0.45);
        nh.param<double>     ("weight_smooth",         weight_smooth_,         0.40);
        nh.param<double>     ("tolerance",             tolerance_,             0.00001);
        nh.param<bool>       ("recaluate_orientation", recaluate_orientation_, true);

        path_sub_ = nh.subscribe(input_topic_,  1, &PathSmoother3D::pathCallback, this);
        path_pub_ = nh.advertise<nav_msgs::Path>(output_topic_, 1, true);

        ROS_INFO("[smoothify] Discrete Curve Path Smoother initialized.");
    }

private:
    ros::Subscriber path_sub_;
    ros::Publisher  path_pub_;

    std::string input_topic_;
    std::string output_topic_;
    double weight_data_;             ///< 数据项权重 (拉向原始点)
    double weight_smooth_;           ///< 平滑项权重 (拉向邻居均值)
    double tolerance_;               ///< 收敛阈值
    bool   recaluate_orientation_;   ///< 是否重算朝向

    /**
     * @brief 路径回调 — 收到路径后执行迭代平滑
     */
    void pathCallback(const nav_msgs::Path::ConstPtr& msg)
    {
        if (msg->poses.size() < 3)
        {
            path_pub_.publish(*msg);  // 太短, 无需平滑
            return;
        }

        nav_msgs::Path smoothed_path = *msg;
        smoothPath(smoothed_path, msg->poses);

        // 重算朝向 (可选)
        if (recaluate_orientation_)
            updateOrientations(smoothed_path);

        ROS_INFO("[smoothify] Got raw path with %ld points.", msg->poses.size());
        ROS_INFO("[smoothify] Smoothed path with %ld points.",
                 smoothed_path.poses.size());

        path_pub_.publish(smoothed_path);
    }

    /**
     * @brief 迭代平滑核心算法
     *
     * 对每个非端点 (i = 1..n-2) 反复应用更新公式:
     *   P_i := P_i + w_data*(O_i-P_i) + w_smooth*(P_{i-1}+P_{i+1}-2*P_i)
     *
     * 终止条件: 累计变化量 < tolerance 或 达到 max_iterations
     *
     * @param[in,out] path           被平滑的路径 (原地修改)
     * @param[in]     original_poses 原始路径点 (保持不变)
     */
    void smoothPath(nav_msgs::Path& path,
                    const std::vector<geometry_msgs::PoseStamped>& original_poses)
    {
        double change        = tolerance_;
        int    max_iterations = 1000;  // 防止死循环
        int    iter           = 0;
        int    n_points       = path.poses.size();

        while (change >= tolerance_ && iter < max_iterations)
        {
            change = 0.0;

            // 遍历所有非端点 (首尾固定)
            for (int i = 1; i < n_points - 1; ++i)
            {
                // 当前点坐标
                double x_i = path.poses[i].pose.position.x;
                double y_i = path.poses[i].pose.position.y;
                double z_i = path.poses[i].pose.position.z;

                // 原始点坐标 (数据项)
                double x_org = original_poses[i].pose.position.x;
                double y_org = original_poses[i].pose.position.y;
                double z_org = original_poses[i].pose.position.z;

                // 邻居坐标 (平滑项)
                double x_prev = path.poses[i - 1].pose.position.x;
                double y_prev = path.poses[i - 1].pose.position.y;
                double z_prev = path.poses[i - 1].pose.position.z;

                double x_next = path.poses[i + 1].pose.position.x;
                double y_next = path.poses[i + 1].pose.position.y;
                double z_next = path.poses[i + 1].pose.position.z;

                // -------- 核心更新公式 --------
                // P_i += w_data * (O_i - P_i) + w_smooth * (P_{i-1} + P_{i+1} - 2*P_i)
                double x_new = x_i
                             + weight_data_   * (x_org - x_i)
                             + weight_smooth_ * (x_prev + x_next - 2.0 * x_i);
                double y_new = y_i
                             + weight_data_   * (y_org - y_i)
                             + weight_smooth_ * (y_prev + y_next - 2.0 * y_i);
                double z_new = z_i
                             + weight_data_   * (z_org - z_i)
                             + weight_smooth_ * (z_prev + z_next - 2.0 * z_i);

                // 累加变化量 (用于收敛判断)
                change += fabs(x_i - x_new)
                        + fabs(y_i - y_new)
                        + fabs(z_i - z_new);

                // 应用新位置
                path.poses[i].pose.position.x = x_new;
                path.poses[i].pose.position.y = y_new;
                path.poses[i].pose.position.z = z_new;
            }
            iter++;
        }
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
            double roll  = 0.0;

            tf2::Quaternion q;
            q.setRPY(roll, -pitch, yaw);
            q.normalize();
            path.poses[i].pose.orientation = tf2::toMsg(q);
        }
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "path_smoother_3d_node");
    PathSmoother3D smoother;
    ros::spin();
    return 0;
}
