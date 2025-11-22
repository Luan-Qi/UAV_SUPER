#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <vector>
#include <cmath>

//Discrete Curve Smoothing Algorithm

class PathSmoother3D
{
public:
    PathSmoother3D()
    {
        ros::NodeHandle nh("~");
        nh.param<std::string>("input_topic", input_topic_, "astar_path");
        nh.param<std::string>("output_topic", output_topic_, "smoothed_path");
        
        // 平滑参数
        // weight_data: 保持原始路径形状的权重 (0.1 - 0.5 之间通常较好)
        nh.param<double>("weight_data", weight_data_, 0.45);
        // weight_smooth: 平滑程度的权重 (0.1 - 0.5 之间通常较好)
        nh.param<double>("weight_smooth", weight_smooth_, 0.40);
        // tolerance: 迭代停止的误差阈值
        nh.param<double>("tolerance", tolerance_, 0.00001);
        nh.param<bool>("recaluate_orientation", recaluate_orientation_, true);

        path_sub_ = nh.subscribe(input_topic_, 1, &PathSmoother3D::pathCallback, this);
        path_pub_ = nh.advertise<nav_msgs::Path>(output_topic_, 1, true);

        ROS_INFO("Discrete Curve Path Smoother initialized.");
    }

private:
    ros::Subscriber path_sub_;
    ros::Publisher path_pub_;
    
    std::string input_topic_;
    std::string output_topic_;
    double weight_data_;
    double weight_smooth_;
    double tolerance_;
    bool recaluate_orientation_;

    void pathCallback(const nav_msgs::Path::ConstPtr& msg)
    {
        if (msg->poses.size() < 3)
        {
            path_pub_.publish(*msg);
            return;
        }

        nav_msgs::Path smoothed_path = *msg;
        smoothPath(smoothed_path, msg->poses);
        if(recaluate_orientation_) updateOrientations(smoothed_path);
        ROS_INFO("Got raw path with %ld points.", msg->poses.size());
        ROS_INFO("Smoothed path with %ld points.", smoothed_path.poses.size());

        path_pub_.publish(smoothed_path);
    }

    void smoothPath(nav_msgs::Path& path, const std::vector<geometry_msgs::PoseStamped>& original_poses)
    {
        double change = tolerance_;
        int max_iterations = 1000; // 防止死循环
        int iter = 0;

        int n_points = path.poses.size();

        while (change >= tolerance_ && iter < max_iterations)
        {
            change = 0.0;
            for (int i = 1; i < n_points - 1; ++i)
            {
                // 获取当前点、原始点、前一个点、后一个点的坐标引用
                double x_i = path.poses[i].pose.position.x;
                double y_i = path.poses[i].pose.position.y;
                double z_i = path.poses[i].pose.position.z;

                double x_org = original_poses[i].pose.position.x;
                double y_org = original_poses[i].pose.position.y;
                double z_org = original_poses[i].pose.position.z;

                double x_prev = path.poses[i-1].pose.position.x;
                double y_prev = path.poses[i-1].pose.position.y;
                double z_prev = path.poses[i-1].pose.position.z;

                double x_next = path.poses[i+1].pose.position.x;
                double y_next = path.poses[i+1].pose.position.y;
                double z_next = path.poses[i+1].pose.position.z;

                // 核心更新公式
                //$$P_i \leftarrow P_i + w_{data} \cdot (O_i - P_i) + w_{smooth} \cdot (P_{i-1} + P_{i+1} - 2P_i)$$
                double x_new = x_i + weight_data_ * (x_org - x_i) + weight_smooth_ * (x_prev + x_next - 2.0 * x_i);
                double y_new = y_i + weight_data_ * (y_org - y_i) + weight_smooth_ * (y_prev + y_next - 2.0 * y_i);
                double z_new = z_i + weight_data_ * (z_org - z_i) + weight_smooth_ * (z_prev + z_next - 2.0 * z_i);

                // 累加变化量用于判断收敛
                change += fabs(x_i - x_new) + fabs(y_i - y_new) + fabs(z_i - z_new);

                path.poses[i].pose.position.x = x_new;
                path.poses[i].pose.position.y = y_new;
                path.poses[i].pose.position.z = z_new;
            }
            iter++;
        }
    }

    // 辅助函数：根据路径走向更新姿态朝向
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
    ros::init(argc, argv, "path_smoother_3d_node");
    PathSmoother3D smoother;
    ros::spin();
    return 0;
}