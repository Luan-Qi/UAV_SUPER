/**
 * @file    pca_direction_node.cpp
 * @brief   PCA 自由空间主方向估计节点 (Straight-Line Planner 辅助)
 *
 * 功能:
 *   对无人机前方的点云执行 PCA (主成分分析), 估计自由空间的主方向,
 *   发布一个 PoseStamped 表示推荐的飞行方向 (位置=当前里程计位姿, 朝向=主方向)。
 *
 * 算法流程:
 *   1. 接收点云 (如深度相机或激光雷达的实时点云)
 *   2. (可选) ROI 裁剪 — 保留前方一定范围的区域
 *   3. VoxelGrid 下采样 — 降噪 + 加速
 *   4. PCA/SVD:
 *      a) 计算点云均值
 *      b) 构建 3x3 协方差矩阵
 *      c) 特征分解, 取最大特征值对应的特征向量作为主方向
 *   5. 确保主方向指向前方 (x > 0)
 *   6. 构造旋转矩阵: x=主方向, z=世界up, y=cross(z,x)
 *   7. 发布 PoseStamped (位置=里程计, 朝向=主方向)
 *
 * 适用场景:
 *   - 无全局地图时的局部导航
 *   - 走廊/隧道等直线结构环境中的方向估计
 *   - 作为 A* 规划的补充 (感知驱动的局部方向)
 *
 * 订阅:
 *   /drone_0_pcl_render_node/cloud  (sensor_msgs::PointCloud2) — 前方点云
 *   /localization                    (nav_msgs::Odometry)       — 里程计
 *
 * 发布:
 *   /pca_direction  (geometry_msgs::PoseStamped) — 推荐飞行方向
 *
 * 参数:
 *   voxel_size  — 体素下采样叶子尺寸 (默认 0.2m)
 *   roi_x_min/max, roi_z_min/max — ROI 裁剪范围 (当前被注释)
 */

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>

#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>

#include <Eigen/Dense>

class PCADirectionEstimator
{
public:
    PCADirectionEstimator(ros::NodeHandle& nh)
    {
        // -------- 订阅 --------
        sub_cloud_ = nh.subscribe("/drone_0_pcl_render_node/cloud", 1,
                                  &PCADirectionEstimator::cloudCallback, this);
        pub_dir_   = nh.advertise<geometry_msgs::PoseStamped>("/pca_direction", 1);
        sub_odom_  = nh.subscribe("/localization", 10,
                                   &PCADirectionEstimator::odomCallback, this);

        // -------- 读取参数 (当前 ROI 裁剪被注释) --------
        nh.param("roi_x_min", roi_x_min_, 0.5);    // 前方最小距离
        nh.param("roi_x_max", roi_x_max_, 10.0);   // 前方最大距离
        nh.param("roi_z_min", roi_z_min_, -10.0);
        nh.param("roi_z_max", roi_z_max_, 10.0);
        nh.param("voxel_size", voxel_size_, 0.2);   // 下采样体素尺寸

        ROS_INFO("[PCA Direction Node] Initialized.");
    }

private:
    ros::Subscriber sub_cloud_;
    ros::Publisher  pub_dir_;
    ros::Subscriber sub_odom_;

    double roi_x_min_, roi_x_max_;
    double roi_z_min_, roi_z_max_;
    double voxel_size_;

    nav_msgs::Odometry latest_odom_;  ///< 最新里程计缓存
    bool odom_received_;

    /**
     * @brief 里程计回调 — 缓存最新的位姿信息
     */
    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
    {
        latest_odom_   = *msg;
        odom_received_ = true;
    }

    /**
     * @brief 点云回调 — 执行 PCA 并发布主方向
     *
     * 处理流水线:
     *   ROS 消息 → PCL → (可选 ROI) → VoxelGrid ↓ → PCA → 构造 Pose → 发布
     */
    void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg)
    {
        // ROS → PCL
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(*msg, *cloud);

        if (cloud->empty())
            return;

        // (可选) ROI 裁剪 — 当前被注释掉
        // pcl::PassThrough<pcl::PointXYZ> pass;
        // pass.setInputCloud(cloud);
        // pass.setFilterFieldName("x");
        // pass.setFilterLimits(roi_x_min_, roi_x_max_);
        // pass.filter(*cloud);
        // pass.setFilterFieldName("z");
        // pass.setFilterLimits(roi_z_min_, roi_z_max_);
        // pass.filter(*cloud);

        if (cloud->size() < 30)
            return;  // 点数太少, 无法可靠估计方向

        // -------- VoxelGrid 下采样 --------
        pcl::VoxelGrid<pcl::PointXYZ> voxel;
        voxel.setInputCloud(cloud);
        voxel.setLeafSize(voxel_size_, voxel_size_, voxel_size_);
        voxel.filter(*cloud);

        if (cloud->size() < 10)
            return;  // 下采样后点数不够

        // ======== PCA / SVD 主方向估计 ========

        // Step 1: 计算均值
        Eigen::Vector3f mean = Eigen::Vector3f::Zero();
        for (auto& p : cloud->points)
            mean += Eigen::Vector3f(p.x, p.y, p.z);
        mean /= cloud->size();

        // Step 2: 构建协方差矩阵 cov = E[(x-μ)(x-μ)^T]
        Eigen::Matrix3f cov = Eigen::Matrix3f::Zero();
        for (auto& p : cloud->points)
        {
            Eigen::Vector3f pt(p.x, p.y, p.z);
            pt -= mean;
            cov += pt * pt.transpose();
        }
        cov /= cloud->size();

        // Step 3: 特征分解 (SelfAdjointEigenSolver 保证实数特征值)
        Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> solver(cov);
        // 特征向量按特征值升序排列: col(0)=最小, col(2)=最大
        Eigen::Vector3f principal_dir = solver.eigenvectors().col(2);

        // Step 4: 确保方向指向前方 (x 正半轴)
        if (principal_dir.x() < 0)
            principal_dir = -principal_dir;

        principal_dir.normalize();

        // ======== 构造朝向 (旋转矩阵) ========

        // x 轴 = 主方向
        Eigen::Vector3f x_axis = principal_dir;
        // z 轴 = 世界 up (0,0,1)
        Eigen::Vector3f z_axis(0, 0, 1);

        // y 轴 = z × x (正交化)
        Eigen::Vector3f y_axis = z_axis.cross(x_axis);
        y_axis.normalize();

        // 重新计算 z 轴确保正交: z = x × y
        z_axis = x_axis.cross(y_axis);
        z_axis.normalize();

        // 构建旋转矩阵: 列向量 = 轴方向
        Eigen::Matrix3f R;
        R.col(0) = x_axis;
        R.col(1) = y_axis;
        R.col(2) = z_axis;

        Eigen::Quaternionf q(R);

        // ======== 发布 PoseStamped ========

        geometry_msgs::PoseStamped pose_msg;
        pose_msg.header.stamp    = msg->header.stamp;
        pose_msg.header.frame_id = "map";

        // 位置 = 里程计当前位置
        pose_msg.pose.position = latest_odom_.pose.pose.position;
        // 朝向 = PCA 主方向
        pose_msg.pose.orientation.x = q.x();
        pose_msg.pose.orientation.y = q.y();
        pose_msg.pose.orientation.z = q.z();
        pose_msg.pose.orientation.w = q.w();

        pub_dir_.publish(pose_msg);
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "pca_direction_node");
    ros::NodeHandle nh("~");

    PCADirectionEstimator estimator(nh);

    ros::spin();
    return 0;
}
