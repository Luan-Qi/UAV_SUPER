/**
 * @file merge_cloud_node.cpp
 * @brief 双激光雷达点云融合节点 —— 通过 message_filters 近似时间同步合并两个 LiDAR 点云。
 *
 * @details
 * 使用 message_filters::ApproximateTime 策略同步两个 PointCloud2 话题，核心功能：
 *   1. 时间同步        — ApproximateTime 近似时间对齐，容忍有限的时间差
 *   2. 6-DOF 变换      — 每个点云独立施加 roll/pitch/yaw + tx/ty/tz 刚体变换
 *   3. 旋转顺序        — ZYX (yaw → pitch → roll)，与 ROS REP-103 一致
 *   4. 点云合并        — 使用 PCL operator+ 简单拼接两个变换后的点云
 *   5. 发布            — 发布合并后的 PointCloud2 到 /merged_cloud 话题 (frame_id=map)
 *
 * 坐标系约定：
 *   - 输入点云各自在自己的 LiDAR 局部坐标系
 *   - 经 6-DOF 变换后统一到 "map" 坐标系
 *
 * 用法：
 *   rosrun uav_util merge_cloud_node _cloud1_topic:=/livox/lidar1 _cloud2_topic:=/livox/lidar2 \\
 *     _roll1:=0.1 _tx2:=0.5
 */

// merge_cloud_node.cpp
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/transforms.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <Eigen/Dense>

// ============================================================================
// 点云合并节点
// ============================================================================

/**
 * @brief 双 LiDAR 点云合并节点 —— 订阅两个 PointCloud2 话题，独立变换后合并发布。
 *
 * 算法流水线 (syncCallback)：
 *   1. ROS → PCL 转换      — 将两个 sensor_msgs::PointCloud2 转为 pcl::PointXYZI
 *   2. 独立 6-DOF 变换     — 对 cloud1/cloud2 分别施加各自的 roll/pitch/yaw/tx/ty/tz
 *   3. 点云拼接            — merged = cloud1_tf + cloud2_tf
 *   4. PCL → ROS 转换      — 转回 sensor_msgs::PointCloud2
 *   5. 发布                — 发布到 /merged_cloud (frame_id="map")
 */
class MergeCloudNode
{
public:
    /**
     * @brief 构造函数：加载参数、初始化 message_filters 同步器、创建发布者。
     * @param nh   全局 ROS 节点句柄（用于 subscribe）
     * @param pnh  私有 ROS 节点句柄（用于加载 param）
     */
    MergeCloudNode(ros::NodeHandle& nh, ros::NodeHandle& pnh)
        : nh_(nh), pnh_(pnh)
    {
        // 读取参数（有默认值）
        pnh_.param<std::string>("cloud1_topic", cloud1_topic_, "/livox/lidar1");
        pnh_.param<std::string>("cloud2_topic", cloud2_topic_, "/livox/lidar2");

        pnh_.param<double>("roll1", roll1_, 0.0);      ///< LiDAR 1 绕 X 轴旋转 [rad]
        pnh_.param<double>("pitch1", pitch1_, 0.0);    ///< LiDAR 1 绕 Y 轴旋转 [rad]
        pnh_.param<double>("yaw1", yaw1_, 0.0);        ///< LiDAR 1 绕 Z 轴旋转 [rad]
        pnh_.param<double>("tx1", tx1_, 0.0);          ///< LiDAR 1 X 平移 [m]
        pnh_.param<double>("ty1", ty1_, 0.0);          ///< LiDAR 1 Y 平移 [m]
        pnh_.param<double>("tz1", tz1_, 0.0);          ///< LiDAR 1 Z 平移 [m]

        pnh_.param<double>("roll2", roll2_, 0.0);      ///< LiDAR 2 绕 X 轴旋转 [rad]
        pnh_.param<double>("pitch2", pitch2_, 0.0);    ///< LiDAR 2 绕 Y 轴旋转 [rad]
        pnh_.param<double>("yaw2", yaw2_, 0.0);        ///< LiDAR 2 绕 Z 轴旋转 [rad]
        pnh_.param<double>("tx2", tx2_, 0.0);          ///< LiDAR 2 X 平移 [m]
        pnh_.param<double>("ty2", ty2_, 0.0);          ///< LiDAR 2 Y 平移 [m]
        pnh_.param<double>("tz2", tz2_, 0.0);          ///< LiDAR 2 Z 平移 [m]

        // message_filters subscribers
        cloud1_sub_.subscribe(nh_, cloud1_topic_, 10);
        cloud2_sub_.subscribe(nh_, cloud2_topic_, 10);

        sync_.reset(new message_filters::Synchronizer<SyncPolicy>(SyncPolicy(10), cloud1_sub_, cloud2_sub_));
        sync_->registerCallback(boost::bind(&MergeCloudNode::syncCallback, this, _1, _2));

        merged_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("merged_cloud", 10);

        ROS_INFO("MergeCloudNode started. Subscribing to [%s] and [%s].",
                 cloud1_topic_.c_str(), cloud2_topic_.c_str());
    }

private:
    /**
     * @brief 同步回调：接收两个时间对齐的点云，独立变换后合并发布。
     * @param cloud1_msg  LiDAR 1 点云消息
     * @param cloud2_msg  LiDAR 2 点云消息
     */
    void syncCallback(const sensor_msgs::PointCloud2ConstPtr& cloud1_msg,
                      const sensor_msgs::PointCloud2ConstPtr& cloud2_msg)
    {
        // 1. 转换到 PCL 点云
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud1(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud2(new pcl::PointCloud<pcl::PointXYZI>);

        pcl::fromROSMsg(*cloud1_msg, *cloud1);
        pcl::fromROSMsg(*cloud2_msg, *cloud2);

        // 2. 计算变换矩阵
        Eigen::Matrix4f transform1 = Eigen::Matrix4f::Identity();
        Eigen::Matrix4f transform2 = Eigen::Matrix4f::Identity();
        setTransformMatrix(transform1, static_cast<float>(roll1_), static_cast<float>(pitch1_), static_cast<float>(yaw1_),
                            static_cast<float>(tx1_), static_cast<float>(ty1_), static_cast<float>(tz1_));
        setTransformMatrix(transform2, static_cast<float>(roll2_), static_cast<float>(pitch2_), static_cast<float>(yaw2_),
                            static_cast<float>(tx2_), static_cast<float>(ty2_), static_cast<float>(tz2_));

        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud1_tf(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud2_tf(new pcl::PointCloud<pcl::PointXYZI>);

        // 3. 施加独立变换
        pcl::transformPointCloud(*cloud1, *cloud1_tf, transform1);
        pcl::transformPointCloud(*cloud2, *cloud2_tf, transform2);

        // 4. 合并点云
        pcl::PointCloud<pcl::PointXYZI>::Ptr merged(new pcl::PointCloud<pcl::PointXYZI>);
        *merged = *cloud1_tf + *cloud2_tf;

        // 5. 转回 ROS msg 并发布
        sensor_msgs::PointCloud2 out_msg;
        pcl::toROSMsg(*merged, out_msg);
        // 使用 cloud1 的 header 时间或当前时间，根据需要我使用当前时间
        out_msg.header.stamp = ros::Time::now();
        out_msg.header.frame_id = "map";

        merged_pub_.publish(out_msg);
        ROS_INFO_STREAM_THROTTLE(1.0, "Published merged cloud, points: " << merged->size());
    }

    /**
     * @brief 根据 6-DOF 参数构建 4x4 齐次变换矩阵。
     *
     * 旋转顺序：ZYX (yaw → pitch → roll)。
     * 变换矩阵形式：T = [R  t; 0  1]，其中 R = R_z(yaw) * R_y(pitch) * R_x(roll)。
     *
     * @param[out] transform  输出的 4x4 变换矩阵
     * @param roll   绕 X 轴旋转角 [rad]
     * @param pitch  绕 Y 轴旋转角 [rad]
     * @param yaw    绕 Z 轴旋转角 [rad]
     * @param tx     X 方向平移 [m]
     * @param ty     Y 方向平移 [m]
     * @param tz     Z 方向平移 [m]
     */
    void setTransformMatrix(Eigen::Matrix4f& transform, float roll, float pitch, float yaw, float tx, float ty, float tz)
    {
        Eigen::AngleAxisf rollAngle(roll, Eigen::Vector3f::UnitX());
        Eigen::AngleAxisf pitchAngle(pitch, Eigen::Vector3f::UnitY());
        Eigen::AngleAxisf yawAngle(yaw, Eigen::Vector3f::UnitZ());
        Eigen::Quaternion<float> q = yawAngle * pitchAngle * rollAngle;
        Eigen::Matrix3f rot = q.matrix();
        transform.setIdentity();
        transform.block<3,3>(0,0) = rot;
        transform(0,3) = tx;
        transform(1,3) = ty;
        transform(2,3) = tz;
    }

    // ROS
    ros::NodeHandle nh_, pnh_;

    // message_filters
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2,
                                                            sensor_msgs::PointCloud2> SyncPolicy;
    message_filters::Subscriber<sensor_msgs::PointCloud2> cloud1_sub_;  ///< LiDAR 1 订阅
    message_filters::Subscriber<sensor_msgs::PointCloud2> cloud2_sub_;  ///< LiDAR 2 订阅
    boost::shared_ptr<message_filters::Synchronizer<SyncPolicy>> sync_; ///< 近似时间同步器

    // publisher
    ros::Publisher merged_pub_;   ///< 合并后点云发布者 (/merged_cloud)

    // params
    std::string cloud1_topic_;    ///< LiDAR 1 话题名
    std::string cloud2_topic_;    ///< LiDAR 2 话题名
    double roll1_, pitch1_, yaw1_, tx1_, ty1_, tz1_;   ///< LiDAR 1 6-DOF 变换参数
    double roll2_, pitch2_, yaw2_, tx2_, ty2_, tz2_;   ///< LiDAR 2 6-DOF 变换参数
};

// ============================================================================
// 主函数
// ============================================================================

/**
 * @brief 程序入口：创建 MergeCloudNode 实例并进入 ROS spin。
 *
 * 用法：
 *   rosrun uav_util merge_cloud_node _cloud1_topic:=/livox/lidar1 _tx1:=0.5 _roll2:=0.1
 */
int main(int argc, char** argv)
{
    ros::init(argc, argv, "merge_cloud_node");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    MergeCloudNode node(nh, pnh);
    ros::spin();
    return 0;
}
