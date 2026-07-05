/**
 * @file global_localization.cpp
 * @brief 基于 FastGICP 的全局 scan-to-map 定位节点。
 *
 * @details
 * 核心功能：
 *   1. 接收全局地图点云 (PCD) 和实时激光扫描点云
 *   2. 使用 FastGICP (可降级为 PCL ICP) 进行 scan-to-map 配准
 *   3. 在 FOV 范围内裁剪地图子图以提高配准效率
 *   4. 多尺度配准：粗配准 (scale=5) → 精配准 (scale=1)
 *   5. 发布 map→odom 变换 (nav_msgs::Odometry)
 *   6. 支持 /initialpose 话题初始化初始位姿
 *
 * 算法流水线：
 *   global_map → cropGlobalMapInFOV → registrationAtScale(coarse, scale=5)
 *     → registrationAtScale(fine, scale=1) → 收敛判定 → 发布 /map_to_odom
 *
 * 坐标系：
 *   - map (全局固定坐标系)
 *   - odom (里程计漂移坐标系)
 *   - camera_init / base_link (机体坐标系)
 *
 * 节点名：fast_lio_localization (历史命名，实际使用 FastGICP 算法)
 * 使用：
 *   rosrun uav_location fast_lio_localization
 *   roslaunch uav_location global_planner_in_sim.launch
 *
 * @note 原 Python 参考实现的重写 C++ 版本。
 */

#include <ros/ros.h>
#include <thread>
#include <mutex>
#include <atomic>
#include <chrono>

#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/registration/icp.h>

#include <fast_gicp/gicp/fast_gicp.hpp>
#include <fast_gicp/gicp/fast_vgicp.hpp>
#define FAST_ICP_ENABLE    ///< 启用 FastGICP 加速配准；注释掉回退至 PCL ICP

#include <tf/transform_datatypes.h>
#include <tf/LinearMath/Transform.h>
#include <Eigen/Core>
#include <Eigen/Geometry>

// ============================================================================
// 类型别名与全局变量
// ============================================================================

using PointT = pcl::PointXYZ;
using CloudT = pcl::PointCloud<PointT>;

/// @brief 全局数据互斥锁，保护以下所有共享变量
std::mutex mutex_;

CloudT::Ptr global_map (new CloudT);       ///< 全局地图点云
CloudT::Ptr cur_scan (new CloudT);         ///< 当前帧扫描点云
nav_msgs::Odometry::ConstPtr cur_odom_msg; ///< 最新里程计消息
std::atomic<bool> got_global_map(false);   ///< 是否已接收全局地图
std::atomic<bool> got_scan(false);         ///< 是否已接收首次扫描
std::atomic<bool> initialized(false);      ///< 是否已接收初始位姿
Eigen::Matrix4f initial;                   ///< 初始位姿矩阵 (map→odom)

// ============================================================================
// 可配置参数 (由 ROS param 加载)
// ============================================================================

int    icp_max_iter       = 20;      ///< ICP 最大迭代次数
double MAP_VOXEL_SIZE     = 0.4;     ///< 地图下采样体素尺寸 [m]
double SCAN_VOXEL_SIZE    = 0.1;     ///< 扫描下采样体素尺寸 [m]
double FREQ_LOCALIZATION  = 0.5;     ///< 全局定位发布频率 [Hz]
double LOCALIZATION_TH    = 0.95;    ///< ICP 伪 fitness 收敛阈值 (0~1，越大越准)
double FOV                = 6.28;    ///< LiDAR 水平视场角 [rad] (360°=6.28)
double FOV_FAR            = 50.0;    ///< FOV 内最大有效距离 [m]

// ============================================================================
// ROS 发布器/订阅器
// ============================================================================

ros::Publisher pub_pc_in_map;        ///< /cur_scan_in_map — 当前扫描在 map 系的可视化
ros::Publisher pub_submap;           ///< /submap — 当前配准用的局部子图可视化
ros::Publisher pub_map_to_odom;      ///< /map_to_odom — 全局定位结果

ros::Subscriber sub_cloud_registered; ///< /cloud_registered — 输入激光点云
ros::Subscriber sub_odom;            ///< /Odometry — 输入局部里程计
ros::Subscriber sub_map;             ///< /map — 输入全局地图
ros::Subscriber sub_initialpose;     ///< /initialpose — 初始位姿估计

// ============================================================================
// 工具函数：位姿消息 ↔ 变换矩阵
// ============================================================================

/**
 * @brief 将 geometry_msgs::Pose 转换为 Eigen::Matrix4f。
 * @param p 输入位姿消息
 * @return 4×4 齐次变换矩阵
 */
Eigen::Matrix4f poseMsgToMatrix(const geometry_msgs::Pose &p)
{
    Eigen::Quaternionf q(p.orientation.w, p.orientation.x, p.orientation.y, p.orientation.z);
    Eigen::Matrix4f T = Eigen::Matrix4f::Identity();
    T.block<3,3>(0,0) = q.normalized().toRotationMatrix();
    T(0,3) = p.position.x;      // | q11 q12 q13 Tx |
    T(1,3) = p.position.y;      // | q21 q22 q23 Ty |
    T(2,3) = p.position.z;      // | q31 q32 q33 Tz |
    return T;                   // |  0   0   0   1 |
}

Eigen::Matrix4f poseWithCovToMatrix(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg)
{
    return poseMsgToMatrix(msg->pose.pose);
}

Eigen::Matrix4f odomToMatrix(const nav_msgs::Odometry::ConstPtr &msg)
{
    return poseMsgToMatrix(msg->pose.pose);
}

// ============================================================================
// 点云处理工具
// ============================================================================

/**
 * @brief VoxelGrid 体素下采样。
 * @param input 输入点云
 * @param leaf  体素边长 [m]
 * @return 下采样后的点云
 */
CloudT::Ptr voxelDownSample(const CloudT::Ptr &input, double leaf)
{
    CloudT::Ptr out(new CloudT);
    pcl::VoxelGrid<PointT> vg;
    vg.setInputCloud(input);
    vg.setLeafSize(leaf, leaf, leaf);
    vg.filter(*out);
    return out;
}

/**
 * @brief 多尺度 ICP 配准。
 *
 * 在给定 scale 下同时对扫描和地图降采样后进行 ICP 配准。
 * scale 越大降采样越激进，速度越快但精度越低。
 *
 * @param scan    源点云（激光扫描）
 * @param map     目标点云（地图子图）
 * @param initial 初始猜测变换矩阵
 * @param scale   降采样尺度因子 (>1 = 更大的体素)
 * @return (最终变换矩阵, 伪_fitness) — fitness 越高越好 (1/(1+fitness_mse))
 */
std::pair<Eigen::Matrix4f, double> registrationAtScale(const CloudT::Ptr &scan,
                                                       const CloudT::Ptr &map,
                                                       const Eigen::Matrix4f &initial,
                                                       double scale)
{
    // 按 scales 下采样
    double scan_leaf = SCAN_VOXEL_SIZE * scale;
    double map_leaf  = MAP_VOXEL_SIZE * scale;

    CloudT::Ptr scan_down = voxelDownSample(scan, scan_leaf);
    CloudT::Ptr map_down  = voxelDownSample(map, map_leaf);

#ifndef FAST_ICP_ENABLE
    pcl::IterativeClosestPoint<PointT, PointT> icp;
#else
    fast_gicp::FastGICP<PointT, PointT> icp;
    icp.setNumThreads(4);
#endif

    icp.setInputSource(scan_down);
    icp.setInputTarget(map_down);
    icp.setMaximumIterations(icp_max_iter);
    icp.setMaxCorrespondenceDistance(1.0 * scale + 0.001);

    CloudT aligned;
    icp.align(aligned, initial.cast<float>());

    Eigen::Matrix4f final_tf = icp.getFinalTransformation();
    double fitness = icp.hasConverged() ? icp.getFitnessScore() : 1e9;

    // PCL getFitnessScore 是距离度量 (越低越好)。
    // 原始 Python 代码中 fitness 高=好，此处映射: pseudo_fitness = 1/(1+fitness)
    double pseudo_fitness = 1.0 / (1.0 + fitness);

    return std::make_pair(final_tf, pseudo_fitness);
}

/**
 * @brief 在 FOV 范围内裁剪全局地图。
 *
 * 根据当前位姿估计和里程计，将全局地图投影到机体坐标系，
 * 仅保留传感器 FOV 内的地图点，减少配准计算量。
 *
 * @param global_map_in  完整全局地图
 * @param pose_estimation 当前 map→odom 变换估计
 * @param cur_odom        当前局部里程计
 * @return FOV 内的局部子图
 */
CloudT::Ptr cropGlobalMapInFOV(const CloudT::Ptr &global_map_in,
                               const Eigen::Matrix4f &pose_estimation,
                               const nav_msgs::Odometry::ConstPtr &cur_odom)
{
    // T_odom_to_base: 局部里程计 (odom → base_link)
    Eigen::Matrix4f T_odom_to_base = odomToMatrix(cur_odom);
    // T_map_to_base = map→odom × odom→base
    Eigen::Matrix4f T_map_to_base = pose_estimation * T_odom_to_base;
    Eigen::Matrix4f T_base_to_map = T_map_to_base.inverse();

    CloudT::Ptr out(new CloudT);
    out->reserve(global_map_in->size());

    for (const auto &pt : global_map_in->points) {
        Eigen::Vector4f p_map(pt.x, pt.y, pt.z, 1.0f);
        Eigen::Vector4f p_base = T_base_to_map * p_map; // 点转到机体坐标系
        float x = p_base.x();
        float y = p_base.y();
        float z = p_base.z();

        bool keep = false;
        if (FOV > 3.14) {
            // 360° LiDAR: 仅距离滤波，角度 < FOV/2
            float ang = std::abs(std::atan2(y, x));
            if ((x*x + y*y) <= (FOV_FAR*FOV_FAR) && ang < (FOV/2.0 + 1e-6)) keep = true;
        } else {
            // 前向 LiDAR: 仅保留前方 (x>0) + 距离 + 角度
            float ang = std::abs(std::atan2(y, x));
            float r = std::sqrt(x*x + y*y);
            if ((x > 0) && (r < FOV_FAR) && ang < (FOV/2.0 + 1e-6)) keep = true;
        }

        if (keep) {
            PointT p; p.x = pt.x; p.y = pt.y; p.z = pt.z;
            out->push_back(p);
        }
    }

    return out;
}

/**
 * @brief 按 XY 平面距离裁剪点云。
 * @param input_cloud 输入点云
 * @param center_point 中心点 (odom 消息)
 * @param radius     裁剪半径 [m]
 * @return 裁剪后的点云
 */
CloudT::Ptr cropPointCloudByDistance(const CloudT::Ptr& input_cloud,
                                     const nav_msgs::Odometry::ConstPtr& center_point,
                                     double radius)
{
    CloudT::Ptr filtered_cloud(new CloudT);
    filtered_cloud->header = input_cloud->header;
    filtered_cloud->is_dense = input_cloud->is_dense;

    for (const auto& point : input_cloud->points) {
        double dx = point.x - center_point->pose.pose.position.x;
        double dy = point.y - center_point->pose.pose.position.y;
        double distance = sqrt(dx * dx + dy * dy);

        if (distance <= radius) {
            filtered_cloud->points.push_back(point);
        }
    }

    return filtered_cloud;
}

/**
 * @brief 按 XY 距离裁剪并变换点云到 map 坐标系。
 * @param input_cloud  输入点云 (在 odom 系)
 * @param pose_estimation map→odom 变换估计
 * @param center_point 中心点
 * @param radius      裁剪半径 [m]
 * @return 裁剪并变换后的点云 (在 map 系)
 */
CloudT::Ptr cropPointCloudByDistance(const CloudT::Ptr& input_cloud,
                                     const Eigen::Matrix4f &pose_estimation,
                                     const nav_msgs::Odometry::ConstPtr& center_point,
                                     double radius)
{
    CloudT::Ptr filtered_cloud(new CloudT);
    filtered_cloud->header = input_cloud->header;
    filtered_cloud->is_dense = input_cloud->is_dense;

    for (const auto& point : input_cloud->points)
    {
        double dx = point.x - center_point->pose.pose.position.x;
        double dy = point.y - center_point->pose.pose.position.y;
        double distance = std::sqrt(dx * dx + dy * dy);

        if (distance <= radius)
        {
            // 将 odom 系点变换到 map 坐标系
            Eigen::Vector4f pt(point.x, point.y, point.z, 1.0f);
            Eigen::Vector4f pt_transformed = pose_estimation * pt;

            PointT new_pt;
            new_pt.x = pt_transformed.x();
            new_pt.y = pt_transformed.y();
            new_pt.z = pt_transformed.z();

            filtered_cloud->points.push_back(new_pt);
        }
    }

    return filtered_cloud;
}

/**
 * @brief 发布 PCL 点云为 ROS sensor_msgs::PointCloud2 消息。
 * @param cloud  输入点云
 * @param header ROS 消息头 (时间戳 + frame_id)
 * @param pub    发布器
 */
void publishPointCloud(const CloudT::Ptr &cloud, const std_msgs::Header &header, ros::Publisher &pub)
{
    sensor_msgs::PointCloud2 msg;
    pcl::toROSMsg(*cloud, msg);
    msg.header = header;
    pub.publish(msg);
}

// ============================================================================
// 全局定位核心算法
// ============================================================================

/**
 * @brief 执行一次全局定位迭代。
 *
 * 流程：
 *   1. 在 FOV 内裁剪局部子图
 *   2. 粗配准 (scale=5，大降采样，快速收敛)
 *   3. 精配准 (scale=1，小降采样，精确定位)
 *   4. 收敛判定 → 发布 /map_to_odom
 *
 * @param[in,out] T_map_to_odom 当前 map→odom 变换（入参为初值，出参为更新值）
 * @return true 定位成功，false 定位失败
 */
bool globalLocalization(Eigen::Matrix4f &T_map_to_odom)
{
    std::lock_guard<std::mutex> lock(mutex_);
    if (!got_global_map || !got_scan || !cur_odom_msg) {
        ROS_WARN_THROTTLE(5.0, "[global] Waiting for map/scan/odom...");
        return false;
    }

    ROS_INFO_THROTTLE(60.0, "[global] Global localization by scan-to-map matching......");

    // 1. 裁剪 FOV 子图
    CloudT::Ptr submap = cropGlobalMapInFOV(global_map, T_map_to_odom, cur_odom_msg);
    if (submap->empty()) {
        ROS_WARN("[global] submap empty after cropping");
        return false;
    }

    // 发布子图可视化
    CloudT::Ptr submap_vis = voxelDownSample(submap, MAP_VOXEL_SIZE);
    std_msgs::Header hdr;
    hdr.stamp = ros::Time::now();
    hdr.frame_id = "map";
    publishPointCloud(submap_vis, hdr, pub_submap);

    CloudT::Ptr scan_copy(new CloudT(*cur_scan));

    // 2. 粗配准 (scale=5，激进降采样，快速收敛到大体位置)
    auto coarse = registrationAtScale(scan_copy, submap, T_map_to_odom, 5.0);
    Eigen::Matrix4f tf_coarse = coarse.first;
    double fitness_coarse = coarse.second;

    if(fitness_coarse < 1e-6)
    {
        ROS_WARN("[global] Coarse registration not converged");
        return false;
    }

    // 3. 精配准 (scale=1，精细降采样，提高精度)
    auto fine = registrationAtScale(scan_copy, submap, tf_coarse, 1.0);
    Eigen::Matrix4f tf_fine = fine.first;
    double fitness = fine.second;

    // 4. 收敛判定
    if (fitness > LOCALIZATION_TH) {
        T_map_to_odom = tf_fine;

        // 发布 Odometry 类型的 map_to_odom 消息
        nav_msgs::Odometry map_to_odom;
        Eigen::Vector3f t = T_map_to_odom.block<3,1>(0,3);
        Eigen::Matrix3f R = T_map_to_odom.block<3,3>(0,0);
        Eigen::Quaternionf q(R);

        map_to_odom.pose.pose.position.x = t.x();
        map_to_odom.pose.pose.position.y = t.y();
        map_to_odom.pose.pose.position.z = t.z();
        map_to_odom.pose.pose.orientation.x = q.x();
        map_to_odom.pose.pose.orientation.y = q.y();
        map_to_odom.pose.pose.orientation.z = q.z();
        map_to_odom.pose.pose.orientation.w = q.w();

        map_to_odom.header.stamp = ros::Time::now();
        map_to_odom.header.frame_id = "map";
        pub_map_to_odom.publish(map_to_odom);
        return true;
    } else {
        ROS_WARN("[global] ICP not converged, fitness (pseudo)=%.3f", fitness);
        return false;
    }
}

// ============================================================================
// ROS 回调函数
// ============================================================================

/// @brief 保存最新局部里程计消息
void cbSaveCurOdom(const nav_msgs::Odometry::ConstPtr &odom_msg)
{
    std::lock_guard<std::mutex> lock(mutex_);
    cur_odom_msg = odom_msg;
}

/// @brief 保存并预处理当前激光扫描
void cbSaveCurScan(const sensor_msgs::PointCloud2::ConstPtr &pc_msg)
{
    std::lock_guard<std::mutex> lock(mutex_);
    CloudT::Ptr tmp(new CloudT);
    pcl::fromROSMsg(*pc_msg, *tmp);
    // 按 FOV_FAR 距离裁剪
    cur_scan = cropPointCloudByDistance(tmp, cur_odom_msg, FOV_FAR);
    got_scan = true;

    // 发布当前扫描在 camera_init 系的可视化
    CloudT::Ptr scan_vis = voxelDownSample(cur_scan, SCAN_VOXEL_SIZE);
    std_msgs::Header hdr;
    hdr.stamp = ros::Time::now();
    hdr.frame_id = "camera_init";
    publishPointCloud(scan_vis, hdr, pub_pc_in_map);
}

/// @brief 接收全局地图 (仅接收一次)
void cbInitGlobalMap(const sensor_msgs::PointCloud2::ConstPtr &pc_msg)
{
    std::lock_guard<std::mutex> lock(mutex_);
    CloudT::Ptr tmp(new CloudT);
    pcl::fromROSMsg(*pc_msg, *tmp);
    global_map = voxelDownSample(tmp, MAP_VOXEL_SIZE);
    got_global_map = true;
    ROS_INFO("[global] Global map received, points: %zu", global_map->size());
}

/// @brief 接收用户发布的初始位姿 (/initialpose)
void cbInitialPose(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &pose_msg)
{
    std::lock_guard<std::mutex> lock(mutex_);
    initial = poseWithCovToMatrix(pose_msg);
    ROS_INFO("[global] get initial pose!");
    initialized = true;
}

// ============================================================================
// 定位线程
// ============================================================================

/**
 * @brief 独立定位线程，按固定频率执行全局定位。
 *
 * 阻塞直到收到 /initialpose，此后周期性调用 globalLocalization()，
 * 以 FREQ_LOCALIZATION 频率更新 map→odom 变换。
 */
void threadLocalization()
{
    Eigen::Matrix4f T_map_to_odom = Eigen::Matrix4f::Identity();
    ros::Rate rate(FREQ_LOCALIZATION);

    // 等待初始位姿
    while (ros::ok() && !initialized)
    {
        ROS_WARN_THROTTLE(5.0, "[global] Waiting for initialization...");
        rate.sleep();
    }
    T_map_to_odom = initial;

    // 主循环：周期性全局定位
    while (ros::ok())
    {
        bool ok = globalLocalization(T_map_to_odom);
        (void)ok;
        rate.sleep();
    }
}

// ============================================================================
// 主函数
// ============================================================================

int main(int argc, char** argv)
{
    ros::init(argc, argv, "fast_lio_localization");
    ros::NodeHandle nh("~");
    ROS_INFO("[global] Localization Node Inited...");

    // ---- 加载参数 ----
    std::string map_in_topic;
    nh.param<int>("icp_max_iter", icp_max_iter, 20);
    nh.param<double>("map_voxel_size", MAP_VOXEL_SIZE, MAP_VOXEL_SIZE);
    nh.param<double>("scan_voxel_size", SCAN_VOXEL_SIZE, SCAN_VOXEL_SIZE);
    nh.param<double>("freq_localization", FREQ_LOCALIZATION, FREQ_LOCALIZATION);
    nh.param<double>("localization_th", LOCALIZATION_TH, LOCALIZATION_TH);
    nh.param<double>("fov", FOV, FOV);
    nh.param<double>("fov_far", FOV_FAR, FOV_FAR);
    nh.param<std::string>("map_in_topic", map_in_topic, "/map");

    // ---- 发布器 ----
    pub_pc_in_map   = nh.advertise<sensor_msgs::PointCloud2>("/cur_scan_in_map", 10);
    pub_submap      = nh.advertise<sensor_msgs::PointCloud2>("/submap", 10);
    pub_map_to_odom = nh.advertise<nav_msgs::Odometry>("/map_to_odom", 10);

    // ---- 订阅器 ----
    sub_cloud_registered = nh.subscribe("/cloud_registered", 5, cbSaveCurScan);
    sub_odom             = nh.subscribe("/Odometry", 10, cbSaveCurOdom);
    sub_map              = nh.subscribe(map_in_topic, 5, cbInitGlobalMap);
    sub_initialpose      = nh.subscribe("/initialpose", 5, cbInitialPose);

    // ---- 等待全局地图加载 ----
    ROS_WARN("[global] Waiting for global map......");
    ros::Rate wait_rate(5.0);
    while (ros::ok() && !got_global_map) {
        ros::spinOnce();
        wait_rate.sleep();
    }

    // ---- 启动定位线程 ----
    std::thread th(threadLocalization);
    th.detach();

    ros::spin();

    return 0;
}
