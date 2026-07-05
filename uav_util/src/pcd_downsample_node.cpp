/**
 * @file pcd_downsample_node.cpp
 * @brief PCD 点云体素降采样节点 —— 读取 PCD 文件，应用 VoxelGrid 滤波，保存并发布。
 *
 * @details
 * 用于对已有 PCD 点云地图进行离线降采样的 ROS 节点，核心功能：
 *   1. 读取 PCD        — 从参数指定的路径加载 PointXYZ 点云
 *   2. 体素降采样      — 使用 pcl::VoxelGrid 均匀降采样，leaf_size 可配置
 *   3. 保存            — 输出文件命名为 <原文件名>_downsampled.<扩展名>
 *   4. 循环发布        — 以 1 [Hz] 频率发布到 /downsampled_cloud 话题供 RViz 可视化
 *
 * 参数：
 *   - pcd_path  (string): 输入 PCD 文件路径（必填）
 *   - leaf_size (double): 体素立方体边长 [m]，默认 0.05
 *
 * 用法：
 *   rosrun uav_util pcd_downsample_node _pcd_path:=/path/to/map.pcd _leaf_size:=0.1
 */

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>
#include <string>
#include <filesystem>

// ============================================================================
// 主函数
// ============================================================================

/**
 * @brief 程序入口：读取参数 → 加载 PCD → VoxelGrid 滤波 → 保存 → 循环发布。
 *
 * 用法：
 *   rosrun uav_util pcd_downsample_node _pcd_path:=/path/to/map.pcd _leaf_size:=0.1
 */
int main(int argc, char** argv)
{
    // ============================================================
    // 初始化
    // ============================================================
    ros::init(argc, argv, "pcd_downsample_node");
    ros::NodeHandle nh("~");

    std::string input_pcd_path;
    double leaf_size;

    // 从参数服务器读取参数
    nh.param<std::string>("pcd_path", input_pcd_path, "");
    nh.param<double>("leaf_size", leaf_size, 0.05);             ///< 体素立方体边长 [m]

    if (input_pcd_path.empty()) {
        ROS_ERROR("Parameter 'pcd_path' is not set!");
        ros::shutdown();
        return -1;
    }

    if (!std::filesystem::exists(input_pcd_path)) {
        ROS_ERROR_STREAM("File not found: " << input_pcd_path);
        ros::shutdown();
        return -1;
    }

    // ============================================================
    // 读取原始点云
    // ============================================================
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
    if (pcl::io::loadPCDFile(input_pcd_path, *cloud) == -1) {
        ROS_ERROR_STREAM("Failed to load PCD file: " << input_pcd_path);
        ros::shutdown();
        return -1;
    }
    ROS_INFO("Loaded point cloud: %s, total points: %zu", input_pcd_path.c_str(), cloud->size());

    // ============================================================
    // 体素降采样
    // ============================================================
    pcl::VoxelGrid<pcl::PointXYZ> voxel_filter;
    voxel_filter.setInputCloud(cloud);
    voxel_filter.setLeafSize(leaf_size, leaf_size, leaf_size);  ///< 3D 体素尺寸 [m]

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>());
    voxel_filter.filter(*cloud_filtered);

    ROS_INFO("Downsampling complete: %zu -> %zu", cloud->size(), cloud_filtered->size());

    // ============================================================
    // 保存降采样后的点云
    // ============================================================
    std::filesystem::path input_path(input_pcd_path);
    std::string stem = input_path.stem().string();               ///< 文件名（无扩展名）
    std::string ext = input_path.extension().string();           ///< 扩展名
    std::filesystem::path output_path = input_path.parent_path() / (stem + "_downsampled" + ext);

    pcl::io::savePCDFileBinary(output_path.string(), *cloud_filtered);
    ROS_INFO("Downsampled point cloud saved to: %s", output_path.c_str());

    // ============================================================
    // 循环发布
    // ============================================================
    ros::Publisher pub = nh.advertise<sensor_msgs::PointCloud2>("/downsampled_cloud", 1, true);
    sensor_msgs::PointCloud2 msg;
    pcl::toROSMsg(*cloud_filtered, msg);
    msg.header.frame_id = "map";

    ROS_INFO("Published downsampled point cloud to /downsampled_cloud");

    ros::Rate r(1.0);                                           ///< 发布频率 1 [Hz]
    while (ros::ok())
    {
        r.sleep();
        ros::spinOnce();
        pub.publish(msg);
    }

    ros::shutdown();
    return 0;
}
