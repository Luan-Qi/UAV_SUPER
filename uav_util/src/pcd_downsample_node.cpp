#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>
#include <string>
#include <filesystem>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "pcd_downsample_node");
    ros::NodeHandle nh("~");

    std::string input_pcd_path;
    double leaf_size;

    // 从参数服务器读取参数
    nh.param<std::string>("pcd_path", input_pcd_path, "");
    nh.param<double>("leaf_size", leaf_size, 0.05);

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

    // 读取原始点云
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
    if (pcl::io::loadPCDFile(input_pcd_path, *cloud) == -1) {
        ROS_ERROR_STREAM("Failed to load PCD file: " << input_pcd_path);
        ros::shutdown();
        return -1;
    }
    ROS_INFO("Loaded point cloud: %s, total points: %zu", input_pcd_path.c_str(), cloud->size());

    // 创建降采样滤波器
    pcl::VoxelGrid<pcl::PointXYZ> voxel_filter;
    voxel_filter.setInputCloud(cloud);
    voxel_filter.setLeafSize(leaf_size, leaf_size, leaf_size);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>());
    voxel_filter.filter(*cloud_filtered);

    ROS_INFO("Downsampling complete: %zu -> %zu", cloud->size(), cloud_filtered->size());

    // 构造输出文件名
    std::filesystem::path input_path(input_pcd_path);
    std::string stem = input_path.stem().string();
    std::string ext = input_path.extension().string();
    std::filesystem::path output_path = input_path.parent_path() / (stem + "_downsampled" + ext);

    // 保存降采样后的点云
    pcl::io::savePCDFileBinary(output_path.string(), *cloud_filtered);
    ROS_INFO("Downsampled point cloud saved to: %s", output_path.c_str());

    // 发布点云
    ros::Publisher pub = nh.advertise<sensor_msgs::PointCloud2>("/downsampled_cloud", 1, true);
    sensor_msgs::PointCloud2 msg;
    pcl::toROSMsg(*cloud_filtered, msg);
    msg.header.frame_id = "map";

    ROS_INFO("Published downsampled point cloud to /downsampled_cloud");

    ros::Rate r(1.0);
    while (ros::ok())
    {
        r.sleep();
        ros::spinOnce();
        pub.publish(msg);
    }
    
    ros::shutdown();
    return 0;
}
