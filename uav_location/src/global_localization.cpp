// fast_lio_localization.cpp
// 编译: 见下方 CMakeLists.txt / package.xml
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

#include <tf/transform_datatypes.h>
#include <tf/LinearMath/Transform.h>
#include <Eigen/Core>
#include <Eigen/Geometry>

using PointT = pcl::PointXYZ;
using CloudT = pcl::PointCloud<PointT>;

std::mutex mutex_;

CloudT::Ptr global_map (new CloudT);
CloudT::Ptr cur_scan (new CloudT);
nav_msgs::Odometry::ConstPtr cur_odom_msg;
std::atomic<bool> got_global_map(false);
std::atomic<bool> got_scan(false);
std::atomic<bool> initialized(false);
Eigen::Matrix4f initial;

double MAP_VOXEL_SIZE = 0.4;
double SCAN_VOXEL_SIZE = 0.1;
double FREQ_LOCALIZATION = 0.5; // Global localization frequency (HZ)
double LOCALIZATION_TH = 0.95;  // The threshold of global localization, only those scan2map-matching with higher fitness than LOCALIZATION_TH will be taken
double FOV = 6.28;              // FOV(rad), modify this according to your LiDAR type
double FOV_FAR = 50.0;          // The farthest distance(meters) within FOV

ros::Publisher pub_pc_in_map;
ros::Publisher pub_submap;
ros::Publisher pub_map_to_odom;

ros::Subscriber sub_cloud_registered;
ros::Subscriber sub_odom;
ros::Subscriber sub_map;
ros::Subscriber sub_initialpose;

// helper: convert PoseWithCovarianceStamped -> Eigen::Matrix4f
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

// voxel downsample helper
CloudT::Ptr voxelDownSample(const CloudT::Ptr &input, double leaf)
{
    CloudT::Ptr out(new CloudT);
    pcl::VoxelGrid<PointT> vg;
    vg.setInputCloud(input);
    vg.setLeafSize(leaf, leaf, leaf);
    vg.filter(*out);
    return out;
}

// registration at scale: use PCL ICP point-to-point, returns transformation and fitness
std::pair<Eigen::Matrix4f, double> registrationAtScale(const CloudT::Ptr &scan,
                                                       const CloudT::Ptr &map,
                                                       const Eigen::Matrix4f &initial,
                                                       double scale)
{
    // downsample
    double scan_leaf = SCAN_VOXEL_SIZE * scale;
    double map_leaf = MAP_VOXEL_SIZE * scale;

    CloudT::Ptr scan_down = voxelDownSample(scan, scan_leaf);
    CloudT::Ptr map_down = voxelDownSample(map, map_leaf);

    // pcl::IterativeClosestPoint<PointT, PointT> icp;
    // icp.setInputSource(scan_down);
    // icp.setInputTarget(map_down);
    // icp.setMaximumIterations(20);
    // icp.setMaxCorrespondenceDistance(1.0 * scale + 0.001); // tune if needed

    // CloudT aligned;
    // icp.align(aligned, initial.cast<float>());

    // Eigen::Matrix4f final_tf = icp.getFinalTransformation();
    // double fitness = icp.hasConverged() ? icp.getFitnessScore() : 1e9; // lower is better for PCL getFitnessScore

    // // NOTE: PCL's getFitnessScore is a distance-based metric (lower better).
    // // In python code fitness was high-is-better. To mimic threshold semantics we will invert:
    // // define "pseudo-fitness" = exp(-fitness) so that higher is better, but to keep simple:
    // // We'll map: pseudo_fitness = 1.0 / (1.0 + fitness)
    // double pseudo_fitness = 1.0 / (1.0 + fitness);

    // return std::make_pair(final_tf, pseudo_fitness);

    fast_gicp::FastGICP<PointT, PointT> icp;
    icp.setNumThreads(4); 
    icp.setInputSource(scan_down);
    icp.setInputTarget(map_down);
    icp.setMaximumIterations(20);
    icp.setMaxCorrespondenceDistance(1.0 * scale + 0.001);

    CloudT aligned;
    icp.align(aligned, initial);

    Eigen::Matrix4f final_tf = icp.getFinalTransformation();
    
    // fast_gicp 的 fitness 计算方式与 PCL 类似
    double fitness = icp.hasConverged() ? icp.getFitnessScore() : 1e9; // lower is better for PCL getFitnessScore
    double pseudo_fitness = 1.0 / (1.0 + fitness);

    return std::make_pair(final_tf, pseudo_fitness);
}

// crop global map in FOV relative to a pose_estimation (T_map_to_odom) and current odom (odom in msg form)
CloudT::Ptr cropGlobalMapInFOV(const CloudT::Ptr &global_map_in,
                               const Eigen::Matrix4f &pose_estimation,
                               const nav_msgs::Odometry::ConstPtr &cur_odom)
{
    // convert odom pose to transform from odom -> base_link (we assume cur_odom is odom->base)
    Eigen::Matrix4f T_odom_to_base = odomToMatrix(cur_odom);
    // T_map_to_base = pose_estimation * T_odom_to_base    (here pose_estimation is map->odom or map->odom?)
    // In Python code, they computed T_map_to_base_link = pose_estimation * T_odom_to_base_link
    Eigen::Matrix4f T_map_to_base = pose_estimation * T_odom_to_base;
    Eigen::Matrix4f T_base_to_map = T_map_to_base.inverse();

    CloudT::Ptr out(new CloudT);
    out->reserve(global_map_in->size());

    for (const auto &pt : global_map_in->points) {
        Eigen::Vector4f p_map(pt.x, pt.y, pt.z, 1.0f);
        Eigen::Vector4f p_base = T_base_to_map * p_map; // point represented in base_link frame
        float x = p_base.x();
        float y = p_base.y();
        float z = p_base.z();

        bool keep = false;
        if (FOV > 3.14) {
            // ring lidar: only distance filter but original python also checks angle < FOV/2; here keep similar
            float ang = std::abs(std::atan2(y, x));
            if ((x*x + y*y) <= (FOV_FAR*FOV_FAR) && ang < (FOV/2.0 + 1e-6)) keep = true;
        } else {
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

CloudT::Ptr cropPointCloudByDistance(const CloudT::Ptr& input_cloud, 
                                     const nav_msgs::Odometry::ConstPtr& center_point, 
                                     double radius) 
{
    CloudT::Ptr filtered_cloud(new CloudT);
    filtered_cloud->header = input_cloud->header;
    filtered_cloud->is_dense = input_cloud->is_dense;
    
    for (const auto& point : input_cloud->points) {
        // 计算点到中心的XY平面距离
        double dx = point.x - center_point->pose.pose.position.x;
        double dy = point.y - center_point->pose.pose.position.y;
        double distance = sqrt(dx * dx + dy * dy);
        
        // 如果距离在设定半径内，保留该点
        if (distance <= radius) {
            filtered_cloud->points.push_back(point);
        }
    }
    
    return filtered_cloud;
}

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
        // 按中心点进行半径裁剪
        double dx = point.x - center_point->pose.pose.position.x;
        double dy = point.y - center_point->pose.pose.position.y;
        double distance = std::sqrt(dx * dx + dy * dy);

        if (distance <= radius) 
        {
            // 将点转为齐次坐标
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

// publish pcl cloud (XYZ) as sensor_msgs::PointCloud2 with header
void publishPointCloud(const CloudT::Ptr &cloud, const std_msgs::Header &header, ros::Publisher &pub)
{
    sensor_msgs::PointCloud2 msg;
    pcl::toROSMsg(*cloud, msg);
    msg.header = header;
    pub.publish(msg);
}

// global localization: several steps, returns true if successful and publishes /map_to_odom
bool globalLocalization(Eigen::Matrix4f &T_map_to_odom)
{
    std::lock_guard<std::mutex> lock(mutex_);
    if (!got_global_map || !got_scan || !cur_odom_msg) {
        ROS_WARN_THROTTLE(5.0, "[global] Waiting for map/scan/odom...");
        return false;
    }

    ROS_INFO_THROTTLE(60.0, "[global] Global localization by scan-to-map matching......");

    // crop map in FOV relative to current odom
    CloudT::Ptr submap = cropGlobalMapInFOV(global_map, T_map_to_odom, cur_odom_msg);
    if (submap->empty()) {
        ROS_WARN("[global] submap empty after cropping");
        return false;
    }

    // publish submap sparse for visualization (downsample heavier)
    CloudT::Ptr submap_vis = voxelDownSample(submap, MAP_VOXEL_SIZE);
    std_msgs::Header hdr;
    hdr.stamp = ros::Time::now();
    hdr.frame_id = "map";
    publishPointCloud(submap_vis, hdr, pub_submap);

    // prepare scan (already in cur_scan in odom frame? In original python, they used cur_scan as given)
    CloudT::Ptr scan_copy(new CloudT(*cur_scan));

    // coarse registration (scale=5)
    auto coarse = registrationAtScale(scan_copy, submap, T_map_to_odom, 5.0);
    Eigen::Matrix4f tf_coarse = coarse.first;
    double fitness_coarse = coarse.second;
    // fine registration (scale=1)
    auto fine = registrationAtScale(scan_copy, submap, tf_coarse, 1.0);
    Eigen::Matrix4f tf_fine = fine.first;
    double fitness = fine.second;

    if (fitness > LOCALIZATION_TH) {
        //ROS_INFO("[global] Localization fitness (pseudo) = %.3f", fitness);
        // update map->odom
        T_map_to_odom = tf_fine;

        // publish Odometry message as map_to_odom
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

// callbacks
void cbSaveCurOdom(const nav_msgs::Odometry::ConstPtr &odom_msg)
{
    std::lock_guard<std::mutex> lock(mutex_);
    cur_odom_msg = odom_msg;
}

void cbSaveCurScan(const sensor_msgs::PointCloud2::ConstPtr &pc_msg)
{
    // similar to python: assume points already in odom or camera_init frame
    std::lock_guard<std::mutex> lock(mutex_);
    CloudT::Ptr tmp(new CloudT);
    pcl::fromROSMsg(*pc_msg, *tmp);
    // store as cur_scan (XYZ)
    cur_scan = cropPointCloudByDistance(tmp, cur_odom_msg, FOV_FAR);
    got_scan = true;

    // publish pc_in_map equivalent - here we just reuse the incoming message header but set frame to camera_init
    CloudT::Ptr scan_vis = voxelDownSample(cur_scan, SCAN_VOXEL_SIZE);
    std_msgs::Header hdr;
    hdr.stamp = ros::Time::now();
    hdr.frame_id = "camera_init";
    publishPointCloud(scan_vis, hdr, pub_pc_in_map);
}

// initialize global map callback or initial message
void cbInitGlobalMap(const sensor_msgs::PointCloud2::ConstPtr &pc_msg)
{
    std::lock_guard<std::mutex> lock(mutex_);
    CloudT::Ptr tmp(new CloudT);
    pcl::fromROSMsg(*pc_msg, *tmp);
    // downsample
    global_map = voxelDownSample(tmp, MAP_VOXEL_SIZE);
    got_global_map = true;
    ROS_INFO("[global] Global map received, points: %zu", global_map->size());
}

// initial pose callback: user might publish /initialpose -> try to initialize
void cbInitialPose(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &pose_msg)
{
    std::lock_guard<std::mutex> lock(mutex_);
    initial = poseWithCovToMatrix(pose_msg);
    ROS_INFO("[global] get initial pose!");
    initialized = true;
}

// thread to run periodic localization (uses a stored T_map_to_odom)
void threadLocalization()
{
    Eigen::Matrix4f T_map_to_odom = Eigen::Matrix4f::Identity();
    ros::Rate rate(FREQ_LOCALIZATION);
    while (ros::ok() && !initialized)
    {
        ROS_WARN_THROTTLE(5.0, "[global] Waiting for initialization...");
        rate.sleep();
    }
    T_map_to_odom = initial;
    while (ros::ok()) 
    {
        bool ok = globalLocalization(T_map_to_odom);
        (void)ok;
        rate.sleep();
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "fast_lio_localization");
    ros::NodeHandle nh("~");
    ROS_INFO("[global] Localization Node Inited...");

    std::string map_in_topic;
    nh.param<double>("map_voxel_size", MAP_VOXEL_SIZE, MAP_VOXEL_SIZE);
    nh.param<double>("scan_voxel_size", SCAN_VOXEL_SIZE, SCAN_VOXEL_SIZE);
    nh.param<double>("freq_localization", FREQ_LOCALIZATION, FREQ_LOCALIZATION);
    nh.param<double>("localization_th", LOCALIZATION_TH, LOCALIZATION_TH);
    nh.param<double>("fov", FOV, FOV);
    nh.param<double>("fov_far", FOV_FAR, FOV_FAR);
    nh.param<std::string>("map_in_topic", map_in_topic, "/map");

    pub_pc_in_map = nh.advertise<sensor_msgs::PointCloud2>("/cur_scan_in_map", 10);
    pub_submap = nh.advertise<sensor_msgs::PointCloud2>("/submap", 10);
    pub_map_to_odom = nh.advertise<nav_msgs::Odometry>("/map_to_odom", 10);

    sub_cloud_registered = nh.subscribe("/cloud_registered", 5, cbSaveCurScan);
    sub_odom = nh.subscribe("/Odometry", 10, cbSaveCurOdom);
    sub_map = nh.subscribe(map_in_topic, 5, cbInitGlobalMap);
    sub_initialpose = nh.subscribe("/initialpose", 5, cbInitialPose);

    // Wait until map is received (blocking similar to rospy.wait_for_message)
    ROS_WARN("[global] Waiting for global map......");
    ros::Rate wait_rate(5.0);
    while (ros::ok() && !got_global_map) {
        ros::spinOnce();
        wait_rate.sleep();
    }

    std::thread th(threadLocalization);
    th.detach();

    ros::spin();

    return 0;
}
