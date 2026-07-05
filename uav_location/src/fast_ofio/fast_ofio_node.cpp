/**
 * @file fast_ofio_node.cpp
 * @brief Fast Optical-Flow Inertial Odometry (fast_ofio) — ROS 节点主入口。
 *
 * 订阅 MAVROS 光流 + IMU，驱动 OfioEstimator，发布 nav_msgs/Odometry。
 *
 * 话题：
 *   订阅 — 光流 (mavros_msgs::OpticalFlowRad), IMU (sensor_msgs::Imu)
 *   发布 — /Odometry (nav_msgs::Odometry)
 *
 * 使用：
 *   rosrun uav_location fast_ofio
 *   roslaunch uav_location fast_ofio.launch
 */

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <mavros_msgs/OpticalFlowRad.h>
#include <mutex>

#include "ofio_estimator.h"

static fast_ofio::OfioEstimator* g_estimator = nullptr;
static std::mutex g_data_mutex;
static ros::Publisher g_odom_pub;

// ============================================================================
// 回调
// ============================================================================

void opticalFlowCallback(const mavros_msgs::OpticalFlowRad::ConstPtr& msg)
{
    std::lock_guard<std::mutex> lock(g_data_mutex);
    if (!g_estimator) return;

    fast_ofio::OpticalFlowMeasurement flow;
    flow.stamp               = msg->header.stamp;
    flow.integration_time_us = static_cast<double>(msg->integration_time_us);
    flow.integrated_x        = static_cast<double>(msg->integrated_x);
    flow.integrated_y        = static_cast<double>(msg->integrated_y);
    flow.integrated_xgyro    = static_cast<double>(msg->integrated_xgyro);
    flow.integrated_ygyro    = static_cast<double>(msg->integrated_ygyro);
    flow.integrated_zgyro    = static_cast<double>(msg->integrated_zgyro);
    flow.distance            = static_cast<double>(msg->distance);
    flow.temperature         = static_cast<double>(msg->temperature) * 0.01;
    flow.quality             = msg->quality;

    g_estimator->updateOpticalFlow(flow);
}

void imuCallback(const sensor_msgs::Imu::ConstPtr& msg)
{
    std::lock_guard<std::mutex> lock(g_data_mutex);
    if (!g_estimator) return;

    fast_ofio::ImuMeasurement imu;
    imu.stamp = msg->header.stamp;
    imu.angular_velocity.x() = msg->angular_velocity.x;
    imu.angular_velocity.y() = msg->angular_velocity.y;
    imu.angular_velocity.z() = msg->angular_velocity.z;
    imu.linear_acceleration.x() = msg->linear_acceleration.x;
    imu.linear_acceleration.y() = msg->linear_acceleration.y;
    imu.linear_acceleration.z() = msg->linear_acceleration.z;
    imu.orientation.w() = msg->orientation.w;
    imu.orientation.x() = msg->orientation.x;
    imu.orientation.y() = msg->orientation.y;
    imu.orientation.z() = msg->orientation.z;

    g_estimator->updateImu(imu);
}

// ============================================================================
// 定时发布
// ============================================================================

void publishTimerCallback(const ros::TimerEvent& event)
{
    (void)event;

    std::lock_guard<std::mutex> lock(g_data_mutex);
    if (!g_estimator || !g_estimator->isInitialized()) return;

    g_estimator->checkTimeout(ros::Time::now());

    nav_msgs::Odometry odom = g_estimator->getOdometry();
    g_odom_pub.publish(odom);
}

// ============================================================================
// 入口
// ============================================================================

int main(int argc, char** argv)
{
    ros::init(argc, argv, "fast_ofio");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    ROS_INFO("[fast_ofio] ============================================");
    ROS_INFO("[fast_ofio] Fast Optical-Flow Inertial Odometry");
    ROS_INFO("[fast_ofio] ============================================");

    // 加载配置
    fast_ofio::OfioConfig config;
    config.loadFromParamServer(pnh);

    ROS_INFO("[fast_ofio] Flow topic:  %s", config.flow_topic.c_str());
    ROS_INFO("[fast_ofio] IMU topic:   %s", config.imu_topic.c_str());
    ROS_INFO("[fast_ofio] Odom topic:  %s", config.odom_topic.c_str());
    ROS_INFO("[fast_ofio] Frame:       %s -> %s",
             config.world_frame_id.c_str(), config.body_frame_id.c_str());
    ROS_INFO("[fast_ofio] Flow offset: (%.3f, %.3f, %.3f) m",
             config.flow_pos_x, config.flow_pos_y, config.flow_pos_z);
    ROS_INFO("[fast_ofio] Scale: %.3f  Rot: %.2f rad  Qmin: %d",
             config.scale_factor, config.flow_rotation, config.quality_min);
    ROS_INFO("[fast_ofio] Height range: [%.2f, %.2f] m  Publish: %.1f Hz",
             config.min_height, config.max_height, config.publish_rate);
    ROS_INFO("[fast_ofio] Init: (%.2f,%.2f,%.2f) yaw=%.1f deg",
             config.initial_x, config.initial_y, config.initial_z,
             config.initial_yaw * 180.0 / M_PI);

    // 估计器
    g_estimator = new fast_ofio::OfioEstimator(config);

    // 订阅
    ros::Subscriber sub_flow = nh.subscribe<mavros_msgs::OpticalFlowRad>(
        config.flow_topic, 10, opticalFlowCallback);
    ros::Subscriber sub_imu = nh.subscribe<sensor_msgs::Imu>(
        config.imu_topic, 50, imuCallback);

    // 定时发布
    g_odom_pub = nh.advertise<nav_msgs::Odometry>(config.odom_topic, 10);
    ros::Timer pub_timer = nh.createTimer(
        ros::Duration(1.0 / config.publish_rate), publishTimerCallback);

    ROS_INFO("[fast_ofio] Ready. Waiting for data...");

    ros::spin();

    delete g_estimator;
    g_estimator = nullptr;
    return 0;
}
