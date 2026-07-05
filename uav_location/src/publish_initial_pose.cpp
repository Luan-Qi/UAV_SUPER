/**
 * @file publish_initial_pose.cpp
 * @brief 初始位姿发布节点 — 将命令行指定的初始位姿发布为 /initialpose 消息。
 *
 * @details
 * 功能：
 *   1. 从命令行参数读取 (x, y, z, roll, pitch, yaw) 初始位姿
 *   2. 转换为 geometry_msgs::PoseWithCovarianceStamped 发布到 /initialpose
 *   3. 随后进入空闲循环保持节点存活
 *
 * 典型用途：
 *   在全局定位流水线中，为 fast_lio_localization 或 icp_registration
 *   提供初始位姿猜测，帮助 ICP 配准从正确的位置开始搜索。
 *
 * 使用：
 *   rosrun uav_location publish_initial_pose x y z roll pitch yaw
 *   例如：
 *   rosrun uav_location publish_initial_pose 0.0 -0.3 1.0 0.0 0.0 0.0
 *
 * @note 发布 /initialpose 后节点保持运行（以维持 ROS topic 连接），
 *       但不持续发布新位姿。
 */

#include <ros/ros.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <iostream>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "publish_initial_pose");
    ros::NodeHandle nh;

    // ---- 参数检查 ----
    if (argc < 7) {
        ROS_ERROR("Usage: rosrun <package_name> publish_initial_pose x y z roll pitch yaw");
        return 1;
    }

    // ---- 解析命令行参数 ----
    double x     = atof(argv[1]);
    double y     = atof(argv[2]);
    double z     = atof(argv[3]);
    double roll  = atof(argv[4]);
    double pitch = atof(argv[5]);
    double yaw   = atof(argv[6]);

    // ---- 发布器 ----
    ros::Publisher pub_pose = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("/initialpose", 1);

    // ---- RPY → 四元数 ----
    tf::Quaternion q;
    q.setRPY(roll, pitch, yaw);
    geometry_msgs::Quaternion q_msg;
    tf::quaternionTFToMsg(q, q_msg);

    // ---- 填充 PoseWithCovarianceStamped 消息 ----
    geometry_msgs::PoseWithCovarianceStamped initial_pose;
    initial_pose.header.stamp = ros::Time::now();
    initial_pose.header.frame_id = "map";
    initial_pose.pose.pose.position.x = x;
    initial_pose.pose.pose.position.y = y;
    initial_pose.pose.pose.position.z = z;
    initial_pose.pose.pose.orientation = q_msg;

    // 协方差初始化为 0 (表示绝对确定/未知，由接收方自行处理)
    for (int i = 0; i < 36; i++)
        initial_pose.pose.covariance[i] = 0.0;

    // ---- 发布初始位姿 ----
    ros::Duration(0.5).sleep();  // 等待发布器连接
    ROS_INFO("[pub_init_pose] Publishing initial pose: x=%.3f y=%.3f z=%.3f roll=%.3f pitch=%.3f yaw=%.3f",
             x, y, z, roll, pitch, yaw);
    pub_pose.publish(initial_pose);

    // ---- 空闲循环：保持节点存活 ----
    ros::Rate rate(10);
    while (ros::ok())
    {
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
