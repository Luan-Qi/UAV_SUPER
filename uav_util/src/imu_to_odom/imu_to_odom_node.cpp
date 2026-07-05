/**
 * @file imu_to_odom_node.cpp
 * @brief IMU 里程计节点 —— 将 IMU 数据积分转换为 Odometry 并发布。
 *
 * @details
 * 精简 ROS 节点包装器，负责：
 *   1. 创建 ImuOdom 实例并传入私有命名空间 nh
 *   2. 进入 ros::spin() 循环，所有处理在 ImuOdom::ImuCallback 中完成
 *
 * 订阅话题（可通过 launch/param 配置）:
 *   - imu_topic  (默认 /imu/data)
 *
 * 发布话题（可通过 launch/param 配置）:
 *   - odom_topic (默认 imu_odom)
 *   - path_topic (默认 imu_path)
 *
 * 坐标系（可配置）:
 *   - odom_frame_id  (默认 "odom")
 *   - base_frame_id  (默认 "base_link")
 *
 * 原始项目: https://github.com/Abin1258/imu_to_odom
 *
 * 用法：
 *   rosrun uav_util imu_to_odom _imu_topic:=/livox/imu _calib_samples:=200
 */

#include "imu_to_odom.hpp"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "imu_to_odom");

    ros::NodeHandle nh("~");  // 私有命名空间，使 param 对每个节点实例独立

    ImuOdom* imu_to_odom = new ImuOdom(nh);

    ros::spin();

    delete imu_to_odom;
    return 0;
}
