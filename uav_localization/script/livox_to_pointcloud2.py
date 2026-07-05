#!/usr/bin/env python3

## @file livox_to_pointcloud2.py
## @brief ROS1 节点：将 Livox LiDAR 的 CustomMsg 转换为标准 sensor_msgs/PointCloud2。
##
## 该节点订阅 Livox ROS 驱动 v2 发布的 `/livox/lidar` 话题（消息类型
## livox_ros_driver2::CustomMsg），提取每个点的三维坐标及反射率，
## 封装为标准的 sensor_msgs::PointCloud2 消息后重新发布到
## `/livox/lidar_cloud` 话题，供 Cartographer 等下游 SLAM 模块使用。
##
## 点云字段：
## - x (float32)       : 点的 X 坐标 [m]
## - y (float32)       : 点的 Y 坐标 [m]
## - z (float32)       : 点的 Z 坐标 [m]
## - intensity (float32): 点的反射率 (0~255 → float)
##
## 坐标系：所有点云发布的 frame_id 为 "lidar_link"。
##
## @author 鸾棋
## @date   2025

import rospy
from livox_ros_driver2.msg import CustomMsg
from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs.point_cloud2 as pc2


## @brief Livox CustomMsg 回调函数：将 CustomMsg 转为 PointCloud2 并发布。
##
## 遍历 Livox CustomMsg 中的每个点，提取 (x, y, z, reflectivity) 四元组，
## 构造 sensor_msgs::PointCloud2 消息，设置 frame_id 为 "lidar_link"，
## 保持原始时间戳，并通过全局发布者 pub 发布到 `/livox/lidar_cloud`。
##
## @param msg  [in] livox_ros_driver2::CustomMsg 类型，包含 Livox LiDAR 原始点云数据。
##               - msg.points  : 点列表，每个点包含 x, y, z, reflectivity 字段。
##               - msg.header  : 原始消息头（frame_id 和时间戳）。
def callback(msg):
    ## @brief 点列表：每个元素为 [x, y, z, reflectivity]。
    points = []
    for p in msg.points:
        points.append([p.x, p.y, p.z, float(p.reflectivity)])

    ## @brief PointCloud2 字段定义。
    ## 字段顺序与 points 列表中每个元素的数据布局一致：
    ## offset 0: x, offset 4: y, offset 8: z, offset 12: intensity。
    fields = [
        PointField('x', 0, PointField.FLOAT32, 1),
        PointField('y', 4, PointField.FLOAT32, 1),
        PointField('z', 8, PointField.FLOAT32, 1),
        PointField('intensity', 12, PointField.FLOAT32, 1),
    ]

    ## @brief 使用 sensor_msgs.point_cloud2.create_cloud 构建点云消息。
    cloud_msg = pc2.create_cloud(msg.header, fields, points)
    cloud_msg.header.frame_id = "lidar_link"
    cloud_msg.header.stamp = msg.header.stamp
    pub.publish(cloud_msg)


## @brief 主函数：初始化 ROS 节点并进入事件循环。
##
## 节点名称：livox_to_pointcloud2
##
## 发布话题：
## - /livox/lidar_cloud (sensor_msgs/PointCloud2, queue_size=10)
##
## 订阅话题：
## - /livox/lidar (livox_ros_driver2/CustomMsg, queue_size=10)
##
## 该节点将持续运行直到收到 SIGINT 或调用 rospy.signal_shutdown()。
if __name__ == "__main__":
    rospy.init_node("livox_to_pointcloud2")

    ## @brief 全局发布者：将转换后的 PointCloud2 发布到 /livox/lidar_cloud。
    pub = rospy.Publisher("/livox/lidar_cloud", PointCloud2, queue_size=10)

    ## @brief 订阅 Livox LiDAR 原始数据，回调函数为 callback。
    rospy.Subscriber("/livox/lidar", CustomMsg, callback, queue_size=10)

    rospy.spin()
