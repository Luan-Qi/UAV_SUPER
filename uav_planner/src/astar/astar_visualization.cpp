/**
 * @file    astar_visualization.cpp
 * @brief   障碍物可视化 — 将 3D 栅格占据地图发布为 PointCloud2 或 MarkerArray
 *
 * 提供两个可视化函数:
 *   - publishObstacleCloud():  发布为 sensor_msgs::PointCloud2 (高效, 适合大量障碍物)
 *   - publishObstacleVoxels(): 发布为 visualization_msgs::MarkerArray (每个体素一个 Cube)
 *
 * 使用方式:
 *   通过函数指针 getObsFunc 回调获取栅格占据状态, 实现与 AstarPathFinder 的解耦
 */

#include "astar_show_obs.h"

/**
 * @brief 发布障碍物为 PointCloud2 (推荐用于大量障碍物的可视化)
 *
 * @param obs_cloud_pub PointCloud2 发布器指针
 * @param resolution    栅格分辨率 (m)
 * @param gl_xl/yl/zl   地图下界 (世界坐标)
 * @param GLX/Y/Z_SIZE  各方向栅格数
 * @param getObsFunc    回调函数: bool(int x, int y, int z) → 该栅格是否为障碍物
 *
 * 遍历整个栅格地图, 将所有占据格点的中心坐标加入点云, 发布到 /map 坐标系
 */
void publishObstacleCloud(ros::Publisher *obs_cloud_pub, double resolution,
                          double gl_xl, double gl_yl, double gl_zl,
                          int GLX_SIZE, int GLY_SIZE, int GLZ_SIZE,
                          bool (*getObsFunc)(int, int, int))
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());

    // 遍历所有栅格, 收集占据格点
    for (int x = 0; x < GLX_SIZE; ++x)
        for (int y = 0; y < GLY_SIZE; ++y)
            for (int z = 0; z < GLZ_SIZE; ++z)
            {
                if (getObsFunc(x, y, z) == 1)  // 1 = 障碍物
                {
                    pcl::PointXYZ pt;
                    pt.x = gl_xl + (x + 0.5) * resolution;
                    pt.y = gl_yl + (y + 0.5) * resolution;
                    pt.z = gl_zl + (z + 0.5) * resolution;
                    cloud->points.push_back(pt);
                }
            }

    cloud->width  = cloud->points.size();
    cloud->height = 1;

    // PCL → ROS 消息转换
    sensor_msgs::PointCloud2 msg;
    pcl::toROSMsg(*cloud, msg);
    msg.header.frame_id = "map";
    msg.header.stamp    = ros::Time::now();

    obs_cloud_pub->publish(msg);
}

/**
 * @brief 发布障碍物为 MarkerArray (每个体素一个半透明红色 Cube)
 *
 * @note 当障碍物数量很大 (>10000) 时, MarkerArray 性能较差, 建议使用 publishObstacleCloud()
 *
 * @param obs_marker_pub MarkerArray 发布器指针
 * @param resolution     栅格分辨率 (m), 同时决定 Cube 的边长
 * @param gl_xl/yl/zl    地图下界 (世界坐标)
 * @param GLX/Y/Z_SIZE   各方向栅格数
 * @param getObsFunc     回调函数: bool(int x, int y, int z) → 该栅格是否为障碍物
 */
void publishObstacleVoxels(ros::Publisher *obs_marker_pub, double resolution,
                           double gl_xl, double gl_yl, double gl_zl,
                           int GLX_SIZE, int GLY_SIZE, int GLZ_SIZE,
                           bool (*getObsFunc)(int, int, int))
{
    visualization_msgs::MarkerArray marker_array;
    visualization_msgs::Marker cube;

    // 所有 Cube 共享的公共属性
    cube.header.frame_id = "map";
    cube.header.stamp    = ros::Time::now();
    cube.ns              = "obstacles";
    cube.type            = visualization_msgs::Marker::CUBE;
    cube.action          = visualization_msgs::Marker::ADD;
    cube.scale.x = cube.scale.y = cube.scale.z = resolution;

    // 半透明红色
    cube.color.r = 1.0;
    cube.color.g = 0.0;
    cube.color.b = 0.0;
    cube.color.a = 0.8;

    int id = 0;
    for (int x = 0; x < GLX_SIZE; ++x)
        for (int y = 0; y < GLY_SIZE; ++y)
            for (int z = 0; z < GLZ_SIZE; ++z)
            {
                if (getObsFunc(x, y, z) == 1)
                {
                    cube.id = id++;
                    cube.pose.position.x = gl_xl + (x + 0.5) * resolution;
                    cube.pose.position.y = gl_yl + (y + 0.5) * resolution;
                    cube.pose.position.z = gl_zl + (z + 0.5) * resolution;
                    marker_array.markers.push_back(cube);
                }
            }

    obs_marker_pub->publish(marker_array);
}
