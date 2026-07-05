/**
 * @file    astar_show_obs.h
 * @brief   障碍物可视化函数声明
 *
 * 提供两种可视化方式:
 *   - publishObstacleCloud():  PointCloud2 — 适合大量障碍物
 *   - publishObstacleVoxels(): MarkerArray — 每个体素一个 Cube, 适合少量障碍物
 *
 * 两个函数均通过函数指针 getObsFunc 获取占据状态, 与 AstarPathFinder 解耦
 */

#ifndef ASTAR_SHOW_OBS_H
#define ASTAR_SHOW_OBS_H

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/MarkerArray.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>

/**
 * @brief 发布障碍物为 PointCloud2
 * @param obs_cloud_pub 发布器指针
 * @param resolution    栅格分辨率 (m)
 * @param gl_xl/yl/zl   地图下界
 * @param GLX/Y/Z_SIZE  栅格数
 * @param getObsFunc    回调: bool(int x,int y,int z) → 是否障碍物
 */
void publishObstacleCloud(ros::Publisher *obs_cloud_pub, double resolution,
                          double gl_xl, double gl_yl, double gl_zl,
                          int GLX_SIZE, int GLY_SIZE, int GLZ_SIZE,
                          bool (*getObsFunc)(int, int, int));

/**
 * @brief 发布障碍物为 MarkerArray (每个体素一个 Cube)
 * @note  大量障碍物时性能较差, 建议使用 publishObstacleCloud()
 */
void publishObstacleVoxels(ros::Publisher *obs_marker_pub, double resolution,
                           double gl_xl, double gl_yl, double gl_zl,
                           int GLX_SIZE, int GLY_SIZE, int GLZ_SIZE,
                           bool (*getObsFunc)(int, int, int));

#endif  // ASTAR_SHOW_OBS_H
