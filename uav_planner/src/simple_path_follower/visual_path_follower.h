/**
 * @file    visual_path_follower.h
 * @brief   路径跟随调试可视化工具 — viz_utils 命名空间
 *
 * 提供一系列向 MarkerArray 添加基本几何体的辅助函数:
 *   - addPoint():  球体标记点
 *   - addLine():   线段 (两点连线)
 *   - addPath():   折线 (多点连线)
 *   - addSphere(): 半透明球体 (用于调试到达区域等)
 *   - addText():   视角朝向文字标签
 *
 * 所有标记默认发布在 "map" 坐标系下
 */

#ifndef VISUALIZATION_UTILS_H
#define VISUALIZATION_UTILS_H

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/Point.h>
#include <Eigen/Dense>

namespace viz_utils
{

/**
 * @brief 添加一个球体标记点
 * @param arr      MarkerArray 引用
 * @param pt       点位置
 * @param id       唯一 marker ID
 * @param frame_id 坐标系 (默认 "map")
 * @param r,g,b    颜色 (默认红色)
 * @param scale    球体直径 (默认 0.1)
 */
void addPoint(
    visualization_msgs::MarkerArray &arr,
    const geometry_msgs::Point &pt,
    int id,
    const std::string &frame_id = "map",
    float r = 1.0, float g = 0.0, float b = 0.0,
    float scale = 0.1);

/**
 * @brief 添加一条线段 (p1 -> p2)
 * @param width 线宽 (默认 0.05)
 */
void addLine(
    visualization_msgs::MarkerArray &arr,
    const geometry_msgs::Point &p1,
    const geometry_msgs::Point &p2,
    int id,
    const std::string &frame_id = "map",
    float r = 0.0, float g = 1.0, float b = 0.0,
    float width = 0.05);

/**
 * @brief 添加一条折线路径 (多点连接)
 */
void addPath(
    visualization_msgs::MarkerArray &arr,
    const std::vector<geometry_msgs::Point> &points,
    int id,
    const std::string &frame_id = "map",
    float r = 0.0, float g = 0.5, float b = 1.0,
    float width = 0.05);

/**
 * @brief 添加一个半透明球体 (用于可视化到达区域)
 * @param center 球心 (Eigen::Vector3d)
 * @param radius 球半径
 * @note  透明度固定为 0.5
 */
void addSphere(
    visualization_msgs::MarkerArray &arr,
    const Eigen::Vector3d &center,
    float radius,
    int id,
    const std::string &frame_id = "map",
    float r = 1.0, float g = 0.5, float b = 0.0);

/**
 * @brief 添加一个视角朝向的文字标签 (TEXT_VIEW_FACING)
 * @param text  显示文字
 * @param scale 文字高度 (默认 0.5)
 */
void addText(
    visualization_msgs::MarkerArray &arr,
    const geometry_msgs::Point &pos,
    const std::string &text,
    int id,
    const std::string &frame_id = "map",
    float r = 1.0, float g = 1.0, float b = 1.0,
    float scale = 0.5);

} // namespace viz_utils

#endif  // VISUALIZATION_UTILS_H
