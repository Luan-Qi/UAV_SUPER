#ifndef VISUALIZATION_UTILS_H
#define VISUALIZATION_UTILS_H

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/Point.h>
#include <Eigen/Dense>

namespace viz_utils
{

// 1. 添加一个点
void addPoint(
    visualization_msgs::MarkerArray &arr,
    const geometry_msgs::Point &pt,
    int id,
    const std::string &frame_id = "map",
    float r = 1.0, float g = 0.0, float b = 0.0,
    float scale = 0.1);

// 2. 添加一个线段（p1 → p2）
void addLine(
    visualization_msgs::MarkerArray &arr,
    const geometry_msgs::Point &p1,
    const geometry_msgs::Point &p2,
    int id,
    const std::string &frame_id = "map",
    float r = 0.0, float g = 1.0, float b = 0.0,
    float width = 0.05);

// 3. 添加一个路径（多个点）
void addPath(
    visualization_msgs::MarkerArray &arr,
    const std::vector<geometry_msgs::Point> &points,
    int id,
    const std::string &frame_id = "map",
    float r = 0.0, float g = 0.5, float b = 1.0,
    float width = 0.05);

// 4. 添加一个球体
void addSphere(
    visualization_msgs::MarkerArray &arr,
    const Eigen::Vector3d &center,
    float radius,
    int id,
    const std::string &frame_id = "map",
    float r = 1.0, float g = 0.5, float b = 0.0);

// 5. 添加一段文字
void addText(
    visualization_msgs::MarkerArray &arr,
    const geometry_msgs::Point &pos,
    const std::string &text,
    int id,
    const std::string &frame_id = "map",
    float r = 1.0, float g = 1.0, float b = 1.0,
    float scale = 0.5);

} // namespace viz_utils

#endif
