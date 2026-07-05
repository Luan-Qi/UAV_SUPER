/**
 * @file    visual_path_follower.cpp
 * @brief   viz_utils 命名空间的实现 — RViz MarkerArray 可视化辅助函数
 *
 * 每个函数构造一个 visualization_msgs::Marker 并追加到 MarkerArray 中。
 * 使用 MarkerArray 而非独立 Marker 的好处: 单次发布即可渲染所有几何体。
 *
 * Marker 类型:
 *   - SPHERE:            实心球体 (addPoint, addSphere)
 *   - LINE_STRIP:        折线 (addLine, addPath)
 *   - TEXT_VIEW_FACING:  始终面向相机的文字 (addText)
 */

#include "visual_path_follower.h"

namespace viz_utils
{

/**
 * @brief 添加实心球体标记点
 */
void addPoint(
    visualization_msgs::MarkerArray &arr,
    const geometry_msgs::Point &pt,
    int id,
    const std::string &frame_id,
    float r, float g, float b,
    float scale)
{
    visualization_msgs::Marker m;
    m.header.frame_id = frame_id;
    m.ns              = "points";
    m.id              = id;
    m.type            = visualization_msgs::Marker::SPHERE;
    m.action          = visualization_msgs::Marker::ADD;

    m.pose.orientation.w = 1.0;
    m.pose.position      = pt;

    m.scale.x = scale;
    m.scale.y = scale;
    m.scale.z = scale;

    m.color.r = r;
    m.color.g = g;
    m.color.b = b;
    m.color.a = 1.0;  // 不透明

    arr.markers.push_back(m);
}

/**
 * @brief 添加线段 (LINE_STRIP with 2 points)
 */
void addLine(
    visualization_msgs::MarkerArray &arr,
    const geometry_msgs::Point &p1,
    const geometry_msgs::Point &p2,
    int id,
    const std::string &frame_id,
    float r, float g, float b,
    float width)
{
    visualization_msgs::Marker m;
    m.header.frame_id = frame_id;
    m.ns              = "lines";
    m.id              = id;
    m.type            = visualization_msgs::Marker::LINE_STRIP;
    m.action          = visualization_msgs::Marker::ADD;

    m.scale.x = width;  // LINE_STRIP 的线宽由 scale.x 控制

    m.points.push_back(p1);
    m.points.push_back(p2);

    m.color.r = r;
    m.color.g = g;
    m.color.b = b;
    m.color.a = 1.0;

    arr.markers.push_back(m);
}

/**
 * @brief 添加折线路径 (LINE_STRIP with N points)
 */
void addPath(
    visualization_msgs::MarkerArray &arr,
    const std::vector<geometry_msgs::Point> &points,
    int id,
    const std::string &frame_id,
    float r, float g, float b,
    float width)
{
    visualization_msgs::Marker m;
    m.header.frame_id = frame_id;
    m.ns              = "paths";
    m.id              = id;
    m.type            = visualization_msgs::Marker::LINE_STRIP;
    m.action          = visualization_msgs::Marker::ADD;

    m.scale.x = width;
    m.points  = points;

    m.color.r = r;
    m.color.g = g;
    m.color.b = b;
    m.color.a = 1.0;

    arr.markers.push_back(m);
}

/**
 * @brief 添加半透明球体 — 用于可视化到达判定区域等
 *
 * @note 球体直径 = radius * 2 (RViz 中 SPHERE 的 scale 是直径而非半径)
 */
void addSphere(
    visualization_msgs::MarkerArray &arr,
    const Eigen::Vector3d &center,
    float radius,
    int id,
    const std::string &frame_id,
    float r, float g, float b)
{
    visualization_msgs::Marker m;
    m.header.frame_id = frame_id;
    m.ns              = "spheres";
    m.id              = id;
    m.type            = visualization_msgs::Marker::SPHERE;
    m.action          = visualization_msgs::Marker::ADD;

    m.pose.orientation.w = 1.0;
    m.pose.position.x    = center(0);
    m.pose.position.y    = center(1);
    m.pose.position.z    = center(2);

    // RViz SPHERE: scale 是直径
    m.scale.x = radius * 2;
    m.scale.y = radius * 2;
    m.scale.z = radius * 2;

    m.color.r = r;
    m.color.g = g;
    m.color.b = b;
    m.color.a = 0.5;  // 半透明

    arr.markers.push_back(m);
}

/**
 * @brief 添加视角朝向文字标签 — 始终面向相机
 */
void addText(
    visualization_msgs::MarkerArray &arr,
    const geometry_msgs::Point &pos,
    const std::string &text,
    int id,
    const std::string &frame_id,
    float r, float g, float b,
    float scale)
{
    visualization_msgs::Marker m;
    m.header.frame_id = frame_id;
    m.ns              = "texts";
    m.id              = id;
    m.type            = visualization_msgs::Marker::TEXT_VIEW_FACING;
    m.action          = visualization_msgs::Marker::ADD;

    m.pose.position      = pos;
    m.pose.orientation.w = 1.0;

    m.scale.z = scale;  // TEXT_VIEW_FACING 的文字高度由 scale.z 控制

    m.color.r = r;
    m.color.g = g;
    m.color.b = b;
    m.color.a = 1.0;

    m.text = text;

    arr.markers.push_back(m);
}

} // namespace viz_utils
