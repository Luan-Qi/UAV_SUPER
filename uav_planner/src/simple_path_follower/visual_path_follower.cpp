#include "visual_path_follower.h"

namespace viz_utils
{

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
    m.ns = "points";
    m.id = id;
    m.type = visualization_msgs::Marker::SPHERE;
    m.action = visualization_msgs::Marker::ADD;

    m.pose.orientation.w = 1.0;
    m.pose.position = pt;

    m.scale.x = scale;
    m.scale.y = scale;
    m.scale.z = scale;

    m.color.r = r;
    m.color.g = g;
    m.color.b = b;
    m.color.a = 1.0;

    arr.markers.push_back(m);
}

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
    m.ns = "lines";
    m.id = id;
    m.type = visualization_msgs::Marker::LINE_STRIP;
    m.action = visualization_msgs::Marker::ADD;

    m.scale.x = width;

    m.points.push_back(p1);
    m.points.push_back(p2);

    m.color.r = r;
    m.color.g = g;
    m.color.b = b;
    m.color.a = 1.0;

    arr.markers.push_back(m);
}

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
    m.ns = "paths";
    m.id = id;
    m.type = visualization_msgs::Marker::LINE_STRIP;
    m.action = visualization_msgs::Marker::ADD;

    m.scale.x = width;

    m.points = points;

    m.color.r = r;
    m.color.g = g;
    m.color.b = b;
    m.color.a = 1.0;

    arr.markers.push_back(m);
}

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
    m.ns = "spheres";
    m.id = id;
    m.type = visualization_msgs::Marker::SPHERE;
    m.action = visualization_msgs::Marker::ADD;

    m.pose.orientation.w = 1.0;
    m.pose.position.x = center(0);
    m.pose.position.y = center(1);
    m.pose.position.z = center(2);

    m.scale.x = radius * 2;
    m.scale.y = radius * 2;
    m.scale.z = radius * 2;

    m.color.r = r;
    m.color.g = g;
    m.color.b = b;
    m.color.a = 0.5;

    arr.markers.push_back(m);
}

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
    m.ns = "texts";
    m.id = id;
    m.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    m.action = visualization_msgs::Marker::ADD;

    m.pose.position = pos;
    m.pose.orientation.w = 1.0;

    m.scale.z = scale;

    m.color.r = r;
    m.color.g = g;
    m.color.b = b;
    m.color.a = 1.0;

    m.text = text;

    arr.markers.push_back(m);
}

} // namespace viz_utils
