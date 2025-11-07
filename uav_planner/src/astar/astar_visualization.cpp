#include "astar_show_obs.h"

void publishObstacleCloud(ros::Publisher * obs_cloud_pub, double resolution, 
                           double gl_xl, double gl_yl, double gl_zl, 
                           int GLX_SIZE, int GLY_SIZE, int GLZ_SIZE, 
                           bool (*getObsFunc)(int, int, int))
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());

    for (int x = 0; x < GLX_SIZE; ++x)
        for (int y = 0; y < GLY_SIZE; ++y)
            for (int z = 0; z < GLZ_SIZE; ++z)
            {
                if (getObsFunc(x, y, z) == 1)
                {
                    pcl::PointXYZ pt;
                    pt.x = gl_xl + (x + 0.5) * resolution;
                    pt.y = gl_yl + (y + 0.5) * resolution;
                    pt.z = gl_zl + (z + 0.5) * resolution;
                    cloud->points.push_back(pt);
                }
            }

    cloud->width = cloud->points.size();
    cloud->height = 1;

    sensor_msgs::PointCloud2 msg;
    pcl::toROSMsg(*cloud, msg);
    msg.header.frame_id = "map";
    msg.header.stamp = ros::Time::now();

    obs_cloud_pub->publish(msg);
}

void publishObstacleVoxels(ros::Publisher * obs_marker_pub, double resolution, 
                           double gl_xl, double gl_yl, double gl_zl, 
                           int GLX_SIZE, int GLY_SIZE, int GLZ_SIZE, 
                           bool (*getObsFunc)(int, int, int))
{
    visualization_msgs::MarkerArray marker_array;
    visualization_msgs::Marker cube;

    cube.header.frame_id = "map";
    cube.header.stamp = ros::Time::now();
    cube.ns = "obstacles";
    cube.type = visualization_msgs::Marker::CUBE;
    cube.action = visualization_msgs::Marker::ADD;
    cube.scale.x = cube.scale.y = cube.scale.z = resolution; // 每个立方体的尺寸
    cube.color.r = 1.0;
    cube.color.g = 0.0;
    cube.color.b = 0.0;
    cube.color.a = 0.8; // 透明度

    int id = 0;
    for (int x = 0; x < GLX_SIZE; ++x)
        for (int y = 0; y < GLY_SIZE; ++y)
            for (int z = 0; z < GLZ_SIZE; ++z)
            {
                if (getObsFunc(x, y, z) == 1) // 表示障碍
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