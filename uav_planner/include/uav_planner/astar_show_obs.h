#ifndef ASTAR_SHOW_OBS_H
#define ASTAR_SHOW_OBS_H

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/MarkerArray.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>


void publishObstacleCloud(ros::Publisher * obs_cloud_pub, double resolution, 
                           double gl_xl, double gl_yl, double gl_zl, 
                           int GLX_SIZE, int GLY_SIZE, int GLZ_SIZE, 
                           bool (*getObsFunc)(int, int, int));

void publishObstacleVoxels(ros::Publisher * obs_marker_pub, double resolution, 
                           double gl_xl, double gl_yl, double gl_zl, 
                           int GLX_SIZE, int GLY_SIZE, int GLZ_SIZE, 
                           bool (*getObsFunc)(int, int, int));



#endif