#include "astar_searcher.h"
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <octomap_msgs/Octomap.h>
#include <nav_msgs/Path.h>
#include <octomap/OcTree.h>
#include <octomap_msgs/conversions.h>
#include "astar_show_obs.h"
#include "uav_px4_ctrl/TakeoffNotify.h"


AstarPathFinder astar;

ros::Publisher path_pub;
ros::Publisher obs_marker_pub;
ros::Publisher obs_cloud_pub;

int inflater_size = 2;
bool have_start = false, have_goal = false;
bool have_octomap = false;
bool publish_obstacle_show = false;
geometry_msgs::PoseStamped start_pose, goal_pose;

// 读取Octomap，设置障碍物
void octomapCallback(const octomap_msgs::OctomapConstPtr &msg)
{
    if (have_octomap)
        return;
    
    ROS_INFO_ONCE("[astar] Received octomap, converting...");
    octomap::OcTree *octree = dynamic_cast<octomap::OcTree*>(octomap_msgs::fullMsgToMap(*msg));

    if (!octree)
    {
        ROS_ERROR("[astar] Octomap conversion failed!");
        return;
    }

    double res = octree->getResolution();

    // 获取地图边界
    double x_min, y_min, z_min, x_max, y_max, z_max;
    octree->getMetricMin(x_min, y_min, z_min);
    octree->getMetricMax(x_max, y_max, z_max);

    astar.initGridMap(res, Eigen::Vector3d(x_min, y_min, z_min),
                           Eigen::Vector3d(x_max, y_max, z_max),
                           (x_max - x_min) / res,
                           (y_max - y_min) / res,
                           (z_max - z_min) / res);

    // // 将Octomap中的占用节点填入栅格
    // for (octomap::OcTree::iterator it = octree->begin(); it != octree->end(); ++it)
    // {
    //     if (octree->isNodeOccupied(*it))
    //     {
    //         auto coord = it.getCoordinate();
    //         astar.setObs(coord.x(), coord.y(), coord.z());
    //     }
    // }

    for (octomap::OcTree::leaf_iterator it = octree->begin_leafs(); it != octree->end_leafs(); ++it)
    {
        if (octree->isNodeOccupied(*it))
        {
            double size = it.getSize(); // 当前节点边长
            auto coord = it.getCoordinate();

            int steps = std::ceil(size / res);
            int inflate = inflater_size;  // 膨胀1格

            // 遍历当前节点对应的所有细分格点，并在此基础上膨胀1格
            for (int dx = -steps / 2 - inflate; dx <= steps / 2 + inflate; ++dx)
                for (int dy = -steps / 2 - inflate; dy <= steps / 2 + inflate; ++dy)
                    for (int dz = -steps / 2 - inflate; dz <= steps / 2 + inflate; ++dz)
                    {
                        double x = coord.x() + dx * res;
                        double y = coord.y() + dy * res;
                        double z = coord.z() + dz * res;
                        astar.setObs(x, y, z);
                    }
        }
    }

    // 可视化3D代价地图 Voxels OR Cloud
    if(publish_obstacle_show)
        publishObstacleCloud(&obs_cloud_pub, res, 
                              x_min, y_min, z_min, 
                              (x_max - x_min) / res,
                              (y_max - y_min) / res,
                              (z_max - z_min) / res, 
                              [](int x, int y, int z){ return astar.getObs(x, y, z); });

    ROS_INFO("[astar] Octomap loaded to grid map.");
    delete octree;
    have_octomap = true;
}

void startCallback(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    if(!have_start)
    {
        start_pose = *msg;
        have_start = true;
        ROS_INFO("[astar] Received start pose [%.1f,%.1f,%.1f]", 
                 start_pose.pose.position.x, start_pose.pose.position.y, start_pose.pose.position.z);
    }
}

void goalCallback(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    if(!have_goal)
    {
        goal_pose = *msg;
        have_goal = true;
        ROS_INFO("[astar] Received goal pose [%.1f,%.1f,%.1f]", 
                 goal_pose.pose.position.x, goal_pose.pose.position.y, goal_pose.pose.position.z);
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "astar_planner_node");
    ros::NodeHandle nh("~");

    std::string octomap_topic;
    std::string start_pose_topic;
    std::string goal_pose_topic;
    std::string planned_path_topic;

    nh.param<std::string>("octomap_topic", octomap_topic, "/octomap_full");
    nh.param<std::string>("start_pose_topic", start_pose_topic, "/start_pose");
    nh.param<std::string>("goal_pose_topic", goal_pose_topic, "/goal_pose");
    nh.param<std::string>("global_path_topic", planned_path_topic, "/planned_path");
    nh.param<bool>("enable_obstacle_show", publish_obstacle_show, false);
    nh.param<int>("inflater_size", inflater_size, 2);

    ros::Subscriber octo_sub = nh.subscribe(octomap_topic, 10, octomapCallback);
    ros::Subscriber start_sub = nh.subscribe(start_pose_topic, 10, startCallback);
    ros::Subscriber goal_sub = nh.subscribe(goal_pose_topic, 10, goalCallback);

    ros::ServiceClient planner_failed_client = nh.serviceClient<uav_px4_ctrl::TakeoffNotify>("/planner_fail_notify");

    path_pub = nh.advertise<nav_msgs::Path>(planned_path_topic, 10, true);

    if (publish_obstacle_show)
        obs_marker_pub = nh.advertise<visualization_msgs::MarkerArray>("obstacle_markers", 10, true);
        obs_cloud_pub  = nh.advertise<sensor_msgs::PointCloud2>("obstacle_cloud", 10, true);

    ROS_INFO("[astar] Waiting for %s%s%s", have_start ? "" : "start_pose,", have_goal ? "" : "goal_pose,", have_octomap ? "" : "octomap");

    ros::Rate rate(10.0);
    while (ros::ok())
    {
        ros::spinOnce();
        if (have_start && have_goal && have_octomap)
        {
            ROS_INFO("[astar] start pose [%.1f,%.1f,%.1f], goal pose [%.1f,%.1f,%.1f]", 
                 start_pose.pose.position.x, start_pose.pose.position.y, start_pose.pose.position.z, 
                 goal_pose.pose.position.x, goal_pose.pose.position.y, goal_pose.pose.position.z);
            ROS_INFO("[astar] Start planning...");
            Eigen::Vector3d start(start_pose.pose.position.x,
                                  start_pose.pose.position.y,
                                  start_pose.pose.position.z);
            Eigen::Vector3d goal(goal_pose.pose.position.x,
                                 goal_pose.pose.position.y,
                                 goal_pose.pose.position.z);

            astar.AstarGraphSearch(start, goal);
            std::vector<Eigen::Vector3d> path_pts = astar.getPath();

            if (path_pts.size() <= 1)
            {
                ROS_ERROR("[astar] A* failed to find a path!");
                have_start = false;

                uav_px4_ctrl::TakeoffNotify srv;
                srv.request.takeoff_done = true;
                if(!planner_failed_client.call(srv))
                    have_octomap = false;
            }
            else
            {
                nav_msgs::Path path_msg;
                path_msg.header.stamp = ros::Time::now();
                path_msg.header.frame_id = "map";

                for (auto &pt : path_pts)
                {
                    geometry_msgs::PoseStamped p;
                    p.pose.position.x = pt(0);
                    p.pose.position.y = pt(1);
                    p.pose.position.z = pt(2);
                    p.pose.orientation.w = 1.0;
                    path_msg.poses.push_back(p);
                }

                path_pub.publish(path_msg);
                ROS_INFO("[astar] Published path with %zu points.", path_msg.poses.size());
                start_pose = goal_pose;
            }

            have_goal = false;  // 等待下次规划
            astar.resetUsedGrids();
            ROS_INFO(" ");
            ROS_INFO("[astar] Waiting for next %s%s%s", have_start ? "" : "start_pose,", have_goal ? "" : "goal_pose,", have_octomap ? "" : "octomap");
        }

        rate.sleep();
    }

    return 0;
}
