#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <visualization_msgs/MarkerArray.h>
#include <Eigen/Dense>
#include <cmath>
#include "visual_path_follower.h"

nav_msgs::Path local_path;
nav_msgs::Odometry cur_map_to_odom;
geometry_msgs::PoseStamped current_goal;
geometry_msgs::PoseStamped current_pose;
Eigen::Matrix4d T_odom_to_map;

bool has_path = false;
bool has_pose = false;
bool has_map_to_odom = false;
bool need_finished = false;
bool has_finished = false;
int current_index = 0;
bool first_start = false;

double goal_distance_threshold = 0.5; // 到达目标的判定距离
int path_step = 5; // 每隔几个点发送一次目标
double publish_rate = 5.0; // Hz
std::string path_topic, odom_topic, goal_topic, map_to_odom_topic;

visualization_msgs::MarkerArray marker_array;
int id = 0;

// -------------------- Pose -> Eigen::Matrix4d --------------------
Eigen::Matrix4d poseToMatrix(const geometry_msgs::Pose &pose)
{
    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
    Eigen::Quaterniond q(pose.orientation.w, pose.orientation.x,
                         pose.orientation.y, pose.orientation.z);
    Eigen::Matrix3d R = q.toRotationMatrix();
    T.block<3, 3>(0, 0) = R;
    T(0, 3) = pose.position.x;
    T(1, 3) = pose.position.y;
    T(2, 3) = pose.position.z;
    return T;
}

// -------------------- Matrix4d -> Pose --------------------
geometry_msgs::Pose matrixToPose(const Eigen::Matrix4d &T)
{
    geometry_msgs::Pose pose;
    Eigen::Matrix3d R = T.block<3, 3>(0, 0);
    Eigen::Quaterniond q(R);
    pose.position.x = T(0, 3);
    pose.position.y = T(1, 3);
    pose.position.z = T(2, 3);
    pose.orientation.x = q.x();
    pose.orientation.y = q.y();
    pose.orientation.z = q.z();
    pose.orientation.w = q.w();
    return pose;
}

void cbPath(const nav_msgs::Path::ConstPtr& msg)
{
    if (msg->poses.empty()) {
        ROS_WARN("Received empty path!");
        return;
    }
    local_path = *msg;
    has_path = true;
    has_finished = false;
    current_index = path_step;
    ROS_INFO("[path_follower_3d] Received new path with %lu points.", msg->poses.size());
}

void cbOdom(const nav_msgs::Odometry::ConstPtr& msg)
{
    current_pose.pose = msg->pose.pose;
    current_pose.header = msg->header;
    has_pose = true;
}

void cbMapToOdom(const nav_msgs::Odometry::ConstPtr &msg)
{
    cur_map_to_odom = *msg;
    T_odom_to_map = poseToMatrix(cur_map_to_odom.pose.pose).inverse();
    has_map_to_odom = true;
}

// 工具函数：3D 距离
double distance3D(const geometry_msgs::PoseStamped& a, const geometry_msgs::PoseStamped& b)
{
    double dx = a.pose.position.x - b.pose.position.x;
    double dy = a.pose.position.y - b.pose.position.y;
    double dz = a.pose.position.z - b.pose.position.z;
    return std::sqrt(dx * dx + dy * dy + dz * dz);
}



int main(int argc, char** argv)
{
    ros::init(argc, argv, "path_follower");
    ros::NodeHandle nh("~");

    // 参数读取
    nh.param("goal_distance_threshold", goal_distance_threshold, 0.5);
    nh.param("path_step", path_step, 1);
    nh.param("publish_rate", publish_rate, 5.0);
    nh.param<std::string>("path_topic", path_topic, "/global_path");
    nh.param<std::string>("odom_topic", odom_topic, "/localization");
    nh.param<std::string>("goal_topic", goal_topic, "/move_base_simple/goal");
    nh.param<std::string>("map_to_odom_topic", map_to_odom_topic, "/map_to_odom");

    ros::Subscriber sub_path = nh.subscribe(path_topic, 1, cbPath);
    ros::Subscriber sub_odom = nh.subscribe(odom_topic, 1, cbOdom);
    ros::Subscriber sub_map_to_odom = nh.subscribe(map_to_odom_topic, 3, cbMapToOdom);
    ros::Publisher pub_goal = nh.advertise<geometry_msgs::PoseStamped>(goal_topic, 1);

    ros::Publisher pub_marker_array = nh.advertise<visualization_msgs::MarkerArray>("/path_follower_debug_array", 10);

    ros::Rate rate(publish_rate);

    ROS_INFO("[path_follower_3d] Node started. Waiting for /path, /odom, /T...");
    current_index = path_step;

    while (ros::ok())
    {
        ros::spinOnce();

        if (has_path && has_pose && has_map_to_odom)
        {
            if (!has_finished)
            {
                if (distance3D(current_pose, current_goal) < goal_distance_threshold || !first_start)
                {
                    first_start = true;

                    current_goal = local_path.poses[current_index];

                    Eigen::Vector3d center(
                        current_goal.pose.position.x,
                        current_goal.pose.position.y,
                        current_goal.pose.position.z);
                    viz_utils::addSphere(marker_array, center, goal_distance_threshold, id++);

                    Eigen::Matrix4d T_global = poseToMatrix(current_goal.pose);
                    Eigen::Matrix4d T_local = T_odom_to_map * T_global;
                    geometry_msgs::PoseStamped current_goal_local;
                    current_goal_local.header = current_goal.header;
                    current_goal_local.pose = matrixToPose(T_local);
                    
                    pub_goal.publish(current_goal_local);

                    ROS_INFO("[path_follower_3d] New goal #%d: (x=%.1f, y=%.1f, z=%.1f)",
                             current_index,
                             current_goal_local.pose.position.x,
                             current_goal_local.pose.position.y,
                             current_goal_local.pose.position.z);

                    if (need_finished){has_finished = true;need_finished=false;continue;}
                    
                    current_index += path_step;
                    if (current_index >= (int)local_path.poses.size())
                    {
                        current_index = local_path.poses.size() - 1;
                        need_finished = true;
                    }
                }
                pub_marker_array.publish(marker_array);
            }
            else
            {
                ROS_INFO("[path_follower_3d] Path finished.");
                has_path = false;
                current_index = path_step;
                marker_array.markers.clear();
                id = 0;
                first_start = false;
            }
        }
        // else
        // {
        //     ROS_INFO_THROTTLE(5.0, "[path_follower_3d] Waiting for %s%s%s", has_path ? "" : "path,", has_pose ? "" : "pose,", has_map_to_odom ? "" : "T");
        // }

        rate.sleep();
    }

    return 0;
}
