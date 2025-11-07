#include <ros/ros.h>
#include <mutex>
#include <Eigen/Dense>

#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>

ros::Publisher pub_local_path;
nav_msgs::Odometry cur_map_to_odom;
bool has_map_to_odom = false;
std::mutex mtx;

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

// -------------------- /map_to_odom 回调 --------------------
void cbMapToOdom(const nav_msgs::Odometry::ConstPtr &msg)
{
    if(!has_map_to_odom)
    {
        std::lock_guard<std::mutex> lock(mtx);
        cur_map_to_odom = *msg;
        has_map_to_odom = true;
    }
}

// -------------------- /global_path 回调 --------------------
void cbGlobalPath(const nav_msgs::Path::ConstPtr &msg)
{
    if (!has_map_to_odom)
    {
        ROS_WARN("[path_trans] Waiting for /map_to_odom transform...");
        ros::Rate rate(10.0);
        while(ros::ok())
        {
            if(has_map_to_odom) break;
            ros::spinOnce();
            rate.sleep();
        }
        return;
    }

    nav_msgs::Path local_path;
    local_path.header = msg->header;
    local_path.header.frame_id = "map";  // 转换后的坐标系

    Eigen::Matrix4d T_map_to_odom;
    {
        std::lock_guard<std::mutex> lock(mtx);
        T_map_to_odom = poseToMatrix(cur_map_to_odom.pose.pose);
    }

    Eigen::Matrix4d T_odom_to_map = T_map_to_odom.inverse();

    // 遍历路径中的每个点，进行坐标变换
    for (const auto &pose_stamped : msg->poses)
    {
        Eigen::Matrix4d T_global = poseToMatrix(pose_stamped.pose);
        Eigen::Matrix4d T_local = T_odom_to_map * T_global;

        geometry_msgs::PoseStamped local_pose;
        local_pose.header = pose_stamped.header;
        local_pose.header.frame_id = "odom";
        local_pose.pose = matrixToPose(T_local);

        local_path.poses.push_back(local_pose);
    }

    pub_local_path.publish(local_path);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "path_transform_node");
    ros::NodeHandle nh("~");

    ROS_INFO("[path_trans] Path Transform Node Started...");

    std::string global_path_topic, local_path_topic, map_to_odom_topic;
    nh.param<std::string>("global_path_topic", global_path_topic, "/global_path");
    nh.param<std::string>("local_path_topic", local_path_topic, "/local_path");
    nh.param<std::string>("map_to_odom_topic", map_to_odom_topic, "/map_to_odom");

    ros::Subscriber sub_map_to_odom = nh.subscribe(map_to_odom_topic, 3, cbMapToOdom);
    ros::Subscriber sub_global_path = nh.subscribe(global_path_topic, 3, cbGlobalPath);
    pub_local_path = nh.advertise<nav_msgs::Path>(local_path_topic, 3, true);

    ros::spin();
    return 0;
}
