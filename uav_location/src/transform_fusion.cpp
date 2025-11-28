#include <ros/ros.h>
#include <thread>
#include <mutex>

#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <Eigen/Dense>

using namespace std;

ros::Publisher pub_localization;
ros::Publisher pub_global_start_pose;
nav_msgs::Odometry cur_odom_to_baselink;
nav_msgs::Odometry cur_map_to_odom;

bool has_odom = false;
bool has_map_to_odom = false;
bool have_global_start_pose = false;
std::mutex mtx;

double FREQ_PUB_LOCALIZATION = 30.0;

// -------------------- 工具函数：Pose -> Eigen::Matrix4d --------------------
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

// -------------------- 回调函数 --------------------
void cbOdom(const nav_msgs::Odometry::ConstPtr &msg)
{
    std::lock_guard<std::mutex> lock(mtx);
    cur_odom_to_baselink = *msg;
    has_odom = true;
}

void cbMapToOdom(const nav_msgs::Odometry::ConstPtr &msg)
{
    std::lock_guard<std::mutex> lock(mtx);
    cur_map_to_odom = *msg;
    has_map_to_odom = true;
}

// -------------------- 坐标融合线程 --------------------
void transformFusion()
{
    tf::TransformBroadcaster br;
    ros::Rate rate(FREQ_PUB_LOCALIZATION);

    while (ros::ok())
    {
        nav_msgs::Odometry cur_odom_copy;
        nav_msgs::Odometry cur_map_copy;

        {
            std::lock_guard<std::mutex> lock(mtx);
            cur_odom_copy = cur_odom_to_baselink;
            cur_map_copy = cur_map_to_odom;
        }

        Eigen::Matrix4d T_map_to_odom = Eigen::Matrix4d::Identity();
        if (has_map_to_odom)
        {
            T_map_to_odom = poseToMatrix(cur_map_copy.pose.pose);
        }

        // 广播 TF: map -> local
        tf::Transform tf_map_to_cam;
        tf::Vector3 t_map_to_cam(T_map_to_odom(0, 3), T_map_to_odom(1, 3), T_map_to_odom(2, 3));
        Eigen::Matrix3d R_map_to_cam = T_map_to_odom.block<3, 3>(0, 0);
        Eigen::Quaterniond q_map_to_cam(R_map_to_cam);
        tf::Quaternion tf_q(q_map_to_cam.x(), q_map_to_cam.y(), q_map_to_cam.z(), q_map_to_cam.w());
        tf_map_to_cam.setOrigin(t_map_to_cam);
        tf_map_to_cam.setRotation(tf_q);
        br.sendTransform(tf::StampedTransform(tf_map_to_cam, ros::Time::now(), "map", "camera_init"));

        // 如果有 odometry，则发布融合后的 localization
        if (has_odom)
        {
            Eigen::Matrix4d T_odom_to_base = poseToMatrix(cur_odom_copy.pose.pose);
            Eigen::Matrix4d T_map_to_base = T_map_to_odom * T_odom_to_base;

            Eigen::Vector3d t = T_map_to_base.block<3, 1>(0, 3);
            Eigen::Matrix3d R = T_map_to_base.block<3, 3>(0, 0);
            Eigen::Quaterniond q(R);

            nav_msgs::Odometry localization;
            localization.header.stamp = cur_odom_copy.header.stamp;
            localization.header.frame_id = "map";
            localization.child_frame_id = "base_link";

            localization.pose.pose.position.x = t.x();
            localization.pose.pose.position.y = t.y();
            localization.pose.pose.position.z = t.z();
            localization.pose.pose.orientation.x = q.x();
            localization.pose.pose.orientation.y = q.y();
            localization.pose.pose.orientation.z = q.z();
            localization.pose.pose.orientation.w = q.w();

            localization.twist = cur_odom_copy.twist;

            pub_localization.publish(localization);

            if(!have_global_start_pose && has_map_to_odom)
            {
                ROS_INFO("[global] Global localization initialized at (%f, %f, %f)", 
                         localization.pose.pose.position.x, 
                         localization.pose.pose.position.y, 
                         localization.pose.pose.position.z);
                have_global_start_pose = true;
            }
        }

        rate.sleep();
    }
    ros::shutdown();
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "transform_fusion");
    ros::NodeHandle nh;

    ROS_INFO("[global] Transform Fusion Node Inited...");

    std::string global_start_topic;
    nh.param<double>("global_locatiation_hz", FREQ_PUB_LOCALIZATION, FREQ_PUB_LOCALIZATION);

    ros::Subscriber sub_odom = nh.subscribe("/Odometry", 10, cbOdom);
    ros::Subscriber sub_map_to_odom = nh.subscribe("/map_to_odom", 2, cbMapToOdom);
    pub_localization = nh.advertise<nav_msgs::Odometry>("/localization", 10);

    // 开启独立线程进行融合发布
    std::thread th(transformFusion);
    th.detach();

    ros::spin();
    return 0;
}
