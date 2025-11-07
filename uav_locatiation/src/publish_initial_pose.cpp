#include <ros/ros.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>
#include <tf/tf.h>
#include <iostream>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "publish_initial_pose");
    ros::NodeHandle nh;

    if (argc < 7) {
        ROS_ERROR("Usage: rosrun <package_name> publish_initial_pose x y z yaw pitch roll");
        return 1;
    }

    double x = atof(argv[1]);
    double y = atof(argv[2]);
    double z = atof(argv[3]);
    double yaw = atof(argv[4]);
    double pitch = atof(argv[5]);
    double roll = atof(argv[6]);

    // 创建publisher
    ros::Publisher pub_pose = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("/initialpose", 1);

    // 欧拉角 -> 四元数
    tf::Quaternion q;
    q.setRPY(roll, pitch, yaw);
    geometry_msgs::Quaternion q_msg;
    tf::quaternionTFToMsg(q, q_msg);

    // 填充Pose
    geometry_msgs::PoseWithCovarianceStamped initial_pose;
    initial_pose.header.stamp = ros::Time::now();
    initial_pose.header.frame_id = "map";
    initial_pose.pose.pose.position.x = x;
    initial_pose.pose.pose.position.y = y;
    initial_pose.pose.pose.position.z = z;
    initial_pose.pose.pose.orientation = q_msg;

    // 可选：协方差初始化为0
    for (int i = 0; i < 36; i++)
        initial_pose.pose.covariance[i] = 0.0;

    ros::Duration(1.0).sleep();  // 等待发布器连接
    ROS_INFO("[pub_init_pose] Publishing initial pose: x=%.3f y=%.3f z=%.3f yaw=%.3f pitch=%.3f roll=%.3f",
             x, y, z, yaw, pitch, roll);
    pub_pose.publish(initial_pose);

    ros::spinOnce();
    ros::Duration(0.5).sleep();  // 确保消息发出
    return 0;
}
