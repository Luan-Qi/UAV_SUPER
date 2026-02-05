#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>


class OdometryToPX4
{
public:
    OdometryToPX4()
    {
        ros::NodeHandle nh;

        // 发布 PX4 期望的位姿话题
        pose_pub_ = nh.advertise<geometry_msgs::PoseStamped>("/mavros/vision_pose/pose", 10);

        // 订阅 Fast-LIO 的 Odometry
        sub_ = nh.subscribe("/Odometry", 10, &OdometryToPX4::odometryCallback, this);

	ROS_INFO("start published PX4 Pose and Twist");
    }

    void odometryCallback(const nav_msgs::Odometry::ConstPtr& msg)
    {
        // 发布位姿
        geometry_msgs::PoseStamped pose_msg;
        pose_msg.header = msg->header;      // 保留时间戳和 frame_id
        pose_msg.header.frame_id = "odom";
        pose_msg.pose = msg->pose.pose;     // 直接复制位姿
        pose_msg.pose.position.x = -msg->pose.pose.position.y;
        pose_msg.pose.position.y = msg->pose.pose.position.x;

        // 绕 Z 轴旋转 180 度 (π 弧度)
        tf2::Quaternion q_orig, q_rot, q_new;
        tf2::fromMsg(msg->pose.pose.orientation, q_orig);
        q_rot.setRPY(0, 0, M_PI/2);  // 绕Z轴旋转180度
        q_new = q_rot * q_orig;    // 旋转叠加
        q_new.normalize();

        pose_msg.pose.orientation = tf2::toMsg(q_new);

        pose_pub_.publish(pose_msg);
    }

private:
    ros::Publisher pose_pub_;
    ros::Subscriber sub_;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "odometry_to_px4_node");

    OdometryToPX4 node;
    ros::spin();

    return 0;
}
