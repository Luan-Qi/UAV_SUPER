/**
 * @file odometry_to_px4.cpp
 * @brief 里程计→PX4 视觉位姿桥接节点 — 坐标变换与话题转发。
 *
 * @details
 * 本节点订阅自主定位模块发布的里程计话题（如 Fast-LIO 的 /Odometry），
 * 经坐标系修正后发布到 MAVROS 视觉位姿接口 /mavros/vision_pose/pose。
 * 核心功能：
 *   1. 坐标变换 — ENU → NED 转换（x' = -y, y' = x, yaw += 90°）
 *   2. 话题转发 — nav_msgs/Odometry → geometry_msgs/PoseStamped
 *
 * 坐标系约定：
 *   - 输入：里程计系 (odom, ENU-like, Fast-LIO 默认)
 *   - 输出：视觉位姿系 (odom, 经旋转后对齐 PX4 NED 约定)
 *   - 注意：仅做位姿转发，不发布速度指令（速度由 px4ctrl_node 处理）
 *
 * 依赖：
 *   - nav_msgs/Odometry
 *   - geometry_msgs/PoseStamped
 *   - tf2 (坐标旋转)
 *
 * 使用示例：
 *   rosrun uav_px4_ctrl_test odometry_to_px4_node
 */

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

// ============================================================================
// OdometryToPX4 — 里程计→PX4 视觉位姿桥接类
// ============================================================================
class OdometryToPX4
{
public:
    /**
     * @brief 构造函数：初始化发布者与订阅者。
     */
    OdometryToPX4()
    {
        ros::NodeHandle nh;

        // 发布 PX4 期望的视觉位姿话题
        pose_pub_ = nh.advertise<geometry_msgs::PoseStamped>("/mavros/vision_pose/pose", 10);

        // 订阅里程计（默认来自 Fast-LIO 的 /Odometry）
        sub_ = nh.subscribe("/Odometry", 10, &OdometryToPX4::odometryCallback, this);

        ROS_INFO("start published PX4 Pose and Twist");
    }

    /**
     * @brief 里程计回调：坐标修正 + 位姿发布。
     * @param msg 里程计消息指针 [nav_msgs::Odometry]
     *
     * 坐标系修正：ENU → NED 等效变换
     *   - position: x' = -y, y' = x
     *   - orientation: 绕 Z 轴旋转 +90° (π/2)
     */
    void odometryCallback(const nav_msgs::Odometry::ConstPtr& msg)
    {
        // 发布位姿
        geometry_msgs::PoseStamped pose_msg;
        pose_msg.header = msg->header;      // 保留时间戳和 frame_id
        pose_msg.header.frame_id = "odom";
        pose_msg.pose = msg->pose.pose;     // 直接复制位姿

        // ENU → NED 位置变换
        pose_msg.pose.position.x = -msg->pose.pose.position.y;
        pose_msg.pose.position.y = msg->pose.pose.position.x;

        // 绕 Z 轴旋转 +90° (π/2)：补偿 ENU→NED 的方向差异
        tf2::Quaternion q_orig, q_rot, q_new;
        tf2::fromMsg(msg->pose.pose.orientation, q_orig);
        q_rot.setRPY(0, 0, M_PI/2);  // 绕 Z 轴旋转 +90°
        q_new = q_rot * q_orig;      // 旋转叠加 (先应用原始姿态，再绕Z转90°)
        q_new.normalize();

        pose_msg.pose.orientation = tf2::toMsg(q_new);

        pose_pub_.publish(pose_msg);
    }

private:
    ros::Publisher pose_pub_;   ///< 视觉位姿发布者 → /mavros/vision_pose/pose
    ros::Subscriber sub_;       ///< 里程计订阅者 ← /Odometry
};

// ============================================================================
// main — 节点入口
// ============================================================================
int main(int argc, char** argv)
{
    ros::init(argc, argv, "odometry_to_px4_node");

    OdometryToPX4 node;
    ros::spin();

    return 0;
}
