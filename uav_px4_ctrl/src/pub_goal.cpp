/**
 * @file pub_goal.cpp
 * @brief 命令行目标点发布工具 — 向指定话题发送一次 goal pose。
 *
 * @details
 * 通过命令行参数指定目标点 (x, y, z, yaw) 和话题名称，发布一次
 * geometry_msgs/PoseStamped 消息后立即退出。适用于：
 *   1. 调试/测试时手动发送目标点
 *   2. shell 脚本中批量发布 waypoints
 *
 * 话题：发布 — 用户指定 topic（默认 /move_base_simple/goal）
 *
 * 使用示例：
 *   rosrun uav_px4_ctrl pub_goal 2.0 3.0 1.0 90 /move_base_simple/goal
 */

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

// ============================================================================
// 入口
// ============================================================================

int main(int argc, char **argv)
{
    ros::init(argc, argv, "send_goal");
    ros::NodeHandle nh;

    // 检查参数数量
    if (argc < 4)
    {
        ROS_ERROR("Usage: rosrun your_package send_goal x y z [yaw_degree] [topic_name]");
        ROS_ERROR("Example: rosrun your_package send_goal 2.0 3.0 0.0 90 /move_base_simple/goal");
        return -1;
    }

    double x = atof(argv[1]);  ///< 目标 X 坐标 [m]
    double y = atof(argv[2]);  ///< 目标 Y 坐标 [m]
    double z = atof(argv[3]);  ///< 目标 Z 坐标 [m]

    double yaw_deg = 0.0;
    if (argc >= 5)
        yaw_deg = atof(argv[4]);           ///< 目标偏航角 [deg]
    double yaw_rad = yaw_deg * M_PI / 180.0;  ///< 目标偏航角 [rad]

    std::string topic_name = "/move_base_simple/goal";
    if (argc >= 6)
        topic_name = argv[5];  ///< 目标话题名称

    // 计算四元数方向 (仅偏航旋转)
    tf2::Quaternion q;
    q.setRPY(0, 0, yaw_rad);

    ros::Publisher goal_pub = nh.advertise<geometry_msgs::PoseStamped>(topic_name, 10);
    ros::Duration(0.5).sleep(); // 给发布器一点初始化时间

    geometry_msgs::PoseStamped goal;
    goal.header.stamp = ros::Time::now();
    goal.header.frame_id = "map"; // 或 odom，根据你的系统

    goal.pose.position.x = x;
    goal.pose.position.y = y;
    goal.pose.position.z = z;
    goal.pose.orientation = tf2::toMsg(q);

    goal_pub.publish(goal);

    ROS_INFO(" ");
    ROS_INFO("Published topic: %s", topic_name.c_str());
    ROS_INFO("Published goal: [x=%.1f, y=%.1f, z=%.1f, yaw=%.1f']", x, y, z, yaw_deg);
    ROS_INFO(" ");

    ros::spinOnce();
    ros::Duration(0.5).sleep(); // 确保消息被发送出去
    return 0;
}
