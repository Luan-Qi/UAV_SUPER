#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

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

    double x = atof(argv[1]);
    double y = atof(argv[2]);
    double z = atof(argv[3]);

    double yaw_deg = 0.0;
    if (argc >= 5)
        yaw_deg = atof(argv[4]);
    double yaw_rad = yaw_deg * M_PI / 180.0;

    std::string topic_name = "/move_base_simple/goal";
    if (argc >= 6)
        topic_name = argv[5];

    // 计算四元数方向
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
