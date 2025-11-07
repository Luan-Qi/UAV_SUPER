#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <cmath>

nav_msgs::Path local_path;
geometry_msgs::PoseStamped current_goal;
geometry_msgs::PoseStamped current_pose;

bool has_path = false;
bool has_pose = false;
int current_index = 0;

double goal_distance_threshold = 0.5; // 到达目标的判定距离
int path_step = 5; // 每隔几个点发送一次目标
double publish_rate = 5.0; // Hz

void cbPath(const nav_msgs::Path::ConstPtr& msg)
{
    if (msg->poses.empty()) {
        ROS_WARN("Received empty path!");
        return;
    }
    local_path = *msg;
    has_path = true;
    current_index = 0;
    ROS_INFO("[path_follower] Received new path with %lu points.", msg->poses.size());
}

void cbOdom(const nav_msgs::Odometry::ConstPtr& msg)
{
    current_pose.pose = msg->pose.pose;
    current_pose.header = msg->header;
    has_pose = true;
}

double distance(const geometry_msgs::PoseStamped& a, const geometry_msgs::PoseStamped& b)
{
    return std::hypot(a.pose.position.x - b.pose.position.x,
                      a.pose.position.y - b.pose.position.y);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "path_follower");
    ros::NodeHandle nh("~");

    // 参数读取
    nh.param("goal_distance_threshold", goal_distance_threshold, 0.5);
    nh.param("path_step", path_step, 5);
    nh.param("publish_rate", publish_rate, 5.0);

    ros::Subscriber sub_path = nh.subscribe("/local_path", 1, cbPath);
    ros::Subscriber sub_odom = nh.subscribe("/Odometry", 1, cbOdom);
    ros::Publisher pub_goal = nh.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 1);

    ros::Rate rate(publish_rate);

    ROS_INFO("[path_follower] Node started. Waiting for /local_path and /odom...");

    while (ros::ok())
    {
        ros::spinOnce();

        if (has_path && has_pose)
        {
            if (current_index < (int)local_path.poses.size())
            {
                // 如果没有目标，或者已经接近当前目标，则切换到下一个点
                if (distance(current_pose, current_goal) < goal_distance_threshold || current_goal.header.seq == 0)
                {
                    current_goal = local_path.poses[current_index];
                    pub_goal.publish(current_goal);

                    ROS_INFO("[path_follower] New goal sent: #%d (x=%.2f, y=%.2f)",
                             current_index,
                             current_goal.pose.position.x,
                             current_goal.pose.position.y);

                    current_index += path_step;
                    if (current_index >= (int)local_path.poses.size())
                        current_index = local_path.poses.size() - 1;
                }
            }
            else
            {
                ROS_INFO_THROTTLE(5.0, "[path_follower] Path finished.");
            }
        }

        rate.sleep();
    }

    return 0;
}
