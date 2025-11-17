#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <Eigen/Dense>
#include <cmath>

nav_msgs::Path local_path;
geometry_msgs::PoseStamped current_goal;
geometry_msgs::PoseStamped current_pose;

bool has_path = false;
bool has_pose = false;
bool need_finished = false;
bool has_finished = false;
int current_index = 0;

double goal_distance_threshold = 0.5; // 到达目标的判定距离
int path_step = 5; // 每隔几个点发送一次目标
double publish_rate = 5.0; // Hz
std::string path_topic, odom_topic, goal_topic;


void cbPath(const nav_msgs::Path::ConstPtr& msg)
{
    if (msg->poses.empty()) {
        ROS_WARN("Received empty path!");
        return;
    }
    local_path = *msg;
    has_path = true;
    has_finished = false;
    current_index = 0;
    ROS_INFO("[path_follower_3d] Received new path with %lu points.", msg->poses.size());
}

void cbOdom(const nav_msgs::Odometry::ConstPtr& msg)
{
    current_pose.pose = msg->pose.pose;
    current_pose.header = msg->header;
    has_pose = true;
}

// 工具函数：3D 距离
double distance3D(const geometry_msgs::PoseStamped& a, const geometry_msgs::PoseStamped& b)
{
    double dx = a.pose.position.x - b.pose.position.x;
    double dy = a.pose.position.y - b.pose.position.y;
    double dz = a.pose.position.z - b.pose.position.z;
    return std::sqrt(dx * dx + dy * dy + dz * dz);
}

// 工具函数：向量归一化
Eigen::Vector3d normalize(const Eigen::Vector3d &v)
{
    if (v.norm() < 1e-6) return Eigen::Vector3d(0,0,0);
    return v.normalized();
}

bool is_in_sphere()
{
    bool reached_goal = false;

    if (current_index >= path_step * 2)
    {
        geometry_msgs::PoseStamped prev_goal = local_path.poses[current_index - path_step * 2];

        // 方向向量（从 prev → current）
        Eigen::Vector3d prev(
            prev_goal.pose.position.x,
            prev_goal.pose.position.y,
            prev_goal.pose.position.z);

        Eigen::Vector3d cur(
            current_goal.pose.position.x,
            current_goal.pose.position.y,
            current_goal.pose.position.z);

        Eigen::Vector3d last(
            local_path.poses.back().pose.position.x,
            local_path.poses.back().pose.position.y,
            local_path.poses.back().pose.position.z);

        Eigen::Vector3d dir = normalize(cur - prev);

        // 当前目标点到最终点的距离（路径剩余长度）
        double remain_dist = (last - cur).norm();

        // 球心：沿当前方向延长 remain_dist
        Eigen::Vector3d center = cur + dir * remain_dist;

        // 球半径
        double radius = remain_dist + goal_distance_threshold;

        // 当前位姿
        Eigen::Vector3d cur_pos(
            current_pose.pose.position.x,
            current_pose.pose.position.y,
            current_pose.pose.position.z);

        // 判断是否进入球体
        double dist_to_center = (cur_pos - center).norm();
        if (dist_to_center < radius)
            reached_goal = true;

        // //Debug
        // ROS_INFO_THROTTLE(0.5,
        //     "[path_follower_3d] Check sphere: remain=%.2f radius=%.2f dist_center=%.2f -> %s",
        //     remain_dist, radius, dist_to_center,
        //     reached_goal ? "ENTER" : "OUT");
    }
    else
    {
        // 第一个点：仍然使用传统距离判断
        if (distance3D(current_pose, current_goal) < goal_distance_threshold)
            reached_goal = true;
    }
    return reached_goal;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "path_follower");
    ros::NodeHandle nh("~");

    // 参数读取
    nh.param("goal_distance_threshold", goal_distance_threshold, 0.5);
    nh.param("path_step", path_step, 5);
    nh.param("publish_rate", publish_rate, 5.0);
    nh.param<std::string>("path_topic", path_topic, "/global_path");
    nh.param<std::string>("odom_topic", odom_topic, "/localization");
    nh.param<std::string>("goal_topic", goal_topic, "/move_base_simple/goal");

    ros::Subscriber sub_path = nh.subscribe(path_topic, 1, cbPath);
    ros::Subscriber sub_odom = nh.subscribe(odom_topic, 1, cbOdom);
    ros::Publisher pub_goal = nh.advertise<geometry_msgs::PoseStamped>(goal_topic, 1);

    ros::Rate rate(publish_rate);

    ROS_INFO("[path_follower_3d] Node started. Waiting for /path and /odom...");

    while (ros::ok())
    {
        ros::spinOnce();

        if (has_path && has_pose)
        {
            if (!has_finished)
            {
                //if (distance3D(current_pose, current_goal) < goal_distance_threshold || current_goal.header.seq == 0)
                if (is_in_sphere() || current_index == 0)
                {
                    current_goal = local_path.poses[current_index];
                    pub_goal.publish(current_goal);

                    ROS_INFO("[path_follower_3d] New goal #%d: (x=%.2f, y=%.2f, z=%.2f)",
                             current_index,
                             current_goal.pose.position.x,
                             current_goal.pose.position.y,
                             current_goal.pose.position.z);

                    if (need_finished){has_finished = true;continue;}
                    current_index += path_step;
                    if (current_index >= (int)local_path.poses.size())
                    {
                        current_index = local_path.poses.size() - 1;
                        need_finished = true;
                    }
                }
            }
            else
            {
                ROS_INFO("[path_follower_3d] Path finished.");
                has_path = false;
                current_index = 0;
            }
        }

        rate.sleep();
    }

    return 0;
}
