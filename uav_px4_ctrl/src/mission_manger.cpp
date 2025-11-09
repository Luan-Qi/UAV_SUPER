#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <vector>
#include <string>
#include <cmath>
#include <regex>
#include "uav_px4_ctrl/TakeoffNotify.h"

class WaypointPublisher
{
public:
    WaypointPublisher(ros::NodeHandle &nh)
    {
        nh.param("pose_topic", pose_topic_, std::string("/mavros/local_position/pose"));
        nh.param("odom_topic", odom_topic_, std::string(""));
        nh.param("goal_topic", goal_topic_, std::string("/move_base_simple/goal"));
        nh.param("distance_threshold", distance_threshold_, 0.2);
        nh.param("wait_time", wait_time_, 0.0);
        nh.param("start_delay", start_delay_, 3.0);
        nh.param("topic_timeout", topic_timeout_, 3.0);

        std::string waypoint_str;
        nh.param("waypoints", waypoint_str, std::string("[[0,0,1.0],[2,0,1.0]]"));
        parseWaypointString(waypoint_str);

        if (waypoints_.empty())
        {
            ROS_ERROR("No waypoints loaded!");
            ros::shutdown();
        }

        ROS_INFO("Loaded %zu waypoints:", waypoints_.size());
        for (size_t i = 0; i < waypoints_.size(); ++i)
        {
            ROS_INFO("  [%zu] x=%.2f, y=%.2f, z=%.2f",
                     i + 1,
                     waypoints_[i].pose.position.x,
                     waypoints_[i].pose.position.y,
                     waypoints_[i].pose.position.z);
        }

        goal_pub_ = nh.advertise<geometry_msgs::PoseStamped>(goal_topic_, 10);

        if (!odom_topic_.empty())
        {
            odom_sub_ = nh.subscribe(odom_topic_, 10, &WaypointPublisher::odomCallback, this);
            use_odom_ = true;
        }
        else
        {
            pose_sub_ = nh.subscribe(pose_topic_, 10, &WaypointPublisher::poseCallback, this);
            use_odom_ = false;
        }

        srv_takeoff_ = nh.advertiseService("/takeoff_notify", &WaypointPublisher::takeoffCallback, this);
        timeout_timer_ = nh.createTimer(ros::Duration(5.0), &WaypointPublisher::timeoutCheckCallback, this);

        current_wp_idx_ = 0;
        reached_ = false;
        last_pose_time_ = ros::Time(0);
    }

    void parseWaypointString(const std::string &input)
    {
        std::regex point_regex("\\[\\s*([-0-9\\.eE]+)\\s*,\\s*([-0-9\\.eE]+)\\s*,\\s*([-0-9\\.eE]+)\\s*\\]");
        std::smatch match;
        std::string s = input;

        while (std::regex_search(s, match, point_regex))
        {
            if (match.size() == 4)
            {
                geometry_msgs::PoseStamped wp;
                wp.header.frame_id = "map";
                wp.pose.position.x = std::stod(match[1]);
                wp.pose.position.y = std::stod(match[2]);
                wp.pose.position.z = std::stod(match[3]);
                wp.pose.orientation.w = 1.0;
                waypoints_.push_back(wp);
            }
            s = match.suffix();
        }
    }

    void odomCallback(const nav_msgs::Odometry::ConstPtr &msg)
    {
        current_pose_.pose = msg->pose.pose;
        last_pose_time_ = msg->header.stamp;
        if(takeoff_done_) checkAndSwitchWaypoint();
    }

    void poseCallback(const geometry_msgs::PoseStamped::ConstPtr &msg)
    {
        current_pose_ = *msg;
        last_pose_time_ = msg->header.stamp;
        if(takeoff_done_) checkAndSwitchWaypoint();
    }

    void checkAndSwitchWaypoint()
    {
        if (waypoints_.empty())
            return;

        geometry_msgs::PoseStamped target = waypoints_[current_wp_idx_];
        double dx = target.pose.position.x - current_pose_.pose.position.x;
        double dy = target.pose.position.y - current_pose_.pose.position.y;
        double dz = target.pose.position.z - current_pose_.pose.position.z;
        double dist = std::sqrt(dx * dx + dy * dy + dz * dz);

        if (!reached_ && dist < distance_threshold_)
        {
            ROS_INFO("Reached waypoint %d (%.2f, %.2f, %.2f)",
                     current_wp_idx_ + 1,
                     target.pose.position.x,
                     target.pose.position.y,
                     target.pose.position.z);
            reached_ = true;
            reach_time_ = ros::Time::now();
        }

        if (reached_ && (ros::Time::now() - reach_time_).toSec() >= wait_time_)
        {
            current_wp_idx_ = (current_wp_idx_ + 1) % waypoints_.size();
            publishGoal();
            reached_ = false;
        }
    }

    void publishGoal()
    {
        if (current_wp_idx_ < waypoints_.size())
        {
            waypoints_[current_wp_idx_].header.stamp = ros::Time::now();
            goal_pub_.publish(waypoints_[current_wp_idx_]);
            ROS_INFO("Published goal %d: (%.2f, %.2f, %.2f)",
                     current_wp_idx_ + 1,
                     waypoints_[current_wp_idx_].pose.position.x,
                     waypoints_[current_wp_idx_].pose.position.y,
                     waypoints_[current_wp_idx_].pose.position.z);
        }
    }

    bool takeoffCallback(uav_px4_ctrl::TakeoffNotify::Request &req,
                         uav_px4_ctrl::TakeoffNotify::Response &res)
    {
        if (req.takeoff_done)
        {
            ROS_INFO("UAV has taken off!");
            res.response = "Hello, world!";
            takeoff_done_ = true;
        }
        return true;
    }

    void timeoutCheckCallback(const ros::TimerEvent &)
    {
        if (last_pose_time_.isZero())
        {
            ROS_WARN_THROTTLE(10, "No position message received yet...");
            return;
        }

        double dt = (ros::Time::now() - last_pose_time_).toSec();
        if (dt > topic_timeout_)
        {
            ROS_WARN("No position update! Check your topics.");
        }
    }

    void spin()
    {
        ros::Rate rate(10);
        ROS_INFO("Waiting for takeoff notification...");
        while (ros::ok())
        {
            if (takeoff_done_) break;
            ros::spinOnce();
            rate.sleep();
        }

        ROS_INFO("Waiting %.1f seconds before starting...", start_delay_);
        ros::Duration(start_delay_).sleep();
        publishGoal();

        while (ros::ok())
        {
            ros::spinOnce();
            rate.sleep();
        }
    }

private:
    ros::Publisher goal_pub_;
    ros::Subscriber pose_sub_;
    ros::Subscriber odom_sub_;
    ros::ServiceServer srv_takeoff_;
    ros::Timer timeout_timer_;

    std::vector<geometry_msgs::PoseStamped> waypoints_;
    geometry_msgs::PoseStamped current_pose_;
    std::string pose_topic_;
    std::string odom_topic_;
    std::string goal_topic_;

    double distance_threshold_;
    double wait_time_;
    double start_delay_;
    double topic_timeout_;

    int current_wp_idx_;
    bool reached_;
    bool use_odom_;
    bool takeoff_done_;

    ros::Time reach_time_;
    ros::Time last_pose_time_;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "waypoint_publisher");
    ros::NodeHandle nh("~");

    WaypointPublisher wp_pub(nh);
    wp_pub.spin();
    return 0;
}
