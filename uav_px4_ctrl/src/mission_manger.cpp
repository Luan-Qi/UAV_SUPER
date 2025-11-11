#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <vector>
#include <string>
#include <cmath>
#include <regex>
#include <Eigen/Dense>
#include "uav_px4_ctrl/TakeoffNotify.h"


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
        nh.param("mission_cycle", mission_cycle_, false);

        std::string waypoint_str;
        nh.param("waypoints", waypoint_str, std::string("[[0,0,1.0],[2,0,1.0]]"));
        parseWaypointString(waypoint_str);

        if (waypoints_.empty())
        {
            ROS_ERROR("[mission] No waypoints loaded!");
            ros::shutdown();
        }

        ROS_INFO("[mission] Loaded %zu waypoints:", waypoints_.size());
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

        trans_sub_ = nh.subscribe("/map_to_odom", 2, &WaypointPublisher::map2odomCallback, this);
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
            ROS_INFO("[mission] Reached waypoint %d (%.2f, %.2f, %.2f)",
                     current_wp_idx_ + 1,
                     target.pose.position.x,
                     target.pose.position.y,
                     target.pose.position.z);
            reached_ = true;
            reach_time_ = ros::Time::now();
        }

        if (reached_ && (ros::Time::now() - reach_time_).toSec() >= wait_time_)
        {
            current_wp_idx_ += 1;
            if(mission_cycle_)
            {
                current_wp_idx_ %= waypoints_.size();
            }
            else
            {
                if(current_wp_idx_ >= waypoints_.size())
                {
                    ROS_INFO("[mission] All missions have been completed !");
                    shutdown_requested_ = true;
                    return;
                }
            }
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
            ROS_INFO("[mission] Published goal %d: (%.2f, %.2f, %.2f)",
                     current_wp_idx_ + 1,
                     waypoints_[current_wp_idx_].pose.position.x,
                     waypoints_[current_wp_idx_].pose.position.y,
                     waypoints_[current_wp_idx_].pose.position.z);
        }
    }

    void map2odomCallback(const nav_msgs::Odometry::ConstPtr &msg)
    {
        if(have_map_to_odom_ || takeoff_done_) return;

        cur_map_to_odom = *msg;
        have_map_to_odom_ = true;

        ROS_INFO("[mission] Got map_to_odom transform!");
        ROS_INFO("[mission] transformed %zu waypoints:", waypoints_.size());

        Eigen::Matrix4d T_map_to_odom = poseToMatrix(cur_map_to_odom.pose.pose);
        Eigen::Matrix4d T_odom_to_map = T_map_to_odom.inverse();
        for (size_t i = 0; i < waypoints_.size(); ++i)
        {
            Eigen::Matrix4d wp_global = poseToMatrix(waypoints_[i].pose);
            Eigen::Matrix4d wp_local = T_odom_to_map * wp_global;
            waypoints_[i].pose = matrixToPose(wp_local);

            ROS_INFO("  [%zu] x=%.2f, y=%.2f, z=%.2f",
                     i + 1,
                     waypoints_[i].pose.position.x,
                     waypoints_[i].pose.position.y,
                     waypoints_[i].pose.position.z);
        }
    }

    bool takeoffCallback(uav_px4_ctrl::TakeoffNotify::Request &req,
                         uav_px4_ctrl::TakeoffNotify::Response &res)
    {
        if (req.takeoff_done)
        {
            ROS_INFO("[mission] UAV has taken off!");
            res.response = "Hello, world!";
            takeoff_done_ = true;
        }
        return true;
    }

    void timeoutCheckCallback(const ros::TimerEvent &)
    {
        if (last_pose_time_.isZero())
        {
            ROS_WARN_THROTTLE(10, "[mission] No position message received yet...");
            return;
        }

        double dt = (ros::Time::now() - last_pose_time_).toSec();
        if (dt > topic_timeout_)
        {
            ROS_WARN("[mission] No position update! Check your topics.");
        }
    }

    void spin()
    {
        ros::Rate rate(10);
        ROS_INFO("[mission] Waiting for takeoff notification...");
        while (ros::ok())
        {
            if (takeoff_done_) break;
            ros::spinOnce();
            rate.sleep();
        }

        ROS_INFO("[mission] Waiting %.1f seconds before starting...", start_delay_);
        ros::Duration(start_delay_).sleep();
        publishGoal();

        while (ros::ok())
        {
            if (shutdown_requested_)
            {
                timeout_timer_.stop();
                return;
            }
            ros::spinOnce();
            rate.sleep();
        }
    }

private:
    ros::Publisher goal_pub_;
    ros::Subscriber pose_sub_;
    ros::Subscriber odom_sub_;
    ros::Subscriber trans_sub_;
    ros::ServiceServer srv_takeoff_;
    ros::ServiceClient srv_accomplish_;
    ros::ServiceClient srv_abort_;
    ros::Timer timeout_timer_;

    std::vector<geometry_msgs::PoseStamped> waypoints_;
    geometry_msgs::PoseStamped current_pose_;
    std::string pose_topic_;
    std::string odom_topic_;
    std::string goal_topic_;
    bool mission_cycle_;

    double distance_threshold_;
    double wait_time_;
    double start_delay_;
    double topic_timeout_;

    int current_wp_idx_;
    bool reached_;
    bool use_odom_;
    bool have_map_to_odom_;
    bool takeoff_done_;
    bool shutdown_requested_;

    nav_msgs::Odometry cur_map_to_odom;
    ros::Time reach_time_;
    ros::Time last_pose_time_;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "mission_manger");
    ros::NodeHandle nh("~");

    WaypointPublisher wp_pub(nh);
    wp_pub.spin();
    ros::shutdown();
    return 0;
}
