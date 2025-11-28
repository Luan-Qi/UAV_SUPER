#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/String.h>
#include <vector>
#include <string>
#include <cmath>
#include <regex>
#include <thread>
#include <mutex>
#include <Eigen/Dense>
#include "uav_px4_ctrl/TakeoffNotify.h"

std::mutex mutex_;
enum WaypointEventType {
    WP_EVENT_DISPATCH = 0, // 发布目标点（开始前往）
    WP_EVENT_ARRIVED = 1   // 到达目标点（结束一段）
};

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
        nh.param("start_topic", start_topic_, std::string("/start_pose"));
        nh.param("goal_topic", goal_topic_, std::string("/move_base_simple/goal"));
        nh.param("distance_threshold", distance_threshold_, 0.2);
        nh.param("wait_time", wait_time_, 0.0);
        nh.param("start_delay", start_delay_, 3.0);
        nh.param("topic_timeout", topic_timeout_, 3.0);
        nh.param("mission_cycle", mission_cycle_, false);

        std::string waypoint_str;
        nh.param("waypoints", waypoint_str, std::string("[[0,0,1.0]*,[2,0,1.0]]"));
        parseWaypointString(waypoint_str);

        if (waypoints_.empty())
        {
            ROS_ERROR("[mission] No waypoints loaded!");
            ros::shutdown();
        }

        ROS_INFO("[mission] Loaded %zu waypoints:", waypoints_.size());
        for (size_t i = 0; i < waypoints_.size(); ++i)
        {
            // Add video control output
            bool rec_flag = (i < waypoints_record_flags_.size()) ? waypoints_record_flags_[i] : false;
            std::string rec_str = rec_flag ? "[REC: ON ]" : "[REC: OFF]";

            // Wait time string
            double wt = (i < waypoints_wait_times_.size()) ?
                        waypoints_wait_times_[i] : wait_time_;

            std::string wait_str;
            if (wt == std::numeric_limits<double>::max())
            {
                wait_str = "∞";
            }
            else if (wt == wait_time_)
            {
                char buf[64];
                snprintf(buf, sizeof(buf), "default(%.1fs)", wt);
                wait_str = buf;
            }
            else
            {
                char buf[32];
                snprintf(buf, sizeof(buf), "%.1fs", wt);
                wait_str = buf;
            }

            ROS_INFO("  [%zu] %s x=%5.1f, y=%5.1f, z=%4.1f, wait=%s",
                     i + 1,
                     rec_str.c_str(),
                     waypoints_[i].pose.position.x,
                     waypoints_[i].pose.position.y,
                     waypoints_[i].pose.position.z,
                     wait_str.c_str());
        }

        start_pub_ = nh.advertise<geometry_msgs::PoseStamped>(start_topic_, 10);
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

        //trans_sub_ = nh.subscribe("/map_to_odom", 2, &WaypointPublisher::map2odomCallback, this);
        srv_takeoff_ = nh.advertiseService("/takeoff_notify", &WaypointPublisher::takeoffCallback, this);
        srv_planfail_ = nh.advertiseService("/planner_fail_notify", &WaypointPublisher::planfailCallback, this);
        srv_video_pub = nh.advertise<std_msgs::String>("/camera_recorder/record_control", 1);
        
        timeout_timer_ = nh.createTimer(ros::Duration(5.0), &WaypointPublisher::timeoutCheckCallback, this);
    }

    void parseWaypointString(const std::string &input)
    {
        waypoints_.clear();
        waypoints_record_flags_.clear();
        waypoints_wait_times_.clear();

        std::regex point_regex("\\[\\s*([-0-9\\.eE]+)\\s*,"
                               "\\s*([-0-9\\.eE]+)\\s*,"
                               "\\s*([-0-9\\.eE]+)"
                               "(?:\\s*,\\s*([0-9\\.eE]+))?"
                               "\\s*\\]\\s*(\\*?)"
        );

        std::smatch match;
        std::string s = input;

        while (std::regex_search(s, match, point_regex))
        {
            if (match.size() >= 4)
            {
                geometry_msgs::PoseStamped wp;
                wp.header.frame_id = "map";
                wp.pose.position.x = std::stod(match[1]);
                wp.pose.position.y = std::stod(match[2]);
                wp.pose.position.z = std::stod(match[3]);
                wp.pose.orientation.w = 1.0;
                waypoints_.push_back(wp);

                // wait time (non-negative)
                double wait_t;
                if (match[4].str().empty())
                {
                    wait_t = wait_time_;
                }
                else
                {
                    wait_t = std::stod(match[4].str());
                    if (wait_t == 0.0)
                        wait_t = std::numeric_limits<double>::max();
                    else if (wait_t < 0.0)
                        wait_t = wait_time_;
                }
                waypoints_wait_times_.push_back(wait_t);

                // record_flag ("*")
                bool record_flag = (match.size() > 5 && match[5].length() > 0 && match[5].str() == "*");
                waypoints_record_flags_.push_back(record_flag);
            }
            s = match.suffix();
        }
    }

    void odomCallback(const nav_msgs::Odometry::ConstPtr &msg)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        current_pose_.pose = msg->pose.pose;
        last_pose_time_ = msg->header.stamp;
    }

    void poseCallback(const geometry_msgs::PoseStamped::ConstPtr &msg)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        current_pose_ = *msg;
        last_pose_time_ = msg->header.stamp;
    }

    void missionReset()
    {
        std::lock_guard<std::mutex> lock(mutex_);
        mission_running_ = false;
    }

    void missionCheckWaypoint()
    {
        ros::Rate rate(10);
        while (ros::ok()) 
        {
            rate.sleep();

            if (!mission_running_) continue;

            if (!reached_)
            {
                std::lock_guard<std::mutex> lock(mutex_);
                geometry_msgs::PoseStamped target = waypoints_[current_wp_idx_];
                double dx = target.pose.position.x - current_pose_.pose.position.x;
                double dy = target.pose.position.y - current_pose_.pose.position.y;
                double dz = target.pose.position.z - current_pose_.pose.position.z;
                double dist = std::sqrt(dx * dx + dy * dy + dz * dz);
                //ROS_INFO("distance:%f", dist);

                if(dist < distance_threshold_)
                {
                    processRecordingLogic(current_wp_idx_, WP_EVENT_ARRIVED);
                    ROS_INFO("[mission] Reached waypoint %d (%.2f, %.2f, %.2f)",
                             current_wp_idx_ + 1,
                             target.pose.position.x,
                             target.pose.position.y,
                             target.pose.position.z);
                    reached_ = true;
                    reach_time_ = ros::Time::now();
                }
                
            }

            if (reached_ && (ros::Time::now() - reach_time_).toSec() >= wait_time_)
            {
                current_wp_idx_ += 1;
                current_wp_retry_times_ = 0;
                if(mission_cycle_)
                {
                    current_wp_idx_ %= waypoints_.size();
                }
                else
                {
                    if(current_wp_idx_ >= waypoints_.size())
                    {
                        ROS_INFO("[mission] All missions have been completed !");
                        mission_running_ = false;
                        return;
                    }
                }
                publishGoal();
                processRecordingLogic(current_wp_idx_, WP_EVENT_DISPATCH);
                reached_ = false;
            }
        }
    }

    void publishGoal()
    {
        if (current_wp_idx_ < waypoints_.size() && current_wp_idx_ >= 0)
        {
            waypoints_[current_wp_idx_].header.stamp = ros::Time::now();
            goal_pub_.publish(waypoints_[current_wp_idx_]);
            ROS_INFO("[mission] Published goal %d: (%.2f, %.2f, %.2f)",
                     current_wp_idx_ + 1,
                     waypoints_[current_wp_idx_].pose.position.x,
                     waypoints_[current_wp_idx_].pose.position.y,
                     waypoints_[current_wp_idx_].pose.position.z);
        }
        else
        {
            ROS_WARN("[mission] current_wp_idx_ out of range! Please check the code.");
        }
    }

    void publishStart()
    {
        current_pose_.header.stamp = ros::Time::now();
        start_pub_.publish(current_pose_);
        ROS_INFO("[mission] Published start pose: (%.2f, %.2f, %.2f)",
                 current_pose_.pose.position.x,
                 current_pose_.pose.position.y,
                 current_pose_.pose.position.z);
    }

    void sendRecordControl(bool start)
    {
        std_msgs::String msg;
        msg.data = start ? "start" : "stop";
        srv_video_pub.publish(msg);
    }

    void processRecordingLogic(int target_index, int event_type)
    {
        if (target_index < 0 || target_index >= waypoints_record_flags_.size()) return;

        bool current_needs_record = waypoints_record_flags_[target_index];
        
        // Event A: About to publish the target point
        if (event_type == WP_EVENT_DISPATCH) 
        {
            // Rule: If the current target point is marked with *, recording will begin at departure
            if (current_needs_record)
            {
                if (!is_recording_active_) 
                {
                    sendRecordControl(true);
                    is_recording_active_ = true;
                    ROS_INFO("[mission] Record Start: Dispatching to marked waypoint %d", target_index);
                }
            }
        }
        // Event B: The target point has been reached
        else if (event_type == WP_EVENT_ARRIVED) 
        {
            // Rule: At this point, we need to decide whether to stop or not
            if (current_needs_record)
            {
                int next_index = target_index + 1;
                bool next_exists_and_record = false;

                // Check if there is a next point and if the next point also needs to be recorded
                if (next_index < waypoints_record_flags_.size()) {
                    if (waypoints_record_flags_[next_index]) {
                        next_exists_and_record = true;
                    }
                }

                if (!next_exists_and_record)
                {
                    if (is_recording_active_) 
                    {
                        sendRecordControl(false);
                        is_recording_active_ = false;
                        ROS_INFO("[mission] Record Stop: Arrived at %d!", target_index);
                    }
                }
                else
                {
                    ROS_INFO("[mission] Record Continue: Arrived at %d!", target_index);
                }
            }
            else
            {
                // exception handling
                if (is_recording_active_) 
                {
                    sendRecordControl(false);
                    is_recording_active_ = false;
                    ROS_WARN("[mission] Video control abnormal exit, please check the code");
                }
            }
        }
    }

    void landingCallback()
    {
        ROS_INFO("[mission] UAV needs to land!");
        takeoff_done_ = false;
        mission_running_ = false;
    }

    bool takeoffCallback(uav_px4_ctrl::TakeoffNotify::Request &req,
                         uav_px4_ctrl::TakeoffNotify::Response &res)
    {
        if (req.takeoff_done)
        {
            ROS_INFO("[mission] UAV has taken off!");
            res.response = "Hello, world!";
            takeoff_done_ = true;
            publishStart();
        }
        return true;
    }

    bool planfailCallback(uav_px4_ctrl::TakeoffNotify::Request &req,
                          uav_px4_ctrl::TakeoffNotify::Response &res)
    {
        if (req.takeoff_done)
        {
            if(current_wp_retry_times_ >= 3)
            {
                ROS_INFO("[mission] Planner has failed! All retries have been used up!");
                return false;
            }
            ROS_INFO("[mission][%d/3] Planner has failed! Retrying...", ++current_wp_retry_times_);
            res.response = "Hello, world!";
            publishStart();
            publishGoal();
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

            geometry_msgs::Pose local_pose = matrixToPose(wp_local);
            geometry_msgs::PoseStamped local_wp;
            local_wp.header.frame_id = "map";
            local_wp.pose = local_pose;
            local_wp.pose.orientation.w = 1.0;
            local_waypoints_.push_back(local_wp);

            ROS_INFO("  [%zu] x=%.2f, y=%.2f, z=%.2f",
                     i + 1,
                     local_waypoints_[i].pose.position.x,
                     local_waypoints_[i].pose.position.y,
                     local_waypoints_[i].pose.position.z);
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
        mission_running_ = true;

        while (ros::ok())
        {
            ros::spinOnce();
            rate.sleep();
        }
    }

private:
    ros::Publisher start_pub_;
    ros::Publisher goal_pub_;
    ros::Subscriber pose_sub_;
    ros::Subscriber odom_sub_;
    ros::Subscriber trans_sub_;
    ros::ServiceServer srv_takeoff_;
    ros::ServiceServer srv_planfail_;
    ros::ServiceClient srv_landing_;
    ros::ServiceClient srv_accomplish_;
    ros::ServiceClient srv_abort_;
    ros::Publisher srv_video_pub;

    ros::Timer timeout_timer_;

    std::vector<geometry_msgs::PoseStamped> waypoints_;
    std::vector<geometry_msgs::PoseStamped> local_waypoints_;
    std::vector<bool> waypoints_record_flags_;
    std::vector<double> waypoints_wait_times_;

    geometry_msgs::PoseStamped current_pose_;
    std::string pose_topic_;
    std::string odom_topic_;
    std::string start_topic_;
    std::string goal_topic_;
    bool mission_cycle_ = false;
    bool mission_running_ = false;

    double distance_threshold_;
    double wait_time_;
    double start_delay_;
    double topic_timeout_;

    int current_wp_idx_ = -1;
    int current_wp_retry_times_ = 0;
    bool reached_ = true;
    bool use_odom_ = false;
    bool have_map_to_odom_ = false;
    bool takeoff_done_ = false;
    bool is_recording_active_ = false;

    nav_msgs::Odometry cur_map_to_odom;
    ros::Time reach_time_ = ros::Time(0);
    ros::Time last_pose_time_ = ros::Time(0);
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "mission_manger");
    ros::NodeHandle nh("~");

    WaypointPublisher wp_pub(nh);
    
    std::thread th(&WaypointPublisher::missionCheckWaypoint, &wp_pub);
    th.detach();
    
    wp_pub.spin();
    ros::shutdown();
    return 0;
}
