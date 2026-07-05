/**
 * @file pub_waypoint.cpp
 * @brief 航点序列发布器 — 按顺序循环发布 waypoints 到全局规划器。
 *
 * @details
 * 从 ROS 参数加载一组航点 [x, y, z] 列表，订阅里程计/位姿话题，
 * 当无人机到达当前航点后自动切换到下一个航点并发布新 goal。
 * 核心功能：
 *   1. 正则表达式解析 waypoint 字符串  — 格式 "[[x,y,z],[x,y,z],...]"
 *   2. 到达判定  — 基于欧氏距离阈值 `distance_threshold` [m]
 *   3. 等待计时  — 到达后停留 `wait_time` [s] 再出发
 *   4. 话题超时检测  — 位姿数据断流告警
 *   5. 话题切换  — 支持 odom (nav_msgs/Odometry) 或 pose (geometry_msgs/PoseStamped)
 *
 * 话题：
 *   订阅 — 里程计或本地位姿（配置 topic）
 *   发布 — goal_pose（geometry_msgs/PoseStamped）
 *
 * 使用：
 *   roslaunch uav_px4_ctrl goal_publisher.launch
 */

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <vector>
#include <string>
#include <cmath>
#include <regex>

// ============================================================================
// WaypointPublisher — 航点序列管理器
// ============================================================================
class WaypointPublisher
{
public:
    /**
     * @brief 构造函数 — 加载参数、解析航点、注册话题。
     * @param nh ROS 私有节点句柄
     */
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

        // odom 话题优先级高于 pose 话题
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

        timeout_timer_ = nh.createTimer(ros::Duration(5.0), &WaypointPublisher::timeoutCheckCallback, this);

        current_wp_idx_ = 0;
        reached_ = false;
        last_pose_time_ = ros::Time(0);
    }

    /**
     * @brief 正则解析航点字符串。
     * @param input 格式 "[[x,y,z],[x,y,z],...]" 的字符串
     */
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
                wp.pose.position.x = std::stod(match[1]);  ///< X 坐标 [m]
                wp.pose.position.y = std::stod(match[2]);  ///< Y 坐标 [m]
                wp.pose.position.z = std::stod(match[3]);  ///< Z 坐标 [m]
                wp.pose.orientation.w = 1.0;               ///< 默认单位四元数
                waypoints_.push_back(wp);
            }
            s = match.suffix();
        }
    }

    /** @brief 里程计回调（优先级高于 pose）。 */
    void odomCallback(const nav_msgs::Odometry::ConstPtr &msg)
    {
        current_pose_.pose = msg->pose.pose;
        last_pose_time_ = msg->header.stamp;
        checkAndSwitchWaypoint();
    }

    /** @brief 本地位姿回调。 */
    void poseCallback(const geometry_msgs::PoseStamped::ConstPtr &msg)
    {
        current_pose_ = *msg;
        last_pose_time_ = msg->header.stamp;
        checkAndSwitchWaypoint();
    }

    /**
     * @brief 到达检测与航点切换。
     *
     * 1. 计算当前位姿与目标航点的欧氏距离
     * 2. 距离 < distance_threshold → 标记 reached_，记录到达时间
     * 3. 等待 wait_time 秒后自动切换到下一航点并发布
     */
    void checkAndSwitchWaypoint()
    {
        if (waypoints_.empty())
            return;

        geometry_msgs::PoseStamped target = waypoints_[current_wp_idx_];
        double dx = target.pose.position.x - current_pose_.pose.position.x;
        double dy = target.pose.position.y - current_pose_.pose.position.y;
        double dz = target.pose.position.z - current_pose_.pose.position.z;
        double dist = std::sqrt(dx * dx + dy * dy + dz * dz);  ///< 欧氏距离 [m]

        // 到达判定
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

        // 等待结束后循环切换到下一航点
        if (reached_ && (ros::Time::now() - reach_time_).toSec() >= wait_time_)
        {
            current_wp_idx_ = (current_wp_idx_ + 1) % waypoints_.size();
            publishGoal();
            reached_ = false;
        }
    }

    /** @brief 发布当前航点 goal。 */
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

    /** @brief 定时检查位姿话题是否超时断流。 */
    void timeoutCheckCallback(const ros::TimerEvent &)
    {
        if (last_pose_time_.isZero())
        {
            ROS_WARN_THROTTLE(10, "No position message received yet...");
            return;
        }

        double dt = (ros::Time::now() - last_pose_time_).toSec();  ///< 距上次接收时间 [s]
        if (dt > topic_timeout_)
        {
            ROS_WARN("No position update! Check your topics.");
        }
    }

    /** @brief 主循环：延时启动后发布首目标并进入 spin。 */
    void spin()
    {
        ros::Rate rate(10);  ///< 10 Hz 主循环
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
    ros::Publisher goal_pub_;      ///< goal 发布器
    ros::Subscriber pose_sub_;     ///< 位姿订阅器
    ros::Subscriber odom_sub_;     ///< 里程计订阅器
    ros::Timer timeout_timer_;     ///< 话题超时检测定时器

    std::vector<geometry_msgs::PoseStamped> waypoints_;  ///< 航点列表
    geometry_msgs::PoseStamped current_pose_;            ///< 当前位姿
    std::string pose_topic_;    ///< 位姿话题名
    std::string odom_topic_;    ///< 里程计话题名
    std::string goal_topic_;    ///< goal 话题名

    double distance_threshold_;  ///< 到达距离阈值 [m]
    double wait_time_;           ///< 到达后等待时间 [s]
    double start_delay_;         ///< 启动延迟时间 [s]
    double topic_timeout_;       ///< 话题超时时间 [s]

    int current_wp_idx_;  ///< 当前航点索引
    bool reached_;        ///< 是否已到达当前航点
    bool use_odom_;       ///< 是否使用 odom 话题（vs pose）

    ros::Time reach_time_;      ///< 到达当前航点的时间
    ros::Time last_pose_time_;  ///< 最近一次位姿消息时间
};

// ============================================================================
// 入口
// ============================================================================

int main(int argc, char **argv)
{
    ros::init(argc, argv, "waypoint_publisher");
    ros::NodeHandle nh("~");

    WaypointPublisher wp_pub(nh);
    wp_pub.spin();
    return 0;
}
