/**
 * @file mission_manger.cpp
 * @brief 任务管理器 — 航点序列调度、录像控制与规划器失败重试。
 *
 * @details
 * 面向全自主飞行任务的高层调度器，核心功能：
 *   1. 航点序列管理  — 正则解析含录像标志与等待时间的航点字符串
 *   2. 录像自动控制  — 根据航点标记 "*" 控制机载录像起停
 *   3. 起飞同步      — 通过 /takeoff_notify 服务等待 px4ctrl 起飞完成后启动任务
 *   4. 规划器重试    — 通过 /planner_fail_notify 服务接收规划失败并自动重试 (最多3次)
 *   5. 位姿超时检测  — topic_timeout 内无位姿更新则告警
 *   6. 循环任务      — 支持 mission_cycle 循环执行
 *   7. 坐标变换      — 支持 map→odom 变换以适配局部里程计系
 *
 * 航点字符串格式：
 *   "[[x,y,z,wait][*,][x,y,z,wait][*,]...]"
 *   - x, y, z: 目标坐标 [m]
 *   - wait:    可选第四个参数 — 到达后等待时间 [s]（0 = 无限等待）
 *   - *:       可选第五个参数 — 录像标记 (录像起/停控制)
 *
 * 话题：
 *   订阅 — odom (nav_msgs/Odometry) 或 pose (geometry_msgs/PoseStamped)
 *   发布 — start_pose, goal_pose (geometry_msgs/PoseStamped), /camera_recorder/record_control (std_msgs/String)
 *   服务 — /takeoff_notify, /planner_fail_notify (uav_px4_ctrl/TakeoffNotify)
 *
 * 使用：
 *   roslaunch uav_px4_ctrl mission_manger.launch
 */

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

std::mutex mutex_;  ///< 全局互斥锁，保护位姿数据线程安全

// ============================================================================
// 航点事件类型
// ============================================================================
/** @brief 航点事件类型 — 用于录像逻辑判定。 */
enum WaypointEventType {
    WP_EVENT_DISPATCH = 0, ///< 发布目标点（开始前往）
    WP_EVENT_ARRIVED = 1   ///< 到达目标点（结束一段）
};

// ============================================================================
// 坐标变换工具函数
// ============================================================================

/**
 * @brief geometry_msgs::Pose → Eigen::Matrix4d (齐次变换矩阵)。
 * @param pose ROS 位姿消息
 * @return 4×4 齐次变换矩阵
 */
Eigen::Matrix4d poseToMatrix(const geometry_msgs::Pose &pose)
{
    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
    Eigen::Quaterniond q(pose.orientation.w, pose.orientation.x,
                         pose.orientation.y, pose.orientation.z);
    Eigen::Matrix3d R = q.toRotationMatrix();      ///< 旋转矩阵 3×3
    T.block<3, 3>(0, 0) = R;
    T(0, 3) = pose.position.x;                    ///< 平移 X [m]
    T(1, 3) = pose.position.y;                    ///< 平移 Y [m]
    T(2, 3) = pose.position.z;                    ///< 平移 Z [m]
    return T;
}

/**
 * @brief Eigen::Matrix4d → geometry_msgs::Pose (位姿消息)。
 * @param T 4×4 齐次变换矩阵
 * @return ROS 位姿消息
 */
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

// ============================================================================
// WaypointPublisher — 任务管理器
// ============================================================================
class WaypointPublisher
{
public:
    /**
     * @brief 构造函数 — 加载参数、解析航点、注册话题/服务。
     * @param nh ROS 私有节点句柄
     */
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

        // 打印加载的航点列表
        ROS_INFO("[mission] Loaded %zu waypoints:", waypoints_.size());
        for (size_t i = 0; i < waypoints_.size(); ++i)
        {
            // 录像标志
            bool rec_flag = (i < waypoints_record_flags_.size()) ? waypoints_record_flags_[i] : false;
            std::string rec_str = rec_flag ? "[REC: ON ]" : "[REC: OFF]";

            // 等待时间字符串
            double wt = (i < waypoints_wait_times_.size()) ?
                        waypoints_wait_times_[i] : wait_time_;

            std::string wait_str;
            if (wt == std::numeric_limits<double>::max())
            {
                wait_str = "∞";  ///< wait_time=0 → 无限等待
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

        // odom 优先级高于 pose
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
        srv_planfail_ = nh.advertiseService("/planner_fail_notify", &WaypointPublisher::planfailCallback, this);
        srv_video_pub = nh.advertise<std_msgs::String>("/camera_recorder/record_control", 1);

        timeout_timer_ = nh.createTimer(ros::Duration(5.0), &WaypointPublisher::timeoutCheckCallback, this);
    }

    /**
     * @brief 正则解析航点字符串（支持 wait_time 和录像标记）。
     *
     * 格式: [[x,y,z[,wait]][*], [x,y,z[,wait]][*], ...]
     *   - x, y, z:  目标坐标 [m]
     *   - wait:     等待时间 [s]（0 = 无限等待）
     *   - *:        录像标记
     *
     * @param input 航点字符串
     */
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
                wp.pose.position.x = std::stod(match[1]);  ///< 目标 X [m]
                wp.pose.position.y = std::stod(match[2]);  ///< 目标 Y [m]
                wp.pose.position.z = std::stod(match[3]);  ///< 目标 Z [m]
                wp.pose.orientation.w = 1.0;
                waypoints_.push_back(wp);

                // wait time（非负数）
                double wait_t;
                if (match[4].str().empty())
                {
                    wait_t = wait_time_;  ///< 未指定 → 使用默认值
                }
                else
                {
                    wait_t = std::stod(match[4].str());
                    if (wait_t == 0.0)
                        wait_t = std::numeric_limits<double>::max();  ///< 0 → 无限等待
                    else if (wait_t < 0.0)
                        wait_t = wait_time_;  ///< 负值 → 使用默认
                }
                waypoints_wait_times_.push_back(wait_t);

                // record_flag ("*")
                bool record_flag = (match.size() > 5 && match[5].length() > 0 && match[5].str() == "*");
                waypoints_record_flags_.push_back(record_flag);
            }
            s = match.suffix();
        }
    }

    /** @brief 里程计回调（线程安全）。 */
    void odomCallback(const nav_msgs::Odometry::ConstPtr &msg)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        current_pose_.pose = msg->pose.pose;
        last_pose_time_ = msg->header.stamp;
    }

    /** @brief 本地位姿回调（线程安全）。 */
    void poseCallback(const geometry_msgs::PoseStamped::ConstPtr &msg)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        current_pose_ = *msg;
        last_pose_time_ = msg->header.stamp;
    }

    /** @brief 任务重置 — 停止当前任务。 */
    void missionReset()
    {
        std::lock_guard<std::mutex> lock(mutex_);
        mission_running_ = false;
    }

    /**
     * @brief 航点到达检测与切换（独立线程）。
     *
     * 流水线：
     *   1. 10 Hz 循环检测距离
     *   2. 距离 < distance_threshold [m] → 标记 reached_，触发录像停/启逻辑
     *   3. 等待 wait_time [s] → 切换到下一航点 → publishGoal()
     *   4. 支持 mission_cycle 循环
     */
    void missionCheckWaypoint()
    {
        ros::Rate rate(10);  ///< 10 Hz 检测频率
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
                double dist = std::sqrt(dx * dx + dy * dy + dz * dz);  ///< 欧氏距离 [m]

                // 到达判定
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

            // 等待结束后切换航点
            if (reached_ && (ros::Time::now() - reach_time_).toSec() >= wait_time_)
            {
                current_wp_idx_ += 1;
                current_wp_retry_times_ = 0;  ///< 重置规划器重试计数

                if(mission_cycle_)
                {
                    current_wp_idx_ %= waypoints_.size();  ///< 循环模式
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

    /** @brief 发布当前航点到 goal 话题。 */
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

    /** @brief 发布起始位姿到 start 话题（用于规划器初始化）。 */
    void publishStart()
    {
        current_pose_.header.stamp = ros::Time::now();
        start_pub_.publish(current_pose_);
        ROS_INFO("[mission] Published start pose: (%.2f, %.2f, %.2f)",
                 current_pose_.pose.position.x,
                 current_pose_.pose.position.y,
                 current_pose_.pose.position.z);
    }

    // ============================================================================
    // 录像控制
    // ============================================================================

    /**
     * @brief 发送录像起停指令到 /camera_recorder/record_control。
     * @param start true = 开始录像, false = 停止录像
     */
    void sendRecordControl(bool start)
    {
        std_msgs::String msg;
        msg.data = start ? "start" : "stop";
        srv_video_pub.publish(msg);
    }

    /**
     * @brief 录像逻辑处理 — 根据航点 "*" 标记和事件类型决定录像起停。
     *
     * 规则：
     *   - WP_EVENT_DISPATCH: 如果当前航点标记了 "*"，开始录像
     *   - WP_EVENT_ARRIVED:  如果当前航点标记了 "*" 且下一航点没有标记 "*"，停止录像
     *
     * @param target_index 当前航点索引
     * @param event_type   事件类型 (DISPATCH / ARRIVED)
     */
    void processRecordingLogic(int target_index, int event_type)
    {
        if (target_index < 0 || target_index >= waypoints_record_flags_.size()) return;

        bool current_needs_record = waypoints_record_flags_[target_index];

        // 事件 A: 即将发布目标点 — 出发时开始录像
        if (event_type == WP_EVENT_DISPATCH)
        {
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
        // 事件 B: 目标点已到达 — 决定是否停止录像
        else if (event_type == WP_EVENT_ARRIVED)
        {
            if (current_needs_record)
            {
                int next_index = target_index + 1;
                bool next_exists_and_record = false;

                // 检查下一个航点是否也需要录像
                if (next_index < waypoints_record_flags_.size()) {
                    if (waypoints_record_flags_[next_index]) {
                        next_exists_and_record = true;
                    }
                }

                // 下一航点不需要录像 → 停止
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
                // 异常处理：不该录像时录像中 → 强制停止
                if (is_recording_active_)
                {
                    sendRecordControl(false);
                    is_recording_active_ = false;
                    ROS_WARN("[mission] Video control abnormal exit, please check the code");
                }
            }
        }
    }

    /** @brief 降落回调 — 重置起飞标志并停止任务。 */
    void landingCallback()
    {
        ROS_INFO("[mission] UAV needs to land!");
        takeoff_done_ = false;
        mission_running_ = false;
    }

    // ============================================================================
    // 服务回调
    // ============================================================================

    /**
     * @brief 起飞通知服务回调 — px4ctrl 完成起飞后触发任务启动。
     * @param req 包含 takeoff_done 标志
     * @param res 服务响应
     * @return true
     */
    bool takeoffCallback(uav_px4_ctrl::TakeoffNotify::Request &req,
                         uav_px4_ctrl::TakeoffNotify::Response &res)
    {
        if (req.takeoff_done)
        {
            ROS_INFO("[mission] UAV has taken off!");
            res.response = "Hello, world!";
            takeoff_done_ = true;
            publishStart();  ///< 立刻发布起始位姿
        }
        return true;
    }

    /**
     * @brief 规划失败服务回调 — 规划器失败时自动重试 (最多3次)。
     * @param req 包含 takeoff_done 标志（复用消息类型）
     * @param res 服务响应
     * @return true (重试) / false (放弃)
     */
    bool planfailCallback(uav_px4_ctrl::TakeoffNotify::Request &req,
                          uav_px4_ctrl::TakeoffNotify::Response &res)
    {
        if (req.takeoff_done)
        {
            if(current_wp_retry_times_ >= 3)
            {
                ROS_INFO("[mission] Planner has failed! All retries have been used up!");
                return false;  ///< 放弃
            }
            ROS_INFO("[mission][%d/3] Planner has failed! Retrying...", ++current_wp_retry_times_);
            res.response = "Hello, world!";
            publishStart();   ///< 重新发布起始位姿
            publishGoal();    ///< 重新发布目标航点
        }
        return true;
    }

    /** @brief 定时检查位姿话题是否超时。 */
    void timeoutCheckCallback(const ros::TimerEvent &)
    {
        if (last_pose_time_.isZero())
        {
            ROS_WARN_THROTTLE(10, "[mission] No position message received yet...");
            return;
        }

        double dt = (ros::Time::now() - last_pose_time_).toSec();  ///< 距上次接收时间 [s]
        if (dt > topic_timeout_)
        {
            ROS_WARN("[mission] No position update! Check your topics.");
        }
    }

    /**
     * @brief map→odom 变换回调 — 将全局系航点转换到局部里程计系。
     *
     * 变换链: P_odom = T_odom_map × P_map
     * 仅在 takeoff_done_ 之前且未获取过变换时执行。
     */
    void map2odomCallback(const nav_msgs::Odometry::ConstPtr &msg)
    {
        if(have_map_to_odom_ || takeoff_done_) return;

        cur_map_to_odom = *msg;
        have_map_to_odom_ = true;

        ROS_INFO("[mission] Got map_to_odom transform!");
        ROS_INFO("[mission] transformed %zu waypoints:", waypoints_.size());

        Eigen::Matrix4d T_map_to_odom = poseToMatrix(cur_map_to_odom.pose.pose);
        Eigen::Matrix4d T_odom_to_map = T_map_to_odom.inverse();  ///< 逆变换

        for (size_t i = 0; i < waypoints_.size(); ++i)
        {
            Eigen::Matrix4d wp_global = poseToMatrix(waypoints_[i].pose);
            Eigen::Matrix4d wp_local = T_odom_to_map * wp_global;  ///< 全局 → 局部

            geometry_msgs::Pose local_pose = matrixToPose(wp_local);
            geometry_msgs::PoseStamped local_wp;
            local_wp.header.frame_id = "map";
            local_wp.pose = local_pose;
            local_wp.pose.orientation.w = 1.0;  ///< 重置四元数
            local_waypoints_.push_back(local_wp);

            ROS_INFO("  [%zu] x=%.2f, y=%.2f, z=%.2f",
                     i + 1,
                     local_waypoints_[i].pose.position.x,
                     local_waypoints_[i].pose.position.y,
                     local_waypoints_[i].pose.position.z);
        }
    }

    /**
     * @brief 主循环 — 等待起飞通知 → 延时启动 → 任务循环。
     *
     * 流水线：
     *   1. 阻塞等待 /takeoff_notify 服务
     *   2. 延时 start_delay [s] 后启动任务
     *   3. 进入 10 Hz spin 循环
     */
    void spin()
    {
        ros::Rate rate(10);  ///< 10 Hz

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
    // ---- 话题 ----
    ros::Publisher start_pub_;        ///< start_pose 发布器
    ros::Publisher goal_pub_;         ///< goal_pose 发布器
    ros::Subscriber pose_sub_;        ///< 本地位姿订阅器
    ros::Subscriber odom_sub_;        ///< 里程计订阅器
    ros::Subscriber trans_sub_;       ///< map→odom 变换订阅器
    ros::ServiceServer srv_takeoff_;  ///< 起飞通知服务
    ros::ServiceServer srv_planfail_; ///< 规划失败通知服务
    ros::ServiceClient srv_landing_;  ///< 降落服务客户端（预留）
    ros::ServiceClient srv_accomplish_; ///< 任务完成服务客户端（预留）
    ros::ServiceClient srv_abort_;    ///< 任务中止服务客户端（预留）
    ros::Publisher srv_video_pub;     ///< 录像控制发布器

    ros::Timer timeout_timer_;        ///< 位姿超时检测定时器 (5s)

    // ---- 航点数据 ----
    std::vector<geometry_msgs::PoseStamped> waypoints_;        ///< 航点列表（全局系）
    std::vector<geometry_msgs::PoseStamped> local_waypoints_;  ///< 航点列表（局部系）
    std::vector<bool> waypoints_record_flags_;                 ///< 录像标记标志
    std::vector<double> waypoints_wait_times_;                 ///< 等待时间 [s]

    // ---- 状态 ----
    geometry_msgs::PoseStamped current_pose_;  ///< 无人机当前位姿
    std::string pose_topic_;    ///< 位姿话题名
    std::string odom_topic_;    ///< 里程计话题名
    std::string start_topic_;   ///< start_pose 话题名
    std::string goal_topic_;    ///< goal_pose 话题名
    bool mission_cycle_ = false;    ///< 是否循环执行任务
    bool mission_running_ = false;  ///< 任务是否运行中

    double distance_threshold_;  ///< 到达距离阈值 [m]
    double wait_time_;           ///< 到达后默认等待时间 [s]
    double start_delay_;         ///< 启动延迟时间 [s]
    double topic_timeout_;       ///< 话题超时时间 [s]

    int current_wp_idx_ = -1;        ///< 当前航点索引
    int current_wp_retry_times_ = 0; ///< 当前航点规划器重试次数
    bool reached_ = true;            ///< 是否已到达当前航点
    bool use_odom_ = false;          ///< 是否使用 odom 话题
    bool have_map_to_odom_ = false;  ///< 是否已获取 map→odom 变换
    bool takeoff_done_ = false;      ///< 是否已完成起飞
    bool is_recording_active_ = false; ///< 当前是否录像中

    nav_msgs::Odometry cur_map_to_odom;  ///< map→odom 变换消息
    ros::Time reach_time_ = ros::Time(0);    ///< 到达当前航点的时间
    ros::Time last_pose_time_ = ros::Time(0); ///< 最近一次位姿消息时间
};

// ============================================================================
// 入口
// ============================================================================

int main(int argc, char **argv)
{
    ros::init(argc, argv, "mission_manger");
    ros::NodeHandle nh("~");

    WaypointPublisher wp_pub(nh);

    // 航点检测在独立线程中运行（避免阻塞服务回调）
    std::thread th(&WaypointPublisher::missionCheckWaypoint, &wp_pub);
    th.detach();

    wp_pub.spin();
    ros::shutdown();
    return 0;
}
