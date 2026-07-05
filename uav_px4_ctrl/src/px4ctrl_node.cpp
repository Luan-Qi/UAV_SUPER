/**
 * @file px4ctrl_node.cpp
 * @brief PX4 无人机主控制节点 — MAVROS offboard 控制、坐标系转换与安全边界。
 *
 * @details
 * 作为 PX4 飞控与外部位姿/轨迹指令之间的中间层，核心功能：
 *   1. 坐标系修正  — Vision ENU → PX4 NED 的坐标旋转（uav_mavros_pose_fix / uav_mavros_cmd_vel_fix）
 *   2. 多模式控制  — 支持 position / velocity / attitude 三种 offboard 控制模式
 *   3. 安全门限   — 位姿/速度/加速度/Jerk 限幅、电池低电压保护、RC 有效性检查
 *   4. 状态机管理 — arm → takeoff → hold/command → land 全流程状态切换
 *   5. 遥控器集成 — RC 通道解析 arm/mode/gear 开关状态
 *
 * 坐标系约定：
 *   - 输入 Odom:  里程计系 (ENU: X-东 Y-北 Z-上)
 *   - 输出 PX4:   经 uav_mavros_pose_fix() 旋转 +90° 绕 Z 后发布到 vision_pose
 *   - 指令目标:   外部规划器给出的目标位姿/速度（与 odom 同系）
 *
 * 话题：
 *   订阅 — /mavros/state, /mavros/battery, /mavros/rc/in, /position_cmd, <odom_topic>
 *   发布 — /mavros/vision_pose/pose, /mavros/setpoint_velocity/cmd_vel,
 *          /mavros/setpoint_position/local, /traj_start_trigger
 *   服务 — /mavros/set_mode, /mavros/cmd/arming, /mavros/cmd/command, /takeoff_notify
 *
 * 使用：
 *   roslaunch uav_px4_ctrl px4ctrl.launch
 *   rosrun uav_px4_ctrl px4ctrl_node
 */

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <mavros_msgs/RCIn.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/BatteryStatus.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/CommandLong.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/ExtendedState.h>
#include <sensor_msgs/BatteryState.h>
#include <nav_msgs/Odometry.h>
#include <quadrotor_msgs/PositionCommand.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <signal.h>
#include "px4ctrl/subscribe.h"
#include "uav_px4_ctrl/TakeoffNotify.h"

// ============================================================================
// 坐标系修正函数 — Vision ENU ↔ PX4 NED
// ============================================================================

/**
 * @brief 里程计位姿坐标系修正 — 绕 Z 轴旋转 +90° (ENU → 飞控期望方向)。
 *
 * 转换公式：
 *   x' = -y
 *   y' =  x
 *   q' = q_rot(+90°绕Z) × q_orig
 *
 * @param pose [in/out] 待修正的位姿
 */
void uav_mavros_pose_fix(geometry_msgs::PoseStamped * pose)
{
    double temp_x = pose->pose.position.x;
    double temp_y = pose->pose.position.y;

    pose->pose.position.x = -temp_y;  ///< X 轴旋转: x' = -y
    pose->pose.position.y = temp_x;   ///< Y 轴旋转: y' =  x

    tf2::Quaternion q_orig, q_rot, q_new;
    tf2::fromMsg(pose->pose.orientation, q_orig);
    q_rot.setRPY(0, 0, M_PI/2);       // 绕Z轴旋转+90°
    q_new = q_rot * q_orig;           // 旋转叠加
    q_new.normalize();
    pose->pose.orientation = tf2::toMsg(q_new);
}

/**
 * @brief 速度指令坐标系修正 — 绕 Z 轴旋转 +90°。
 *
 * 转换公式：
 *   vx' = -vy
 *   vy' =  vx
 *
 * @param vel [in/out] 待修正的速度指令
 */
void uav_mavros_cmd_vel_fix(geometry_msgs::TwistStamped * vel)
{
    double temp_x = vel->twist.linear.x;
    double temp_y = vel->twist.linear.y;

    vel->twist.linear.x = -temp_y;   ///< X 轴旋转: vx' = -vy
    vel->twist.linear.y = temp_x;    ///< Y 轴旋转: vy' =  vx
}

/**
 * @brief 偏航角 → 四元数（仅绕 Z 轴旋转）。
 * @param yaw_rad 偏航角 [rad]
 * @return geometry_msgs::Quaternion
 */
geometry_msgs::Quaternion yawToQuaternion(double yaw_rad) {
    tf2::Quaternion q;
    q.setRPY(0, 0, yaw_rad);
    return tf2::toMsg(q);
}

// ============================================================================
// DroneCtrl — 无人机主控制类
// ============================================================================
class DroneCtrl
{
public:
    ros::NodeHandle nh;

    // ---- 订阅器 ----
    ros::Subscriber state_sub;     ///< /mavros/state — PX4 状态
    ros::Subscriber battery_sub;   ///< /mavros/battery — 电池状态
    ros::Subscriber cmd_sub;       ///< /position_cmd — 轨迹指令
    ros::Subscriber rc_sub;        ///< /mavros/rc/in — 遥控器输入
    ros::Subscriber odom_sub;      ///< <odom_topic> — 里程计

    // ---- 发布器 ----
    ros::Publisher pose_pub;               ///< /mavros/vision_pose/pose — 视觉位姿 (PX4 vision 输入)
    ros::Publisher vel_pub;                ///< /mavros/setpoint_velocity/cmd_vel — 速度指令
    ros::Publisher local_pos_pub;          ///< /mavros/setpoint_position/local — 位置指令
    ros::Publisher traj_start_trigger_pub; ///< /traj_start_trigger — 轨迹启动触发

    // ---- 服务客户端 ----
    ros::ServiceClient set_FCU_mode_srv;   ///< /mavros/set_mode — 设置飞控模式
    ros::ServiceClient arming_client_srv;  ///< /mavros/cmd/arming — 解锁/上锁
    ros::ServiceClient reboot_FCU_srv;     ///< /mavros/cmd/command — 飞控指令
    ros::ServiceClient takeoff_client_src; ///< /takeoff_notify — 起飞通知

    // ---- 数据缓存 ----
    RC_Data_t rc_data;            ///< 遥控器数据
    State_Data_t state_data;      ///< 飞控状态
    Battery_Data_t battery_data;  ///< 电池数据
    Command_Data_t cmd_data;      ///< 轨迹指令
    Odom_Data_t odom_data;        ///< 里程计数据

    /**
     * @brief 构造函数 — 加载参数、注册话题与服务、设置控制模式。
     * @note 使用私有命名空间 "~" 加载参数。
     */
    DroneCtrl() : nh("~"){
        nh.param<double>("/takeoff_height", takeoff_height, takeoff_height);
        nh.param<std::string>("control_mode", control_mode, "position");
        nh.param<double>("battery_limit", battery_limit, battery_limit);
        nh.param<double>("position_max_x", position_max_x, position_max_x);
        nh.param<double>("position_min_x", position_min_x, position_min_x);
        nh.param<double>("position_max_y", position_max_y, position_max_y);
        nh.param<double>("position_min_y", position_min_y, position_min_y);
        nh.param<double>("position_max_z", position_max_z, position_max_z);
        nh.param<double>("position_min_z", position_min_z, position_min_z);
        nh.param<double>("position_max_vel", position_max_vel, position_max_vel);
        nh.param<double>("position_max_acc", position_max_acc, position_max_acc);
        nh.param<double>("cmd_vel_max_x", cmd_vel_max_x, cmd_vel_max_x);
        nh.param<double>("cmd_vel_max_y", cmd_vel_max_y, cmd_vel_max_y);
        nh.param<double>("cmd_vel_max_z", cmd_vel_max_z, cmd_vel_max_z);
        nh.param<double>("cmd_vel_max_acc", cmd_vel_max_acc, cmd_vel_max_acc);
        nh.param<double>("cmd_vel_max_jerk", cmd_vel_max_jerk, cmd_vel_max_jerk);
        nh.param<std::string>("odom_topic", odom_topic, std::string("/Odometry"));

        // ---- 订阅 ----
        state_sub = nh.subscribe<mavros_msgs::State>(
            "/mavros/state", 10, boost::bind(&State_Data_t::feed, &state_data, _1));

        battery_sub = nh.subscribe<sensor_msgs::BatteryState>(
            "/mavros/battery", 10, boost::bind(&Battery_Data_t::feed, &battery_data, _1));

        cmd_sub = nh.subscribe<quadrotor_msgs::PositionCommand>(
            "/position_cmd", 100, boost::bind(&Command_Data_t::feed, &cmd_data, _1),
            ros::VoidConstPtr(), ros::TransportHints().tcpNoDelay());  ///< TCP no-delay 降低指令延迟

        rc_sub = nh.subscribe<mavros_msgs::RCIn>(
            "/mavros/rc/in", 10, boost::bind(&RC_Data_t::feed, &rc_data, _1));

        odom_sub = nh.subscribe(odom_topic, 10, &DroneCtrl::odometryCallback, this);

        // ---- 发布 ----
        pose_pub = nh.advertise<geometry_msgs::PoseStamped>("/mavros/vision_pose/pose", 10);
        vel_pub = nh.advertise<geometry_msgs::TwistStamped>("/mavros/setpoint_velocity/cmd_vel", 10);
        local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>("/mavros/setpoint_position/local", 10);
        traj_start_trigger_pub = nh.advertise<geometry_msgs::PoseStamped>("/traj_start_trigger", 10);

        // ---- 服务 ----
        set_FCU_mode_srv = nh.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");
        arming_client_srv = nh.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");
        reboot_FCU_srv = nh.serviceClient<mavros_msgs::CommandLong>("/mavros/cmd/command");
        takeoff_client_src = nh.serviceClient<uav_px4_ctrl::TakeoffNotify>("/takeoff_notify");

        // ---- 控制模式编码 ----
        if(control_mode == "position")
            control_status = 0;     ///< 0: 位置控制
        else if(control_mode == "velocity")
            control_status = 1;     ///< 1: 速度控制
        else if(control_mode == "attitude")
            control_status = 2;     ///< 2: 姿态控制
        else
            ROS_ERROR("[PX4CTRL] Invalid control mode, default to position control mode.");
    }

    // ---- 数据新鲜度检查 ----
    bool rc_is_received(const ros::Time &now_time){return rc_data.is_received(now_time);}
    bool state_is_received(const ros::Time &now_time){return state_data.is_received(now_time);}
    bool battery_is_received(const ros::Time &now_time){return battery_data.is_received(now_time);}
    bool odom_is_received(const ros::Time &now_time){return odom_data.is_received(now_time);}
    bool cmd_is_received(const ros::Time &now_time){return cmd_data.is_received(now_time);}

    // ---- 控制接口 ----
    void init_target_takeoff();
    void update_target(const geometry_msgs::PoseStamped& target);
    void update_target(const nav_msgs::Odometry& target);
    void update_target(const quadrotor_msgs::PositionCommand& target);
    void update_cmd_vel(const quadrotor_msgs::PositionCommand& msg);
    void clear_cmd_vel();

    /** @brief 主状态机：arm → takeoff → hold/command → land */
    void process();

    void publish_target();
    void publish_cmd_vel();

    /** @brief 里程计回调 — 坐标系修正后转发到 PX4 vision_pose。 */
    void odometryCallback(const nav_msgs::Odometry::ConstPtr& msg);

private:
    // ---- 状态标志 ----
    bool is_armed = false;       ///< 是否已解锁
    bool is_takeoff = false;     ///< 是否已完成起飞
    bool is_land = true;         ///< 是否在地面
    bool is_landing = false;     ///< 是否正在降落
    bool enter_hold = false;     ///< 是否进入悬停
    bool enter_land = false;     ///< 是否进入降落
    bool check_battery = false;  ///< 电池电压是否正常
    bool check_rc_stick = true;  ///< 是否检查摇杆回中

    // ---- 时间戳 ----
    ros::Time last_arm_request;   ///< 上次请求解锁时间
    ros::Time last_target_pub;    ///< 上次发布目标位姿时间
    ros::Time last_cmd_vel_pub;   ///< 上次发布速度指令时间

    // ---- 目标位姿/速度 ----
    geometry_msgs::PoseStamped desired_target;   ///< 期望目标位姿（坐标系修正前）
    geometry_msgs::PoseStamped last_target;      ///< 上一帧发布的目标位姿
    geometry_msgs::TwistStamped desired_cmd_vel; ///< 期望速度指令
    geometry_msgs::TwistStamped last_cmd_vel;    ///< 上一帧发布的速度指令

    // ---- 控制参数 ----
    int control_status = 0;          ///< 0:位置 1:速度 2:姿态
    std::string control_mode;        ///< 控制模式字符串
    std::string odom_topic = "/Odometry";  ///< 里程计话题名
    double takeoff_height = 0.5;     ///< 起飞高度 [m]
    double battery_limit = 19.8;     ///< 电池低电压阈值 [V]

    // ---- 位置限幅 ----
    double position_max_x = 0.0;     ///< 最大 X 位置 [m]
    double position_min_x = 0.0;     ///< 最小 X 位置 [m]
    double position_max_y = 0.0;     ///< 最大 Y 位置 [m]
    double position_min_y = 0.0;     ///< 最小 Y 位置 [m]
    double position_max_z = 0.0;     ///< 最大 Z 位置 [m]
    double position_min_z = 0.0;     ///< 最小 Z 位置 [m]
    double position_max_vel = 0.0;   ///< 最大位置变化速率 [m/s]
    double position_max_acc = 0.0;   ///< 最大位置变化加速度 [m/s²]

    // ---- 速度限幅 ----
    double cmd_vel_max_x = 0.0;      ///< 最大 X 速度 [m/s]
    double cmd_vel_max_y = 0.0;      ///< 最大 Y 速度 [m/s]
    double cmd_vel_max_z = 0.0;      ///< 最大 Z 速度 [m/s]
    double cmd_vel_max_acc = 0.0;    ///< 最大加速度 [m/s²]
    double cmd_vel_max_jerk = 0.0;   ///< 最大 jerk [m/s³]
};

// ============================================================================
// DroneCtrl — 里程计回调
// ============================================================================

void DroneCtrl::odometryCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
    odom_data.feed(msg);

    // 组装 PoseStamped → 坐标系修正 → 发布到 PX4 vision_pose
    geometry_msgs::PoseStamped pose_msg;
    pose_msg.header = msg->header;      // 保留时间戳和 frame_id
    pose_msg.header.frame_id = "odom";
    pose_msg.pose = msg->pose.pose;     // 直接复制位姿

    uav_mavros_pose_fix(&pose_msg);     // 坐标系修正: ENU → PX4期望方向

    pose_pub.publish(pose_msg);
}

// ============================================================================
// DroneCtrl — 目标初始化
// ============================================================================

void DroneCtrl::init_target_takeoff()
{
    desired_target.pose.position.z = takeoff_height;  ///< 起飞目标 Z = takeoff_height [m]
}

// ============================================================================
// DroneCtrl — 目标更新（三重重载）
// ============================================================================

void DroneCtrl::update_target(const geometry_msgs::PoseStamped& target)
{
    desired_target.pose.position.x = target.pose.position.x;
    desired_target.pose.position.y = target.pose.position.y;
    desired_target.pose.position.z = target.pose.position.z;
    desired_target.pose.orientation.x = target.pose.orientation.x;
    desired_target.pose.orientation.y = target.pose.orientation.y;
    desired_target.pose.orientation.z = target.pose.orientation.z;
    desired_target.pose.orientation.w = target.pose.orientation.w;
    uav_mavros_pose_fix(&desired_target);  ///< 坐标系修正
}

void DroneCtrl::update_target(const nav_msgs::Odometry& target)
{
    desired_target.pose.position.x = target.pose.pose.position.x;
    desired_target.pose.position.y = target.pose.pose.position.y;
    desired_target.pose.position.z = target.pose.pose.position.z;
    desired_target.pose.orientation.x = target.pose.pose.orientation.x;
    desired_target.pose.orientation.y = target.pose.pose.orientation.y;
    desired_target.pose.orientation.z = target.pose.pose.orientation.z;
    desired_target.pose.orientation.w = target.pose.pose.orientation.w;
    uav_mavros_pose_fix(&desired_target);  ///< 坐标系修正
}

void DroneCtrl::update_target(const quadrotor_msgs::PositionCommand& target)
{
    desired_target.pose.position.x = target.position.x;
    desired_target.pose.position.y = target.position.y;
    desired_target.pose.position.z = target.position.z;
    desired_target.pose.orientation = yawToQuaternion((double)target.yaw);  ///< 偏航 → 四元数
    uav_mavros_pose_fix(&desired_target);  ///< 坐标系修正
}

// ============================================================================
// DroneCtrl — 速度指令
// ============================================================================

void DroneCtrl::update_cmd_vel(const quadrotor_msgs::PositionCommand& msg)
{
    desired_cmd_vel.header.stamp = ros::Time::now();
    desired_cmd_vel.twist.linear.x = msg.velocity.x;    ///< 线速度 X [m/s]
    desired_cmd_vel.twist.linear.y = msg.velocity.y;    ///< 线速度 Y [m/s]
    desired_cmd_vel.twist.linear.z = msg.velocity.z;    ///< 线速度 Z [m/s]
    uav_mavros_cmd_vel_fix(&desired_cmd_vel);           ///< 坐标系修正
}

void DroneCtrl::clear_cmd_vel()
{
    desired_cmd_vel.header.stamp = ros::Time::now();
    desired_cmd_vel.twist.linear.x = 0;   ///< 归零 X
    desired_cmd_vel.twist.linear.y = 0;   ///< 归零 Y
    desired_cmd_vel.twist.linear.z = 0;   ///< 归零 Z
    desired_cmd_vel.twist.angular.x = 0.0;
    desired_cmd_vel.twist.angular.y = 0.0;
    desired_cmd_vel.twist.angular.z = 0.0;
}

// ============================================================================
// DroneCtrl — 主状态机
// ============================================================================

void DroneCtrl::process()
{
    // 主状态机流水线：
    // 1. Arm 解锁 — RC arm + offboard + 电池正常 → 请求解锁
    // 2. 悬停/Hold — hold 模式或未起飞/手动hold: 设定当前位姿为 target, 等待起飞
    // 3. 指令执行 — command 模式: 根据 control_status 切换到位置/速度/姿态控制
    // 4. 安全保护 — 电池低电压告警

    geometry_msgs::PoseStamped target;
    static bool have_hold_set = false;

    // ---- 1. Arm 解锁逻辑 ----
    if(rc_data.is_armed && rc_data.is_offboard_mode &&!is_armed && check_battery && check_rc_stick)
    {
        if((ros::Time::now() - last_arm_request).toSec() > 3.0)  ///< 至少间隔 3s 重试
        {
            mavros_msgs::CommandBool arm_cmd;
            arm_cmd.request.value = true;

            if (arming_client_srv.call(arm_cmd) &&
                arm_cmd.response.success)
            {
                is_armed = true;
            }
            else
            {
                ROS_ERROR("[PX4CTRL] Vehicle while failed armimg, please check your system and restart!");
            }
            last_arm_request = ros::Time::now();
        }
    }
    else
    {
        if(!state_data.current_state.armed)
        {
            is_armed = false;
            is_landing = false;
        }
    }

    // ---- 2. Offboard 模式下的控制分支 ----
    if(rc_data.is_offboard_mode && !is_landing)
    {
        // ---- 2a. 悬停/未起飞/手动Hold ----
        if(rc_data.is_hold_mode || (rc_data.is_command_mode && !is_takeoff) || (rc_data.is_command_mode && enter_hold))
        {
            // 首次进入 hold: 锁定当前位姿
            if(!have_hold_set)
            {
                update_target(odom_data.msg);
                have_hold_set = true;
            }

            // 初始起飞: 设定 Z = takeoff_height
            if(is_land && !is_takeoff)
            {
                init_target_takeoff();
                is_land = false;
            }

            // 高度达标后通知起飞完成
            if(odom_data.msg.pose.pose.position.z > takeoff_height - 0.1 && !is_takeoff)
            {
                uav_px4_ctrl::TakeoffNotify srv;
                srv.request.takeoff_done = true;

                if(!takeoff_client_src.call(srv))
                    enter_hold = true;   ///< 通知失败 → 保持 hold
                is_takeoff = true;
            }
            publish_target();
        }
        // ---- 2b. 指令模式 ----
        else
        {
            if(rc_data.is_command_mode)
            {
                switch(control_status)
                {
                    case 0:  // 位置控制
                        if(cmd_is_received(ros::Time::now())) update_target(cmd_data.msg);
                        publish_target();
                        break;
                    case 1:  // 速度控制
                        if(cmd_is_received(ros::Time::now())){update_cmd_vel(cmd_data.msg);}
                        else{clear_cmd_vel();}
                        publish_cmd_vel();
                        break;
                    case 2:  // 姿态控制（未实现）
                        ROS_ERROR("[PX4CTRL] Attitude control not implemented yet!");
                        break;
                    default:
                        ROS_ERROR("[PX4CTRL] Invalid control mode, please check your code !!!!!!!!!!!!.");
                        break;
                }
            }
            // ---- 2c. 非 offboard 非 command → 降落 ----
            else
            {
                if(!is_landing) is_land = true;
                if(!is_land && !is_takeoff) is_landing = true;
                is_takeoff = false;
                enter_hold = false;
            }
            have_hold_set = false;
        }
    }
    // ---- 3. 非 Offboard 模式 — 重置所有飞行状态 ----
    else
    {
        is_takeoff = false;
        is_land = true;
        enter_hold = false;
        enter_land = false;
        have_hold_set = false;
    }

    // ---- 4. 电池低电压检测 ----
    if(battery_data.battery.voltage < battery_limit)
    {
        check_battery = false;
        ROS_WARN_THROTTLE(5.0, "[PX4CTRL] Battery voltage low: %.2f V < %.2f V", battery_data.battery.voltage, battery_limit);
    }
    else
    {
        check_battery = true;
    }
}

// ============================================================================
// DroneCtrl — 位置指令发布（含限幅和安全检查）
// ============================================================================

void DroneCtrl::publish_target()
{
    // 流水线：
    // 1. 拷贝目标
    // 2. 位置限幅 clamp（含坐标系旋转映射: x↔y 带符号翻转）
    // 3. 安全检查（默认禁用）— 速度超限 → 进入 hold
    // 4. 发布到 /mavros/setpoint_position/local

    geometry_msgs::PoseStamped target = desired_target;

    /** @brief 值域限幅函数。 */
    auto clamp = [](double value, double min_val, double max_val) {
        if (value < min_val)
            return min_val;
        else if (value > max_val)
            return max_val;
        else
            return value;
    };

    // 位置限幅（已考虑坐标系旋转后的方向映射）
    target.pose.position.x = clamp(target.pose.position.x, -position_max_y, -position_min_y);
    target.pose.position.y = clamp(target.pose.position.y,  position_min_x,  position_max_x);
    target.pose.position.z = clamp(target.pose.position.z,  position_min_z,  position_max_z);

    // 速度超限检查（默认禁用，由 if(false) 控制）
    if(false)
    {
        ros::Time now = ros::Time::now();
        double dt = (now - last_target_pub).toSec();

        if (dt > 0.001) // 避免除零
        {
            double cur_x = last_target.pose.position.x;
            double cur_y = last_target.pose.position.y;
            double cur_z = last_target.pose.position.z;

            double dx = target.pose.position.x - cur_x;
            double dy = target.pose.position.y - cur_y;
            double dz = target.pose.position.z - cur_z;

            double vel = sqrt(dx * dx + dy * dy + dz * dz) / dt;  ///< 等效速度 [m/s]

            if (vel > position_max_vel)
            {
                ROS_WARN("[PX4CTRL] Velocity limit exceeded: %.2f m/s > %.2f m/s", vel, position_max_vel);
                enter_hold = true;
                return;
            }
        }
        else
        {
            ROS_WARN("[PX4CTRL] Invalid time interval dt=%.6f", dt);
            return;
        }
    }

    last_target = target;
    last_target_pub = ros::Time::now();
    local_pos_pub.publish(target);
}

// ============================================================================
// DroneCtrl — 速度指令发布（含限幅和安全检查）
// ============================================================================

void DroneCtrl::publish_cmd_vel()
{
    // 流水线：
    // 1. 拷贝速度指令
    // 2. 逐轴限幅 clamp（含坐标系旋转映射: x↔y）
    // 3. 加速度检查（默认禁用）
    // 4. 发布到 /mavros/setpoint_velocity/cmd_vel

    geometry_msgs::TwistStamped cmd_vel = desired_cmd_vel;

    /** @brief 对称值域限幅函数。 */
    auto clamp = [](double value, double max_val) {
        if (value > max_val){
            return max_val;
            ROS_WARN_THROTTLE(5.0, "[PX4CTRL] Velocity limit exceeded: %.2f > %.2f", value, max_val);}
        else if (value < -max_val){
            return -max_val;
            ROS_WARN_THROTTLE(5.0, "[PX4CTRL] Velocity limit exceeded: %.2f < %.2f", value, -max_val);}
        else{
            return value;}
    };

    // 速度限幅（已考虑坐标系旋转后的方向映射）
    cmd_vel.twist.linear.x = clamp(cmd_vel.twist.linear.x,  cmd_vel_max_y);
    cmd_vel.twist.linear.y = clamp(cmd_vel.twist.linear.y,  cmd_vel_max_x);
    cmd_vel.twist.linear.z = clamp(cmd_vel.twist.linear.z,  cmd_vel_max_z);

    // 加速度超限检查（默认禁用）
    if(false)
    {
        ros::Time now = ros::Time::now();
        double dt = (now - last_cmd_vel_pub).toSec();

        if (dt > 0.001)  // 避免除零
        {
            double vx = cmd_vel.twist.linear.x;
            double vy = cmd_vel.twist.linear.y;
            double vz = cmd_vel.twist.linear.z;

            double last_vx = last_cmd_vel.twist.linear.x;
            double last_vy = last_cmd_vel.twist.linear.y;
            double last_vz = last_cmd_vel.twist.linear.z;

            double ax = (vx - last_vx) / dt;  ///< 加速度 X [m/s²]
            double ay = (vy - last_vy) / dt;  ///< 加速度 Y [m/s²]
            double az = (vz - last_vz) / dt;  ///< 加速度 Z [m/s²]

            // 加速度模长
            double acc = sqrt(ax * ax + ay * ay + az * az);

            if (acc > cmd_vel_max_acc)
            {
                ROS_WARN("[PX4CTRL] Acc limit exceeded: %.2f m/s^2 > %.2f m/s^2",
                        acc, cmd_vel_max_acc);
                enter_hold = true;
                return;
            }
        }
        else
        {
            ROS_WARN("[PX4CTRL] Invalid time interval dt=%.6f", dt);
            return;
        }
    }

    last_cmd_vel = cmd_vel;
    last_cmd_vel_pub = ros::Time::now();
    vel_pub.publish(cmd_vel);
}

// ============================================================================
// 信号处理与入口
// ============================================================================

void mySigintHandler(int sig)
{
    ROS_INFO("[PX4Ctrl] exit...");
    ros::shutdown();
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "px4ctrl");
    signal(SIGINT, mySigintHandler);  ///< Ctrl+C 优雅退出

    DroneCtrl node;

    ros::Duration(1).sleep();  ///< 等待各话题建立连接

    // ---- 1/3: 等待 PX4 连接 ----
    int trials = 0;
    while (ros::ok() && !node.state_data.current_state.connected)
    {
        ros::spinOnce();
        ros::Duration(1.0).sleep();
        if (trials++ > 5)
            ROS_ERROR_THROTTLE(2.0, "[PX4CTRL] Unable to connnect to PX4!!!");
    }
    ROS_INFO("[PX4CTRL][1/3] PX4 connected.");

    // ---- 2/3: 等待遥控器信号 ----
    while (ros::ok())
    {
        ros::spinOnce();
        if (node.state_data.current_state.manual_input)
        {
            ROS_INFO("[PX4CTRL][2/3] RC received.");
            node.rc_data.check_validity();
            break;
        }
        ros::Duration(0.1).sleep();
    }

    // ---- 3/3: 等待里程计数据 ----
    while (ros::ok())
    {
        ros::spinOnce();
        if (node.odom_is_received(ros::Time::now()))
        {
            ROS_INFO("[PX4CTRL][3/3] Odometry received and publishing.");
            break;
        }
        ros::Duration(0.1).sleep();
    }

    ROS_INFO("[PX4CTRL] Mode: %s, Battery: %.1f V, Current: %.2f A",
        node.state_data.current_state.mode.c_str(), node.battery_data.battery.voltage, node.battery_data.battery.current);
    ROS_INFO("[PX4CTRL] Please manually switch to offboard mode and manually arm it to automatically takeoff.");

    // ---- 主循环 30 Hz ----
    ros::Rate r(30.0);
    while (ros::ok())
    {
        r.sleep();
        ros::spinOnce();
        node.process();  ///< 执行主状态机
    }

    return 0;
}
