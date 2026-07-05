/**
 * @file px4_vel_simple_test.cpp
 * @brief PX4 综合控制测试节点 — RC 状态机 + 位置/速度双模式 + 里程计桥接。
 *
 * @details
 * 本节点是 px4ctrl_node 的简化测试版本，集成了以下核心功能：
 *   1. 遥控器状态机   — RC 通道解析 (CH5=arm, CH6=offboard, CH7=command/hold)
 *   2. 里程计桥接     — /Odometry → /mavros/vision_pose/pose (坐标修正)
 *   3. 位置控制模式   — 悬停保持 + 起飞检测 (setpoint_position/local)
 *   4. 速度控制模式   — 6 阶段指令序列 (X±, Y±, Z±) 每阶段 3 秒 (cmd_vel)
 *   5. 安全门限       — 电池低压检测 + 位置/速度限幅 (clamp)
 *   6. 坐标修正       — uav_mavros_pose_fix (ENU→NED) + uav_mavros_cmd_vel_fix
 *
 * 控制模式切换：
 *   - control_status = 0: 位置控制 (默认)
 *   - control_status = 1: 速度控制
 *
 * RC 状态机 (Gear 通道 CH7)：
 *   CH7 > 1750  → command_mode (执行速度指令序列)
 *   CH7 1250~1750 → hold_mode (悬停+起飞)
 *   CH7 < 1250  → idle (降落/空闲)
 *
 * 坐标系约定：
 *   - Odom 输入：ENU 系（Fast-LIO 等），经 uav_mavros_pose_fix 转换为 NED
 *   - 输出指令：NED 系 (PX4 本地坐标系)
 *   - 位置限幅：x 与 y 互换限幅（配合旋转后的坐标语义）
 *
 * 依赖：
 *   - subscribe.h (RC_Data_t, State_Data_t, Battery_Data_t, Command_Data_t, Odom_Data_t)
 *   - mavros_msgs (State, RCIn, CommandBool, SetMode, CommandLong)
 *   - quadrotor_msgs/PositionCommand
 *   - nav_msgs/Odometry
 *   - sensor_msgs/BatteryState
 *   - tf2 (坐标旋转)
 *
 * 使用示例：
 *   roslaunch uav_px4_ctrl_test px4_vel_simple_test.launch
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
#include "subscribe.h"

// ============================================================================
// 坐标变换工具函数
// ============================================================================

/**
 * @brief 位姿坐标修正：ENU → NED 等效变换。
 * @param[in,out] pose 待修正的位姿消息
 *
 * 位置：x' = -y, y' = x (交换并取反)
 * 姿态：绕 Z 轴旋转 +90° (π/2)
 */
void uav_mavros_pose_fix(geometry_msgs::PoseStamped * pose)
{
    double temp_x = pose->pose.position.x;
    double temp_y = pose->pose.position.y;

    pose->pose.position.x = -temp_y;
    pose->pose.position.y = temp_x;

    tf2::Quaternion q_orig, q_rot, q_new;
    tf2::fromMsg(pose->pose.orientation, q_orig);
    q_rot.setRPY(0, 0, M_PI/2);  // 绕 Z 轴旋转 +90°
    q_new = q_rot * q_orig;      // 旋转叠加
    q_new.normalize();
    pose->pose.orientation = tf2::toMsg(q_new);
}

/**
 * @brief 速度指令坐标修正：ENU → NED 等效变换（仅线速度 XY）。
 * @param[in,out] vel 待修正的速度消息
 *
 * 线速度：x' = -y, y' = x (交换并取反)
 * 角速度不做变换（直接公式映射已足够）
 */
void uav_mavros_cmd_vel_fix(geometry_msgs::TwistStamped * vel)
{
    double temp_x = vel->twist.linear.x;
    double temp_y = vel->twist.linear.y;

    vel->twist.linear.x = -temp_y;
    vel->twist.linear.y = temp_x;
}

/**
 * @brief 偏航角 → 四元数转换。
 * @param yaw_rad 偏航角 [rad]
 * @return 对应四元数 (仅绕 Z 轴旋转)
 */
geometry_msgs::Quaternion yawToQuaternion(double yaw_rad)
{
    tf2::Quaternion q;
    q.setRPY(0, 0, yaw_rad);
    return tf2::toMsg(q);
}

// ============================================================================
// DroneCtrl — 无人机综合控制类
// ============================================================================
/**
 * @brief 无人机控制核心类，封装全部 ROS 通信、状态数据、控制逻辑。
 *
 * 维护遥控器/飞控/电池/里程计/轨迹指令五类状态数据，
 * 实现位置控制（悬停+起飞）和速度控制（指令序列）双模式切换。
 */
class DroneCtrl
{
public:
    ros::NodeHandle nh;

    // ---- 订阅者 ----
    ros::Subscriber state_sub;       ///< 飞控状态订阅
    ros::Subscriber battery_sub;     ///< 电池状态订阅
    ros::Subscriber cmd_sub;        ///< 轨迹指令订阅 (TCP_NODELAY)
    ros::Subscriber rc_sub;         ///< RC 遥控器订阅
    ros::Subscriber odom_sub;       ///< 里程计订阅

    // ---- 发布者 ----
    ros::Publisher pose_pub;                ///< 视觉位姿 → /mavros/vision_pose/pose
    ros::Publisher vel_pub;                 ///< 速度指令 → /mavros/setpoint_velocity/cmd_vel
    ros::Publisher local_pos_pub;           ///< 位置指令 → /mavros/setpoint_position/local
    ros::Publisher traj_start_trigger_pub;  ///< 轨迹启动触发 (预留)

    // ---- 服务客户端 ----
    ros::ServiceClient set_FCU_mode_srv;    ///< 模式切换服务
    ros::ServiceClient arming_client_srv;   ///< 解锁服务
    ros::ServiceClient reboot_FCU_srv;      ///< 飞控重启服务 (预留)

    // ---- 状态数据结构 ----
    RC_Data_t rc_data;              ///< 遥控器数据 (通道解析 + 状态机)
    State_Data_t state_data;        ///< 飞控状态缓存
    Battery_Data_t battery_data;    ///< 电池状态缓存
    Command_Data_t cmd_data;        ///< 轨迹指令缓存
    Odom_Data_t odom_data;          ///< 里程计数据缓存

    /**
     * @brief 构造函数 — 初始化所有 ROS 通信接口。
     *
     * 使用 boost::bind 绑定状态数据结构的 feed() 方法作为回调，
     * 实现数据即到即更新的零拷贝模式。
     */
    DroneCtrl() : nh("~")
    {
        state_sub = nh.subscribe<mavros_msgs::State>(
            "/mavros/state", 10, boost::bind(&State_Data_t::feed, &state_data, _1));

        battery_sub = nh.subscribe<sensor_msgs::BatteryState>(
            "/mavros/battery", 10, boost::bind(&Battery_Data_t::feed, &battery_data, _1));

        cmd_sub = nh.subscribe<quadrotor_msgs::PositionCommand>(
            "/position_cmd", 100, boost::bind(&Command_Data_t::feed, &cmd_data, _1),
            ros::VoidConstPtr(), ros::TransportHints().tcpNoDelay());

        rc_sub = nh.subscribe<mavros_msgs::RCIn>(
            "/mavros/rc/in", 10, boost::bind(&RC_Data_t::feed, &rc_data, _1));

        odom_sub = nh.subscribe("/Odometry", 10, &DroneCtrl::odometryCallback, this);

        pose_pub = nh.advertise<geometry_msgs::PoseStamped>(
            "/mavros/vision_pose/pose", 10);
        vel_pub = nh.advertise<geometry_msgs::TwistStamped>(
            "/mavros/setpoint_velocity/cmd_vel", 10);
        local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>(
            "/mavros/setpoint_position/local", 10);
        traj_start_trigger_pub = nh.advertise<geometry_msgs::PoseStamped>(
            "/traj_start_trigger", 10);

        set_FCU_mode_srv = nh.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");
        arming_client_srv = nh.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");
        reboot_FCU_srv = nh.serviceClient<mavros_msgs::CommandLong>("/mavros/cmd/command");
    }

    // ---- 数据到达检测 ----
    bool rc_is_received(const ros::Time &now_time)      { return rc_data.is_received(now_time); }
    bool state_is_received(const ros::Time &now_time)    { return state_data.is_received(now_time); }
    bool battery_is_received(const ros::Time &now_time)  { return battery_data.is_received(now_time); }
    bool odom_is_received(const ros::Time &now_time)     { return odom_data.is_received(now_time); }
    bool cmd_is_received(const ros::Time &now_time)      { return cmd_data.is_received(now_time); }

    // ---- 控制接口 ----
    void init_target_takeoff();
    void update_target(const nav_msgs::Odometry& target);
    void update_cmd_vel(const quadrotor_msgs::PositionCommand& msg);
    void process();
    void publish_target();
    void publish_cmd_vel();

    void odometryCallback(const nav_msgs::Odometry::ConstPtr& msg);

private:
    // ---- 飞行状态标志 ----
    bool is_armed = false;       ///< 是否已解锁
    bool is_takeoff = false;     ///< 是否已完成起飞
    bool is_land = true;         ///< 是否在地面
    bool is_landing = false;    ///< 是否正在降落
    bool enter_hold = false;    ///< 是否进入悬停模式
    bool enter_land = false;    ///< 是否进入降落
    bool check_battery = false; ///< 电池电压是否正常
    bool check_rc_stick = true; ///< 是否检查 RC 摇杆回中

    ros::Time last_arm_request;  ///< 上次请求解锁时间 [3s 间隔重试]

    // ---- 期望指令 ----
    geometry_msgs::PoseStamped desired_target;     ///< 期望位置 (悬停/起飞用)
    geometry_msgs::TwistStamped desired_cmd_vel;   ///< 期望速度 (指令序列用)

    // ---- 控制参数 ----
    int control_status = 0;          ///< 0: 位置控制, 1: 速度控制, 2: 姿态控制
    std::string control_mode;        ///< 控制模式字符串
    double takeoff_height = 0.5;     ///< 起飞高度 [m]
    double battery_limit = 19.8;     ///< 电池低压阈值 [V] (4S: 19.8V ≈ 3.3V/cell)

    // ---- 位置安全限幅 (经坐标旋转后的 NED 系) ----
    double position_max_x = 2.0;    ///< X 最大位置 [m]
    double position_min_x = -2.0;   ///< X 最小位置 [m]
    double position_max_y = 2.0;    ///< Y 最大位置 [m]
    double position_min_y = -2.0;   ///< Y 最小位置 [m]
    double position_max_z = 1.5;    ///< Z 最大位置 [m]
    double position_min_z = 0.0;    ///< Z 最小位置 [m]

    // ---- 速度安全限幅 ----
    double cmd_vel_max_x = 1.0;     ///< X 最大速度 [m/s]
    double cmd_vel_max_y = 1.0;     ///< Y 最大速度 [m/s]
    double cmd_vel_max_z = 0.5;     ///< Z 最大速度 [m/s]

    // ---- 指令序列状态机 ----
    int cmd_stage = 0;              ///< 当前步骤 (0=idle, 1~6=执行中)
    ros::Time cmd_start_time;       ///< 当前步骤开始时间
    bool cmd_running = false;       ///< 是否正在执行指令序列
    bool cmd_finish = false;        ///< 是否已完成指令序列
};

// ============================================================================
// DroneCtrl 成员函数实现
// ============================================================================

/**
 * @brief 里程计回调 — 坐标修正 + 视觉位姿发布。
 *
 * 算法流水线：
 *   1. 缓存里程计数据到 odom_data
 *   2. 提取位姿到 PoseStamped
 *   3. 应用 uav_mavros_pose_fix (ENU→NED)
 *   4. 发布到 /mavros/vision_pose/pose
 */
void DroneCtrl::odometryCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
    odom_data.feed(msg);

    geometry_msgs::PoseStamped pose_msg;
    pose_msg.header = msg->header;
    pose_msg.header.frame_id = "odom";
    pose_msg.pose = msg->pose.pose;

    uav_mavros_pose_fix(&pose_msg);  // ENU → NED

    pose_pub.publish(pose_msg);
}

/**
 * @brief 初始化起飞目标 — 设置默认起飞高度。
 */
void DroneCtrl::init_target_takeoff()
{
    desired_target.pose.position.z = 0.5;  // [m]
}

/**
 * @brief 更新期望位置 (来自轨迹规划器)。
 * @param target 轨迹规划器输出的目标里程计消息
 *
 * 从 odometry 中提取位姿后，经 uav_mavros_pose_fix 坐标修正。
 */
void DroneCtrl::update_target(const nav_msgs::Odometry& target)
{
    desired_target.pose.position.x = target.pose.pose.position.x;
    desired_target.pose.position.y = target.pose.pose.position.y;
    desired_target.pose.position.z = target.pose.pose.position.z;
    desired_target.pose.orientation.x = target.pose.pose.orientation.x;
    desired_target.pose.orientation.y = target.pose.pose.orientation.y;
    desired_target.pose.orientation.z = target.pose.pose.orientation.z;
    desired_target.pose.orientation.w = target.pose.pose.orientation.w;
    uav_mavros_pose_fix(&desired_target);  // ENU → NED
}

/**
 * @brief 更新期望速度 (来自轨迹规划器 PositionCommand)。
 * @param msg 轨迹规划器的全状态指令
 *
 * 从 PositionCommand 中提取速度分量，经 uav_mavros_cmd_vel_fix 坐标修正。
 */
void DroneCtrl::update_cmd_vel(const quadrotor_msgs::PositionCommand& msg)
{
    desired_cmd_vel.twist.linear.x = msg.velocity.x;
    desired_cmd_vel.twist.linear.y = msg.velocity.y;
    desired_cmd_vel.twist.linear.z = msg.velocity.z;
    // 角速度暂不使用（由 PX4 内部姿态控制处理）
    // desired_cmd_vel.twist.angular.x = 0.0;
    // desired_cmd_vel.twist.angular.y = 0.0;
    // desired_cmd_vel.twist.angular.z = msg->yaw_dot;
    uav_mavros_cmd_vel_fix(&desired_cmd_vel);
}

/**
 * @brief 主控制逻辑 — 每帧调用的核心状态机。
 *
 * 算法流水线：
 *   1. 解锁判定 — RC arm 信号 + 电池/摇杆条件 → 3s 间隔解锁
 *   2. 模式路由 — offboard 模式 + 未降落:
 *      a) hold_mode / 未起飞 / enter_hold → 位置控制 (悬停+起飞)
 *      b) command_mode → 速度指令序列 (6 阶段 × 3s)
 *   3. 电池监测 — 电压 < battery_limit 触发告警
 */
void DroneCtrl::process()
{
    geometry_msgs::PoseStamped target;
    static bool have_hold_set = false;

    // ---- 1. 解锁判定 ----
    if(rc_data.is_armed && rc_data.is_offboard_mode && !is_armed && check_battery && check_rc_stick)
    {
        // 3 秒重试间隔
        if((ros::Time::now() - last_arm_request).toSec() > 3.0)
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

    // ---- 2. Offboard 模式下的控制路由 ----
    if(rc_data.is_offboard_mode && !is_landing)
    {
        // 2a. 悬停模式 / 未起飞 / 进入悬停
        if(rc_data.is_hold_mode || (rc_data.is_command_mode && !is_takeoff) || (rc_data.is_command_mode && enter_hold))
        {
            if(!have_hold_set)
            {
                update_target(odom_data.msg);
                have_hold_set = true;
            }

            // 首次起飞逻辑
            if(is_land && !is_takeoff)
            {
                init_target_takeoff();
                is_land = false;
            }

            // 起飞完成判定
            if(odom_data.msg.pose.pose.position.z > takeoff_height - 0.5 && !is_takeoff)
            {
                is_takeoff = true;
            }
            publish_target();
        }
        // 2b. 指令模式 — 执行速度指令序列
        else
        {
            if(rc_data.is_command_mode)
            {
                if(!cmd_finish)
                {
                    if(!cmd_running)
                    {
                        cmd_running = true;
                        cmd_stage = 1;
                        cmd_start_time = ros::Time::now();
                    }

                    double dt = (ros::Time::now() - cmd_start_time).toSec();

                    // 6 阶段指令序列，每阶段 3 秒：
                    //   Stage 1: X+ (前进)    0.5 m/s
                    //   Stage 2: X- (后退)   -0.5 m/s
                    //   Stage 3: Y+ (左移)   -0.5 m/s (NED 系 Y+=左)
                    //   Stage 4: Y- (右移)    0.5 m/s
                    //   Stage 5: Z+ (上升)    0.1 m/s
                    //   Stage 6: Z- (下降)   -0.1 m/s
                    if(cmd_stage == 1)
                    {
                        if(dt < 3.0) {
                            quadrotor_msgs::PositionCommand cmd;
                            cmd.velocity.x = 0.5;          // X 正向 [m/s]
                            update_cmd_vel(cmd);
                            publish_cmd_vel();
                        } else {
                            cmd_stage = 2;
                            cmd_start_time = ros::Time::now();
                        }
                    }
                    else if(cmd_stage == 2)
                    {
                        if(dt < 3.0) {
                            quadrotor_msgs::PositionCommand cmd;
                            cmd.velocity.x = -0.5;         // X 负向 [m/s]
                            update_cmd_vel(cmd);
                            publish_cmd_vel();
                        } else {
                            cmd_stage = 3;
                            cmd_start_time = ros::Time::now();
                        }
                    }
                    else if(cmd_stage == 3)
                    {
                        if(dt < 3.0) {
                            quadrotor_msgs::PositionCommand cmd;
                            cmd.velocity.y = -0.5;         // Y 正向 (NED 系 Y+ 向左) [m/s]
                            update_cmd_vel(cmd);
                            publish_cmd_vel();
                        } else {
                            cmd_stage = 4;
                            cmd_start_time = ros::Time::now();
                        }
                    }
                    else if(cmd_stage == 4)
                    {
                        if(dt < 3.0) {
                            quadrotor_msgs::PositionCommand cmd;
                            cmd.velocity.y = 0.5;          // Y 负向 (NED 系 Y- 向右) [m/s]
                            update_cmd_vel(cmd);
                            publish_cmd_vel();
                        } else {
                            cmd_stage = 5;
                            cmd_start_time = ros::Time::now();
                        }
                    }
                    else if(cmd_stage == 5)
                    {
                        if(dt < 3.0) {
                            quadrotor_msgs::PositionCommand cmd;
                            cmd.velocity.z = 0.1;          // Z 正向 (上升) [m/s]
                            update_cmd_vel(cmd);
                            publish_cmd_vel();
                        } else {
                            cmd_stage = 6;
                            cmd_start_time = ros::Time::now();
                        }
                    }
                    else if(cmd_stage == 6)
                    {
                        if(dt < 3.0) {
                            quadrotor_msgs::PositionCommand cmd;
                            cmd.velocity.z = -0.1;         // Z 负向 (下降) [m/s]
                            update_cmd_vel(cmd);
                            publish_cmd_vel();
                        } else {
                            cmd_finish = true;
                            cmd_running = false;
                        }
                    }
                }
            }
            // 2c. 非悬停非指令 → 准备降落
            else
            {
                if(!is_landing) is_land = true;
                if(!is_land && !is_takeoff) is_landing = true;
                is_takeoff = false;
                enter_hold = false;
                cmd_finish = false;
            }
            have_hold_set = false;
        }
    }
    // 3. 非 offboard → 重置所有状态
    else
    {
        is_takeoff = false;
        is_land = true;
        enter_hold = false;
        enter_land = false;
        have_hold_set = false;
    }

    // ---- 4. 电池低压检测 ----
    if(battery_data.battery.voltage < battery_limit)
    {
        check_battery = false;
        ROS_WARN_THROTTLE(5.0, "[PX4CTRL] Battery voltage low: %.2f V < %.2f V",
            battery_data.battery.voltage, battery_limit);
    }
    else
    {
        check_battery = true;
    }
}

/**
 * @brief 发布期望位置 (带安全限幅)。
 *
 * 坐标限幅特殊处理：旋转后的坐标 (x→-y, y→x) 导致限幅轴交叉映射:
 *   target.x ∈ [-position_max_y, -position_min_y]
 *   target.y ∈ [position_min_x, position_max_x]
 *   target.z ∈ [position_min_z, position_max_z]
 */
void DroneCtrl::publish_target()
{
    geometry_msgs::PoseStamped target = desired_target;

    // 安全限幅 lambda
    auto clamp = [](double value, double min_val, double max_val) {
        if (value < min_val)
            return min_val;
        else if (value > max_val)
            return max_val;
        else
            return value;
    };

    target.pose.position.x = clamp(target.pose.position.x, -position_max_y, -position_min_y);
    target.pose.position.y = clamp(target.pose.position.y,  position_min_x,  position_max_x);
    target.pose.position.z = clamp(target.pose.position.z,  position_min_z,  position_max_z);
    local_pos_pub.publish(target);
}

/**
 * @brief 发布期望速度 (带安全限幅)。
 *
 * 速度限幅同样适应坐标旋转的交叉映射。
 */
void DroneCtrl::publish_cmd_vel()
{
    geometry_msgs::TwistStamped cmd_vel = desired_cmd_vel;

    auto clamp = [](double value, double max_val) {
        if (value > max_val){
            return max_val;
            ROS_WARN_THROTTLE(5.0, "[PX4CTRL] Velocity limit exceeded: %.2f > %.2f", value, max_val);}
        else{
            return value;}
    };

    cmd_vel.twist.linear.x = clamp(cmd_vel.twist.linear.x,  cmd_vel_max_y);
    cmd_vel.twist.linear.y = clamp(cmd_vel.twist.linear.y,  cmd_vel_max_x);
    cmd_vel.twist.linear.z = clamp(cmd_vel.twist.linear.z,  cmd_vel_max_z);
    vel_pub.publish(cmd_vel);
}

// ============================================================================
// SIGINT 信号处理
// ============================================================================
/**
 * @brief SIGINT (Ctrl-C) 信号处理器 — 优雅退出。
 */
void mySigintHandler(int sig)
{
    ROS_INFO("[PX4Ctrl] exit...");
    ros::shutdown();
}

// ============================================================================
// main — 控制节点入口
// ============================================================================
/**
 * @brief 控制节点主函数 — 三阶段启动等待 + 主循环。
 *
 * 启动流水线：
 *   1/3 — 等待 PX4 连接 (最多 5 次重试)
 *   2/3 — 等待 RC 遥控器信号
 *   3/3 — 等待里程计数据
 *   进入主循环 — 30 Hz 调用 DroneCtrl::process()
 */
int main(int argc, char *argv[])
{
    ros::init(argc, argv, "px4ctrl");
    signal(SIGINT, mySigintHandler);

    DroneCtrl node;

    ros::Duration(1).sleep();

    // ---- 阶段 1/3: 等待 PX4 连接 ----
    int trials = 0;
    while (ros::ok() && !node.state_data.current_state.connected)
    {
        ros::spinOnce();
        ros::Duration(1.0).sleep();
        if (trials++ > 5)
            ROS_ERROR_THROTTLE(2.0, "[PX4CTRL] Unable to connnect to PX4!!!");
    }
    ROS_INFO("[PX4CTRL][1/3] PX4 connected.");

    // ---- 阶段 2/3: 等待 RC 遥控器信号 ----
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

    // ---- 阶段 3/3: 等待里程计数据 ----
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

    // ---- 状态汇总 ----
    ROS_INFO("[PX4CTRL] Mode: %s, Battery: %.1f V, Current: %.2f A",
        node.state_data.current_state.mode.c_str(),
        node.battery_data.battery.voltage,
        node.battery_data.battery.current);
    ROS_INFO("[PX4CTRL] Please manually switch to offboard mode and manually arm it to automatically takeoff.");

    // ---- 主循环 (30 Hz) ----
    ros::Rate r(30.0);
    while (ros::ok())
    {
        r.sleep();
        ros::spinOnce();
        node.process();
    }

    return 0;
}
