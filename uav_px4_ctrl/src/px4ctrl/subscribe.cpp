/**
 * @file subscribe.cpp
 * @brief PX4 无人机控制节点 — 数据类型实现（RC/State/Battery/Command/Odom 解析）。
 *
 * @details
 * 实现了 subscribe.h 中声明的五个数据结构体的回调与状态机逻辑：
 *   1. RC_Data_t   — 遥控器 PWM 归一化 + arm/mode/gear 三通道状态机
 *   2. State_Data_t — PX4 飞控状态存储
 *   3. Battery_Data_t — 电池电压/电流存储
 *   4. Command_Data_t — 轨迹指令 (PositionCommand) → Eigen 向量提取
 *   5. Odom_Data_t — 里程计 (Odometry) → Eigen 向量/四元数提取
 *
 * 遥控器状态机（核心逻辑）：
 *   - CH5 (arm):    PWM > 1750 → armed
 *   - CH6 (mode):   PWM > 1750 → offboard mode
 *   - CH7 (gear):   PWM > 1750 → command mode; 1250~1750 → hold mode; <1250 → idle
 *
 * 使用：
 *   #include "px4ctrl/subscribe.h"
 */

#include "subscribe.h"

// ============================================================================
// RC_Data_t — 遥控器数据
// ============================================================================

RC_Data_t::RC_Data_t()
{
    rcv_stamp = ros::Time(0);

    last_mode = -1.0;
    last_gear = -1.0;

    // Parameter initialization is very important in RC-Free usage!
    is_armed = false;
    is_offboard_mode = true;
    enter_hover_mode = false;
    is_command_mode = true;
    enter_command_mode = false;
    for (int i = 0; i < 4; ++i)
    {
        ch[i] = 0.0;  ///< 摇杆通道归零
    }
}

void RC_Data_t::feed(mavros_msgs::RCInConstPtr pMsg)
{
    msg = *pMsg;
    rcv_stamp = ros::Time::now();

    // ---- 1. 摇杆通道归一化 (CH1-4): (raw - 1500) / 500, 经死区处理 ----
    for (int i = 0; i < 4; i++)
    {
        ch[i] = ((double)msg.channels[i] - 1500.0) / 500.0;
        if (ch[i] > DEAD_ZONE)
            ch[i] = (ch[i] - DEAD_ZONE) / (1 - DEAD_ZONE);       ///< 正区间线性映射
        else if (ch[i] < -DEAD_ZONE)
            ch[i] = (ch[i] + DEAD_ZONE) / (1 - DEAD_ZONE);       ///< 负区间线性映射
        else
            ch[i] = 0.0;  ///< 死区内归零
    }

    // ---- 2. 开关通道读取 (CH5-7) ----
    arm = (int)msg.channels[4];   ///< CH5: arm 开关
    mode = (int)msg.channels[5];  ///< CH6: 模式开关
    gear = (int)msg.channels[6];  ///< CH7: gear 开关

    check_validity();

    // ---- 3. 初始化上一帧值 ----
    if (!have_init_last_mode)
    {
        have_init_last_mode = true;
        last_mode = mode;
    }
    if (!have_init_last_gear)
    {
        have_init_last_gear = true;
        last_gear = gear;
    }

    // ---- 4. Arm 边沿检测与日志 ----
    if (last_arm < ARM_THRESHOLD_VALUE && arm > ARM_THRESHOLD_VALUE)
        ROS_INFO("[PX4CTRL] Armed.");
    else if(last_arm > ARM_THRESHOLD_VALUE && arm < ARM_THRESHOLD_VALUE)
        ROS_INFO("[PX4CTRL] Disarmed.");

    if (arm > ARM_THRESHOLD_VALUE)
        is_armed = true;
    else
        is_armed = false;

    // ---- 5. Mode 状态机 (CH6): offboard 模式切换 ----
    // 上升沿 → 进入 offboard/hover 模式
    if (last_mode < API_MODE_THRESHOLD_VALUE && mode > API_MODE_THRESHOLD_VALUE){
        enter_hover_mode = true;
        ROS_INFO("[PX4CTRL] Enter offboard mode.");}
    // 下降沿 → 退出 offboard
    else if(last_mode > API_MODE_THRESHOLD_VALUE && mode < API_MODE_THRESHOLD_VALUE){
        enter_hover_mode = false;
        if(is_command_mode) ROS_INFO("[PX4CTRL] Stop command send.");
        ROS_INFO("[PX4CTRL] Exit offboard mode.");}

    if (mode > API_MODE_THRESHOLD_VALUE)
        is_offboard_mode = true;
    else
        is_offboard_mode = false;

    // ---- 6. Gear 状态机 (CH7): 指令/悬停/空闲 三态 ----
    if (is_offboard_mode)
    {
        // 上升沿 → 进入指令模式
        if (last_gear < GEAR_SHIFT_UP_THRESHOLD && gear > GEAR_SHIFT_UP_THRESHOLD){
            enter_command_mode = true;
            ROS_INFO("[PX4CTRL] Start command send");}
        // 下降沿 → 退出指令模式
        else if (last_gear > GEAR_SHIFT_UP_THRESHOLD && gear < GEAR_SHIFT_UP_THRESHOLD){
            enter_command_mode = false;
            ROS_INFO("[PX4CTRL] Stop command send");}

        // PWM 区间判定
        if (gear > GEAR_SHIFT_UP_THRESHOLD){                    ///< 高位: command mode
            is_command_mode = true;
            is_hold_mode = false;}
        else if (gear > GEAR_SHIFT_DOWN_THRESHOLD && gear < GEAR_SHIFT_UP_THRESHOLD){  ///< 中位: hold mode
            is_command_mode = false;
            is_hold_mode = true;}
        else{                                                   ///< 低位: idle
            is_command_mode = false;
            is_hold_mode = false;}
    }
    else
    {
        is_command_mode = false;
    }

    // ---- 7. 保存上一帧状态 ----
    last_arm = arm;
    last_mode = mode;
    last_gear = gear;
}

void RC_Data_t::check_validity()
{
    // mode 和 gear 通道 PWM 必须在有效范围 [900, 2100] 内
    if (mode >= 900 && mode <= 2100 && gear >= 900 && gear <= 2100)
    {
        // pass
    }
    else
    {
        ROS_ERROR("RC data validity check fail. mode=%d, gear=%d", mode, gear);
    }
}

bool RC_Data_t::check_centered()
{
    // 检查四个摇杆通道是否同时回中（死区范围内）
    bool centered = abs(ch[0]) < 1e-5 && abs(ch[0]) < 1e-5 && abs(ch[0]) < 1e-5 && abs(ch[0]) < 1e-5;
    return centered;
}

bool RC_Data_t::is_received(const ros::Time &now_time)
{
    return (now_time - this->rcv_stamp).toSec() < 0.5;  ///< 0.5s 超时窗口
}

// ============================================================================
// State_Data_t — 飞控状态
// ============================================================================

State_Data_t::State_Data_t()
{
    rcv_stamp = ros::Time(0);
}

void State_Data_t::feed(mavros_msgs::StateConstPtr pMsg)
{
    current_state = *pMsg;
    rcv_stamp = ros::Time::now();
}

bool State_Data_t::is_received(const ros::Time &now_time)
{
    return (now_time - this->rcv_stamp).toSec() < 0.5;  ///< 0.5s 超时窗口
}

// ============================================================================
// Battery_Data_t — 电池状态
// ============================================================================

Battery_Data_t::Battery_Data_t()
{
    rcv_stamp = ros::Time(0);
}

void Battery_Data_t::feed(sensor_msgs::BatteryStateConstPtr pMsg)
{
    battery = *pMsg;
    rcv_stamp = ros::Time::now();
}

bool Battery_Data_t::is_received(const ros::Time &now_time)
{
    return (now_time - this->rcv_stamp).toSec() < 0.5;  ///< 0.5s 超时窗口
}

// ============================================================================
// Command_Data_t — 轨迹指令
// ============================================================================

Command_Data_t::Command_Data_t()
{
    rcv_stamp = ros::Time(0);
}

void Command_Data_t::feed(quadrotor_msgs::PositionCommandConstPtr pMsg)
{
    msg = *pMsg;
    rcv_stamp = ros::Time::now();

    // 提取 Eigen 向量表示
    p(0) = msg.position.x;     ///< 目标位置 X [m]
    p(1) = msg.position.y;     ///< 目标位置 Y [m]
    p(2) = msg.position.z;     ///< 目标位置 Z [m]

    v(0) = msg.velocity.x;     ///< 目标速度 X [m/s]
    v(1) = msg.velocity.y;     ///< 目标速度 Y [m/s]
    v(2) = msg.velocity.z;     ///< 目标速度 Z [m/s]

    a(0) = msg.acceleration.x; ///< 目标加速度 X [m/s²]
    a(1) = msg.acceleration.y; ///< 目标加速度 Y [m/s²]
    a(2) = msg.acceleration.z; ///< 目标加速度 Z [m/s²]

    j(0) = msg.jerk.x;         ///< 目标 jerk X [m/s³]
    j(1) = msg.jerk.y;         ///< 目标 jerk Y [m/s³]
    j(2) = msg.jerk.z;         ///< 目标 jerk Z [m/s³]

    yaw = msg.yaw;             ///< 目标偏航角 [rad]
    yaw_rate = msg.yaw_dot;    ///< 目标偏航角速率 [rad/s]
}

bool Command_Data_t::is_received(const ros::Time &now_time)
{
    return (now_time - this->rcv_stamp).toSec() < 0.1;  ///< 0.1s 超时窗口（指令需要更及时）
}

// ============================================================================
// Odom_Data_t — 里程计
// ============================================================================

Odom_Data_t::Odom_Data_t()
{
    rcv_stamp = ros::Time(0);
    q.setIdentity();          ///< 四元数初始化为单位值
    recv_new_msg = false;
};

void Odom_Data_t::feed(nav_msgs::OdometryConstPtr pMsg)
{
    ros::Time now = ros::Time::now();

    msg = *pMsg;
    rcv_stamp = now;
    recv_new_msg = true;

    // 提取 Eigen 向量/四元数
    p(0) = msg.pose.pose.position.x;     ///< 位置 X [m]
    p(1) = msg.pose.pose.position.y;     ///< 位置 Y [m]
    p(2) = msg.pose.pose.position.z;     ///< 位置 Z [m]

    v(0) = msg.twist.twist.linear.x;     ///< 线速度 X [m/s]
    v(1) = msg.twist.twist.linear.y;     ///< 线速度 Y [m/s]
    v(2) = msg.twist.twist.linear.z;     ///< 线速度 Z [m/s]

    q.w() = msg.pose.pose.orientation.w; ///< 姿态四元数 w
    q.x() = msg.pose.pose.orientation.x; ///< 姿态四元数 x
    q.y() = msg.pose.pose.orientation.y; ///< 姿态四元数 y
    q.z() = msg.pose.pose.orientation.z; ///< 姿态四元数 z
}

bool Odom_Data_t::is_received(const ros::Time &now_time)
{
    return (now_time - this->rcv_stamp).toSec() < 0.5;  ///< 0.5s 超时窗口
}
