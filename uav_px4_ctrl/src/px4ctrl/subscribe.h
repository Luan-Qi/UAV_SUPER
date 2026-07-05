/**
 * @file subscribe.h
 * @brief PX4 无人机控制节点 — 数据类型定义与 MAVROS 话题订阅封装。
 *
 * @details
 * 本文件定义了 px4ctrl 节点所需的 ROS 话题数据接收与缓存结构：
 *   1. RC_Data_t     — 遥控器输入解析，含通道归一化、模式/gear 状态机
 *   2. Odom_Data_t   — 里程计位姿/速度缓存（Eigen 向量 + 四元数）
 *   3. State_Data_t  — PX4 飞控状态缓存
 *   4. Battery_Data_t — 电池状态缓存
 *   5. Command_Data_t — 外部轨迹指令缓存（位置/速度/加速度/Jerk）
 *
 * 坐标系约定：
 *   - Odom 位姿/速度在里程计系（通常为 ENU 或 FLU body-aligned）
 *   - 外部指令目标点为 map 或 odom 系
 *
 * 使用：
 *   #include "px4ctrl/subscribe.h"
 *   配合 px4ctrl_node.cpp 编译为 px4ctrl_node 可执行文件
 */

#ifndef __INPUT_H
#define __INPUT_H

#include <ros/ros.h>
#include <Eigen/Dense>
#include <quadrotor_msgs/PositionCommand.h>
#include <nav_msgs/Odometry.h>
#include <mavros_msgs/RCIn.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/BatteryStatus.h>
#include <mavros_msgs/ExtendedState.h>
#include <sensor_msgs/BatteryState.h>

// ============================================================================
// 遥控器数据 (RC_Data_t)
// ============================================================================
/** @brief 遥控器输入解析与状态机。
 *
 * 通道映射（假设日本手）：
 *   ch[0] = roll  (CH1)
 *   ch[1] = pitch (CH2)
 *   ch[2] = throttle (CH3)
 *   ch[3] = yaw   (CH4)
 *   ch[4] = arm 开关 (CH5)
 *   ch[5] = mode 开关 (CH6)
 *   ch[6] = gear 开关 (CH7)
 *
 * 通道值归一化：(raw - 1500) / 500，经 DEAD_ZONE 处理后映射到 [-1, 1]。
 */
class RC_Data_t
{
public:
    int arm;         ///< CH5 原始 PWM 值 (≈1000-2000)
    int mode;        ///< CH6 原始 PWM 值
    int gear;        ///< CH7 原始 PWM 值
    int last_arm;    ///< 上一帧 arm 值，用于边沿检测
    int last_mode;   ///< 上一帧 mode 值
    int last_gear;   ///< 上一帧 gear 值
    bool have_init_last_mode{false};  ///< 是否已初始化 last_mode
    bool have_init_last_gear{false};  ///< 是否已初始化 last_gear
    double ch[4];    ///< 摇杆通道归一化值 [-1, 1]  (roll/pitch/throttle/yaw)

    mavros_msgs::RCIn msg;     ///< 原始 MAVROS RC 消息
    ros::Time rcv_stamp;       ///< 最近一帧接收时间

    bool is_armed;             ///< 当前 arm 状态（由 CH5 判定）
    bool is_command_mode;      ///< 当前是否在指令模式（由 CH7 高位判定）
    bool enter_command_mode;   ///< 刚进入指令模式的边沿标志
    bool is_offboard_mode;     ///< 当前是否在 offboard 模式（由 CH6 高位判定）
    bool enter_hover_mode;     ///< 刚进入 offboard 模式的边沿标志
    bool is_hold_mode;         ///< 当前是否在悬停模式（CH7 中位）

    // ---- PWM 阈值常量 ----
    static constexpr int ARM_THRESHOLD_VALUE = 1750;       ///< arm 判定阈值 [PWM]
    static constexpr int GEAR_SHIFT_UP_THRESHOLD = 1750;   ///< gear 上阈值 [PWM]
    static constexpr int GEAR_SHIFT_DOWN_THRESHOLD = 1250; ///< gear 下阈值 [PWM]
    static constexpr int API_MODE_THRESHOLD_VALUE = 1750;  ///< offboard 模式切换阈值 [PWM]
    static constexpr double DEAD_ZONE = 0.25;              ///< 摇杆死区 (归一化值)

    RC_Data_t();

    /** @brief 检查 PWM 有效性 (mode/gear 在 [900, 2100] 范围)。 */
    void check_validity();

    /** @brief 检查摇杆是否回中。 */
    bool check_centered();

    /** @brief RC 消息回调：归一化 + 状态机更新。 */
    void feed(mavros_msgs::RCInConstPtr pMsg);

    /** @brief 是否在超时窗口内收到数据（默认 0.5s）。 */
    bool is_received(const ros::Time &now_time);
};

// ============================================================================
// 里程计数据 (Odom_Data_t)
// ============================================================================
/** @brief 里程计位姿/速度缓存 — 从 nav_msgs/Odometry 提取 Eigen 表示。 */
class Odom_Data_t
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  Eigen::Vector3d p;    ///< 位置 [m] (里程计系)
  Eigen::Vector3d v;    ///< 速度 [m/s] (里程计系)
  Eigen::Quaterniond q; ///< 姿态四元数
  Eigen::Vector3d w;    ///< 角速度 [rad/s]（未使用，预留）

  nav_msgs::Odometry msg;  ///< 原始里程计消息
  ros::Time rcv_stamp;     ///< 最近一帧接收时间
  bool recv_new_msg;       ///< 是否收到过消息

  Odom_Data_t();

  /** @brief 里程计消息回调。 */
  void feed(nav_msgs::OdometryConstPtr pMsg);

  /** @brief 是否在超时窗口内收到数据（默认 0.5s）。 */
  bool is_received(const ros::Time &now_time);
};

// ============================================================================
// 飞控状态数据 (State_Data_t)
// ============================================================================
/** @brief PX4 飞控状态缓存 — connected/armed/mode 等。 */
class State_Data_t
{
public:
    mavros_msgs::State current_state;          ///< 当前飞控状态
    mavros_msgs::State state_before_offboard;  ///< 进入 offboard 前的状态（预留）

    ros::Time rcv_stamp;  ///< 最近一帧接收时间

    State_Data_t();

    /** @brief 飞控状态消息回调。 */
    void feed(mavros_msgs::StateConstPtr pMsg);

    /** @brief 是否在超时窗口内收到数据（默认 0.5s）。 */
    bool is_received(const ros::Time &now_time);
};

// ============================================================================
// 电池数据 (Battery_Data_t)
// ============================================================================
/** @brief 电池状态缓存 — 电压/电流等。 */
class Battery_Data_t
{
public:
    sensor_msgs::BatteryState battery;  ///< 电池状态
    ros::Time rcv_stamp;               ///< 最近一帧接收时间

    Battery_Data_t();

    /** @brief 电池状态消息回调。 */
    void feed(sensor_msgs::BatteryStateConstPtr pMsg);

    /** @brief 是否在超时窗口内收到数据（默认 0.5s）。 */
    bool is_received(const ros::Time &now_time);
};

// ============================================================================
// 轨迹指令数据 (Command_Data_t)
// ============================================================================
/** @brief 外部轨迹规划器指令缓存 — 位置/速度/加速度/Jerk 全状态。 */
class Command_Data_t
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    Eigen::Vector3d p;  ///< 目标位置 [m]
    Eigen::Vector3d v;  ///< 目标速度 [m/s]
    Eigen::Vector3d a;  ///< 目标加速度 [m/s²]
    Eigen::Vector3d j;  ///< 目标 jerk [m/s³]
    double yaw;         ///< 目标偏航角 [rad]
    double yaw_rate;    ///< 目标偏航角速率 [rad/s]

    quadrotor_msgs::PositionCommand msg;  ///< 原始指令消息
    ros::Time rcv_stamp;                  ///< 最近一帧接收时间

    Command_Data_t();

    /** @brief 轨迹指令消息回调。 */
    void feed(quadrotor_msgs::PositionCommandConstPtr pMsg);

    /** @brief 是否在超时窗口内收到数据（默认 0.1s）。 */
    bool is_received(const ros::Time &now_time);
};


#endif
