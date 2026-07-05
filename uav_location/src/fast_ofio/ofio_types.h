/**
 * @file ofio_types.h
 * @brief Fast Optical-Flow Inertial Odometry (fast_ofio) — 数据类型与配置参数定义。
 *
 * @details
 * 本文件定义了 fast_ofio 节点所需的：
 *   - 光流传感器输入数据结构（从 MAVROS OpticalFlowRad 消息提取）
 *   - IMU 输入数据结构（从 sensor_msgs/Imu 提取）
 *   - 里程计估计器状态结构
 *   - 所有可调配置参数及其默认值
 *
 * 参考：PX4 EKF2 optical flow 积分算法
 *   - src/modules/ekf2/EKF/optflow_control.cpp
 */

#ifndef OFIO_TYPES_H
#define OFIO_TYPES_H

#include <Eigen/Dense>
#include <ros/ros.h>

namespace fast_ofio
{

// ============================================================================
// 光流传感器输入数据（从 MAVROS mavros_msgs::OpticalFlowRad 提取）
// ============================================================================
/** @brief 单帧光流传感器测量数据。
 *  @note 参考 PX4 MAVLink 消息 OPTICAL_FLOW_RAD (#106)。 */
struct OpticalFlowMeasurement
{
    ros::Time stamp;

    double integration_time_us;     // 积分时间 [us]
    double integrated_x;            // X 轴积分光流 [rad]
    double integrated_y;            // Y 轴积分光流 [rad]
    double integrated_xgyro;        // X 轴积分陀螺 [rad]
    double integrated_ygyro;        // Y 轴积分陀螺 [rad]
    double integrated_zgyro;        // Z 轴积分陀螺 [rad]
    double distance;                // 到表面距离 [m]（-1 表示无效）
    double temperature;             // 温度 [°C]
    uint8_t quality;                // 质量 (0–255)
};

// ============================================================================
// IMU 输入数据（从 sensor_msgs::Imu 提取）
// ============================================================================
/** @brief IMU 状态快照。角速度与加速度均在 body 系下表达。MAVROS 已转 ENU。 */
struct ImuMeasurement
{
    ros::Time stamp;

    Eigen::Vector3d angular_velocity;    // [rad/s] (body frame)
    Eigen::Vector3d linear_acceleration; // [m/s²] (body frame)
    Eigen::Quaterniond orientation;      // body → world (ENU)
};

// ============================================================================
// 里程计估计器状态
// ============================================================================
/** @brief 光流-惯导里程计估计器内部状态。 */
struct EstimatorState
{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    Eigen::Vector3d position;           // 世界系位置 [m] (ENU)
    Eigen::Vector3d velocity;           // 世界系速度 [m/s]
    Eigen::Quaterniond orientation;     // body → world 姿态

    Eigen::Vector2d flow_bias;          // 光流偏置 [rad]（陀螺零偏等慢变误差）

    double height;                      // 距地高度 [m]
    double height_velocity;             // 垂直速度 [m/s]

    ros::Time last_update_time;
    ros::Time last_flow_time;
    ros::Time last_imu_time;

    bool initialized;

    EstimatorState()
    {
        position.setZero();
        velocity.setZero();
        orientation.setIdentity();
        flow_bias.setZero();
        height = 0.0;
        height_velocity = 0.0;
        last_update_time = ros::Time(0);
        last_flow_time = ros::Time(0);
        last_imu_time = ros::Time(0);
        initialized = false;
    }
};

// ============================================================================
// Range 分类状态（两套高度语义的核心）
// ============================================================================
/** @brief Range 传感器观测分类。
 *  - NORMAL:   地面正常可见，range 可用于 XY 光流尺度和 global Z 修正。
 *  - OBSTACLE: 检测到障碍物出现/消失，range 不可用于 global Z 修正；
 *              XY 光流尺度使用 fallback range，不更新偏置。
 *  - INVALID:  range 完全不可用（超量程/无效值），仅做预测传播。 */
enum class RangeStatus { NORMAL, OBSTACLE, INVALID };

// ============================================================================
// 估计器配置参数
// ============================================================================
/**
 * @brief fast_ofio 估计器全部可调参数。所有参数均支持 ROS param 配置。
 *
 * 参考 PX4 对应参数：
 *   - SENS_FLOW_ROT   → flow_rotation
 *   - SENS_FLOW_SCL   → scale_factor
 *   - SENS_FLOW_MAXR  → flow_max_rate
 *   - SENS_FLOW_MINH  → min_height
 *   - SENS_FLOW_MAXH  → max_height
 *   - EKF2_OF_QMIN    → quality_min
 *   - EKF2_OF_N_MIN   → flow_noise
 *   - SENS_FLOW_POSX/Y/Z → flow_pos_x/y/z  (光流相对IMU的安装偏移)
 */
struct OfioConfig
{
    // ---- 话题名称 ----
    std::string flow_topic;       // 默认 /mavros/optical_flow_rad/raw/send
    std::string imu_topic;        // 默认 /mavros/imu/data
    std::string odom_topic;       // 默认 /Odometry
    std::string world_frame_id;   // 默认 "map"
    std::string body_frame_id;    // 默认 "base_link"
    std::string odom_frame_id;    // 默认 "odom" (里程计发布坐标系)

    // ---- 里程计坐标系模式 ----
    bool use_body_frame_odom;     // 默认 true: body-aligned FLU 里程计 (X=前 Y=左 Z=上)
                                  // false: world ENU (X=东 Y=北 Z=上), 随yaw旋转

    // ---- Range 初始偏移 ----
    bool auto_range_offset;       // 默认 true: 自动识别初始离地距离并扣除

    // ---- 光流传感器安装位置偏移（相对IMU/机体原点，body系） ----
    double flow_pos_x;            // 光流 X 偏移 [m]（前为正）
    double flow_pos_y;            // 光流 Y 偏移 [m]（右为正）
    double flow_pos_z;            // 光流 Z 偏移 [m]（下为正，通常为正值表示低于IMU）

    // ---- 光流传感器标定 ----
    double scale_factor;          // 光流尺度因子（>1 表示读数偏小）
    double flow_rotation;         // 光流传感器绕 Z 轴旋转安装角 [rad]

    // ---- 质量与有效性门限 ----
    int    quality_min;           // 最低质量阈值 (0–255)
    double flow_max_rate;         // 最大有效光流角速率 [rad/s]
    double min_height;            // 最小有效测距高度 [m]
    double max_height;            // 最大有效测距高度 [m]
    double max_gyro_rate;         // 静止判定角速度阈值 [rad/s]

    // ---- Z 轴障碍物检测 ----
    double obstacle_z_threshold;              // 障碍物判定 innovation 阈值 [m]
    double z_imu_stationary_threshold;        // IMU 定高判定阈值 [m/s²]
    int    obstacle_confirm_frames;           // 连续确认 OBSTACLE 所需帧数
    int    obstacle_recover_frames;           // 连续确认 NORMAL 恢复所需帧数
    double obstacle_recovery_threshold;       // 恢复时的 innovation 阈值 [m]（应小于 obstacle_z_threshold）
    double z_velocity_decay_tau;              // OBSTACLE 下高度速度衰减时间常数 [s]
    double max_z_prediction_time;             // 最大无 range 校正预测时间 [s]

    // ---- 噪声与协方差参数 ----
    double flow_noise;            // 光流基础观测噪声 [rad/s]
    double flow_noise_qual_min;   // 光流质量关联噪声系数 [(rad/s)/quality]
    double gyro_noise;            // 陀螺噪声密度 [rad/s/√Hz]

    // ---- 漂移与偏置估计 ----
    double bias_estimation_tau;   // 偏置估计低通时间常数 [s]
    double bias_max;              // 偏置上限截断值 [rad]

    // ---- 低通滤波 ----
    double velocity_lpf_cutoff;   // 速度低通截止频率 [Hz]
    double height_lpf_cutoff;     // 高度低通截止频率 [Hz]

    // ---- 时间同步 ----
    double max_flow_imu_dt;       // 光流与 IMU 最大允许时间差 [s]
    double flow_timeout;          // 光流数据超时时间 [s]

    // ---- 初始位置 ----
    double initial_x;             // 初始 X [m]
    double initial_y;             // 初始 Y [m]
    double initial_z;             // 初始 Z [m]
    double initial_yaw;           // 初始偏航 [rad]

    // ---- 发布频率 ----
    double publish_rate;          // [Hz]

    /** @brief 默认构造函数。 */
    OfioConfig()
    {
        flow_topic      = "/mavros/optical_flow_rad/raw/send";
        imu_topic       = "/mavros/imu/data";
        odom_topic      = "/Odometry";
        world_frame_id  = "map";
        body_frame_id   = "base_link";
        odom_frame_id   = "odom";

        use_body_frame_odom = true;
        auto_range_offset   = true;

        flow_pos_x      = 0.0;
        flow_pos_y      = 0.0;
        flow_pos_z      = 0.0;

        scale_factor    = 1.0;
        flow_rotation   = 0.0;

        quality_min     = 50;
        flow_max_rate   = 2.5;
        min_height      = 0.1;
        max_height      = 30.0;
        max_gyro_rate   = 0.05;

        obstacle_z_threshold          = 0.3;
        z_imu_stationary_threshold    = 0.5;
        obstacle_confirm_frames       = 3;
        obstacle_recover_frames       = 5;
        obstacle_recovery_threshold   = 0.2;
        z_velocity_decay_tau          = 2.0;
        max_z_prediction_time         = 5.0;

        flow_noise          = 0.05;
        flow_noise_qual_min = 0.001;
        gyro_noise          = 0.001;

        bias_estimation_tau = 10.0;
        bias_max            = 0.1;

        velocity_lpf_cutoff = 5.0;
        height_lpf_cutoff   = 2.0;

        max_flow_imu_dt     = 0.05;
        flow_timeout        = 0.5;

        initial_x   = 0.0;
        initial_y   = 0.0;
        initial_z   = 0.0;
        initial_yaw = 0.0;

        publish_rate = 30.0;
    }

    /** @brief 从 ROS 参数服务器加载配置。 */
    void loadFromParamServer(ros::NodeHandle& nh)
    {
        nh.param<std::string>("flow_topic",       flow_topic,       flow_topic);
        nh.param<std::string>("imu_topic",        imu_topic,        imu_topic);
        nh.param<std::string>("odom_topic",       odom_topic,       odom_topic);
        nh.param<std::string>("world_frame_id",   world_frame_id,   world_frame_id);
        nh.param<std::string>("body_frame_id",    body_frame_id,    body_frame_id);
        nh.param<std::string>("odom_frame_id",    odom_frame_id,    odom_frame_id);

        nh.param<bool>("use_body_frame_odom",     use_body_frame_odom, use_body_frame_odom);
        nh.param<bool>("auto_range_offset",       auto_range_offset,   auto_range_offset);

        nh.param<double>("flow_pos_x",            flow_pos_x,       flow_pos_x);
        nh.param<double>("flow_pos_y",            flow_pos_y,       flow_pos_y);
        nh.param<double>("flow_pos_z",            flow_pos_z,       flow_pos_z);

        nh.param<double>("scale_factor",          scale_factor,     scale_factor);
        nh.param<double>("flow_rotation",         flow_rotation,    flow_rotation);

        nh.param<int>("quality_min",              quality_min,      quality_min);
        nh.param<double>("flow_max_rate",         flow_max_rate,    flow_max_rate);
        nh.param<double>("min_height",            min_height,       min_height);
        nh.param<double>("max_height",            max_height,       max_height);
        nh.param<double>("max_gyro_rate",         max_gyro_rate,    max_gyro_rate);

        nh.param<double>("obstacle_z_threshold",       obstacle_z_threshold,       obstacle_z_threshold);
        nh.param<double>("z_imu_stationary_threshold", z_imu_stationary_threshold, z_imu_stationary_threshold);
        nh.param<int>("obstacle_confirm_frames",       obstacle_confirm_frames,    obstacle_confirm_frames);
        nh.param<int>("obstacle_recover_frames",       obstacle_recover_frames,    obstacle_recover_frames);
        nh.param<double>("obstacle_recovery_threshold",obstacle_recovery_threshold,obstacle_recovery_threshold);
        nh.param<double>("z_velocity_decay_tau",       z_velocity_decay_tau,       z_velocity_decay_tau);
        nh.param<double>("max_z_prediction_time",      max_z_prediction_time,      max_z_prediction_time);

        nh.param<double>("flow_noise",            flow_noise,       flow_noise);
        nh.param<double>("flow_noise_qual_min",   flow_noise_qual_min, flow_noise_qual_min);
        nh.param<double>("gyro_noise",            gyro_noise,       gyro_noise);

        nh.param<double>("bias_estimation_tau",   bias_estimation_tau, bias_estimation_tau);
        nh.param<double>("bias_max",              bias_max,         bias_max);

        nh.param<double>("velocity_lpf_cutoff",   velocity_lpf_cutoff, velocity_lpf_cutoff);
        nh.param<double>("height_lpf_cutoff",     height_lpf_cutoff, height_lpf_cutoff);

        nh.param<double>("max_flow_imu_dt",       max_flow_imu_dt,  max_flow_imu_dt);
        nh.param<double>("flow_timeout",          flow_timeout,     flow_timeout);

        nh.param<double>("initial_x",             initial_x,        initial_x);
        nh.param<double>("initial_y",             initial_y,        initial_y);
        nh.param<double>("initial_z",             initial_z,        initial_z);
        nh.param<double>("initial_yaw",           initial_yaw,      initial_yaw);

        nh.param<double>("publish_rate",          publish_rate,     publish_rate);
    }
};

} // namespace fast_ofio

#endif // OFIO_TYPES_H
