/**
 * @file ofio_estimator.cpp
 * @brief Fast Optical-Flow Inertial Odometry (fast_ofio) — 核心估计器实现。
 *
 * 算法流水线（参考 PX4 EKF2 optflow_control.cpp）：
 *   光流帧 → 质量门限 → 旋转补偿 → 尺度/偏置校正
 *     → 高度补偿(flow→velocity) → 杠杆臂补偿(v_sensor→v_body)
 *     → 坐标变换 → 位移积分 → 协方差传播 → 偏置估计(静止判定)
 */

#include "ofio_estimator.h"
#include <cmath>
#include <ros/ros.h>

namespace fast_ofio
{

// ============================================================================
// 构造与重置
// ============================================================================

OfioEstimator::OfioEstimator(const OfioConfig& config)
    : config_(config)
    , imu_received_(false)
    , initial_range_offset_(0.0)
    , range_offset_captured_(false)
    , last_trusted_flow_range_(0.0)
    , last_valid_height_velocity_(0.0)
    , imu_z_acc_lpf_(0.0)
    , z_prediction_time_(0.0)
    , range_status_(RangeStatus::INVALID)
    , obstacle_frame_count_(0)
    , recovery_frame_count_(0)
    , velocity_lpf_state_(Eigen::Vector3d::Zero())
    , pose_covariance_(Eigen::Matrix<double, 6, 6>::Identity() * 0.01)
    , twist_covariance_(Eigen::Matrix<double, 6, 6>::Identity() * 0.01)
    , flow_timed_out_(false)
    , is_stationary_(false)
{
    state_.position.x() = config_.initial_x;
    state_.position.y() = config_.initial_y;
    state_.position.z() = config_.initial_z;
    state_.height = config_.initial_z;

    Eigen::AngleAxisd yaw_rot(config_.initial_yaw, Eigen::Vector3d::UnitZ());
    state_.orientation = Eigen::Quaterniond(yaw_rot);
    initial_orientation_ = state_.orientation;

    ROS_INFO("[fast_ofio] Estimator init: pos=(%.2f,%.2f,%.2f) yaw=%.1f deg"
             "  flow_offset=(%.3f,%.3f,%.3f) m"
             "  range_offset=%s  odom_mode=%s"
             "  obstacle: thresh=%.2fm confirm=%d recover=%d decay_tau=%.1fs",
             config_.initial_x, config_.initial_y, config_.initial_z,
             config_.initial_yaw * 180.0 / M_PI,
             config_.flow_pos_x, config_.flow_pos_y, config_.flow_pos_z,
             config_.auto_range_offset ? "auto" : "manual",
             config_.use_body_frame_odom ? "FLU(body-aligned)" : "ENU(world)",
             config_.obstacle_z_threshold, config_.obstacle_confirm_frames,
             config_.obstacle_recover_frames, config_.z_velocity_decay_tau);
}

void OfioEstimator::reset()
{
    state_ = EstimatorState();
    state_.position.x() = config_.initial_x;
    state_.position.y() = config_.initial_y;
    state_.position.z() = config_.initial_z;
    state_.height = config_.initial_z;

    Eigen::AngleAxisd yaw_rot(config_.initial_yaw, Eigen::Vector3d::UnitZ());
    state_.orientation = Eigen::Quaterniond(yaw_rot);
    initial_orientation_ = state_.orientation;

    initial_range_offset_        = 0.0;
    range_offset_captured_       = false;
    last_trusted_flow_range_     = 0.0;
    last_valid_height_velocity_  = 0.0;
    imu_z_acc_lpf_               = 0.0;
    z_prediction_time_           = 0.0;
    range_status_                = RangeStatus::INVALID;
    obstacle_frame_count_        = 0;
    recovery_frame_count_        = 0;

    velocity_lpf_state_.setZero();
    pose_covariance_   = Eigen::Matrix<double, 6, 6>::Identity() * 0.01;
    twist_covariance_  = Eigen::Matrix<double, 6, 6>::Identity() * 0.01;
    flow_timed_out_    = false;
    is_stationary_     = false;
    imu_received_      = false;

    ROS_INFO("[fast_ofio] Estimator reset.");
}

void OfioEstimator::setInitialPose(double x, double y, double z, double yaw)
{
    state_.position.x() = x;
    state_.position.y() = y;
    state_.position.z() = z;
    state_.height = z;

    Eigen::AngleAxisd yaw_rot(yaw, Eigen::Vector3d::UnitZ());
    state_.orientation = Eigen::Quaterniond(yaw_rot);
    initial_orientation_ = state_.orientation;

    config_.initial_x = x;
    config_.initial_y = y;
    config_.initial_z = z;
    config_.initial_yaw = yaw;

    ROS_INFO("[fast_ofio] Pose set: (%.2f,%.2f,%.2f) yaw=%.1f deg",
             x, y, z, yaw * 180.0 / M_PI);
}

// ============================================================================
// 数据输入
// ============================================================================

void OfioEstimator::updateImu(const ImuMeasurement& imu)
{
    latest_imu_ = imu;
    imu_received_ = true;
    state_.last_imu_time = imu.stamp;
}

void OfioEstimator::updateOpticalFlow(const OpticalFlowMeasurement& flow)
{
    // 0. 前置检查
    if (!imu_received_)
    {
        ROS_WARN_THROTTLE(5.0,
            "[fast_ofio] Flow received but no IMU yet. Skipping.");
        return;
    }

    // 1. 时间同步
    double flow_imu_dt = (flow.stamp - latest_imu_.stamp).toSec();
    if (std::abs(flow_imu_dt) > config_.max_flow_imu_dt)
    {
        ROS_WARN_THROTTLE(5.0,
            "[fast_ofio] Flow-IMU dt=%.3fs > %.3fs. Skip.",
            std::abs(flow_imu_dt), config_.max_flow_imu_dt);
        return;
    }

    // 2. 积分时间 (us → s)
    double dt = flow.integration_time_us * 1e-6;
    if (dt <= 0.0 || dt > 1.0)
    {
        ROS_WARN_THROTTLE(5.0,
            "[fast_ofio] Invalid dt=%.6fs. Skip.", dt);
        return;
    }

    // 3. 质量门限（仅 quality + distance > 0，range 量程约束移至 classifyRange）
    if (!checkQualityGate(flow, dt))
        return;

    // === 3.5. Range 初始偏移首次捕获（必须在 classifyRange/updateHeight 之前） ===
    if (config_.auto_range_offset && !range_offset_captured_)
    {
        initial_range_offset_ = flow.distance;
        range_offset_captured_ = true;
        ROS_INFO("[fast_ofio] Range offset captured: %.3f m. "
                 "Effective Z will start at 0.",
                 initial_range_offset_);
    }

    // 4. 计算 effective_range（已扣除初始偏移）
    double effective_range = flow.distance;
    if (config_.auto_range_offset && range_offset_captured_)
    {
        effective_range = flow.distance - initial_range_offset_;
    }

    // 5. Range 三分类 — NORMAL / OBSTACLE / INVALID
    range_status_ = classifyRange(effective_range, dt, flow.distance);

    // 6. 确定 flow_range_for_xy（用于光流速度尺度计算）
    double flow_range_for_xy = flow.distance;
    bool   skip_xy_update   = false;

    switch (range_status_)
    {
    case RangeStatus::NORMAL:
        flow_range_for_xy = flow.distance;
        last_trusted_flow_range_ = flow.distance;
        break;
    case RangeStatus::OBSTACLE:
        if (last_trusted_flow_range_ > 0.0)
        {
            flow_range_for_xy = last_trusted_flow_range_;
            ROS_DEBUG_THROTTLE(1.0,
                "[fast_ofio] OBSTACLE: using fallback flow_range=%.3f (raw=%.3f)",
                last_trusted_flow_range_, flow.distance);
        }
        else
        {
            skip_xy_update = true;
            ROS_DEBUG_THROTTLE(1.0,
                "[fast_ofio] OBSTACLE: no trusted flow range, skipping XY.");
        }
        break;
    case RangeStatus::INVALID:
        skip_xy_update = true;
        ROS_DEBUG_THROTTLE(1.0,
            "[fast_ofio] INVALID range: skipping XY+Z update.");
        break;
    }

    // 7. 旋转补偿：flow_trans = flow_meas - gyro_integrated
    Eigen::Vector2d flow_trans = compensateRotation(flow);

    // 8. 尺度与安装角校正
    Eigen::Vector2d flow_scaled = applyScale(flow_trans);

    // 9. 偏置补偿
    Eigen::Vector2d flow_unbiased = compensateBias(flow_scaled);

    // 10. 光流角速率上限检查
    double rate_x = std::abs(flow_unbiased.x()) / dt;
    double rate_y = std::abs(flow_unbiased.y()) / dt;
    if (rate_x > config_.flow_max_rate || rate_y > config_.flow_max_rate)
    {
        ROS_WARN_THROTTLE(5.0,
            "[fast_ofio] Flow rate (%.3f,%.3f) > %.3f rad/s. Skip.",
            rate_x, rate_y, config_.flow_max_rate);
        return;
    }

    // 11. XY 速度更新（使用 flow_range_for_xy）
    if (!skip_xy_update)
    {
        // 高度补偿：角速率光流 → 传感器处 body 系线速度
        Eigen::Vector3d v_sensor = flowToVelocity(flow_unbiased, flow_range_for_xy, dt);

        // 杠杆臂补偿：修正传感器偏移 (v_body = v_sensor - ω × r)
        Eigen::Vector3d v_body = compensateLeverArm(v_sensor);

        // 坐标变换：body → world
        Eigen::Vector3d v_world = transformToWorld(v_body);

        // 速度低通滤波
        Eigen::Vector3d v_filtered = applyLowPassFilter(v_world, dt);

        // 状态更新
        state_.velocity = v_filtered;

        // 位移积分
        integratePosition(v_filtered, dt);
    }
    else
    {
        // 跳过 XY 更新 — 仅保持当前速度不变（协方差将在 updateCovariance 中增大）
    }

    // 12. 高度更新（两套语义核心）
    updateHeight(effective_range, dt, range_status_);

    // 13. 偏置估计（仅在 NORMAL 时更新）
    Eigen::Vector2d flow_raw(flow.integrated_x, flow.integrated_y);
    updateBias(flow_raw, dt, range_status_);

    // 14. 协方差（使用实际 flow_range_for_xy）
    double cov_range = skip_xy_update ? last_trusted_flow_range_ : flow_range_for_xy;
    updateCovariance(static_cast<double>(flow.quality), cov_range, dt, range_status_);

    // 15. 姿态与时间戳
    state_.orientation = latest_imu_.orientation;
    state_.last_flow_time = flow.stamp;
    state_.last_update_time = flow.stamp;

    if (!state_.initialized)
    {
        state_.initialized = true;
        ROS_INFO("[fast_ofio] Initialized. quality=%d range=%.2fm dt=%.1fms.",
                 flow.quality, flow.distance, dt * 1000.0);
    }
}

// ============================================================================
// 质量门限
// ============================================================================

bool OfioEstimator::checkQualityGate(const OpticalFlowMeasurement& flow, double dt) const
{
    (void)dt;
    if (static_cast<int>(flow.quality) < config_.quality_min)
    {
        ROS_DEBUG_THROTTLE(5.0,
            "[fast_ofio] Quality %d < %d.", flow.quality, config_.quality_min);
        return false;
    }
    if (flow.distance <= 0.0)
    {
        ROS_DEBUG_THROTTLE(5.0, "[fast_ofio] Invalid distance %.3f.", flow.distance);
        return false;
    }
    // range 量程约束 (min_height/max_height) 移至 classifyRange() 返回 INVALID
    return true;
}

// ============================================================================
// Range 三分类（两套高度语义核心）
// ============================================================================

RangeStatus OfioEstimator::classifyRange(double effective_range, double dt, double raw_distance)
{
    // 0. 基础有效性检查（超出量程 → INVALID）
    if (raw_distance < config_.min_height || raw_distance > config_.max_height)
    {
        obstacle_frame_count_ = 0;
        recovery_frame_count_ = 0;
        return RangeStatus::INVALID;
    }

    // 1. 计算 predicted global Z（短时预测）
    double pred_z = state_.height + state_.height_velocity * dt
                  + 0.5 * imu_z_acc_lpf_ * dt * dt;

    // 2. Innovation = 当前观测 - 预测
    double innovation = effective_range - pred_z;
    double abs_innovation = std::abs(innovation);

    // 3. IMU 是否支持该高度变化
    bool imu_supports = (std::abs(imu_z_acc_lpf_) > config_.z_imu_stationary_threshold)
                     && (innovation * imu_z_acc_lpf_ > 0.0);  // 同号

    // 4. 状态机
    if (range_status_ == RangeStatus::OBSTACLE)
    {
        // 处在 OBSTACLE — 检查恢复条件
        if (abs_innovation < config_.obstacle_recovery_threshold)
        {
            recovery_frame_count_++;
            if (recovery_frame_count_ >= config_.obstacle_recover_frames)
            {
                ROS_INFO("[fast_ofio] OBSTACLE cleared → NORMAL. "
                         "innovation=%.3f frames=%d",
                         innovation, recovery_frame_count_);
                obstacle_frame_count_ = 0;
                recovery_frame_count_ = 0;
                return RangeStatus::NORMAL;
            }
        }
        else
        {
            recovery_frame_count_ = 0;
        }
        // IMU 现在支持变化 + innovation 同向 → 可能是真的升降
        if (imu_supports && innovation * state_.height_velocity >= 0.0)
        {
            ROS_INFO("[fast_ofio] OBSTACLE → NORMAL (IMU supports). "
                     "innovation=%.3f imu_acc_z=%.3f",
                     innovation, imu_z_acc_lpf_);
            obstacle_frame_count_ = 0;
            recovery_frame_count_ = 0;
            return RangeStatus::NORMAL;
        }
        return RangeStatus::OBSTACLE;
    }

    // 5. 非 OBSTACLE → 检查是否需要进入 OBSTACLE
    if (abs_innovation > config_.obstacle_z_threshold && !imu_supports)
    {
        obstacle_frame_count_++;
        recovery_frame_count_ = 0;
        if (obstacle_frame_count_ >= config_.obstacle_confirm_frames)
        {
            ROS_WARN("[fast_ofio] → OBSTACLE: innovation=%.3f (pred=%.3f obs=%.3f) "
                     "imu_acc_z=%.3f frames=%d",
                     innovation, pred_z, effective_range,
                     imu_z_acc_lpf_, obstacle_frame_count_);
            return RangeStatus::OBSTACLE;
        }
    }
    else
    {
        obstacle_frame_count_ = 0;
    }

    return RangeStatus::NORMAL;
}

// ============================================================================
// 旋转补偿
// ============================================================================

Eigen::Vector2d OfioEstimator::compensateRotation(const OpticalFlowMeasurement& flow) const
{
    // PX4: flow_comp = flow_meas - gyro_integrated
    // 传感器内置陀螺在同一积分窗口内测得旋转角增量，直接相减分离旋转分量
    return Eigen::Vector2d(
        flow.integrated_x - flow.integrated_xgyro,
        flow.integrated_y - flow.integrated_ygyro);
}

// ============================================================================
// 尺度与旋转安装角校正
// ============================================================================

Eigen::Vector2d OfioEstimator::applyScale(const Eigen::Vector2d& flow_trans) const
{
    double cos_r = std::cos(config_.flow_rotation);
    double sin_r = std::sin(config_.flow_rotation);

    Eigen::Vector2d flow_rotated;
    flow_rotated.x() =  cos_r * flow_trans.x() + sin_r * flow_trans.y();
    flow_rotated.y() = -sin_r * flow_trans.x() + cos_r * flow_trans.y();

    return flow_rotated * config_.scale_factor;
}

// ============================================================================
// 偏置补偿
// ============================================================================

Eigen::Vector2d OfioEstimator::compensateBias(const Eigen::Vector2d& flow_scaled) const
{
    return flow_scaled - state_.flow_bias;
}

// ============================================================================
// 高度补偿：光流 → body 系线速度（传感器所在位置的速度）
// ============================================================================

Eigen::Vector3d OfioEstimator::flowToVelocity(
    const Eigen::Vector2d& flow_unbiased, double range, double dt) const
{
    // 物理模型（参考 PX4 EKF2）：
    //   向下安装的传感器，特征点运动与光流的关系为：
    //     flow_x = atan2(v_y × dt, range)  →  v_y = range × tan(flow_x) / dt
    //     flow_y = atan2(-v_x × dt, range) →  v_x = -range × tan(flow_y) / dt
    //
    // 小角度（|flow| < 0.1 rad）使用线性近似：
    //     v_x = -flow_y × range / dt
    //     v_y =  flow_x × range / dt

    const double kSmallAngleThresh = 0.1;
    double flow_x = flow_unbiased.x();
    double flow_y = flow_unbiased.y();
    double vx, vy;

    if (std::abs(flow_x) < kSmallAngleThresh && std::abs(flow_y) < kSmallAngleThresh)
    {
        // 线性模型
        vx = -flow_y * range / dt;
        vy =  flow_x * range / dt;
    }
    else
    {
        // 非线性精确模型
        vx = -range * std::tan(flow_y) / dt;
        vy =  range * std::tan(flow_x) / dt;
    }

    return Eigen::Vector3d(vx, vy, 0.0);
}

// ============================================================================
// 杠杆臂补偿：v_body = v_sensor - ω × r_offset
// ============================================================================

Eigen::Vector3d OfioEstimator::compensateLeverArm(const Eigen::Vector3d& v_sensor) const
{
    // 光流传感器安装在机体偏移位置 r = (flow_pos_x, flow_pos_y, flow_pos_z)。
    // 传感器处测量的地面速度包含了旋转导致的杠杆臂速度：
    //   v_sensor = v_body + ω_body × r_offset
    //
    // 因此机体原点（IMU 中心）的真实速度为：
    //   v_body = v_sensor - ω_body × r_offset
    //
    // 在 body 系中展开叉乘：
    //   (ω × r)_x = ω_y × r_z - ω_z × r_y
    //   (ω × r)_y = ω_z × r_x - ω_x × r_z
    //   (ω × r)_z = ω_x × r_y - ω_y × r_x

    Eigen::Vector3d omega = latest_imu_.angular_velocity;
    Eigen::Vector3d r(config_.flow_pos_x, config_.flow_pos_y, config_.flow_pos_z);

    Eigen::Vector3d v_lever = omega.cross(r);   // ω × r

    return v_sensor - v_lever;
}

// ============================================================================
// 坐标变换
// ============================================================================

Eigen::Vector3d OfioEstimator::transformToWorld(const Eigen::Vector3d& v_body) const
{
    if (config_.use_body_frame_odom)
    {
        // Body-aligned FLU 里程计坐标系 (X=前 Y=左 Z=上)
        // body FRD → FLU: (X, Y, Z)_FLU = (X, -Y, Z)   (Z 由 updateHeight 单独处理)
        // 不旋转到 world ENU，保持与初始机体朝向对齐
        return Eigen::Vector3d(v_body.x(), -v_body.y(), v_body.z());
    }
    // 世界 ENU 模式：v_world = R_body2world × v_body
    return state_.orientation * v_body;
}

// ============================================================================
// 位移积分
// ============================================================================

void OfioEstimator::integratePosition(const Eigen::Vector3d& v_world, double dt)
{
    state_.position.x() += v_world.x() * dt;
    state_.position.y() += v_world.y() * dt;
    // Z 由 updateHeight() 独立处理
}

// ============================================================================
// 高度更新：两套语义核心
// ============================================================================

void OfioEstimator::updateHeight(double effective_range, double dt, RangeStatus status)
{
    if (dt <= 0.0) return;

    // ---- 持续更新 IMU 垂向加速度低通值 ----
    Eigen::Vector3d acc_world = state_.orientation * latest_imu_.linear_acceleration;
    double az_world = acc_world.z() - 9.81;  // 减重力 (ENU: +Z 朝上)
    double acc_lpf_tau = 0.2;
    double alpha_acc = std::min(dt / (dt + acc_lpf_tau), 1.0);
    imu_z_acc_lpf_ = (1.0 - alpha_acc) * imu_z_acc_lpf_ + alpha_acc * az_world;

    switch (status)
    {
    case RangeStatus::NORMAL:
    {
        // === NORMAL: range 直接修正 global Z ===
        // Z 位置：LPF 平滑
        double z_lpf_tau = 1.0 / (2.0 * M_PI * config_.height_lpf_cutoff);
        double alpha_z = std::min(dt / (dt + z_lpf_tau), 1.0);
        double z_new = (1.0 - alpha_z) * state_.height + alpha_z * effective_range;

        // Z 速度：位置差分 + LPF
        double z_vel_diff = (z_new - state_.height) / dt;
        double alpha_vz = std::min(dt / (dt + 0.1), 1.0);  // 快速速度跟踪
        double vz_filtered = (1.0 - alpha_vz) * state_.height_velocity
                           + alpha_vz * z_vel_diff;

        // 写入状态
        state_.height = z_new;
        state_.height_velocity = vz_filtered;
        state_.position.z() = z_new;

        // 保存有效速度（OBSTACLE 下传播用）
        last_valid_height_velocity_ = vz_filtered;
        z_prediction_time_ = 0.0;
        break;
    }

    case RangeStatus::OBSTACLE:
    {
        // === OBSTACLE: 拒绝 range 校正，预测传播 Z ===
        // 检查最大预测时间
        z_prediction_time_ += dt;
        if (z_prediction_time_ > config_.max_z_prediction_time)
        {
            ROS_WARN_THROTTLE(1.0,
                "[fast_ofio] OBSTACLE prediction timeout: %.1fs. "
                "Z covariance growing rapidly.",
                z_prediction_time_);
            // 不转 INVALID，继续衰减传播但协方差已极大
        }

        predictGlobalZ(dt);

        // 速度衰减
        double decay = std::exp(-dt / config_.z_velocity_decay_tau);
        last_valid_height_velocity_ *= decay;

        state_.height_velocity = last_valid_height_velocity_;
        state_.position.z() = state_.height;
        break;
    }

    case RangeStatus::INVALID:
    {
        // === INVALID: 仅预测传播，不更新有效速度 ===
        z_prediction_time_ += dt;
        predictGlobalZ(dt);

        // 速度快速衰减
        double decay = std::exp(-dt / (config_.z_velocity_decay_tau * 0.5));
        last_valid_height_velocity_ *= decay;
        state_.height_velocity = last_valid_height_velocity_;
        state_.position.z() = state_.height;
        break;
    }
    }
}

void OfioEstimator::predictGlobalZ(double dt)
{
    // 用上一次有效速度 + IMU 加速度短时预测
    double vz_pred = last_valid_height_velocity_;

    // 前 0.5s 叠加 IMU 加速度（之后仅依赖速度衰减，避免 IMU 积分漂移）
    if (z_prediction_time_ < 0.5)
    {
        vz_pred += imu_z_acc_lpf_ * dt;
    }

    state_.height += vz_pred * dt;
}

// ============================================================================
// 偏置在线估计
// ============================================================================

void OfioEstimator::updateBias(const Eigen::Vector2d& flow_meas, double dt, RangeStatus status)
{
    // OBSTACLE/INVALID 时不更新偏置（异常光流或错误尺度会污染偏置估计）
    if (status != RangeStatus::NORMAL)
    {
        is_stationary_ = false;
        return;
    }

    // 静止判定（参考 PX4 EKF2 静止检测）：
    //   三轴角速度 < max_gyro_rate 且 速度 < 0.2 m/s
    Eigen::Vector3d gyro = latest_imu_.angular_velocity;
    bool gyro_low = (std::abs(gyro.x()) < config_.max_gyro_rate)
                 && (std::abs(gyro.y()) < config_.max_gyro_rate)
                 && (std::abs(gyro.z()) < config_.max_gyro_rate);
    bool vel_low = (state_.velocity.norm() < 0.2);

    is_stationary_ = gyro_low && vel_low;

    if (is_stationary_)
    {
        double gamma = std::min(std::max(dt / (dt + config_.bias_estimation_tau), 0.0), 1.0);

        state_.flow_bias.x() = (1.0 - gamma) * state_.flow_bias.x() + gamma * flow_meas.x();
        state_.flow_bias.y() = (1.0 - gamma) * state_.flow_bias.y() + gamma * flow_meas.y();

        // 限幅
        state_.flow_bias.x() = std::max(-config_.bias_max,
            std::min(config_.bias_max, state_.flow_bias.x()));
        state_.flow_bias.y() = std::max(-config_.bias_max,
            std::min(config_.bias_max, state_.flow_bias.y()));
    }
}

// ============================================================================
// 速度低通滤波
// ============================================================================

Eigen::Vector3d OfioEstimator::applyLowPassFilter(const Eigen::Vector3d& v_raw, double dt)
{
    double RC = 1.0 / (2.0 * M_PI * config_.velocity_lpf_cutoff);
    double alpha = std::min(std::max(dt / (dt + RC), 0.0), 1.0);
    velocity_lpf_state_ = alpha * v_raw + (1.0 - alpha) * velocity_lpf_state_;
    return velocity_lpf_state_;
}

// ============================================================================
// 协方差传播
// ============================================================================

void OfioEstimator::updateCovariance(double quality, double flow_range_for_xy,
                                      double dt, RangeStatus status)
{
    // 光流噪声模型（PX4 EKF2）：
    //   σ_flow = flow_noise + flow_noise_qual_min × (255 - quality) / 255
    double qual_norm = static_cast<double>(255 - static_cast<int>(quality)) / 255.0;
    double sigma_flow = config_.flow_noise
                      + config_.flow_noise_qual_min * std::max(0.0, qual_norm);

    // 速度噪声（使用实际 flow_range_for_xy 传播）：σ_v = (range/dt) × σ_flow
    double effective_range = (flow_range_for_xy > 0.0) ? flow_range_for_xy : 1.0;
    double sigma_v = (effective_range / dt) * sigma_flow;
    double speed = state_.velocity.head<2>().norm();
    double sigma_v_speed = sigma_v * (1.0 + 0.1 * speed);

    // 根据 range 状态缩放协方差
    double xy_scale = 1.0;
    double z_scale  = 1.0;
    switch (status)
    {
    case RangeStatus::OBSTACLE:
        xy_scale = 2.0;   // fallback range 不可靠
        z_scale  = 1.0 + z_prediction_time_ * 5.0;  // 随无校正时间线性增长
        break;
    case RangeStatus::INVALID:
        xy_scale = 5.0;
        z_scale  = 1.0 + z_prediction_time_ * 20.0;
        break;
    case RangeStatus::NORMAL:
    default:
        break;
    }

    twist_covariance_(0, 0) = sigma_v_speed * sigma_v_speed * xy_scale;
    twist_covariance_(1, 1) = sigma_v_speed * sigma_v_speed * xy_scale;
    twist_covariance_(2, 2) = 0.01 * z_scale;

    double sigma_p_inc = sigma_v_speed * dt;
    pose_covariance_(0, 0) += sigma_p_inc * sigma_p_inc * xy_scale;
    pose_covariance_(1, 1) += sigma_p_inc * sigma_p_inc * xy_scale;
    pose_covariance_(2, 2) += 0.02 * dt * 0.02 * dt * z_scale;

    double sigma_att = config_.gyro_noise * std::sqrt(dt);
    pose_covariance_(3, 3) += sigma_att * sigma_att;
    pose_covariance_(4, 4) += sigma_att * sigma_att;
    pose_covariance_(5, 5) += sigma_att * sigma_att;
}

// ============================================================================
// 超时监测
// ============================================================================

void OfioEstimator::checkTimeout(const ros::Time& now)
{
    if (!state_.initialized)
        return;

    double dt_since_flow = (now - state_.last_flow_time).toSec();
    if (dt_since_flow > config_.flow_timeout)
    {
        if (!flow_timed_out_)
        {
            ROS_WARN_THROTTLE(1.0,
                "[fast_ofio] Flow timeout: %.2fs. Position hold, cov growing.",
                dt_since_flow);
            flow_timed_out_ = true;
        }
        double sigma_grow = 0.01 * (dt_since_flow - config_.flow_timeout);
        pose_covariance_(0, 0) += sigma_grow;
        pose_covariance_(1, 1) += sigma_grow;
    }
    else
    {
        flow_timed_out_ = false;
    }
}

// ============================================================================
// 里程计消息组装
// ============================================================================

nav_msgs::Odometry OfioEstimator::getOdometry() const
{
    nav_msgs::Odometry odom;

    odom.header.stamp = state_.last_update_time;
    odom.header.frame_id = config_.odom_frame_id;
    odom.child_frame_id = config_.body_frame_id;

    // 位姿
    odom.pose.pose.position.x = state_.position.x();
    odom.pose.pose.position.y = state_.position.y();
    odom.pose.pose.position.z = state_.position.z();

    odom.pose.pose.orientation.w = state_.orientation.w();
    odom.pose.pose.orientation.x = state_.orientation.x();
    odom.pose.pose.orientation.y = state_.orientation.y();
    odom.pose.pose.orientation.z = state_.orientation.z();

    // 速度
    odom.twist.twist.linear.x = state_.velocity.x();
    odom.twist.twist.linear.y = state_.velocity.y();
    odom.twist.twist.linear.z = state_.height_velocity;

    odom.twist.twist.angular.x = latest_imu_.angular_velocity.x();
    odom.twist.twist.angular.y = latest_imu_.angular_velocity.y();
    odom.twist.twist.angular.z = latest_imu_.angular_velocity.z();

    // 协方差 (6×6 → 36 行主序)
    for (int i = 0; i < 6; ++i)
        for (int j = 0; j < 6; ++j)
        {
            odom.pose.covariance[i * 6 + j] = pose_covariance_(i, j);
            odom.twist.covariance[i * 6 + j] = twist_covariance_(i, j);
        }

    return odom;
}

} // namespace fast_ofio
