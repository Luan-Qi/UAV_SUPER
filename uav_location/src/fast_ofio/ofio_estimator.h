/**
 * @file ofio_estimator.h
 * @brief Fast Optical-Flow Inertial Odometry (fast_ofio) — 核心估计器声明。
 *
 * @details
 * 基于 PX4 EKF2 光流积分算法参考的独立里程计估计器。核心功能：
 *   1. 姿态/旋转补偿   — 从光流中扣除机体旋转分量
 *   2. 杠杆臂补偿       — 传感器非IMU中心安装时的速度修正 (ω×r)
 *   3. 高度/尺度补偿   — 利用测距信息将角流转换为线速度
 *   4. 坐标变换        — body 系速度变换到 world 系并积分位移
 *   5. 时间同步        — 光流与 IMU 时间戳对齐
 *   6. 质量门限        — 基于传感器质量和范围阈值剔除异常测量
 *   7. 漂移/偏置估计  — 基于静止判定的慢变偏置在线估计
 *   8. 协方差传播      — 从传感器噪声模型推算位姿不确定性
 *
 * 坐标系约定：
 *   - Body frame: FRD (X-前 Y-右 Z-下)，光流传感器朝下安装
 *   - World frame: ENU (X-东 Y-北 Z-上)，与 ROS / MAVROS 一致
 *
 * 参考：
 *   - PX4/src/modules/ekf2/EKF/optflow_control.cpp
 *   - PX4/src/lib/optical_flow/OpticalFlow.hpp
 */

#ifndef OFIO_ESTIMATOR_H
#define OFIO_ESTIMATOR_H

#include <Eigen/Dense>
#include <nav_msgs/Odometry.h>

#include "ofio_types.h"

namespace fast_ofio
{

class OfioEstimator
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    explicit OfioEstimator(const OfioConfig& config);
    ~OfioEstimator() = default;

    // === 数据输入 ===
    /** @brief 光流测量输入，执行完整算法流水线。 */
    void updateOpticalFlow(const OpticalFlowMeasurement& flow);
    /** @brief IMU 测量输入（缓存最新角速度、加速度、姿态）。 */
    void updateImu(const ImuMeasurement& imu);

    // === 状态输出 ===
    nav_msgs::Odometry getOdometry() const;
    const EstimatorState& getState() const { return state_; }
    Eigen::Quaterniond getOrientation() const { return state_.orientation; }
    Eigen::Vector3d getPosition() const { return state_.position; }
    Eigen::Vector3d getVelocity() const { return state_.velocity; }
    bool isInitialized() const { return state_.initialized; }

    void reset();
    void setInitialPose(double x, double y, double z, double yaw);

    // === 运行时配置 ===
    void setConfig(const OfioConfig& config) { config_ = config; }
    const OfioConfig& getConfig() const { return config_; }

    /** @brief 光流超时检测（应由外部定时器周期性调用）。 */
    void checkTimeout(const ros::Time& now);

private:
    // === 算法子步骤 ===
    /** @brief 质量门限：仅检查 quality，range 约束移至 classifyRange。 */
    bool checkQualityGate(const OpticalFlowMeasurement& flow, double dt) const;

    /**
     * @brief Range 三分类：NORMAL / OBSTACLE / INVALID。
     * @param effective_range  已扣除初始偏移的有效高度 [m]
     * @param dt               积分周期 [s]
     * @param raw_distance     传感器原始距离读数 [m]
     * @return 分类结果，并更新内部 hysteresis 计数器。
     */
    RangeStatus classifyRange(double effective_range, double dt, double raw_distance);

    /** @brief 旋转补偿：flow_trans = flow_meas - gyro_integrated */
    Eigen::Vector2d compensateRotation(const OpticalFlowMeasurement& flow) const;

    /** @brief 尺度与旋转安装角校正：flow_corrected = R(θ) × flow_trans × scale */
    Eigen::Vector2d applyScale(const Eigen::Vector2d& flow_trans) const;

    /** @brief 偏置补偿：flow_unbiased = flow_scaled - flow_bias */
    Eigen::Vector2d compensateBias(const Eigen::Vector2d& flow_scaled) const;

    /**
     * @brief 高度补偿：角速率光流 → body 系线速度。
     * @param flow_unbiased  偏置补偿后的光流 [rad]
     * @param range          用于尺度计算的 range [m]（flow_range_for_xy，非 effective_range）
     * @param dt             积分周期 [s]
     */
    Eigen::Vector3d flowToVelocity(
        const Eigen::Vector2d& flow_unbiased, double range, double dt) const;

    /**
     * @brief 杠杆臂补偿：修正传感器非IMU中心安装引起的速度误差。
     * v_body = v_sensor - ω × r_offset
     */
    Eigen::Vector3d compensateLeverArm(const Eigen::Vector3d& v_sensor) const;

    /** @brief 坐标变换：body → world (body_frame_odom 模式下不旋转,保持 FLU) */
    Eigen::Vector3d transformToWorld(const Eigen::Vector3d& v_body) const;

    /** @brief 位移积分：p += v_world × dt（XY 由光流，Z 由 updateHeight） */
    void integratePosition(const Eigen::Vector3d& v_world, double dt);

    /**
     * @brief 高度更新：两套语义核心。
     * @param effective_range  已扣除初始偏移的有效高度 [m]
     * @param dt               积分周期 [s]
     * @param status           当前 range 分类
     *
     * NORMAL:   effective_range 修正 global Z，估计 height_velocity。
     * OBSTACLE: 拒绝 range 校正，用 last_valid_height_velocity_ 衰减传播 Z。
     * INVALID:  同 OBSTACLE，协方差增长更快。
     */
    void updateHeight(double effective_range, double dt, RangeStatus status);

    /** @brief OBSTACLE/INVALID 下 Z 预测传播 */
    void predictGlobalZ(double dt);

    /** @brief 偏置在线估计：仅在 NORMAL 时更新 */
    void updateBias(const Eigen::Vector2d& flow_meas, double dt, RangeStatus status);

    /** @brief 一阶IIR速度低通滤波 */
    Eigen::Vector3d applyLowPassFilter(const Eigen::Vector3d& v_raw, double dt);

    /**
     * @brief 协方差传播。
     * @param quality           光流质量
     * @param flow_range_for_xy 实际用于光流速度计算的 range（非 raw distance）
     * @param dt                积分周期 [s]
     * @param status            当前 range 分类
     */
    void updateCovariance(double quality, double flow_range_for_xy, double dt, RangeStatus status);

    // === 成员变量 ===
    OfioConfig     config_;
    EstimatorState state_;

    ImuMeasurement latest_imu_;
    bool           imu_received_;

    // 初始方向（用于 body_frame_odom 模式）
    Eigen::Quaterniond initial_orientation_;

    // Range 初始偏移
    double initial_range_offset_;
    bool   range_offset_captured_;

    // 两套高度语义 — 光流尺度用
    double last_trusted_flow_range_;   // 上一帧可信 surface range，XY fallback

    // 两套高度语义 — global Z 用
    double last_valid_height_velocity_; // NORMAL 时估计的 Z 速度，OBSTACLE 下传播用
    double imu_z_acc_lpf_;             // IMU 垂向加速度低通值
    double z_prediction_time_;         // 无 range 校正的累计时间

    // Range 分类状态机
    RangeStatus range_status_;
    int         obstacle_frame_count_;
    int         recovery_frame_count_;

    Eigen::Vector3d velocity_lpf_state_;
    Eigen::Matrix<double, 6, 6> pose_covariance_;
    Eigen::Matrix<double, 6, 6> twist_covariance_;

    bool flow_timed_out_;
    bool is_stationary_;
};

} // namespace fast_ofio

#endif // OFIO_ESTIMATOR_H
