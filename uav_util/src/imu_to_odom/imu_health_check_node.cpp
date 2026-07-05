/**
 * @file imu_health_check_node.cpp
 * @brief 一次性 IMU 原始数据健康检测节点 —— 晃动-静止对比测试。
 *
 * @details
 * 通过"先静止 → 晃动 → 再静止"的四阶段测试，对比晃动前后 IMU 原始数据的统计特征，
 * 自动诊断传感器潜在故障。核心功能：
 *   1. 基准采集      — 静止状态下采集 baseline 样本，计算加速度/角速度均值与标准差
 *   2. 晃动激励      — 提示用户晃动 IMU，记录运动峰值与饱和检测
 *   3. 恢复静止      — 预留 settling 缓冲期后采集 post-static 样本
 *   4. 统计对比      — 比较前后 gyro/accel 零偏变化、噪声增加、跳变和疑似饱和
 *   5. 结论输出      — 分级 (OK / WARN / FAIL) 输出诊断结论并退出节点
 *
 * 算法流水线 (analyzeAndPrint)：
 *   1. 重力幅度检查    — 静止窗口加速度模长是否接近 9.81 [m/s²]
 *   2. 陀螺零偏变化    — 晃动后陀螺零偏漂移是否超标
 *   3. 加速度均值变化  — 晃动后加速度静态均值是否漂移
 *   4. 噪声增加检测    — 后静止窗口噪声是否显著增大
 *   5. 后静止审查      — 后静止窗口本身是否足够静止
 *   6. 跳变检测        — 静止窗口内是否存在大阶跃跳变
 *   7. 饱和检测        — 晃动期间是否达到传感器量程上限
 *   8. 时间戳连续性    — 是否存在丢包或时间跳变
 *
 * 用法：
 *   rosrun uav_util imu_health_check _imu_topic:=/livox/imu
 */

#include <ros/ros.h>
#include <geometry_msgs/Vector3.h>
#include <sensor_msgs/Imu.h>

#include <Eigen/Dense>

#include <algorithm>
#include <cmath>
#include <limits>
#include <string>
#include <vector>

namespace
{

// ============================================================================
// 数据结构
// ============================================================================

/** @brief 单帧 IMU 原始数据样本。 */
struct ImuSample
{
    ros::Time stamp;             ///< 时间戳
    Eigen::Vector3d acc;         ///< 加速度 [m/s²] (body 系)
    Eigen::Vector3d gyro;        ///< 角速度 [rad/s] (body 系)
};

/** @brief 一组 IMU 样本的统计特征。 */
struct WindowStats
{
    int count;                                        ///< 样本数量
    Eigen::Vector3d acc_mean;                         ///< 加速度均值 [m/s²]
    Eigen::Vector3d gyro_mean;                        ///< 角速度均值 [rad/s]
    Eigen::Vector3d acc_std;                          ///< 加速度标准差 [m/s²]
    Eigen::Vector3d gyro_std;                         ///< 角速度标准差 [rad/s]
    double acc_norm_mean;                             ///< 加速度模长均值 [m/s²]
    double gyro_norm_mean;                            ///< 角速度模长均值 [rad/s]
    double acc_norm_std;                              ///< 加速度模长标准差
    double gyro_norm_std;                             ///< 角速度模长标准差
    double max_acc_step;                              ///< 相邻帧加速度最大跳变 [m/s²]
    double max_gyro_step;                             ///< 相邻帧角速度最大跳变 [rad/s]
    double max_dt;                                    ///< 最大帧间隔 [s]
    double mean_dt;                                   ///< 平均帧间隔 [s]

    WindowStats()
        : count(0),
          acc_mean(Eigen::Vector3d::Zero()),
          gyro_mean(Eigen::Vector3d::Zero()),
          acc_std(Eigen::Vector3d::Zero()),
          gyro_std(Eigen::Vector3d::Zero()),
          acc_norm_mean(0.0),
          gyro_norm_mean(0.0),
          acc_norm_std(0.0),
          gyro_norm_std(0.0),
          max_acc_step(0.0),
          max_gyro_step(0.0),
          max_dt(0.0),
          mean_dt(0.0)
    {
    }
};

/** @brief 晃动阶段的运动极值统计。 */
struct MotionStats
{
    double max_acc_axis_abs;    ///< 晃动期间加速度各轴最大绝对值 [m/s²]
    double max_gyro_axis_abs;   ///< 晃动期间角速度各轴最大绝对值 [rad/s]
    double max_acc_norm;        ///< 晃动期间加速度模长最大值 [m/s²]
    double max_gyro_norm;       ///< 晃动期间角速度模长最大值 [rad/s]
    double min_acc_norm;        ///< 晃动期间加速度模长最小值 [m/s²]

    MotionStats()
        : max_acc_axis_abs(0.0),
          max_gyro_axis_abs(0.0),
          max_acc_norm(0.0),
          max_gyro_norm(0.0),
          min_acc_norm(std::numeric_limits<double>::infinity())
    {
    }
};

/** @brief 检测结论分级：通过 / 警告 / 失败。 */
enum CheckLevel
{
    LEVEL_OK = 0,    ///< 检测通过
    LEVEL_WARN = 1,  ///< 存在可疑症状，建议复测或检查
    LEVEL_FAIL = 2   ///< 存在明确故障，需要排查硬件
};

// ============================================================================
// 状态机
// ============================================================================

class ImuHealthCheck
{
public:
    /**
     * @brief 构造函数：加载参数，订阅 IMU 话题，进入校准阶段。
     * @param nh  ROS 私有节点句柄
     */
    explicit ImuHealthCheck(ros::NodeHandle& nh)
        : nh_(nh),
          state_(CALIBRATING),
          restore_notice_printed_(false),
          settling_printed_(false),
          finished_(false)
    {
        nh_.param<std::string>("imu_topic", imu_topic_, "/livox/imu");
        nh_.param<int>("calib_samples", calib_samples_, 200);              ///< 基准采样数 [帧]
        nh_.param<int>("post_static_samples", post_static_samples_, 200);  ///< 后静止采样数 [帧]
        nh_.param<double>("shake_duration", shake_duration_, 8.0);         ///< 晃动阶段时长 [s]
        nh_.param<double>("restore_notice_before", restore_notice_before_, 3.0); ///< 提前提示时间 [s]
        nh_.param<double>("settle_duration", settle_duration_, 2.0);       ///< 机械静置缓冲时间 [s]

        nh_.param<double>("gravity_expected", gravity_expected_, 9.80665);              ///< 期望重力加速度 [m/s²]
        nh_.param<double>("gravity_tolerance", gravity_tolerance_, 0.8);                ///< 重力幅度容差 [m/s²]
        nh_.param<double>("gyro_bias_change_thresh", gyro_bias_change_thresh_, 0.02);   ///< 陀螺零偏变化阈值 [rad/s]
        nh_.param<double>("acc_bias_change_thresh", acc_bias_change_thresh_, 0.20);     ///< 加速度零偏变化阈值 [m/s²]
        nh_.param<double>("acc_norm_change_thresh", acc_norm_change_thresh_, 0.20);     ///< 加速度模长变化阈值 [m/s²]
        nh_.param<double>("noise_ratio_thresh", noise_ratio_thresh_, 3.0);              ///< 噪声增大倍率阈值
        nh_.param<double>("gyro_noise_increase_abs_thresh", gyro_noise_increase_abs_thresh_, 0.01); ///< 陀螺噪声绝对增加阈值 [rad/s]
        nh_.param<double>("acc_noise_increase_abs_thresh", acc_noise_increase_abs_thresh_, 0.10);   ///< 加速度噪声绝对增加阈值 [m/s²]
        nh_.param<double>("gyro_static_std_warn", gyro_static_std_warn_, 0.02);         ///< 静止陀螺标准差警告阈值 [rad/s]
        nh_.param<double>("acc_static_std_warn", acc_static_std_warn_, 0.15);           ///< 静止加速度标准差警告阈值 [m/s²]
        nh_.param<double>("gyro_static_jump_thresh", gyro_static_jump_thresh_, 0.20);   ///< 静止陀螺跳变阈值 [rad/s]
        nh_.param<double>("acc_static_jump_thresh", acc_static_jump_thresh_, 1.00);     ///< 静止加速度跳变阈值 [m/s²]
        nh_.param<double>("max_dt_warn", max_dt_warn_, 0.20);                           ///< 最大帧间隔警告阈值 [s]
        nh_.param<double>("gyro_saturation_abs", gyro_saturation_abs_, 34.9);           ///< 陀螺饱和判定阈值 [rad/s]
        nh_.param<double>("acc_saturation_abs", acc_saturation_abs_, 156.9);            ///< 加速度饱和判定阈值 [m/s²]

        if (calib_samples_ < 10)
            calib_samples_ = 10;
        if (post_static_samples_ < 10)
            post_static_samples_ = 10;
        if (shake_duration_ < 1.0)
            shake_duration_ = 1.0;
        if (restore_notice_before_ < 0.0)
            restore_notice_before_ = 0.0;
        if (restore_notice_before_ > shake_duration_)
            restore_notice_before_ = shake_duration_ * 0.5;
        if (settle_duration_ < 0.0)
            settle_duration_ = 0.0;

        imu_sub_ = nh_.subscribe(imu_topic_, 200, &ImuHealthCheck::imuCallback, this);

        ROS_INFO("IMU health check: subscribe %s", imu_topic_.c_str());
        ROS_INFO("Step 1/4: keep aircraft/IMU STATIONARY, collecting %d samples for baseline.",
                 calib_samples_);
    }

private:
    /** @brief 测试状态机枚举。 */
    enum State
    {
        CALIBRATING,   ///< 初始静止校准
        SHAKING,       ///< 晃动激励阶段
        SETTLING,      ///< 机械静置缓冲
        POST_STATIC,   ///< 后静止采样
        DONE           ///< 测试完成
    };

    ros::NodeHandle nh_;
    ros::Subscriber imu_sub_;

    std::string imu_topic_;

    // ---- 阶段控制参数 ----
    int calib_samples_;               ///< 基准采样帧数
    int post_static_samples_;         ///< 后静止采样帧数
    double shake_duration_;           ///< 晃动阶段时长 [s]
    double restore_notice_before_;    ///< 提前提示停止晃动的时间 [s]
    double settle_duration_;          ///< 机械静置缓冲时间 [s]

    // ---- 检测阈值参数 ----
    double gravity_expected_;                     ///< 期望重力加速度 [m/s²]
    double gravity_tolerance_;                    ///< 重力幅度容差 [m/s²]
    double gyro_bias_change_thresh_;              ///< 陀螺零偏变化阈值 [rad/s]
    double acc_bias_change_thresh_;               ///< 加速度零偏变化阈值 [m/s²]
    double acc_norm_change_thresh_;               ///< 加速度模长变化阈值 [m/s²]
    double noise_ratio_thresh_;                   ///< 噪声倍率阈值
    double gyro_noise_increase_abs_thresh_;       ///< 陀螺噪声绝对增加阈值 [rad/s]
    double acc_noise_increase_abs_thresh_;        ///< 加速度噪声绝对增加阈值 [m/s²]
    double gyro_static_std_warn_;                 ///< 静止陀螺标准差警告 [rad/s]
    double acc_static_std_warn_;                  ///< 静止加速度标准差警告 [m/s²]
    double gyro_static_jump_thresh_;              ///< 静止陀螺跳变阈值 [rad/s]
    double acc_static_jump_thresh_;               ///< 静止加速度跳变阈值 [m/s²]
    double max_dt_warn_;                          ///< 最大帧间隔警告 [s]
    double gyro_saturation_abs_;                  ///< 陀螺饱和阈值 [rad/s]
    double acc_saturation_abs_;                   ///< 加速度饱和阈值 [m/s²]

    // ---- 运行时状态 ----
    State state_;
    ros::Time phase_start_;
    bool restore_notice_printed_;
    bool settling_printed_;
    bool finished_;

    std::vector<ImuSample> baseline_samples_;
    std::vector<ImuSample> post_samples_;
    MotionStats motion_stats_;

    // ========================================================================
    // 回调处理
    // ========================================================================

    /** @brief 从 ROS 消息提取时间戳（优先 header.stamp，回退 ros::Time::now()）。 */
    static ros::Time sampleTime(const sensor_msgs::Imu& msg)
    {
        if (!msg.header.stamp.isZero())
            return msg.header.stamp;
        return ros::Time::now();
    }

    /** @brief 将 geometry_msgs::Vector3 转换为 Eigen::Vector3d。 */
    static Eigen::Vector3d vectorFromMsg(const geometry_msgs::Vector3& v)
    {
        return Eigen::Vector3d(v.x, v.y, v.z);
    }

    /** @brief 检查 Eigen 向量是否所有分量均为有限值。 */
    static bool finiteVector(const Eigen::Vector3d& v)
    {
        return std::isfinite(v.x()) && std::isfinite(v.y()) && std::isfinite(v.z());
    }

    /** @brief IMU 数据回调 —— 根据当前状态分派到对应处理函数。 */
    void imuCallback(const sensor_msgs::Imu& msg)
    {
        if (finished_)
            return;

        ImuSample sample;
        sample.stamp = sampleTime(msg);
        sample.acc = vectorFromMsg(msg.linear_acceleration);
        sample.gyro = vectorFromMsg(msg.angular_velocity);

        if (!finiteVector(sample.acc) || !finiteVector(sample.gyro))
        {
            ROS_WARN_THROTTLE(1.0, "IMU health check: got non-finite IMU value, ignored.");
            return;
        }

        switch (state_)
        {
        case CALIBRATING:
            handleCalibrating(sample);
            break;
        case SHAKING:
            handleShaking(sample);
            break;
        case SETTLING:
            handleSettling(sample);
            break;
        case POST_STATIC:
            handlePostStatic(sample);
            break;
        case DONE:
            break;
        }
    }

    /** @brief CALIBRATING 状态：收集基准样本，满后切到 SHAKING。 */
    void handleCalibrating(const ImuSample& sample)
    {
        baseline_samples_.push_back(sample);
        if ((int)baseline_samples_.size() < calib_samples_)
            return;

        const WindowStats baseline = computeWindowStats(baseline_samples_);
        ROS_INFO("Step 1/4 complete: baseline gyro mean [%.6f %.6f %.6f] rad/s, acc mean [%.4f %.4f %.4f] m/s^2, |acc|=%.4f",
                 baseline.gyro_mean.x(), baseline.gyro_mean.y(), baseline.gyro_mean.z(),
                 baseline.acc_mean.x(), baseline.acc_mean.y(), baseline.acc_mean.z(),
                 baseline.acc_norm_mean);
        ROS_INFO("Step 2/4: SHAKE aircraft/IMU now for %.1f s. Move it enough to expose possible bias jump or saturation.",
                 shake_duration_);

        state_ = SHAKING;
        phase_start_ = sample.stamp;
        updateMotionStats(sample);
    }

    /** @brief SHAKING 状态：持续记录运动极值，到期前提示恢复静止。 */
    void handleShaking(const ImuSample& sample)
    {
        updateMotionStats(sample);

        const double elapsed = (sample.stamp - phase_start_).toSec();
        if (!restore_notice_printed_ && elapsed >= std::max(0.0, shake_duration_ - restore_notice_before_))
        {
            restore_notice_printed_ = true;
            ROS_WARN("Step 3/4: prepare to STOP shaking and put aircraft/IMU back to STATIONARY. Static sampling starts after %.1f s settling.",
                     settle_duration_);
        }

        if (elapsed < shake_duration_)
            return;

        state_ = SETTLING;
        phase_start_ = sample.stamp;
        ROS_WARN("Stop shaking now. Keep aircraft/IMU STATIONARY.");
    }

    /** @brief SETTLING 状态：等待机械静置缓冲期结束。 */
    void handleSettling(const ImuSample& sample)
    {
        const double elapsed = (sample.stamp - phase_start_).toSec();
        if (!settling_printed_)
        {
            settling_printed_ = true;
            ROS_INFO("Waiting %.1f s for mechanical settling before post-static detection.", settle_duration_);
        }

        if (elapsed < settle_duration_)
            return;

        state_ = POST_STATIC;
        post_samples_.clear();
        post_samples_.push_back(sample);
        ROS_INFO("Step 4/4: post-static detection started, collecting %d samples. Keep absolutely stationary.",
                 post_static_samples_);
    }

    /** @brief POST_STATIC 状态：收集后静止样本，满后进入分析。 */
    void handlePostStatic(const ImuSample& sample)
    {
        post_samples_.push_back(sample);
        if ((int)post_samples_.size() < post_static_samples_)
            return;

        state_ = DONE;
        finished_ = true;
        analyzeAndPrint();
        ros::shutdown();
    }

    /** @brief 更新晃动阶段的运动极值统计。 */
    void updateMotionStats(const ImuSample& sample)
    {
        motion_stats_.max_acc_axis_abs = std::max(motion_stats_.max_acc_axis_abs,
                                                  sample.acc.cwiseAbs().maxCoeff());
        motion_stats_.max_gyro_axis_abs = std::max(motion_stats_.max_gyro_axis_abs,
                                                   sample.gyro.cwiseAbs().maxCoeff());
        motion_stats_.max_acc_norm = std::max(motion_stats_.max_acc_norm, sample.acc.norm());
        motion_stats_.max_gyro_norm = std::max(motion_stats_.max_gyro_norm, sample.gyro.norm());
        motion_stats_.min_acc_norm = std::min(motion_stats_.min_acc_norm, sample.acc.norm());
    }

    // ========================================================================
    // 统计分析
    // ========================================================================

    /**
     * @brief 计算一组 IMU 样本的完整统计特征。
     * @param samples  IMU 样本向量
     * @return        包含均值、标准差、跳变、时间间隔等统计的 WindowStats
     */
    static WindowStats computeWindowStats(const std::vector<ImuSample>& samples)
    {
        WindowStats stats;
        stats.count = (int)samples.size();
        if (samples.empty())
            return stats;

        for (size_t i = 0; i < samples.size(); ++i)
        {
            stats.acc_mean += samples[i].acc;
            stats.gyro_mean += samples[i].gyro;
            stats.acc_norm_mean += samples[i].acc.norm();
            stats.gyro_norm_mean += samples[i].gyro.norm();

            if (i > 0)
            {
                stats.max_acc_step = std::max(stats.max_acc_step,
                                              (samples[i].acc - samples[i - 1].acc).norm());
                stats.max_gyro_step = std::max(stats.max_gyro_step,
                                               (samples[i].gyro - samples[i - 1].gyro).norm());
                const double dt = (samples[i].stamp - samples[i - 1].stamp).toSec();
                if (dt > 0.0)
                {
                    stats.max_dt = std::max(stats.max_dt, dt);
                    stats.mean_dt += dt;
                }
            }
        }

        const double n = (double)samples.size();
        stats.acc_mean /= n;
        stats.gyro_mean /= n;
        stats.acc_norm_mean /= n;
        stats.gyro_norm_mean /= n;
        if (samples.size() > 1)
            stats.mean_dt /= (double)(samples.size() - 1);

        for (size_t i = 0; i < samples.size(); ++i)
        {
            const Eigen::Vector3d acc_d = samples[i].acc - stats.acc_mean;
            const Eigen::Vector3d gyro_d = samples[i].gyro - stats.gyro_mean;
            stats.acc_std += acc_d.cwiseProduct(acc_d);
            stats.gyro_std += gyro_d.cwiseProduct(gyro_d);

            const double acc_norm_d = samples[i].acc.norm() - stats.acc_norm_mean;
            const double gyro_norm_d = samples[i].gyro.norm() - stats.gyro_norm_mean;
            stats.acc_norm_std += acc_norm_d * acc_norm_d;
            stats.gyro_norm_std += gyro_norm_d * gyro_norm_d;
        }

        stats.acc_std = (stats.acc_std / n).cwiseSqrt();
        stats.gyro_std = (stats.gyro_std / n).cwiseSqrt();
        stats.acc_norm_std = std::sqrt(stats.acc_norm_std / n);
        stats.gyro_norm_std = std::sqrt(stats.gyro_norm_std / n);

        return stats;
    }

    /** @brief 计算比值（after / before），分母防零保护。 */
    static double ratio(double after, double before)
    {
        const double floor = 1e-9;
        return after / std::max(before, floor);
    }

    /**
     * @brief 条件判定辅助：根据 bad 标志输出 OK/BAD 消息并更新整体结论等级。
     * @param current   当前最高结论等级
     * @param level     本条检测的严重等级
     * @param ok_text   通过时打印的消息
     * @param bad_text  失败时打印的消息
     * @param bad       条件是否满足（true 表示异常）
     * @return          更新后的最高结论等级
     */
    CheckLevel addCheck(CheckLevel current,
                        CheckLevel level,
                        const std::string& ok_text,
                        const std::string& bad_text,
                        bool bad) const
    {
        if (bad)
        {
            if (level == LEVEL_FAIL)
                ROS_ERROR("%s", bad_text.c_str());
            else
                ROS_WARN("%s", bad_text.c_str());
            return std::max(current, level);
        }

        ROS_INFO("%s", ok_text.c_str());
        return current;
    }

    /**
     * @brief 核心分析函数 —— 执行完整 8 项检测并输出结论。
     *
     * 算法流水线：
     *   1. 重力幅度检查    — fabs(|acc| - 9.81) > tolerance ?
     *   2. 陀螺零偏变化    — |gyro_mean_delta| > thresh ?
     *   3. 加速度均值变化  — |acc_mean_delta| > thresh 或 |acc_norm_delta| > thresh ?
     *   4. 噪声增加检测    — 噪声比 > ratio_thresh 且绝对增加 > abs_thresh ?
     *   5. 后静止审查      — 后静止窗口自身标准差是否超标 ?
     *   6. 跳变检测        — 相邻帧最大阶跃 > jump_thresh ?
     *   7. 饱和检测        — 晃动期间是否达到量程上限 ?
     *   8. 时间戳连续性    — 最大帧间隔 > dt_warn ?
     */
    void analyzeAndPrint() const
    {
        const WindowStats baseline = computeWindowStats(baseline_samples_);
        const WindowStats post = computeWindowStats(post_samples_);

        const Eigen::Vector3d gyro_bias_delta = post.gyro_mean - baseline.gyro_mean;
        const Eigen::Vector3d acc_bias_delta = post.acc_mean - baseline.acc_mean;
        const Eigen::Vector3d gyro_noise_delta = post.gyro_std - baseline.gyro_std;
        const Eigen::Vector3d acc_noise_delta = post.acc_std - baseline.acc_std;
        const double acc_norm_delta = post.acc_norm_mean - baseline.acc_norm_mean;

        const double gyro_bias_delta_max = gyro_bias_delta.cwiseAbs().maxCoeff();
        const double acc_bias_delta_max = acc_bias_delta.cwiseAbs().maxCoeff();
        const double gyro_noise_ratio = ratio(post.gyro_std.maxCoeff(), baseline.gyro_std.maxCoeff());
        const double acc_noise_ratio = ratio(post.acc_std.maxCoeff(), baseline.acc_std.maxCoeff());

        ROS_INFO("============================================================");
        ROS_INFO("IMU health check result");
        ROS_INFO("Baseline samples=%d, post-static samples=%d", baseline.count, post.count);
        ROS_INFO("Baseline gyro mean [%.6f %.6f %.6f] rad/s, std [%.6f %.6f %.6f]",
                 baseline.gyro_mean.x(), baseline.gyro_mean.y(), baseline.gyro_mean.z(),
                 baseline.gyro_std.x(), baseline.gyro_std.y(), baseline.gyro_std.z());
        ROS_INFO("Post gyro mean     [%.6f %.6f %.6f] rad/s, std [%.6f %.6f %.6f]",
                 post.gyro_mean.x(), post.gyro_mean.y(), post.gyro_mean.z(),
                 post.gyro_std.x(), post.gyro_std.y(), post.gyro_std.z());
        ROS_INFO("Gyro mean delta    [%.6f %.6f %.6f] rad/s, max_abs=%.6f",
                 gyro_bias_delta.x(), gyro_bias_delta.y(), gyro_bias_delta.z(),
                 gyro_bias_delta_max);

        ROS_INFO("Baseline acc mean  [%.4f %.4f %.4f] m/s^2, std [%.4f %.4f %.4f], |acc|=%.4f +/- %.4f",
                 baseline.acc_mean.x(), baseline.acc_mean.y(), baseline.acc_mean.z(),
                 baseline.acc_std.x(), baseline.acc_std.y(), baseline.acc_std.z(),
                 baseline.acc_norm_mean, baseline.acc_norm_std);
        ROS_INFO("Post acc mean      [%.4f %.4f %.4f] m/s^2, std [%.4f %.4f %.4f], |acc|=%.4f +/- %.4f",
                 post.acc_mean.x(), post.acc_mean.y(), post.acc_mean.z(),
                 post.acc_std.x(), post.acc_std.y(), post.acc_std.z(),
                 post.acc_norm_mean, post.acc_norm_std);
        ROS_INFO("Acc mean delta     [%.4f %.4f %.4f] m/s^2, max_abs=%.4f, |acc| delta=%.4f",
                 acc_bias_delta.x(), acc_bias_delta.y(), acc_bias_delta.z(),
                 acc_bias_delta_max, acc_norm_delta);
        ROS_INFO("Post static max step: gyro=%.6f rad/s, acc=%.4f m/s^2; dt mean=%.4f s max=%.4f s",
                 post.max_gyro_step, post.max_acc_step, post.mean_dt, post.max_dt);
        ROS_INFO("Shake observed: max |gyro axis|=%.3f rad/s, max |acc axis|=%.3f m/s^2, max |gyro|=%.3f, |acc| range=[%.3f, %.3f]",
                 motion_stats_.max_gyro_axis_abs, motion_stats_.max_acc_axis_abs,
                 motion_stats_.max_gyro_norm, motion_stats_.min_acc_norm,
                 motion_stats_.max_acc_norm);

        CheckLevel level = LEVEL_OK;

        // 1. 重力幅度检查
        const bool baseline_gravity_bad =
            std::fabs(baseline.acc_norm_mean - gravity_expected_) > gravity_tolerance_;
        const bool post_gravity_bad =
            std::fabs(post.acc_norm_mean - gravity_expected_) > gravity_tolerance_;
        level = addCheck(level, LEVEL_WARN,
                         "PASS: accel norm is close to gravity in both static windows.",
                         "WARN: accel norm is not close to configured gravity. Check IMU units, scale, axis mapping, or driver config.",
                         baseline_gravity_bad || post_gravity_bad);

        // 2. 陀螺零偏变化
        level = addCheck(level, LEVEL_FAIL,
                         "PASS: gyro zero bias did not show a large post-shake jump.",
                         "FAIL: gyro zero bias changed noticeably after shaking. Suspect IMU bias instability, mechanical mounting stress, vibration, power, or driver issue.",
                         gyro_bias_delta_max > gyro_bias_change_thresh_);

        // 3. 加速度均值变化
        level = addCheck(level, LEVEL_FAIL,
                         "PASS: accel static mean did not show a large post-shake jump.",
                         "FAIL: accel static mean changed noticeably after shaking. Suspect accelerometer bias instability, mounting stress, axis/scale issue, or driver issue.",
                         acc_bias_delta_max > acc_bias_change_thresh_ ||
                             std::fabs(acc_norm_delta) > acc_norm_change_thresh_);

        // 4. 噪声增加检测
        const bool gyro_noise_bad =
            gyro_noise_ratio > noise_ratio_thresh_ &&
            gyro_noise_delta.cwiseAbs().maxCoeff() > gyro_noise_increase_abs_thresh_;
        const bool acc_noise_bad =
            acc_noise_ratio > noise_ratio_thresh_ &&
            acc_noise_delta.cwiseAbs().maxCoeff() > acc_noise_increase_abs_thresh_;
        level = addCheck(level, LEVEL_WARN,
                         "PASS: post-static noise did not increase significantly.",
                         "WARN: post-static noise increased significantly after shaking. Check vibration isolation, connector/cable, power noise, and IMU mounting.",
                         gyro_noise_bad || acc_noise_bad);

        // 5. 后静止审查
        const bool post_not_static =
            post.gyro_std.maxCoeff() > gyro_static_std_warn_ ||
            post.acc_std.maxCoeff() > acc_static_std_warn_;
        level = addCheck(level, LEVEL_WARN,
                         "PASS: post-shake window looks stationary enough for this test.",
                         "WARN: post-shake window still looks noisy or moving. Repeat the test with the aircraft fully stationary, or loosen thresholds if this IMU is known noisy.",
                         post_not_static);

        // 6. 跳变检测
        const bool jump_bad =
            post.max_gyro_step > gyro_static_jump_thresh_ ||
            post.max_acc_step > acc_static_jump_thresh_;
        level = addCheck(level, LEVEL_FAIL,
                         "PASS: no large raw-data jump was seen in post-static window.",
                         "FAIL: large raw IMU jump was seen after returning static. Suspect driver timestamp/data glitch, loose wiring, power, or hardware issue.",
                         jump_bad);

        // 7. 饱和检测
        const bool saturation_bad =
            motion_stats_.max_gyro_axis_abs >= gyro_saturation_abs_ ||
            motion_stats_.max_acc_axis_abs >= acc_saturation_abs_;
        level = addCheck(level, LEVEL_WARN,
                         "PASS: no configured saturation threshold was reached during shaking.",
                         "WARN: shaking reached configured saturation threshold. Increase IMU range or reduce motion; saturated samples can corrupt integration.",
                         saturation_bad);

        // 8. 时间戳连续性
        const bool dt_bad = baseline.max_dt > max_dt_warn_ || post.max_dt > max_dt_warn_;
        level = addCheck(level, LEVEL_WARN,
                         "PASS: IMU timestamps look continuous in static windows.",
                         "WARN: large IMU timestamp gap detected. Check sensor rate, rosbag playback, driver load, or network transport.",
                         dt_bad);

        ROS_INFO("------------------------------------------------------------");
        if (level == LEVEL_OK)
        {
            ROS_INFO("CONCLUSION: IMU raw data looks stable in this shake-then-static test. If imu_to_odom still drifts after motion, the main cause is more likely pure IMU integration / gravity compensation / lack of ZUPT or external fusion, not an obvious IMU hardware fault.");
        }
        else if (level == LEVEL_WARN)
        {
            ROS_WARN("CONCLUSION: IMU has warning-level symptoms. Review the WARN lines above, repeat once with the aircraft firmly stationary after shaking, then tune thresholds or inspect mounting/power/driver configuration.");
        }
        else
        {
            ROS_ERROR("CONCLUSION: IMU shows fail-level post-shake instability or jumps. This supports checking IMU hardware, mounting, wiring, power, range, timestamping, and driver configuration before trusting integration odometry.");
        }
        ROS_INFO("IMU health check finished, node will exit.");
        ROS_INFO("============================================================");
    }
};

}  // namespace

// ============================================================================
// 主函数
// ============================================================================

/**
 * @brief 程序入口：创建 ImuHealthCheck 实例，进入 ROS spin。
 *
 * 用法：
 *   rosrun uav_util imu_health_check _imu_topic:=/livox/imu _shake_duration:=10.0
 */
int main(int argc, char** argv)
{
    ros::init(argc, argv, "imu_health_check");
    ros::NodeHandle nh("~");

    ImuHealthCheck checker(nh);
    ros::spin();

    return 0;
}
