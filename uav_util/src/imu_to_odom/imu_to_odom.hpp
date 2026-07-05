/**
 * @file imu_to_odom.hpp
 * @brief IMU 里程计积分器 —— 纯 IMU 数据积分生成 Odometry，支持 ZUPT、自适应阻尼与重力补偿。
 *
 * @details
 * 基于纯 IMU 数据（加速度 + 角速度）的里程计积分器，核心功能：
 *   1. 在线校准           — 启动时采集静止数据，估计重力矢量与陀螺零偏
 *   2. 重力补偿           — 从加速度中减去重力分量 (acc_l - gravity)
 *   3. 姿态积分           — Rodrigues 公式进行 SO(3) 旋转矩阵递推 (sigma + B 矩阵)
 *   4. 速度积分           — 加速度死区抑制 + 梯形积分
 *   5. 零速检测 (ZUPT)    — 基于加速度与角速度滑动窗口方差的静止判定
 *   6. 自适应速度阻尼     — 静止时强阻尼指数衰减 (tau=0.5s)，运动时 mild 阻尼
 *   7. 位置梯形积分       — p += dt * 0.5 * (v_prev + v_curr)
 *
 * 坐标系约定：
 *   - IMU 输入为 body 系 (前-左-上 或取决于 IMU 安装方向)
 *   - 输出 Odometry 的 orientation 为 body → world 旋转矩阵
 *   - 位置/速度在 world 系表达
 *
 * 参考：
 *   - https://github.com/Abin1258/imu_to_odom
 *
 * 用法：
 *   在 ROS 节点中包含本头文件并实例化 ImuOdom(nh)，通过 IMU 回调驱动。
 */

#ifndef UAV_UTIL_IMU_TO_ODOM_HPP
#define UAV_UTIL_IMU_TO_ODOM_HPP

// ROS includes
#include "ros/ros.h"
#include "ros/time.h"
#include <sensor_msgs/Imu.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <cmath>
#include <deque>
#include <numeric>
#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>

// ============================================================================
// 数据结构
// ============================================================================

/** @brief IMU 里程计状态 —— 位置、姿态、速度、角速度。
 *  @note 方向使用旋转矩阵表示，利于 Rodrigues 递推。 */
struct Point
{
    Eigen::Vector3d pos;      ///< 位置 [m] (world 系)
    Eigen::Matrix3d orien;    ///< 姿态旋转矩阵 (body → world)
    Eigen::Vector3d w;        ///< 角速度 [rad/s]（已扣除陀螺零偏）
    Eigen::Vector3d v;        ///< 线速度 [m/s] (world 系)
};

// ============================================================================
// IMU 里程计处理类
// ============================================================================

/**
 * @brief IMU 里程计积分器 —— 纯 IMU 数据积分生成 Odometry 与 Path。
 *
 * 算法流水线 (ImuCallback)：
 *   1. 校准阶段       — 采集 calib_samples 帧静止数据，估计重力矢量与陀螺零偏
 *   2. 静止检测维护   — 维护加速度/角速度模长滑动窗口
 *   3. 姿态积分       — Rodrigues 公式 SO(3) 递推 (calcOrientation)
 *   4. 速度积分       — 重力去除 → 加速度死区 → 欧拉积分 → 自适应阻尼 (calcPosition)
 *   5. ZUPT 静止阻尼  — 静止判定为真时强制速度指数衰减到零 (tau=0.5s)
 *   6. 运动阻尼       — 非静止时使用 mild 阻尼 (vel_damping)
 *   7. 位置梯形积分   — p += dt * 0.5 * (v_prev + v_curr)
 *   8. Odometry 发布  — 发布 nav_msgs::Odometry 与 nav_msgs::Path
 */
class ImuOdom
{
private:
    ros::NodeHandle nhi;
    ros::Subscriber imusub;
    ros::Publisher odompub;
    ros::Publisher pathpub;
    nav_msgs::Odometry odom;
    nav_msgs::Path path;
    ros::Time time;
    Point point;
    Eigen::Vector3d gravity;            ///< 估计的重力矢量 [m/s²] (IMU frame)
    Eigen::Vector3d gyro_bias;          ///< 估计的陀螺零偏 [rad/s]

    double deltaT;                      ///< 当前帧时间间隔 [s]
    bool firstT;                        ///< 是否为第一帧

    // ---- 校准相关 ----
    int calib_count;                    ///< 校准已采集样本数
    int calib_samples;                  ///< 校准所需样本数
    Eigen::Vector3d acc_sum;            ///< 校准加速度累加器 [m/s²]
    Eigen::Vector3d gyro_sum;           ///< 校准角速度累加器 [rad/s]

    // ---- 静止检测 (ZUPT) ----
    int    zupt_window;                  ///< 滑动窗口大小 [帧数]
    double zupt_acc_thresh;             ///< 加速度方差阈值 [m²/s⁴]
    double zupt_gyro_thresh;            ///< 角速度方差阈值 [rad²/s²]
    std::deque<double> acc_mag_window;  ///< 加速度模长历史滑动窗口
    std::deque<double> gyro_mag_window; ///< 角速度模长历史滑动窗口

    // ---- 平移响应调优 ----
    double acc_deadzone;                ///< 加速度死区阈值 [m/s²]（低于此值的加速度分量被置零）
    double vel_damping;                 ///< 速度阻尼系数（非静止时 mild 阻尼因子）

    // 可配置的话题名
    std::string imu_topic;
    std::string odom_topic;
    std::string path_topic;

    // 可配置的坐标系
    std::string odom_frame_id;
    std::string base_frame_id;

public:
    /**
     * @brief 构造函数：加载 ROS 参数、初始化订阅/发布者、进入校准状态。
     * @param nh  ROS 私有节点句柄
     */
    ImuOdom(ros::NodeHandle& nh);

    ~ImuOdom();

    /**
     * @brief IMU 数据回调 —— 校准阶段累加 → 积分阶段驱动姿态/速度/位置更新。
     *
     * 算法流水线：
     *   1. 校准阶段      — 采集 calib_samples 帧，计算 gravity 与 gyro_bias
     *   2. deltaT 计算   — 前后帧时间差，>0.5s 则重置
     *   3. 静止窗口维护  — 更新 acc/gyro 模长滑动窗口
     *   4. 姿态积分      — calcOrientation (Rodrigues SO(3))
     *   5. 速度/位置积分 — calcPosition (重力去除 + 死区 + 阻尼 + 梯形积分)
     *   6. 发布 Odometry — updateodom
     *
     * @param msg  IMU 传感器消息
     */
    void ImuCallback(const sensor_msgs::Imu &msg);

    /**
     * @brief 静止检测 (ZUPT)：基于加速度与角速度模长方差的滑动窗口判断。
     * @return 窗口未满或方差均低于阈值时返回 true（静止）
     */
    bool isStationary();

    /**
     * @brief 速度与位置积分：重力去除 → 加速度死区 → 自适应阻尼 → 梯形积分。
     *
     * 算法步骤：
     *   1. 重力去除          — acc_motion = R * (acc_l - gravity)
     *   2. 加速度死区        — |acc_motion[i]| < deadzone → 0
     *   3. 速度欧拉积分      — v += dt * acc_motion
     *   4. ZUPT 静止阻尼     — 静止时 v *= exp(-dt/tau), tau=0.5s; |v|<0.001→0
     *   5. 运动阻尼          — 非静止时 v *= (1 - vel_damping * dt)
     *   6. 位置梯形积分      — p += dt * 0.5 * (v_prev + v_curr)
     *
     * @param msg  加速度消息 (body 系) [m/s²]
     */
    void calcPosition(const geometry_msgs::Vector3 &msg);

    /**
     * @brief 姿态积分：使用 Rodrigues 公式进行 SO(3) 旋转矩阵递推。
     *
     * 算法步骤：
     *   1. 陀螺零偏去除      — w = msg - gyro_bias
     *   2. sigma 计算        — sigma = |w| * dt
     *   3. 小角判断          — sigma < 1e-10 → 跳过
     *   4. B 矩阵构造        — 反对称矩阵 [w×] * dt
     *   5. Rodrigues 递推    — R_{k+1} = R_k * (I + sin(sigma)/sigma * B - (1-cos(sigma))/sigma² * B²)
     *
     * @param msg  角速度消息 (body 系) [rad/s]
     */
    void calcOrientation(const geometry_msgs::Vector3 &msg);

    /**
     * @brief 将当前状态写入 Odometry 消息并发布，同时追加到 Path。
     * @param point  当前里程计状态
     */
    void updateodom(const Point point);
};

// ============================================================================
// 构造函数
// ============================================================================

ImuOdom::ImuOdom(ros::NodeHandle& nh) : nhi(nh)
{
    nhi.param<std::string>("imu_topic", imu_topic, "/imu/data");
    nhi.param<std::string>("odom_topic", odom_topic, "imu_odom");
    nhi.param<std::string>("path_topic", path_topic, "imu_path");
    nhi.param<std::string>("odom_frame_id", odom_frame_id, "odom");
    nhi.param<std::string>("base_frame_id", base_frame_id, "base_link");

    // 校准参数
    nhi.param<int>("calib_samples", calib_samples, 100);            ///< 校准样本数

    // 静止检测参数
    nhi.param<int>("zupt_window", zupt_window, 20);                 ///< 滑动窗口帧数
    nhi.param<double>("zupt_acc_thresh", zupt_acc_thresh, 0.01);   ///< 加速度方差阈值 [m²/s⁴]
    nhi.param<double>("zupt_gyro_thresh", zupt_gyro_thresh, 0.005);///< 角速度方差阈值 [rad²/s²]

    // 平移响应参数（静止时自动应用强阻尼，运动时使用 mild 阻尼）
    nhi.param<double>("acc_deadzone", acc_deadzone, 0.05);          ///< 加速度死区 [m/s²]
    nhi.param<double>("vel_damping", vel_damping, 0.5);             ///< 速度阻尼系数

    imusub = nhi.subscribe(imu_topic, 32, &ImuOdom::ImuCallback, this);
    odompub = nhi.advertise<nav_msgs::Odometry>(odom_topic, 32);
    pathpub = nhi.advertise<nav_msgs::Path>(path_topic, 32);

    odom.header.frame_id = odom_frame_id;
    odom.child_frame_id = base_frame_id;
    path.header.frame_id = odom_frame_id;

    Eigen::Vector3d zero(0, 0, 0);
    point.pos = zero;
    point.orien = Eigen::Matrix3d::Identity();
    point.v = zero;
    point.w = zero;

    gravity = zero;
    gyro_bias = zero;
    acc_sum = zero;
    gyro_sum = zero;
    calib_count = 0;

    firstT = true;

    ROS_INFO("IMU Odometry: calibration started, collecting %d samples... "
             "Please keep the IMU stationary.", calib_samples);
}

ImuOdom::~ImuOdom() {}

// ============================================================================
// 静止检测 (ZUPT)
// ============================================================================

// ---- 静止检测：基于加速度和角速度的滑动窗口方差 ----
bool ImuOdom::isStationary()
{
    if ((int)acc_mag_window.size() < zupt_window)
        return true;  // 窗口未满，保守认为是静止

    // 加速度模长方差
    double acc_mean = std::accumulate(acc_mag_window.begin(),
                                       acc_mag_window.end(), 0.0) / zupt_window;
    double acc_var = 0.0;
    for (double v : acc_mag_window)
        acc_var += (v - acc_mean) * (v - acc_mean);
    acc_var /= zupt_window;

    // 角速度模长方差
    double gyro_mean = std::accumulate(gyro_mag_window.begin(),
                                        gyro_mag_window.end(), 0.0) / zupt_window;
    double gyro_var = 0.0;
    for (double v : gyro_mag_window)
        gyro_var += (v - gyro_mean) * (v - gyro_mean);
    gyro_var /= zupt_window;

    return (acc_var < zupt_acc_thresh) && (gyro_var < zupt_gyro_thresh);
}

// ============================================================================
// IMU 回调（主循环）
// ============================================================================

void ImuOdom::ImuCallback(const sensor_msgs::Imu &msg)
{
    // ============================================================
    // 校准阶段
    // ============================================================
    if (calib_count < calib_samples)
    {
        acc_sum[0]  += msg.linear_acceleration.x;
        acc_sum[1]  += msg.linear_acceleration.y;
        acc_sum[2]  += msg.linear_acceleration.z;
        gyro_sum[0] += msg.angular_velocity.x;
        gyro_sum[1] += msg.angular_velocity.y;
        gyro_sum[2] += msg.angular_velocity.z;
        calib_count++;

        if (calib_count == calib_samples)
        {
            gravity   = acc_sum  / calib_samples;
            gyro_bias = gyro_sum / calib_samples;

            ROS_INFO("IMU Odometry: calibration complete (%d samples).",
                     calib_samples);
            ROS_INFO("  Gravity (IMU frame):  [%.4f, %.4f, %.4f], norm=%.4f",
                     gravity[0], gravity[1], gravity[2], gravity.norm());
            ROS_INFO("  Gyro bias (rad/s):    [%.6f, %.6f, %.6f]",
                     gyro_bias[0], gyro_bias[1], gyro_bias[2]);
            ROS_INFO("  ZUPT window=%d, acc_thresh=%.4f, gyro_thresh=%.4f",
                     zupt_window, zupt_acc_thresh, zupt_gyro_thresh);

            time = msg.header.stamp;
        }
        return;
    }

    // ============================================================
    // 正常运行阶段
    // ============================================================
    if (firstT)
    {
        time = msg.header.stamp;
        deltaT = 0;
        firstT = false;
        return;
    }

    deltaT = (msg.header.stamp - time).toSec();
    time = msg.header.stamp;

    if (deltaT > 0.5)
    {
        ROS_WARN("IMU Odometry: large dt=%.3fs, resetting integration.", deltaT);
        deltaT = 0;
        return;
    }

    // ---- 维护静止检测滑动窗口 ----
    double acc_mag = std::sqrt(msg.linear_acceleration.x * msg.linear_acceleration.x +
                               msg.linear_acceleration.y * msg.linear_acceleration.y +
                               msg.linear_acceleration.z * msg.linear_acceleration.z);
    double gyro_mag = std::sqrt(msg.angular_velocity.x * msg.angular_velocity.x +
                                msg.angular_velocity.y * msg.angular_velocity.y +
                                msg.angular_velocity.z * msg.angular_velocity.z);
    acc_mag_window.push_back(acc_mag);
    gyro_mag_window.push_back(gyro_mag);
    while ((int)acc_mag_window.size() > zupt_window)
        acc_mag_window.pop_front();
    while ((int)gyro_mag_window.size() > zupt_window)
        gyro_mag_window.pop_front();

    odom.header.seq   = msg.header.seq;
    odom.header.stamp = msg.header.stamp;

    // 1. 姿态积分
    calcOrientation(msg.angular_velocity);
    // 2. 速度/位置积分
    calcPosition(msg.linear_acceleration);
    // 3. 发布 Odometry
    updateodom(point);
}

// ============================================================================
// 姿态积分 (Rodrigues SO(3))
// ============================================================================

void ImuOdom::calcOrientation(const geometry_msgs::Vector3 &msg)
{
    // 1. 陀螺零偏去除
    point.w << msg.x - gyro_bias[0],
               msg.y - gyro_bias[1],
               msg.z - gyro_bias[2];

    // 2. sigma = |w| * dt
    double sigma = std::sqrt(point.w[0]*point.w[0] +
                             point.w[1]*point.w[1] +
                             point.w[2]*point.w[2]) * deltaT;

    // 3. 小角判断：跳过极小旋转
    if (sigma < 1e-10)
        return;

    // 4. 反对称矩阵 B = [w×] * dt
    Eigen::Matrix3d B;
    B << 0,               -point.w[2]*deltaT,  point.w[1]*deltaT,
         point.w[2]*deltaT,  0,               -point.w[0]*deltaT,
        -point.w[1]*deltaT,  point.w[0]*deltaT,  0;

    // 5. Rodrigues 旋转矩阵递推
    //    R_{k+1} = R_k * (I + sin(sigma)/sigma * B - (1-cos(sigma))/sigma² * B²)
    point.orien = point.orien *
                  (Eigen::Matrix3d::Identity() + (std::sin(sigma) / sigma) * B -
                   ((1.0 - std::cos(sigma)) / (sigma * sigma)) * B * B);
}

// ============================================================================
// 速度与位置积分
// ============================================================================

void ImuOdom::calcPosition(const geometry_msgs::Vector3 &msg)
{
    // 1. 重力去除：acc_motion = R * (acc_l - gravity)
    Eigen::Vector3d acc_l(msg.x, msg.y, msg.z);
    Eigen::Vector3d acc_motion = point.orien * (acc_l - gravity);

    // 2. 加速度死区：抑制微小噪声
    for (int i = 0; i < 3; i++)
    {
        if (std::abs(acc_motion[i]) < acc_deadzone)
            acc_motion[i] = 0.0;
    }

    Eigen::Vector3d v_prev = point.v;

    // 3. 速度欧拉积分
    point.v = point.v + deltaT * acc_motion;

    // 4. 静止检测 + 自适应阻尼
    bool stationary = isStationary();

    if (stationary)
    {
        // IMU 静止：强制速度指数衰减到零，防止陀螺漂移导致位置发散
        // 时间常数 ~0.5s，比运动阻尼快得多
        double tau = 0.5;                                     ///< 静止阻尼时间常数 [s]
        double decay = std::exp(-deltaT / tau);
        point.v = point.v * decay;

        // 速度足够小就直接清零
        if (point.v.norm() < 0.001)
            point.v = Eigen::Vector3d::Zero();
    }
    else if (vel_damping > 0.0)
    {
        // IMU 运动：使用 mild 阻尼，允许速度保持
        double decay = 1.0 - vel_damping * deltaT;
        if (decay < 0.0) decay = 0.0;
        point.v = point.v * decay;
    }

    // 5. 梯形积分：p += dt * 0.5 * (v_prev + v_curr)
    point.pos = point.pos + deltaT * 0.5 * (v_prev + point.v);
}

// ============================================================================
// Odometry 发布
// ============================================================================

void ImuOdom::updateodom(const Point point)
{
    odom.pose.pose.position.x = point.pos(0);
    odom.pose.pose.position.y = point.pos(1);
    odom.pose.pose.position.z = point.pos(2);

    Eigen::Quaterniond q(point.orien);
    odom.pose.pose.orientation.x = q.x();
    odom.pose.pose.orientation.y = q.y();
    odom.pose.pose.orientation.z = q.z();
    odom.pose.pose.orientation.w = q.w();

    odom.twist.twist.linear.x = point.v(0);
    odom.twist.twist.linear.y = point.v(1);
    odom.twist.twist.linear.z = point.v(2);

    odom.twist.twist.angular.x = point.w(0);
    odom.twist.twist.angular.y = point.w(1);
    odom.twist.twist.angular.z = point.w(2);

    odompub.publish(odom);

    // 发布 path
    geometry_msgs::PoseStamped pose_stamped;
    pose_stamped.header.stamp    = odom.header.stamp;
    pose_stamped.header.frame_id = odom.header.frame_id;
    pose_stamped.pose            = odom.pose.pose;
    path.header.stamp = odom.header.stamp;
    path.poses.push_back(pose_stamped);
    pathpub.publish(path);
}

#endif
