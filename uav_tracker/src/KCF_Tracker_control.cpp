/**
 * @file KCF_Tracker_control.cpp
 * @brief 视觉伺服无人机速度控制节点 — 接收 KCF 跟踪器输出的目标位置，计算并发布速度指令。
 *
 * 根据 /tracker/target_position 中的像素误差与深度距离，经 PID 控制器解算
 * 无人机机体坐标系下的线速度与角速度。支持三种控制模式以适应不同任务场景。
 *
 * 控制模式：
 *   MODE_FIXED_POS    (0) — 悬停不动，仅调整偏航角 (Yaw) 追踪目标
 *   MODE_FOLLOW        (1) — XYZ 平移 + Yaw 旋转，保持期望距离并居中
 *   MODE_FOLLOW_NO_YAW (2) — XY 平移 + Z 高度控制，Yaw 不主动旋转
 *
 * 话题：
 *   订阅 — /tracker/target_position       (geometry_msgs::Point)
 *          /tracker/is_tracking            (std_msgs::Bool)
 *          mavros/state                    (mavros_msgs::State)
 *          mavros/local_position/pose      (geometry_msgs::PoseStamped, 仅 MODE_FOLLOW)
 *   发布 — /mavros/setpoint_velocity/cmd_vel_unstamped (geometry_msgs::Twist)
 *
 * 参数 (~):
 *   mode                — 控制模式 (默认 0)
 *   desired_distance    — 期望跟随距离 [m] (默认 1.0)
 *   camera_mount_pitch  — 相机安装俯仰角 [deg] (默认 0)
 *   debug_mode          — 调试模式 (默认 true, 仅打印不发布速度)
 *
 * 使用：
 *   rosrun uav_tracker KCFTracker_control_node
 *   roslaunch uav_tracker KCFTracker.launch
 */

#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Bool.h>
#include <mavros_msgs/State.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <cmath>

// ============================================================================
// 控制模式枚举
// ============================================================================

/// 控制模式枚举 — 决定无人机对跟踪目标的响应方式
enum ControlMode {
    MODE_FIXED_POS = 0,   ///< 原地不动，仅调整偏航角追踪
    MODE_FOLLOW = 1,      ///< XYZ 平移 + Yaw 旋转，保持期望距离并居中
    MODE_FOLLOW_NO_YAW = 2 ///< XY 平移 + Z 高度控制，Yaw 不旋转
};

// ============================================================================
// PID 控制器
// ============================================================================

/**
 * @brief 通用 PID 控制器 — 比例-积分-微分控制，带积分限幅和输出限幅。
 *
 * 控制律: output = Kp * err + Ki * ∫err dt + Kd * d(err)/dt
 * 每次 compute() 调用时自动更新积分项和微分项。
 */
class PID {
public:
    float kp;            ///< 比例增益
    float ki;            ///< 积分增益 [1/s]
    float kd;            ///< 微分增益 [s]
    float limit;         ///< 输出限幅 (同时用于积分限幅)
    float prev_error;    ///< 上一次误差值
    float integral;      ///< 积分累积值

    /**
     * @brief 构造 PID 控制器。
     * @param p   比例增益 Kp
     * @param i   积分增益 Ki [1/s]
     * @param d   微分增益 Kd [s]
     * @param lim 输出/积分限幅
     */
    PID(float p, float i, float d, float lim)
        : kp(p), ki(i), kd(d), limit(lim), prev_error(0), integral(0) {}

    /**
     * @brief 计算 PID 控制量。
     * @param error 当前误差
     * @param dt    距上次调用时间间隔 [s]
     * @return PID 输出值 (已限幅)
     */
    float compute(float error, float dt) {
        if (dt <= 0) return 0;
        integral += error * dt;
        // 积分限幅
        if (integral > limit) integral = limit;
        if (integral < -limit) integral = -limit;

        float derivative = (error - prev_error) / dt;
        float output = kp * error + ki * integral + kd * derivative;
        prev_error = error;

        // 输出限幅
        if (output > limit) output = limit;
        if (output < -limit) output = -limit;
        return output;
    }

    /// 重置积分与微分历史值
    void reset() { prev_error = 0; integral = 0; }
};

// ============================================================================
// 无人机控制器
// ============================================================================

/**
 * @brief 视觉伺服无人机控制器 — 接收跟踪目标位置，通过 PID 计算速度指令。
 *
 * 订阅 KCF 跟踪器输出的像素误差与深度，结合无人机当前姿态进行坐标系补偿，
 * 发布 OFFBOARD 模式下的速度控制指令。
 *
 * 坐标系：
 *   - 图像坐标系: X 右, Y 下 (像素)
 *   - 机体坐标系 (FRD): X 前, Y 右, Z 下
 *   - target_pos_.x — 水平像素误差 (>0 目标在画面右侧)
 *   - target_pos_.y — 垂直像素误差 (>0 目标在画面下方)
 *   - target_pos_.z — 目标距离相机的直线距离 [m]
 */
class DroneController {
private:
    ros::NodeHandle nh_;
    ros::Subscriber target_sub_;   ///< 目标位置订阅
    ros::Subscriber status_sub_;   ///< 跟踪状态订阅
    ros::Subscriber state_sub_;    ///< 飞控状态订阅
    ros::Subscriber pose_sub_;     ///< 无人机位姿订阅
    ros::Publisher vel_pub_;       ///< 速度指令发布

    mavros_msgs::State current_state_;        ///< 当前飞控状态
    geometry_msgs::PoseStamped current_pose_; ///< 当前机体位姿
    bool is_tracking_;                        ///< 是否正在跟踪
    geometry_msgs::Point target_pos_;         ///< 目标位置 (x,y=像素误差 [pixel], z=深度 [m])
    ros::Time last_time_;                     ///< 上一次控制循环时间戳

    // PID 控制器
    PID pid_dist;  ///< 距离控制 (Body X 前向速度) [m/s]
    PID pid_h;     ///< 高度控制 (Body Z 垂直速度) [m/s]
    PID pid_yaw;   ///< 偏航角控制 (Angular Z) [rad/s]

    // 参数
    int control_mode_;               ///< 控制模式 (0=FIXED_POS, 1=FOLLOW, 2=FOLLOW_NO_YAW)
    float desired_distance_;         ///< 期望跟随距离 [m]
    float camera_mount_pitch_deg_;   ///< 摄像头安装俯仰角 [deg] (向下为正)
    bool debug_mode_;                ///< 调试模式 (true=仅打印不发布)

    // 状态变量
    double current_roll_;   ///< 当前滚转角 [rad]
    double current_pitch_;  ///< 当前俯仰角 [rad]
    double current_yaw_;    ///< 当前偏航角 [rad]

public:
    /**
     * @brief 构造无人机控制器 — 初始化 PID、加载参数、建立话题连接。
     *
     * PID 缺省参数 (可在运行时通过 ~ 参数动态覆盖):
     *   - 距离 PID: Kp=0.6,  Ki=0.005,  Kd=0.05,  Limit=1.0  [m/s]
     *   - 高度 PID: Kp=0.002, Ki=0.0,    Kd=0.0002, Limit=0.5  [m/s]
     *   - 偏航 PID: Kp=0.003, Ki=0.0,    Kd=0.0001, Limit=0.8  [rad/s]
     */
    DroneController() :
        // PID 参数需要根据实际飞行调试 [Kp, Ki, Kd, Limit]
        pid_dist(0.6, 0.005, 0.05, 1.0),    // 距离PID (输出 m/s)
        pid_h(0.002, 0.0, 0.0002, 0.5),       // 高度PID (输出 m/s)
        pid_yaw(0.003, 0.0, 0.0001, 0.8)  // 偏航PID (输入是像素 [pixel], 输出 rad/s)
    {
        // 参数加载
        ros::NodeHandle private_nh("~");
        private_nh.param("mode", control_mode_, 0); // 默认为0 (原地模式)
        private_nh.param("desired_distance", desired_distance_, 1.0f);
        private_nh.param("camera_mount_pitch", camera_mount_pitch_deg_, 0.0f);
        private_nh.param("debug_mode", debug_mode_, true);

        state_sub_ = nh_.subscribe<mavros_msgs::State>("mavros/state", 10, &DroneController::stateCb, this);
        status_sub_ = nh_.subscribe<std_msgs::Bool>("/tracker/is_tracking", 1, &DroneController::statusCb, this);
        target_sub_ = nh_.subscribe<geometry_msgs::Point>("/tracker/target_position", 1, &DroneController::targetCb, this);
        if(control_mode_ == MODE_FOLLOW)
            pose_sub_ = nh_.subscribe<geometry_msgs::PoseStamped>("mavros/local_position/pose", 10, &DroneController::poseCb, this);

        vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/mavros/setpoint_velocity/cmd_vel_unstamped", 1);

        is_tracking_ = false;
        last_time_ = ros::Time::now();
        current_roll_ = current_pitch_ = current_yaw_ = 0;
    }

    /// 飞控状态回调 — 更新当前连接状态与飞行模式
    void stateCb(const mavros_msgs::State::ConstPtr& msg) {
        current_state_ = *msg;
    }

    /// 位姿回调 — 解析四元数获取 RPY 欧拉角
    void poseCb(const geometry_msgs::PoseStamped::ConstPtr& msg) {
        current_pose_ = *msg;
        tf2::Quaternion q(
            msg->pose.orientation.x,
            msg->pose.orientation.y,
            msg->pose.orientation.z,
            msg->pose.orientation.w);
        tf2::Matrix3x3 m(q);
        m.getRPY(current_roll_, current_pitch_, current_yaw_);
    }

    /// 跟踪状态回调 — 更新 is_tracking_, 丢失时重置 PID
    void statusCb(const std_msgs::Bool::ConstPtr& msg) {
        is_tracking_ = msg->data;
        if (!is_tracking_) {
            pid_dist.reset();
            pid_h.reset();
            pid_yaw.reset();
        }
    }

    /// 目标位置回调 — 更新目标位置并触发控制循环
    void targetCb(const geometry_msgs::Point::ConstPtr& msg) {
        target_pos_ = *msg;
        controlLoop();
    }

    /**
     * @brief 控制循环 — 根据目标位置计算并发布速度指令。
     *
     * 算法流程:
     *   1. 计算 dt — 距上次调用时间 [s], 过滤异常值 (dt <= 0 或 dt > 1.0)
     *   2. 检查 OFFBOARD 模式与跟踪状态 — 不满足则跳过
     *   3. 解析目标位置 — pixel_err_x, pixel_err_y, depth
     *   4. 计算总俯仰角 — 相机安装角 + 无人机俯仰角补偿
     *   5. 偏航 PID — 输入 pixel_err_x (取负使目标居中), 输出 cmd_yaw_rate [rad/s]
     *   6. 距离 PID — 输入 depth - desired_distance_, 输出 v_cam_forward [m/s]
     *   7. 高度 PID — 输入 pixel_err_y (取负使目标居中), 输出 v_cam_vertical [m/s]
     *   8. 速度分解 — 将相机系速度 (forward, vertical) 分解到机体 FRD 系 (x, z)
     *         v_body_x = v_forward*cos(pitch) - v_vertical*sin(pitch)
     *         v_body_z = v_forward*sin(pitch) + v_vertical*cos(pitch)
     *   9. 模式分支:
     *        MODE_FOLLOW        → cmd_vel.x=v_body_x, cmd_vel.z=v_body_z, cmd_vel.y=0, 使用 cmd_yaw_rate
     *        MODE_FOLLOW_NO_YAW → cmd_vel.x=v_body_x, cmd_vel.z=v_body_z, cmd_vel.y=cmd_yaw_rate, yaw=0
     *   10.发布 geometry_msgs::Twist (debug_mode 下仅打印)
     */
    void controlLoop() {
        ros::Time current_time = ros::Time::now();
        float dt = (current_time - last_time_).toSec();
        last_time_ = current_time;

        if (dt <= 0 || dt > 1.0) // 过滤异常时间差
        {
            ROS_WARN("Invalid dt: %.2f", dt);
            return;
        }

        geometry_msgs::Twist vel_msg;

        if ((current_state_.mode != "OFFBOARD" && !debug_mode_) || !is_tracking_)
            return;

        // KCF输出: x(水平像素误差), y(垂直像素误差), z(深度距离)
        float pixel_err_x = target_pos_.x; // >0 表示目标在画面右侧 [pixel]
        float pixel_err_y = target_pos_.y; // >0 表示目标在画面下方 [pixel]
        float depth = target_pos_.z;       // 目标距离相机的直线距离 [m]

        // 计算相机安装角度 + 无人机当前姿态的总俯仰角
        double total_pitch_rad = (camera_mount_pitch_deg_ * M_PI / 180.0) - current_pitch_;

        // 简化模型：
        // 假设像素误差对应角度误差。这里简单用像素值做PID输入，但在输出层做旋转补偿。
        //但更精确的控制方法是：修正"目标在画面中心的垂直误差"造成的虚假高度变化。

        // 偏航控制 (Yaw)
        float cmd_yaw_rate = pid_yaw.compute(-pixel_err_x, dt);

        float cmd_vel_x = 0;
        float cmd_vel_z = 0;
        float cmd_vel_y = 0;

        // 计算直线距离
        float dist_error = depth - desired_distance_;

        // 相机前向速度 (用于接近/远离) [m/s]
        float v_cam_forward = pid_dist.compute(dist_error, dt);
        // 相机垂直速度 (用于修正画面上下偏差) [m/s]
        float v_cam_vertical = pid_h.compute(-pixel_err_y, dt);

        // 速度分解与姿态补偿 (仅在 FOLLOW 模式下应用)
        float v_body_x = v_cam_forward * cos(total_pitch_rad) - v_cam_vertical * sin(total_pitch_rad);
        float v_body_z = v_cam_forward * sin(total_pitch_rad) + v_cam_vertical * cos(total_pitch_rad);

        if (control_mode_ == MODE_FOLLOW)
        {
            cmd_vel_x = v_body_x;
            cmd_vel_z = v_body_z;
            cmd_vel_y = 0;
        }
        else if (control_mode_ == MODE_FOLLOW_NO_YAW)
        {
            cmd_vel_x = v_body_x;
            cmd_vel_z = v_body_z;
            cmd_vel_y = cmd_yaw_rate;
            cmd_yaw_rate = 0;
        }

        if (debug_mode_)
        {
            ROS_INFO_THROTTLE(1, "V_CAM: (|:%.2f, -:%.2f)", v_cam_vertical, v_cam_forward);
            ROS_INFO_THROTTLE(1, "CMD_VEL: (x:%.2f, y:%.2f, z:%.2f, yaw:%.2f)", cmd_vel_x, cmd_vel_y, cmd_vel_z, cmd_yaw_rate);
        }
        else
        {
            vel_msg.linear.x = cmd_vel_x;
            vel_msg.linear.y = cmd_vel_y;
            vel_msg.linear.z = cmd_vel_z;
            vel_msg.angular.z = cmd_yaw_rate;

            vel_pub_.publish(vel_msg);
        }
    }
};

// ============================================================================
// 主函数
// ============================================================================

/**
 * @brief ROS 节点入口 — 初始化 "vision_control_node"，启动 DroneController。
 *
 * @param argc 命令行参数个数
 * @param argv 命令行参数列表
 * @return int 退出码 (0 = 正常)
 */
int main(int argc, char **argv) {
    ros::init(argc, argv, "vision_control_node");
    DroneController controller;
    ros::spin();
    return 0;
}
