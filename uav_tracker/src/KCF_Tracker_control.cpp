/**
 * vision_control_node.cpp
 * 功能：
 * 1. 接收 KCF Tracker 的目标位置 (像素误差 + 深度)
 * 2. 订阅无人机姿态 (Local Pose) 进行坐标系补偿
 * 3. 实现两种模式：
 * - Mode 0 (FIXED_POS): 悬停不动，只调整偏航角(Yaw)追踪目标
 * - Mode 1 (FOLLOW): XYZ移动 + Yaw旋转，保持距离并居中
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

// 定义控制模式
enum ControlMode {
    MODE_FIXED_POS = 0, // 原地不动，只转Yaw
    MODE_FOLLOW = 1,    // 移动跟随
    MODE_FOLLOW_NO_YAW = 2 // 移动跟随，但不转Yaw
};

class PID {
public:
    float kp, ki, kd, limit;
    float prev_error, integral;

    PID(float p, float i, float d, float lim) 
        : kp(p), ki(i), kd(d), limit(lim), prev_error(0), integral(0) {}

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

    void reset() { prev_error = 0; integral = 0; }
};

class DroneController {
private:
    ros::NodeHandle nh_;
    ros::Subscriber target_sub_;
    ros::Subscriber status_sub_;
    ros::Subscriber state_sub_;
    ros::Subscriber pose_sub_;
    ros::Publisher vel_pub_;

    mavros_msgs::State current_state_;
    geometry_msgs::PoseStamped current_pose_;
    bool is_tracking_;
    geometry_msgs::Point target_pos_; // x,y为像素误差, z为深度(m)
    ros::Time last_time_;

    // PID 控制器
    PID pid_dist;  // 距离控制 (Body X)
    PID pid_h;     // 高度控制 (Body Z)
    PID pid_yaw;   // 偏航角控制 (Angular Z)

    // 参数
    int control_mode_;       // 0: Fixed, 1: Follow
    float desired_distance_; // 期望距离 (m)
    float camera_mount_pitch_deg_; // 摄像头安装角度 (向下为正，例如45度)
    bool debug_mode_;

    // 状态变量
    double current_roll_, current_pitch_, current_yaw_;

public:
    DroneController() : 
        // PID 参数需要根据实际飞行调试 [Kp, Ki, Kd, Limit]
        pid_dist(0.6, 0.005, 0.05, 1.0),    // 距离PID (输出m/s)
        pid_h(0.002, 0.0, 0.0002, 0.5),       // 高度PID (输出m/s)
        pid_yaw(0.003, 0.0, 0.0001, 0.8)  // 偏航PID (输入是像素，输出rad/s)
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

    void stateCb(const mavros_msgs::State::ConstPtr& msg) {
        current_state_ = *msg;
    }

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

    void statusCb(const std_msgs::Bool::ConstPtr& msg) {
        is_tracking_ = msg->data;
        if (!is_tracking_) {
            pid_dist.reset();
            pid_h.reset();
            pid_yaw.reset();
        }
    }

    void targetCb(const geometry_msgs::Point::ConstPtr& msg) {
        target_pos_ = *msg;
        controlLoop();
    }

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
        float pixel_err_x = target_pos_.x; // >0 表示目标在画面右侧
        float pixel_err_y = target_pos_.y; // >0 表示目标在画面下方
        float depth = target_pos_.z;       // 目标距离相机的直线距离

        // 计算相机安装角度 + 无人机当前姿态的总俯仰角
        double total_pitch_rad = (camera_mount_pitch_deg_ * M_PI / 180.0) - current_pitch_;
        
        // 简化模型：
        // 假设像素误差对应角度误差。这里简单用像素值做PID输入，但在输出层做旋转补偿。
        //但更精确的控制方法是：修正“目标在画面中心的垂直误差”造成的虚假高度变化。

        // 偏航控制 (Yaw)
        float cmd_yaw_rate = pid_yaw.compute(-pixel_err_x, dt);

        float cmd_vel_x = 0;
        float cmd_vel_z = 0;
        float cmd_vel_y = 0;

        // 计算直线距离
        float dist_error = depth - desired_distance_;
        
        // 相机前向速度 (用于接近/远离)
        float v_cam_forward = pid_dist.compute(dist_error, dt); 
        // 相机垂直速度 (用于修正画面上下偏差)
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

int main(int argc, char **argv) {
    ros::init(argc, argv, "vision_control_node");
    DroneController controller;
    ros::spin();
    return 0;
}