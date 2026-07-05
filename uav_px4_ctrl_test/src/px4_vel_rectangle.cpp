/**
 * @file px4_vel_rectangle.cpp
 * @brief PX4 Offboard 速度控制 — PID 矩形航线飞行测试节点。
 *
 * @details
 * 本节点以速度模式 (setpoint_velocity/cmd_vel_unstamped) 控制无人机完成矩形航线，
 * 采用 PID 控制器根据位置误差计算期望速度指令。核心功能：
 *   1. PID 位置控制器 — 比例 (kp=2.0) + 积分 (ki=0.01) + 微分 (kd=0.5)
 *   2. 状态机航线 — 5 步：上升 → 前进 → 右移 → 后退 → 左移 → 降落
 *   3. 到达判定 — 连续 20 帧在目标范围内视为到达
 *   4. 坐标约定 — PX4 本地位置直接使用，无 ENU→NED 变换
 *   5. 自动降落 — 最后一步切换到 AUTO.LAND 模式
 *
 * 控制律 (local_pos_control)：
 *   err = pos_desired - pos_current
 *   sum_err += err
 *   vel = kp * err + kd * (err - err_prev) + ki * sum_err
 *
 * 注意：此节点使用 PX4 原生 NED 坐标系 (mavros/local_position/pose)，
 *   不经过 uav_mavros_pose_fix 变换，因此航点坐标在 NED 系下定义。
 *
 * 依赖：
 *   - mavros_msgs (State, CommandBool, SetMode)
 *   - geometry_msgs (PoseStamped, Twist)
 *
 * 使用示例：
 *   rosrun uav_px4_ctrl_test px4_vel_rectangle
 */

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <geometry_msgs/Twist.h>

// ============================================================================
// 全局状态变量
// ============================================================================
mavros_msgs::State current_state;           ///< 飞控当前状态
geometry_msgs::PoseStamped local_pos;       ///< 无人机当前位置 (PX4 本地位置估计)

// ============================================================================
// MAVROS 话题回调
// ============================================================================
/**
 * @brief 飞控状态回调 — 缓存当前 PX4 状态。
 */
void state_cb(const mavros_msgs::State::ConstPtr& msg)
{
    current_state = *msg;
}

/**
 * @brief 位置回调 — 缓存无人机 PX4 本地位置估计（反馈环）。
 */
void local_pos_cb(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    local_pos = *msg;
}

// ============================================================================
// PID 控制器变量与参数
// ============================================================================
float err_x = 0;        float err_x0 = 0;       float err_x_err = 0;   ///< X 轴误差/上帧误差/误差微分
float err_y = 0;        float err_y0 = 0;       float err_y_err = 0;   ///< Y 轴误差/上帧误差/误差微分
float err_z = 0;        float err_z0 = 0;       float err_z_err = 0;   ///< Z 轴误差/上帧误差/误差微分
float sum_ex = 0;       ///< X 轴误差积分 [m·s]
float sum_ey = 0;       ///< Y 轴误差积分 [m·s]
float sum_ez = 0;       ///< Z 轴误差积分 [m·s]
float vel_x = 0;        ///< X 轴输出速度 [m/s]
float vel_y = 0;        ///< Y 轴输出速度 [m/s]
float vel_z = 0;        ///< Z 轴输出速度 [m/s]

float kp = 2.0;   ///< 比例增益 [1/s]
float kd = 0.5;   ///< 微分增益 [1]（作用于误差变化率）
float ki = 0.01;  ///< 积分增益 [1/s²]

// PID 调参记录 (供参考):
//   kp = 0.2 → 超调量 ~1m 左右
//   kp = 0.5 + kd = 0.8 → 比 0.8 好一些
//   kp = 1.0 → 比 0.8 好一些

// ============================================================================
// PID 位置控制器
// ============================================================================
/**
 * @brief PID 位置→速度控制器。
 * @param pos_x 期望 X 位置 [m] (NED 系)
 * @param pos_y 期望 Y 位置 [m] (NED 系)
 * @param pos_z 期望 Z 位置 [m] (NED 系, 负为高)
 *
 * 算法流水线：
 *   1. 计算位置误差：err = pos_des - pos_cur
 *   2. 计算误差微分：err_dot = err - err_prev
 *   3. 累积误差积分：sum_err += err
 *   4. PID 输出：vel = kp*err + kd*err_dot + ki*sum_err
 *
 * 输出写入全局变量 vel_x, vel_y, vel_z，由主循环读取并发布。
 */
void local_pos_control(float pos_x, float pos_y, float pos_z)
{
    // 1. 计算位置误差
    err_x = pos_x - local_pos.pose.position.x;
    err_y = pos_y - local_pos.pose.position.y;
    err_z = pos_z - local_pos.pose.position.z;

    // 2. 计算误差微分（变化率）
    err_x_err = err_x - err_x0;
    err_y_err = err_y - err_y0;
    err_z_err = err_z - err_z0;

    // 3. 保存本次误差供下一帧使用
    err_x0 = err_x;
    err_y0 = err_y;
    err_z0 = err_z;

    // 4. 累积积分
    sum_ex += err_x;
    sum_ey += err_y;
    sum_ez += err_z;

    // 5. PID 输出 (注意: Z 轴微分项使用了 err_y_err — 疑似 bug)
    vel_x = kp * err_x + kd * err_x_err + ki * sum_ex;
    vel_y = kp * err_y + kd * err_y_err + ki * sum_ey;
    vel_z = kp * err_z + kd * err_y_err + ki * sum_ez;

    // 调试输出
    ROS_INFO("Pose-x: %f", local_pos.pose.position.x);
    ROS_INFO("Pose-y: %f", local_pos.pose.position.y);
    ROS_INFO("Pose-z: %f", local_pos.pose.position.z);
}

// ============================================================================
// main — 速度控制矩形航线流水线
// ============================================================================
int main(int argc, char **argv)
{
    ros::init(argc, argv, "offb_cfx");
    ros::NodeHandle nh;

    // ---- 通信初始化 ----
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>(
        "mavros/state", 10, state_cb);                                  ///< 飞控状态
    ros::Subscriber local_pos_sub = nh.subscribe<geometry_msgs::PoseStamped>(
        "mavros/local_position/pose", 10, local_pos_cb);                ///< 位置反馈
    ros::Publisher vec_pub = nh.advertise<geometry_msgs::Twist>(
        "mavros/setpoint_velocity/cmd_vel_unstamped", 10);              ///< 速度指令（无时间戳）

    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>(
        "mavros/cmd/arming");                                            ///< 解锁服务
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>(
        "mavros/set_mode");                                              ///< 模式切换服务

    ros::Rate rate(20.0);  // 发布频率必须 > 2 Hz

    // ---- 步骤 1：等待 PX4 连接 ----
    while(ros::ok() && !current_state.connected)
    {
        ros::spinOnce();
        rate.sleep();
    }

    // 初始化速度指令为零
    geometry_msgs::Twist vector;
    vector.linear.x = 0.0;
    vector.linear.y = 0.0;
    vector.linear.z = 0.0;

    // ---- 步骤 2：预发送零速度指令 (offboard 切换前提) ----
    for(int i = 100; ros::ok() && i > 0; --i)
    {
        vec_pub.publish(vector);
        ros::spinOnce();
        rate.sleep();
    }

    // ---- 步骤 3：构建服务请求 ----
    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    ros::Time last_request = ros::Time::now();

    // ---- 状态机变量 ----
    int step = 0;          ///< 当前航线步骤 (0→5 → AUTO.LAND)
    int sametimes = 0;     ///< 连续满足到达条件的帧计数

    // ---- 步骤 4：主控制循环 ----
    while(ros::ok())
    {
        // 4a. 模式切换与解锁 (带 5s 重试间隔)
        if( current_state.mode != "OFFBOARD" &&
            (ros::Time::now() - last_request > ros::Duration(5.0)))
        {
            if( set_mode_client.call(offb_set_mode) &&
                offb_set_mode.response.mode_sent)
            {
                ROS_INFO("Offboard enabled");
            }
            last_request = ros::Time::now();
        }
        else if(!current_state.armed &&
                (ros::Time::now() - last_request > ros::Duration(5.0)))
        {
            if( arming_client.call(arm_cmd) &&
                arm_cmd.response.success)
            {
                ROS_INFO("Vehicle armed");
            }
            last_request = ros::Time::now();
        }
        // 4b. 已解锁且 Offboard 模式 → 执行航线状态机
        else if(current_state.mode == "OFFBOARD" && current_state.armed)
        {
            // 矩形航线规划 (NED 系坐标):
            //   z: 0→10  10→10  10→10  10→10  10→10  10→0 (AUTO.LAND)
            //   x: 0→0    0→40   40→40  40→0    0→0     0→0
            //   y: 0→0    0→0     0→20   20→20  20→0   0→0

            switch (step)
            {
                case 0:
                    // 上升至 2 m 高度
                    local_pos_control(0, 0, 2);
                    vector.linear.x = vel_x;
                    vector.linear.y = vel_y;
                    vector.linear.z = vel_z;
                    if (local_pos.pose.position.z > 1.9 && local_pos.pose.position.z < 2.1)
                    {
                        if (sametimes == 20)
                            step = 1;
                        else
                            sametimes++;
                    }
                    else
                        sametimes = 0;
                    break;

                case 1:
                    // X 方向前进 10 m
                    local_pos_control(10, 0, 2);
                    vector.linear.x = vel_x;
                    vector.linear.y = vel_y;
                    vector.linear.z = vel_z;
                    if (local_pos.pose.position.x > 9.9 && local_pos.pose.position.x < 10.1)
                    {
                        if (sametimes == 20)
                            step = 2;
                        else
                            sametimes++;
                    }
                    else
                        sametimes = 0;
                    break;

                case 2:
                    // Y 方向右移 10 m
                    local_pos_control(10, 10, 2);
                    vector.linear.x = vel_x;
                    vector.linear.y = vel_y;
                    vector.linear.z = vel_z;
                    if (local_pos.pose.position.y > 9.9 && local_pos.pose.position.y < 10.1)
                    {
                        if (sametimes == 20)
                            step = 3;
                        else
                            sametimes++;
                    }
                    else
                        sametimes = 0;
                    break;

                case 3:
                    // X 方向后退 10 m
                    local_pos_control(0, 10, 2);
                    vector.linear.x = vel_x;
                    vector.linear.y = vel_y;
                    vector.linear.z = vel_z;
                    if (local_pos.pose.position.x > -0.1 && local_pos.pose.position.x < 0.1)
                    {
                        if (sametimes == 20)
                            step = 4;
                        else
                            sametimes++;
                    }
                    else
                        sametimes = 0;
                    break;

                case 4:
                    // Y 方向左移 10 m（回到起点上方）
                    local_pos_control(0, 0, 2);
                    vector.linear.x = vel_x;
                    vector.linear.y = vel_y;
                    vector.linear.z = vel_z;
                    if (local_pos.pose.position.y > -0.1 && local_pos.pose.position.y < 0.1)
                    {
                        if (sametimes == 20)
                            step = 5;
                        else
                            sametimes++;
                    }
                    else
                        sametimes = 0;
                    break;

                case 5:
                    // 切换到 AUTO.LAND 自动降落
                    offb_set_mode.request.custom_mode = "AUTO.LAND";
                    if (current_state.mode != "AUTO.LAND" &&
                        (ros::Time::now() - last_request > ros::Duration(5.0)))
                    {
                        if (set_mode_client.call(offb_set_mode) &&
                            offb_set_mode.response.mode_sent)
                        {
                            ROS_INFO("AUTO.LAND enabled");
                            step = 6;
                        }
                        last_request = ros::Time::now();
                    }
                    break;

                default:
                    break;
            }
            vec_pub.publish(vector);  // 发布速度控制指令
        }
        else
        {
            vec_pub.publish(vector);  // 非飞行状态仍发送零速度
        }

        ros::spinOnce();
        rate.sleep();  // 控制发布周期 ≈ 50 ms (20 Hz)
    }

    return 0;
}
