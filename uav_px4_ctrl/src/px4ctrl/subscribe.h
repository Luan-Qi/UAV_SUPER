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

class RC_Data_t
{
public:
    int arm;
    int mode;
    int gear;
    int last_arm;
    int last_mode;
    int last_gear;
    bool have_init_last_mode{false};
    bool have_init_last_gear{false};
    double ch[4];

    mavros_msgs::RCIn msg;
    ros::Time rcv_stamp;

    bool is_armed;
    bool is_command_mode;
    bool enter_command_mode;
    bool is_offboard_mode;
    bool enter_hover_mode;
    bool is_hold_mode;

    static constexpr int ARM_THRESHOLD_VALUE = 1750;
    static constexpr int GEAR_SHIFT_UP_THRESHOLD = 1750;
    static constexpr int GEAR_SHIFT_DOWN_THRESHOLD = 1250;
    static constexpr int API_MODE_THRESHOLD_VALUE = 1750;
    static constexpr double DEAD_ZONE = 0.25;

    RC_Data_t();
    void check_validity();
    bool check_centered();
    void feed(mavros_msgs::RCInConstPtr pMsg);
    bool is_received(const ros::Time &now_time);
};

class Odom_Data_t
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Eigen::Vector3d p;
  Eigen::Vector3d v;
  Eigen::Quaterniond q;
  Eigen::Vector3d w;

  nav_msgs::Odometry msg;
  ros::Time rcv_stamp;
  bool recv_new_msg;

  Odom_Data_t();
  void feed(nav_msgs::OdometryConstPtr pMsg);
  bool is_received(const ros::Time &now_time);
};

class State_Data_t
{
public:
    mavros_msgs::State current_state;
    mavros_msgs::State state_before_offboard;

    ros::Time rcv_stamp;

    State_Data_t();
    void feed(mavros_msgs::StateConstPtr pMsg);
    bool is_received(const ros::Time &now_time);
};

class Battery_Data_t
{
public:
    sensor_msgs::BatteryState battery;
    ros::Time rcv_stamp;

    Battery_Data_t();
    void feed(sensor_msgs::BatteryStateConstPtr pMsg);
    bool is_received(const ros::Time &now_time);
};

class Command_Data_t
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    Eigen::Vector3d p;
    Eigen::Vector3d v;
    Eigen::Vector3d a;
    Eigen::Vector3d j;
    double yaw;
    double yaw_rate;

    quadrotor_msgs::PositionCommand msg;
    ros::Time rcv_stamp;

    Command_Data_t();
    void feed(quadrotor_msgs::PositionCommandConstPtr pMsg);
    bool is_received(const ros::Time &now_time);
};


#endif