/**
 * @file KCF_Tracker.h
 * @brief KCF 跟踪器 ROS 封装节点 — ImageConverter 类声明。
 *
 * @details
 * 将 KCF 跟踪器封装为 ROS 节点，订阅 RGB 相机图像与深度图像话题，
 * 执行目标跟踪并发布目标三维位置。支持动态参数配置（HOG / 固定窗口 / 多尺度 / Lab 颜色特征）。
 *
 * 核心功能：
 *   1. 图像订阅       — 订阅 RGB 图像话题，驱动跟踪更新
 *   2. 深度订阅       — 订阅深度图像话题，获取目标距离信息
 *   3. 位置发布       — 发布目标在相机坐标系下的三维坐标
 *   4. 状态发布       — 发布跟踪器工作状态（跟踪中 / 丢失）
 *   5. 距离门限       — 根据最小/最大有效距离过滤不可靠测量
 *
 * 依赖：
 *   - ROS (roscpp, image_transport, cv_bridge)
 *   - OpenCV
 *   - kcftracker (KCF 跟踪算法核心)
 *
 * 坐标系约定：
 *   - 像素坐标：图像左上角为原点，x 向右，y 向下
 *   - 深度值：来自深度相机的原始距离读数 [mm]
 */

//
// Created by yahboom on 2021/7/30.
//

#ifndef TRANSBOT_ASTRA_KCF_TRACKER_H
#define TRANSBOT_ASTRA_KCF_TRACKER_H

#include <iostream>
#include <algorithm>
#include <dirent.h>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Point.h"
#include <std_msgs/Bool.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "kcftracker.h"
#include <dynamic_reconfigure/server.h>
#include <time.h>
#include <iomanip>
#include <cmath>

using namespace std;
using namespace cv;

// ============================================================================
// ImageConverter — KCF 跟踪器 ROS 节点
// ============================================================================

class ImageConverter
{
public:
    ros::NodeHandle n;                    ///< ROS 节点句柄
    ros::Subscriber image_sub_;           ///< RGB 图像订阅器
    ros::Subscriber depth_sub_;           ///< 深度图像订阅器
    ros::Publisher target_pub_;           ///< 目标位置发布器 (geometry_msgs::Point)
    ros::Publisher status_pub_;           ///< 跟踪状态发布器 (std_msgs::Bool)

    float LIMIT_MAX_DIST;                 ///< 最大有效深度距离 [mm]
    float LIMIT_MIN_DIST;                 ///< 最小有效深度距离 [mm]
    bool SHOW_DEPTH_WINDOW = false;       ///< 是否显示深度图像调试窗口

    const char *RGB_WINDOW = "rgb_img";       ///< RGB 显示窗口名称
    const char *DEPTH_WINDOW = "depth_img";   ///< 深度显示窗口名称
    bool enable_get_depth = false;            ///< 是否启用深度获取
    float dist_val[5];                        ///< 目标区域五点采样深度值 [mm]
    bool HOG = true;                          ///< 是否使用 HOG 特征（默认开启）
    bool FIXEDWINDOW = false;                 ///< 是否使用固定窗口
    bool MULTISCALE = true;                   ///< 是否使用多尺度跟踪
    bool LAB = false;                         ///< 是否使用 Lab 颜色特征
    KCFTracker tracker;                       ///< KCF 跟踪器核心实例

    /** @brief 构造函数，初始化 ROS 节点、订阅器与发布器。 @param n ROS 节点句柄 */
    ImageConverter(ros::NodeHandle &n);

    /** @brief 析构函数，释放窗口资源。 */
    ~ImageConverter();

    /** @brief 重置跟踪器到待初始化状态。 */
    void Reset();

    /** @brief 取消当前跟踪。 */
    void Cancel();

    /** @brief RGB 图像回调：执行目标跟踪并发布结果。 @param msg 输入 RGB 图像消息 */
    void imageCb(const sensor_msgs::ImageConstPtr &msg);

    /** @brief 深度图像回调：提取目标区域深度值。 @param msg 输入深度图像消息 */
    void depthCb(const sensor_msgs::ImageConstPtr &msg);
};


#endif //TRANSBOT_ASTRA_KCF_TRACKER_H
