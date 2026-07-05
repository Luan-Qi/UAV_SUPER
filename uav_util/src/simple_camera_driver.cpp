/**
 * @file simple_camera_driver.cpp
 * @brief 简易 V4L2 USB 摄像头 ROS 驱动 —— 从 /dev/video* 采集图像并发布 raw + compressed。
 *
 * @details
 * 基于 OpenCV VideoCapture 的轻量级摄像头 ROS1 驱动节点，核心功能：
 *   1. V4L2 采集       — 通过 cv::CAP_V4L2 打开 /dev/video<device_id>
 *   2. MJPEG 支持      — 可选 MJPEG FOURCC 格式，高分辨率下提升帧率
 *   3. 原始图像        — BGR→RGB 转换后以 "rgb8" 编码发布到 /simple_camera/image_raw
 *   4. 压缩图像        — 以 JPEG 压缩发布到 /simple_camera/image_compressed
 *   5. 参数校验        — OpenCV 自动适配不支持的参数，启动时打印实际分辨率与帧率
 *
 * 参数：
 *   - video_device_id (int)   : /dev/video 设备编号，默认 0
 *   - image_width     (int)   : 图像宽度 [pixel]，默认 640
 *   - image_height    (int)   : 图像高度 [pixel]，默认 480
 *   - framerate       (int)   : 目标帧率 [Hz]，默认 30
 *   - camera_frame_id (string): 相机坐标系名称，默认 "camera_link"
 *   - use_mjpeg       (bool)  : 是否使用 MJPEG 格式，默认 false
 *
 * 用法：
 *   rosrun uav_util simple_camera_driver _video_device_id:=0 _image_width:=1280 _image_height:=720 _framerate:=30
 */

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include "sensor_msgs/CompressedImage.h"
#include <opencv2/opencv.hpp>
#include <iostream>

// ============================================================================
// 主函数
// ============================================================================

/**
 * @brief 程序入口：解析参数 → 打开摄像头 → 循环采集与发布。
 *
 * 算法流水线：
 *   1. 初始化 ROS 节点与参数句柄
 *   2. 获取 Launch 参数 (device_id, resolution, framerate, mjpeg)
 *   3. 打开 V4L2 设备，设置 MJPEG/分辨率/帧率
 *   4. 打印实际生效的参数（OpenCV 自动适配后）
 *   5. 设置 image_transport raw 发布 + compressed 发布
 *   6. 主循环：cap >> frame → BGR→RGB → cv_bridge → publish
 */
int main(int argc, char** argv)
{
    // 1. 初始化 ROS 节点
    ros::init(argc, argv, "simple_camera_driver");
    ros::NodeHandle nh;           // 公共句柄
    ros::NodeHandle nh_private("~"); // 私有句柄，用于获取 <node> 标签内的参数

    // 2. 获取 Launch 文件中设定的参数
    int video_device_id;
    int image_width, image_height;
    int framerate;
    std::string camera_frame_id;
    bool use_mjpeg; // 某些摄像头在高分辨率下需要开启 MJPEG 格式才能达到高帧率

    // param_name, variable, default_value
    nh_private.param("video_device_id", video_device_id, 0);              ///< /dev/video 设备编号
    nh_private.param("image_width", image_width, 640);                    ///< 图像宽度 [pixel]
    nh_private.param("image_height", image_height, 480);                  ///< 图像高度 [pixel]
    nh_private.param("framerate", framerate, 30);                         ///< 目标帧率 [Hz]
    nh_private.param("camera_frame_id", camera_frame_id, std::string("camera_link"));
    nh_private.param("use_mjpeg", use_mjpeg, false);                      ///< 是否使用 MJPEG

    // 3. 打开摄像头
    cv::VideoCapture cap(video_device_id, cv::CAP_V4L2);

    if (!cap.isOpened()) {
        ROS_ERROR("Could not open video device %d", video_device_id);
        return -1;
    }

    // 4. 设置参数
    if (use_mjpeg) {
        cap.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M','J','P','G'));
        ROS_INFO("Setting format to MJPEG");
    }

    cap.set(cv::CAP_PROP_FRAME_WIDTH, image_width);
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, image_height);
    cap.set(cv::CAP_PROP_FPS, framerate);

    // 打印实际生效的参数（有时候硬件不支持设定的值，OpenCV会自动调整）
    ROS_INFO("Camera opened on device %d", video_device_id);
    ROS_INFO("Resolution: %d x %d", (int)cap.get(cv::CAP_PROP_FRAME_WIDTH), (int)cap.get(cv::CAP_PROP_FRAME_HEIGHT));
    ROS_INFO("FPS: %d", (int)cap.get(cv::CAP_PROP_FPS));

    // 5. 设置 ROS 图像发布者
    image_transport::ImageTransport it(nh);
    image_transport::Publisher pub = it.advertise("/simple_camera/image_raw", 1);
    ros::Publisher pub_compressed = nh.advertise<sensor_msgs::CompressedImage>("simple_camera/image_compressed", 1);

    cv::Mat frame;
    cv::Mat frame_rgb;
    ros::Rate loop_rate(framerate);     ///< 发布循环频率 [Hz]

    // 6. 主循环
    while (nh.ok()) {
        cap >> frame; // 捕获一帧

        if (!frame.empty()) {
            // 将 OpenCV 图像转换为 ROS 消息
            std_msgs::Header header;
            header.stamp = ros::Time::now();
            header.frame_id = "map";

            // cv_bridge::CvImage(header, encoding, image)
            // 通常摄像头读取出来是 BGR 格式
            cv::cvtColor(frame, frame_rgb, cv::COLOR_BGR2RGB);
            sensor_msgs::ImagePtr msg = cv_bridge::CvImage(header, "rgb8", frame_rgb).toImageMsg();
            sensor_msgs::CompressedImagePtr comps_msg = cv_bridge::CvImage(header, "bgr8", frame).toCompressedImageMsg();

            // 发布 ROS 消息
            pub.publish(msg);
            pub_compressed.publish(*comps_msg);
        } else {
            ROS_WARN("Captured empty frame");
        }

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
