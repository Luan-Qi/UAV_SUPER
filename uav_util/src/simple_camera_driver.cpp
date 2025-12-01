#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include "sensor_msgs/CompressedImage.h"
#include <opencv2/opencv.hpp>
#include <iostream>

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
    nh_private.param("video_device_id", video_device_id, 0);
    nh_private.param("image_width", image_width, 640);
    nh_private.param("image_height", image_height, 480);
    nh_private.param("framerate", framerate, 30);
    nh_private.param("camera_frame_id", camera_frame_id, std::string("camera_link"));
    nh_private.param("use_mjpeg", use_mjpeg, false);

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
    ros::Rate loop_rate(framerate);

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