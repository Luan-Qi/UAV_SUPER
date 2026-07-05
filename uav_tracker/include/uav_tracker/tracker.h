/**
 * @file tracker.h
 * @brief KCF/CSK 跟踪器抽象基类声明。
 *
 * @details
 * 为核化相关滤波（KCF）和循环结构核（CSK）跟踪器提供统一的抽象接口。
 * 所有具体的跟踪器实现均需继承此类并实现纯虚函数 init() 和 update()。
 *
 * 核心功能：
 *   1. 初始化跟踪器   — 根据初始帧与目标区域初始化内部状态
 *   2. 更新目标位置   — 在新一帧图像中定位目标并返回包围框
 *
 * 坐标系约定：
 *   - 图像坐标：左上角为原点，x 向右，y 向下
 *   - ROI 使用浮点精度 cv::Rect_<float> 以支持亚像素定位
 *
 * 参考：
 *   - [1] J. F. Henriques et al., "High-Speed Tracking with Kernelized Correlation Filters", TPAMI 2015.
 *   - [2] J. F. Henriques et al., "Exploiting the Circulant Structure of Tracking-by-detection with Kernels", ECCV 2012.
 *
 * 来源：KCF 开源实现 (Joao Faro, Christian Bailer, Joao F. Henriques)
 */

/*
 * File:   BasicTracker.h
 * Author: Joao F. Henriques, Joao Faro, Christian Bailer
 * Contact address: henriques@isr.uc.pt, joaopfaro@gmail.com, Christian.Bailer@dfki.de
 * Instute of Systems and Robotics- University of COimbra / Department Augmented Vision DFKI
 *
 * This source code is provided for for research purposes only. For a commercial license or a different use case please contact us.
 * You are not allowed to publish the unmodified sourcecode on your own e.g. on your webpage. Please refer to the official download page instead.
 * If you want to publish a modified/extended version e.g. because you wrote a publication with a modified version of the sourcecode you need our
 * permission (Please contact us for the permission).
 *
 * We reserve the right to change the license of this sourcecode anytime to BSD, GPL or LGPL.
 * By using the sourcecode you agree to possible restrictions and requirements of these three license models so that the license can be changed
 * anytime without you knowledge.
 */



#pragma once

#include <opencv2/opencv.hpp>
#include <string>

// ============================================================================
// Tracker 抽象基类
// ============================================================================

class Tracker
{
public:
    Tracker()  {}
   virtual  ~Tracker() { }

    /** @brief 初始化跟踪器，设定初始目标区域。 @param roi 初始目标包围框 [pixel] @param image 初始帧图像 */
    virtual void init(const cv::Rect &roi, cv::Mat image) = 0;

    /** @brief 在新一帧中更新目标位置。 @param image 当前帧图像 @return 目标在当前帧中的包围框 [pixel] */
    virtual cv::Rect  update( cv::Mat image)=0;


protected:
    cv::Rect_<float> _roi;  ///< 当前目标包围框（浮点精度，支持亚像素） [pixel]
};
