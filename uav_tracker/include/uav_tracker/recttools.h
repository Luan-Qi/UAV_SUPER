/**
 * @file recttools.h
 * @brief 矩形区域工具函数集（命名空间 RectTools）。
 *
 * @details
 * 提供一组轻量级的 OpenCV 矩形操作模板函数，用于目标跟踪中的
 * ROI 裁剪、边界限制和图像子窗口提取。
 *
 * 核心功能：
 *   1. center()        — 计算矩形中心坐标
 *   2. x2() / y2()    — 获取矩形右/下边界
 *   3. resize()        — 以中心锚点缩放矩形
 *   4. limit()         — 将矩形限制在指定边界内
 *   5. getBorder()     — 计算裁切后的边界偏移
 *   6. subwindow()     — 从图像中提取子窗口（含边界填充）
 *   7. getGrayImage()  — 转换为灰度浮点图像
 *
 * 来源：KCF 开源实现 (Christian Bailer, DFKI)
 */

/*
Author: Christian Bailer
Contact address: Christian.Bailer@dfki.de
Department Augmented Vision DFKI

                          License Agreement
               For Open Source Computer Vision Library
                       (3-clause BSD License)

Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:

  * Redistributions of source code must retain the above copyright notice,
    this list of conditions and the following disclaimer.

  * Redistributions in binary form must reproduce the above copyright notice,
    this list of conditions and the following disclaimer in the documentation
    and/or other materials provided with the distribution.

  * Neither the names of the copyright holders nor the names of the contributors
    may be used to endorse or promote products derived from this software
    without specific prior written permission.

This software is provided by the copyright holders and contributors "as is" and
any express or implied warranties, including, but not limited to, the implied
warranties of merchantability and fitness for a particular purpose are disclaimed.
In no event shall copyright holders or contributors be liable for any direct,
indirect, incidental, special, exemplary, or consequential damages
(including, but not limited to, procurement of substitute goods or services;
loss of use, data, or profits; or business interruption) however caused
and on any theory of liability, whether in contract, strict liability,
or tort (including negligence or otherwise) arising in any way out of
the use of this software, even if advised of the possibility of such damage.
*/

#pragma once

//#include <cv.h>
#include "opencv2/highgui/highgui.hpp"
#include <math.h>

#ifndef _OPENCV_RECTTOOLS_HPP_
#define _OPENCV_RECTTOOLS_HPP_
#endif

// ============================================================================
// RectTools — 矩形工具函数命名空间
// ============================================================================

namespace RectTools
{

/** @brief 计算矩形中心坐标。 @param rect 输入矩形 @return 中心点坐标 (x + w/2, y + h/2) [pixel] */
template <typename t>
inline cv::Vec<t, 2 > center(const cv::Rect_<t> &rect)
{
    return cv::Vec<t, 2 > (rect.x + rect.width / (t) 2, rect.y + rect.height / (t) 2);
}

/** @brief 获取矩形右边界 x 坐标。 @param rect 输入矩形 @return rect.x + rect.width [pixel] */
template <typename t>
inline t x2(const cv::Rect_<t> &rect)
{
    return rect.x + rect.width;
}

/** @brief 获取矩形下边界 y 坐标。 @param rect 输入矩形 @return rect.y + rect.height [pixel] */
template <typename t>
inline t y2(const cv::Rect_<t> &rect)
{
    return rect.y + rect.height;
}

/** @brief 以中心为锚点等比缩放矩形。
 *  @param rect   [in/out] 待缩放的矩形
 *  @param scalex x 方向缩放因子（>1 放大，<1 缩小）
 *  @param scaley y 方向缩放因子（默认与 scalex 相同）
 */
template <typename t>
inline void resize(cv::Rect_<t> &rect, float scalex, float scaley = 0)
{
    if (!scaley)scaley = scalex;
    rect.x -= rect.width * (scalex - 1.f) / 2.f;
    rect.width *= scalex;

    rect.y -= rect.height * (scaley - 1.f) / 2.f;
    rect.height *= scaley;

}

/** @brief 将矩形裁剪至指定边界内。
 *  @param rect  [in/out] 待限制的矩形
 *  @param limit 边界矩形
 */
template <typename t>
inline void limit(cv::Rect_<t> &rect, cv::Rect_<t> limit)
{
    if (rect.x + rect.width > limit.x + limit.width)rect.width = (limit.x + limit.width - rect.x);
    if (rect.y + rect.height > limit.y + limit.height)rect.height = (limit.y + limit.height - rect.y);
    if (rect.x < limit.x)
    {
        rect.width -= (limit.x - rect.x);
        rect.x = limit.x;
    }
    if (rect.y < limit.y)
    {
        rect.height -= (limit.y - rect.y);
        rect.y = limit.y;
    }
    if(rect.width<0)rect.width=0;
    if(rect.height<0)rect.height=0;
}

/** @brief 将矩形裁剪至指定尺寸范围内（重载版本）。
 *  @param rect   [in/out] 待限制的矩形
 *  @param width  最大允许宽度 [pixel]
 *  @param height 最大允许高度 [pixel]
 *  @param x      左边界 [pixel]（默认 0）
 *  @param y      上边界 [pixel]（默认 0）
 */
template <typename t>
inline void limit(cv::Rect_<t> &rect, t width, t height, t x = 0, t y = 0)
{
    limit(rect, cv::Rect_<t > (x, y, width, height));
}

/** @brief 计算裁切后相对于原始矩形的边界偏移。
 *  @param original 原始矩形
 *  @param limited  裁切后的矩形
 *  @return 边界偏移矩形 (x=左偏移, y=上偏移, width=右偏移, height=下偏移) [pixel]
 */
template <typename t>
inline cv::Rect getBorder(const cv::Rect_<t > &original, cv::Rect_<t > & limited)
{
    cv::Rect_<t > res;
    res.x = limited.x - original.x;
    res.y = limited.y - original.y;
    res.width = x2(original) - x2(limited);
    res.height = y2(original) - y2(limited);
    assert(res.x >= 0 && res.y >= 0 && res.width >= 0 && res.height >= 0);
    return res;
}

/** @brief 从图像中提取子窗口（含边界复制填充）。
 *  @param in         输入图像
 *  @param window     目标窗口区域 [pixel]
 *  @param borderType 边界填充模式（默认: cv::BORDER_CONSTANT）
 *  @return 提取的子窗口图像
 *
 *  若窗口超出图像边界，使用 borderType 模式填充超出区域。
 *  若窗口完全在图像内部，直接返回 ROI 引用。
 */
inline cv::Mat subwindow(const cv::Mat &in, const cv::Rect & window, int borderType = cv::BORDER_CONSTANT)
{
    cv::Rect cutWindow = window;
    RectTools::limit(cutWindow, in.cols, in.rows);
    if (cutWindow.height <= 0 || cutWindow.width <= 0)assert(0); //return cv::Mat(window.height,window.width,in.type(),0) ;
    cv::Rect border = RectTools::getBorder(window, cutWindow);
    cv::Mat res = in(cutWindow);

    if (border != cv::Rect(0, 0, 0, 0))
    {
        cv::copyMakeBorder(res, res, border.y, border.height, border.x, border.width, borderType);
    }
    return res;
}

/** @brief 将图像转换为归一化灰度浮点图。
 *  @param img 输入 BGR 彩色图像
 *  @return 归一化灰度浮点图（像素值范围 [0, 1]）
 */
inline cv::Mat getGrayImage(cv::Mat img)
{
    cv::cvtColor(img, img, 6);
//    cv::cvtColor(img, img, CV_BGR2GRAY);
    img.convertTo(img, CV_32F, 1 / 255.f);
    return img;
}

}
