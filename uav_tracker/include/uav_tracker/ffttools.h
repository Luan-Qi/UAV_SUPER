/**
 * @file ffttools.h
 * @brief KCF 跟踪器频域（FFT）工具函数集（命名空间 FFTTools）。
 *
 * @details
 * 提供基于 OpenCV DFT 的频域操作函数，用于 KCF/CSK 跟踪器中
 * 循环矩阵对角化、核相关计算和响应图求解等核心步骤。
 *
 * 核心功能：
 *   1. fftd()                     — 前向/逆向离散傅里叶变换
 *   2. real() / imag()            — 提取复数矩阵的实部/虚部
 *   3. magnitude()                — 计算傅里叶频谱幅值
 *   4. complexMultiplication()    — 逐元素复数乘法
 *   5. complexDivision()          — 逐元素复数除法
 *   6. rearrange()                — 频域四象限重排（将低频移至中心）
 *   7. normalizedLogTransform()   — 归一化对数变换（可视化用）
 *
 * 参考：
 *   - J. F. Henriques et al., "High-Speed Tracking with Kernelized Correlation Filters", TPAMI 2015.
 *   - OpenCV DFT 文档
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

#ifndef _OPENCV_FFTTOOLS_HPP_
#define _OPENCV_FFTTOOLS_HPP_
#endif

//NOTE: FFTW support is still shaky, disabled for now.
/*#ifdef USE_FFTW
#include <fftw3.h>
#endif*/

// ============================================================================
// FFTTools — 频域工具函数命名空间
// ============================================================================

namespace FFTTools
{
// 前向声明，避免编译警告
cv::Mat fftd(cv::Mat img, bool backwards = false);
cv::Mat real(cv::Mat img);
cv::Mat imag(cv::Mat img);
cv::Mat magnitude(cv::Mat img);
cv::Mat complexMultiplication(cv::Mat a, cv::Mat b);
cv::Mat complexDivision(cv::Mat a, cv::Mat b);
void rearrange(cv::Mat &img);
void normalizedLogTransform(cv::Mat &img);


/** @brief 前向/逆向离散傅里叶变换。
 *  @param img       输入图像（单通道实数或双通道复数）
 *  @param backwards 是否执行逆向变换（默认: false = 前向 DFT）
 *  @return 频域/时域复数图像（CV_64FC2 或 CV_32FC2）
 *
 *  前向 DFT 输出复数频谱；逆向 DFT 时自动应用 DFT_SCALE 归一化。
 *  输入为单通道实数时自动扩充为双通道复数（虚部置零）。
 */
cv::Mat fftd(cv::Mat img, bool backwards)
{
/*
#ifdef USE_FFTW

    fftw_complex * fm = (fftw_complex*) fftw_malloc(sizeof (fftw_complex) * img.cols * img.rows);

    fftw_plan p = fftw_plan_dft_2d(img.rows, img.cols, fm, fm, backwards ? 1 : -1, 0 * FFTW_ESTIMATE);


    if (img.channels() == 1)
    {
        for (int i = 0; i < img.rows; i++)
            for (int j = 0; j < img.cols; j++)
            {
                fm[i * img.cols + j][0] = img.at<float>(i, j);
                fm[i * img.cols + j][1] = 0;
            }
    }
    else
    {
        assert(img.channels() == 2);
        for (int i = 0; i < img.rows; i++)
            for (int j = 0; j < img.cols; j++)
            {
                fm[i * img.cols + j][0] = img.at<cv::Vec2d > (i, j)[0];
                fm[i * img.cols + j][1] = img.at<cv::Vec2d > (i, j)[1];
            }
    }
    fftw_execute(p);
    cv::Mat res(img.rows, img.cols, CV_64FC2);


    for (int i = 0; i < img.rows; i++)
        for (int j = 0; j < img.cols; j++)
        {
            res.at<cv::Vec2d > (i, j)[0] = fm[i * img.cols + j][0];
            res.at<cv::Vec2d > (i, j)[1] = fm[i * img.cols + j][1];

            //  _iout(fm[i * img.cols + j][0]);
        }

    if (backwards)res *= 1.d / (float) (res.cols * res.rows);

    fftw_free(p);
    fftw_free(fm);
    return res;

#else
*/
    if (img.channels() == 1)
    {
        cv::Mat planes[] = {cv::Mat_<float> (img), cv::Mat_<float>::zeros(img.size())};
        //cv::Mat planes[] = {cv::Mat_<double> (img), cv::Mat_<double>::zeros(img.size())};
        cv::merge(planes, 2, img);
    }
    cv::dft(img, img, backwards ? (cv::DFT_INVERSE | cv::DFT_SCALE) : 0 );

    return img;

/*#endif*/

}

/** @brief 提取复数矩阵的实部。 @param img 双通道复数图像 @return 单通道实部矩阵 */
cv::Mat real(cv::Mat img)
{
    std::vector<cv::Mat> planes;
    cv::split(img, planes);
    return planes[0];
}

/** @brief 提取复数矩阵的虚部。 @param img 双通道复数图像 @return 单通道虚部矩阵 */
cv::Mat imag(cv::Mat img)
{
    std::vector<cv::Mat> planes;
    cv::split(img, planes);
    return planes[1];
}

/** @brief 计算傅里叶频谱幅值。 @param img 单通道或双通道复数图像 @return 频谱幅值矩阵 */
cv::Mat magnitude(cv::Mat img)
{
    cv::Mat res;
    std::vector<cv::Mat> planes;
    cv::split(img, planes); // planes[0] = Re(DFT(I)), planes[1] = Im(DFT(I))
    if (planes.size() == 1) res = cv::abs(img);
    else if (planes.size() == 2) cv::magnitude(planes[0], planes[1], res); // planes[0] = magnitude
    else assert(0);
    return res;
}

/** @brief 逐元素复数乘法: c = a * b。
 *  @param a 复数矩阵 a
 *  @param b 复数矩阵 b
 *  @return 复数乘积矩阵
 */
cv::Mat complexMultiplication(cv::Mat a, cv::Mat b)
{
    std::vector<cv::Mat> pa;
    std::vector<cv::Mat> pb;
    cv::split(a, pa);
    cv::split(b, pb);

    std::vector<cv::Mat> pres;
    pres.push_back(pa[0].mul(pb[0]) - pa[1].mul(pb[1]));
    pres.push_back(pa[0].mul(pb[1]) + pa[1].mul(pb[0]));

    cv::Mat res;
    cv::merge(pres, res);

    return res;
}

/** @brief 逐元素复数除法: c = a / b。
 *  @param a 复数矩阵 a（被除数）
 *  @param b 复数矩阵 b（除数）
 *  @return 复数商矩阵
 */
cv::Mat complexDivision(cv::Mat a, cv::Mat b)
{
    std::vector<cv::Mat> pa;
    std::vector<cv::Mat> pb;
    cv::split(a, pa);
    cv::split(b, pb);

    cv::Mat divisor = 1. / (pb[0].mul(pb[0]) + pb[1].mul(pb[1]));

    std::vector<cv::Mat> pres;

    pres.push_back((pa[0].mul(pb[0]) + pa[1].mul(pb[1])).mul(divisor));
    pres.push_back((pa[1].mul(pb[0]) + pa[0].mul(pb[1])).mul(divisor));

    cv::Mat res;
    cv::merge(pres, res);
    return res;
}

/** @brief 频域四象限重排（将直流分量与低频移至图像中心）。
 *  @param img [in/out] 待重排的复数频谱图像
 *
 *  交换四个象限：左上↔右下，右上↔左下。
 *  用于将 DFT 输出从"四角低频"布局转换为"中心低频"布局。
 */
void rearrange(cv::Mat &img)
{
    // img = img(cv::Rect(0, 0, img.cols & -2, img.rows & -2));
    int cx = img.cols / 2;
    int cy = img.rows / 2;

    cv::Mat q0(img, cv::Rect(0, 0, cx, cy)); // 左上象限
    cv::Mat q1(img, cv::Rect(cx, 0, cx, cy)); // 右上象限
    cv::Mat q2(img, cv::Rect(0, cy, cx, cy)); // 左下象限
    cv::Mat q3(img, cv::Rect(cx, cy, cx, cy)); // 右下象限

    cv::Mat tmp;                      // 交换左上↔右下
    q0.copyTo(tmp);
    q3.copyTo(q0);
    tmp.copyTo(q3);
    q1.copyTo(tmp);                   // 交换右上↔左下
    q2.copyTo(q1);
    tmp.copyTo(q2);
}
/*
template < typename type>
cv::Mat fouriertransFull(const cv::Mat & in)
{
    return fftd(in);

    cv::Mat planes[] = {cv::Mat_<type > (in), cv::Mat_<type>::zeros(in.size())};
    cv::Mat t;
    assert(planes[0].depth() == planes[1].depth());
    assert(planes[0].size == planes[1].size);
    cv::merge(planes, 2, t);
    cv::dft(t, t);

    //cv::normalize(a, a, 0, 1, CV_MINMAX);
    //cv::normalize(t, t, 0, 1, CV_MINMAX);

    // cv::imshow("a",real(a));
    //  cv::imshow("b",real(t));
    // cv::waitKey(0);

    return t;
}*/

/** @brief 归一化对数变换（可视化频谱用）。
 *  @param img [in/out] 输入频谱幅值图像，输出对数归一化结果
 *
 *  计算: log(|img| + 1)，将大动态范围的频谱压缩到可视化范围。
 */
void normalizedLogTransform(cv::Mat &img)
{
    img = cv::abs(img);
    img += cv::Scalar::all(1);
    cv::log(img, img);
    // cv::normalize(img, img, 0, 1, CV_MINMAX);
}

}
