/**
 * @file kcftracker.h
 * @brief 核化相关滤波（KCF）跟踪器算法类声明。
 *
 * @details
 * KCFTracker 实现了基于核化相关滤波的实时视觉目标跟踪算法。
 * 支持两种特征模式：
 *   - HOG 特征（默认）：梯度方向直方图特征，多通道扩展
 *   - 灰度特征          ：原始像素灰度值，单通道（等价于 CSK [2]）
 *
 * 核心算法流水线：
 *   1. 特征提取        — 从图像子窗口中提取 HOG + Lab 或灰度特征
 *   2. 加窗预处理      — 应用汉宁窗抑制边界效应
 *   3. 高斯核相关      — 计算训练样本与检测样本的核相关矩阵
 *   4. 频域检测        — 利用循环矩阵对角化在频域高效计算响应图
 *   5. 亚像素峰值定位  — 抛物线拟合获取亚像素精度位移
 *   6. 多尺度自适应    — 在多个尺度上检测并选择最优尺度
 *   7. 在线更新        — 采用线性插值平滑更新目标模板和滤波器系数
 *
 * 坐标系约定：
 *   - ROI 中心坐标：相对于图像左上角 [pixel]
 *   - 响应图峰值：相对于模板中心，正值为向右/下
 *
 * 参考：
 *   - [1] J. F. Henriques, R. Caseiro, P. Martins, J. Batista,
 *         "High-Speed Tracking with Kernelized Correlation Filters", TPAMI 2015.
 *   - [2] J. F. Henriques, R. Caseiro, P. Martins, J. Batista,
 *         "Exploiting the Circulant Structure of Tracking-by-detection with Kernels", ECCV 2012.
 *
 * 来源：KCF 开源实现 (Joao Faro, Christian Bailer, Joao F. Henriques)
 */

/*

Tracker based on Kernelized Correlation Filter (KCF) [1] and Circulant Structure with Kernels (CSK) [2].
CSK is implemented by using raw gray level features, since it is a single-channel filter.
KCF is implemented by using HOG features (the default), since it extends CSK to multiple channels.

[1] J. F. Henriques, R. Caseiro, P. Martins, J. Batista,
"High-Speed Tracking with Kernelized Correlation Filters", TPAMI 2015.

[2] J. F. Henriques, R. Caseiro, P. Martins, J. Batista,
"Exploiting the Circulant Structure of Tracking-by-detection with Kernels", ECCV 2012.

Authors: Joao Faro, Christian Bailer, Joao F. Henriques
Contacts: joaopfaro@gmail.com, Christian.Bailer@dfki.de, henriques@isr.uc.pt
Institute of Systems and Robotics - University of Coimbra / Department Augmented Vision DFKI


Constructor parameters, all boolean:
    hog: use HOG features (default), otherwise use raw pixels
    fixed_window: fix window size (default), otherwise use ROI size (slower but more accurate)
    multiscale: use multi-scale tracking (default; cannot be used with fixed_window = true)

Default values are set for all properties of the tracker depending on the above choices.
Their values can be customized further before calling init():
    interp_factor: linear interpolation factor for adaptation
    sigma: gaussian kernel bandwidth
    lambda: regularization
    cell_size: HOG cell size
    padding: horizontal area surrounding the target, relative to its size
    output_sigma_factor: bandwidth of gaussian target
    template_size: template size in pixels, 0 to use ROI size
    scale_step: scale step for multi-scale estimation, 1 to disable it
    scale_weight: to downweight detection scores of other scales for added stability

For speed, the value (template_size/cell_size) should be a power of 2 or a product of small prime numbers.

Inputs to init():
   image is the initial frame.
   roi is a cv::Rect with the target positions in the initial frame

Inputs to update():
   image is the current frame.

Outputs of update():
   cv::Rect with target positions for the current frame


By downloading, copying, installing or using the software you agree to this license.
If you do not agree to this license, do not download, install,
copy or use the software.


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

#include "tracker.h"

#ifndef _OPENCV_KCFTRACKER_HPP_
#define _OPENCV_KCFTRACKER_HPP_
#endif

// ============================================================================
// KCFTracker — 核化相关滤波跟踪器
// ============================================================================

class KCFTracker : public Tracker {
public:
    // ========================================================================
    // 构造与初始化
    // ========================================================================

    /** @brief KCF 跟踪器构造函数。
     *  @param hog           是否使用 HOG 特征（默认: true）
     *  @param fixed_window  是否固定窗口大小（默认: true）
     *  @param multiscale    是否启用多尺度估计（默认: true）
     *  @param lab           是否启用 Lab 颜色特征（默认: true，仅在 HOG 模式下有效）
     */
    KCFTracker(bool hog = true, bool fixed_window = true, bool multiscale = true, bool lab = true);

    /** @brief 初始化跟踪器，提取初始帧特征并训练模型。
     *  @param roi   初始目标包围框 [pixel]
     *  @param image 初始帧图像
     */
    virtual void init(const cv::Rect &roi, cv::Mat image);

    /** @brief 在新一帧中更新目标位置（含多尺度检测与在线训练）。
     *  @param image 当前帧图像
     *  @return 更新后的目标包围框 [pixel]
     */
    virtual cv::Rect update(cv::Mat image);

    // ========================================================================
    // 可调参数（建议在 init() 前修改）
    // ========================================================================

    float interp_factor;         ///< 模板线性插值因子（学习率）
    float sigma;                 ///< 高斯核带宽 [pixel]
    float lambda;                ///< 正则化系数（防止除零）
    int cell_size;               ///< HOG 单元尺寸 [pixel]
    int cell_sizeQ;              ///< cell_size 的平方，避免重复计算
    float padding;               ///< 目标周围扩展区域比例（相对于目标尺寸）
    float output_sigma_factor;   ///< 高斯目标输出的带宽因子
    int template_size;           ///< 模板尺寸 [pixel]（0 表示使用 ROI 尺寸）
    float scale_step;            ///< 多尺度估计的尺度步长（1 表示禁用）
    float scale_weight;          ///< 其他尺度检测得分的衰减权重

protected:
    // ========================================================================
    // 核心算法子步骤
    // ========================================================================

    /** @brief 在当前帧中检测目标位置。
     *  @param z          训练模板特征
     *  @param x          当前帧检测区域特征
     *  @param peak_value [out] 响应图峰值（用于多尺度比较）
     *  @return 目标相对于模板中心的位移 [pixel]
     */
    cv::Point2f detect(cv::Mat z, cv::Mat x, float &peak_value);

    /** @brief 使用单帧图像在线训练/更新跟踪模型。
     *  @param x                   当前帧特征
     *  @param train_interp_factor 训练插值因子（1.0 = 完全替换，<1.0 = 平滑更新）
     */
    void train(cv::Mat x, float train_interp_factor);

    /** @brief 计算两个特征图之间的高斯核相关矩阵。
     *  @param x1 特征图1
     *  @param x2 特征图2
     *  @return 高斯核相关矩阵（频域复数格式）
     *
     * 对于 HOG 特征，逐通道计算频谱乘法后求和；对于灰度特征，直接计算全图频谱乘法。
     */
    cv::Mat gaussianCorrelation(cv::Mat x1, cv::Mat x2);

    /** @brief 创建二维高斯峰值目标（仅首帧调用）。
     *  @param sizey 行数
     *  @param sizex 列数
     *  @return 频域高斯峰值矩阵（已 FFT 变换）
     */
    cv::Mat createGaussianPeak(int sizey, int sizex);

    /** @brief 从图像中提取子窗口并提取特征（HOG + Lab 或灰度）。
     *  @param image        输入图像
     *  @param inithann     是否初始化汉宁窗（首帧为 true）
     *  @param scale_adjust 尺度调整因子（1.0 = 当前尺度，<1.0 = 缩小，>1.0 = 放大）
     *  @return 经汉宁窗加权的特征矩阵
     */
    cv::Mat getFeatures(const cv::Mat & image, bool inithann, float scale_adjust = 1.0f);

    /** @brief 初始化汉宁窗矩阵（仅首帧调用）。
     *
     * 对于 HOG 特征，为每个特征通道复制二维汉宁窗；
     * 对于灰度特征，直接使用二维汉宁窗。
     */
    void createHanningMats();

    /** @brief 一维亚像素峰值估计（抛物线拟合）。
     *  @param left   左邻域值
     *  @param center 中心峰值
     *  @param right  右邻域值
     *  @return 亚像素偏移量（相对于中心） [pixel]
     */
    float subPixelPeak(float left, float center, float right);

    // ========================================================================
    // 内部状态变量
    // ========================================================================

    cv::Mat _alphaf;        ///< 频域滤波器系数 α
    cv::Mat _prob;          ///< 频域高斯目标 y
    cv::Mat _tmpl;          ///< 目标模板（在线更新的外观模型）
    cv::Mat _num;           ///< 滤波器分子（备用公式）
    cv::Mat _den;           ///< 滤波器分母（备用公式）
    cv::Mat _labCentroids;  ///< Lab 颜色空间聚类中心（15 个簇，3 通道）

private:
    int size_patch[3];      ///< 特征图尺寸 [行, 列, 通道数]
    cv::Mat hann;           ///< 汉宁窗矩阵（用于抑制边界效应）
    cv::Size _tmpl_sz;      ///< 模板尺寸 [pixel]
    float _scale;           ///< 当前尺度因子
    int _gaussian_size;     ///< 高斯核相关矩阵尺寸
    bool _hogfeatures;      ///< 是否使用 HOG 特征
    bool _labfeatures;      ///< 是否使用 Lab 颜色特征
};
