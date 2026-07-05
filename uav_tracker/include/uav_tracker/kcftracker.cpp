/**
 * @file kcftracker.cpp
 * @brief 核化相关滤波（KCF）跟踪器算法实现。
 *
 * @details
 * KCFTracker 的核心算法实现，包含完整的跟踪流水线：
 *
 * 算法流水线步骤（对应 update() 调用）：
 *   1. 边界约束      — 确保 ROI 保持在图像范围内
 *   2. 特征提取      — 从当前帧提取 HOG + Lab 或灰度特征
 *   3. 核相关计算    — 计算当前特征与训练模板的高斯核相关
 *   4. 频域检测      — 在频域应用滤波器得到响应图
 *   5. 亚像素定位    — 对响应图峰值进行抛物线拟合
 *   6. 多尺度搜索    — 在缩放后的尺度上重复步骤 2-5 并择优
 *   7. 位置更新      — 根据检测位移更新目标包围框
 *   8. 边界约束      — 再次确保更新后的 ROI 有效
 *   9. 特征提取      — 在新位置上提取特征用于训练
 *   10. 在线训练     — 线性插值更新目标模板和滤波器系数
 *
 * 坐标系约定：
 *   - 响应图峰值坐标：相对于模板中心，正值向右/下 [pixel]
 *   - ROI 坐标：相对于图像左上角 [pixel]
 *
 * 参考：
 *   - [1] J. F. Henriques et al., "High-Speed Tracking with Kernelized Correlation Filters", TPAMI 2015.
 *   - [2] J. F. Henriques et al., "Exploiting the Circulant Structure of Tracking-by-detection with Kernels", ECCV 2012.
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
    padding: area surrounding the target, relative to its size
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
#include <iostream>

#ifndef _KCFTRACKER_HEADERS

#include "kcftracker.h"
#include "ffttools.h"
#include "recttools.h"
#include "fhog.h"
#include "labdata.h"

#endif

// ============================================================================
// 构造函数
// ============================================================================

KCFTracker::KCFTracker(bool hog, bool fixed_window, bool multiscale, bool lab) {

    // Parameters equal in all cases
    lambda = 0.0001;
    padding = 2.5;
    //output_sigma_factor = 0.1;
    output_sigma_factor = 0.125;


    if (hog) {    // HOG
        // VOT
        interp_factor = 0.012;
        sigma = 0.6;
        // TPAMI
        //interp_factor = 0.02;
        //sigma = 0.5;
        cell_size = 4;
        _hogfeatures = true;

        if (lab) {
            interp_factor = 0.005;
            sigma = 0.4;
            //output_sigma_factor = 0.025;
            output_sigma_factor = 0.1;

            _labfeatures = true;
            _labCentroids = cv::Mat(nClusters, 3, CV_32FC1, &data);
            cell_sizeQ = cell_size * cell_size;
        } else {
            _labfeatures = false;
        }
    } else {   // RAW
        interp_factor = 0.075;
        sigma = 0.2;
        cell_size = 1;
        _hogfeatures = false;

        if (lab) {
            printf("Lab features are only used with HOG features.\n");
            _labfeatures = false;
        }
    }


    if (multiscale) { // multiscale
        template_size = 96;
        //template_size = 100;
        scale_step = 1.05;
        scale_weight = 0.95;
        if (!fixed_window) {
            //printf("Multiscale does not support non-fixed window.\n");
            fixed_window = true;
        }
    } else if (fixed_window) {  // fit correction without multiscale
        template_size = 96;
        //template_size = 100;
        scale_step = 1;
    } else {
        template_size = 1;
        scale_step = 1;
    }
}

// ============================================================================
// init() — 初始化跟踪器
// ============================================================================

void KCFTracker::init(const cv::Rect &roi, cv::Mat image) {
    // 步骤 1: 保存初始 ROI
    _roi = roi;
    assert(roi.width >= 0 && roi.height >= 0);
    // 步骤 2: 提取初始帧特征（含汉宁窗初始化）
    _tmpl = getFeatures(image, 1);
    // 步骤 3: 创建高斯峰值目标（频域期望输出）
    _prob = createGaussianPeak(size_patch[0], size_patch[1]);
    // 步骤 4: 初始化频域滤波器为零
    _alphaf = cv::Mat(size_patch[0], size_patch[1], CV_32FC2, float(0));
    //_num = cv::Mat(size_patch[0], size_patch[1], CV_32FC2, float(0));
    //_den = cv::Mat(size_patch[0], size_patch[1], CV_32FC2, float(0));
    // 步骤 5: 使用初始帧完全训练模型
    train(_tmpl, 1.0); // train with initial frame
}

// ============================================================================
// update() — 更新目标位置
// ============================================================================

cv::Rect KCFTracker::update(cv::Mat image) {
    // 步骤 1: 边界约束 — 确保 ROI 在图像范围内
    if (_roi.x + _roi.width <= 0) _roi.x = -_roi.width + 1;
    if (_roi.y + _roi.height <= 0) _roi.y = -_roi.height + 1;
    if (_roi.x >= image.cols - 1) _roi.x = image.cols - 2;
    if (_roi.y >= image.rows - 1) _roi.y = image.rows - 2;

    // 步骤 2: 记录当前目标中心
    float cx = _roi.x + _roi.width / 2.0f;
    float cy = _roi.y + _roi.height / 2.0f;

    // 步骤 3: 在当前尺度检测目标
    float peak_value;
    cv::Point2f res = detect(_tmpl, getFeatures(image, 0, 1.0f), peak_value);

    // 步骤 4: 多尺度搜索（若启用）
    if (scale_step != 1) {
        // 步骤 4a: 测试缩小尺度
        float new_peak_value;
        cv::Point2f new_res = detect(_tmpl, getFeatures(image, 0, 1.0f / scale_step), new_peak_value);

        if (scale_weight * new_peak_value > peak_value) {
            res = new_res;
            peak_value = new_peak_value;
            _scale /= scale_step;
            _roi.width /= scale_step;
            _roi.height /= scale_step;
        }

        // 步骤 4b: 测试放大尺度
        new_res = detect(_tmpl, getFeatures(image, 0, scale_step), new_peak_value);

        if (scale_weight * new_peak_value > peak_value) {
            res = new_res;
            peak_value = new_peak_value;
            _scale *= scale_step;
            _roi.width *= scale_step;
            _roi.height *= scale_step;
        }
    }

    // 步骤 5: 根据检测位移更新 ROI 位置
    // Adjust by cell size and _scale
    _roi.x = cx - _roi.width / 2.0f + ((float) res.x * cell_size * _scale);
    _roi.y = cy - _roi.height / 2.0f + ((float) res.y * cell_size * _scale);

    // 步骤 6: 再次边界约束
    if (_roi.x >= image.cols - 1) _roi.x = image.cols - 1;
    if (_roi.y >= image.rows - 1) _roi.y = image.rows - 1;
    if (_roi.x + _roi.width <= 0) _roi.x = -_roi.width + 2;
    if (_roi.y + _roi.height <= 0) _roi.y = -_roi.height + 2;

    assert(_roi.width >= 0 && _roi.height >= 0);
    // 步骤 7: 在新位置提取特征用于在线更新
    cv::Mat x = getFeatures(image, 0);
    // 步骤 8: 在线训练 — 平滑更新模板和滤波器
    train(x, interp_factor);

    return _roi;
}

// ============================================================================
// detect() — 检测目标位置
// ============================================================================

cv::Point2f KCFTracker::detect(cv::Mat z, cv::Mat x, float &peak_value) {
    using namespace FFTTools;

    // 步骤 1: 计算高斯核相关 k^xz = exp(-...)
    cv::Mat k = gaussianCorrelation(x, z);
    // 步骤 2: 频域响应: response = F^{-1}(alpha_f * F(k^xz))
    cv::Mat res = (real(fftd(complexMultiplication(_alphaf, fftd(k)), true)));

    // 步骤 3: 寻找响应图峰值（整数像素精度）
    //minMaxLoc only accepts doubles for the peak, and integer points for the coordinates
    cv::Point2i pi;
    double pv;

    cv::Point2i pi_min;
    double pv_min;
    cv::minMaxLoc(res, &pv_min, &pv, &pi_min, &pi);
    peak_value = (float) pv;
//    std::cout << "min reponse : " << pv_min << " max response :" << pv << std::endl;

    // 步骤 4: 亚像素峰值估计 — 抛物线拟合提升精度
    //subpixel peak estimation, coordinates will be non-integer
    cv::Point2f p((float) pi.x, (float) pi.y);

    if (pi.x > 0 && pi.x < res.cols - 1) {
        p.x += subPixelPeak(res.at<float>(pi.y, pi.x - 1), peak_value, res.at<float>(pi.y, pi.x + 1));
    }

    if (pi.y > 0 && pi.y < res.rows - 1) {
        p.y += subPixelPeak(res.at<float>(pi.y - 1, pi.x), peak_value, res.at<float>(pi.y + 1, pi.x));
    }

    // 步骤 5: 转换为相对于模板中心的位移
    p.x -= (res.cols) / 2;
    p.y -= (res.rows) / 2;

    return p;
}

// ============================================================================
// train() — 在线训练更新模型
// ============================================================================

void KCFTracker::train(cv::Mat x, float train_interp_factor) {
    using namespace FFTTools;

    // 步骤 1: 计算当前帧特征的核自相关 k^xx
    cv::Mat k = gaussianCorrelation(x, x);
    // 步骤 2: 计算频域最优滤波器: alpha_f = y / (k^xx + lambda)
    cv::Mat alphaf = complexDivision(_prob, (fftd(k) + lambda));

    // 步骤 3: 线性插值更新模板和滤波器: model = (1-eta)*model_old + eta*model_new
    _tmpl = (1 - train_interp_factor) * _tmpl + (train_interp_factor) * x;
    _alphaf = (1 - train_interp_factor) * _alphaf + (train_interp_factor) * alphaf;


    /*cv::Mat kf = fftd(gaussianCorrelation(x, x));
    cv::Mat num = complexMultiplication(kf, _prob);
    cv::Mat den = complexMultiplication(kf, kf + lambda);
    _tmpl = (1 - train_interp_factor) * _tmpl + (train_interp_factor) * x;
    _num = (1 - train_interp_factor) * _num + (train_interp_factor) * num;
    _den = (1 - train_interp_factor) * _den + (train_interp_factor) * den;
    _alphaf = complexDivision(_num, _den);*/

}

// ============================================================================
// gaussianCorrelation() — 高斯核相关计算
// ============================================================================

cv::Mat KCFTracker::gaussianCorrelation(cv::Mat x1, cv::Mat x2) {
    using namespace FFTTools;
    cv::Mat c = cv::Mat(cv::Size(size_patch[1], size_patch[0]), CV_32F, cv::Scalar(0));
    // HOG features
    if (_hogfeatures) {
        // 步骤 1: 逐通道计算互相关（频域乘法）
        cv::Mat caux;
        cv::Mat x1aux;
        cv::Mat x2aux;
        for (int i = 0; i < size_patch[2]; i++) {
            x1aux = x1.row(i);   // Procedure do deal with cv::Mat multichannel bug
            x1aux = x1aux.reshape(1, size_patch[0]);
            x2aux = x2.row(i).reshape(1, size_patch[0]);
            cv::mulSpectrums(fftd(x1aux), fftd(x2aux), caux, 0, true);
            caux = fftd(caux, true);
            rearrange(caux);
            caux.convertTo(caux, CV_32F);
            c = c + real(caux);
        }
    }
        // Gray features
    else {
        // 步骤 2: 单通道直接频域乘法
        cv::mulSpectrums(fftd(x1), fftd(x2), c, 0, true);
        c = fftd(c, true);
        rearrange(c);
        c = real(c);
    }
    // 步骤 3: 计算欧氏距离: ||x1||^2 + ||x2||^2 - 2*c
    cv::Mat d;
    cv::max(((cv::sum(x1.mul(x1))[0] + cv::sum(x2.mul(x2))[0]) - 2. * c) /
            (size_patch[0] * size_patch[1] * size_patch[2]), 0, d);

    // 步骤 4: 指数变换: k = exp(-d / sigma^2)
    cv::Mat k;
    cv::exp((-d / (sigma * sigma)), k);
    return k;
}

// ============================================================================
// createGaussianPeak() — 创建高斯峰值目标
// ============================================================================

cv::Mat KCFTracker::createGaussianPeak(int sizey, int sizex) {
    cv::Mat_<float> res(sizey, sizex);

    // 步骤 1: 计算中心坐标
    int syh = (sizey) / 2;
    int sxh = (sizex) / 2;

    // 步骤 2: 计算高斯核带宽（基于模板尺寸与输出 sigma 因子）
    float output_sigma = std::sqrt((float) sizex * sizey) / padding * output_sigma_factor;
    float mult = -0.5 / (output_sigma * output_sigma);

    // 步骤 3: 逐像素生成高斯峰值: g(i,j) = exp(mult * (i^2 + j^2))
    for (int i = 0; i < sizey; i++)
        for (int j = 0; j < sizex; j++) {
            int ih = i - syh;
            int jh = j - sxh;
            res(i, j) = std::exp(mult * (float) (ih * ih + jh * jh));
        }
    // 步骤 4: 变换至频域（期望输出的傅里叶变换）
    return FFTTools::fftd(res);
}

// ============================================================================
// getFeatures() — 提取图像特征
// ============================================================================

cv::Mat KCFTracker::getFeatures(const cv::Mat &image, bool inithann, float scale_adjust) {
    cv::Rect extracted_roi;

    float cx = _roi.x + _roi.width / 2;
    float cy = _roi.y + _roi.height / 2;

    if (inithann) {
        // 步骤 1: 计算 padding 扩展后的尺寸
        int padded_w = _roi.width * padding;
        int padded_h = _roi.height * padding;

        // 步骤 2: 根据模板尺寸或 ROI 确定采样尺度
        if (template_size > 1) {  // Fit largest dimension to the given template size
            if (padded_w >= padded_h)  //fit to width
                _scale = padded_w / (float) template_size;
            else
                _scale = padded_h / (float) template_size;

            _tmpl_sz.width = padded_w / _scale;
            _tmpl_sz.height = padded_h / _scale;
        } else {  //No template size given, use ROI size
            _tmpl_sz.width = padded_w;
            _tmpl_sz.height = padded_h;
            _scale = 1;
            // original code from paper:
            /*if (sqrt(padded_w * padded_h) >= 100) {   //Normal size
                _tmpl_sz.width = padded_w;
                _tmpl_sz.height = padded_h;
                _scale = 1;
            }
            else {   //ROI is too big, track at half size
                _tmpl_sz.width = padded_w / 2;
                _tmpl_sz.height = padded_h / 2;
                _scale = 2;
            }*/
        }

        // 步骤 3: 对齐到 cell_size 倍数（便于 HOG 划分单元）
        if (_hogfeatures) {
            // Round to cell size and also make it even
            _tmpl_sz.width = (((int) (_tmpl_sz.width / (2 * cell_size))) * 2 * cell_size) + cell_size * 2;
            _tmpl_sz.height = (((int) (_tmpl_sz.height / (2 * cell_size))) * 2 * cell_size) + cell_size * 2;
        } else {  //Make number of pixels even (helps with some logic involving half-dimensions)
            _tmpl_sz.width = (_tmpl_sz.width / 2) * 2;
            _tmpl_sz.height = (_tmpl_sz.height / 2) * 2;
        }
    }

    // 步骤 4: 计算提取区域尺寸（含尺度调整）
    extracted_roi.width = scale_adjust * _scale * _tmpl_sz.width;
    extracted_roi.height = scale_adjust * _scale * _tmpl_sz.height;

    // 步骤 5: 以目标中心对齐提取区域
    // center roi with new size
    extracted_roi.x = cx - extracted_roi.width / 2;
    extracted_roi.y = cy - extracted_roi.height / 2;

    // 步骤 6: 从图像中提取子窗口
    cv::Mat FeaturesMap;
    cv::Mat z = RectTools::subwindow(image, extracted_roi, cv::BORDER_REPLICATE);

    // 步骤 7: 重采样至模板尺寸
    if (z.cols != _tmpl_sz.width || z.rows != _tmpl_sz.height) {
        cv::resize(z, z, _tmpl_sz);
    }

    // 步骤 8: 提取 HOG 特征
    if (_hogfeatures) {
        // 步骤 8a: 调用 FHOG 特征提取
        //IplImage z_ipl = z;
        IplImage z_ipl = cvIplImage(z);
        CvLSVMFeatureMapCaskade *map;
        getFeatureMaps(&z_ipl, cell_size, &map);
        // 步骤 8b: 归一化与截断
        normalizeAndTruncate(map, 0.2f);
        // 步骤 8c: PCA 降维
        PCAFeatureMaps(map);
        // 步骤 8d: 记录特征图尺寸
        size_patch[0] = map->sizeY;
        size_patch[1] = map->sizeX;
        size_patch[2] = map->numFeatures;

        FeaturesMap = cv::Mat(cv::Size(map->numFeatures, map->sizeX * map->sizeY), CV_32F,
                              map->map);  // Procedure do deal with cv::Mat multichannel bug
        FeaturesMap = FeaturesMap.t();
        freeFeatureMapObject(&map);

        // 步骤 9: 提取 Lab 颜色特征（可选）
        if (_labfeatures) {
            cv::Mat imgLab;
            cvtColor(z, imgLab, CV_BGR2Lab);
            unsigned char *input = (unsigned char *) (imgLab.data);

            // 步骤 9a: 创建稀疏输出向量（15 维 × 单元数）
            // Sparse output vector
            cv::Mat outputLab = cv::Mat(_labCentroids.rows, size_patch[0] * size_patch[1], CV_32F, float(0));

            int cntCell = 0;
            // 步骤 9b: 遍历每个单元
            // Iterate through each cell
            for (int cY = cell_size; cY < z.rows - cell_size; cY += cell_size) {
                for (int cX = cell_size; cX < z.cols - cell_size; cX += cell_size) {
                    // 步骤 9c: 遍历单元内每个像素
                    // Iterate through each pixel of cell (cX,cY)
                    for (int y = cY; y < cY + cell_size; ++y) {
                        for (int x = cX; x < cX + cell_size; ++x) {
                            // Lab components for each pixel
                            float l = (float) input[(z.cols * y + x) * 3];
                            float a = (float) input[(z.cols * y + x) * 3 + 1];
                            float b = (float) input[(z.cols * y + x) * 3 + 2];

                            // 步骤 9d: 寻找最近的聚类中心（最近邻分类）
                            // Iterate trough each centroid
                            float minDist = FLT_MAX;
                            int minIdx = 0;
                            float *inputCentroid = (float *) (_labCentroids.data);
                            for (int k = 0; k < _labCentroids.rows; ++k) {
                                float dist = ((l - inputCentroid[3 * k]) * (l - inputCentroid[3 * k]))
                                             + ((a - inputCentroid[3 * k + 1]) * (a - inputCentroid[3 * k + 1]))
                                             + ((b - inputCentroid[3 * k + 2]) * (b - inputCentroid[3 * k + 2]));
                                if (dist < minDist) {
                                    minDist = dist;
                                    minIdx = k;
                                }
                            }
                            // 步骤 9e: 在对应聚类中心累加归一化投票
                            // Store result at output
                            outputLab.at<float>(minIdx, cntCell) += 1.0 / cell_sizeQ;
                            //((float*) outputLab.data)[minIdx * (size_patch[0]*size_patch[1]) + cntCell] += 1.0 / cell_sizeQ;
                        }
                    }
                    cntCell++;
                }
            }
            // 步骤 9f: 将 Lab 特征附加到 HOG 特征之后
            // Update size_patch[2] and add features to FeaturesMap
            size_patch[2] += _labCentroids.rows;
            FeaturesMap.push_back(outputLab);
        }
    } else {
        // 步骤 10: 提取灰度特征（单通道，等价于 CSK）
        FeaturesMap = RectTools::getGrayImage(z);
        FeaturesMap -= (float) 0.5; // In Paper;
        size_patch[0] = z.rows;
        size_patch[1] = z.cols;
        size_patch[2] = 1;
    }

    // 步骤 11: 应用汉宁窗（抑制边界效应）
    if (inithann) {
        createHanningMats();
    }
    FeaturesMap = hann.mul(FeaturesMap);
    return FeaturesMap;
}

// ============================================================================
// createHanningMats() — 初始化汉宁窗
// ============================================================================

void KCFTracker::createHanningMats() {
    // 步骤 1: 创建一维汉宁窗向量
    cv::Mat hann1t = cv::Mat(cv::Size(size_patch[1], 1), CV_32F, cv::Scalar(0));
    cv::Mat hann2t = cv::Mat(cv::Size(1, size_patch[0]), CV_32F, cv::Scalar(0));

    // 步骤 2: 余弦窗公式: w(x) = 0.5 * (1 - cos(2*pi*x / (N-1)))
    for (int i = 0; i < hann1t.cols; i++)
        hann1t.at<float>(0, i) = 0.5 * (1 - std::cos(2 * 3.14159265358979323846 * i / (hann1t.cols - 1)));
    for (int i = 0; i < hann2t.rows; i++)
        hann2t.at<float>(i, 0) = 0.5 * (1 - std::cos(2 * 3.14159265358979323846 * i / (hann2t.rows - 1)));

    // 步骤 3: 外积生成二维汉宁窗
    cv::Mat hann2d = hann2t * hann1t;
    // 步骤 4: 对于 HOG 特征，为每个通道复制二维汉宁窗
    if (_hogfeatures) {
        cv::Mat hann1d = hann2d.reshape(1, 1); // Procedure do deal with cv::Mat multichannel bug

        hann = cv::Mat(cv::Size(size_patch[0] * size_patch[1], size_patch[2]), CV_32F, cv::Scalar(0));
        for (int i = 0; i < size_patch[2]; i++) {
            for (int j = 0; j < size_patch[0] * size_patch[1]; j++) {
                hann.at<float>(i, j) = hann1d.at<float>(0, j);
            }
        }
    }
        // 步骤 5: 对于灰度特征，直接使用二维汉宁窗
    else {
        hann = hann2d;
    }
}

// ============================================================================
// subPixelPeak() — 亚像素峰值定位
// ============================================================================

float KCFTracker::subPixelPeak(float left, float center, float right) {
    // 抛物线拟合: offset = 0.5 * (right - left) / (2*center - right - left)
    float divisor = 2 * center - right - left;

    if (divisor == 0)
        return 0;

    return 0.5 * (right - left) / divisor;
}
