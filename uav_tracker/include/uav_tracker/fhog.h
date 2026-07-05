/**
 * @file fhog.h
 * @brief FHOG（Felzenszwalb 梯度方向直方图）特征提取模块声明。
 *
 * @details
 * 源自 OpenCV latentsvm 模块的快速 HOG（FHOG）特征提取实现。
 * 用于 KCF 跟踪器的多通道视觉特征提取。
 *
 * 核心功能：
 *   1. getFeatureMaps()                    — 从图像提取 FHOG 特征图
 *   2. normalizeAndTruncate()              — 特征图归一化与截断
 *   3. PCAFeatureMaps()                    — PCA 降维（32 → 31 维）
 *   4. allocFeatureMapObject()             — 分配特征图对象内存
 *   5. freeFeatureMapObject()              — 释放特征图对象内存
 *
 * 特征维度：
 *   - 原始梯度直方图：9 bins × (有符号 + 无符号) = 18 维
 *   - 纹理特征：4 × 9 bins = 36 维
 *   - 归一化与截断后：4 × 9 bins × 4 邻域归一化 = 108 维
 *   - PCA 降维后：31 维（18 方向敏感 + 9 方向不敏感 + 4 纹理）
 *
 * 数据结构：
 *   CvLSVMFeatureMapCaskade — 特征图容器 (sizeX × sizeY 单元，每单元 numFeatures 维)
 *
 * 参考：
 *   - P. Felzenszwalb et al., "Object Detection with Discriminatively Trained Part Based Models", TPAMI 2010.
 *   - OpenCV latentsvm 模块 (lsvmc_featurepyramid.cpp)
 *
 * 来源：OpenCV latentsvm 模块（修改自 "_lsvmc_latentsvm.h" 与 "lsvmc_routine.h"）
 */

/*M///////////////////////////////////////////////////////////////////////////////////////
//
//  IMPORTANT: READ BEFORE DOWNLOADING, COPYING, INSTALLING OR USING.
//
//  By downloading, copying, installing or using the software you agree to this license.
//  If you do not agree to this license, do not download, install,
//  copy or use the software.
//
//
//                           License Agreement
//                For Open Source Computer Vision Library
//
// Copyright (C) 2010-2013, University of Nizhny Novgorod, all rights reserved.
// Third party copyrights are property of their respective owners.
//
// Redistribution and use in source and binary forms, with or without modification,
// are permitted provided that the following conditions are met:
//
//   * Redistribution's of source code must retain the above copyright notice,
//     this list of conditions and the following disclaimer.
//
//   * Redistribution's in binary form must reproduce the above copyright notice,
//     this list of conditions and the following disclaimer in the documentation
//     and/or other materials provided with the distribution.
//
//   * The name of the copyright holders may not be used to endorse or promote products
//     derived from this software without specific prior written permission.
//
// This software is provided by the copyright holders and contributors "as is" and
// any express or implied warranties, including, but not limited to, the implied
// warranties of merchantability and fitness for a particular purpose are disclaimed.
// In no event shall the Intel Corporation or contributors be liable for any direct,
// indirect, incidental, special, exemplary, or consequential damages
// (including, but not limited to, procurement of substitute goods or services;
// loss of use, data, or profits; or business interruption) however caused
// and on any theory of liability, whether in contract, strict liability,
// or tort (including negligence or otherwise) arising in any way out of
// the use of this software, even if advised of the possibility of such damage.
//
//M*/


//Modified from latentsvm module's "_lsvmc_latentsvm.h".


/*****************************************************************************/
/*                      Latent SVM prediction API                            */
/*****************************************************************************/

#ifndef _FHOG_H_
#define _FHOG_H_

#include <stdio.h>
//#include "_lsvmc_types.h"
//#include "_lsvmc_error.h"
//#include "_lsvmc_routine.h"

//#include "opencv2/imgproc.hpp"
#include "opencv2/imgproc/imgproc_c.h"

// ============================================================================
// CvLSVMFeatureMapCaskade — 特征图数据结构
// ============================================================================

//modified from "_lsvmc_types.h"

// DataType: STRUCT featureMap
// FEATURE MAP DESCRIPTION
//   Rectangular map (sizeX x sizeY),
//   every cell stores feature vector (dimension = numFeatures)
// map             - matrix of feature vectors
//                   to set and get feature vectors (i,j)
//                   used formula map[(j * sizeX + i) * p + k], where
//                   k - component of feature vector in cell (i, j)
typedef struct{
    int sizeX;          ///< 特征图水平单元数
    int sizeY;          ///< 特征图垂直单元数
    int numFeatures;    ///< 每个单元的特征向量维数
    float *map;         ///< 特征向量矩阵（按行优先存储，索引公式: map[(j*sizeX + i)*p + k]） [sizeX*sizeY*numFeatures]
} CvLSVMFeatureMapCaskade;


#include "float.h"

#define PI    CV_PI                        ///< 圆周率 π

#define EPS 0.000001                       ///< 浮点比较容差

#define F_MAX FLT_MAX                      ///< 最大浮点值
#define F_MIN -FLT_MAX                     ///< 最小浮点值

// The number of elements in bin
// The number of sectors in gradient histogram building
#define NUM_SECTOR 9                       ///< 梯度方向直方图的扇区数（每 20° 一个 bin，覆盖 180°）

// The number of levels in image resize procedure
// We need Lambda levels to resize image twice
#define LAMBDA 10                          ///< 图像缩放的最大层数

// Block size. Used in feature pyramid building procedure
#define SIDE_LENGTH 8                      ///< 归一化块尺寸 [pixel]

#define VAL_OF_TRUNCATE 0.2f               ///< 截断阈值


//modified from "_lsvm_error.h"
#define LATENT_SVM_OK 0                    ///< 操作成功
#define LATENT_SVM_MEM_NULL 2              ///< 内存分配失败（空指针）
#define DISTANCE_TRANSFORM_OK 1
#define DISTANCE_TRANSFORM_GET_INTERSECTION_ERROR -1
#define DISTANCE_TRANSFORM_ERROR -2
#define DISTANCE_TRANSFORM_EQUAL_POINTS -3
#define LATENT_SVM_GET_FEATURE_PYRAMID_FAILED -4
#define LATENT_SVM_SEARCH_OBJECT_FAILED -5
#define LATENT_SVM_FAILED_SUPERPOSITION -6
#define FILTER_OUT_OF_BOUNDARIES -7
#define LATENT_SVM_TBB_SCHEDULE_CREATION_FAILED -8
#define LATENT_SVM_TBB_NUMTHREADS_NOT_CORRECT -9
#define FFT_OK 2
#define FFT_ERROR -10
#define LSVM_PARSER_FILE_NOT_FOUND -11



/*
// Getting feature map for the selected subimage
//
// API
// int getFeatureMaps(const IplImage * image, const int k, featureMap **map);
// INPUT
// image             - selected subimage
// k                 - size of cells
// OUTPUT
// map               - feature map
// RESULT
// Error status
*/

// ============================================================================
// 函数声明
// ============================================================================

/** @brief 从图像中提取 FHOG 特征图。
 *  @param image 输入图像（IplImage 格式）
 *  @param k     单元尺寸 [pixel]（每个 cell 包含 k×k 像素）
 *  @param map   [out] 输出的特征图对象
 *  @return LATENT_SVM_OK 成功，负值表示错误
 */
int getFeatureMaps(const IplImage * image, const int k, CvLSVMFeatureMapCaskade **map);


/*
// Feature map Normalization and Truncation
//
// API
// int normalizationAndTruncationFeatureMaps(featureMap *map, const float alfa);
// INPUT
// map               - feature map
// alfa              - truncation threshold
// OUTPUT
// map               - truncated and normalized feature map
// RESULT
// Error status
*/

/** @brief 特征图归一化与截断。
 *  @param map  [in/out] 特征图（输入 18 维，输出 108 维）
 *  @param alfa 截断阈值（超出此值的元素被截断，典型: 0.2）
 *  @return LATENT_SVM_OK 成功
 *
 *  对每个单元进行 4 种邻域归一化（上、下、左、右 2×2 块），
 *  每种归一化输出 18 维有符号梯度 + 18 维无符号梯度 = 36 维，
 *  共计 36 × 4 种归一化 + 9 维附加 = 108 维，
 *  最后将所有值截断到 [0, alfa]。
 */
int normalizeAndTruncate(CvLSVMFeatureMapCaskade *map, const float alfa);

/*
// Feature map reduction
// In each cell we reduce dimension of the feature vector
// according to original paper special procedure
//
// API
// int PCAFeatureMaps(featureMap *map)
// INPUT
// map               - feature map
// OUTPUT
// map               - feature map
// RESULT
// Error status
*/

/** @brief 特征图降维（PCA-style 投影）。
 *  @param map [in/out] 特征图（输入 108 维，输出 31 维）
 *  @return LATENT_SVM_OK 成功
 *
 *  将 108 维归一化特征降为 31 维：
 *    - 18 维：方向敏感特征（有符号梯度，9 bins × 2 方向归一化）
 *    - 9 维：  方向不敏感特征（无符号梯度）
 *    - 4 维：  纹理特征（4 种归一化下的梯度能量和）
 */
int PCAFeatureMaps(CvLSVMFeatureMapCaskade *map);


//modified from "lsvmc_routine.h"

/** @brief 分配特征图对象内存。
 *  @param obj        [out] 分配的特征图对象指针
 *  @param sizeX      水平单元数
 *  @param sizeY      垂直单元数
 *  @param numFeatures 每单元特征维数
 *  @return LATENT_SVM_OK 成功，LATENT_SVM_MEM_NULL 失败
 *
 *  分配内存并将所有特征值初始化为 0。
 */
int allocFeatureMapObject(CvLSVMFeatureMapCaskade **obj, const int sizeX, const int sizeY,
                          const int p);

/** @brief 释放特征图对象内存。
 *  @param obj [in/out] 待释放的特征图对象指针（释放后置 NULL）
 *  @return LATENT_SVM_OK 成功，LATENT_SVM_MEM_NULL 空指针
 */
int freeFeatureMapObject (CvLSVMFeatureMapCaskade **obj);


#endif
