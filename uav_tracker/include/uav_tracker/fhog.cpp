/**
 * @file fhog.cpp
 * @brief FHOG（Felzenszwalb 梯度方向直方图）特征提取模块实现。
 *
 * @details
 * 源自 OpenCV latentsvm 模块的快速 HOG（FHOG）特征提取算法实现。
 * 该算法用于 KCF 跟踪器的视觉特征前端，从图像中提取 31 维梯度方向直方图特征。
 *
 * 算法流水线（getFeatureMaps 内部步骤）：
 *   1. 梯度计算      — 使用 [-1, 0, 1] 卷积核在 x 和 y 方向计算三通道梯度
 *   2. 梯度幅值      — 选择三通道中梯度幅值最大的通道
 *   3. 方向离散化    — 将梯度方向分配到 18 个扇区（9 有符号 + 9 无符号）
 *   4. 空间聚合      — 在每个 cell（k×k 像素）内对梯度方向进行加权投票
 *   5. 归一化与截断  — 4 种邻域归一化（2×2 块）后截断至 [0, 0.2]
 *   6. PCA 降维      — 从 108 维降为 31 维标准 FHOG 特征
 *
 * 参考：
 *   - P. Felzenszwalb et al., "Object Detection with Discriminatively Trained Part Based Models", TPAMI 2010.
 *   - OpenCV latentsvm 模块 (lsvmc_featurepyramid.cpp)
 *
 * 来源：OpenCV latentsvm 模块（修改自 "lsvmc_featurepyramid.cpp"）
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


//Modified from latentsvm module's "lsvmc_featurepyramid.cpp".

//#include "precomp.hpp"
//#include "_lsvmc_latentsvm.h"
//#include "_lsvmc_resizeimg.h"

#include "fhog.h"


#ifdef HAVE_TBB
#include <tbb/tbb.h>
#include "tbb/parallel_for.h"
#include "tbb/blocked_range.h"
#endif

#ifndef max
#define max(a,b)            (((a) > (b)) ? (a) : (b))
#endif

#ifndef min
#define min(a,b)            (((a) < (b)) ? (a) : (b))
#endif


// ============================================================================
// getFeatureMaps() — FHOG 特征图提取
// ============================================================================

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
int getFeatureMaps(const IplImage* image, const int k, CvLSVMFeatureMapCaskade **map)
{
    int sizeX, sizeY;
    int p, px, stringSize;
    int height, width, numChannels;
    int i, j, kk, c, ii, jj, d;
    float  * datadx, * datady;

    int   ch;
    float magnitude, x, y, tx, ty;

    IplImage * dx, * dy;
    int *nearest;
    float *w, a_x, b_x;

    // 步骤 1: 定义梯度卷积核 [-1, 0, 1]
    float kernel[3] = {-1.f, 0.f, 1.f};
    CvMat kernel_dx = cvMat(1, 3, CV_32F, kernel);
    CvMat kernel_dy = cvMat(3, 1, CV_32F, kernel);

    float * r;
    int   * alfa;

    float boundary_x[NUM_SECTOR + 1];
    float boundary_y[NUM_SECTOR + 1];
    float max, dotProd;
    int   maxi;

    height = image->height;
    width  = image->width ;

    numChannels = image->nChannels;

    // 步骤 2: 创建 x 和 y 方向梯度图像
    dx    = cvCreateImage(cvSize(image->width, image->height),
                          IPL_DEPTH_32F, 3);
    dy    = cvCreateImage(cvSize(image->width, image->height),
                          IPL_DEPTH_32F, 3);

    // 步骤 3: 计算特征图输出尺寸
    sizeX = width  / k;
    sizeY = height / k;
    px    = 3 * NUM_SECTOR;        // 每个单元的特征数: 9方向 * 3 (有符号+无符号)
    p     = px;
    stringSize = sizeX * p;
    allocFeatureMapObject(map, sizeX, sizeY, p);

    // 步骤 4: 对每个颜色通道分别计算 x 和 y 方向梯度
    cvFilter2D(image, dx, &kernel_dx, cvPoint(-1, 0));
    cvFilter2D(image, dy, &kernel_dy, cvPoint(0, -1));

    // 步骤 5: 预计算 18 个扇区的边界方向向量
    float arg_vector;
    for(i = 0; i <= NUM_SECTOR; i++)
    {
        arg_vector    = ( (float) i ) * ( (float)(PI) / (float)(NUM_SECTOR) );
        boundary_x[i] = cosf(arg_vector);
        boundary_y[i] = sinf(arg_vector);
    }/*for(i = 0; i <= NUM_SECTOR; i++) */

    r    = (float *)malloc( sizeof(float) * (width * height));
    alfa = (int   *)malloc( sizeof(int  ) * (width * height * 2));

    // 步骤 6: 逐像素计算梯度幅值和方向离散化
    for(j = 1; j < height - 1; j++)
    {
        datadx = (float*)(dx->imageData + dx->widthStep * j);
        datady = (float*)(dy->imageData + dy->widthStep * j);
        for(i = 1; i < width - 1; i++)
        {
            c = 0;
            x = (datadx[i * numChannels + c]);
            y = (datady[i * numChannels + c]);

            // 步骤 6a: 初始梯度幅值
            r[j * width + i] =sqrtf(x * x + y * y);
            // 步骤 6b: 从所有通道中选择最大梯度幅值
            for(ch = 1; ch < numChannels; ch++)
            {
                tx = (datadx[i * numChannels + ch]);
                ty = (datady[i * numChannels + ch]);
                magnitude = sqrtf(tx * tx + ty * ty);
                if(magnitude > r[j * width + i])
                {
                    r[j * width + i] = magnitude;
                    c = ch;
                    x = tx;
                    y = ty;
                }
            }/*for(ch = 1; ch < numChannels; ch++)*/

            // 步骤 6c: 方向离散化为 18 个扇区（0-8 有符号，9-17 无符号）
            max  = boundary_x[0] * x + boundary_y[0] * y;
            maxi = 0;
            for (kk = 0; kk < NUM_SECTOR; kk++)
            {
                dotProd = boundary_x[kk] * x + boundary_y[kk] * y;
                if (dotProd > max)
                {
                    max  = dotProd;
                    maxi = kk;
                }
                else
                {
                    if (-dotProd > max)
                    {
                        max  = -dotProd;
                        maxi = kk + NUM_SECTOR;
                    }
                }
            }
            alfa[j * width * 2 + i * 2    ] = maxi % NUM_SECTOR;
            alfa[j * width * 2 + i * 2 + 1] = maxi;
        }/*for(i = 0; i < width; i++)*/
    }/*for(j = 0; j < height; j++)*/

    // 步骤 7: 预计算空间插值权重（双线性插值到相邻 cell）
    nearest = (int  *)malloc(sizeof(int  ) *  k);
    w       = (float*)malloc(sizeof(float) * (k * 2));

    for(i = 0; i < k / 2; i++)
    {
        nearest[i] = -1;
    }/*for(i = 0; i < k / 2; i++)*/
    for(i = k / 2; i < k; i++)
    {
        nearest[i] = 1;
    }/*for(i = k / 2; i < k; i++)*/

    for(j = 0; j < k / 2; j++)
    {
        b_x = k / 2 + j + 0.5f;
        a_x = k / 2 - j - 0.5f;
        w[j * 2    ] = 1.0f/a_x * ((a_x * b_x) / ( a_x + b_x));
        w[j * 2 + 1] = 1.0f/b_x * ((a_x * b_x) / ( a_x + b_x));
    }/*for(j = 0; j < k / 2; j++)*/
    for(j = k / 2; j < k; j++)
    {
        a_x = j - k / 2 + 0.5f;
        b_x =-j + k / 2 - 0.5f + k;
        w[j * 2    ] = 1.0f/a_x * ((a_x * b_x) / ( a_x + b_x));
        w[j * 2 + 1] = 1.0f/b_x * ((a_x * b_x) / ( a_x + b_x));
    }/*for(j = k / 2; j < k; j++)*/

    // 步骤 8: 空间聚合 — 对每个 cell 进行梯度方向加权投票
    for(i = 0; i < sizeY; i++)
    {
      for(j = 0; j < sizeX; j++)
      {
        for(ii = 0; ii < k; ii++)
        {
          for(jj = 0; jj < k; jj++)
          {
            if ((i * k + ii > 0) &&
                (i * k + ii < height - 1) &&
                (j * k + jj > 0) &&
                (j * k + jj < width  - 1))
            {
              d = (k * i + ii) * width + (j * k + jj);
              // 当前 cell 投票
              (*map)->map[ i * stringSize + j * (*map)->numFeatures + alfa[d * 2    ]] +=
                  r[d] * w[ii * 2] * w[jj * 2];
              (*map)->map[ i * stringSize + j * (*map)->numFeatures + alfa[d * 2 + 1] + NUM_SECTOR] +=
                  r[d] * w[ii * 2] * w[jj * 2];
              // 向垂直相邻 cell 投票
              if ((i + nearest[ii] >= 0) &&
                  (i + nearest[ii] <= sizeY - 1))
              {
                (*map)->map[(i + nearest[ii]) * stringSize + j * (*map)->numFeatures + alfa[d * 2    ]             ] +=
                  r[d] * w[ii * 2 + 1] * w[jj * 2 ];
                (*map)->map[(i + nearest[ii]) * stringSize + j * (*map)->numFeatures + alfa[d * 2 + 1] + NUM_SECTOR] +=
                  r[d] * w[ii * 2 + 1] * w[jj * 2 ];
              }
              // 向水平相邻 cell 投票
              if ((j + nearest[jj] >= 0) &&
                  (j + nearest[jj] <= sizeX - 1))
              {
                (*map)->map[i * stringSize + (j + nearest[jj]) * (*map)->numFeatures + alfa[d * 2    ]             ] +=
                  r[d] * w[ii * 2] * w[jj * 2 + 1];
                (*map)->map[i * stringSize + (j + nearest[jj]) * (*map)->numFeatures + alfa[d * 2 + 1] + NUM_SECTOR] +=
                  r[d] * w[ii * 2] * w[jj * 2 + 1];
              }
              // 向对角线相邻 cell 投票
              if ((i + nearest[ii] >= 0) &&
                  (i + nearest[ii] <= sizeY - 1) &&
                  (j + nearest[jj] >= 0) &&
                  (j + nearest[jj] <= sizeX - 1))
              {
                (*map)->map[(i + nearest[ii]) * stringSize + (j + nearest[jj]) * (*map)->numFeatures + alfa[d * 2    ]             ] +=
                  r[d] * w[ii * 2 + 1] * w[jj * 2 + 1];
                (*map)->map[(i + nearest[ii]) * stringSize + (j + nearest[jj]) * (*map)->numFeatures + alfa[d * 2 + 1] + NUM_SECTOR] +=
                  r[d] * w[ii * 2 + 1] * w[jj * 2 + 1];
              }
            }
          }/*for(jj = 0; jj < k; jj++)*/
        }/*for(ii = 0; ii < k; ii++)*/
      }/*for(j = 1; j < sizeX - 1; j++)*/
    }/*for(i = 1; i < sizeY - 1; i++)*/

    // 步骤 9: 释放临时资源
    cvReleaseImage(&dx);
    cvReleaseImage(&dy);


    free(w);
    free(nearest);

    free(r);
    free(alfa);

    return LATENT_SVM_OK;
}

// ============================================================================
// normalizeAndTruncate() — 特征图归一化与截断
// ============================================================================

/*
// Feature map Normalization and Truncation
//
// API
// int normalizeAndTruncate(featureMap *map, const float alfa);
// INPUT
// map               - feature map
// alfa              - truncation threshold
// OUTPUT
// map               - truncated and normalized feature map
// RESULT
// Error status
*/
int normalizeAndTruncate(CvLSVMFeatureMapCaskade *map, const float alfa)
{
    int i,j, ii;
    int sizeX, sizeY, p, pos, pp, xp, pos1, pos2;
    float * partOfNorm;    // C(i,j) 的 L2 范数
    float * newData;
    float   valOfNorm;

    sizeX     = map->sizeX;
    sizeY     = map->sizeY;
    partOfNorm = (float *)malloc (sizeof(float) * (sizeX * sizeY));

    p  = NUM_SECTOR;
    xp = NUM_SECTOR * 3;
    pp = NUM_SECTOR * 12;

    // 步骤 1: 计算每个 cell 的归一化因子（前 9 维 L2 范数）
    for(i = 0; i < sizeX * sizeY; i++)
    {
        valOfNorm = 0.0f;
        pos = i * map->numFeatures;
        for(j = 0; j < p; j++)
        {
            valOfNorm += map->map[pos + j] * map->map[pos + j];
        }/*for(j = 0; j < p; j++)*/
        partOfNorm[i] = valOfNorm;
    }/*for(i = 0; i < sizeX * sizeY; i++)*/

    sizeX -= 2;
    sizeY -= 2;

    newData = (float *)malloc (sizeof(float) * (sizeX * sizeY * pp));
    // 步骤 2: 4 种邻域归一化（2x2 块的上、下、左、右组合）
//normalization
    for(i = 1; i <= sizeY; i++)
    {
        for(j = 1; j <= sizeX; j++)
        {
            // 归一化类型 1: 右下邻域
            valOfNorm = sqrtf(
                partOfNorm[(i    )*(sizeX + 2) + (j    )] +
                partOfNorm[(i    )*(sizeX + 2) + (j + 1)] +
                partOfNorm[(i + 1)*(sizeX + 2) + (j    )] +
                partOfNorm[(i + 1)*(sizeX + 2) + (j + 1)]) + FLT_EPSILON;
            pos1 = (i  ) * (sizeX + 2) * xp + (j  ) * xp;
            pos2 = (i-1) * (sizeX    ) * pp + (j-1) * pp;
            for(ii = 0; ii < p; ii++)
            {
                newData[pos2 + ii        ] = map->map[pos1 + ii    ] / valOfNorm;
            }/*for(ii = 0; ii < p; ii++)*/
            for(ii = 0; ii < 2 * p; ii++)
            {
                newData[pos2 + ii + p * 4] = map->map[pos1 + ii + p] / valOfNorm;
            }/*for(ii = 0; ii < 2 * p; ii++)*/
            // 归一化类型 2: 右上邻域
            valOfNorm = sqrtf(
                partOfNorm[(i    )*(sizeX + 2) + (j    )] +
                partOfNorm[(i    )*(sizeX + 2) + (j + 1)] +
                partOfNorm[(i - 1)*(sizeX + 2) + (j    )] +
                partOfNorm[(i - 1)*(sizeX + 2) + (j + 1)]) + FLT_EPSILON;
            for(ii = 0; ii < p; ii++)
            {
                newData[pos2 + ii + p    ] = map->map[pos1 + ii    ] / valOfNorm;
            }/*for(ii = 0; ii < p; ii++)*/
            for(ii = 0; ii < 2 * p; ii++)
            {
                newData[pos2 + ii + p * 6] = map->map[pos1 + ii + p] / valOfNorm;
            }/*for(ii = 0; ii < 2 * p; ii++)*/
            // 归一化类型 3: 左下邻域
            valOfNorm = sqrtf(
                partOfNorm[(i    )*(sizeX + 2) + (j    )] +
                partOfNorm[(i    )*(sizeX + 2) + (j - 1)] +
                partOfNorm[(i + 1)*(sizeX + 2) + (j    )] +
                partOfNorm[(i + 1)*(sizeX + 2) + (j - 1)]) + FLT_EPSILON;
            for(ii = 0; ii < p; ii++)
            {
                newData[pos2 + ii + p * 2] = map->map[pos1 + ii    ] / valOfNorm;
            }/*for(ii = 0; ii < p; ii++)*/
            for(ii = 0; ii < 2 * p; ii++)
            {
                newData[pos2 + ii + p * 8] = map->map[pos1 + ii + p] / valOfNorm;
            }/*for(ii = 0; ii < 2 * p; ii++)*/
            // 归一化类型 4: 左上邻域
            valOfNorm = sqrtf(
                partOfNorm[(i    )*(sizeX + 2) + (j    )] +
                partOfNorm[(i    )*(sizeX + 2) + (j - 1)] +
                partOfNorm[(i - 1)*(sizeX + 2) + (j    )] +
                partOfNorm[(i - 1)*(sizeX + 2) + (j - 1)]) + FLT_EPSILON;
            for(ii = 0; ii < p; ii++)
            {
                newData[pos2 + ii + p * 3 ] = map->map[pos1 + ii    ] / valOfNorm;
            }/*for(ii = 0; ii < p; ii++)*/
            for(ii = 0; ii < 2 * p; ii++)
            {
                newData[pos2 + ii + p * 10] = map->map[pos1 + ii + p] / valOfNorm;
            }/*for(ii = 0; ii < 2 * p; ii++)*/
        }/*for(j = 1; j <= sizeX; j++)*/
    }/*for(i = 1; i <= sizeY; i++)*/
    // 步骤 3: 截断 — 将所有值限制在 [0, alfa] 范围内
//truncation
    for(i = 0; i < sizeX * sizeY * pp; i++)
    {
        if(newData [i] > alfa) newData [i] = alfa;
    }/*for(i = 0; i < sizeX * sizeY * pp; i++)*/
//swop data

    map->numFeatures  = pp;
    map->sizeX = sizeX;
    map->sizeY = sizeY;

    free (map->map);
    free (partOfNorm);

    map->map = newData;

    return LATENT_SVM_OK;
}

// ============================================================================
// PCAFeatureMaps() — PCA 降维
// ============================================================================

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
int PCAFeatureMaps(CvLSVMFeatureMapCaskade *map)
{
    int i,j, ii, jj, k;
    int sizeX, sizeY, p,  pp, xp, yp, pos1, pos2;
    float * newData;
    float val;
    float nx, ny;

    sizeX = map->sizeX;
    sizeY = map->sizeY;
    p     = map->numFeatures;
    pp    = NUM_SECTOR * 3 + 4;     // 输出维度: 31
    yp    = 4;
    xp    = NUM_SECTOR;

    nx    = 1.0f / sqrtf((float)(xp * 2));
    ny    = 1.0f / sqrtf((float)(yp    ));

    newData = (float *)malloc (sizeof(float) * (sizeX * sizeY * pp));

    // 步骤 1: 对每个 cell 执行 PCA 投影
    for(i = 0; i < sizeY; i++)
    {
        for(j = 0; j < sizeX; j++)
        {
            pos1 = ((i)*sizeX + j)*p;
            pos2 = ((i)*sizeX + j)*pp;
            k = 0;
            // 步骤 1a: 18 维方向敏感特征
            for(jj = 0; jj < xp * 2; jj++)
            {
                val = 0;
                for(ii = 0; ii < yp; ii++)
                {
                    val += map->map[pos1 + yp * xp + ii * xp * 2 + jj];
                }/*for(ii = 0; ii < yp; ii++)*/
                newData[pos2 + k] = val * ny;
                k++;
            }/*for(jj = 0; jj < xp * 2; jj++)*/
            // 步骤 1b: 9 维方向不敏感特征
            for(jj = 0; jj < xp; jj++)
            {
                val = 0;
                for(ii = 0; ii < yp; ii++)
                {
                    val += map->map[pos1 + ii * xp + jj];
                }/*for(ii = 0; ii < yp; ii++)*/
                newData[pos2 + k] = val * ny;
                k++;
            }/*for(jj = 0; jj < xp; jj++)*/
            // 步骤 1c: 4 维纹理特征
            for(ii = 0; ii < yp; ii++)
            {
                val = 0;
                for(jj = 0; jj < 2 * xp; jj++)
                {
                    val += map->map[pos1 + yp * xp + ii * xp * 2 + jj];
                }/*for(jj = 0; jj < xp; jj++)*/
                newData[pos2 + k] = val * nx;
                k++;
            } /*for(ii = 0; ii < yp; ii++)*/
        }/*for(j = 0; j < sizeX; j++)*/
    }/*for(i = 0; i < sizeY; i++)*/
//swop data

    map->numFeatures = pp;

    free (map->map);

    map->map = newData;

    return LATENT_SVM_OK;
}


//modified from "lsvmc_routine.cpp"

// ============================================================================
// allocFeatureMapObject() — 分配特征图对象内存
// ============================================================================

int allocFeatureMapObject(CvLSVMFeatureMapCaskade **obj, const int sizeX,
                          const int sizeY, const int numFeatures)
{
    int i;
    (*obj) = (CvLSVMFeatureMapCaskade *)malloc(sizeof(CvLSVMFeatureMapCaskade));
    (*obj)->sizeX       = sizeX;
    (*obj)->sizeY       = sizeY;
    (*obj)->numFeatures = numFeatures;
    (*obj)->map = (float *) malloc(sizeof (float) *
                                  (sizeX * sizeY  * numFeatures));
    for(i = 0; i < sizeX * sizeY * numFeatures; i++)
    {
        (*obj)->map[i] = 0.0f;
    }
    return LATENT_SVM_OK;
}

// ============================================================================
// freeFeatureMapObject() — 释放特征图对象内存
// ============================================================================

int freeFeatureMapObject (CvLSVMFeatureMapCaskade **obj)
{
    if(*obj == NULL) return LATENT_SVM_MEM_NULL;
    free((*obj)->map);
    free(*obj);
    (*obj) = NULL;
    return LATENT_SVM_OK;
}
