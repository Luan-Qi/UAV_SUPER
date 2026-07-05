/**
 * @file PID.cpp
 * @brief 离散 PID 控制器实现。
 *
 * @details
 * 实现标准位置式 PID 控制算法：
 *   output = Kp * error + Ki * integral + Kd * derivative
 *
 * 算法流水线：
 *   1. 计算误差     — error = target - current
 *   2. 积分累加     — integral += error
 *   3. 微分计算     — derivative = error - prevError
 *   4. PID 输出     — targetpoint = Kp*error + Ki*integral + Kd*derivative
 *   5. 状态记录     — prevError = error
 *
 * 依赖：
 *   - PID.h
 */

#include "PID.h"
#include <iostream>
using namespace std;

// ============================================================================
// 构造函数
// ============================================================================

PID::PID(float kp, float ki, float kd) {
    this->kp = kp;
    this->ki = ki;
    this->kd = kd;
}

// ============================================================================
// compute() — PID 控制计算
// ============================================================================

float PID::compute(float target, float current) {
    // 步骤 1: 计算误差
    float error = target - current;
    // 步骤 2: 积分累加
    intergral += error;
    // 步骤 3: 计算与上一次误差的差值（微分项）
    derivative = error - prevError;
    // 步骤 4: 根据 PID 公式计算输出
    targetpoint = kp * error + ki * intergral + kd * derivative;
    // 步骤 5: 记录上一次的误差
    prevError = error;
    return targetpoint;
}

// ============================================================================
// reset() — 重置控制器状态
// ============================================================================

void PID::reset() {
    targetpoint = 0;
    intergral = 0;
    derivative = 0;
    prevError = 0;
}

// ============================================================================
// Set_PID() — 运行时修改 PID 增益
// ============================================================================

void PID::Set_PID(float kp, float ki, float kd) {
    this->kp = kp;
    this->ki = ki;
    this->kd = kd;
}
