/**
 * @file PID.h
 * @brief 离散 PID 控制器类声明。
 *
 * @details
 * 实现标准位置式 PID 控制算法，用于目标跟踪中的运动控制。
 *
 * 核心功能：
 *   1. compute()   — 根据目标值与当前值计算控制输出
 *   2. reset()     — 重置积分累加与上一次误差状态
 *   3. Set_PID()   — 运行时修改 PID 增益参数
 *
 * PID 公式：
 *   output = Kp * error + Ki * integral + Kd * derivative
 *   error       = target - current
 *   integral   += error
 *   derivative  = error - prevError
 *
 * 依赖：
 *   - 无外部依赖（仅需标准 C++）
 */

// ============================================================================
// PID 控制器
// ============================================================================

class PID {
public:
    float kp;             ///< 比例增益
    float ki;             ///< 积分增益
    float kd;             ///< 微分增益
    float targetpoint;    ///< 控制输出值
    float prevError;      ///< 上一次的误差
    float intergral;      ///< 误差积分累计值
    float derivative;     ///< 误差微分量

    /** @brief 构造函数，设定 PID 增益参数。
     *  @param kp 比例增益
     *  @param ki 积分增益
     *  @param kd 微分增益
     */
    PID(float kp, float ki, float kd);

    /** @brief 运行时修改 PID 增益参数。
     *  @param kp 新的比例增益
     *  @param ki 新的积分增益
     *  @param kd 新的微分增益
     */
    void Set_PID(float kp, float ki, float kd);

    /**
     * @brief PID 控制计算函数。
     * @param target  目标值
     * @param current 当前值
     * @return 控制输出值
     */
    float compute(float target, float current);

    /**
     * @brief 重置控制器状态。
     *
     * 将积分累加、上一次误差和控制输出全部清零，
     * 使得后续控制输出不受历史状态影响。
     */
    void reset();
};
