# AGENTS.md — fast_ofio

> 给后续 agent 的速查手册。**短、准、可迭代**。每犯一次错就更新一次。

## 目录结构

```
uav_location/src/fast_ofio/
├── AGENTS.md              ← 本文件
├── ofio_types.h           ← 数据结构 + OfioConfig（所有 ROS param 定义在此）
├── ofio_estimator.h       ← OfioEstimator 接口
├── ofio_estimator.cpp     ← 核心算法（PX4 EKF2 optflow 参考实现）
├── fast_ofio_node.cpp     ← ROS 节点入口（订阅/发布/定时器）
uav_location/launch/
└── fast_ofio.launch       ← 启动文件（所有参数可配置）
uav_location/CMakeLists.txt ← 添加了 fast_ofio target + mavros_msgs 依赖
uav_location/package.xml    ← 添加了 mavros_msgs 依赖
```

## 构建与运行

```bash
# 编译
catkin build uav_location

# 运行（默认参数）
roslaunch uav_location fast_ofio.launch

# 带传感器偏移的运行
roslaunch uav_location fast_ofio.launch \
    flow_pos_x:=0.05 flow_pos_y:=-0.02 flow_pos_z:=0.10 \
    scale_factor:=1.05
```

## 话题接口

| 方向 | 话题 | 类型 |
|------|------|------|
| 订阅 | `/mavros/optical_flow_rad/raw/send` | `mavros_msgs::OpticalFlowRad` |
| 订阅 | `/mavros/imu/data` | `sensor_msgs::Imu` |
| 发布 | `/Odometry` | `nav_msgs::Odometry` （含6×6协方差） |

## 工程约定

- C++11，ROS1 catkin，Eigen3
- 命名空间 `fast_ofio`，节点名 `fast_ofio`
- 所有 `.h` 和 `.cpp` 放在同一个 `src/fast_ofio/` 目录（不用 `include/` 分离）
- include 用引号形式：`#include "ofio_types.h"`
- ROS param 通过 `OfioConfig::loadFromParamServer()` 统一加载，默认值在构造函数
- 新增参数：在 `OfioConfig` 结构体中添加字段 → 构造函数设默认值 → `loadFromParamServer` 加载 → launch 文件添加 arg → 估计器中使用 `config_.xxx`

## 算法要点

参考 PX4 `ekf2/EKF/optflow_control.cpp`，流水线：

1. 时间同步（flow-IMU dt < max_flow_imu_dt）
2. 质量门限（quality + distance>0 基础检查）
3. **Range 初始偏移捕获**（auto_range_offset: 首次合法帧记录离地距离，在 classify/updateHeight 之前）
4. **Range 三分类** `classifyRange()` → NORMAL / OBSTACLE / INVALID
   - innovation-based：`effective_range - predicted_global_z`
   - hysteresis: confirm_frames + recover_frames 防抖动
   - IMU 垂向加速度辅助判定
5. **确定 flow_range_for_xy**（NORMAL→raw, OBSTACLE→fallback, INVALID→skipXY）
6. 旋转补偿 `flow_trans = flow_meas - gyro_integrated`
7. 尺度+安装角校正
8. 偏置补偿
9. 高度补偿 `v = f(flow, flow_range_for_xy, dt)` ← 使用 flow_range_for_xy 非 raw distance
10. **杠杆臂补偿** `v_body = v_sensor - ω × r_offset`
11. 坐标变换 body→odom（FLU 机体对齐 或 ENU 世界系）+ 低通滤波
12. **高度更新** `updateHeight(effective_range, dt, RangeStatus)`
    - NORMAL: range 修正 global Z + 估计 height_velocity
    - OBSTACLE: 拒绝 range 校正，last_valid_height_velocity_ 衰减传播
    - INVALID: 仅预测传播，速度快速衰减
13. 位移积分（XY 来自光流，Z 来自 updateHeight）
14. 偏置在线估计（仅 NORMAL 时更新，静止判定 → LPF）
15. 协方差传播（使用 flow_range_for_xy，按 RangeStatus 缩放 XY/Z）

## 坐标系

- Body: FRD (X-前 Y-右 Z-下)
- 里程计（默认 `use_body_frame_odom=true`）: FLU (X-前 Y-左 Z-上)，机体对齐，不随 yaw 旋转
- 里程计（`use_body_frame_odom=false`）: World ENU (X-东 Y-北 Z-上)，与 MAVROS 一致
- 光流传感器朝下安装，安装偏移 `flow_pos_z` 下为正

### 坐标系模式说明

- **FLU body-aligned (默认)**: velocity 不旋转到世界系，直接沿初始机体方向积分。
  前→+X, 左→+Y, 升→+Z。适合无外部航向参考的纯光流里程计。
- **ENU world**: velocity 通过 IMU 姿态旋转到世界 ENU 系。
  东→+X, 北→+Y, 上→+Z。适合有外部航向观测量或融合场景。

## 限制与待改进

- 仅支持单目光流，不支持双目/多传感器
- Yaw 航向完全依赖 IMU 积分（长期无观测量会漂移）
- 偏置估计仅在静止时更新，动态时偏置不变
- 未使用外部视觉/激光里程计做多源融合
- Z 通道障碍物检测依赖 IMU 垂向加速度低通值，极慢速升降可能与障碍物跳变难以区分

## 验收标准

- [ ] 订阅到 `mavros_msgs::OpticalFlowRad` 和 `sensor_msgs::Imu` 后能正常初始化
- [ ] 静止时里程计位置在 1m 范围内不漂移超过 0.1m（偏置估计生效）
- [ ] 静止地面时 Z=0（auto_range_offset 自动扣除初始离地距离）
- [ ] 传感器偏移参数变化后速度估计相应补偿（杠杆臂正确）
- [ ] 质量门限正确丢弃低质量帧
- [ ] 光流超时后协方差增大、位置保持
- [ ] 输出 `/Odometry` 的 frame_id/child_frame_id 与配置一致
- [ ] 前移→X 增大，左移→Y 增大，上升→Z 增大（FLU body-aligned 模式）
- [ ] 定高飞行中障碍物出现/消失时 Z 不跳变

## 迭代记录

<!-- 遇到问题在此记录，下次 agent 直接扫码此项避免重复犯错 -->
<!-- 格式: YYYY-MM-DD HH:MM 问题描述 → 解决方法 -->

2026-06-17 18:15 两套高度语义重构：
  - 核心设计：拆分 local/surface range（XY光流尺度用）和 global odom z（里程计高度输出）。
  - Range 三分类 RangeStatus {NORMAL, OBSTACLE, INVALID}：采用 innovation-based
    状态机（effective_range - predicted_global_z），带 hysteresis（confirm_frames=3
    recover_frames=5），配合 IMU 垂向加速度判断变化是否可信。
  - flow_range_for_xy 解耦：NORMAL 时用 raw flow.distance 并更新 last_trusted_flow_range_；
    OBSTACLE 时用 fallback last_trusted_flow_range_，若无则跳过 XY；INVALID 跳过 XY。
  - updateHeight(..., RangeStatus)：NORMAL 时 range 修正 global z + 估计 height_velocity；
    OBSTACLE 时拒绝 range 校正，用 last_valid_height_velocity_ 衰减传播 Z（tau=2.0s），
    前 0.5s 叠加 IMU 加速度；INVALID 同但速度衰减更快。max_z_prediction_time=5.0s。
  - 偏置仅在 NORMAL 时更新；协方差使用 flow_range_for_xy 计算，OBSTACLE/INVALID 时
    XY/Z 按状态放大（OBSTACLE XY×2, INVALID XY×5; Z 随 z_prediction_time_ 线性增长）。
  - 初始 offset 捕获移至 classifyRange/updateHeight 之前，确保第一帧 Z=0。
  - 新增参数：obstacle_confirm_frames=3, obstacle_recover_frames=5,
    obstacle_recovery_threshold=0.2m, z_velocity_decay_tau=2.0s, max_z_prediction_time=5.0s。
  - 原 Z_imu_stationary 逻辑移除，不再简单 freeze+vel=0；改为预测传播保证斜向升降不断。

2026-06-17 17:50 初始三轮修复：
  - Range 初始偏移：`auto_range_offset`(default true)，首次合法帧捕获 offset，Z 用 effective_range。
  - Z 轴重写：去掉互补滤波，range 直读 + LPF + 障碍物检测（已被本次两套语义重构替代）。
  - 坐标系：`use_body_frame_odom`(default true) FLU 机体对齐，`odom_frame_id`(default "odom")。
