# AGENTS.md — uav_tracker

> 给后续 agent 的速查手册。**短、准、可迭代**。每犯一次错就更新一次。

## 目录结构

```
uav_tracker/
├── README.md               ← 本包说明
├── AGENTS.md               ← 本文件
├── CMakeLists.txt           ← 编译配置 (kcf_tracker 库 + 2 个 C++ 节点)
├── package.xml              ← 包依赖
├── include/uav_tracker/
│   ├── tracker.h            ← 跟踪器抽象基类
│   ├── kcftracker.h         ← KCF 算法头 (继承 Tracker)
│   ├── kcftracker.cpp       ← KCF 算法实现 (检测/训练/高斯核/多尺度)
│   ├── KCF_Tracker.h        ← ROS ImageConverter 类声明
│   ├── fftools.h            ← FFT 复数运算工具 (namespace FFTTools)
│   ├── fhog.h               ← FHOG 特征提取头 (OpenCV latentsvm)
│   ├── fhog.cpp             ← FHOG 特征提取实现 (梯度直方图+PCA降维)
│   ├── recttools.h          ← 矩形/ROI 工具函数 (namespace RectTools)
│   ├── labdata.h            ← Lab 颜色空间 15 个聚类中心点
│   ├── PID.h                ← 一维 PID 控制器头
│   └── PID.cpp              ← 一维 PID 控制器实现
├── src/
│   ├── KCF_Tracker.cpp      ← KCF ROS 节点 (鼠标选ROI + 跟踪 + 深度)
│   └── KCF_Tracker_control.cpp ← 视觉伺服控制节点 (3模式 PID)
├── launch/
│   ├── KCFTracker.launch    ← KCF + Orbbec 相机启动
│   └── AstraTracker.launch  ← Python 颜色跟踪启动
├── script/
│   └── astra_Tracker.py     ← Astra 相机 Python 跟踪脚本
└── cfg/                     ← dynamic_reconfigure 配置
```

## 构建与运行

```bash
# 编译
catkin build uav_tracker

# KCF + Orbbec 相机 (完整 pipeline)
roslaunch uav_tracker KCFTracker.launch

# 仅 KCF 跟踪 (无相机，需外部提供 /camera/* 话题)
rosrun uav_tracker KCFTracker_node _limit_max_dist:=10.0 _limit_min_dist:=0.3

# 视觉伺服控制
rosrun uav_tracker KCFTracker_control_node _mode:=0 _desired_distance:=1.0

# Python Astra 跟踪
roslaunch uav_tracker AstraTracker.launch tracker_type:=color
```

## 话题接口

```
/camera/color/image_raw ──→ KCF_Tracker ──→ /tracker/target_position
/camera/depth/image_raw ──→              ──→ /tracker/is_tracking
                                          ──→ RGB_WINDOW (imshow)

/tracker/target_position ──→ KCF_Tracker_control ──→ /mavros/setpoint_velocity/cmd_vel_unstamped
/mavros/local_position/pose ──→ 
/mavros/state ──→
```

## 工程约定

- C++11, ROS1 catkin, OpenCV (C++ API + 部分 C API for FHOG)
- KCF 算法无命名空间，全局作用域
- FFT 工具在 `FFTTools` 命名空间，矩形工具在 `RectTools` 命名空间
- PID 类在 `KCF_Tracker_control.cpp` 中有独立副本（不同于 `include/uav_tracker/PID`）
- 图像话题编码: BGR8 (color), TYPE_32FC1 (depth, 单位 mm)
- 深度值从 mm 转 m: `depthimage.at<float>(y,x) / 1000`
- 新增参数: struct/class 中添加字段 → 构造函数设默认值 → `nh.param()` 加载 → launch 文件添加 arg

## 算法流水线

### KCF 跟踪 (kcftracker.cpp)
1. 边界约束 — 确保 ROI 不超出图像边界
2. 特征提取 — `getFeatures()`: 提取 ROI → resize → FHOG+Lab 或 RAW 像素
3. 检测 — `detect()`: 高斯核相关 → FFT → 响应图 → 亚像素峰值定位
4. 多尺度 — 以 `1/scale_step` 和 `scale_step` 各检测一次，选最优
5. 位置更新 — `cx/res.x +/- width/2` 计算新 ROI
6. 在线训练 — `train()`: 高斯核自相关 → 频域除法 → 线性插值更新模板

### 视觉伺服控制 (KCF_Tracker_control.cpp)
1. 检查状态 — OFFBOARD 模式 + is_tracking 标志
2. 计算距离误差 — `depth - desired_distance`
3. 相机系速度 — `v_cam_forward = PID(dist_error)`, `v_cam_vertical = PID(-pixel_err_y)`
4. 姿态补偿 — 总俯仰角 `camera_mount_pitch + (-current_pitch)`
5. 速度分解 — `v_body_x = v_cam_forward*cos(θ) - v_cam_vertical*sin(θ)`
6. 按模式发布 — MODE_FIXED_POS: yaw only; MODE_FOLLOW: XYZ; MODE_FOLLOW_NO_YAW: XY+Z

## 已知问题与陷阱

- KCF 需要手动鼠标框选 ROI（无自动检测）
- Orbbec SDK 需预先安装，否则 `KCFTracker.launch` 相机包含会失败
- LAB 特征仅在 HOG=true 时有效；RAW 模式下 LAB 强制关闭
- KCF 默认参数针对 VOT 基准调优，实际场景可能需要调整 `interp_factor`, `sigma`, `padding`
- `KCF_Tracker_control.cpp` 中 PID 类与 `include/uav_tracker/PID.h` 是独立实现，参数和接口不同
- 深度图像为 32FC1 (mm)，需除 1000 转米；`minDist == 0` 或 inf 时视为丢失
- `template_size / cell_size` 应为 2 的幂或小质数乘积以获得最佳 FFT 性能
- `AstraTracker.launch` 中的节点类型 `astra_Tracker.py` 位于 `script/` 目录

## 验收标准

- [ ] `catkin build uav_tracker` 编译成功
- [ ] `KCFTracker.launch` 启动后在 RGB 窗口中可鼠标框选目标
- [ ] 框选后跟踪框持续跟随目标，`/tracker/is_tracking` 为 true
- [ ] `/tracker/target_position` 发布目标像素误差和深度
- [ ] 按 `r` 重置 ROI，按 `q` 退出
- [ ] 深度在有效范围内 [min_dist, max_dist] 时正常取值
- [ ] `KCF_Tracker_control_node` 在跟踪状态下发布速度指令

## 迭代记录

<!-- 格式: YYYY-MM-DD 问题描述 → 解决方法 -->

2026-07-05 代码注释规范化:
  - 所有源文件和 launch 文件添加 Doxygen/XML 注释
  - 创建包级 README.md 和 AGENTS.md
  - 修复 PID.h/PID.cpp 中乱码中文注释
