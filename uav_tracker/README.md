# uav_tracker — KCF视觉目标跟踪与视觉伺服

## 功能简介

基于核化相关滤波（KCF）的实时视觉目标跟踪包，支持 HOG+Lab 多特征融合、多尺度估计、亚像素峰值定位。
配合深度相机可实现目标 3D 定位，并通过视觉伺服节点控制无人机跟踪目标。

## 节点说明

| 节点 | 类型 | 功能 |
|------|------|------|
| `KCF_Tracker` | C++ | KCF 目标跟踪 — 鼠标框选 ROI，发布目标像素误差与深度 |
| `KCF_Tracker_control` | C++ | 视觉伺服控制 — 根据目标位置计算无人机速度指令，3 种控制模式 |
| `Astra_Tracker` | Python | Astra 相机颜色跟踪 — 支持 8 种 OpenCV 跟踪器 |

## 话题接口

| 话题 | 方向 | 类型 | 说明 |
|------|------|------|------|
| `/camera/color/image_raw` | sub | `sensor_msgs::Image` | 彩色图像输入 (BGR8) |
| `/camera/depth/image_raw` | sub | `sensor_msgs::Image` | 深度图像输入 (32FC1) |
| `/tracker/target_position` | pub | `geometry_msgs::Point` | 目标位置 (x,y:像素误差, z:深度[m]) |
| `/tracker/is_tracking` | pub | `std_msgs::Bool` | 跟踪状态 |
| `/mavros/local_position/pose` | sub | `geometry_msgs::PoseStamped` | 无人机当前姿态 (视觉伺服用) |
| `/mavros/setpoint_velocity/cmd_vel_unstamped` | pub | `geometry_msgs::Twist` | 速度指令输出 |

## 控制模式 (KCF_Tracker_control)

| 模式 | 值 | 行为 |
|------|-----|------|
| `MODE_FIXED_POS` | 0 | 原地悬停，仅调整偏航角追踪目标 |
| `MODE_FOLLOW` | 1 | XYZ 移动 + Yaw 旋转，保持期望距离并居中 |
| `MODE_FOLLOW_NO_YAW` | 2 | XYZ 移动跟随，不转偏航角（用 Y 轴速度替代） |

## 依赖

- ROS1 (Noetic)
- OpenCV (≥3.x)
- cv_bridge, image_transport
- MAVROS (mavros_msgs)
- tf2
- Orbbec SDK (KCFTracker.launch) 或 Astra SDK (AstraTracker.launch)

## 快速开始

```bash
# 编译
catkin build uav_tracker

# 启动 KCF 跟踪 + Orbbec 相机
roslaunch uav_tracker KCFTracker.launch

# 启动后，在 RGB 窗口中鼠标框选目标即可开始跟踪
# 键盘控制: r=重置ROI, q=退出

# 启动视觉伺服控制 (另一终端)
rosrun uav_tracker KCFTracker_control_node _mode:=0

# 使用 Astra 相机的颜色跟踪
roslaunch uav_tracker AstraTracker.launch tracker_type:=KCF
```

## 算法参考

- J. F. Henriques et al., "High-Speed Tracking with Kernelized Correlation Filters", TPAMI 2015.
- J. F. Henriques et al., "Exploiting the Circulant Structure of Tracking-by-detection with Kernels", ECCV 2012.
- FHOG 特征提取修改自 OpenCV latentsvm 模块
- Lab 颜色特征参考: Bertinetto et al., "Staple: Complementary Learners for Real-Time Tracking", CVPR 2016.

## 坐标系约定

- 图像坐标：左上角原点，x 向右 [pixel]，y 向下 [pixel]
- 目标位置：`x > 0` 目标在画面右侧，`y > 0` 目标在画面下方
- 视觉伺服补偿：相机安装俯仰角 `camera_mount_pitch` + 当前无人机 pitch → 速度分解

## 参数配置

关键 ROS 参数（通过 launch 文件或 `rosrun _param:=value` 设置）：

| 参数 | 默认值 | 单位 | 说明 |
|------|--------|------|------|
| `limit_max_dist` | 10.0 | [m] | 最大有效深度 |
| `limit_min_dist` | 0.3 | [m] | 最小有效深度 |
| `mode` | 0 | - | 控制模式 (0/1/2) |
| `desired_distance` | 1.0 | [m] | 期望跟随距离 |
| `camera_mount_pitch` | 0.0 | [deg] | 相机安装俯仰角 |

## 迭代记录

2026-07-05 代码注释规范化:
  - 为所有源文件和 launch 文件添加 Doxygen/XML 注释
  - 创建包级 README.md 和 AGENTS.md

## 许可证

KCF 核心算法代码基于 KCF 项目 (BSD 3-Clause)，FHOG 代码基于 OpenCV (BSD 3-Clause)。
