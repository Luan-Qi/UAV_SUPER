# uav_px4_ctrl_test — PX4 Offboard 控制测试包

PX4 无人机 Offboard 模式控制算法的测试与验证包。提供从简单起飞到综合 RC 状态机控制的多种测试节点，用于在仿真或实飞环境中验证 PX4 控制接口的正确性。

## 目录

- [包概览](#包概览)
- [节点列表](#节点列表)
- [快速开始](#快速开始)
- [启动文件](#启动文件)
- [坐标系约定](#坐标系约定)
- [依赖](#依赖)
- [构建](#构建)
- [文件结构](#文件结构)
- [相关包](#相关包)

## 包概览

```
uav_px4_ctrl_test/
├── CMakeLists.txt              # Catkin 构建配置 (6 个可执行文件)
├── package.xml                 # ROS 包清单
├── README.md                   # 本文件
├── CLAUDE.md                   # AI Agent 快速指南
├── launch/
│   ├── px4_vel_simple_test.launch    # PX4 综合控制测试 (fast_lio + px4ctrl)
│   └── uav_super_px4_livox.launch   # PX4 + Livox MID360 传感器联合启动
└── src/
    ├── odometry_to_px4.cpp         # 里程计→PX4 视觉位姿桥接
    ├── px4_pos_takeoff.cpp         # 位置控制 — 定高起飞
    ├── px4_pos_rectangle.cpp       # 位置控制 — 矩形航线
    ├── px4_pos_up-and-down.cpp     # 位置控制 — 往复升降
    ├── px4_vel_rectangle.cpp       # 速度控制 — PID 矩形航线
    └── px4_vel_simple_test.cpp     # 综合控制 RC 状态机 + 双模式
```

## 节点列表

| 节点 | 可执行文件 | 控制模式 | 复杂度 | 说明 |
|------|-----------|---------|--------|------|
| Odometry→PX4 桥接 | `odometry_to_px4_node` | — | ★ | 订阅 /Odometry，坐标修正后发布到 /mavros/vision_pose/pose |
| 定高起飞 | `px4_takeoff` | 位置 | ★ | 从地面起飞到 1m 高度并悬停 |
| 矩形航线 (位置) | `px4_rectangle` | 位置 | ★★ | 飞越四个矩形顶点，每点悬停 3 秒，循环执行 |
| 往复升降 | `px4_up-and-down` | 位置 | ★ | 在 0.5m ↔ 1.0m 高度间每 5 秒切换 |
| 矩形航线 (速度) | `px4_vel_rectangle` | 速度 | ★★★ | PID 位置→速度控制器，6 阶段状态机，自动降落 |
| 综合控制 | `px4_vel_simple_test` | 位置/速度 | ★★★★ | RC 状态机 + 位置悬停 + 6 阶段速度指令 + 电池安全 + 坐标桥接 |

### 能力矩阵

| 功能 | takeoff | rectangle | up-and-down | vel_rectangle | vel_simple_test |
|------|:---:|:---:|:---:|:---:|:---:|
| Offboard 模式管理 | ✓ | ✓ | ✓ | ✓ | ✓ |
| 自动解锁 | ✓ | ✓ | ✓ | ✓ | ✓ |
| ENU→NED 坐标修正 | ✓ | ✓ | ✓ | — | ✓ |
| 位置反馈控制 | — | — | — | ✓ | — |
| PID 控制器 | — | — | — | ✓ | — |
| RC 遥控器状态机 | — | — | — | — | ✓ |
| 电池低压检测 | — | — | — | — | ✓ |
| 位置安全限幅 | — | — | — | — | ✓ |
| 速度安全限幅 | — | — | — | — | ✓ |
| 视觉里程计桥接 | — | — | — | — | ✓ |
| 自动降落 | — | — | — | ✓ | — |
| 多模式切换 | — | — | — | — | ✓ |

## 快速开始

### 1. 最简单的起飞测试

```bash
# 终端 1：启动 PX4 + Livox 传感器驱动
roslaunch uav_px4_ctrl_test uav_super_px4_livox.launch

# 终端 2：启动激光里程计
roslaunch fast_lio mapping_mid360.launch

# 终端 3：启动里程计桥接 + 定高起飞
rosrun uav_px4_ctrl_test odometry_to_px4_node &
rosrun uav_px4_ctrl_test px4_takeoff
```

### 2. 矩形航线 (位置控制)

```bash
rosrun uav_px4_ctrl_test px4_rectangle
```

航线：从原点起飞 → (1,0,1) → (1,1,1) → (0,1,1) → 循环

### 3. 综合 RC 遥控器控制

```bash
roslaunch uav_px4_ctrl_test px4_vel_simple_test.launch
```

遥控器操作：
- **CH5 > 1750**: 解锁 (Arm)
- **CH6 > 1750**: 进入 Offboard 模式
- **CH7 > 1750**: 指令模式 (自动执行 6 阶段速度序列)
- **CH7 1250~1750**: 悬停模式 (自动起飞到 0.5m)
- **CH7 < 1250**: 空闲 (降落)

## 启动文件

### px4_vel_simple_test.launch

启动 Fast-LIO 激光里程计 + PX4 综合控制节点。

```
roslaunch uav_px4_ctrl_test px4_vel_simple_test.launch
```

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `rviz` (fast_lio) | `false` | 是否启动 RViz 可视化 |

### uav_super_px4_livox.launch

启动 MAVROS + Livox MID360 激光雷达驱动。

```
roslaunch uav_px4_ctrl_test uav_super_px4_livox.launch
```

| 可选参数 | 说明 |
|---------|------|
| MAVROS `fcu_url` | 串口设备路径 (默认自动检测) |
| Livox `xfer_format` | 点云格式 (0=PointCloud2) |

## 坐标系约定

本项目涉及两种坐标系的变换：

| 坐标系 | 轴定义 | 来源 |
|--------|--------|------|
| ENU (输入) | X=东 Y=北 Z=上 | Fast-LIO / 里程计 |
| NED (输出) | X=前 Y=右 Z=下 | PX4 本地坐标系 |

**坐标修正函数 `uav_mavros_pose_fix()`**：
```
位置:  x' = -y,  y' = x     (交换并取反)
姿态:  绕 Z 轴旋转 +90° (π/2)
```

> **注意**：[px4_vel_rectangle.cpp](src/px4_vel_rectangle.cpp) 不经过坐标修正，
> 直接使用 PX4 原生 NED 坐标，因此其航点定义在 NED 系下。

## 依赖

### ROS 包
- `roscpp` — ROS C++ 客户端库
- `std_msgs` — 标准消息类型
- `geometry_msgs` — 位姿/速度/点消息
- `nav_msgs` — 里程计消息
- `mavros_msgs` — MAVROS 飞控消息与服务
- `quadrotor_msgs` — 四旋翼轨迹指令消息
- `tf2` + `tf2_geometry_msgs` — 坐标变换

### 外部依赖
- `fast_lio` — 激光里程计 (Livox MID360)
- `livox_ros_driver2` — Livox 激光雷达驱动
- `mavros` — PX4 MAVLink 通信桥
- `uav_px4_ctrl` — subscribe.h/cpp 数据类型定义

### 硬件
- Pixhawk / Cube 系列飞控 (运行 PX4)
- Livox MID360 激光雷达
- OrangePi / Jetson / x86 机载计算机
- Futaba 遥控器 (7 通道，日本手)

## 构建

```bash
# 在 catkin workspace 根目录
catkin build uav_px4_ctrl_test

# 或单独编译
catkin_make --pkg uav_px4_ctrl_test
```

## 文件结构

```
uav_px4_ctrl_test/
├── CMakeLists.txt                      # 6 个可执行目标
├── package.xml                         # 包清单 (format=2)
├── README.md                           # 用户文档
├── CLAUDE.md                           # AI Agent 开发指南
├── launch/
│   ├── px4_vel_simple_test.launch      # 综合控制启动
│   └── uav_super_px4_livox.launch      # 传感器驱动启动
└── src/
    ├── odometry_to_px4.cpp             # 里程计桥接 (★)
    ├── px4_pos_takeoff.cpp             # 定高起飞 (★)
    ├── px4_pos_rectangle.cpp           # 矩形航线-位置 (★★)
    ├── px4_pos_up-and-down.cpp         # 往复升降 (★)
    ├── px4_vel_rectangle.cpp           # 矩形航线-速度 (★★★)
    └── px4_vel_simple_test.cpp         # 综合控制 (★★★★)
```

## 相关包

| 包 | 关系 | 接口 |
|----|------|------|
| [uav_px4_ctrl](../uav_px4_ctrl/) | 数据类型提供 | subscribe.h/cpp 共享使用 |
| [uav_location](../uav_location/) | 里程计输入 | /Odometry (Fast-LIO / OFIO) |
| [uav_planner](../uav_planner/) | 轨迹指令源 | /position_cmd → 速度指令 |
| [fast_lio](../fast_lio/) | 激光里程计 | /Odometry 发布 |
| [livox_ros_driver2](../livox_ros_driver2/) | 激光雷达驱动 | /livox/lidar PointCloud2 |

## 测试进度

| 节点 | 仿真测试 | 实飞测试 | 备注 |
|------|:---:|:---:|------|
| odometry_to_px4 | ✓ | ✓ | 基本功能正常 |
| px4_takeoff | ✓ | ✓ | 稳定起飞悬停 |
| px4_rectangle | ✓ | — | 待实飞验证 |
| px4_up-and-down | ✓ | — | 待实飞验证 |
| px4_vel_rectangle | ✓ | — | PID 参数需调优 |
| px4_vel_simple_test | ✓ | ✓ | 综合功能验证通过 |
