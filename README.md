# UAV-Super

UAV autonomous driving system — LiDAR-inertial localization, global/local planning, and PX4 control in a unified ROS workspace.

---

## 系统架构

```
┌──────────────────────────────────────────────────────────────┐
  感知层
  livox_ros_driver2 (MID360)  ·  orbbec_camera (Gemini 330)
  nlink_parser (UWB)
└──────────────────────────┬───────────────────────────────────┘
                           │
                           ▼
┌──────────────────────────────────────────────────────────────┐
  定位层
  FAST-LIO 前端 (LiDAR-IMU 里程计)
  global_localization (全局 ikd-tree GN 匹配)
  UWB 测距融合 (实验性, 沿道约束)
  输出: /Odometry, /localization, TF map -> body
└──────────────────────────┬───────────────────────────────────┘
                           │
                           ▼
┌──────────────────────────────────────────────────────────────┐
  规划层
  EGOX-Planner (B 样条局部轨迹 + 自主探索)
  A* 全局路径 (uav_planner)
  路径简化 / 平滑
└──────────────────────────┬───────────────────────────────────┘
                           │
                           ▼
┌──────────────────────────────────────────────────────────────┐
  控制层
  px4ctrl (SO3 几何控制器)
  mission_manger (航点序列任务管理)
  tracking_controller (起降、轨迹跟踪, 未编译)
└──────────────────────────┬───────────────────────────────────┘
                           │
                           ▼
                       PX4 飞控
```

---

## 快速开始

### 1. 编译

```bash
mkdir -p UAV_SUPER/src
git clone --recursive https://github.com/Luan-Qi/UAV_SUPER.git src/

catkin_make -j
source devel/setup.bash
```

### 2. 硬件驱动

```bash
bash shfile/uav_super_interfance.sh
# 等价于:
#   roslaunch mavros px4.launch          # PX4 飞控连接
#   roslaunch orbbec_camera gemini_330_series.launch  # 双目相机
#   roslaunch livox_ros_driver2 msg_MID360.launch     # MID360 雷达
```

### 3. 建图模式

**手持**无人机或**手动遥控**飞行采集点云，通过 FAST-LIO SLAM 实时建图。

点云文件默认位于`src/FAST_LIO_UWB/PCD/scan.pcd`。**及时备份 PCD 文件以免覆盖**。

```bash
bash shfile/uav_super_mapping.sh
# 等价于 roslaunch uav_location local_mapping.launch
```

自动生成的PCD点云一般比较大，建议使用`uav_util/pcd_downsample_node`节点进行降采样，减少后续节点运行压力。**经验显示 `0.05-0.10` 叶子尺寸的降采样就够了**

```bash
roslaunch uav_util pcd_downsample.launch pcd_path:=/path/to/scan.pcd leaf_size:=0.05
```

### 4. 自动飞行模式（已知全局地图）

需要提前准备好 `map.pcd` 并在 launch 文件中设置航点序列、停留时间等参数。

推荐使用[Mission-Generator](https://github.com/Luan-Qi/Mission-Generator)进行点云的查看和航点的配置，软件可以自动生成配置参数，放到launch文件中对应位置即可，具体请参考 Mission-Generator 的使用文档。

```bash
bash shfile/uav_super_main.sh
# 等价于 roslaunch uav_location global_planner.launch pcd_file:=/path/to/map.pcd
```

系统流程:
1. 无人机完成全局重定位（若启动时无法完成，手动晃动无人机）
2. 切换到 Offboard 模式，自动飞行开关拨到高位
3. RC 主动解锁后，无人机自动执行航点序列
4. 无人机自动起飞并开始巡航，到达终点后手动关闭自动飞行开关，自动降落
5. **发生意外须立即手动接管**

### 5. 仿真模式

用于验证航点和规划算法可行性，不涉及定位和PX4控制部分。

```bash
roslaunch uav_location global_planner_in_sim.launch
```

### 6. 局部探索模式（未完成，请勿使用）

无人机使用局部定位在未知地图上进行探索，自动生成全局地图。

```bash
bash shfile/uav_super_explorer.sh
# 未完成，请勿使用
```

---

## 模式对比

| 模式 | 启动文件 | 定位方式 | 路径规划 | 适用场景 |
|------|---------|---------|---------|---------|
| 建图 | `local_mapping.launch` | FAST-LIO 里程计 | 无 (手动飞行) | 采集环境点云 |
| 自动飞行 | `global_planner.launch` | 全局定位 (ikd-tree GN) | A* + EGO Planner | 已知地图的自主任务 |
| 局部规划 | `local_planner.launch` | FAST-LIO 里程计 | EGO Planner | 无地图的探索/巡航 |
| 仿真 | `global_planner_in_sim.launch` | 仿真真值 | A* + EGO Planner | 算法测试 |

---

## 系统测试脚本[UAV_SUPER_shfile](https://github.com/Luan-Qi/UAV_SUPER_shfile)

| 脚本 | 用途 |
|------|------|
| `shfile/uav_super_interfance.sh` | 启动所有硬件驱动 (MAVROS + 相机 + LiDAR) |
| `shfile/uav_super_main.sh` | 自动飞行模式 (全局定位 + 规划 + 控制) |
| `shfile/uav_super_mapping.sh` | 建图模式 (FAST-LIO SLAM + PCD 保存) |
| `shfile/uav_super_explorer.sh` | 局部探索模式 (无全局地图) |
| `shfile/uav_super_test_fastliouwb.sh` | FAST-LIO + UWB 全局纯定位测试 |
| `shfile/uav_super_test_local.sh` | 局部定位测试 (local_planner) |
| `shfile/uav_super_test_position.sh` | 位置控制测试 (FAST-LIO + px4ctrl) |
| `shfile/test_path.sh` | 路径规划测试 (发布测试 Path) |
| `shfile/test_takeoff.sh` | 起飞服务调用测试 |

---

## 从零开始测试清单

首次部署或久置后复飞时，按以下顺序逐项验证，确保每一步通过后再进入下一步。

| 步骤 | 脚本 / 命令 | 验证目标 | 通过标准 |
|:---:|---|---|---|
| **1** | `bash shfile/uav_super_interfance.sh` | 传感器与飞控连接 | MAVROS 显示飞控心跳正常；`rostopic hz /livox/imu` 有稳定输出；相机图像正常 |
| **2** | `bash shfile/uav_super_test_position.sh` | 定位建图与 PX4 位置控制 | FAST-LIO 里程计正常输出 `/Odometry`；RViz 中点云与 odom 轨迹一致；RC 切 POSITION 模式后无人机可稳定悬停并响应摇杆移动 |
| **3** | `bash shfile/uav_super_test_local.sh` | 局部避障 | EGO Planner 可在简单障碍物环境中生成避障轨迹；无人机按轨迹飞行不碰撞 |
| **4** | `bash shfile/uav_super_mapping.sh` | 全局地图构建 | 手持/手动飞行采集完整环境点云；`roslaunch uav_util pcd_downsample.launch` 降采样（推荐 leaf_size 0.05–0.10）；生成 `.pcd` 文件备用 |
| **5** | Mission-Generator | 航点规划 | 在点云地图上配置航点序列、停留时间、录像控制等参数；导出 launch 配置 |
| **6** | `roslaunch uav_location global_planner_in_sim.launch` | 航线可行性 | 仿真环境中路径规划成功；A* 全局路径 + EGO 局部轨迹均无碰撞；航点序列可全部到达 |
| **7** | `bash shfile/uav_super_main.sh` | 正式自主飞行 | 全局重定位成功；Offboard 解锁后自动起飞并按航点巡航；全程无异常；**随时准备手动接管** |

> **原则**: 每步失败都应排查并修复后再继续，不要跳过中间步骤直接飞行。

---

## 目录结构

```
UAV-Super/
├── build/                         # ROS 编译目录
├── devel/                         # ROS 编译依赖库
├── shfile/                        # Shell 启动脚本
└── src/                           # ROS 源代码目录
    ├── EGOX_Planner/              # EGO-Swarm 局部规划器（含自主探索）
    │   ├── planner/               #   规划器核心 (plan_manage, plan_env, bspline_opt 等)
    │   └── uav_simulator/         #   UAV 仿真环境 (mockamap, fake_drone, local_sensing)
    ├── FAST_LIO_UWB/              # FAST-LIO2 + 全局定位后端（核心定位）
    │   ├── src/                   #   laserMapping (前端), global_localization (后端)
    │   ├── include/               #   ikd-Tree, IKFoM, UWB/GPS 接口
    │   ├── launch/                #   建图/定位启动文件 + PARAMETERS.md
    │   ├── config/                #   mid360, mid360_extIMU, avia 参数
    │   ├── maps/                  #   先验地图存放
    │   └── rviz_cfg/              #   RViz 可视化配置
    ├── fast_gicp/                 # 快速 GICP 点云配准库 (VGICP / NDT CUDA)
    ├── quadrotor_msgs/            # 四旋翼消息定义 (21 个 .msg)
    ├── tracking_controller/       # 轨迹跟踪与起降控制 (CATKIN_IGNORE, 未编译)
    │   ├── launch/                #   起降/悬停/跟踪圆/轨迹跟踪 + VINS→MAVROS
    │   └── src/                   #   px4 控制节点, px4_pos_estimator
    ├── uav_localization/          # Cartographer 定位 (备用方案)
    │   ├── launch/                #   localization_mid360, mapping_mid360
    │   ├── script/                #   Python 辅助脚本
    │   └── urdf/                  #   四旋翼 URDF 模型
    ├── uav_location/              # 全局定位 + 规划集成启动
    │   ├── launch/
    │   │   ├── global_planner.launch       # 自动飞行 (全局地图 + A* + EGO)
    │   │   ├── global_planner_in_sim.launch# 仿真自动飞行
    │   │   ├── local_planner.launch        # 局部规划 (无全局地图)
    │   │   ├── local_mapping.launch        # 建图 (FAST-LIO SLAM)
    │   │   ├── fast_ofio.launch            # FAST-LIO 光流里程计
    │   │   └── icp_registration.launch     # ICP 重定位测试
    │   └── src/
    │       ├── global_localization.cpp     # 全局定位算法
    │       ├── transform_fusion.cpp        # 坐标融合发布
    │       ├── publish_initial_pose.cpp    # 初始位姿估计
    │       ├── odom_republisher.cpp        # 里程计转发
    │       ├── fast_ofio/                  # 光流惯性里程计
    │       └── icp_registration/           # ICP 重定位测试
    ├── uav_planner/               # 路径规划算法库
    │   ├── launch/
    │   │   ├── astar_planner.launch        # A* 全局路径搜索
    │   │   ├── planner_path_simplify.launch# 路径简化
    │   │   ├── planner_path_smoothify.launch# B 样条路径平滑
    │   │   └── octomap_publisher.launch    # PCD → OctoMap 转换
    │   └── src/
    │       ├── slp_planner/                # 单线激光规划器 (自研)
    │       ├── astar/                      # A* 搜索算法
    │       ├── path_simplify/              # 路径简化
    │       ├── path_smoothify/             # 路径平滑
    │       ├── simple_path_follower/       # 简易轨迹跟踪
    │       ├── fast_path_follower/         # 快速路径跟随器
    │       ├── fforward_path_follower/     # 前馈路径跟随器
    │       ├── visual_path_follower/       # 路径跟随可视化
    │       ├── transform_path.cpp          # 路径坐标变换
    │       └── pca_direction_node.cpp      # PCA 主方向分析 (SLP)
    ├── uav_px4_ctrl/              # PX4 控制算法库
    │   ├── launch/
    │   │   ├── px4ctrl.launch              # SO3 几何控制器
    │   │   ├── mission_manger.launch       # 多航点任务管理
    │   │   ├── goal_publisher.launch       # 目标点发布测试
    │   │   ├── uav_super_localization.launch   # 完整自主飞行 (定位+规划+控制)
    │   │   └── uav_super_mapping.launch        # 完整建图模式
    │   ├── script/
    │   │   └── convert_tf_to_odom.py       # Cartographer TF → Odom 转换
    │   ├── src/
    │   │   ├── px4ctrl_node.cpp            # 主控制器
    │   │   ├── px4ctrl/                    # SO3 控制算法
    │   │   ├── mission_manger.cpp          # 任务管理
    │   │   ├── pub_goal.cpp                # 目标发布
    │   │   └── pub_waypoint.cpp            # 航点发布
    │   └── srv/                            # 自定义服务消息
    ├── uav_px4_ctrl_test/         # PX4 控制测试
    │   ├── launch/
    │   │   ├── uav_super_px4_livox.launch  # MAVROS + LiDAR 联合启动
    │   │   └── px4_vel_simple_test.launch  # RC 状态机综合控制测试
    │   └── src/
    │       ├── px4_pos_takeoff.cpp         # 位置控制起飞 (1m)
    │       ├── px4_pos_rectangle.cpp       # 位置控制矩形航线
    │       ├── px4_pos_up-and-down.cpp     # 位置控制往复升降
    │       ├── px4_vel_rectangle.cpp       # 速度控制矩形航线
    │       ├── px4_vel_simple_test.cpp     # RC 状态机 + 双模式控制
    │       └── odometry_to_px4.cpp         # 里程计→PX4 视觉位姿桥接
    ├── uav_tracker/               # 视觉跟踪算法库
    │   ├── launch/                        # AstraTracker (颜色) / KCFTracker
    │   └── src/                            # KCF 跟踪算法
    └── uav_util/                  # 工具库
        ├── launch/
        │   ├── generate_pcd_map.launch     # 程序化 PCD 地图生成 (cube/sphere/corridor)
        │   ├── pcd_downsample.launch       # PCD 降采样
        │   ├── merge_cloud.launch          # 多帧点云合并
        │   ├── simple_camera.launch        # OpenCV 简易相机驱动
        │   ├── imu_health_check.launch     # IMU 健康检测 (静止校准+晃动评估)
        │   └── imu_to_odom.launch          # IMU 积分里程计 (ZUPT 零速修正)
        └── src/
            ├── generate_pcd_map.cpp
            ├── pcd_downsample_node.cpp
            ├── merge_cloud_node.cpp
            ├── simple_camera_driver.cpp
            └── imu_to_odom/                # IMU 里程计 (ZUPT)
```

---

## 核心模块说明

### 定位: FAST_LIO_UWB

项目默认定位方案，基于 FAST-LIO2 + 全局 ikd-tree GN 匹配:

- **前端**: FAST-LIO2 LiDAR-IMU 紧耦合里程计 (IESKF + ikd-tree)
- **后端**: 全局 ikd-tree 子图 + OpenMP 并行 GN 优化 (scan-to-map)
- **UWB 融合** (实验性): 单锚点 1D 测距约束，针对矿道等直线退化场景
- 详见 [FAST_LIO_UWB/README.md](FAST_LIO_UWB/README.md)

备用方案: `uav_localization` 提供 Cartographer 3D SLAM 定位，保留兼容性。
旧版代码：`uav_location` 留作纪念。

### 规划: EGOX-Planner

基于梯度的 B 样条局部轨迹规划器，编译 5 个子包 (plan_manage, plan_env, path_searching, bspline_opt, traj_utils):

- 实时避障 (ESDF-free 梯度优化)
- 自主探索 (基于 EPIC，开发中)
- 仿真与实机部署
- 详见 [EGOX_Planner/README.md](EGOX_Planner/README.md)

### 全局路径: uav_planner

- **A\* 搜索**: 基于 OctoMap 的全局最短路径 (3 种模式: 标准/连续/里程计)
- **SLP Planner**: 单线激光雷达规划器 (自研), PCA 主方向分析
- **路径后处理**: 简化 (Douglas-Peucker + greedy) + 平滑 (Bezier + 离散曲线优化)
- **路径跟随**: fast / fforward / visual 三种跟随器

### 控制: uav_px4_ctrl + tracking_controller

- **SO3 几何控制器**: 四旋翼姿态+推力非线性控制，3 种控制模式 (position/velocity/attitude)
- **mission_manger**: 航点序列管理，支持到达判定、悬停等待、录像控制、planner 失败重试
- **tracking_controller** (CATKIN_IGNORE): VINS 视觉目标跟踪 (KCF + B 样条轨迹)，起飞/悬停/圆形跟踪

### 配准: fast_gicp

高性能点云配准库，支持 FastGICP / FastVGICP / FastVGICPCuda / NDTCuda 多种实现，用于 ICP 重定位等场景。

### 工具: uav_util

- **程序化地图生成**: 支持 cube / sphere / cylinder / corridor / cylinder_corridor 形状
- **IMU 健康检测**: 静止校准 + 晃动评估，检测 IMU 偏置/噪声/饱和
- **IMU 里程计**: 纯 IMU 积分 + ZUPT 零速修正，含重力补偿和陀螺仪偏置校准
- **多帧点云合并**: message_filters 时间同步双雷达点云融合

---

## 依赖

| 依赖 | 用途 |
|------|------|
| ROS Melodic/Noetic | 框架 |
| PCL ≥ 1.8 | 点云处理 |
| Eigen3 | 线性代数 |
| OpenMP | 并行加速 |
| livox_ros_driver2 | MID360 LiDAR 驱动 |
| MAVROS | PX4 飞控通信 |
| nlink_parser | UWB 测距 (可选) |
| OctoMap | 三维占据栅格地图 |
| OpenCV | 相机驱动 / 视觉跟踪 |
| CUDA (可选) | fast_gicp GPU 加速 |

---

## 组件修改说明

1. **livox_ros_driver2**: IMU 数据处理时考虑 LiDAR 外参
2. **EGOX-Planner**: 解锁 EGO Planner z 轴限制 (原限制 z=1.0m)
3. **livox_ros_driver2**: 修复 driver 输出数据类型 (`output_type` 参数)
4. **FAST_LIO_UWB**: 支持通用 PointCloud2 输入 (`lidar_type: 4`)，但默认使用 CustomMsg 原生路径 (`lidar_type: 1`) + driver 双消息发布

---

## 引用

- [FAST-LIO2](https://github.com/hku-mars/FAST_LIO)
- [EGO-Planner](https://github.com/ZJU-FAST-Lab/EGO-Planner)
- [EPIC](https://github.com/Robotics-STAR-Lab/EPIC)
- [fast_gicp](https://github.com/SMRT-AIST/fast_gicp)
