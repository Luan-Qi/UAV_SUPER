# uav_location — UAV 定位与导航规划集成包

UAV 自主导航系统的定位与规划集成 ROS 包。整合了光流里程计、激光 SLAM 全局定位、ICP 配准、坐标融合、以及全局/局部规划启动管理。

## 目录结构

```
uav_location/
├── README.md                               ← 本文件
├── AGENTS.md                               ← Agent 速查手册
├── CMakeLists.txt                           ← 构建配置
├── package.xml                              ← ROS 包清单
├── launch/
│   ├── fast_ofio.launch                     ← 光流-惯导里程计
│   ├── local_mapping.launch                 ← 局部建图 (SLAM)
│   ├── local_planner.launch                 ← 局部规划 (无全局地图)
│   ├── global_planner.launch                ← 全局规划 (实飞模式)
│   ├── global_planner_in_sim.launch         ← 全局规划 (仿真模式)
│   └── icp_registration.launch              ← ICP 全局配准
├── rviz/
│   └── global_planner.rviz                  ← RViz 配置文件
└── src/
    ├── fast_ofio/
    │   ├── AGENTS.md                        ← fast_ofio 子模块速查手册
    │   ├── ofio_types.h                     ← 数据类型 + 配置参数
    │   ├── ofio_estimator.h                 ← 核心估计器接口
    │   ├── ofio_estimator.cpp               ← 核心算法实现
    │   └── fast_ofio_node.cpp               ← ROS 节点入口
    ├── global_localization.cpp              ← 全局 scan-to-map 定位 (FastGICP)
    ├── icp_registration/
    │   └── icp_registration_node.cpp       ← ICP 点面配准节点
    ├── transform_fusion.cpp                 ← 全局/局部坐标融合
    ├── publish_initial_pose.cpp             ← 初始位姿发布
    └── odom_republisher.cpp                 ← 里程计话题重发布
```

## 节点总览

| 节点名 | 源文件 | 功能 |
|--------|--------|------|
| `fast_ofio` | [src/fast_ofio/](src/fast_ofio/) | 光流-惯导里程计 (PX4 EKF2 optflow 参考实现) |
| `fast_lio_localization` | [src/global_localization.cpp](src/global_localization.cpp) | 基于 FastGICP 的全局 scan-to-map 定位 |
| `icp_registration` | [src/icp_registration/icp_registration_node.cpp](src/icp_registration/icp_registration_node.cpp) | 多候选初值 ICP 点面配准 |
| `transform_fusion` | [src/transform_fusion.cpp](src/transform_fusion.cpp) | map→odom 与 odom→base 融合为全局位姿 |
| `publish_initial_pose` | [src/publish_initial_pose.cpp](src/publish_initial_pose.cpp) | 命令行指定初始位姿发布到 /initialpose |
| `odom_republisher` | [src/odom_republisher.cpp](src/odom_republisher.cpp) | 修改 frame_id 后重发布里程计 |

## 启动文件与使用场景

### 1. 光流里程计 — [fast_ofio.launch](launch/fast_ofio.launch)

纯光流-惯导里程计，不依赖激光雷达和全局地图。适用于室内无 GPS 环境的低成本定位。

```bash
roslaunch uav_location fast_ofio.launch scale_factor:=1.05
```

**依赖：** MAVROS (光流+IMU), 板载光流传感器 (如 PMW3901+VL53L1X)

### 2. 局部建图 — [local_mapping.launch](launch/local_mapping.launch)

启动 FAST-LIO 激光 SLAM 在线建图，用于手动飞行采集环境点云。

```bash
# 实飞模式
roslaunch uav_location local_mapping.launch

# 回放 rosbag 建图
roslaunch uav_location local_mapping.launch use_sim_time:=true
```

**依赖：** fast_lio_uwb (mapping_mid360)

### 3. 局部规划 — [local_planner.launch](launch/local_planner.launch)

无全局地图的局部探索/巡航。FAST-LIO 提供里程计，EGO Planner 做局部轨迹规划。

```bash
roslaunch uav_location local_planner.launch
```

**依赖：** fast_lio_uwb, ego_planner, uav_px4_ctrl

### 4. 全局规划 (实飞) — [global_planner.launch](launch/global_planner.launch)

完整自主导航：全局定位 + 全局地图 + A* 路径规划 + EGO Planner 局部规划 + PX4 控制。

```bash
roslaunch uav_location global_planner.launch  \
    pcd_file:=/path/to/map.pcd  \
    use_sim_time:=true  \
    init_x:=0 init_y:=0 init_z:=1.0
```

**依赖：** fast_lio_uwb, uav_planner, ego_planner, uav_px4_ctrl

### 5. 全局规划 (仿真) — [global_planner_in_sim.launch](launch/global_planner_in_sim.launch)

仿真环境下完整全局规划流水线。使用 `fast_lio_localization` + `transform_fusion` 替代 FAST-LIO 全局定位。

```bash
roslaunch uav_location global_planner_in_sim.launch  \
    pcd_file:=/path/to/map.pcd
```

**依赖：** fast_lio_localization, transform_fusion, uav_planner, ego_planner, uav_px4_ctrl

### 6. ICP 全局配准 — [icp_registration.launch](launch/icp_registration.launch)

独立 ICP 点面配准节点，将当前激光扫描与预建 PCD 地图对齐，发布 map→odom TF。

```bash
roslaunch uav_location icp_registration.launch  \
    pcd_path:=/path/to/map.pcd  \
    pointcloud_topic:=/livox/lidar/pointcloud
```

**依赖：** PCL (带法向 ICP), tf2

## 核心话题接口

| 方向 | 话题 | 类型 | 说明 |
|------|------|------|------|
| 发布 | `/Odometry` | `nav_msgs::Odometry` | fast_ofio 光流里程计 |
| 发布 | `/localization` | `nav_msgs::Odometry` | transform_fusion 融合全局位姿 |
| 发布 | `/map_to_odom` | `nav_msgs::Odometry` | 全局定位 map→odom 变换 |
| 发布 | `/initialpose` | `geometry_msgs::PoseWithCovarianceStamped` | 初始位姿 |
| 发布 | `/cur_scan_in_map` | `sensor_msgs::PointCloud2` | 当前扫描在 map 系可視化 |
| 发布 | `/submap` | `sensor_msgs::PointCloud2` | 局部子图可视化 |
| 订阅 | `/mavros/optical_flow_rad/raw/send` | `mavros_msgs::OpticalFlowRad` | 光流传感器 (fast_ofio) |
| 订阅 | `/mavros/imu/data` | `sensor_msgs::Imu` | IMU 数据 (fast_ofio) |
| 订阅 | `/cloud_registered` | `sensor_msgs::PointCloud2` | 激光点云 (全局定位) |
| 订阅 | `/map` | `sensor_msgs::PointCloud2` | 全局地图 (全局定位) |
| 订阅 | `/Odometry` | `nav_msgs::Odometry` | 局部里程计 (transform_fusion) |

## 系统数据流

### 实飞全局规划流程

```
全局地图 PCD ──→ OctoMap ──→ A* 全局路径 ──→ EGO Planner
                                                 │
LiDAR ──→ FAST-LIO 全局定位 ──→ /localization    │
                                      │           │
                                      └───→ 任务管理器 ──→ PX4 控制器 ──→ 飞控
```

### 仿真全局规划流程

```
全局地图 PCD ──→ OctoMap ──→ A* 全局路径 ──→ Path Downsample ──→ EGO Planner
                                                                      │
LiDAR ──→ ICP scan-to-map ──→ /map_to_odom ──→ transform_fusion ────┤
  +                                          (× odom→base)           │
/Odometry ──────────────────────────────────────────→ /localization  │
                                                                      │
                                                           任务管理器 ──→ PX4 Ctrl
```

## 坐标系约定

| 坐标系 | 说明 | 来源 |
|--------|------|------|
| `map` | 全局固定坐标系 (与预建地图对齐) | 全局定位 |
| `odom` | 里程计漂移系 (world-fixed, 可漂移) | FAST-LIO / fast_ofio |
| `camera_init` | SLAM 初始相机坐标系 | FAST-LIO 启动位置 |
| `base_link` | 机体 FRD 坐标系 (X-前 Y-右 Z-下) | MAVROS |
| `lidar_odom` | 激光里程计源系 | FAST-LIO |
| `laser` | LiDAR 传感器安装系 | 传感器标定 |

### fast_ofio 坐标系补充

- **Body FLU (默认)**: X-前 Y-左 Z-上，机体对齐，不随 yaw 旋转 — 适合无外部航向参考的纯光流里程计
- **World ENU**: X-东 Y-北 Z-上 — 适合有外部航向观测量或融合场景

## 依赖

### 系统依赖
- ROS1 (catkin)
- Eigen3
- PCL (点云库)
- tf / tf2

### ROS 包依赖
- `mavros_msgs` — MAVROS 光流消息定义
- `fast_gicp` — 加速 GICP 配准
- `fast_lio_uwb` — FAST-LIO 激光 SLAM
- `ego_planner` — EGO 局部轨迹规划
- `uav_planner` — A* 全局路径规划 + OctoMap
- `uav_px4_ctrl` — PX4 控制与任务管理

## 构建

```bash
# 编译 uav_location 包
catkin build uav_location

# 首次构建或依赖有变化时
catkin build uav_location --force-cmake
```

## 快速验证

```bash
# 1. 光流里程计 (需 MAVROS + 光流传感器)
roslaunch uav_location fast_ofio.launch

# 2. 局部建图 (需 Livox Mid-360)
roslaunch uav_location local_mapping.launch

# 3. 完整全局规划 (实飞, 需预建 PCD 地图)
roslaunch uav_location global_planner.launch pcd_file:=/path/to/map.pcd

# 4. 全局规划 (仿真, 需 rosbag 和 PCD 地图)
roslaunch uav_location global_planner_in_sim.launch pcd_file:=/path/to/map.pcd
```

## 工程约定

- C++11，ROS1 catkin，Eigen3
- 所有 `.cpp` 源文件使用 Doxygen 风格注释 (`@file`, `@brief`, `@details`, `@param`, `@return`)
- 新增节点：在 `CMakeLists.txt` 中添加 `add_executable` + `target_link_libraries`
- 参数通过 ROS param server 加载，launch 文件中以 `<arg>` → `<param>` 传递
- 坐标系遵循 REP-105 (map → odom → base_link)
- 节点日志前缀统一使用 `[模块名]` 格式 (如 `[fast_ofio]`, `[global]`, `[odom_republisher]`)

## 许可

TODO
