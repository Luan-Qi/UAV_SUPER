# AGENTS.md — uav_location

> 给后续 agent 的速查手册。**短、准、可迭代**。每犯一次错就更新一次。

## 目录结构

```
uav_location/
├── AGENTS.md                          ← 本文件
├── README.md                          ← 用户文档
├── CMakeLists.txt                     ← 构建配置 (fast_ofio target)
├── package.xml                        ← ROS 包清单 (依赖 mavros_msgs, fast_gicp)
├── launch/
│   ├── fast_ofio.launch               ← 光流-惯导里程计
│   ├── local_mapping.launch           ← 局部 SLAM 建图 (FAST-LIO)
│   ├── local_planner.launch           ← 局部规划 (FAST-LIO + EGO + PX4)
│   ├── global_planner.launch          ← 全局规划实飞 (FAST-LIO 定位 + A* + EGO + PX4)
│   ├── global_planner_in_sim.launch   ← 全局规划仿真 (ICP 定位 + A* + EGO + PX4)
│   └── icp_registration.launch       ← 独立 ICP 点面配准
├── rviz/
│   └── global_planner.rviz            ← RViz 全局规划配置
└── src/
    ├── fast_ofio/
    │   ├── AGENTS.md                  ← fast_ofio 子模块速查 (必读)
    │   ├── ofio_types.h               ← OfioConfig + 数据结构
    │   ├── ofio_estimator.h           ← OfioEstimator 接口
    │   ├── ofio_estimator.cpp         ← 核心算法 (PX4 EKF2 参考)
    │   └── fast_ofio_node.cpp         ← ROS 节点入口
    ├── global_localization.cpp        ← FastGICP scan-to-map 全局定位
    ├── icp_registration/
    │   └── icp_registration_node.cpp ← 多候选初值 ICP 点面配准
    ├── transform_fusion.cpp           ← 全局-局部坐标融合 (map→odom × odom→base)
    ├── publish_initial_pose.cpp       ← 命令行初始位姿发布
    └── odom_republisher.cpp           ← 里程计 frame_id 重发布
```

## 构建与运行

```bash
# 编译
catkin build uav_location

# 各场景运行
## 光流里程计
roslaunch uav_location fast_ofio.launch scale_factor:=1.05

## 局部建图
roslaunch uav_location local_mapping.launch

## 局部规划 (无全局地图)
roslaunch uav_location local_planner.launch

## 全局规划 (实飞, 需 PCD 地图)
roslaunch uav_location global_planner.launch pcd_file:=/path/to/map.pcd

## 全局规划 (仿真, 需 rosbag + PCD 地图)
roslaunch uav_location global_planner_in_sim.launch pcd_file:=/path/to/map.pcd

## 独立 ICP 配准
roslaunch uav_location icp_registration.launch pcd_path:=/path/to/map.pcd
```

## 节点全景

| 节点名 | 源文件 | 功能关键词 |
|--------|--------|-----------|
| `fast_ofio` | [src/fast_ofio/](src/fast_ofio/) | 光流里程计, PX4 EKF2, MAVROS |
| `fast_lio_localization` | [src/global_localization.cpp](src/global_localization.cpp) | 全局定位, FastGICP, scan-to-map, 多尺度 |
| `icp_registration` | [src/icp_registration/icp_registration_node.cpp](src/icp_registration/icp_registration_node.cpp) | ICP 点面配准, 多候选初值, 粗→精 |
| `transform_fusion` | [src/transform_fusion.cpp](src/transform_fusion.cpp) | 坐标融合, map×odom→global pose |
| `publish_initial_pose` | [src/publish_initial_pose.cpp](src/publish_initial_pose.cpp) | 初始位姿, /initialpose |
| `odom_republisher` | [src/odom_republisher.cpp](src/odom_republisher.cpp) | frame_id 改写, 话题转发 |

## 三大运行场景的数据流

### 场景 A: fast_ofio 光流里程计 (独立)

```
光流传感器 ──→ /mavros/optical_flow_rad/raw/send ──→ fast_ofio ──→ /Odometry
IMU ────────→ /mavros/imu/data ──────────────────────→
```

**关键文件：** [ofio_estimator.cpp](src/fast_ofio/ofio_estimator.cpp) 第 125 行 `updateOpticalFlow()` 是算法主入口。
**配置入口：** [ofio_types.h](src/fast_ofio/ofio_types.h) `OfioConfig::loadFromParamServer()`

### 场景 B: 全局规划 (实飞)

```
PCD ──→ OctoMap ──→ A* ──→ EGO Planner (以 FAST-LIO /localization 为 odom)
LiDAR ──→ FAST-LIO locating ──→ /Odometry ──→ /localization (全局位姿)
```

**关键文件：** [global_planner.launch](launch/global_planner.launch)
**全局定位由** `fast_lio_uwb/locating_mid360.launch` **提供，非本包节点。**

### 场景 C: 全局规划 (仿真)

```
PCD ──→ OctoMap ──→ A* ──→ Path Downsample ──→ EGO Planner
LiDAR仿真 ──→ fast_lio_localization (GICP) ──→ /map_to_odom ──┐
/Odometry(仿真) ─────────────────────────────────────────→ transform_fusion ──→ /localization
```

**关键文件：** [global_planner_in_sim.launch](launch/global_planner_in_sim.launch)
**全局定位链：** `fast_lio_localization` → `/map_to_odom` → `transform_fusion` → `/localization`

## 话题接口速查

| 方向 | 话题 | 类型 | 生产者/消费者 |
|------|------|------|-------------|
| 订阅 | `/mavros/optical_flow_rad/raw/send` | `mavros_msgs::OpticalFlowRad` | fast_ofio ← MAVROS |
| 订阅 | `/mavros/imu/data` | `sensor_msgs::Imu` | fast_ofio ← MAVROS |
| 发布 | `/Odometry` | `nav_msgs::Odometry` | fast_ofio → 下游 |
| 订阅 | `/Odometry` | `nav_msgs::Odometry` | transform_fusion ← FAST-LIO |
| 订阅 | `/cloud_registered` | `sensor_msgs::PointCloud2` | fast_lio_localization ← 激光里程计 |
| 订阅 | `/map` | `sensor_msgs::PointCloud2` | fast_lio_localization ← 全局地图 |
| 发布 | `/map_to_odom` | `nav_msgs::Odometry` | fast_lio_localization → transform_fusion |
| 发布 | `/localization` | `nav_msgs::Odometry` | transform_fusion → Mission/PX4 |
| 发布 | `/initialpose` | `geometry_msgs::PoseWithCovarianceStamped` | publish_initial_pose → 定位 |
| 订阅 | `/initialpose` | `geometry_msgs::PoseWithCovarianceStamped` | fast_lio_localization ← 用户/脚本 |

## 工程约定

- C++11，ROS1 catkin，Eigen3，PCL，tf/tf2
- 节点名与二进制名一致
- 所有 `.cpp` 和 `.h` 使用 Doxygen 风格注释：`@file`, `@brief`, `@details`, `@param`, `@return`
- Section 分隔使用 `// ====` 风格 (与 fast_ofio 一致)
- ROS param 通过 launch 文件 `<arg>` → `<param>` 两层传递
- 扩展参数的方法：
  1. 在源码中添加 `nh.param(...)` 加载和全局变量
  2. 在对应的 launch 文件中添加 `<arg name="..." default="..."/>`
  3. 在 `<node>` 标签内添加 `<param name="..." value="$(arg ...)"/>`
  4. 在 README.md 的话题表格或参数说明中补充
- 新节点必须在 `CMakeLists.txt` 中添加 `add_executable` + `target_link_libraries` + `add_dependencies`

## 坐标系约定

| Frame | 英文含义 | 方向 | 来源 |
|-------|----------|------|------|
| `map` | 全局固定坐标系 | 与预建地图对齐 | 全局定位发布 map→odom |
| `odom` | 里程计漂移系 | world-fixed, 可漂移 | FAST-LIO / fast_ofio |
| `camera_init` | SLAM 初始位姿系 | = SLAM 启动时的 base_link | FAST-LIO |
| `base_link` | 机体 FRD | X-前 Y-右 Z-下 | MAVROS |
| `lidar_odom` | 激光里程计源 | = FAST-LIO 里程计系 | ICP 配准的 range_odom |
| `laser` | LiDAR 安装系 | 传感器标定 | TF 静态发布 |

**fast_ofio 特殊规则：**
- `use_body_frame_odom=true` (默认): Body FLU, 机体对齐，不随 yaw 旋转
- `use_body_frame_odom=false`: World ENU, 随 IMU 姿态旋转

## 关键索引位置

| 你要找什么 | 去哪儿看 |
|-----------|---------|
| fast_ofio 算法流水线 | [ofio_estimator.cpp:125](src/fast_ofio/ofio_estimator.cpp#L125) `updateOpticalFlow()` |
| fast_ofio 全部配置参数 | [ofio_types.h:122](src/fast_ofio/ofio_types.h#L122) `OfioConfig` |
| fast_ofio 子模块细节 | [src/fast_ofio/AGENTS.md](src/fast_ofio/AGENTS.md) |
| 全局定位核心算法 | [global_localization.cpp:249](src/global_localization.cpp#L249) `globalLocalization()` |
| ICP 多候选搜索 | [icp_registration_node.cpp:185](src/icp_registration/icp_registration_node.cpp#L185) `multiAlignSync()` |
| 坐标融合公式 | [transform_fusion.cpp:89](src/transform_fusion.cpp#L89) `transformFusion()` |
| 全局规划 (实飞) launch | [global_planner.launch](launch/global_planner.launch) |
| 全局规划 (仿真) launch | [global_planner_in_sim.launch](launch/global_planner_in_sim.launch) |
| CMake 构建定义 | [CMakeLists.txt:46](CMakeLists.txt#L46) `fast_ofio` target |

## 已知问题与限制

1. **CMakeLists.txt 有注释掉的旧 target** ([CMakeLists.txt:33-44](CMakeLists.txt#L33-L44))：
   `fast_lio_localization`, `publish_initial_pose`, `transform_fusion` 被注释掉。
   这些节点的可执行文件需要从其他包 (如 `fast_lio_uwb`) 获取，或单独编译。
   **添加新 target 前先确认二进制来源，避免重复编译。**

2. **fast_lio_localization 命名历史问题**：节点名含 "fast_lio" 但实际使用 FastGICP 算法，
   不依赖 FAST-LIO。修改节点名需要同步改 launch 文件中的 `type` 引用。

3. **global_planner.launch 与 global_planner_in_sim.launch 定位方案不同**：
   - 实飞版：依赖外部 `fast_lio_uwb/locating_mid360.launch`
   - 仿真版：使用本包的 `fast_lio_localization` + `transform_fusion` + `publish_initial_pose`
   - 不要混用两套定位方案。

4. **包依赖 `mavros_msgs`** — 即使不编译 fast_ofio，`package.xml` 中也声明了此依赖。
   在没有 MAVROS 的环境中 `catkin build` 可能失败，需安装或移除该依赖。

## 迭代记录

<!-- 遇到问题在此记录，下次 agent 直接扫码此项避免重复犯错 -->
<!-- 格式: YYYY-MM-DD HH:MM 问题描述 → 解决方法 -->

2026-07-05 初始文档化：
  - 为所有 launch 文件添加了结构化注释 (功能/数据流/依赖/参数说明)
  - 为所有 .cpp 源文件添加了 Doxygen 风格注释 (与 fast_ofio 一致)
  - 编写了 README.md (用户文档) 和 AGENTS.md (本文件)
  - CMakeLists.txt 中有 3 个被注释的旧 target，需注意二进制来源

