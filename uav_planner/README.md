# uav_planner

UAV 3D 路径规划与跟踪综合包，基于 ROS1 (Noetic) / Catkin 构建。包含 A* 搜索、路径简化、路径平滑、路径跟随和坐标变换等模块。

## 架构总览

```
  PCD / 传感器点云
        │
        ▼
┌───────────────────┐
│ octomap_server    │  ← octomap_publisher.launch
│ (PCD → OctoMap)   │
└────────┬──────────┘
         │ /octomap_full
         ▼
┌───────────────────┐     /start_pose      /odometry (模式2)
│ astar_planner_node│◄────────────────────────────────────
│ (A* 3D 路径搜索)   │     /goal_pose
└────────┬──────────┘
         │ /global_path_raw (nav_msgs::Path)
         │
    ┌────┴────┐
    ▼         ▼
┌────────┐ ┌──────────┐
│ 简化    │ │ 平滑      │
│ DP /   │ │ Bezier / │
│ Greedy │ │ DC       │
└───┬────┘ └────┬─────┘
    │            │
    ▼            ▼
 /global_path   /global_path_smoothed
    │
    ▼
┌───────────────────┐
│ path_follower     │ ← 逐点跟踪，发布目标点
│ (路径跟随)         │
└───────────────────┘
```

## 目录结构

```
uav_planner/
├── CMakeLists.txt                      # 构建配置
├── package.xml                         # 包元数据
├── README.md                           # 本文件
├── AGENT.md                            # AI Agent 项目索引
├── launch/
│   ├── astar_planner.launch            # A* 全局路径规划
│   ├── octomap_publisher.launch        # PCD 文件→OctoMap 发布
│   ├── planner_path_simplify.launch    # 路径简化 (DP / Greedy)
│   └── planner_path_smoothify.launch   # 路径平滑 (Bezier / DC)
├── include/uav_planner/
│   ├── astar_searcher.h                # A* 搜索器类 + GridNode 结构体
│   └── astar_show_obs.h                # 障碍物可视化 (PointCloud/MarkerArray)
└── src/
    ├── astar/
    │   ├── astar_planner_node.cpp      # A* 规划器 ROS 节点 ★
    │   ├── astar_searcher.cpp          # A* 核心算法实现
    │   └── astar_visualization.cpp     # 障碍物可视化辅助
    ├── transform_path.cpp              # 全局路径→局部坐标系变换
    ├── path_simplify/
    │   ├── DP_path_simplify.cpp        # Douglas-Peucker 3D 简化
    │   ├── greedy_path_simplify.cpp    # 贪心碰撞检测简化
    │   ├── simple_path_simplify.cpp    # 简单最近点简化
    │   └── path_downsampling.cpp       # 均匀降采样
    ├── path_smoothify/
    │   ├── bezier_path_smoothify.cpp   # 二次/三次贝塞尔平滑
    │   └── discrete_curve_path_smoothify.cpp  # 离散曲线优化平滑
    ├── simple_path_follower/
    │   ├── fast_path_follower.cpp      # 快速路径跟随器 (主要使用)
    │   ├── fforward_path_follower.cpp  # 前馈路径跟随器
    │   ├── visual_path_follower.cpp    # 跟随可视化工具
    │   └── visual_path_follower.h      # 可视化工具头文件
    └── slp_planner/
        └── pca_direction_node.cpp      # PCA 主方向分析
```

## 节点详解

### 1. astar_planner_node — A* 3D 路径规划器 ★

**功能**：接收 OctoMap 障碍物地图，在收到起点/终点后执行 3D A* 搜索，发布全局路径。

| 项目 | 内容 |
|------|------|
| **订阅** | 配置的 octomap 话题 (默认 `/octomap_full`) |
| | 配置的 start_pose 话题 (默认 `/start_pose`) |
| | 配置的 goal_pose 话题 (默认 `/goal_pose`) |
| | 配置的 odometry 话题 (默认 `/odometry`，仅模式2) |
| **发布** | `nav_msgs::Path` (默认 `/planned_path`) |
| | `sensor_msgs::PointCloud2` (`obstacle_cloud`，可选) |
| | `visualization_msgs::MarkerArray` (`obstacle_markers`，可选) |
| **服务** | 规划失败时调用 `/planner_fail_notify` (uav_px4_ctrl::TakeoffNotify) |

**关键参数**：

| 参数 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `octomap_topic` | string | `/octomap_full` | 八叉树地图话题 |
| `start_pose_topic` | string | `/start_pose` | 起点话题 |
| `goal_pose_topic` | string | `/goal_pose` | 终点话题 |
| `global_path_topic` | string | `/planned_path` | 输出路径话题 |
| `planning_mode` | int | `0` | 规划模式 (见下方) |
| `odometry_topic` | string | `/odometry` | 里程计话题 (模式2) |
| `enable_obstacle_show` | bool | `false` | 是否发布障碍物可视化 |
| `inflater_size` | int | `2` | 障碍物膨胀格数 |

**规划模式 (planning_mode)**：

| 值 | 模式 | 行为 |
|----|------|------|
| `0` | 标准模式 (默认) | 每次规划都需要分别输入起点和终点，规划后清空两者 |
| `1` | 连续模式 | 首次需要输入起点和终点，规划成功后终点自动成为下一次的起点，只需继续输入新的终点 |
| `2` | 里程计模式 | 持续订阅 `/odometry` 追踪当前位姿，收到新终点时以当前里程计位姿作为起点进行规划 |

**注意**：OctoMap 只在第一次收到时加载，之后不再更新。如需重新加载请重启节点。

---

### 2. path_simplify 系列 — 路径简化

| 节点 | 算法 | 特点 |
|------|------|------|
| `dp_path_simplify` | Douglas-Peucker 3D | 保留关键拐点，阈值控制简化程度 |
| `greedy_path_simplify` | 贪心碰撞检测 | 考虑障碍物碰撞，逐步删减冗余点 |
| `simple_path_simplify` | 最近点投影 | 最简单，速度最快 |
| `path_downsampler` | 均匀降采样 | 固定间隔采样 |

**通用参数**：`input_topic`, `output_topic`

---

### 3. path_smoothify 系列 — 路径平滑

| 节点 | 算法 | 特点 |
|------|------|------|
| `bezier_path_smoothify` | 二次/三次贝塞尔 | 可调阶数(order 2 C1连续, order 3 C2连续) |
| `dc_path_smoothify` | 离散曲线优化 | 数据项+平滑项迭代优化 |

**通用参数**：`input_topic`, `output_topic`, `recaluate_orientation`

---

### 4. fast_path_follower — 路径跟随器

**功能**：接收全局路径和里程计，按步长依次发布下一个目标点用于飞控跟踪。

| 项目 | 内容 |
|------|------|
| **订阅** | `path_topic` (Path), `odom_topic` (Odometry), `map_to_odom_topic` (Odometry) |
| **发布** | `goal_topic` (PoseStamped), MarkerArray 调试可视化 |
| **关键参数** | `goal_distance_threshold` (到达阈值), `path_step` (跳过点数), `publish_rate` |

---

### 5. transform_path — 坐标变换

**功能**：将 map 坐标系下的全局路径转换到 odom 坐标系。

| 项目 | 内容 |
|------|------|
| **订阅** | `global_path_topic` (Path), `map_to_odom_topic` (Odometry) |
| **发布** | `local_path_topic` (Path, frame_id="odom") |

---

### 6. pca_direction_node — PCA 方向分析

计算路径点云的主成分方向，用于辅助飞行决策。

---

### 7. octomap_publisher.launch — 离线地图加载

从 PCD 文件发布点云，经 `octomap_server` 转换为 OctoMap 供规划器使用。

**关键参数**：
- `pcd_file`: PCD 文件路径
- `resolution`: OctoMap 分辨率 (默认 0.2m)

---

## 典型使用流程

### 离线点云规划

```bash
# 1. 加载 PCD 发布 OctoMap
roslaunch uav_planner octomap_publisher.launch pcd_file:=/path/to/map.pcd

# 2. 启动 A* 规划器 (标准模式)
roslaunch uav_planner astar_planner.launch planning_mode:=0

# 3. (可选) 路径简化
roslaunch uav_planner planner_path_simplify.launch planner_type:=greedy

# 4. (可选) 路径平滑
roslaunch uav_planner planner_path_smoothify.launch planner_type:=bezier

# 5. 发布起点和终点
rostopic pub /start_pose geometry_msgs/PoseStamped "..."
rostopic pub /goal_pose geometry_msgs/PoseStamped "..."
```

### 在线里程计模式

```bash
# 启动规划器 (模式2: 以当前里程计位姿为起点)
roslaunch uav_planner astar_planner.launch planning_mode:=2 odometry_topic:=/mavros/local_position/odom

# 只需发布终点即可自动规划
rostopic pub /goal_pose geometry_msgs/PoseStamped "..."
```

---

## 依赖

- ROS1 (Noetic)
- Eigen3
- OctoMap + octomap_msgs
- PCL (pcl_ros, pcl_conversions)
- geometry_msgs, nav_msgs, sensor_msgs, visualization_msgs
- tf2 + tf2_geometry_msgs
- uav_px4_ctrl (自定义消息包)

## 构建

```bash
# 在 catkin workspace 中
catkin build uav_planner
# 或
catkin_make --pkg uav_planner
```
