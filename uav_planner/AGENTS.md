# CLAUDE.md — uav_planner AI Agent 快速指南

## 项目概要

- **包名**: `uav_planner`
- **类型**: ROS1 (Noetic) Catkin 包
- **功能**: UAV 3D 路径规划与跟踪 — A* 全局搜索、路径简化、平滑、跟随、坐标变换
- **依赖**: Eigen3, OctoMap, PCL, tf2, uav_px4_ctrl
- **硬件平台**: OrangePi (ARM) / Ubuntu

## 当前迭代状态

| 日期 | 变更 |
|------|------|
| 2026-07-05 | astar_planner_node 新增 `planning_mode` 参数 (0/1/2)，模式2 支持里程计实时起点 |
| 2026-07-05 | 新增 AGENT.md, README.md |

## 关键文件索引

### 核心入口 — 启动文件
| 文件 | 作用 |
|------|------|
| [launch/astar_planner.launch](launch/astar_planner.launch) | A* 规划器启动 (含 planning_mode) |
| [launch/octomap_publisher.launch](launch/octomap_publisher.launch) | PCD→OctoMap 离线加载 |
| [launch/planner_path_simplify.launch](launch/planner_path_simplify.launch) | 路径简化 (dp/greedy) |
| [launch/planner_path_smoothify.launch](launch/planner_path_smoothify.launch) | 路径平滑 (bezier/dc) |

### A* 规划模块 ★
| 文件 | 作用 |
|------|------|
| [src/astar/astar_planner_node.cpp](src/astar/astar_planner_node.cpp) | ROS 节点主入口 (订阅/发布/模式控制) |
| [src/astar/astar_searcher.cpp](src/astar/astar_searcher.cpp) | A* 核心算法 (26邻域3D搜索, 对角启发式) |
| [include/uav_planner/astar_searcher.h](include/uav_planner/astar_searcher.h) | GridNode 结构体 + AstarPathFinder 类定义 |
| [src/astar/astar_visualization.cpp](src/astar/astar_visualization.cpp) | 障碍物可视化 (PointCloud2 / MarkerArray) |
| [include/uav_planner/astar_show_obs.h](include/uav_planner/astar_show_obs.h) | 可视化函数声明 |

### 路径后处理
| 文件 | 算法 | 关键参数 |
|------|------|----------|
| [src/path_simplify/DP_path_simplify.cpp](src/path_simplify/DP_path_simplify.cpp) | Douglas-Peucker 3D | `simplification_threshold`, `min_segment_length` |
| [src/path_simplify/greedy_path_simplify.cpp](src/path_simplify/greedy_path_simplify.cpp) | 贪心碰撞检测 | `check_step_size`, `collision_radius` |
| [src/path_simplify/simple_path_simplify.cpp](src/path_simplify/simple_path_simplify.cpp) | 简单最近点投影 | — |
| [src/path_simplify/path_downsampling.cpp](src/path_simplify/path_downsampling.cpp) | 均匀降采样 | — |
| [src/path_smoothify/bezier_path_smoothify.cpp](src/path_smoothify/bezier_path_smoothify.cpp) | 二次/三次贝塞尔 | `bezier_order` (2=C1, 3=C2), `bezier_radius`, `step_size` |
| [src/path_smoothify/discrete_curve_path_smoothify.cpp](src/path_smoothify/discrete_curve_path_smoothify.cpp) | 离散曲线优化 | `weight_data`, `weight_smooth`, `tolerance` |

### 路径跟随
| 文件 | 作用 |
|------|------|
| [src/simple_path_follower/fast_path_follower.cpp](src/simple_path_follower/fast_path_follower.cpp) | 主要跟随器 — 逐点发布目标 |
| [src/simple_path_follower/fforward_path_follower.cpp](src/simple_path_follower/fforward_path_follower.cpp) | 前馈跟随器 |
| [src/simple_path_follower/visual_path_follower.cpp](src/simple_path_follower/visual_path_follower.cpp) | 可视化辅助函数 |
| [src/simple_path_follower/visual_path_follower.h](src/simple_path_follower/visual_path_follower.h) | 可视化头文件 |

### 其他
| 文件 | 作用 |
|------|------|
| [src/transform_path.cpp](src/transform_path.cpp) | map→odom 坐标系路径变换 |
| [src/slp_planner/pca_direction_node.cpp](src/slp_planner/pca_direction_node.cpp) | PCA 主方向分析 |
| [CMakeLists.txt](CMakeLists.txt) | 构建配置 (所有可执行文件定义在此) |

## 关键数据流

```
PCD → octomap_server → /octomap_full → astar_planner_node
                                            ↑ /start_pose (模式0/1)
                                            ↑ /goal_pose  (所有模式)
                                            ↑ /odometry    (模式2)
                                            ↓
                                     /global_path_raw (或 /planned_path)
                                            ↓
                              ┌─────────────┴─────────────┐
                              ↓                           ↓
                       path_simplify              path_smoothify
                              ↓                           ↓
                       /global_path            /global_path_smoothed
                              ↓                           ↓
                              └─────────────┬─────────────┘
                                            ↓
                                   fast_path_follower
                                            ↓
                              /move_base_simple/goal (逐点)
```

## 消息类型约定

| 话题 | 类型 | 用途 |
|------|------|------|
| `/octomap_full` | `octomap_msgs::Octomap` | 障碍物地图输入 |
| `/start_pose` | `geometry_msgs::PoseStamped` | 路径起点 |
| `/goal_pose` | `geometry_msgs::PoseStamped` | 路径终点 |
| `/odometry` | `nav_msgs::Odometry` | 里程计位姿 (模式2) |
| 输出路径 | `nav_msgs::Path` | 规划结果 (frame_id="map") |
| `/map_to_odom` | `nav_msgs::Odometry` | map→odom 变换 (pose = T_map_odom) |

## 规划模式速查 (astar_planner_node)

```cpp
// 模式 0 — 标准模式 (默认)
// 每次需输入 start + goal → 规划 → 清空两者 → 等待下一次 start + goal

// 模式 1 — 连续模式
// 首次需 start + goal → 规划 → goal 成为新 start → 只需新 goal

// 模式 2 — 里程计模式
// 持续订阅 /odometry → 收到 goal 时以当前 odom 为 start → 规划
```

详见 [astar_planner_node.cpp:128](src/astar/astar_planner_node.cpp#L128) 的 `planning_mode` 参数读取处。

## 修改指南

### 修改 A* 搜索行为
- 启发式函数: [astar_searcher.cpp:190](src/astar/astar_searcher.cpp#L190) `getHeu()` — 可选 Euclidean / Manhattan / Diagonal
- 邻域扩展: [astar_searcher.cpp:153](src/astar/astar_searcher.cpp#L153) `AstarGetSucc()` — 当前为 26 邻域
- 障碍物膨胀: [astar_planner_node.cpp](src/astar/astar_planner_node.cpp) `inflater_size` 参数

### 添加新节点
1. 在 `src/` 对应子目录创建 `.cpp`
2. 在 [CMakeLists.txt](CMakeLists.txt) 添加 `add_executable` + `target_link_libraries`
3. 在 `launch/` 创建对应 `.launch` 文件

### 修改规划模式行为
- 所有模式逻辑在 [astar_planner_node.cpp](src/astar/astar_planner_node.cpp):
  - 模式分派 (goalCallback): 行 ~119
  - 模式分派 (规划后): 行 ~200
  - 参数读取: 行 ~135

## 构建命令

```bash
# 在工作空间根目录
catkin build uav_planner
```

## 相关包

- `uav_px4_ctrl`: 飞控接口 (TakeoffNotify 服务, pub_goal 节点)
- `fast_gicp`: 点云配准 (SLAM 前端)
- `octomap_server`: 八叉树地图服务
