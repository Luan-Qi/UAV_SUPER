# uav_localization — UAV 激光 SLAM 定位 ROS 包

基于 [Cartographer](https://google-cartographer-ros.readthedocs.io/) 的 UAV 激光 SLAM 定位方案，使用 **Livox Mid-360** 固态激光雷达实现三维建图与纯定位。

## 目录

- [硬件需求](#硬件需求)
- [软件架构](#软件架构)
- [节点说明](#节点说明)
- [坐标系与 TF 树](#坐标系与-tf-树)
- [话题列表](#话题列表)
- [快速开始](#快速开始)
- [Launch 文件](#launch-文件)
- [配置文件](#配置文件)
- [包结构](#包结构)
- [依赖](#依赖)
- [参数调优指南](#参数调优指南)

---

## 硬件需求

| 设备 | 型号 | 用途 |
|------|------|------|
| 激光雷达 | Livox Mid-360 | 3D 环境感知 |
| IMU | Livox Mid-360 内置 | 姿态估计辅助 |
| 计算平台 | Orange Pi / NVIDIA Jetson | ROS 节点运行 |

---

## 软件架构

```
┌──────────────────────────────────────────────────┐
│               UAV Localization Stack              │
├──────────────────────────────────────────────────┤
│                                                   │
│  ┌─────────────┐    /livox/lidar    ┌───────────┐│
│  │ Livox ROS   │ ──── CustomMsg ──> │livox_to_  ││
│  │ Driver v2   │                    │pointcloud2││
│  └─────────────┘                    └─────┬─────┘│
│                                          │       │
│                                   /livox/ │       │
│                                  lidar_cloud      │
│                                (PointCloud2)      │
│                                          │       │
│  ┌─────────────┐   /livox/imu    ┌───────▼──────┐│
│  │ Livox IMU   │ ──────────────> │ cartographer ││
│  └─────────────┘                 │    _node      ││
│                                  └───────┬──────┘│
│  ┌─────────────┐   /Odometry             │       │
│  │ 里程计源     │ ────────────────────────┘       │
│  └─────────────┘                          │       │
│                                           ▼       │
│                                  ┌──────────────┐ │
│                                  │cartographer_  │ │
│                                  │occupancy_grid │ │
│                                  │    _node      │ │
│                                  └──────────────┘ │
│                                                   │
└──────────────────────────────────────────────────┘
```

---

## 节点说明

### livox_to_pointcloud2

**文件**: `script/livox_to_pointcloud2.py`

将 Livox ROS Driver v2 的 `CustomMsg` 格式点云转换为 ROS 标准的 `sensor_msgs/PointCloud2`，适配 Cartographer 的 `points2` 输入接口。

| 项目 | 说明 |
|------|------|
| 订阅话题 | `/livox/lidar` (`livox_ros_driver2/CustomMsg`) |
| 发布话题 | `/livox/lidar_cloud` (`sensor_msgs/PointCloud2`) |
| 点云字段 | `x`, `y`, `z`, `intensity` (均为 float32) |
| frame_id | `lidar_link` |

### cartographer_node (外部)

Cartographer 核心 SLAM 节点，来自 `cartographer_ros` 包。根据 launch 文件加载不同配置文件以运行在建图或纯定位模式。

### cartographer_occupancy_grid_node (外部)

将 Cartographer 子图（submap）发布为占据栅格地图话题。

### robot_state_publisher (外部)

解析 `urdf/uav_super.urdf` 并发布 UAV 机体的 TF 坐标变换。

---

## 坐标系与 TF 树

```
map (全局地图原点)
│
└── camera_init (里程计原点, odom_frame)
    │
    └── base_link (UAV 基座, tracking_frame / published_frame)
        ├── livox_frame (IMU 坐标系)
        ├── lidar_link  (LiDAR 坐标系)
        └── camera_init (视觉里程计帧)
```

所有传感器通过 fixed joint 刚性连接到 `base_link`，当前偏移量为零，可根据实际安装位置在 `urdf/uav_super.urdf` 中调整。

---

## 话题列表

| 话题 | 方向 | 消息类型 | 说明 |
|------|------|---------|------|
| `/livox/lidar` | 订阅 | `livox_ros_driver2/CustomMsg` | Livox LiDAR 原始数据 |
| `/livox/lidar_cloud` | 发布 | `sensor_msgs/PointCloud2` | 转换后的标准点云 |
| `/livox/imu` | 订阅 | `sensor_msgs/Imu` | Livox 内置 IMU 数据 |
| `/Odometry` | 订阅 | `nav_msgs/Odometry` | 外部里程计（如视觉里程计） |

---

## 快速开始

### 1. 环境依赖

```bash
# ROS (Melodic/Noetic)
sudo apt install ros-<distro>-cartographer-ros
sudo apt install ros-<distro>-livox-ros-driver2
```

### 2. 编译

```bash
cd ~/catkin_ws
catkin_make --only-pkg-with-deps uav_localization
source devel/setup.bash
```

### 3. 建图 (SLAM Mapping)

```bash
# 使用 rosbag 建图
roslaunch uav_localization mapping_mid360.launch

# 实时建图（不播放 rosbag）
roslaunch uav_localization mapping_mid360.launch use_bag:=false

# 保存地图
rosservice call /finish_trajectory 0
rosservice call /write_state "{filename: '${HOME}/map.pbstream'}"
```

### 4. 纯定位 (Pure Localization)

```bash
# 确保地图文件已放置在 /mnt/nvme/map.pbstream
roslaunch uav_localization localization_mid360.launch

# 实时定位（不播放 rosbag）
roslaunch uav_localization localization_mid360.launch use_bag:=false
```

---

## Launch 文件

### mapping_mid360.launch

Cartographer 3D SLAM 建图启动文件。

| 参数 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `use_bag` | bool | `true` | 是否回放 rosbag（`/mnt/nvme/rosbag/float2.bag`） |
| `rviz` | bool | `true` | 是否启动 rviz 可视化 |

启动的节点：
- `robot_state_publisher` — 发布 TF 变换
- `cartographer_node` — SLAM 核心（配置: `mid360_3d.lua`）
- `cartographer_occupancy_grid_node` — 栅格地图发布
- `playbag` (条件) — rosbag 回放
- `rviz` (条件) — 3D 可视化

### localization_mid360.launch

Cartographer 纯定位启动文件。需要预先建好的 `.pbstream` 地图文件。

| 参数 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `use_bag` | bool | `true` | 是否回放 rosbag，回放时跳过前 30 秒 |
| `rviz` | bool | `true` | 是否启动 rviz 可视化 |

与建图模式的区别：
- 加载 `mid360_3d_localization.lua`（增加纯定位参数）
- 通过 `-load_state_filename` 加载已有地图
- 使用 `pure_localization_trimmer` 控制内存

---

## 配置文件

### mid360_3d.lua — 3D 建图配置

Cartographer 3D SLAM 基础配置，针对 Livox Mid-360 优化。

**关键参数**：

| 参数 | 值 | 说明 |
|------|-----|------|
| `num_point_clouds` | 1 | 单个 3D 点云传感器 |
| `num_accumulated_range_data` | 2 | 累积 2 帧为一个 scan，增厚点云 |
| `pose_publish_period_sec` | 0.005 | 200 Hz 位姿发布频率 |
| `optimize_every_n_nodes` | 320 | 每 320 个节点全局优化 |
| `constraint_builder.sampling_ratio` | 0.03 | 3% 约束采样率 |
| `constraint_builder.min_score` | 0.62 | 回环检测最低分数 |
| `global_localization_min_score` | 0.66 | 全局重定位最低分数 |
| `huber_scale` | 500 | Huber 损失尺度 |
| `max_num_iterations` | 10 | Ceres 优化最大迭代次数 |
| `num_background_threads` | 7 | 后台线程数 |

### mid360_3d_localization.lua — 纯定位配置

继承 `mid360_3d.lua`，增加纯定位专用参数。

| 参数 | 值 | 说明 |
|------|-----|------|
| `max_submaps_to_keep` | 3 | 纯定位仅保留 3 个活跃子图 |
| `optimize_every_n_nodes` | 100 | 比建图更频繁的位姿图优化 |

---

## 包结构

```
uav_localization/
├── CMakeLists.txt                       # 构建配置
├── package.xml                          # ROS 包元信息
├── README.md                            # 本文档
├── config/
│   ├── mid360_3d.lua                    # 3D 建图 Cartographer 配置
│   ├── mid360_3d_localization.lua       # 纯定位 Cartographer 配置
│   ├── mapping_3d.rviz                  # rviz 配置（备用）
│   └── demo_3d.rviz                     # rviz 演示配置
├── launch/
│   ├── mapping_mid360.launch            # 建图 launch 文件
│   └── localization_mid360.launch       # 纯定位 launch 文件
├── script/
│   └── livox_to_pointcloud2.py          # Livox → PointCloud2 转换节点
└── urdf/
    └── uav_super.urdf                   # UAV URDF 模型
```

---

## 依赖

### ROS 包

| 包名 | 用途 |
|------|------|
| `cartographer_ros` | Cartographer SLAM 核心 |
| `livox_ros_driver2` | Livox LiDAR ROS 驱动 |
| `robot_state_publisher` | URDF TF 发布 |
| `roscpp` | ROS C++ 核心库 |
| `std_msgs` | 标准消息类型 |
| `geometry_msgs` | 几何消息类型 |
| `nav_msgs` | 导航消息类型 |
| `sensor_msgs` | 传感器消息类型 |

### 系统依赖

- ROS Melodic 或 Noetic
- Python 3
- Catkin 构建系统

---

## 参数调优指南

### 提升建图质量

1. **增厚点云**: 增大 `num_accumulated_range_data`（代价是延迟增加），Mid-360 点云较稀疏，设为 2~3 效果较好。
2. **回环检测**: 降低 `constraint_builder.min_score` 可检测更多回环，但可能引入误匹配。
3. **优化频率**: 降低 `optimize_every_n_nodes` 使全局优化更频繁，轨迹更平滑。

### 降低计算负载

1. **减少线程**: 降低 `num_background_threads`。
2. **增大优化间隔**: 增大 `optimize_every_n_nodes`。
3. **降低采样率**: 增大 `constraint_builder.sampling_ratio` 的分母（当前 0.03）。

### 提升定位精度

1. **更频繁优化**: 纯定位模式下 `optimize_every_n_nodes = 100` 适合高动态场景；稳定环境可适当增大。
2. **子图数量**: 纯定位模式下 `max_submaps_to_keep = 3` 足够，增大可提高鲁棒性但增加内存。
3. **IMU 标定**: 确保 `urdf/uav_super.urdf` 中 IMU 的安装位置和姿态准确。

---

## 许可

Cartographer 相关文件使用 Apache License 2.0。

## 作者

鸾棋
