# uav_util — 无人机工具集

## 功能简介

通用无人机工具包，包含程序化点云地图生成、IMU 惯性里程计、IMU 健康检测、双 LiDAR 点云融合、PCD 降采样、USB 相机驱动等实用节点。

## 节点说明

| 节点 | 类型 | 功能 |
|------|------|------|
| `pcd_map_generator` | C++ | 程序化生成三维点云地图 — 支持立方体/球体/走廊等形状 |
| `imu_to_odom` | C++ | IMU 惯性里程计 — 重力补偿 + 陀螺零偏校准 + ZUPT 静止检测 |
| `imu_health_check` | C++ | IMU 晃动静止健康检测 — 比较前后静止窗口的 bias/噪声/跳变 |
| `merge_cloud_node` | C++ | 双 LiDAR 点云时间同步与空间融合 — message_filters 近似时间对齐 |
| `pcd_downsample_node` | C++ | PCD 点云体素降采样 — VoxelGrid 滤波器 |
| `simple_camera_driver` | C++ | V4L2 USB 摄像头 ROS 驱动 — 支持 MJPEG/YUYV 格式 |

另有遗留节点 `generate_pcd_map_old`（仅支持 PointXYZI，功能已被新版取代）和 `compare_topic_timestamps.py` 时间戳对比脚本。

## 话题接口

### pcd_map_generator
| 话题 | 方向 | 类型 | 说明 |
|------|------|------|------|
| `/map` | pub | `sensor_msgs::PointCloud2` | 生成的点云地图 (latched) |

### imu_to_odom
| 话题 | 方向 | 类型 | 说明 |
|------|------|------|------|
| `imu_topic` (可配) | sub | `sensor_msgs::Imu` | IMU 原始数据 |
| `odom_topic` (可配) | pub | `nav_msgs::Odometry` | 积分里程计 |
| `path_topic` (可配) | pub | `nav_msgs::Path` | 累积轨迹路径 |

### merge_cloud_node
| 话题 | 方向 | 类型 | 说明 |
|------|------|------|------|
| `cloud1_topic` (可配) | sub | `sensor_msgs::PointCloud2` | LiDAR 1 点云 |
| `cloud2_topic` (可配) | sub | `sensor_msgs::PointCloud2` | LiDAR 2 点云 |
| `merged_cloud` | pub | `sensor_msgs::PointCloud2` | 融合后点云 |

### simple_camera_driver
| 话题 | 方向 | 类型 | 说明 |
|------|------|------|------|
| `/simple_camera/image_raw` | pub | `sensor_msgs::Image` | 原始 RGB8 图像 |
| `/simple_camera/image_compressed` | pub | `sensor_msgs::CompressedImage` | JPEG 压缩图像 |

## 依赖

- ROS1 (Noetic)
- OpenCV (simple_camera)
- PCL (pcd_map_generator, pcd_downsample, merge_cloud)
- Eigen3 (imu_to_odom, generate_pcd_map)
- message_filters (merge_cloud)
- V4L2 (simple_camera)

## 快速开始

```bash
# 编译
catkin build uav_util

# 生成圆柱走廊点云地图
roslaunch uav_util generate_pcd_map.launch shape_type:=cylinder_corridor

# IMU 里程计
roslaunch uav_util imu_to_odom.launch

# IMU 健康检测 (启动后按提示晃动飞机)
roslaunch uav_util imu_health_check.launch

# 双 LiDAR 融合
roslaunch uav_util merge_cloud.launch

# PCD 降采样
roslaunch uav_util pcd_downsample.launch pcd_path:=/path/to/map.pcd leaf_size:=0.1

# USB 相机驱动
roslaunch uav_util simple_camera.launch device_id:=0 width:=640 height:=480
```

## 算法参考

### IMU 里程计 (imu_to_odom)
1. **静态校准** — 启动时采集 N 帧静止数据，估计重力方向与陀螺零偏
2. **姿态积分** — 角速度去偏置后用 Rodrigues 公式积分旋转矩阵
3. **速度积分** — 加速度变换到世界系，扣除重力，积分为速度
4. **ZUPT 静止检测** — 滑动窗口内加速度/角速度方差判定静止
5. **自适应阻尼** — 静止时速度指数衰减到零，运动时 mild 阻尼
6. **梯形积分** — 使用梯形法则积位移 `pos += dt * (v_prev + v_curr) / 2`

参考: https://github.com/Abin1258/imu_to_odom

### IMU 健康检测
1. 初始静止采样 → 基准统计
2. 提示晃动飞机
3. 恢复静止采样 → 后静止统计
4. 对比前后 gyro/accel 均值、噪声、跳变、疑似饱和 → 输出结论

## 坐标系约定

- IMU 原始数据: IMU 本地坐标系
- 重力向量: 校准阶段在 IMU 系中估计（通常 Z 轴向上 ~9.8 m/s²）
- 里程计输出: odom 坐标系，与 MAVROS ENU 一致
- 生成地图: shape 局部坐标系 → 6-DOF 变换 (roll→pitch→yaw + trans) → map 系

## 参数配置

关键 launch 文件参数见各 `.launch` 文件内的 XML 注释。通用注意事项：

- 物理量单位: [m], [rad], [rad/s], [m/s²], [Hz], [pixel], [mm]
- IMU 数据格式遵循 `sensor_msgs/Imu` 标准 (accel=m/s², gyro=rad/s)
- 深度图像为 32FC1 (mm)，需除 1000 转米
- `merge_cloud` 变换顺序始终为 ZYX (yaw → pitch → roll)

## 迭代记录

2026-07-05 代码注释规范化:
  - 为所有源文件和 launch 文件添加 Doxygen/XML 注释
  - 创建包级 README.md 和 AGENTS.md

## 许可证

imu_to_odom 参考自 https://github.com/Abin1258/imu_to_odom
