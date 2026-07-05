# AGENTS.md — uav_util

> 给后续 agent 的速查手册。**短、准、可迭代**。每犯一次错就更新一次。

## 目录结构

```
uav_util/
├── README.md               ← 本包说明
├── AGENTS.md               ← 本文件
├── CMakeLists.txt           ← 编译配置 (6 个 C++ 节点)
├── package.xml              ← 包依赖
├── src/
│   ├── generate_pcd_map.cpp      ← 程序化点云地图生成 (支持 xyz/xyzi/xyzrgb)
│   ├── generate_pcd_map_old.cpp  ← 旧版地图生成 (仅 XYZI, 未编译)
│   ├── imu_to_odom/
│   │   ├── imu_to_odom.hpp       ← IMU 积分里程计 (header-only 实现)
│   │   ├── imu_to_odom_node.cpp  ← IMU 里程计 ROS 节点入口
│   │   └── imu_health_check_node.cpp ← IMU 晃动静止健康检测
│   ├── merge_cloud_node.cpp      ← 双 LiDAR 时间同步空间融合
│   ├── pcd_downsample_node.cpp   ← PCD 体素降采样
│   └── simple_camera_driver.cpp  ← V4L2 USB 相机 ROS 驱动
├── launch/
│   ├── generate_pcd_map.launch   ← 地图生成 (5 种形状 + 噪声/稀疏/孔洞)
│   ├── imu_health_check.launch   ← IMU 健康检测 (18 个阈值参数)
│   ├── imu_to_odom.launch        ← IMU 里程计 + RViz
│   ├── merge_cloud.launch        ← 双 LiDAR 融合 (各自独立外参)
│   ├── pcd_downsample.launch     ← PCD 降采样
│   └── simple_camera.launch      ← USB 相机驱动
├── rviz/
│   └── imu_to_odom.rviz          ← IMU 里程计 RViz 配置
└── script/
    └── compare_topic_timestamps.py ← 话题时间戳对比工具
```

## 构建与运行

```bash
# 编译
catkin build uav_util

# 地图生成
roslaunch uav_util generate_pcd_map.launch shape_type:=cylinder_corridor

# IMU 里程计
roslaunch uav_util imu_to_odom.launch imu_topic:=/mavros/imu/data_raw

# IMU 健康检测
roslaunch uav_util imu_health_check.launch

# 双 LiDAR 融合
roslaunch uav_util merge_cloud.launch

# PCD 降采样
roslaunch uav_util pcd_downsample.launch pcd_path:=/home/user/map.pcd leaf_size:=0.1

# USB 相机
roslaunch uav_util simple_camera.launch device_id:=0 width:=1280 height:=720
```

## 节点速查表

| 节点 | 二进製名 | 关键依赖 | 输入/输出 | 核心算法 |
|------|---------|---------|-----------|---------|
| pcd_map_generator | `pcd_map_generator_node` | PCL, Eigen3 | PointCloud2 发布 | 参数化几何体生成 + 6-DOF 变换 + 后处理 |
| imu_to_odom | `imu_to_odom_node` | Eigen3 | sub Imu / pub Odometry+Path | Rodrigues 姿态积分 + ZUPT + 自适应阻尼 |
| imu_health_check | `imu_health_check_node` | Eigen3 | sub Imu / 终端输出 | 前后静止窗口差分统计 |
| merge_cloud_node | `merge_cloud_node` | PCL, message_filters | sub 2×PointCloud2 / pub merged | ApproximateTime 同步 + PCL transform |
| pcd_downsample_node | `pcd_downsample_node` | PCL, std::filesystem | 文件 PCD → VoxelGrid → _downsampled.pcd | VoxelGrid 体素滤波器 |
| simple_camera_driver | `simple_camera_driver` | OpenCV, image_transport | /dev/videoN → image_raw+compressed | cv::VideoCapture V4L2 |

## 工程约定

- C++11/14, ROS1 catkin, PCL 1.8+, Eigen3, OpenCV
- `imu_to_odom.hpp` 为 header-only 实现（类声明与定义在同一文件）
- `generate_pcd_map.cpp` 使用模板编程（`template<typename PointT>`），支持 xyz/xyzi/xyzrgb
- `generate_pcd_map_old.cpp` 不在 CMakeLists.txt 中编译，仅保留供参考
- 所有 launch 文件参数通过 `ros::NodeHandle("~")` 私有命名空间加载
- 物理量单位必须在注释中标注: [m], [rad], [rad/s], [m/s²], [Hz], [pixel], [mm]
- 6-DOF 变换顺序: roll(pitch(yaw)) 即 ZYX (绕定轴旋转)
- 新增参数的 4 步流程: 代码变量 → `nh.param()` → launch arg → launch node 块

## 关键算法细节

### imu_to_odom 积分链
```
IMU raw → gyro_bias 扣除 → Rodrigues 姿态积分 → 重力方向估计
       → accel_bias 扣除 → 世界系加速度 → 死区过滤 → 速度积分
       → ZUPT 静止判定 → 静止:速度衰减 / 运动:mild 阻尼 → 梯形位置积分
```

### merge_cloud 变换链
```
cloud1 → transform1(R1|t1) ─┐
                              ├→ operator+ → merged → publish
cloud2 → transform2(R2|t2) ─┘
```

### generate_pcd_map 流水线
```
ShapeFactory.create(type) → ShapeBase::generate(cloud) → PostProcessing(noise/sparse/hole)
→ pcl::toROSMsg → publish + savePCD
```

## 已知问题与陷阱

- **imu_to_odom**: 纯惯性积分长期漂移无外部校正；ZUPT 窗口未满时保守返回静止；Yaw 完全依赖 IMU 积分
- **imu_health_check**: 需要人为晃动操作，无法自动化；结果受阈值配置影响大；输出的 "结论" 仅供参考
- **merge_cloud**: 仅支持恰好 2 个 LiDAR；无时间戳外推/插值；`ApproximateTime` 策略可能在不同帧率时丢失匹配
- **pcd_downsample**: 使用 `std::filesystem`（需 C++17）；输入文件不存在时报错退出；发布频率固定 1 Hz
- **simple_camera**: 摄像头实际参数可能不支持设定值（OpenCV 会自动调整）；MJPEG 格式需要摄像头硬件支持
- **generate_pcd_map**: 模板编译时 `PointAssigner<T>` 对于未特化的类型不作任何赋值

## 验收标准

- [ ] `catkin build uav_util` 编译成功 (6 个可执行文件)
- [ ] `generate_pcd_map.launch` 可生成 .pcd 文件并发布到 `/map`
- [ ] `imu_to_odom.launch` 可订阅 IMU 并发布 /Odometry 和 /Path
- [ ] IMU 静止时里程计位置不漂移（ZUPT 生效）
- [ ] `imu_health_check.launch` 按流程操作后输出检测结论
- [ ] `merge_cloud.launch` 可同步订阅双 LiDAR 并发布融合点云
- [ ] `pcd_downsample.launch` 可读取 PCD 并输出降采样后的文件
- [ ] `simple_camera.launch` 可打开摄像头并发布图像话题

## 迭代记录

<!-- 格式: YYYY-MM-DD 问题描述 → 解决方法 -->

2026-07-05 代码注释规范化:
  - 为所有源文件和 launch 文件添加 Doxygen/XML 注释
  - 创建包级 README.md 和 AGENTS.md
  - generate_pcd_map_old.cpp 标注为遗留代码
