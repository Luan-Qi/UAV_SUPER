-- Copyright 2016 The Cartographer Authors
--
-- Licensed under the Apache License, Version 2.0 (the "License");
-- you may not use this file except in compliance with the License.
-- You may obtain a copy of the License at
--
--      http://www.apache.org/licenses/LICENSE-2.0
--
-- Unless required by applicable law or agreed to in writing, software
-- distributed under the License is distributed on an "AS IS" BASIS,
-- WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
-- See the License for the specific language governing permissions and
-- limitations under the License.

--- @file mid360_3d.lua
--- @brief Cartographer 3D SLAM 建图配置文件（Livox Mid-360 LiDAR）。
---
--- 该文件是 Cartographer 3D SLAM 的主配置文件，基于 Cartographer 官方
--- map_builder.lua 和 trajectory_builder.lua 模板。针对 Livox Mid-360
--- 固态激光雷达的特性进行了以下定制：
---
--- 硬件假设：
--- - 1 个 3D 点云传感器 (num_point_clouds = 1)
--- - 内置 IMU（通过 /livox/imu 话题）
--- - 外部里程计（通过 /Odometry 话题）
--- - 非 2D 投影模式（适用于 UAV 三维运动）
---
--- 关键坐标帧：
--- - map_frame       = "map"         : 全局地图坐标系
--- - tracking_frame  = "base_link"   : SLAM 跟踪的机器人基座帧
--- - published_frame = "base_link"   : 发布的位姿参考帧
--- - odom_frame      = "camera_init" : 里程计原点帧
---
--- @section params 关键参数说明
--- - num_accumulated_range_data = 2  : 累积 2 帧点云作为一个 scan，增厚点云。
--- - pose_publish_period_sec = 5e-3   : 200 Hz 高频位姿发布。
--- - submap_publish_period_sec = 0.3  : 每 0.3 s 发布一次子图。
--- - POSE_GRAPH.optimize_every_n_nodes = 320 : 每 320 个节点触发一次全局优化。
--- - constraint_builder.sampling_ratio = 0.03 : 3% 约束采样率（节省计算量）。
--- - constraint_builder.min_score = 0.62     : 回环检测最低分数阈值。
--- - global_localization_min_score = 0.66    : 全局重定位最低分数阈值。
---
--- @author 鸾棋
--- @date   2025

include "map_builder.lua"
include "trajectory_builder.lua"

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,
  map_frame = "map",
  tracking_frame = "base_link",
  published_frame = "base_link",
  odom_frame = "camera_init",
  provide_odom_frame = false,
  publish_frame_projected_to_2d = false,
  use_pose_extrapolator = false,
  use_odometry = false,
  use_nav_sat = false,
  use_landmarks = false,
  num_laser_scans = 0,
  num_multi_echo_laser_scans = 0,
  num_subdivisions_per_laser_scan = 5,
  num_point_clouds = 1,
  lookup_transform_timeout_sec = 0.2,
  submap_publish_period_sec = 0.3,
  pose_publish_period_sec = 5e-3,
  trajectory_publish_period_sec = 30e-3,
  rangefinder_sampling_ratio = 1.,
  odometry_sampling_ratio = 1.,
  fixed_frame_pose_sampling_ratio = 1.,
  imu_sampling_ratio = 1.,
  landmarks_sampling_ratio = 1.,
}

--- 3D 轨迹构建器：累积 2 帧点云作为一个 scan，增厚稀疏点云。
TRAJECTORY_BUILDER_3D.num_accumulated_range_data = 2

--- 启用 3D 轨迹构建器，使用 7 个后台线程。
MAP_BUILDER.use_trajectory_builder_3d = true
MAP_BUILDER.num_background_threads = 7

--- 位姿图优化参数。
--- huber_scale = 500：Huber 损失尺度，对大误差使用 L1 范数以增强鲁棒性。
POSE_GRAPH.optimization_problem.huber_scale = 5e2
--- 每 320 个节点触发一次全局位姿图优化。
POSE_GRAPH.optimize_every_n_nodes = 320
--- 约束构建时仅采样 3% 的点以加速计算。
POSE_GRAPH.constraint_builder.sampling_ratio = 0.03
--- Ceres 求解器最大迭代 10 次。
POSE_GRAPH.optimization_problem.ceres_solver_options.max_num_iterations = 10
--- 回环检测最低分数阈值（低于此分数的候选回环被丢弃）。
POSE_GRAPH.constraint_builder.min_score = 0.62
--- 全局重定位最低分数阈值。
POSE_GRAPH.constraint_builder.global_localization_min_score = 0.66

return options
