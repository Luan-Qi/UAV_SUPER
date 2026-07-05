# uav_px4_ctrl

UAV→PX4 飞控接口与控制综合包，基于 ROS1 (Noetic) / Catkin 构建。负责坐标修正、多模式 Offboard 控制、安全门限、任务调度、航点管理与录像联动。

## 架构总览

```
  RC遥控器 ──→ /mavros/rc/in
  里程计 ────→ /Odometry
  轨迹规划器 → /position_cmd
  电池状态 ──→ /mavros/battery
  飞控状态 ──→ /mavros/state
       │
       ▼
┌──────────────────────────────────────────────────┐
│                 px4ctrl_node                      │
│  ┌──────────┐  ┌──────────┐  ┌───────────────┐  │
│  │ RC 状态机 │  │ 坐标修正  │  │ 多模式控制     │  │
│  │ arm/mode/ │  │ ENU→NED  │  │ position/     │  │
│  │ gear 解析 │  │ +90°绕Z  │  │ velocity/atti │  │
│  └──────────┘  └──────────┘  └───────────────┘  │
│  ┌──────────────────────────────────────────┐   │
│  │         安全门限：位置/速度/加速度/Jerk限幅   │   │
│  └──────────────────────────────────────────┘   │
└──────────────────────┬───────────────────────────┘
                       │
     ┌─────────────────┼─────────────────┐
     ▼                 ▼                 ▼
/mavros/         /mavros/          /takeoff_notify
vision_pose/     setpoint_*        (srv)
pose             (position/velocity)

     ▲
     │ /takeoff_notify
     │
┌──────────────────────────────────────────────────┐
│              mission_manger                       │
│  ┌──────────┐  ┌──────────┐  ┌───────────────┐  │
│  │ 航点序列  │  │ 录像控制  │  │ 规划器重试     │  │
│  │ 正则解析  │  │ * 标记   │  │ 最多3次       │  │
│  └──────────┘  └──────────┘  └───────────────┘  │
│  ┌──────────────────────────────────────────┐   │
│  │ 到达检测 → 等待 → 切换航点 → 发布goal_pose  │   │
│  └──────────────────────────────────────────┘   │
└──────────────────────┬───────────────────────────┘
                       │
          ┌────────────┴────────────┐
          ▼                         ▼
     /start_pose              /goal_pose
     (规划器起点)              (规划器目标)

  pub_goal / pub_waypoint ──→ /move_base_simple/goal
    (CLI 单次发布 / 循环序列)
```

## 目录结构

```
uav_px4_ctrl/
├── CMakeLists.txt                      # 构建配置 (4 个可执行文件 + 1 个自定义服务)
├── package.xml                         # 包元数据 (依赖 mavros_msgs, quadrotor_msgs 等)
├── README.md                           # 本文件
├── AGENTS.md                           # AI Agent 项目索引 (迭代状态/关键位置)
├── srv/
│   └── TakeoffNotify.srv               # 起飞通知服务定义 (bool → string)
├── launch/
│   ├── px4ctrl.launch                  # PX4 主控制节点 ★
│   ├── mission_manger.launch           # 任务管理器 (航点序列调度)
│   ├── goal_publisher.launch           # 命令行单次 goal 发布
│   ├── uav_super_localization.launch   # 全自主飞行总启动 ★
│   └── uav_super_mapping.launch        # 建图模式启动
├── script/
│   └── convert_tf_to_odom.py           # TF → Odometry 桥接 (map→base_link)
├── rviz/
│   └── demo_3d.rviz                    # RViz 可视化配置
└── src/
    ├── px4ctrl_node.cpp                # PX4 主控制节点 ★
    ├── px4ctrl/
    │   ├── subscribe.h                 # 5 个数据类型定义 (RC/State/Battery/Command/Odom)
    │   └── subscribe.cpp               # 数据类型实现 (PWM归一化/状态机/Eigen提取)
    ├── mission_manger.cpp              # 全自主任务管理器 ★
    ├── pub_goal.cpp                    # 命令行单次 goal 发布工具
    └── pub_waypoint.cpp                # 航点序列循环发布器 (演示用)
```

## 节点详解

### 1. px4ctrl_node — PX4 主控制节点 ★

**功能**：MAVROS offboard 控制中间层，作为外部位姿/轨迹指令与 PX4 飞控之间的桥梁。

| 项目 | 内容 |
|------|------|
| **订阅** | `/mavros/state` (State), `/mavros/battery` (BatteryState), `/mavros/rc/in` (RCIn), `/position_cmd` (PositionCommand), `<odom_topic>` (Odometry) |
| **发布** | `/mavros/vision_pose/pose` (PoseStamped), `/mavros/setpoint_velocity/cmd_vel` (TwistStamped), `/mavros/setpoint_position/local` (PoseStamped), `/traj_start_trigger` (PoseStamped) |
| **服务** | `/mavros/set_mode`, `/mavros/cmd/arming`, `/mavros/cmd/command`, `/takeoff_notify` |

**关键参数**：

| 参数 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `control_mode` | string | `"position"` | 控制模式 (position / velocity / attitude) |
| `battery_limit` | double | `17.4` | 电池低电压阈值 [V] |
| `takeoff_height` | double | `0.6` | 起飞高度 [m] (全局参数) |
| `odom_topic` | string | `"/Odometry"` | 里程计话题名称 |
| `position_max_x/y/z` | double | `±100.0/5.0` | 位置控制安全边界 [m] |
| `position_max_vel` | double | `6.0` | 位置变化最大速率 [m/s] |
| `position_max_acc` | double | `6.0` | 位置变化最大加速度 [m/s²] |
| `cmd_vel_max_x/y/z` | double | `0.5` | 速度控制最大线速度 [m/s] |
| `cmd_vel_max_acc` | double | `5.0` | 速度控制最大加速度 [m/s²] |
| `cmd_vel_max_jerk` | double | `5.0` | 速度控制最大 jerk [m/s³] |

**控制模式**：

| 值 | 模式 | 行为 |
|----|------|------|
| `position` | 位置控制 | 发布 PoseStamped 到 `/mavros/setpoint_position/local` |
| `velocity` | 速度控制 | 发布 TwistStamped 到 `/mavros/setpoint_velocity/cmd_vel` |
| `attitude` | 姿态控制 | 未实现 (预留) |

**状态机流程**：

```
未连接 ──→ PX4连接 ──→ RC收到 ──→ 里程计收到 ──→ 手动切Offboard+Arm
                                                       │
                                           ┌───────────┴───────────┐
                                           ▼                       ▼
                                      Hold模式                Command模式
                                   (锁定当前位姿)          (接收外部指令)
                                           │                       │
                                           ▼                       ▼
                                      自动起飞              位置/速度/姿态控制
                                      (Z→takeoff_height)   (含安全限幅)
```

**坐标系修正**: 所有输入/输出位姿经 `uav_mavros_pose_fix()` 绕 Z 轴旋转 +90° (ENU → PX4 期望方向)。

---

### 2. mission_manger — 全自主任务管理器 ★

**功能**：航点序列调度、录像自动控制、规划器失败重试、起飞同步。

| 项目 | 内容 |
|------|------|
| **订阅** | odom (Odometry) 或 pose (PoseStamped，odom 优先) |
| **发布** | `start_pose` (PoseStamped), `goal_pose` (PoseStamped), `/camera_recorder/record_control` (String) |
| **服务** | `/takeoff_notify` (TakeoffNotify，接收), `/planner_fail_notify` (TakeoffNotify，接收) |

**关键参数**：

| 参数 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `waypoints` | string | — | 航点字符串 (格式见下方) |
| `distance_threshold` | double | `0.5` | 到达判定欧氏距离阈值 [m] |
| `wait_time` | double | `5.0` | 到达后默认等待时间 [s] |
| `start_delay` | double | `5.0` | 起飞后任务启动延迟 [s] |
| `topic_timeout` | double | `2.0` | 位姿话题超时告警 [s] |
| `mission_cycle` | bool | `true` | 是否循环执行航点 |

**航点字符串格式**：`"[[x,y,z[,wait]][*], [x,y,z[,wait]][*], ...]"`

| 元素 | 说明 |
|------|------|
| `x, y, z` | 目标坐标 [m] |
| `wait` | 到达后等待时间 [s] (可选; 0 = 无限等待; 负值 = 使用默认值) |
| `*` | 录像标记 (可选; 该航点段录像) |

**示例**: `"[[35.0, 0.0, 0.6]*, [52.0, -10.0, 0.8], [52.0, -0.0, 1.0]*]"`

**录像逻辑**：
- 出发航点标记 `*` → 开始录像
- 到达航点标记 `*` 且下一航点无 `*` → 停止录像
- 连续 `*` 航点 → 录像持续不中断

**规划器重试**：收到 `/planner_fail_notify` 后自动重发 start + goal，最多 3 次。

---

### 3. pub_goal — 命令行 goal 发布工具

**功能**：向指定话题发送一次 goal pose 后立即退出。适用于调试/脚本编排。

```bash
rosrun uav_px4_ctrl pub_goal 2.0 3.0 1.0 90 /move_base_simple/goal
#                         x   y   z   yaw(deg)  topic(可选)
```

---

### 4. pub_waypoint — 航点序列发布器 (演示用)

**功能**：从参数加载航点列表、循环切换、到达检测。比 mission_manger 简化，无起飞同步和录像控制。

---

### 5. convert_tf_to_odom.py — TF → Odometry 桥接

**功能**：订阅 `/tf`，将 `map→base_link` 变换转为 `nav_msgs/Odometry` 发布到 `/global_Odometry`。

---

## 自定义服务

### TakeoffNotify.srv

```
bool takeoff_done        # 请求：起飞是否完成
---
string response          # 响应：确认消息
```

**复用场景**：
- `/takeoff_notify`: px4ctrl → mission_manger (起飞完成通知)
- `/planner_fail_notify`: astar_planner → mission_manger (规划失败通知，复用 takeoff_done 字段)

---

## 典型使用流程

### 手动飞行 (仅控制)

```bash
roslaunch uav_px4_ctrl px4ctrl.launch control_mode:=position
# 手动切 RC: Offboard模式 → Arm → Hold/Command
```

### 全自主飞行

```bash
roslaunch uav_px4_ctrl uav_super_localization.launch pcd_file:=/path/to/map.pcd
# 系统自动完成: 定位 → 地图加载 → 规划 → 起飞 → 任务执行
```

### 建图模式

```bash
roslaunch uav_px4_ctrl uav_super_mapping.launch
# 仅启动定位 + 里程计，手动飞行建图
```

### 调试：单目标点发布

```bash
roslaunch uav_px4_ctrl goal_publisher.launch x:=5.0 y:=0.0 z:=1.0 yaw:=90
```

### 调试：航点循环

```bash
roslaunch uav_px4_ctrl mission_manger.launch \
    waypoints:="[[5,0,1,3]*,[10,0,1,5],[0,0,1,3]]"
```

---

## 依赖

- ROS1 (Noetic)
- Eigen3
- mavros + mavros_msgs (PX4 通信)
- quadrotor_msgs (轨迹指令消息)
- geometry_msgs, nav_msgs, sensor_msgs, std_msgs
- tf2 + tf2_geometry_msgs

## 相关包

| 包 | 关系 |
|----|------|
| `uav_planner` | 全局/局部路径规划，通过 goal_pose/position_cmd 与本包交互 |
| `uav_location` | 定位与 SLAM，提供里程计输入 |
| `uav_localization` | Cartographer 重定位，提供全局定位 |
| `fast_lio` | 激光里程计，提供局部里程计 |
| `ego_planner` | 局部避障规划，通过 position_cmd 发送轨迹指令 |
| `fast_gicp` | 点云配准 (SLAM 前端) |

## 构建

```bash
# 在 catkin workspace 根目录
catkin build uav_px4_ctrl
```
