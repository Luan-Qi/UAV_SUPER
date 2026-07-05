# CLAUDE.md — uav_px4_ctrl AI Agent 快速指南

## 项目概要

- **包名**: `uav_px4_ctrl`
- **类型**: ROS1 (Noetic) Catkin 包
- **功能**: UAV→PX4 飞控接口 — 坐标修正、多模式 Offboard 控制、安全门限、任务调度、航点管理、录像联动
- **依赖**: mavros_msgs, quadrotor_msgs, Eigen3, tf2, geometry_msgs, nav_msgs
- **硬件平台**: OrangePi (ARM) / Ubuntu, Pixhawk 飞控
- **构建产物**: 4 个 C++ 可执行文件 + 1 Python 脚本 + 1 自定义服务

## 当前迭代状态

| 日期 | 变更 |
|------|------|
| 2026-07-05 | 全文件 Doxygen 注释补全 (11 个源/launch 文件)，新增 README.md, CLAUDE.md |
| 2026-07-05 | mission_manger 航点字符串扩展：支持 wait_time 和录像标记 `*` |
| 2026-07-05 | mission_manger 新增规划器失败重试机制 (`/planner_fail_notify`，最多3次) |
| 2026-07-05 | 摄像头录像自动控制集成 (`/camera_recorder/record_control`) |

## 关键文件索引

### 核心入口 — 启动文件

| 文件 | 作用 | 启动哪些模块 |
|------|------|-------------|
| [launch/px4ctrl.launch](launch/px4ctrl.launch) | PX4 主控制节点 ★ | px4ctrl_node |
| [launch/mission_manger.launch](launch/mission_manger.launch) | 任务管理器 | mission_manger |
| [launch/goal_publisher.launch](launch/goal_publisher.launch) | CLI 单次 goal 发布 | pub_goal |
| [launch/uav_super_localization.launch](launch/uav_super_localization.launch) | 全自主飞行总启动 ★ | localization + fast_lio + octomap + astar + path_follower + ego_planner + mission_manger + px4ctrl |
| [launch/uav_super_mapping.launch](launch/uav_super_mapping.launch) | 建图模式启动 | localization(mapping) + fast_lio + tf→odom |

### PX4 控制模块 ★

| 文件 | 作用 | 关键入口 |
|------|------|----------|
| [src/px4ctrl_node.cpp](src/px4ctrl_node.cpp) | 主控制节点 — 全部核心逻辑 | `main()` 行 503, `DroneCtrl::process()` 行 268 |
| [src/px4ctrl/subscribe.h](src/px4ctrl/subscribe.h) | 5 个数据类型定义 | RC_Data_t, Odom_Data_t, State_Data_t, Battery_Data_t, Command_Data_t |
| [src/px4ctrl/subscribe.cpp](src/px4ctrl/subscribe.cpp) | 数据类型实现 (PWM归一化/状态机) | `RC_Data_t::feed()` 行 23 (状态机核心) |

**关键代码位置**：

| 功能 | 文件:行号 | 说明 |
|------|----------|------|
| 坐标修正 (ENU→NED) | [px4ctrl_node.cpp:23](src/px4ctrl_node.cpp#L23) | `uav_mavros_pose_fix()` — 绕Z+90° |
| 速度坐标修正 | [px4ctrl_node.cpp:38](src/px4ctrl_node.cpp#L38) | `uav_mavros_cmd_vel_fix()` |
| 主状态机 | [px4ctrl_node.cpp:268](src/px4ctrl_node.cpp#L268) | `DroneCtrl::process()` — arm/takeoff/hold/command/land 全流程 |
| Arm 解锁 | [px4ctrl_node.cpp:272](src/px4ctrl_node.cpp#L272) | 3秒间隔重试逻辑 |
| Hold/起飞 | [px4ctrl_node.cpp:301](src/px4ctrl_node.cpp#L301) | 首次锁定位姿 → Z=起飞高度 → 通知mission |
| 位置控制分支 | [px4ctrl_node.cpp:334](src/px4ctrl_node.cpp#L334) | case 0: update_target + publish_target |
| 速度控制分支 | [px4ctrl_node.cpp:339](src/px4ctrl_node.cpp#L339) | case 1: update_cmd_vel + publish_cmd_vel |
| 位置安全限幅 | [px4ctrl_node.cpp:393](src/px4ctrl_node.cpp#L393) | clamp + 坐标系旋转映射 (x↔y) |
| 速度安全限幅 | [px4ctrl_node.cpp:449](src/px4ctrl_node.cpp#L449) | clamp + 坐标系旋转映射 |
| RC 摇杆归一化 | [subscribe.cpp:30](src/px4ctrl/subscribe.cpp#L30) | (raw-1500)/500 + 死区处理 |
| Mode 状态机 | [subscribe.cpp:67](src/px4ctrl/subscribe.cpp#L67) | CH6: offboard 进入/退出 |
| Gear 状态机 | [subscribe.cpp:81](src/px4ctrl/subscribe.cpp#L81) | CH7: command/hold/idle 三态 |
| 参数加载 | [px4ctrl_node.cpp:80](src/px4ctrl_node.cpp#L80) | nh.param 批量读取 |
| 3 阶段启动等待 | [px4ctrl_node.cpp:513](src/px4ctrl_node.cpp#L513) | PX4连接 → RC信号 → 里程计数据 |

### 任务管理模块 ★

| 文件 | 作用 | 关键入口 |
|------|------|----------|
| [src/mission_manger.cpp](src/mission_manger.cpp) | 任务管理器 — 全部逻辑 | `main()` 行 513, `missionCheckWaypoint()` 行 205, `spin()` 行 445 |

**关键代码位置**：

| 功能 | 文件:行号 | 说明 |
|------|----------|------|
| 航点正则解析 | [mission_manger.cpp:133](src/mission_manger.cpp#L133) | `parseWaypointString()` — 支持 wait_time + `*` |
| 航点检测线程 | [mission_manger.cpp:205](src/mission_manger.cpp#L205) | `missionCheckWaypoint()` — 10Hz 独立线程 |
| 到达判定 | [mission_manger.cpp:224](src/mission_manger.cpp#L224) | 欧氏距离 < distance_threshold |
| 航点切换 | [mission_manger.cpp:238](src/mission_manger.cpp#L238) | 等待 → idx++ → cycle → publish |
| 录像逻辑 | [mission_manger.cpp:297](src/mission_manger.cpp#L297) | `processRecordingLogic()` — DISPATCH/ARRIVED 两事件 |
| 起飞同步 | [mission_manger.cpp:367](src/mission_manger.cpp#L367) | `takeoffCallback()` — 阻塞等待 /takeoff_notify |
| 规划失败重试 | [mission_manger.cpp:380](src/mission_manger.cpp#L380) | `planfailCallback()` — 最多3次 |
| 主循环 | [mission_manger.cpp:445](src/mission_manger.cpp#L445) | `spin()` — 等待起飞→延时→spin |
| 坐标变换 | [mission_manger.cpp:413](src/mission_manger.cpp#L413) | `map2odomCallback()` — map→odom 航点变换 |

### 辅助节点

| 文件 | 作用 | 特点 |
|------|------|------|
| [src/pub_goal.cpp](src/pub_goal.cpp) | CLI 单次 goal 发布 | argc/argv 解析，发布后退出 |
| [src/pub_waypoint.cpp](src/pub_waypoint.cpp) | 航点序列循环发布 | 简化版 mission_manger，无起飞同步 |

### 脚本与工具

| 文件 | 作用 | 说明 |
|------|------|------|
| [script/convert_tf_to_odom.py](script/convert_tf_to_odom.py) | TF → Odometry 桥接 | map→base_link → /global_Odometry |
| [srv/TakeoffNotify.srv](srv/TakeoffNotify.srv) | 起飞通知服务定义 | `bool takeoff_done → string response`，也复用为规划失败通知 |
| [rviz/demo_3d.rviz](rviz/demo_3d.rviz) | RViz 可视化配置 | 全自主飞行默认配置 |

### 构建配置

| 文件 | 说明 |
|------|------|
| [CMakeLists.txt](CMakeLists.txt) | 4 个可执行文件: px4ctrl_node, pub_goal, waypoint_publisher, mission_manger |
| [package.xml](package.xml) | 依赖: mavros_msgs, quadrotor_msgs, tf2, Eigen3 |

## 关键数据流

```
                        ┌─────────────────────────────────────┐
  RC遥控器              │           px4ctrl_node              │
  /mavros/rc/in ───────→│  rc_data (arm/mode/gear状态机)      │
                         │                                    │
  里程计                 │  odom_data → uav_mavros_pose_fix() │
  /Odometry ───────────→│         → /mavros/vision_pose/pose │
                         │                                    │
  轨迹指令               │  cmd_data → update_target()        │
  /position_cmd ───────→│         → /mavros/setpoint_*        │
                         │                                    │
                         │  DroneCtrl::process() 主状态机      │
                         │  arm→takeoff→hold→command→land     │
                         └──────────────┬──────────────────────┘
                                        │ /takeoff_notify (srv)
                                        ▼
                         ┌─────────────────────────────────────┐
                         │         mission_manger              │
  /localization ────────→│  current_pose_                     │
  or /mavros/pose         │                                    │
                         │  missionCheckWaypoint() (独立线程)  │
                         │    到达检测 → 等待 → 切换航点       │
                         │                                    │
                         │  processRecordingLogic()            │
                         │    DISPATCH → 开始录像              │
                         │    ARRIVED → 停止录像               │
                         │         │                          │
                         │         └→ /camera_recorder/        │
                         │            record_control           │
                         │                                    │
                         │  planfailCallback()                 │
                         │    收到规划失败 → 重试(最多3次)     │
                         └──────────────┬──────────────────────┘
                                        │
                          ┌─────────────┴─────────────┐
                          ▼                           ▼
                     /start_pose                  /goal_pose
                     (规划器起点)                  (规划器目标)
                          │                           │
                          └───────────┬───────────────┘
                                      ▼
                              uav_planner
                        (全局/局部路径规划)
```

## 消息与话题约定

| 话题 | 类型 | 方向 | 用途 |
|------|------|------|------|
| `/mavros/state` | `mavros_msgs::State` | 订阅 | PX4 飞控状态 (connected/armed/mode) |
| `/mavros/rc/in` | `mavros_msgs::RCIn` | 订阅 | RC 遥控器 8 通道原始 PWM |
| `/mavros/battery` | `sensor_msgs::BatteryState` | 订阅 | 电池电压/电流 |
| `/position_cmd` | `quadrotor_msgs::PositionCommand` | 订阅 | 外部轨迹规划器全状态指令 |
| `<odom_topic>` | `nav_msgs::Odometry` | 订阅 | 里程计位姿/速度 (默认 /Odometry) |
| `/mavros/vision_pose/pose` | `geometry_msgs::PoseStamped` | 发布 | 视觉位姿输入 PX4 (坐标修正后) |
| `/mavros/setpoint_position/local` | `geometry_msgs::PoseStamped` | 发布 | PX4 offboard 位置指令 |
| `/mavros/setpoint_velocity/cmd_vel` | `geometry_msgs::TwistStamped` | 发布 | PX4 offboard 速度指令 |
| `/traj_start_trigger` | `geometry_msgs::PoseStamped` | 发布 | 轨迹启动触发 (预留) |
| `/start_pose` | `geometry_msgs::PoseStamped` | 发布 | 规划器起点 (当前位姿) |
| `/goal_pose` 或 `/move_base_simple/goal` | `geometry_msgs::PoseStamped` | 发布 | 规划器目标航点 |
| `/camera_recorder/record_control` | `std_msgs::String` | 发布 | 录像起停 ("start"/"stop") |
| `/global_Odometry` | `nav_msgs::Odometry` | 发布 | TF 桥接输出的全局里程计 |
| `/takeoff_notify` | `uav_px4_ctrl::TakeoffNotify` | 服务 | px4ctrl → mission_manger 起飞完成 |
| `/planner_fail_notify` | `uav_px4_ctrl::TakeoffNotify` | 服务 | 规划器 → mission_manger 规划失败 |

## RC 通道映射与状态机

### 通道映射 (日本手)

| 通道 | 功能 | 归一化 | 用途 |
|------|------|--------|------|
| CH1 | Roll | (raw-1500)/500 → [-1,1] | 横滚 |
| CH2 | Pitch | (raw-1500)/500 → [-1,1] | 俯仰 |
| CH3 | Throttle | (raw-1500)/500 → [-1,1] | 油门 |
| CH4 | Yaw | (raw-1500)/500 → [-1,1] | 偏航 |
| CH5 | Arm 开关 | PWM > 1750 = Armed | 解锁/上锁 |
| CH6 | Mode 开关 | PWM > 1750 = Offboard | 飞行模式 |
| CH7 | Gear 开关 | >1750=Command; 1250~1750=Hold; <1250=Idle | 控制模式 |

### Gear 状态机

```
CH7 PWM > 1750  ──→ is_command_mode = true  (执行轨迹指令)
        1250~1750 → is_hold_mode = true     (悬停保持)
        < 1250    → idle                    (降落/空闲)
```

## 修改指南

### 修改 PX4 控制行为

- 坐标修正: [px4ctrl_node.cpp:23](src/px4ctrl_node.cpp#L23) `uav_mavros_pose_fix()` — 修改旋转角度/方向
- 主状态机: [px4ctrl_node.cpp:268](src/px4ctrl_node.cpp#L268) `DroneCtrl::process()` — 修改 arm/takeoff/land 流程
- 安全限幅: [px4ctrl_node.cpp:381](src/px4ctrl_node.cpp#L381) `publish_target()` / `publish_cmd_vel()` — 修改限幅值或策略
- 添加新控制模式: [px4ctrl_node.cpp:332](src/px4ctrl_node.cpp#L332) switch(control_status) — 添加 case 3/4/...

### 修改 RC 状态机

- 阈值常量: [subscribe.h:37](src/px4ctrl/subscribe.h#L37) ARM_THRESHOLD_VALUE 等
- 死区: [subscribe.h:41](src/px4ctrl/subscribe.h#L41) DEAD_ZONE
- 状态机逻辑: [subscribe.cpp:23](src/px4ctrl/subscribe.cpp#L23) `RC_Data_t::feed()`

### 修改任务管理行为

- 航点格式: [mission_manger.cpp:133](src/mission_manger.cpp#L133) `parseWaypointString()` — 修改正则匹配
- 到达判定: [mission_manger.cpp:224](src/mission_manger.cpp#L224) distance_threshold
- 录像逻辑: [mission_manger.cpp:297](src/mission_manger.cpp#L297) `processRecordingLogic()` — 修改录像起停规则
- 规划器重试次数: [mission_manger.cpp:385](src/mission_manger.cpp#L385) `current_wp_retry_times_ >= 3`

### 添加新节点

1. 在 `src/` 创建 `.cpp` 文件
2. 在 [CMakeLists.txt](CMakeLists.txt) 添加 `add_executable` + `target_link_libraries` + `add_dependencies`
3. 在 `launch/` 创建对应 `.launch` 文件
4. 如需自定义消息/服务，在 `srv/` 添加 `.srv` 并在 CMakeLists.txt 注册

### 添加新参数

1. 在 `launch/*.launch` 的 `<arg>` 和 `<param>` 中添加
2. 在对应 C++ 节点的构造函数中添加 `nh.param(...)` 读取
3. 在成员变量区添加带 `///<` 注释的变量声明

## 构建命令

```bash
# 在 catkin workspace 根目录
catkin build uav_px4_ctrl

# 或单独编译
catkin_make --pkg uav_px4_ctrl
```

## 运行时检查清单

1. PX4 飞控已连接 (MAVROS `/mavros/state` 中 `connected=true`)
2. RC 遥控器已开启 (`manual_input=true`)
3. 里程计话题正常发布 (检查 `/Odometry` 频率)
4. 电池电压正常 (> `battery_limit`)
5. 如使用全局规划，确保 PCD 地图路径正确

## 相关包

| 包 | 关系 | 关键接口 |
|----|------|----------|
| `uav_planner` | 全局/局部路径规划 | 通过 `/goal_pose`, `/position_cmd` 交互 |
| `uav_location` | 定位与 SLAM | 提供 `/Odometry` 里程计输入 |
| `uav_localization` | Cartographer 重定位 | 提供 TF (map→base_link) |
| `fast_lio` | 激光里程计 | 提供局部里程计 |
| `ego_planner` | 局部避障规划 | 通过 `/position_cmd` 发送轨迹指令 |
