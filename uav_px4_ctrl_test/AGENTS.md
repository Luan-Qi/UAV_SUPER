# AGENTS.md — uav_px4_ctrl_test AI Agent 快速指南

## 项目概要

- **包名**: `uav_px4_ctrl_test`
- **类型**: ROS1 (Noetic) Catkin 测试包
- **功能**: PX4 Offboard 控制算法的独立测试与验证 — 从简单起飞到综合 RC 状态机的渐进式测试节点集合
- **依赖**: mavros_msgs, quadrotor_msgs, tf2, geometry_msgs, nav_msgs, sensor_msgs
- **硬件平台**: OrangePi (ARM) / Ubuntu, Pixhawk 飞控, Livox MID360
- **构建产物**: 6 个 C++ 可执行文件 + 2 个 launch 文件
- **代码共享**: subscribe.h/cpp 来自 [uav_px4_ctrl](../uav_px4_ctrl/) 包

## 与 uav_px4_ctrl 的关系

本包是 `uav_px4_ctrl` 的测试与验证配套包：
- `uav_px4_ctrl` — 生产级控制节点 (px4ctrl_node + mission_manger)
- `uav_px4_ctrl_test` — 测试/验证/实验节点 (6 个独立测试程序)
- 共享 `subscribe.h/cpp` 数据类型
- `px4_vel_simple_test` 是 `px4ctrl_node` 的简化测试版本

## 当前迭代状态

| 日期 | 变更 |
|------|------|
| 2026-07-05 | 全文件 Doxygen 注释补全 (6 cpp + 2 launch)，新增 README.md, CLAUDE.md |

## 关键文件索引

### 启动文件

| 文件 | 作用 | 启动哪些模块 |
|------|------|-------------|
| [launch/px4_vel_simple_test.launch](launch/px4_vel_simple_test.launch) | PX4 综合控制测试 | fast_lio (mapping_mid360) + px4_vel_simple_test |
| [launch/uav_super_px4_livox.launch](launch/uav_super_px4_livox.launch) | 传感器驱动启动 | mavros (px4.launch) + livox_ros_driver2 (MID360) |

### 测试节点（按复杂度递增）

| 文件 | 可执行文件 | 控制模式 | 代码行数 | 核心功能 |
|------|-----------|---------|---------|---------|
| [src/odometry_to_px4.cpp](src/odometry_to_px4.cpp) | `odometry_to_px4_node` | — | ~80 | 里程计→PX4 位姿桥接 |
| [src/px4_pos_takeoff.cpp](src/px4_pos_takeoff.cpp) | `px4_takeoff` | 位置 | ~140 | 定高起飞悬停 |
| [src/px4_pos_up-and-down.cpp](src/px4_pos_up-and-down.cpp) | `px4_up-and-down` | 位置 | ~160 | 定时往复升降 |
| [src/px4_pos_rectangle.cpp](src/px4_pos_rectangle.cpp) | `px4_rectangle` | 位置 | ~200 | 矩形航线 + 到达判定 |
| [src/px4_vel_rectangle.cpp](src/px4_vel_rectangle.cpp) | `px4_vel_rectangle` | 速度 | ~260 | PID 控制 + 6 步状态机 + 自动降落 |
| [src/px4_vel_simple_test.cpp](src/px4_vel_simple_test.cpp) | `px4_vel_simple_test` | 位置/速度 | ~470 | RC 状态机 + 双模式 + 电池安全 |

### 关键代码位置

#### 坐标变换（复用函数，出现在多个文件中）

| 函数 | 文件:行号 (以 px4_vel_simple_test.cpp 为准) | 说明 |
|------|----------|------|
| `uav_mavros_pose_fix()` | [px4_vel_simple_test.cpp:21](src/px4_vel_simple_test.cpp#L21) | ENU→NED 位姿修正: x'=-y, y'=x, yaw+=90° |
| `uav_mavros_cmd_vel_fix()` | [px4_vel_simple_test.cpp:37](src/px4_vel_simple_test.cpp#L37) | ENU→NED 速度修正 (仅线速度 XY) |
| `yawToQuaternion()` | [px4_vel_simple_test.cpp:46](src/px4_vel_simple_test.cpp#L46) | 偏航角→四元数 |

#### odometry_to_px4 — 里程计桥接

| 功能 | 行号 | 说明 |
|------|------|------|
| 构造函数 | [odometry_to_px4.cpp:24](src/odometry_to_px4.cpp#L24) | 订阅 /Odometry，发布到 /mavros/vision_pose/pose |
| 坐标修正 | [odometry_to_px4.cpp:40](src/odometry_to_px4.cpp#L40) | ENU→NED 位置+姿态变换 |

#### px4_pos_takeoff — 定高起飞

| 功能 | 行号 | 说明 |
|------|------|------|
| 目标位姿 | [px4_pos_takeoff.cpp:113](src/px4_pos_takeoff.cpp#L113) | z=1m, 无旋转 |
| 预发送 setpoint | [px4_pos_takeoff.cpp:125](src/px4_pos_takeoff.cpp#L125) | 20 帧预发送 (Offboard 切换前提) |
| Offboard 切换 | [px4_pos_takeoff.cpp:132](src/px4_pos_takeoff.cpp#L132) | SetMode service call |
| 解锁 | [px4_pos_takeoff.cpp:148](src/px4_pos_takeoff.cpp#L148) | CommandBool arm=true |
| 悬停循环 | [px4_pos_takeoff.cpp:163](src/px4_pos_takeoff.cpp#L163) | 持续发布目标位置，监控模式 |

#### px4_pos_rectangle — 矩形航线 (位置控制)

| 功能 | 行号 | 说明 |
|------|------|------|
| 航点定义 | [px4_pos_rectangle.cpp:121](src/px4_pos_rectangle.cpp#L121) | 4 顶点 + z_height=1m + side=1m |
| 到达判定 | [px4_pos_rectangle.cpp:88](src/px4_pos_rectangle.cpp#L88) | 3D 欧氏距离 < 0.1m |
| 悬停等待 | [px4_pos_rectangle.cpp:185](src/px4_pos_rectangle.cpp#L185) | 3 秒 hold |
| 航点切换 | [px4_pos_rectangle.cpp:196](src/px4_pos_rectangle.cpp#L196) | 循环索引 (idx+1) % N |

#### px4_pos_up-and-down — 往复升降

| 功能 | 行号 | 说明 |
|------|------|------|
| 双航点定义 | [px4_pos_up-and-down.cpp:119](src/px4_pos_up-and-down.cpp#L119) | z=0.5m 和 z=1.0m |
| 定时切换 | [px4_pos_up-and-down.cpp:157](src/px4_pos_up-and-down.cpp#L157) | 每 5 秒 target_idx = 1 - target_idx |

#### px4_vel_rectangle — 矩形航线 (速度控制)

| 功能 | 行号 | 说明 |
|------|------|------|
| PID 控制器 | [px4_vel_rectangle.cpp:79](src/px4_vel_rectangle.cpp#L79) | `local_pos_control()` — kp=2.0, ki=0.01, kd=0.5 |
| PID 参数 | [px4_vel_rectangle.cpp:65](src/px4_vel_rectangle.cpp#L65) | 全局变量区，含调参记录 |
| 状态机步骤 | [px4_vel_rectangle.cpp:157](src/px4_vel_rectangle.cpp#L157) | case 0→5 (上升→前进→右移→后退→左移→AUTO.LAND) |
| 到达判定 | [px4_vel_rectangle.cpp:168](src/px4_vel_rectangle.cpp#L168) | 连续 20 帧在容差窗口内 |
| 自动降落 | [px4_vel_rectangle.cpp:230](src/px4_vel_rectangle.cpp#L230) | case 5: 切换到 AUTO.LAND |
| 已知缺陷 | [px4_vel_rectangle.cpp:88](src/px4_vel_rectangle.cpp#L88) | Z 轴 PID 微分项误用了 err_y_err (应为 err_z_err) |

#### px4_vel_simple_test — 综合控制 (★ 最复杂)

| 功能 | 行号 | 说明 |
|------|------|------|
| DroneCtrl 类定义 | [px4_vel_simple_test.cpp:106](src/px4_vel_simple_test.cpp#L106) | 全部控制逻辑封装 |
| 构造函数 | [px4_vel_simple_test.cpp:141](src/px4_vel_simple_test.cpp#L141) | boost::bind 绑定 5 个数据结构的 feed() |
| 里程计回调 | [px4_vel_simple_test.cpp:169](src/px4_vel_simple_test.cpp#L169) | odom → 坐标修正 → vision_pose |
| 起飞初始化 | [px4_vel_simple_test.cpp:183](src/px4_vel_simple_test.cpp#L183) | z=0.5m 默认 |
| 更新期望位置 | [px4_vel_simple_test.cpp:188](src/px4_vel_simple_test.cpp#L188) | 从 odom 提取 + 坐标修正 |
| 更新期望速度 | [px4_vel_simple_test.cpp:200](src/px4_vel_simple_test.cpp#L200) | 从 PositionCommand 提取 + 坐标修正 |
| **主状态机** | [px4_vel_simple_test.cpp:211](src/px4_vel_simple_test.cpp#L211) | `process()` — arm → mode route → hold/command |
| 解锁逻辑 | [px4_vel_simple_test.cpp:216](src/px4_vel_simple_test.cpp#L216) | 3s 间隔重试，需 is_armed && is_offboard && battery && stick |
| 悬停模式 | [px4_vel_simple_test.cpp:247](src/px4_vel_simple_test.cpp#L247) | 首次锁定位姿 → Z=起飞高度 |
| 指令序列 | [px4_vel_simple_test.cpp:271](src/px4_vel_simple_test.cpp#L271) | 6 阶段 × 3s: vx=±0.5, vy=±0.5, vz=±0.1 m/s |
| 位置发布 | [px4_vel_simple_test.cpp:389](src/px4_vel_simple_test.cpp#L389) | `publish_target()` — 带坐标旋转限幅 |
| 速度发布 | [px4_vel_simple_test.cpp:408](src/px4_vel_simple_test.cpp#L408) | `publish_cmd_vel()` — 带速度限幅 |
| 电池检测 | [px4_vel_simple_test.cpp:378](src/px4_vel_simple_test.cpp#L378) | voltage < 19.8V → 禁止解锁 |
| 三阶段启动 | [px4_vel_simple_test.cpp:441](src/px4_vel_simple_test.cpp#L441) | PX4连接 → RC信号 → Odom数据 |
| 主循环 | [px4_vel_simple_test.cpp:476](src/px4_vel_simple_test.cpp#L476) | 30 Hz, node.process() |

## 关键数据流

```
                          ┌──────────────────────────────────────────┐
  Livox MID360            │             fast_lio                     │
  /livox/lidar ──────────→│  mapping_mid360.launch                   │
                          │          ↓                               │
                          │     /Odometry (ENU 系)                   │
                          └──────────────┬───────────────────────────┘
                                         │
                    ┌────────────────────┼───────────────────────┐
                    │                    ↓                       │
                    │  ┌─────────────────────────────────────┐   │
                    │  │   odometry_to_px4_node              │   │
                    │  │   uav_mavros_pose_fix (ENU→NED)    │   │
                    │  │        ↓                            │   │
                    │  │   /mavros/vision_pose/pose          │   │
                    │  └─────────────────────────────────────┘   │
                    │                                            │
                    │  ┌─────────────────────────────────────┐   │
                    │  │   px4_vel_simple_test               │   │
                    │  │                                     │   │
  RC 遥控器         │  │  rc_data ──→ process() 主状态机     │   │
  /mavros/rc/in ────┼──→│    ├── hold_mode: publish_target()  │   │
                    │  │    │     └→ setpoint_position/local  │   │
                    │  │    └── command_mode: publish_cmd_vel()│  │
                    │  │         └→ setpoint_velocity/cmd_vel │   │
                    │  │                                     │   │
                    │  │  safety: battery_limit=19.8V         │   │
                    │  │         position/velocity clamp      │   │
                    │  └─────────────────────────────────────┘   │
                    │                                            │
                    │  ┌─────────────────────────────────────┐   │
                    │  │   px4_pos_* / px4_vel_rectangle     │   │
                    │  │   独立测试节点 (不依赖RC)            │   │
                    │  │   直接调用 mavros 服务/话题          │   │
                    │  └─────────────────────────────────────┘   │
                    └────────────────────────────────────────────┘
```

## 消息与话题约定

### 公共话题 (所有节点共享)

| 话题 | 类型 | 方向 | 用途 |
|------|------|------|------|
| `/mavros/state` | `mavros_msgs::State` | 订阅 | PX4 飞控状态 (connected/armed/mode) |
| `/mavros/setpoint_position/local` | `geometry_msgs::PoseStamped` | 发布 | Offboard 位置指令 (NED 系) |
| `/mavros/cmd/arming` | `mavros_msgs::CommandBool` | 服务 | 解锁/上锁 |
| `/mavros/set_mode` | `mavros_msgs::SetMode` | 服务 | 飞行模式切换 |

### px4_vel_simple_test 额外话题

| 话题 | 类型 | 方向 | 用途 |
|------|------|------|------|
| `/Odometry` | `nav_msgs::Odometry` | 订阅 | 里程计位姿/速度 (ENU 系) |
| `/mavros/rc/in` | `mavros_msgs::RCIn` | 订阅 | RC 遥控器 8 通道 PWM |
| `/mavros/battery` | `sensor_msgs::BatteryState` | 订阅 | 电池电压/电流 |
| `/position_cmd` | `quadrotor_msgs::PositionCommand` | 订阅 | 外部轨迹规划器指令 |
| `/mavros/vision_pose/pose` | `geometry_msgs::PoseStamped` | 发布 | 视觉位姿输入 PX4 (坐标修正后) |
| `/mavros/setpoint_velocity/cmd_vel` | `geometry_msgs::TwistStamped` | 发布 | Offboard 速度指令 (NED 系) |
| `/traj_start_trigger` | `geometry_msgs::PoseStamped` | 发布 | 轨迹启动触发 (预留) |

### px4_vel_rectangle 额外话题

| 话题 | 类型 | 方向 | 用途 |
|------|------|------|------|
| `/mavros/local_position/pose` | `geometry_msgs::PoseStamped` | 订阅 | PX4 本地位置估计 (反馈环) |
| `/mavros/setpoint_velocity/cmd_vel_unstamped` | `geometry_msgs::Twist` | 发布 | 速度指令 (无时间戳版本) |

### px4_pos_rectangle 额外话题

| 话题 | 类型 | 方向 | 用途 |
|------|------|------|------|
| `/mavros/local_position/pose` | `geometry_msgs::PoseStamped` | 订阅 | PX4 本地位置估计 (到达判定) |

## 控制模式对比

| 特性 | 位置控制 (setpoint_position) | 速度控制 (setpoint_velocity) |
|------|---------------------------|---------------------------|
| 使用节点 | takeoff, rectangle, up-and-down | vel_rectangle, vel_simple_test |
| 控制方式 | 期望位置 → PX4 内部位置环 | 期望速度 → PX4 内部速度环 |
| 优点 | 简单，PX4 处理姿态/速度环 | 灵活，可自定义控制律 |
| 缺点 | 无法自定义速度曲线 | 需要外部位置闭环 |
| 坐标修正 | ENU→NED | 无修正 (NED 原生) 或 ENU→NED |
| 典型场景 | 简单测试、航点飞行 | PID 控制、外部轨迹规划 |

## RC 通道映射 (px4_vel_simple_test)

### 通道配置 (日本手)

| 通道 | 功能 | 归一化 | 阈值 |
|------|------|--------|------|
| CH1 | Roll | (raw-1500)/500 → [-1,1] | 死区 ±0.25 |
| CH2 | Pitch | (raw-1500)/500 → [-1,1] | 死区 ±0.25 |
| CH3 | Throttle | (raw-1500)/500 → [-1,1] | 死区 ±0.25 |
| CH4 | Yaw | (raw-1500)/500 → [-1,1] | 死区 ±0.25 |
| CH5 | Arm 开关 | PWM > 1750 = Armed | `ARM_THRESHOLD_VALUE` |
| CH6 | Offboard 模式 | PWM > 1750 = Offboard | `API_MODE_THRESHOLD_VALUE` |
| CH7 | Gear/模式 | >1750=Command; 1250~1750=Hold; <1250=Idle | 双阈值 |

### 操作流程

```
1. 遥控器上电 → CH5 LOW, CH6 LOW, CH7 MID
2. 启动 px4_vel_simple_test.launch
3. 等待 [PX4CTRL][1/3], [2/3], [3/3] 全部就绪
4. CH6 > 1750 → 进入 Offboard 模式
5. CH5 > 1750 → 自动解锁 (3s 内)
   ├── CH7 MID (1250~1750) → 悬停 + 自动起飞到 0.5m
   └── CH7 HIGH (>1750)   → 执行 6 阶段速度指令序列
       ├── Stage 1-2: X±0.5 m/s
       ├── Stage 3-4: Y±0.5 m/s
       ├── Stage 5-6: Z±0.1 m/s
       └── 序列完成 → cmd_finish=true (不再重复)
6. CH5 LOW → 上锁降落
```

## 修改指南

### 修改起飞行为

- 起飞高度: 修改目标位姿的 `pose.position.z` 值
  - [px4_pos_takeoff.cpp:113](src/px4_pos_takeoff.cpp#L113) — `pose.pose.position.z = 1`
  - [px4_vel_simple_test.cpp:138](src/px4_vel_simple_test.cpp#L138) — `takeoff_height = 0.5`

### 修改坐标变换

- ENU→NED 变换: `uav_mavros_pose_fix()` — 修改旋转角度 (当前 +90°)
  - [px4_vel_simple_test.cpp:21](src/px4_vel_simple_test.cpp#L21)
- 速度变换: `uav_mavros_cmd_vel_fix()` — 修改线性映射

### 修改 PID 参数 (px4_vel_rectangle)

- [px4_vel_rectangle.cpp:65](src/px4_vel_rectangle.cpp#L65) — `kp`, `ki`, `kd` 全局变量
- 调参记录已写在注释中

### 修改指令序列 (px4_vel_simple_test)

- 阶段数量/顺序: [px4_vel_simple_test.cpp:271](src/px4_vel_simple_test.cpp#L271) — switch(cmd_stage) 分支
- 每阶段持续时间: `dt < 3.0` → 修改为其他秒数
- 速度幅值: `cmd.velocity.x = 0.5` → 修改速度值 [m/s]

### 修改安全门限

- 电池阈值: [px4_vel_simple_test.cpp:139](src/px4_vel_simple_test.cpp#L139) — `battery_limit = 19.8`
- 位置限幅: [px4_vel_simple_test.cpp:143](src/px4_vel_simple_test.cpp#L143) — `position_max/min_*`
- 速度限幅: [px4_vel_simple_test.cpp:150](src/px4_vel_simple_test.cpp#L150) — `cmd_vel_max_*`

### 添加新测试节点

1. 在 [src/](src/) 创建 `px4_xxx.cpp` 文件
2. 从现有简单节点 (如 px4_pos_takeoff.cpp) 复制模板
3. 在 [CMakeLists.txt](CMakeLists.txt) 添加:
   ```cmake
   add_executable(px4_xxx src/px4_xxx.cpp)
   target_link_libraries(px4_xxx ${catkin_LIBRARIES})
   add_dependencies(px4_xxx ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
   ```
4. 如需使用 subscribe.h，添加 `../uav_px4_ctrl/src/px4ctrl/subscribe.cpp` 到源文件列表

## 构建命令

```bash
# 在 catkin workspace 根目录
catkin build uav_px4_ctrl_test

# 或单独编译
catkin_make --pkg uav_px4_ctrl_test
```

## 运行时检查清单

1. ✅ PX4 飞控已连接 (`/mavros/state` connected=true)
2. ✅ 如使用 fast_lio，Livox MID360 已上电
3. ✅ 如使用 px4_vel_simple_test，RC 遥控器已开启
4. ✅ 里程计话题正常发布 (检查 `/Odometry` 频率)
5. ✅ 如使用 px4_vel_simple_test，电池电压正常 (> `battery_limit`)
6. ✅ 首次飞行建议在仿真环境 (SITL) 测试

## 已知问题

| 问题 | 文件 | 说明 |
|------|------|------|
| Z 轴 PID 微分项 bug | [px4_vel_rectangle.cpp:88](src/px4_vel_rectangle.cpp#L88) | `vel_z` 微分项误用 `err_y_err` 而非 `err_z_err` |
| PID 积分未限幅 | [px4_vel_rectangle.cpp](src/px4_vel_rectangle.cpp) | `sum_ez` 等积分项无抗饱和 (anti-windup) 限幅 |
| 指令序列不可重复 | [px4_vel_simple_test.cpp](src/px4_vel_simple_test.cpp) | `cmd_finish=true` 后不再执行，需重新进入模式才能重置 |
| Z 轴坐标限幅逻辑 | [px4_vel_simple_test.cpp:403](src/px4_vel_simple_test.cpp#L403) | `position_min/max_z` 未经过坐标旋转交叉映射（但 Z 轴不变） |

## 相关包

| 包 | 关系 | 关键接口 |
|----|------|----------|
| `uav_px4_ctrl` | 数据类型来源 | subscribe.h/cpp 共享 (RC_Data_t, Odom_Data_t 等) |
| `uav_location` | 里程计源 | /Odometry (fast_ofio / Fast-LIO) |
| `fast_lio` | 激光里程计 | mapping_mid360.launch → /Odometry |
| `livox_ros_driver2` | 激光雷达驱动 | msg_MID360.launch → /livox/lidar |
| `mavros` | PX4 通信桥 | /mavros/* 话题与服务 |
| `uav_planner` | 轨迹指令源 | /position_cmd (仅 px4_vel_simple_test 使用) |
