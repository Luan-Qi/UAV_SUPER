# UAV_SUPER
Self summarized and prepared for UAV auto drive system


#### 部分组件修改说明：  
1.(livox_ros_driver2): Take the lidar extrinsics into account when processing IMU data.  
2.(egoplanner):  Unlock egoplanner z limmit at z=1.0.  
3.(livox_ros_driver2): fix driver's output data type (<arg name="output_type" default="0"/>).  
4.(fast_lio): Use faslio lidar input No.4 for normal cloudpoint (XYZI).  

## 基础功能
### 硬件驱动
*使用飞行和建图模式之前请先打开驱动，并确保驱动正常运行*
* 连接PX4飞控
* 启动双目相机驱动
* 启动MID360雷达驱动
```
bash ./shfile/uav_super_interfance.sh
```
或者
```
roslaunch mavros px4.launch & sleep 4;
roslaunch orbbec_camera gemini_330_series.launch & sleep 3;
roslaunch livox_ros_driver2 msg_MID360.launch & sleep 2;
```

### 手动飞行建图模式
启动之后无人机可以使用定点模式完成悬停，使用RC控制器手动控制飞行，完成飞行后手动降落关闭节点即可得到地图。**及时修改文件名以免覆盖**
```
bash ./shfile/uav_super_mapping.sh
```
或者
```
roslaunch uav_location global_mapping.launch & sleep 2;

echo "starting camera and wainting 10 seconds...";
roslaunch uav_util simple_camera.launch  & sleep 10;

echo "start recording...";
rosbag record -O /mnt/nvme/rosbag/scans.bag /livox/imu /livox/lidar /simple_camera/image_compressed
```

### 仿真自主飞行模式
提前在launch文件中设置好目标点的位置、停留时间、录像控制等参数。
启动节点之后，等待无人机完成重定位，如果无法立即完成请手动晃动无人机。
无人机完成定位之后，手动切换到Offboard模式，并且切换自动飞行按钮到高位，等待RC主动解锁，然后无人机开始自动飞行。
**发生意外需要及时手动控制**。
```
roslaunch uav_location global_planner_in_sim.launch;
```

### 自动飞行模式
```
bash ./shfile/uav_super_main.sh
```
或者
```
roslaunch uav_location global_planner.launch;
```

## 目录结构
```
UAV_SUPER
   ├─build                    -- ROS编译目录
   ├─devel                    -- ROS编译依赖库
   ├─shfile                   -- shell脚本目录
   └─src                      -- ROS源代码目录
      ├─fast_gicp             -- 快速ICP算法库
      ├─quadrotor_msgs        -- 四旋翼消息定义库
      ├─uav_localization      -- cartographer定位库
      │    ├─config                    -- cartographer配置文件
      │    ├─include                   -- cpp头文件
      │    ├─launch                    -- cartographer启动器
      │    │  ├─localization_mid360    -- cartographer重定位模式
      │    │  └─mapping_mid360         -- cartographer建图模式
      │    ├─script                    -- python脚本
      │    ├─src                       -- cpp源代码
      │    └─urdf                      -- 四旋翼URDF模型
      ├─uav_location          -- UAV定位算法库
      │    ├─include                   -- cpp头文件
      │    ├─launch                    -- UAV_SUPER启动文件
      │    │  ├─global_mapping         -- UAV_SUPER建图模式
      │    │  ├─global_planner_in_sim  -- UAV_SUPER仿真自主飞行模式
      │    │  ├─global_planner         -- UAV_SUPER自主飞行模式
      │    │  ├─icp_registration       -- icp重定位算法测试
      │    │  └─local_planner          -- UAV_SUPER局部路径规划模式
      │    ├─rviz                      -- rviz配置文件
      │    └─src                       -- cpp源代码
      │       ├─icp_registration       -- icp重定位算法测试（未测试）
      │       ├─global_localization    -- UAV_SUPER全局定位算法
      │       ├─odom_republisher       -- odom消息转发（未使用）
      │       ├─publish_initial_pose   -- UAV_SUPER初始位置估计
      │       └─transform_fusion       -- UAV_SUPER全局坐标融合发布
      ├─uav_planner           -- UAV路径规划算法库
      │    ├─include                   -- cpp头文件
      │    ├─launch                    -- 算法测试启动文件
      │    │  ├─astar_planner          -- A*算法测试
      │    │  ├─octomap_publisher      -- 转换PCD地图到octomap
      │    │  ├─planner_path_simplify  -- 路径简化算法测试
      │    │  └─planner_path_smoothify -- 路径平滑算法测试
      │    ├─rviz                      -- rviz配置文件
      │    └─src                       -- cpp源代码
      │       ├─astar                  -- A*算法
      │       ├─path_simplify          -- 路径简化算法
      │       ├─path_smoothify         -- 路径平滑算法
      │       ├─simple_path_follower   -- 简易轨迹跟踪算法
      │       └─slp_planner            -- single_lidar_planner算法（自研）
      ├─uav_px4_ctrl          -- UAV->PX4控制算法库
      │    ├─include                   -- cpp头文件
      │    ├─launch                    -- cpp头文件
      │    │  ├─goal_publisher         -- 目标点发布测试
      │    │  ├─mission_manger         -- 多目标点发布器测试
      │    │  ├─px4ctrl                -- UAV_SUPER->px4控制器节点
      │    │  ├─uav_super_localization -- 基于cartographer的UAV_SUPER自主飞行模式
      │    │  ├─uav_super_mapping      -- 基于cartographer的UAV_SUPER建图模式
      │    │  ├─waypoint_*_in_sim      -- 仿真环境发布多目标点
      │    │  └─waypoint_publisher     -- 局部定位的多目标点测试（演示用）
      │    ├─rviz                      -- 基于cartographer的UAV_SUPER的rviz配置文件
      │    ├─script                    -- python脚本
      │    │  └─convert_tf_to_odom     -- 转换cartographer坐标到可用odom
      │    ├─src                       -- cpp源代码
      │    └─srv                       -- px4_ctrl自定义消息
      ├─uav_px4_ctrl_test     -- PX4控制算法测试库
      │    ├─include
      │    └─launch/src
      │       ├─uav_super_px4_livox    -- launch启动硬件节点
      │       ├─odometry_to_px4        -- 手动发送odom给px4
      │       ├─px4_pos_*              -- px4位置控制测试
      │       └─px4_vel_*              -- px4速度控制测试
      ├─uav_tracker           -- UAV跟踪算法库
      │    ├─cfg                       -- 颜色跟踪配置文件
      │    ├─include                   -- KCF跟踪算法头文件
      │    ├─launch                    -- 
      │    │  ├─AstraTracker           -- 颜色跟踪算法启动器
      │    │  └─KCFTracker             -- KCF跟踪算法启动器
      │    ├─script                    -- 
      │    │  └─astra_Tracker          -- 颜色跟踪算法库
      │    └─src                       --
      │       └─KCF_Tracker            -- KCF跟踪算法库
      └─uav_util              -- UAV工具库
           ├─include
           └─launch/src
              ├─generate_pcd_map       -- PCD点云地图生成器
              ├─merge_cloud            -- livox系列点云合并（未测试）
              ├─pcd_downsample         -- PCD点云下采样工具
              └─simple_camera          -- 基于OPENCV的简易相机驱动
```