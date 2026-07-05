/**
 * @file transform_fusion.cpp
 * @brief 坐标融合节点 — 将全局定位 (map→odom) 与局部里程计 (odom→base) 融合为全局位姿。
 *
 * @details
 * 功能：
 *   1. 订阅局部里程计 /Odometry (odom → base_link)
 *   2. 订阅全局定位结果 /map_to_odom (map → odom)
 *   3. 融合计算 T_map_to_base = T_map_to_odom × T_odom_to_base
 *   4. 发布 /localization (nav_msgs::Odometry, map → base_link)
 *   5. 广播 TF: map → camera_init
 *
 * 坐标系关系：
 *   map (全局固定)
 *    └── odom (里程计漂移系) ── /map_to_odom 提供
 *         └── base_link (机体) ── /Odometry 提供
 *
 *   最终发布: /localization = map → base_link (融合后)
 *
 * 典型用途：
 *   在全局定位流水线中，作为全局定位与局部里程计之间的桥梁，
 *   为下游节点 (planner, controller) 提供统一的全局位姿估计。
 *
 * 使用：
 *   roslaunch uav_location global_planner_in_sim.launch  (自动启动)
 *   或独立运行：
 *   rosrun uav_location transform_fusion _global_locatiation_hz:=30.0
 */

#include <ros/ros.h>
#include <thread>
#include <mutex>

#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <Eigen/Dense>

using namespace std;

// ============================================================================
// 全局共享变量 (互斥保护)
// ============================================================================

ros::Publisher pub_localization;           ///< /localization — 融合后全局位姿
ros::Publisher pub_global_start_pose;      ///< /global_start_pose (预留)

nav_msgs::Odometry cur_odom_to_baselink;   ///< 最新局部里程计 (odom→base_link)
nav_msgs::Odometry cur_map_to_odom;        ///< 最新全局定位 (map→odom)

bool has_odom         = false;             ///< 是否收到局部里程计
bool has_map_to_odom  = false;             ///< 是否收到全局定位
bool have_global_start_pose = false;       ///< 是否已获取全局起始位姿
std::mutex mtx;                            ///< 数据互斥锁

double FREQ_PUB_LOCALIZATION = 30.0;       ///< 融合发布频率 [Hz]

// ============================================================================
// 工具函数
// ============================================================================

/**
 * @brief 将 geometry_msgs::Pose 转换为 Eigen::Matrix4d 齐次变换矩阵。
 * @param pose 输入位姿消息
 * @return 4×4 变换矩阵
 */
Eigen::Matrix4d poseToMatrix(const geometry_msgs::Pose &pose)
{
    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
    Eigen::Quaterniond q(pose.orientation.w, pose.orientation.x,
                         pose.orientation.y, pose.orientation.z);
    Eigen::Matrix3d R = q.toRotationMatrix();
    T.block<3, 3>(0, 0) = R;
    T(0, 3) = pose.position.x;
    T(1, 3) = pose.position.y;
    T(2, 3) = pose.position.z;
    return T;
}

// ============================================================================
// 回调函数
// ============================================================================

/// @brief 保存最新局部里程计
void cbOdom(const nav_msgs::Odometry::ConstPtr &msg)
{
    std::lock_guard<std::mutex> lock(mtx);
    cur_odom_to_baselink = *msg;
    has_odom = true;
}

/// @brief 保存最新全局定位结果 (map→odom)
void cbMapToOdom(const nav_msgs::Odometry::ConstPtr &msg)
{
    std::lock_guard<std::mutex> lock(mtx);
    cur_map_to_odom = *msg;
    has_map_to_odom = true;
}

// ============================================================================
// 融合线程
// ============================================================================

/**
 * @brief 独立融合线程，按固定频率发布全局融合位姿。
 *
 * 融合公式：
 *   T_map_to_base = T_map_to_odom × T_odom_to_base
 *
 * 同步发布 TF: map → camera_init，用于 RViz 可视化对齐。
 */
void transformFusion()
{
    tf::TransformBroadcaster br;
    ros::Rate rate(FREQ_PUB_LOCALIZATION);

    while (ros::ok())
    {
        nav_msgs::Odometry cur_odom_copy;
        nav_msgs::Odometry cur_map_copy;

        // 加锁拷贝最新数据
        {
            std::lock_guard<std::mutex> lock(mtx);
            cur_odom_copy = cur_odom_to_baselink;
            cur_map_copy  = cur_map_to_odom;
        }

        // 构造 map→odom 变换 (如未收到则用单位阵)
        Eigen::Matrix4d T_map_to_odom = Eigen::Matrix4d::Identity();
        if (has_map_to_odom)
        {
            T_map_to_odom = poseToMatrix(cur_map_copy.pose.pose);
        }

        // ---- 广播 TF: map → camera_init ----
        // camera_init 是 FAST-LIO 等 SLAM 系统的初始坐标系
        tf::Transform tf_map_to_cam;
        tf::Vector3 t_map_to_cam(T_map_to_odom(0, 3), T_map_to_odom(1, 3), T_map_to_odom(2, 3));
        Eigen::Matrix3d R_map_to_cam = T_map_to_odom.block<3, 3>(0, 0);
        Eigen::Quaterniond q_map_to_cam(R_map_to_cam);
        tf::Quaternion tf_q(q_map_to_cam.x(), q_map_to_cam.y(), q_map_to_cam.z(), q_map_to_cam.w());
        tf_map_to_cam.setOrigin(t_map_to_cam);
        tf_map_to_cam.setRotation(tf_q);
        br.sendTransform(tf::StampedTransform(tf_map_to_cam, ros::Time::now(), "map", "camera_init"));

        // ---- 融合并发布 /localization ----
        if (has_odom)
        {
            // T_map_to_base = T_map_to_odom × T_odom_to_base
            Eigen::Matrix4d T_odom_to_base = poseToMatrix(cur_odom_copy.pose.pose);
            Eigen::Matrix4d T_map_to_base  = T_map_to_odom * T_odom_to_base;

            Eigen::Vector3d t = T_map_to_base.block<3, 1>(0, 3);
            Eigen::Matrix3d R = T_map_to_base.block<3, 3>(0, 0);
            Eigen::Quaterniond q(R);

            nav_msgs::Odometry localization;
            localization.header.stamp    = cur_odom_copy.header.stamp;
            localization.header.frame_id = "map";
            localization.child_frame_id  = "base_link";

            localization.pose.pose.position.x    = t.x();
            localization.pose.pose.position.y    = t.y();
            localization.pose.pose.position.z    = t.z();
            localization.pose.pose.orientation.x = q.x();
            localization.pose.pose.orientation.y = q.y();
            localization.pose.pose.orientation.z = q.z();
            localization.pose.pose.orientation.w = q.w();

            // 速度信息直接继承局部里程计
            localization.twist = cur_odom_copy.twist;

            pub_localization.publish(localization);

            // 首次获得有效全局位姿时输出日志
            if(!have_global_start_pose && has_map_to_odom)
            {
                ROS_INFO("[global] Global localization initialized at (%f, %f, %f)",
                         localization.pose.pose.position.x,
                         localization.pose.pose.position.y,
                         localization.pose.pose.position.z);
                have_global_start_pose = true;
            }
        }

        rate.sleep();
    }
    ros::shutdown();
}

// ============================================================================
// 主函数
// ============================================================================

int main(int argc, char **argv)
{
    ros::init(argc, argv, "transform_fusion");
    ros::NodeHandle nh;

    ROS_INFO("[global] Transform Fusion Node Inited...");

    // ---- 加载参数 ----
    std::string global_start_topic;
    nh.param<double>("global_locatiation_hz", FREQ_PUB_LOCALIZATION, FREQ_PUB_LOCALIZATION);

    // ---- 订阅器 ----
    ros::Subscriber sub_odom        = nh.subscribe("/Odometry", 10, cbOdom);
    ros::Subscriber sub_map_to_odom = nh.subscribe("/map_to_odom", 2, cbMapToOdom);

    // ---- 发布器 ----
    pub_localization = nh.advertise<nav_msgs::Odometry>("/localization", 10);

    // ---- 启动融合线程 ----
    std::thread th(transformFusion);
    th.detach();

    ros::spin();
    return 0;
}
