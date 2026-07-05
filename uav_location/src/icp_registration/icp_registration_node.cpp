/**
 * @file icp_registration_node.cpp
 * @brief 基于 ICP (点-面 法向ICP) 的 LiDAR 全局配准节点。
 *
 * @details
 * 功能：接收 LiDAR 点云，通过多候选初值的两阶段 ICP (粗→精) 将当前扫描
 *       与预建 PCD 地图对齐，发布 map → odom 的 TF 广播。
 *
 * 算法流水线：
 *   1. 加载 PCD 全局地图，生成粗/精两个体素尺度的法向点云
 *   2. 首次收到扫描 → 使用 initial_pose 作为初值
 *   3. 多候选初值搜索：以初始位姿为中心，在 xy_offset×yaw_offset 范围内
 *      枚举候选变换矩阵 (9×N_yaw 个候选)
 *   4. 粗配准 (rough_leaf_size)：遍历所有候选，保留 best fitness < 2×thresh
 *   5. 精配准 (refine_leaf_size)：从最佳粗结果出发，做精细 ICP
 *   6. 查 tf2 获取 laser→odom 变换 → 计算 map→odom → 广播 TF
 *
 * 坐标系：
 *   - map:            全局固定坐标系
 *   - odom:           里程计漂移坐标系
 *   - lidar_odom:     激光里程计源坐标系 (如 FAST-LIO 的 odom)
 *   - laser:          LiDAR 传感器安装坐标系
 *
 * 依赖：PCL (ICPWithNormals, NormalEstimation, VoxelGrid), tf2
 * 节点名：icp_registration
 * 使用：
 *   roslaunch uav_location icp_registration.launch pcd_path:=/path/to/map.pcd
 *
 * @note 与原 Python 实现对齐：多候选初值 + 粗→精两阶段 ICP。
 */

#include <ros/ros.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <Eigen/Dense>
#include <thread>
#include <mutex>
#include <filesystem>

/**
 * @class IcpNode
 * @brief ICP 全局配准节点类，封装点云处理、候选搜索与 TF 广播。
 */
class IcpNode
{
public:
    using PointCloudXYZI  = pcl::PointCloud<pcl::PointXYZI>;
    using PointCloudXYZIN = pcl::PointCloud<pcl::PointXYZINormal>;

    /**
     * @brief 构造函数：加载 PCD 地图，初始化体素滤波与 ICP 对象。
     * @param nh  全局节点句柄
     * @param pnh 私有节点句柄 (用于加载 ~param)
     * @throws std::runtime_error 如果 PCD 文件不存在
     */
    IcpNode(ros::NodeHandle& nh, ros::NodeHandle& pnh)
        : nh_(nh), pnh_(pnh), first_scan_(true), is_ready_(false),
          rough_iter_(10), refine_iter_(5)
    {
        cloud_in_.reset(new PointCloudXYZI);

        // ---- 加载配准参数 ----
        double rough_leaf_size, refine_leaf_size;
        pnh_.param("rough_leaf_size", rough_leaf_size, 0.4);
        pnh_.param("refine_leaf_size", refine_leaf_size, 0.1);

        voxel_rough_filter_.setLeafSize(rough_leaf_size, rough_leaf_size, rough_leaf_size);
        voxel_refine_filter_.setLeafSize(refine_leaf_size, refine_leaf_size, refine_leaf_size);

        // ---- 加载 PCD 地图 ----
        pnh_.param<std::string>("pcd_path", pcd_path_, "");
        if (!std::filesystem::exists(pcd_path_))
        {
            ROS_ERROR("Invalid PCD path: %s", pcd_path_.c_str());
            throw std::runtime_error("Invalid pcd path");
        }

        // 读取 PCD 并生成粗/精两版法向地图
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new PointCloudXYZI);
        pcl::PCDReader reader;
        reader.read(pcd_path_, *cloud);
        voxel_refine_filter_.setInputCloud(cloud);
        voxel_refine_filter_.filter(*cloud);
        refine_map_ = addNorm(cloud);

        pcl::PointCloud<pcl::PointXYZI>::Ptr rough(new PointCloudXYZI);
        voxel_rough_filter_.setInputCloud(cloud);
        voxel_rough_filter_.filter(*rough);
        rough_map_ = addNorm(rough);

        // 配置 ICP (法向点对平面 ICP)
        icp_rough_.setMaximumIterations(rough_iter_);
        icp_rough_.setInputTarget(rough_map_);
        icp_refine_.setMaximumIterations(refine_iter_);
        icp_refine_.setInputTarget(refine_map_);

        // ---- 坐标系与搜索参数 ----
        pnh_.param<std::string>("map_frame_id", map_frame_id_, "map");
        pnh_.param<std::string>("odom_frame_id", odom_frame_id_, "odom");
        pnh_.param<std::string>("range_odom_frame_id", range_odom_frame_id_, "lidar_odom");
        pnh_.param<std::string>("laser_frame_id", laser_frame_id_, "laser");
        pnh_.param("thresh", thresh_, 0.15);                  ///< 精配准 fitness 阈值
        pnh_.param("xy_offset", xy_offset_, 0.2);             ///< XY 搜索偏移 [m]
        pnh_.param("yaw_offset", yaw_offset_, 30.0);          ///< Yaw 搜索范围 [deg]
        pnh_.param("yaw_resolution", yaw_resolution_, 10.0);  ///< Yaw 搜索步长 [deg]

        // 角度转弧度
        yaw_offset_     *= M_PI / 180.0;
        yaw_resolution_ *= M_PI / 180.0;

        // ---- 初始位姿 ----
        std::vector<double> initial_pose_vec(6, 0.0);
        pnh_.param("initial_pose", initial_pose_vec, initial_pose_vec);
        initial_pose_.position.x = initial_pose_vec[0];
        initial_pose_.position.y = initial_pose_vec[1];
        initial_pose_.position.z = initial_pose_vec[2];
        tf2::Quaternion q;
        q.setRPY(initial_pose_vec[3], initial_pose_vec[4], initial_pose_vec[5]);
        tf2::convert(q, initial_pose_.orientation);

        // ---- 话题订阅 ----
        std::string pointcloud_topic;
        pnh_.param<std::string>("pointcloud_topic", pointcloud_topic, "/livox/lidar/pointcloud");
        pointcloud_sub_   = nh_.subscribe(pointcloud_topic, 1, &IcpNode::pointcloudCallback, this);
        initial_pose_sub_ = nh_.subscribe("/initialpose", 1, &IcpNode::initialPoseCallback, this);

        // ---- TF 监听与广播线程 ----
        tf_listener_  = std::make_shared<tf2_ros::TransformListener>(tf_buffer_);
        tf_pub_thread_ = std::thread(&IcpNode::tfPublishLoop, this);

        ROS_INFO("ICP registration initialized.");
    }

    ~IcpNode()
    {
        if (tf_pub_thread_.joinable()) tf_pub_thread_.join();
    }

private:
    // === ROS 句柄与通信 ===
    ros::NodeHandle nh_, pnh_;
    ros::Subscriber pointcloud_sub_, initial_pose_sub_;
    tf2_ros::Buffer tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    tf2_ros::TransformBroadcaster tf_broadcaster_;

    // === 点云处理 ===
    pcl::VoxelGrid<pcl::PointXYZI> voxel_rough_filter_, voxel_refine_filter_;
    pcl::IterativeClosestPointWithNormals<pcl::PointXYZINormal, pcl::PointXYZINormal> icp_rough_, icp_refine_;
    PointCloudXYZIN::Ptr refine_map_;   ///< 精配准目标地图 (带法向)
    PointCloudXYZIN::Ptr rough_map_;    ///< 粗配准目标地图 (带法向)
    PointCloudXYZI::Ptr  cloud_in_;     ///< 当前输入点云

    // === 坐标系配置 ===
    std::string map_frame_id_;          ///< "map"
    std::string odom_frame_id_;         ///< "odom" (map→odom 的 child)
    std::string range_odom_frame_id_;   ///< 里程计源系 "lidar_odom"
    std::string laser_frame_id_;        ///< LiDAR 系 "laser"
    std::string pcd_path_;              ///< PCD 地图文件路径

    // === 内部状态 ===
    geometry_msgs::TransformStamped map_to_odom_;
    geometry_msgs::Pose initial_pose_;
    std::mutex mutex_;
    std::thread tf_pub_thread_;
    bool first_scan_;
    bool is_ready_;                     ///< map_to_odom 是否已就绪
    bool success_;                      ///< 当前 ICP 是否成功
    int rough_iter_, refine_iter_;      ///< ICP 迭代次数
    double thresh_;                     ///< 精配准 fitness 阈值
    double score_;                      ///< 最近一次配准的 fitness 值
    double xy_offset_, yaw_offset_, yaw_resolution_;

    // ========================================================================
    // 回调函数
    // ========================================================================

    /**
     * @brief 点云回调：保存当前扫描，首次时触发初始位姿配准。
     */
    void pointcloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg)
    {
        pcl::fromROSMsg(*msg, *cloud_in_);
        if (first_scan_)
        {
            geometry_msgs::PoseWithCovarianceStamped pose_msg;
            pose_msg.header = msg->header;
            pose_msg.pose.pose = initial_pose_;
            initialPoseCallback(boost::make_shared<geometry_msgs::PoseWithCovarianceStamped>(pose_msg));
            first_scan_ = false;
        }
    }

    /**
     * @brief 初始位姿回调：执行多候选 ICP 配准并计算 map→odom 变换。
     *
     * 流程：
     *   1. 多候选搜索粗配准 → 最佳 candidate
     *   2. 精配准
     *   3. 查 tf 获取 laser→range_odom 变换
     *   4. map→odom = map→laser × laser→odom
     */
    void initialPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
    {
        // 构造初始猜测矩阵
        Eigen::Vector3d pos(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);
        Eigen::Quaterniond q(msg->pose.pose.orientation.w, msg->pose.pose.orientation.x,
                            msg->pose.pose.orientation.y, msg->pose.pose.orientation.z);
        Eigen::Matrix4d init_guess = Eigen::Matrix4d::Identity();
        init_guess.block<3,3>(0,0) = q.toRotationMatrix();
        init_guess.block<3,1>(0,3) = pos;

        // 多候选搜索 + 粗→精两阶段 ICP
        Eigen::Matrix4d map_to_laser = multiAlignSync(cloud_in_, init_guess);
        if (!success_)
        {
            ROS_ERROR("ICP failed");
            return;
        }

        // 查询 laser → range_odom 的 TF 变换
        geometry_msgs::TransformStamped odom_tf;
        try
        {
            odom_tf = tf_buffer_.lookupTransform(laser_frame_id_, range_odom_frame_id_,
                                                  ros::Time(0), ros::Duration(2.0));
        }
        catch (tf2::TransformException &ex)
        {
            ROS_ERROR("%s", ex.what());
            return;
        }

        // laser→odom 矩阵
        Eigen::Matrix4d laser_to_odom = Eigen::Matrix4d::Identity();
        Eigen::Vector3d t(odom_tf.transform.translation.x,
                          odom_tf.transform.translation.y,
                          odom_tf.transform.translation.z);
        Eigen::Quaterniond q_odom(odom_tf.transform.rotation.w, odom_tf.transform.rotation.x,
                                  odom_tf.transform.rotation.y, odom_tf.transform.rotation.z);
        laser_to_odom.block<3,3>(0,0) = q_odom.toRotationMatrix();
        laser_to_odom.block<3,1>(0,3) = t;

        // map→odom = map→laser × laser→odom
        Eigen::Matrix4d result = map_to_laser * laser_to_odom;
        {
            std::lock_guard<std::mutex> lock(mutex_);
            map_to_odom_.transform.translation.x = result(0,3);
            map_to_odom_.transform.translation.y = result(1,3);
            map_to_odom_.transform.translation.z = result(2,3);
            Eigen::Quaterniond q_final(result.block<3,3>(0,0));
            map_to_odom_.transform.rotation.w = q_final.w();
            map_to_odom_.transform.rotation.x = q_final.x();
            map_to_odom_.transform.rotation.y = q_final.y();
            map_to_odom_.transform.rotation.z = q_final.z();
            map_to_odom_.header.frame_id = map_frame_id_;
            map_to_odom_.child_frame_id  = odom_frame_id_;
            is_ready_ = true;
        }
    }

    // ========================================================================
    // 多候选同步对齐
    // ========================================================================

    /**
     * @brief 多候选初值搜索 + 粗→精两阶段 ICP。
     *
     * 首先生成以初始位姿为中心的 9×N_yaw 个候选 (X/Y: -1,0,1 × xy_offset;
     * Yaw: -N..N × yaw_resolution)，每个候选执行粗 ICP，取最佳结果再做精 ICP。
     *
     * @param source     源点云 (当前扫描)
     * @param init_guess 初始猜测变换 (从 /initialpose 获得)
     * @return 最优 map→laser 变换矩阵 (4×4 double)，失败返回零矩阵
     */
    Eigen::Matrix4d IcpNode::multiAlignSync(PointCloudXYZI::Ptr source,
                                            const Eigen::Matrix4d &init_guess)
    {
        // 辅助 lambda：旋转矩阵 → RPY
        static auto rotate2rpy = [](const Eigen::Matrix3d &rot) -> Eigen::Vector3d
        {
            double roll  = std::atan2(rot(2,1), rot(2,2));
            double pitch = std::asin(std::clamp(-rot(2,0), -1.0, 1.0));
            double yaw   = std::atan2(rot(1,0), rot(0,0));
            return Eigen::Vector3d(roll, pitch, yaw);
        };

        success_ = false;
        Eigen::Vector3d xyz = init_guess.block<3,1>(0,3);
        Eigen::Matrix3d rot = init_guess.block<3,3>(0,0);
        Eigen::Vector3d rpy = rotate2rpy(rot);

        // 固定 roll/pitch 旋转 (不搜索)
        Eigen::AngleAxisf rollAngle((float)rpy(0), Eigen::Vector3f::UnitX());
        Eigen::AngleAxisf pitchAngle((float)rpy(1), Eigen::Vector3f::UnitY());

        // ---- 生成候选初值 (9 XY × N_yaw) ----
        std::vector<Eigen::Matrix4f> candidates;
        candidates.reserve(9 * 21);

        int yaw_steps = 0;
        if (yaw_resolution_ > 1e-6)
        {
            yaw_steps = static_cast<int>(std::round(yaw_offset_ / yaw_resolution_));
        }

        for (int ix = -1; ix <= 1; ++ix)
        {
            for (int iy = -1; iy <= 1; ++iy)
            {
                for (int ky = -yaw_steps; ky <= yaw_steps; ++ky)
                {
                    float yaw = static_cast<float>(rpy(2) + ky * yaw_resolution_);
                    Eigen::AngleAxisf yawAngle(yaw, Eigen::Vector3f::UnitZ());
                    Eigen::Matrix4f T = Eigen::Matrix4f::Identity();
                    T.block<3,3>(0,0) = (rollAngle * pitchAngle * yawAngle).toRotationMatrix();
                    T.block<3,1>(0,3) = Eigen::Vector3f(
                        static_cast<float>(xyz(0) + ix * xy_offset_),
                        static_cast<float>(xyz(1) + iy * xy_offset_),
                        static_cast<float>(xyz(2))
                    );
                    candidates.push_back(T);
                }
            }
        }

        // ---- 下采样源点云 ----
        PointCloudXYZI::Ptr rough_source(new PointCloudXYZI);
        PointCloudXYZI::Ptr refine_source(new PointCloudXYZI);
        voxel_rough_filter_.setInputCloud(source);
        voxel_rough_filter_.filter(*rough_source);
        voxel_refine_filter_.setInputCloud(source);
        voxel_refine_filter_.filter(*refine_source);

        // 计算法向
        PointCloudXYZIN::Ptr rough_source_norm = addNorm(rough_source);
        PointCloudXYZIN::Ptr refine_source_norm = addNorm(refine_source);
        PointCloudXYZIN::Ptr align_point(new PointCloudXYZIN);

        // ---- 粗配准：遍历所有候选 ----
        Eigen::Matrix4f best_rough_transform = Eigen::Matrix4f::Identity();
        double best_rough_score = std::numeric_limits<double>::infinity();
        bool rough_converge = false;

        auto tic = std::chrono::high_resolution_clock::now();
        for (const Eigen::Matrix4f &init_pose : candidates) {
            icp_rough_.setInputSource(rough_source_norm);
            icp_rough_.align(*align_point, init_pose);

            if (!icp_rough_.hasConverged()) continue;

            double rough_score = static_cast<double>(icp_rough_.getFitnessScore());
            // 丢弃过差的候选 (heuristic: 2× thresh)
            if (rough_score > 2.0 * thresh_) continue;

            if (rough_score < best_rough_score)
            {
                best_rough_score = rough_score;
                rough_converge = true;
                best_rough_transform = icp_rough_.getFinalTransformation();
            }
        }

        if (!rough_converge)
        {
            ROS_DEBUG("multiAlignSync: no rough candidate converged");
            return Eigen::Matrix4d::Zero();
        }

        // ---- 精配准：从最佳粗结果出发 ----
        icp_refine_.setInputSource(refine_source_norm);
        icp_refine_.align(*align_point, best_rough_transform);
        score_ = static_cast<double>(icp_refine_.getFitnessScore());

        if (!icp_refine_.hasConverged())
        {
            ROS_DEBUG("multiAlignSync: refine ICP did not converge");
            return Eigen::Matrix4d::Zero();
        }
        if (score_ > thresh_)
        {
            ROS_DEBUG("multiAlignSync: fitness score too large: %f (thresh %f)", score_, thresh_);
            return Eigen::Matrix4d::Zero();
        }

        success_ = true;
        auto toc = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> duration = toc - tic;
        ROS_INFO("multiAlignSync used: %f ms", duration.count() * 1000.0);
        ROS_INFO("multiAlignSync best score: %f", score_);

        Eigen::Matrix4f final_tf_f = icp_refine_.getFinalTransformation();
        Eigen::Matrix4d final_tf = final_tf_f.cast<double>();
        return final_tf;
    }

    // ========================================================================
    // TF 广播线程
    // ========================================================================

    /**
     * @brief 独立线程按 50Hz 频率发布 map→odom TF 变换。
     */
    void tfPublishLoop()
    {
        ros::Rate rate(50);
        while (ros::ok())
        {
            if (is_ready_)
            {
                std::lock_guard<std::mutex> lock(mutex_);
                map_to_odom_.header.stamp = ros::Time::now();
                tf_broadcaster_.sendTransform(map_to_odom_);
            }
            rate.sleep();
        }
    }

    // ========================================================================
    // 点云法向计算
    // ========================================================================

    /**
     * @brief 为 XYZI 点云计算法向，输出带法向的 XYZINormal 点云。
     * @param cloud 输入点云 (XYZI)
     * @return 带法向的点云 (XYZINormal)，K=15 最近邻搜索
     */
    PointCloudXYZIN::Ptr addNorm(PointCloudXYZI::Ptr cloud)
    {
        pcl::NormalEstimation<pcl::PointXYZI, pcl::Normal> ne;
        pcl::search::KdTree<pcl::PointXYZI>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZI>());
        pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
        ne.setInputCloud(cloud);
        ne.setSearchMethod(tree);
        ne.setKSearch(15);   ///< K 近邻搜索点数
        ne.compute(*normals);
        PointCloudXYZIN::Ptr out(new PointCloudXYZIN);
        pcl::concatenateFields(*cloud, *normals, *out);
        return out;
    }
};

// ============================================================================
// 主函数
// ============================================================================

int main(int argc, char** argv)
{
    ros::init(argc, argv, "icp_registration");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");
    try
    {
        IcpNode node(nh, pnh);
        ros::spin();
    }
    catch (const std::exception& e)
    {
        ROS_ERROR("Exception: %s", e.what());
    }
    return 0;
}
