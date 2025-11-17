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

class IcpNode
{
public:
    using PointCloudXYZI = pcl::PointCloud<pcl::PointXYZI>;
    using PointCloudXYZIN = pcl::PointCloud<pcl::PointXYZINormal>;

    IcpNode(ros::NodeHandle& nh, ros::NodeHandle& pnh)
        : nh_(nh), pnh_(pnh), first_scan_(true), is_ready_(false),
          rough_iter_(10), refine_iter_(5)
    {
        cloud_in_.reset(new PointCloudXYZI);

        double rough_leaf_size, refine_leaf_size;
        pnh_.param("rough_leaf_size", rough_leaf_size, 0.4);
        pnh_.param("refine_leaf_size", refine_leaf_size, 0.1);

        voxel_rough_filter_.setLeafSize(rough_leaf_size, rough_leaf_size, rough_leaf_size);
        voxel_refine_filter_.setLeafSize(refine_leaf_size, refine_leaf_size, refine_leaf_size);

        pnh_.param<std::string>("pcd_path", pcd_path_, "");
        if (!std::filesystem::exists(pcd_path_))
        {
            ROS_ERROR("Invalid PCD path: %s", pcd_path_.c_str());
            throw std::runtime_error("Invalid pcd path");
        }

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

        icp_rough_.setMaximumIterations(rough_iter_);
        icp_rough_.setInputTarget(rough_map_);
        icp_refine_.setMaximumIterations(refine_iter_);
        icp_refine_.setInputTarget(refine_map_);

        pnh_.param<std::string>("map_frame_id", map_frame_id_, "map");
        pnh_.param<std::string>("odom_frame_id", odom_frame_id_, "odom");
        pnh_.param<std::string>("range_odom_frame_id", range_odom_frame_id_, "lidar_odom");
        pnh_.param<std::string>("laser_frame_id", laser_frame_id_, "laser");
        pnh_.param("thresh", thresh_, 0.15);
        pnh_.param("xy_offset", xy_offset_, 0.2);
        pnh_.param("yaw_offset", yaw_offset_, 30.0);
        pnh_.param("yaw_resolution", yaw_resolution_, 10.0);

        yaw_offset_ *= M_PI / 180.0;
        yaw_resolution_ *= M_PI / 180.0;

        std::vector<double> initial_pose_vec(6, 0.0);
        pnh_.param("initial_pose", initial_pose_vec, initial_pose_vec);
        initial_pose_.position.x = initial_pose_vec[0];
        initial_pose_.position.y = initial_pose_vec[1];
        initial_pose_.position.z = initial_pose_vec[2];
        tf2::Quaternion q;
        q.setRPY(initial_pose_vec[3], initial_pose_vec[4], initial_pose_vec[5]);
        tf2::convert(q, initial_pose_.orientation);

        std::string pointcloud_topic;
        pnh_.param<std::string>("pointcloud_topic", pointcloud_topic, "/livox/lidar/pointcloud");
        pointcloud_sub_ = nh_.subscribe(pointcloud_topic, 1, &IcpNode::pointcloudCallback, this);
        initial_pose_sub_ = nh_.subscribe("/initialpose", 1, &IcpNode::initialPoseCallback, this);

        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(tf_buffer_);
        tf_pub_thread_ = std::thread(&IcpNode::tfPublishLoop, this);

        ROS_INFO("ICP registration initialized.");
    }

    ~IcpNode()
    {
        if (tf_pub_thread_.joinable()) tf_pub_thread_.join();
    }

private:
    ros::NodeHandle nh_, pnh_;
    ros::Subscriber pointcloud_sub_, initial_pose_sub_;
    tf2_ros::Buffer tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    tf2_ros::TransformBroadcaster tf_broadcaster_;

    pcl::VoxelGrid<pcl::PointXYZI> voxel_rough_filter_, voxel_refine_filter_;
    pcl::IterativeClosestPointWithNormals<pcl::PointXYZINormal, pcl::PointXYZINormal> icp_rough_, icp_refine_;
    PointCloudXYZIN::Ptr refine_map_, rough_map_;
    PointCloudXYZI::Ptr cloud_in_;

    std::string map_frame_id_, odom_frame_id_, range_odom_frame_id_, laser_frame_id_;
    std::string pcd_path_;
    geometry_msgs::TransformStamped map_to_odom_;
    geometry_msgs::Pose initial_pose_;

    std::mutex mutex_;
    std::thread tf_pub_thread_;
    bool first_scan_, is_ready_, success_;
    int rough_iter_, refine_iter_;
    double thresh_, xy_offset_, yaw_offset_, yaw_resolution_;

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

    void initialPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
    {
        Eigen::Vector3d pos(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);
        Eigen::Quaterniond q(msg->pose.pose.orientation.w, msg->pose.pose.orientation.x,
                            msg->pose.pose.orientation.y, msg->pose.pose.orientation.z);
        Eigen::Matrix4d init_guess = Eigen::Matrix4d::Identity();
        init_guess.block<3,3>(0,0) = q.toRotationMatrix();
        init_guess.block<3,1>(0,3) = pos;

        Eigen::Matrix4d map_to_laser = multiAlignSync(cloud_in_, init_guess);
        if (!success_)
        {
            ROS_ERROR("ICP failed");
            return;
        }

        geometry_msgs::TransformStamped odom_tf;
        try
        {
            odom_tf = tf_buffer_.lookupTransform(laser_frame_id_, range_odom_frame_id_, ros::Time(0), ros::Duration(2.0));
        }
        catch (tf2::TransformException &ex)
        {
            ROS_ERROR("%s", ex.what());
            return;
        }

        Eigen::Matrix4d laser_to_odom = Eigen::Matrix4d::Identity();
        Eigen::Vector3d t(odom_tf.transform.translation.x, odom_tf.transform.translation.y, odom_tf.transform.translation.z);
        Eigen::Quaterniond q_odom(odom_tf.transform.rotation.w, odom_tf.transform.rotation.x,
                                  odom_tf.transform.rotation.y, odom_tf.transform.rotation.z);
        
        laser_to_odom.block<3,3>(0,0) = q_odom.toRotationMatrix();
        laser_to_odom.block<3,1>(0,3) = t;

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
            map_to_odom_.child_frame_id = odom_frame_id_;
            is_ready_ = true;
        }
    }

    Eigen::Matrix4d IcpNode::multiAlignSync(PointCloudXYZI::Ptr source,
                                            const Eigen::Matrix4d &init_guess)
    {
        // Helper: convert rotation matrix -> roll, pitch, yaw
        static auto rotate2rpy = [](const Eigen::Matrix3d &rot) -> Eigen::Vector3d
        {
            double roll  = std::atan2(rot(2,1), rot(2,2));
            double pitch = std::asin(std::clamp(-rot(2,0), -1.0, 1.0));
            double yaw   = std::atan2(rot(1,0), rot(0,0));
            return Eigen::Vector3d(roll, pitch, yaw);
        };

        success_ = false;
        // extract xyz + rotation
        Eigen::Vector3d xyz = init_guess.block<3,1>(0,3);
        Eigen::Matrix3d rot = init_guess.block<3,3>(0,0);
        Eigen::Vector3d rpy = rotate2rpy(rot);

        // Prepare angle axes for roll & pitch (we won't perturb these in search)
        Eigen::AngleAxisf rollAngle((float)rpy(0), Eigen::Vector3f::UnitX());
        Eigen::AngleAxisf pitchAngle((float)rpy(1), Eigen::Vector3f::UnitY());

        // Build candidate transforms around initial guess by offsetting x/y and yaw
        std::vector<Eigen::Matrix4f> candidates;
        candidates.reserve(9 * 21); // heuristic reserve

        // compute yaw steps properly (yaw_offset_ and yaw_resolution_ are radians)
        int yaw_steps = 0;
        if (yaw_resolution_ > 1e-6)
        {
            yaw_steps = static_cast<int>(std::round(yaw_offset_ / yaw_resolution_));
        }
        // if yaw_steps is 0 still include single yaw (the initial yaw)
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

        // downsample source for rough/refine stage
        PointCloudXYZI::Ptr rough_source(new PointCloudXYZI);
        PointCloudXYZI::Ptr refine_source(new PointCloudXYZI);
        voxel_rough_filter_.setInputCloud(source);
        voxel_rough_filter_.filter(*rough_source);
        voxel_refine_filter_.setInputCloud(source);
        voxel_refine_filter_.filter(*refine_source);

        // add normals (ICP that uses normals expects PointXYZIN)
        PointCloudXYZIN::Ptr rough_source_norm = addNorm(rough_source);
        PointCloudXYZIN::Ptr refine_source_norm = addNorm(refine_source);
        PointCloudXYZIN::Ptr align_point(new PointCloudXYZIN);

        // Search over candidates using rough ICP
        Eigen::Matrix4f best_rough_transform = Eigen::Matrix4f::Identity();
        double best_rough_score = std::numeric_limits<double>::infinity();
        bool rough_converge = false;

        auto tic = std::chrono::high_resolution_clock::now();
        for (const Eigen::Matrix4f &init_pose : candidates) {
            icp_rough_.setInputSource(rough_source_norm);
            // align: provide initial guess
            icp_rough_.align(*align_point, init_pose);

            if (!icp_rough_.hasConverged())
            {
                // skip non-converged
                continue;
            }
            double rough_score = static_cast<double>(icp_rough_.getFitnessScore());
            // discard overly bad initial fits (2x thresh is heuristic, same as original)
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

        // refine using refine ICP initialized from best rough transform
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

        // Return final transform as double-precision 4x4 matrix
        Eigen::Matrix4f final_tf_f = icp_refine_.getFinalTransformation();
        Eigen::Matrix4d final_tf = final_tf_f.cast<double>();
        return final_tf;
    }


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

    PointCloudXYZIN::Ptr addNorm(PointCloudXYZI::Ptr cloud)
    {
        pcl::NormalEstimation<pcl::PointXYZI, pcl::Normal> ne;
        pcl::search::KdTree<pcl::PointXYZI>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZI>());
        pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
        ne.setInputCloud(cloud);
        ne.setSearchMethod(tree);
        ne.setKSearch(15);
        ne.compute(*normals);
        PointCloudXYZIN::Ptr out(new PointCloudXYZIN);
        pcl::concatenateFields(*cloud, *normals, *out);
        return out;
    }
};


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
