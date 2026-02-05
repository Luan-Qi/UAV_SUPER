#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/Vector3Stamped.h>

#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>

#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>

#include <Eigen/Dense>

class PCADirectionEstimator
{
public:
    PCADirectionEstimator(ros::NodeHandle& nh)
    {
        sub_cloud_ = nh.subscribe("/livox/lidar", 1,
                                  &PCADirectionEstimator::cloudCallback, this);

        pub_dir_ = nh.advertise<geometry_msgs::Vector3Stamped>(
            "/pca_direction", 1);

        // 参数（可通过 rosparam 配）
        nh.param("roi_x_min", roi_x_min_, 0.5);   // 前方 0.5m
        nh.param("roi_x_max", roi_x_max_, 10.0);  // 前方 10m
        nh.param("roi_z_min", roi_z_min_, -1.0);
        nh.param("roi_z_max", roi_z_max_, 1.5);

        nh.param("voxel_size", voxel_size_, 0.2);

        ROS_INFO("[PCA Direction Node] Initialized.");
    }

private:
    ros::Subscriber sub_cloud_;
    ros::Publisher  pub_dir_;

    double roi_x_min_, roi_x_max_;
    double roi_z_min_, roi_z_max_;
    double voxel_size_;

    void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(*msg, *cloud);

        if (cloud->empty())
            return;

        // 1️⃣ ROI 裁剪（只保留前方）
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_roi(new pcl::PointCloud<pcl::PointXYZ>);

        pcl::PassThrough<pcl::PointXYZ> pass;
        pass.setInputCloud(cloud);
        pass.setFilterFieldName("x");
        pass.setFilterLimits(roi_x_min_, roi_x_max_);
        pass.filter(*cloud_roi);

        pass.setInputCloud(cloud_roi);
        pass.setFilterFieldName("z");
        pass.setFilterLimits(roi_z_min_, roi_z_max_);
        pass.filter(*cloud_roi);

        if (cloud_roi->size() < 30)
            return;

        // 2️⃣ 下采样
        pcl::VoxelGrid<pcl::PointXYZ> voxel;
        voxel.setInputCloud(cloud_roi);
        voxel.setLeafSize(voxel_size_, voxel_size_, voxel_size_);
        voxel.filter(*cloud_roi);

        if (cloud_roi->size() < 10)
            return;

        // 3️⃣ PCA / SVD
        Eigen::Vector3f mean = Eigen::Vector3f::Zero();
        for (auto& p : cloud_roi->points)
            mean += Eigen::Vector3f(p.x, p.y, p.z);
        mean /= cloud_roi->size();

        Eigen::Matrix3f cov = Eigen::Matrix3f::Zero();
        for (auto& p : cloud_roi->points)
        {
            Eigen::Vector3f pt(p.x, p.y, p.z);
            pt -= mean;
            cov += pt * pt.transpose();
        }
        cov /= cloud_roi->size();

        Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> solver(cov);
        Eigen::Vector3f principal_dir = solver.eigenvectors().col(2);

        // 保证方向指向前方（x 正方向）
        if (principal_dir.x() < 0)
            principal_dir = -principal_dir;

        principal_dir.normalize();

        // 4️⃣ 发布方向
        geometry_msgs::Vector3Stamped dir_msg;
        dir_msg.header = msg->header;
        dir_msg.vector.x = principal_dir.x();
        dir_msg.vector.y = principal_dir.y();
        dir_msg.vector.z = principal_dir.z();

        pub_dir_.publish(dir_msg);
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "pca_direction_node");
    ros::NodeHandle nh("~");

    PCADirectionEstimator estimator(nh);

    ros::spin();
    return 0;
}
