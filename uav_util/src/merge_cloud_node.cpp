// merge_cloud_node.cpp
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/transforms.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <Eigen/Dense>

class MergeCloudNode
{
public:
	MergeCloudNode(ros::NodeHandle& nh, ros::NodeHandle& pnh)
		: nh_(nh), pnh_(pnh)
	{
		// 读取参数（有默认值）
		pnh_.param<std::string>("cloud1_topic", cloud1_topic_, "/livox/lidar1");
		pnh_.param<std::string>("cloud2_topic", cloud2_topic_, "/livox/lidar2");

		pnh_.param<double>("roll1", roll1_, 0.0);
		pnh_.param<double>("pitch1", pitch1_, 0.0);
		pnh_.param<double>("yaw1", yaw1_, 0.0);
		pnh_.param<double>("tx1", tx1_, 0.0);
		pnh_.param<double>("ty1", ty1_, 0.0);
		pnh_.param<double>("tz1", tz1_, 0.0);

		pnh_.param<double>("roll2", roll2_, 0.0);
		pnh_.param<double>("pitch2", pitch2_, 0.0);
		pnh_.param<double>("yaw2", yaw2_, 0.0);
		pnh_.param<double>("tx2", tx2_, 0.0);
		pnh_.param<double>("ty2", ty2_, 0.0);
		pnh_.param<double>("tz2", tz2_, 0.0);

		// message_filters subscribers
		cloud1_sub_.subscribe(nh_, cloud1_topic_, 10);
		cloud2_sub_.subscribe(nh_, cloud2_topic_, 10);

		sync_.reset(new message_filters::Synchronizer<SyncPolicy>(SyncPolicy(10), cloud1_sub_, cloud2_sub_));
		sync_->registerCallback(boost::bind(&MergeCloudNode::syncCallback, this, _1, _2));

		merged_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("merged_cloud", 10);

		ROS_INFO("MergeCloudNode started. Subscribing to [%s] and [%s].",
				 cloud1_topic_.c_str(), cloud2_topic_.c_str());
	}

private:
	void syncCallback(const sensor_msgs::PointCloud2ConstPtr& cloud1_msg,
					  const sensor_msgs::PointCloud2ConstPtr& cloud2_msg)
	{
		// 转换到 PCL 点云
		pcl::PointCloud<pcl::PointXYZI>::Ptr cloud1(new pcl::PointCloud<pcl::PointXYZI>);
		pcl::PointCloud<pcl::PointXYZI>::Ptr cloud2(new pcl::PointCloud<pcl::PointXYZI>);

		pcl::fromROSMsg(*cloud1_msg, *cloud1);
		pcl::fromROSMsg(*cloud2_msg, *cloud2);

		// 计算变换矩阵
		Eigen::Matrix4f transform1 = Eigen::Matrix4f::Identity();
		Eigen::Matrix4f transform2 = Eigen::Matrix4f::Identity();
		setTransformMatrix(transform1, static_cast<float>(roll1_), static_cast<float>(pitch1_), static_cast<float>(yaw1_),
							static_cast<float>(tx1_), static_cast<float>(ty1_), static_cast<float>(tz1_));
		setTransformMatrix(transform2, static_cast<float>(roll2_), static_cast<float>(pitch2_), static_cast<float>(yaw2_),
							static_cast<float>(tx2_), static_cast<float>(ty2_), static_cast<float>(tz2_));

		pcl::PointCloud<pcl::PointXYZI>::Ptr cloud1_tf(new pcl::PointCloud<pcl::PointXYZI>);
		pcl::PointCloud<pcl::PointXYZI>::Ptr cloud2_tf(new pcl::PointCloud<pcl::PointXYZI>);

		pcl::transformPointCloud(*cloud1, *cloud1_tf, transform1);
		pcl::transformPointCloud(*cloud2, *cloud2_tf, transform2);

		// 合并点云
		pcl::PointCloud<pcl::PointXYZI>::Ptr merged(new pcl::PointCloud<pcl::PointXYZI>);
		*merged = *cloud1_tf + *cloud2_tf;

		// 转回 ROS msg 并发布
		sensor_msgs::PointCloud2 out_msg;
		pcl::toROSMsg(*merged, out_msg);
		// 使用 cloud1 的 header 时间或当前时间，根据需要我使用当前时间
		out_msg.header.stamp = ros::Time::now();
		out_msg.header.frame_id = "map";

		merged_pub_.publish(out_msg);
		ROS_INFO_STREAM_THROTTLE(1.0, "Published merged cloud, points: " << merged->size());
	}

	void setTransformMatrix(Eigen::Matrix4f& transform, float roll, float pitch, float yaw, float tx, float ty, float tz)
	{
		Eigen::AngleAxisf rollAngle(roll, Eigen::Vector3f::UnitX());
		Eigen::AngleAxisf pitchAngle(pitch, Eigen::Vector3f::UnitY());
		Eigen::AngleAxisf yawAngle(yaw, Eigen::Vector3f::UnitZ());
		Eigen::Quaternion<float> q = yawAngle * pitchAngle * rollAngle;
		Eigen::Matrix3f rot = q.matrix();
		transform.setIdentity();
		transform.block<3,3>(0,0) = rot;
		transform(0,3) = tx;
		transform(1,3) = ty;
		transform(2,3) = tz;
	}

	// ROS
	ros::NodeHandle nh_, pnh_;

	// message_filters
	typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2,
															sensor_msgs::PointCloud2> SyncPolicy;
	message_filters::Subscriber<sensor_msgs::PointCloud2> cloud1_sub_;
	message_filters::Subscriber<sensor_msgs::PointCloud2> cloud2_sub_;
	boost::shared_ptr<message_filters::Synchronizer<SyncPolicy>> sync_;

	// publisher
	ros::Publisher merged_pub_;

	// params
	std::string cloud1_topic_, cloud2_topic_;
	double roll1_, pitch1_, yaw1_, tx1_, ty1_, tz1_;
	double roll2_, pitch2_, yaw2_, tx2_, ty2_, tz2_;
};

int main(int argc, char** argv)
{
	ros::init(argc, argv, "merge_cloud_node");
	ros::NodeHandle nh;
	ros::NodeHandle pnh("~");

	MergeCloudNode node(nh, pnh);
	ros::spin();
	return 0;
}
