#include <ros/ros.h>
#include <nav_msgs/Odometry.h>

class OdomRepublisher
{
public:
    OdomRepublisher()
    {
        ros::NodeHandle nh;
        ros::NodeHandle pnh("~");

        // 可从参数服务器设置输入输出话题与frame
        pnh.param<std::string>("input_topic", input_topic_, "/Odometry");
        pnh.param<std::string>("output_topic", output_topic_, "/local_odom");
        pnh.param<std::string>("frame_id", frame_id_, "map");
        pnh.param<std::string>("child_frame_id", child_frame_id_, "base_link");

        pub_ = nh.advertise<nav_msgs::Odometry>(output_topic_, 10);
        sub_ = nh.subscribe(input_topic_, 10, &OdomRepublisher::odomCallback, this);

        ROS_INFO("[odom_republisher] Listening on %s, publishing to %s",
                 input_topic_.c_str(), output_topic_.c_str());
    }

private:
    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
    {
        nav_msgs::Odometry new_msg = *msg;
        new_msg.header.frame_id = frame_id_;
        new_msg.child_frame_id = child_frame_id_;
        pub_.publish(new_msg);
    }

    ros::Subscriber sub_;
    ros::Publisher pub_;
    std::string input_topic_, output_topic_;
    std::string frame_id_, child_frame_id_;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "odom_republisher");
    OdomRepublisher node;
    ros::spin();
    return 0;
}
