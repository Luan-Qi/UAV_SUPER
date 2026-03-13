#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>

class PathDownsampler
{
private:
    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;

    ros::Subscriber path_sub_;
    ros::Publisher path_pub_;

    // parameters
    int step_;
    double distance_thresh_;
    std::string mode_;

public:
    PathDownsampler():pnh_("~")
    {
        // 读取参数
        pnh_.param("step", step_, 5);                           // 每隔step个点取一个点
        pnh_.param("distance_thresh", distance_thresh_, 0.5);   // 两个点之间的距离小于distance_thresh时，取一个点
        pnh_.param<std::string>("mode", mode_, "step");         // 取点模式，step表示每隔step个点取一个点，distance表示距离小于distance_thresh时取一个点

        std::string input_topic, output_topic;
        pnh_.param<std::string>("input_topic", input_topic, "/input_path");
        pnh_.param<std::string>("output_topic", output_topic, "/downsampled_path");

        path_sub_ = nh_.subscribe(input_topic, 1, &PathDownsampler::pathCallback, this);

        path_pub_ = nh_.advertise<nav_msgs::Path>(output_topic, 1, true);

        ROS_INFO("[PathSimplify] Path Downsampler initialized");
    }

    double distance(const geometry_msgs::PoseStamped& a,
                    const geometry_msgs::PoseStamped& b)
    {
        double dx = a.pose.position.x - b.pose.position.x;
        double dy = a.pose.position.y - b.pose.position.y;
        double dz = a.pose.position.z - b.pose.position.z;

        return sqrt(dx*dx + dy*dy + dz*dz);
    }

    nav_msgs::Path downsampleStep(const nav_msgs::Path& path)
    {
        nav_msgs::Path result;
        result.header = path.header;

        for(size_t i = 0; i < path.poses.size(); i += step_)
        {
            result.poses.push_back(path.poses[i]);
        }

        //保证最后一个点存在
        if(!path.poses.empty())
        {
           result.poses.push_back(path.poses.back());
        }

        return result;
    }

    nav_msgs::Path downsampleDistance(const nav_msgs::Path& path)
    {
        nav_msgs::Path result;
        result.header = path.header;

        if(path.poses.empty())
            return result;

        result.poses.push_back(path.poses.front());

        for(size_t i = 1; i < path.poses.size(); i++)
        {
            if(distance(path.poses[i], result.poses.back()) > distance_thresh_)
            {
                result.poses.push_back(path.poses[i]);
            }
        }

        return result;
    }

    void pathCallback(const nav_msgs::PathConstPtr& msg)
    {
        nav_msgs::Path output;

        if(mode_ == "step")
        {
            output = downsampleStep(*msg);
            ROS_INFO("[PathSimplify] Downsampled step: %zu -> %zu", msg->poses.size(), output.poses.size());
        }
        else if(mode_ == "distance")
        {
            output = downsampleDistance(*msg);
            ROS_INFO("[PathSimplify] Downsampled distance: %zu -> %zu", msg->poses.size(), output.poses.size());
        }
        else
        {
            ROS_WARN("[PathSimplify] Unknown mode, using step mode");
            output = downsampleStep(*msg);
        }

        path_pub_.publish(output);
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "path_downsampler");
    PathDownsampler node;

    ros::spin();

    return 0;
}