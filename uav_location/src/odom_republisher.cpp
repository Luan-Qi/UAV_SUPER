/**
 * @file odom_republisher.cpp
 * @brief 里程计话题重发布节点 — 修改 frame_id 后重新发布 Odometry 消息。
 *
 * @details
 * 功能：将输入的里程计话题重命名 frame_id/child_frame_id 后发布到新话题。
 * 典型用途：
 *   - 将不同坐标系下的里程计统一到同一 frame 体系
 *   - 为下游节点提供具有特定 frame_id 的里程计副本
 *
 * 话题：
 *   订阅（可配置）输入 Odometry
 *   发布（可配置）输出 Odometry
 *
 * 使用：
 *   rosrun uav_location odom_republisher  \
 *       _input_topic:=/Odometry  \
 *       _output_topic:=/local_odom  \
 *       _frame_id:=map  _child_frame_id:=base_link
 *
 * @note 仅修改 header.frame_id 和 child_frame_id，不解算/缓存/重放数据。
 */

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>

/**
 * @class OdomRepublisher
 * @brief 订阅一个 Odometry 话题，修改坐标系后发布到另一个话题。
 */
class OdomRepublisher
{
public:
    OdomRepublisher()
    {
        ros::NodeHandle nh;
        ros::NodeHandle pnh("~");

        // ---- 参数加载：话题与坐标系 ----
        pnh.param<std::string>("input_topic",    input_topic_,    "/Odometry");
        pnh.param<std::string>("output_topic",   output_topic_,   "/local_odom");
        pnh.param<std::string>("frame_id",       frame_id_,       "map");
        pnh.param<std::string>("child_frame_id", child_frame_id_, "base_link");

        // ---- 发布器与订阅器 ----
        pub_ = nh.advertise<nav_msgs::Odometry>(output_topic_, 10);
        sub_ = nh.subscribe(input_topic_, 10, &OdomRepublisher::odomCallback, this);

        ROS_INFO("[odom_republisher] Listening on %s, publishing to %s",
                 input_topic_.c_str(), output_topic_.c_str());
    }

private:
    /**
     * @brief 里程计回调：复制消息并改写坐标系后发布。
     */
    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
    {
        nav_msgs::Odometry new_msg = *msg;
        new_msg.header.frame_id = frame_id_;
        new_msg.child_frame_id  = child_frame_id_;
        pub_.publish(new_msg);
    }

    ros::Subscriber sub_;
    ros::Publisher  pub_;
    std::string input_topic_, output_topic_;
    std::string frame_id_, child_frame_id_;
};

// ============================================================================
// 主函数
// ============================================================================

int main(int argc, char** argv)
{
    ros::init(argc, argv, "odom_republisher");
    OdomRepublisher node;
    ros::spin();
    return 0;
}
