#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <vector>
#include <cmath>
#include <algorithm>

class PathSimplifier
{
public:
    PathSimplifier()
    {
        // Load parameters
        ros::NodeHandle nh("~");
        nh.param<std::string>("input_topic", input_topic_, "/input_topic");
        nh.param<std::string>("output_topic", output_topic_, "/output_topic");
        nh.param<double>("simplification_threshold", threshold_, 0.15);
        nh.param<double>("min_segment_length", min_seg_len_, 0.5);

        sub_ = nh.subscribe(input_topic_, 1, &PathSimplifier::pathCallback, this);
        pub_ = nh.advertise<nav_msgs::Path>(output_topic_, 1, true);

        ROS_INFO("[PathSimplify] PathSimplifier initialized.");
        // ROS_INFO("Input: %s, Output: %s", input_topic_.c_str(), output_topic_.c_str());
        // ROS_INFO("Threshold: %.3f m, Min segment length: %.3f m", threshold_, min_seg_len_);
    }

private:
    void pathCallback(const nav_msgs::Path::ConstPtr& msg)
    {
        if (msg->poses.empty())
        {
            pub_.publish(*msg);
            return;
        }

        int max_recursion_depth = 0; // trace for max recursion depth
        ros::WallTime start_time = ros::WallTime::now();
        
        size_t input_size = msg->poses.size();
        std::vector<geometry_msgs::Point> original_points;
        original_points.reserve(input_size);
        for (const auto& pose : msg->poses)
        {
            original_points.push_back(pose.pose.position);
        }

        std::vector<geometry_msgs::Point> simplified;
        douglasPeucker(original_points, 0, original_points.size() - 1, threshold_, simplified, 0, max_recursion_depth);

        // Ensure start and end are included
        if (simplified.empty())
        {
            simplified.push_back(original_points.front());
            simplified.push_back(original_points.back());
        } else
        {
            if (simplified.front() != original_points.front())
                simplified.insert(simplified.begin(), original_points.front());
            if (simplified.back() != original_points.back())
                simplified.push_back(original_points.back());
        }

        // Optional: Merge very short segments
        simplified = mergeShortSegments(simplified, min_seg_len_);
        size_t output_size = simplified.size();

        // Convert back to Path
        nav_msgs::Path out_path = *msg;
        out_path.poses.clear();
        for (const auto& pt : simplified)
        {
            geometry_msgs::PoseStamped ps;
            ps.header = msg->header;
            ps.pose.position = pt;
            ps.pose.orientation.w = 1.0; // Identity orientation
            out_path.poses.push_back(ps);
        }

        pub_.publish(out_path);

        ros::WallTime end_time = ros::WallTime::now();
        double execution_time_ms = (end_time - start_time).toSec() * 1000.0; // 转换为毫秒
        double reduction_percent = 0.0;
        if (input_size > 0) {
            reduction_percent = (1.0 - (static_cast<double>(output_size) / input_size)) * 100.0;
        }

        // 3. 打印 Debug 信息
        ROS_INFO_STREAM("[PathSimplify] Max Depth: " << max_recursion_depth);
        ROS_INFO_STREAM("[PathSimplify] In: " << input_size 
                        << " -> Out: " << output_size 
                        << " (" << std::fixed << std::setprecision(2) << reduction_percent << "%)"
                        << " | Time: " << std::setprecision(3) << execution_time_ms << " ms");
    }

    // Douglas-Peucker in 3D
    void douglasPeucker(const std::vector<geometry_msgs::Point>& points,
                        size_t start,
                        size_t end,
                        double epsilon,
                        std::vector<geometry_msgs::Point>& simplified, 
                        int current_depth,
                        int& max_depth_tracker)
    {
        if (current_depth > max_depth_tracker)
        {
            max_depth_tracker = current_depth;
        }
        if (end - start < 1) return;

        double dmax = 0.0;
        size_t index = 0;

        geometry_msgs::Point A = points[start];
        geometry_msgs::Point B = points[end];

        for (size_t i = start + 1; i < end; ++i)
        {
            double dist = pointToLineDistance(points[i], A, B);
            if (dist > dmax)
            {
                dmax = dist;
                index = i;
            }
        }

        if (dmax > epsilon)
        {
            // Recursive call
            std::vector<geometry_msgs::Point> rec_results1, rec_results2;
            douglasPeucker(points, start, index, epsilon, rec_results1, current_depth + 1, max_depth_tracker);
            douglasPeucker(points, index, end, epsilon, rec_results2, current_depth + 1, max_depth_tracker);

            // Combine results (avoid duplicating index point)
            simplified.insert(simplified.end(), rec_results1.begin(), rec_results1.end());
            if (!rec_results2.empty()) simplified.insert(simplified.end(), rec_results2.begin() + 1, rec_results2.end());
        }
        else
        {
            // Add endpoints only
            simplified.push_back(points[start]);
            simplified.push_back(points[end]);
        }
    }

    // Compute perpendicular distance from point P to line AB in 3D
    double pointToLineDistance(const geometry_msgs::Point& P,
                               const geometry_msgs::Point& A,
                               const geometry_msgs::Point& B)
    {
        double ax = B.x - A.x;
        double ay = B.y - A.y;
        double az = B.z - A.z;

        double px = P.x - A.x;
        double py = P.y - A.y;
        double pz = P.z - A.z;

        double cross_x = ay * pz - az * py;
        double cross_y = az * px - ax * pz;
        double cross_z = ax * py - ay * px;

        double cross_norm = std::sqrt(cross_x * cross_x + cross_y * cross_y + cross_z * cross_z);
        double ab_norm = std::sqrt(ax * ax + ay * ay + az * az);

        if (ab_norm < 1e-8) return 0.0; // A and B are the same

        return cross_norm / ab_norm;
    }

    // Merge consecutive segments shorter than min_len
    std::vector<geometry_msgs::Point> mergeShortSegments(const std::vector<geometry_msgs::Point>& pts,
                                                         double min_len)
    {
        if (pts.size() <= 2) return pts;

        std::vector<geometry_msgs::Point> result;
        result.push_back(pts[0]);

        for (size_t i = 1; i < pts.size() - 1; ++i)
        {
            double dx = pts[i].x - result.back().x;
            double dy = pts[i].y - result.back().y;
            double dz = pts[i].z - result.back().z;
            double dist = std::sqrt(dx*dx + dy*dy + dz*dz);

            if (dist >= min_len)
            {
                result.push_back(pts[i]);
            }
            // else: skip this point (merge into next segment)
        }

        // Always keep the last point
        if (result.back() != pts.back())
        {
            result.push_back(pts.back());
        }

        return result;
    }

    ros::Subscriber sub_;
    ros::Publisher pub_;
    std::string input_topic_;
    std::string output_topic_;
    double threshold_;
    double min_seg_len_;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "path_simplifier");
    PathSimplifier simplifier;
    ros::spin();
    return 0;
}