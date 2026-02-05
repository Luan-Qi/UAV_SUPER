#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <vector>
#include <cmath>

struct Vec3 {
    double x, y, z;
    Vec3() : x(0), y(0), z(0) {}
    Vec3(double _x, double _y, double _z) : x(_x), y(_y), z(_z) {}
};

inline Vec3 toVec3(const geometry_msgs::Point &p){ return Vec3(p.x, p.y, p.z); }
inline Vec3 toVec3(const geometry_msgs::PoseStamped &ps){ return Vec3(ps.pose.position.x, ps.pose.position.y, ps.pose.position.z); }

inline Vec3 operator-(const Vec3 &a, const Vec3 &b){ return Vec3(a.x-b.x, a.y-b.y, a.z-b.z); }
inline Vec3 operator+(const Vec3 &a, const Vec3 &b){ return Vec3(a.x+b.x, a.y+b.y, a.z+b.z); }
inline Vec3 operator*(const Vec3 &a, double s){ return Vec3(a.x*s, a.y*s, a.z*s); }

inline double norm(const Vec3 &a){ return std::sqrt(a.x*a.x + a.y*a.y + a.z*a.z); }

inline double dot(const Vec3 &a, const Vec3 &b){ return a.x*b.x + a.y*b.y + a.z*b.z; }
// cross product magnitude
inline Vec3 crossv(const Vec3 &a, const Vec3 &b)
{
    return Vec3(a.y*b.z - a.z*b.y,
                a.z*b.x - a.x*b.z,
                a.x*b.y - a.y*b.x);
}

inline Vec3 normalize(const Vec3 &a)
{
    double n = norm(a);
    if (n <= 1e-12) return Vec3(0,0,0);
    return Vec3(a.x/n, a.y/n, a.z/n);
}

// Compute perpendicular distance from point p to line through p0 with direction dir (dir must be normalized)
inline double lateralDistanceToLine(const Vec3 &p0, const Vec3 &dir, const Vec3 &p)
{
    Vec3 v = p - p0;
    // cross product magnitude divided by |dir| (dir normalized -> /1)
    Vec3 c = crossv(v, dir);
    return norm(c); // since |dir|==1, this equals distance
}

class PathSimplifier 
{
public:
    PathSimplifier(ros::NodeHandle &nh): nh_(nh) 
    {
        nh_.param<std::string>("input_topic", input_topic_, "/global_path");
        nh_.param<std::string>("output_topic", output_topic_, "/global_path_simplified");
        nh_.param<int>("queue_size", queue_size_, 1);
        nh_.param<double>("max_deviation", max_deviation_, 0.2);
        nh_.param<double>("dist_scale", dist_scale_, 0.5);
        nh_.param<double>("min_segment_length", min_segment_length_, 0.05);
        nh_.param<bool>("publish_header_frame_id", publish_header_frame_id_, true);
        nh_.param<double>("publish_rate", publish_rate_, 0.0);

        sub_ = nh_.subscribe(input_topic_, queue_size_, &PathSimplifier::pathCallback, this);
        pub_ = nh_.advertise<nav_msgs::Path>(output_topic_, 1);

        if (publish_rate_ > 0.0) 
        {
            timer_ = nh_.createTimer(ros::Duration(1.0 / publish_rate_), &PathSimplifier::timerCallback, this);
        }
    }

    void pathCallback(const nav_msgs::Path::ConstPtr &msg)
    {
        if (msg->poses.empty())
        {
            ROS_WARN("Received empty path");
            return;
        }
        last_input_ = *msg;

        nav_msgs::Path simplified;
        simplifyPath(last_input_, simplified);

        if (publish_header_frame_id_) simplified.header.frame_id = last_input_.header.frame_id;
        simplified.header.stamp = ros::Time::now();
        pub_.publish(simplified);
    }

    void timerCallback(const ros::TimerEvent &)
    {
        if (publish_rate_ <= 0.0) return;
        if (last_input_.poses.empty()) return;

        nav_msgs::Path simplified;
        simplifyPath(last_input_, simplified);

        if (publish_header_frame_id_) simplified.header.frame_id = last_input_.header.frame_id;
        simplified.header.stamp = ros::Time::now();
        pub_.publish(simplified);
    }

private:
    ros::NodeHandle nh_;
    ros::Subscriber sub_;
    ros::Publisher pub_;
    ros::Timer timer_;

    // params
    std::string input_topic_, output_topic_;
    int queue_size_;
    double max_deviation_;
    double dist_scale_;
    double min_segment_length_;
    bool publish_header_frame_id_;
    double publish_rate_;

    nav_msgs::Path last_input_;

    void simplifyPath(const nav_msgs::Path &in, nav_msgs::Path &out)
    {
        out.poses.clear();
        const auto &pts = in.poses;
        if (pts.empty()) return;
        size_t N = pts.size();
        // Always include start
        out.poses.push_back(pts.front());

        size_t s = 0; // start index of current segment
        while (s < N - 1)
        {
            // choose next point as initial direction
            size_t next = s + 1;
            Vec3 p_s = toVec3(pts[s]);
            Vec3 p_next = toVec3(pts[next]);
            Vec3 dir = p_next - p_s;
            double dir_norm = norm(dir);
            if (dir_norm < 1e-12)
            {
                // consecutive identical points: skip
                s = next;
                continue;
            }
            dir = normalize(dir);

            size_t j = next + 1;
            size_t last_good = next; // last point that still fits line

            for (; j < N; ++j)
            {
                Vec3 p_j = toVec3(pts[j]);
                double distance = norm(p_j - p_s);
                double allowed = max_deviation_ / (1.0 + dist_scale_ * distance);
                double lateral = lateralDistanceToLine(p_s, dir, p_j);

                if (lateral <= allowed)
                {
                    last_good = j;
                    continue;
                }
                else
                {
                    // --- Simple enhanced logic ---
                    if (j == next + 1)
                    {
                        Vec3 new_dir = p_j - p_s;
                        if (norm(new_dir) > 1e-12)
                        {
                            dir = normalize(new_dir);
                            // recompute deviation using new direction
                            lateral = lateralDistanceToLine(p_s, dir, p_j);
                            if (lateral <= allowed)
                            {
                                last_good = j;
                                continue;
                            }
                        }
                    }
                    break;
                }
            }

            // If nothing beyond next fits (last_good == next), then we still make a short segment to next
            size_t seg_end = last_good;

            // Optionally ensure min length: if seg too short and we have more points, extend to at least next
            double seg_len = norm(toVec3(pts[seg_end]) - p_s);
            if (seg_len < min_segment_length_ && seg_end + 1 < N)
            {
                // try to extend one more point (to avoid degenerate tiny segments)
                seg_end = std::min(seg_end + 1, N - 1);
            }

            // Append segment end as a key vertex (unless it's equal to last appended)
            if (!out.poses.empty())
            {
                const auto &last_app = out.poses.back();
                const auto &candidate = pts[seg_end];
                if (!posesEqual(last_app, candidate)) 
                {
                    out.poses.push_back(candidate);
                }
            }
            else 
            {
                out.poses.push_back(pts[seg_end]);
            }

            // Next segment starts from seg_end
            if (seg_end == s) {
                // avoid infinite loop, advance at least by 1
                s = s + 1;
            } else {
                s = seg_end;
            }
        }

        // Ensure final point is included
        if (!out.poses.empty())
        {
            const auto &last_app = out.poses.back();
            const auto &final_pt = pts.back();
            if (!posesEqual(last_app, final_pt)) out.poses.push_back(final_pt);
        }
    }

    static bool posesEqual(const geometry_msgs::PoseStamped &a, const geometry_msgs::PoseStamped &b)
    {
        const double eps = 1e-6;
        return (std::fabs(a.pose.position.x - b.pose.position.x) < eps &&
                std::fabs(a.pose.position.y - b.pose.position.y) < eps &&
                std::fabs(a.pose.position.z - b.pose.position.z) < eps);
    }
};

int main(int argc, char **argv){
  ros::init(argc, argv, "simplify_path_node");
  ros::NodeHandle nh("~");
  PathSimplifier ps(nh);
  ros::spin();
  return 0;
}
