#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/pcd_io.h>

#include <memory>
#include <random>
#include <chrono>
#include <cmath>
#include <limits>

using Cloud = pcl::PointCloud<pcl::PointXYZI>;

/* ============================================================
 * 工具函数
 * ============================================================ */

inline pcl::PointXYZI makePointXYZI(
    float x, float y, float z,
    float intensity = 1.0f
) {
    pcl::PointXYZI pt;
    pt.x = x;
    pt.y = y;
    pt.z = z;
    pt.intensity = intensity;
    return pt;
}

double randUniform(double a, double b)
{
    static std::mt19937 gen{std::random_device{}()};
    std::uniform_real_distribution<double> dist(a, b);
    return dist(gen);
}

double randNormal(double mean, double stddev)
{
    static std::mt19937 gen{std::random_device{}()};
    std::normal_distribution<double> dist(mean, stddev);
    return dist(gen);
}

/* ============================================================
 * 生成不同形状
 * ============================================================ */

class ShapeBase
{
public:
    virtual ~ShapeBase() = default;

    virtual void generate(Cloud::Ptr cloud) = 0;

protected:
    double resolution_;
    double length_;
    double width_;
    double height_;
    double radius_;
    double thickness_;
    double cx, cy, cz;

    void loadCommonParams(ros::NodeHandle& nh)
    {
        nh.param("resolution", resolution_, 0.1);
        nh.param("length", length_, 10.0);
        nh.param("width", width_, 5.0);
        nh.param("height", height_, 3.0);
        nh.param("radius", radius_, 3.0);
        nh.param("thickness", thickness_, 0.1);
        nh.param("center_x", cx, 0.0);
        nh.param("center_y", cy, 0.0);
        nh.param("center_z", cz, 0.0);
    }
};

class CubeShape : public ShapeBase
{
public:
    CubeShape(ros::NodeHandle& nh)
    {
        loadCommonParams(nh);
    }

    void generate(Cloud::Ptr cloud) override
    {
        for (double x = -length_/2; x <= length_/2; x += resolution_)
        for (double y = -width_/2;  y <= width_/2;  y += resolution_)
        for (double z = -height_/2; z <= height_/2; z += resolution_)
        {
            bool on_surface =
                fabs(fabs(x) - length_/2) < thickness_ ||
                fabs(fabs(y) - width_/2)  < thickness_ ||
                fabs(fabs(z) - height_/2) < thickness_;

            if (on_surface)
                cloud->push_back(
                    makePointXYZI(static_cast<float>(x + cx), 
                                  static_cast<float>(y + cy), 
                                  static_cast<float>(z + cz)));

        }
    }
};

class SphereShape : public ShapeBase
{
public:
    SphereShape(ros::NodeHandle& nh)
    {
        loadCommonParams(nh);
    }

    void generate(Cloud::Ptr cloud) override
    {
        for (double x = -radius_; x <= radius_; x += resolution_)
        for (double y = -radius_; y <= radius_; y += resolution_)
        for (double z = -radius_; z <= radius_; z += resolution_)
        {
            double r = sqrt(x*x + y*y + z*z);
            if (fabs(r - radius_) < thickness_)
                cloud->push_back(
                    makePointXYZI(static_cast<float>(x + cx), 
                                  static_cast<float>(y + cy), 
                                  static_cast<float>(z + cz)));
        }
    }
};

class BoxCorridorShape : public ShapeBase
{
public:
    BoxCorridorShape(ros::NodeHandle& nh)
    {
        loadCommonParams(nh);
    }

    void generate(Cloud::Ptr cloud) override
    {
        for (double x = 0; x <= length_; x += resolution_)
        for (double y = -width_/2; y <= width_/2; y += resolution_)
        for (double z = -height_/2; z <= height_/2; z += resolution_)
        {
            bool wall =
                fabs(fabs(y) - width_/2) < thickness_ ||
                fabs(fabs(z) - height_/2) < thickness_;

            bool cap =
                x < thickness_ || fabs(x - length_) < thickness_;

            if (wall || cap)
                cloud->push_back(
                    makePointXYZI(static_cast<float>(x + cx), 
                                  static_cast<float>(y + cy), 
                                  static_cast<float>(z + cz)));
        }
    }
};

class CylinderCorridorShape : public ShapeBase
{
public:
    CylinderCorridorShape(ros::NodeHandle& nh)
    {
        loadCommonParams(nh);
    }

    void generate(Cloud::Ptr cloud) override
    {
        for (double x = 0; x <= length_; x += resolution_)
        for (double y = -radius_; y <= radius_; y += resolution_)
        for (double z = -radius_; z <= radius_; z += resolution_)
        {
            double r = sqrt(y*y + z*z);

            bool wall = fabs(r - radius_) < thickness_;
            bool cap  = x < thickness_ || fabs(x - length_) < thickness_ && (r <= radius_);

            if (wall || cap)
                cloud->push_back(
                    makePointXYZI(static_cast<float>(x + cx), 
                                  static_cast<float>(y + cy), 
                                  static_cast<float>(z + cz)));
        }
    }
};

class ShapeFactory
{
public:
    static std::unique_ptr<ShapeBase> create(
        const std::string& type, ros::NodeHandle& nh)
    {
        if (type == "cube") return std::make_unique<CubeShape>(nh);
        if (type == "sphere") return std::make_unique<SphereShape>(nh);
        if (type == "box_corridor") return std::make_unique<BoxCorridorShape>(nh);
        if (type == "cylinder_corridor") return std::make_unique<CylinderCorridorShape>(nh);

        ROS_ERROR("Unknown shape type: %s", type.c_str());
        return nullptr;
    }
};

/* ============================================================
 * 后处理
 * ============================================================ */

void applyNoise(Cloud::Ptr cloud, double stddev)
{
    for (auto& p : cloud->points)
    {
        p.x += randNormal(0, stddev);
        p.y += randNormal(0, stddev);
        p.z += randNormal(0, stddev);
    }
}

void applySparse(Cloud::Ptr cloud, double keep_ratio)
{
    Cloud tmp;
    for (auto& p : cloud->points)
        if (randUniform(0,1) < keep_ratio)
            tmp.push_back(p);
    cloud->swap(tmp);
}

void applyHole(Cloud::Ptr cloud,
               double hx, double hy, double hz, double hr)
{
    Cloud tmp;
    for (auto& p : cloud->points)
    {
        double d = std::sqrt(
            (p.x-hx)*(p.x-hx) +
            (p.y-hy)*(p.y-hy) +
            (p.z-hz)*(p.z-hz));
        if (d > hr)
            tmp.push_back(p);
    }
    cloud->swap(tmp);
}

/* ============================================================
 * 主函数
 * ============================================================ */

int main(int argc, char** argv)
{
    ros::init(argc, argv, "pcd_map_generator");
    ros::NodeHandle nh("~");

    /* ---------- 参数 ---------- */
    std::string shape_type;
    std::string pcd_path;
    bool pub_cloud;
    nh.param("shape_type", shape_type, std::string("cube"));
    nh.param("pcd_output_path", pcd_path, std::string("map.pcd"));
    nh.param("publish_cloud", pub_cloud, true);

    /* 后处理参数 */
    bool enable_noise, enable_sparse, enable_hole;
    nh.param("enable_noise", enable_noise, false);
    nh.param("enable_sparse", enable_sparse, false);
    nh.param("enable_hole", enable_hole, false);

    double noise_std, sparse_keep;
    nh.param("noise_std", noise_std, 0.02);
    nh.param("sparse_keep_ratio", sparse_keep, 0.7);

    double hole_x, hole_y, hole_z, hole_r;
    nh.param("hole_x", hole_x, 0.0);
    nh.param("hole_y", hole_y, 0.0);
    nh.param("hole_z", hole_z, 0.0);
    nh.param("hole_radius", hole_r, 1.0);

    /* ---------- 生成 ---------- */
    auto t0 = std::chrono::steady_clock::now();

    auto shape = ShapeFactory::create(shape_type, nh);
    if (!shape) return 1;

    Cloud::Ptr cloud(new Cloud);

    shape->generate(cloud);

    cloud->width = cloud->size();
    cloud->height = 1;
    cloud->is_dense = true;

    /* ---------- 后处理 ---------- */
    if (enable_noise)  applyNoise(cloud, noise_std);
    if (enable_sparse) applySparse(cloud, sparse_keep);
    if (enable_hole)   applyHole(cloud, hole_x, hole_y, hole_z, hole_r);

    auto t1 = std::chrono::steady_clock::now();
    double time_ms =
        std::chrono::duration<double,std::milli>(t1 - t0).count();

    ros::Publisher pub = nh.advertise<sensor_msgs::PointCloud2>("map", 10, true);
    if(pub_cloud)
    {
        sensor_msgs::PointCloud2 msg;
        pcl::toROSMsg(*cloud, msg);
        msg.header.frame_id = "map";
        pub.publish(msg);
        ROS_INFO("Map generated and published");
        ros::spinOnce();
    }

    pcl::io::savePCDFileBinary(pcd_path, *cloud);

    /* ---------- 统计 ---------- */

    size_t N = cloud->points.size();

    float minx=1e9, miny=1e9, minz=1e9;
    float maxx=-1e9, maxy=-1e9, maxz=-1e9;
    for (auto& p : cloud->points)
    {
        minx = std::min(minx, p.x);
        miny = std::min(miny, p.y);
        minz = std::min(minz, p.z);
        maxx = std::max(maxx, p.x);
        maxy = std::max(maxy, p.y);
        maxz = std::max(maxz, p.z);
    }

    ROS_INFO_STREAM("\n================= PCD MAP ================="
        << "\nShape type      : " << shape_type
        << "\nPoint count     : " << N
        << "\nResolution (m)  : " << nh.param("resolution", 0.1)
        << "\nGeneration time : " << time_ms << " ms"
        << "\nAABB size       : "
        << (maxx-minx) << " x "
        << (maxy-miny) << " x "
        << (maxz-minz)
        << "\nPCD file        : " << pcd_path
        << "\n============================================");

    ros::spin();
}

