#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/common.h>

#include <memory>
#include <random>
#include <chrono>
#include <cmath>
#include <limits>
#include <Eigen/Dense>

/* ============================================================
 *   核心数学与转换工具 (Transform Utils)
 * ============================================================ */

struct TransformParams
{
    double cx, cy, cz;       // Translation
    double roll, pitch, yaw; // Rotation (Radians)
    Eigen::Affine3f transform_matrix;

    void updateMatrix()
    {
        Eigen::Translation3f translation(cx, cy, cz);
        Eigen::AngleAxisf rot_x(roll, Eigen::Vector3f::UnitX());
        Eigen::AngleAxisf rot_y(pitch, Eigen::Vector3f::UnitY());
        Eigen::AngleAxisf rot_z(yaw, Eigen::Vector3f::UnitZ());
        
        // 变换顺序：先旋转，再平移 (R * p + T)
        transform_matrix = translation * rot_z * rot_y * rot_x;
    }
};

// 生成一个在 [a, b] 范围内的均匀分布的随机数
// @param a 随机数范围的下界
// @param b 随机数范围的上界
// @return 返回一个在 [a, b] 范围内的随机数
double randUniform(double a, double b)
{
    static std::mt19937 gen{std::random_device{}()}; // 静态随机数生成器
    std::uniform_real_distribution<double> dist(a, b); // 均匀分布
    return dist(gen);
}

// 生成一个服从正态分布的随机数
// @param mean 正态分布的均值
// @param stddev 正态分布的标准差
// @return 返回一个服从正态分布的随机数
double randNormal(double mean, double stddev)
{
    static std::mt19937 gen{std::random_device{}()};
    std::normal_distribution<double> dist(mean, stddev); // 正态分布
    return dist(gen);
}

// 解析6位16进制颜色字符串，返回 RGB 值
bool parsePointColor(const std::string& hex_color, float& r, float& g, float& b)
{
    if (hex_color.length() == 6) 
    {
        std::stringstream ss;
        ss << std::hex << hex_color;

        int hex_value;
        ss >> hex_value;

        r = ((hex_value >> 16) & 0xFF) / 255.0f * 255;
        g = ((hex_value >> 8) & 0xFF) / 255.0f * 255;
        b = (hex_value & 0xFF) / 255.0f * 255;
        
        ROS_INFO("parsePointColor: %s -> %f, %f, %f", hex_color.c_str(), r, g, b);
        return true;
    } 
    else 
    {
        r = g = b = 255.0f;
        return false;
    }
}


/* ============================================================
 *   点属性赋值特征 (Point Traits) - 解决不同点云类型赋值问题
 * ============================================================ */

// 基础模板：默认什么都不做
template <typename PointT>
struct PointAssigner 
{
    static void apply(PointT& p, float intensity, float r, float g, float b) {}
};

// 特化：针对 PointXYZI
template <>
struct PointAssigner<pcl::PointXYZI>
{
    static void apply(pcl::PointXYZI& p, float intensity, float r, float g, float b)
    {
        p.intensity = intensity;
    }
};

// 特化：针对 PointXYZRGB
template <>
struct PointAssigner<pcl::PointXYZRGB> {
    static void apply(pcl::PointXYZRGB& p, float intensity, float r, float g, float b)
    {
        p.r = static_cast<uint8_t>(r);
        p.g = static_cast<uint8_t>(g);
        p.b = static_cast<uint8_t>(b);
    }
};

/* ============================================================
 *   形状生成基类 (Templated)
 * ============================================================ */

template <typename PointT>
class ShapeBase
{
public:
    using Ptr = typename pcl::PointCloud<PointT>::Ptr;

    virtual ~ShapeBase() = default;
    virtual void generate(Ptr cloud) = 0;

protected:
    double resolution_;
    double length_, width_, height_;
    double radius_, thickness_;
    bool use_color_;
    float r_, g_, b_;
    std::string intensity_dir_;
    TransformParams tf_params_;

    void loadCommonParams(ros::NodeHandle& nh)
    {
        nh.param("resolution", resolution_, 0.1);
        nh.param("length", length_, 10.0);
        nh.param("width", width_, 5.0);
        nh.param("height", height_, 3.0);
        nh.param("radius", radius_, 3.0);
        nh.param("thickness", thickness_, 0.1);
        
        // 位置
        nh.param("center_x", tf_params_.cx, 0.0);
        nh.param("center_y", tf_params_.cy, 0.0);
        nh.param("center_z", tf_params_.cz, 0.0);

        // 旋转
        nh.param("roll", tf_params_.roll, 0.0);
        nh.param("pitch", tf_params_.pitch, 0.0);
        nh.param("yaw", tf_params_.yaw, 0.0);

        nh.param<std::string>("intensity_dir", intensity_dir_, "");
        // use_color_ = parsePointColor(nh.param("point_color", ""), r_, g_, b_);
        use_color_ = false;

        tf_params_.updateMatrix();
    }

    void processAndAddPoint(Ptr cloud, double local_x, double local_y, double local_z)
    {
        PointT pt;
        
        // 1. 坐标变换 (使用 Eigen 进行旋转和平移)
        Eigen::Vector3f local_vec(local_x, local_y, local_z);
        Eigen::Vector3f global_vec = tf_params_.transform_matrix * local_vec;

        pt.x = global_vec.x();
        pt.y = global_vec.y();
        pt.z = global_vec.z();

        // 2. 生成一些模拟的颜色/强度数据 (基于局部坐标，产生渐变效果)
        float intensity = 1.0f;
        if (intensity_dir_ == "x") {
            intensity = 1.0f + (local_x / length_);
        } else if (intensity_dir_ == "y") {
            intensity = 1.0f + (local_y / width_);
        } else if (intensity_dir_ == "z") {
            intensity = 1.0f + (local_z / height_);
        }

        float r = 128 + 127 * sin(local_x);
        float g = 128 + 127 * cos(local_y);
        float b = 128 + 127 * sin(local_z);
        if(use_color_){ r = r_;g = g_;b = b_; }

        // 3. 根据类型赋值属性 (XYZ忽略, XYZI赋值强度, XYZRGB赋值颜色)
        PointAssigner<PointT>::apply(pt, intensity, r, g, b);

        cloud->push_back(pt);
    }
};

/* ============================================================
 *   具体形状实现 (Templated)
 * ============================================================ */

template <typename PointT>
class CubeShape : public ShapeBase<PointT>
{
public:
    CubeShape(ros::NodeHandle& nh) { this->loadCommonParams(nh); }

    void generate(typename ShapeBase<PointT>::Ptr cloud) override
    {
        for (double x = -this->length_/2; x <= this->length_/2; x += this->resolution_)
        for (double y = -this->width_/2;  y <= this->width_/2;  y += this->resolution_)
        for (double z = -this->height_/2; z <= this->height_/2; z += this->resolution_)
        {
            bool on_surface =
                fabs(fabs(x) - this->length_/2) < this->thickness_ ||
                fabs(fabs(y) - this->width_/2)  < this->thickness_ ||
                fabs(fabs(z) - this->height_/2) < this->thickness_;

            if (on_surface) this->processAndAddPoint(cloud, x, y, z);
        }
    }
};

template <typename PointT>
class SphereShape : public ShapeBase<PointT>
{
public:
    SphereShape(ros::NodeHandle& nh) { this->loadCommonParams(nh); }

    void generate(typename ShapeBase<PointT>::Ptr cloud) override
    {
        for (double x = -this->radius_; x <= this->radius_; x += this->resolution_)
        for (double y = -this->radius_; y <= this->radius_; y += this->resolution_)
        for (double z = -this->radius_; z <= this->radius_; z += this->resolution_)
        {
            double r = sqrt(x*x + y*y + z*z);
            if (fabs(r - this->radius_) < this->thickness_)
                this->processAndAddPoint(cloud, x, y, z);
        }
    }
};

template <typename PointT>
class BoxCorridorShape : public ShapeBase<PointT>
{
public:
    BoxCorridorShape(ros::NodeHandle& nh) { this->loadCommonParams(nh); }

    void generate(typename ShapeBase<PointT>::Ptr cloud) override
    {
        for (double x = 0;                x <= this->length_;   x += this->resolution_)
        for (double y = -this->width_/2;  y <= this->width_/2;  y += this->resolution_)
        for (double z = -this->height_/2; z <= this->height_/2; z += this->resolution_)
        {
            bool wall = fabs(fabs(y) - this->width_/2) < this->thickness_ ||
                        fabs(fabs(z) - this->height_/2) < this->thickness_;
            bool cap = x < this->thickness_ || fabs(x - this->length_) < this->thickness_;

            if (wall || cap) this->processAndAddPoint(cloud, x, y, z);
        }
    }
};

template <typename PointT>
class CylinderCorridorShape : public ShapeBase<PointT>
{
public:
    CylinderCorridorShape(ros::NodeHandle& nh) { this->loadCommonParams(nh); }

    void generate(typename ShapeBase<PointT>::Ptr cloud) override
    {
        for (double x = 0;              x <= this->length_; x += this->resolution_)
        for (double y = -this->radius_; y <= this->radius_; y += this->resolution_)
        for (double z = -this->radius_; z <= this->radius_; z += this->resolution_)
        {
            double r = sqrt(y*y + z*z);

            bool wall = fabs(r - this->radius_) < this->thickness_;
            bool cap  = (x < this->thickness_ || fabs(x - this->length_) < this->thickness_) 
                        && (r <= this->radius_);

            if (wall || cap) this->processAndAddPoint(cloud, x, y, z);
        }
    }
};

template <typename PointT>
class SemiCylinderCorridorShape : public ShapeBase<PointT>
{
public:
    SemiCylinderCorridorShape(ros::NodeHandle& nh) { this->loadCommonParams(nh); }

    void generate(typename ShapeBase<PointT>::Ptr cloud) override
    {
        for (double x = 0;              x <= this->length_; x += this->resolution_)
        for (double y = -this->radius_; y <= this->radius_; y += this->resolution_)
        for (double z = -this->height_; z <= this->radius_; z += this->resolution_)
        {
            bool wall, cap;
            if(z >= 0)
            {
                double r = sqrt(y*y + z*z);
                wall = fabs(r - this->radius_) < this->thickness_;
                cap  = (x < this->thickness_ || fabs(x - this->length_) < this->thickness_) 
                            && (r <= this->radius_);
            }
            else
            {
                wall = fabs(fabs(y) - this->radius_) < this->thickness_ ||
                            fabs(fabs(z) - this->height_) < this->thickness_;
                cap  = (x < this->thickness_ || fabs(x - this->length_) < this->thickness_);
            }

            if (wall || cap) this->processAndAddPoint(cloud, x, y, z);
        }
    }
};

template <typename PointT>
class ShapeFactory
{
public:
    static std::unique_ptr<ShapeBase<PointT>> create(const std::string& type, ros::NodeHandle& nh)
    {
        if (type == "cube") return std::unique_ptr<ShapeBase<PointT>>(new CubeShape<PointT>(nh));
        if (type == "sphere") return std::unique_ptr<ShapeBase<PointT>>(new SphereShape<PointT>(nh));
        if (type == "box_corridor") return std::unique_ptr<ShapeBase<PointT>>(new BoxCorridorShape<PointT>(nh));
        if (type == "cylinder_corridor") return std::unique_ptr<ShapeBase<PointT>>(new CylinderCorridorShape<PointT>(nh));
        ROS_ERROR("Unknown shape type: %s", type.c_str());
        return nullptr;
    }
};

/* ============================================================
 *   后处理 (Templated)
 * ============================================================ */

template <typename PointT>
void applyPostProcessing(typename pcl::PointCloud<PointT>::Ptr cloud, ros::NodeHandle& nh)
{
    bool enable_noise, enable_sparse, enable_hole;
    nh.param("enable_noise", enable_noise, false);
    nh.param("enable_sparse", enable_sparse, false);
    nh.param("enable_hole", enable_hole, false);

    double noise_std, sparse_keep;
    nh.param("noise_std", noise_std, 0.02);
    nh.param("sparse_keep_ratio", sparse_keep, 0.7);

    double hx, hy, hz, hr;
    nh.param("hole_x", hx, 0.0);
    nh.param("hole_y", hy, 0.0);
    nh.param("hole_z", hz, 0.0);
    nh.param("hole_radius", hr, 1.0);

    // 噪声
    if (enable_noise)
    {
        for (auto& p : cloud->points)
        {
            p.x += randNormal(0, noise_std);
            p.y += randNormal(0, noise_std);
            p.z += randNormal(0, noise_std);
        }
    }

    // 稀疏化
    if (enable_sparse)
    {
        typename pcl::PointCloud<PointT> tmp;
        for (auto& p : cloud->points)
            if (randUniform(0, 1) < sparse_keep) tmp.push_back(p);
        cloud->swap(tmp);
    }

    // 挖洞
    if (enable_hole)
    {
        typename pcl::PointCloud<PointT> tmp;
        for (auto& p : cloud->points)
        {
            double d = std::sqrt(pow(p.x - hx, 2) + pow(p.y - hy, 2) + pow(p.z - hz, 2));
            if (d > hr) tmp.push_back(p);
        }
        cloud->swap(tmp);
    }
}

/* ============================================================
 *   执行引擎 (Driver)
 * ============================================================ */

template <typename PointT>
void runGenerator(ros::NodeHandle& nh)
{
    std::string shape_type, pcd_path;
    bool pub_cloud;
    nh.param("shape_type", shape_type, std::string("cube"));
    nh.param("pcd_output_path", pcd_path, std::string("map.pcd"));
    nh.param("publish_cloud", pub_cloud, true);

    auto shape = ShapeFactory<PointT>::create(shape_type, nh);
    if (!shape) return;

    typename pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);
    
    // 生成
    auto t0 = std::chrono::steady_clock::now();
    shape->generate(cloud);
    
    // 后处理
    applyPostProcessing<PointT>(cloud, nh);
    
    auto t1 = std::chrono::steady_clock::now();
    double time_ms = std::chrono::duration<double, std::milli>(t1 - t0).count();

    cloud->width = cloud->size();
    cloud->height = 1;
    cloud->is_dense = true;

    // 发布
    ros::Publisher pub = nh.advertise<sensor_msgs::PointCloud2>("map", 1, true);
    if (pub_cloud)
    {
        sensor_msgs::PointCloud2 msg;
        pcl::toROSMsg(*cloud, msg);
        msg.header.frame_id = "map";
        msg.header.stamp = ros::Time::now();
        pub.publish(msg);
        ROS_INFO("Map published (%s)", typeid(PointT).name());
        ros::spinOnce();
    }

    // 保存
    if (pcl::io::savePCDFileBinary(pcd_path, *cloud) == -1)
    {
        ROS_ERROR("Failed to save PCD file");
    }
    else
    {
        ROS_INFO("Saved %lu points to %s", cloud->size(), pcd_path.c_str());
    }

    // 统计输出
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
        << "\nPoint type      : " << nh.param("point_type", std::string("xyz"))
        << "\nPoint count     : " << cloud->size()
        << "\nResolution (m)  : " << nh.param("resolution", 0.1)
        << "\nGeneration time : " << time_ms << " ms"
        << "\nAABB size       : "
        << (maxx-minx) << " x "
        << (maxy-miny) << " x "
        << (maxz-minz)
        << "\nGrid range      : (" << minx << ", " << miny << ", " << minz 
        << ") to (" << maxx << ", " << maxy << ", " << maxz << ")"
        << "\nPCD file        : " << pcd_path
        << "\n============================================");
    
    ros::spin();
}

/* ============================================================
 *   主函数
 * ============================================================ */

int main(int argc, char** argv)
{
    ros::init(argc, argv, "pcd_map_generator");
    ros::NodeHandle nh("~");

    std::string point_type;
    nh.param("point_type", point_type, std::string("xyz"));

    ROS_INFO("Starting generator with point type: %s", point_type.c_str());

    if (point_type == "xyz") 
    {
        runGenerator<pcl::PointXYZ>(nh);
    }
    else if (point_type == "xyzi") 
    {
        runGenerator<pcl::PointXYZI>(nh);
    }
    else if (point_type == "xyzrgb") 
    {
        runGenerator<pcl::PointXYZRGB>(nh);
    }
    else
    {
        ROS_ERROR("Unsupported point_type. Use: xyz, xyzi, or xyzrgb");
        return 1;
    }

    return 0;
}