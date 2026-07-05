/**
 * @file generate_pcd_map.cpp
 * @brief 程序化点云地图生成工具 —— 支持多种几何形状、多种点云类型、坐标变换与后处理。
 *
 * @details
 * 基于模板的 PCD 地图生成器，核心功能：
 *   1. 几何形状生成    — 立方体、球体、方形走廊、圆柱走廊、半圆柱走廊五种形状
 *   2. 点类型支持      — pcl::PointXYZ / PointXYZI / PointXYZRGB，通过 PointAssigner 特化适配
 *   3. 坐标变换        — 支持 6-DOF 刚体变换（先旋转 ZYX，再平移），局部形状坐标 → 全局地图坐标
 *   4. 后处理          — 高斯噪声叠加、随机稀疏化、球形挖洞裁剪
 *   5. 发布与保存      — 发布到 ROS /map 话题，同时保存为二进制 PCD 文件
 *   6. 统计输出        — 自动计算并打印 AABB 包围盒范围与生成耗时
 *
 * 坐标系约定：
 *   - 局部形状坐标系：以形状几何中心为原点，各形状坐标定义在各自的生成函数中
 *   - 全局地图坐标系：通过 TransformParams 将局部坐标变换到 world/map 坐标系
 *   - 旋转顺序：ZYX (yaw → pitch → roll)，与 ROS REP-103 惯例一致
 *
 * 用法：
 *   rosrun uav_util generate_pcd_map _shape_type:=cube _point_type:=xyzrgb
 */

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

// ============================================================================
// 核心数学与转换工具 (Transform Utils)
// ============================================================================

/** @brief 6-DOF 刚体变换参数结构体。
 *  @details 旋转顺序 ZYX (yaw → pitch → roll)，变换矩阵 apply 顺序：先旋转后平移。
 *           外部设置 cx/cy/cz/roll/pitch/yaw 后需调用 updateMatrix() 更新内联变换矩阵。 */
struct TransformParams
{
    double cx;                           ///< 平移 X [m]
    double cy;                           ///< 平移 Y [m]
    double cz;                           ///< 平移 Z [m]
    double roll;                         ///< 绕 X 轴旋转 [rad]
    double pitch;                        ///< 绕 Y 轴旋转 [rad]
    double yaw;                          ///< 绕 Z 轴旋转 [rad]
    Eigen::Affine3f transform_matrix;    ///< 组合后的 4x4 变换矩阵 (translation * rotZ * rotY * rotX)

    /** @brief 根据当前 roll/pitch/yaw/cx/cy/cz 更新内联变换矩阵。 */
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

/**
 * @brief 生成 [a, b] 范围内的均匀分布随机数。
 * @param a  随机数范围下界
 * @param b  随机数范围上界
 * @return   [a, b] 内的均匀分布随机数
 */
double randUniform(double a, double b)
{
    static std::mt19937 gen{std::random_device{}()};     // 静态随机数生成器，避免重复初始化
    std::uniform_real_distribution<double> dist(a, b);   // 均匀分布
    return dist(gen);
}

/**
 * @brief 生成服从正态分布 N(mean, stddev^2) 的随机数。
 * @param mean    正态分布的均值
 * @param stddev  正态分布的标准差
 * @return        服从正态分布的随机数
 */
double randNormal(double mean, double stddev)
{
    static std::mt19937 gen{std::random_device{}()};
    std::normal_distribution<double> dist(mean, stddev); // 正态分布
    return dist(gen);
}

/**
 * @brief 解析 6 位十六进制颜色字符串 (#RRGGBB) 为 RGB 分量。
 * @param hex_color  6 位十六进制字符串（如 "FF8800"）
 * @param[out] r     红色分量 [0, 255]
 * @param[out] g     绿色分量 [0, 255]
 * @param[out] b     蓝色分量 [0, 255]
 * @return           解析成功返回 true，否则返回 false 并默认设置为白色
 */
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


// ============================================================================
// 点属性赋值特征 (Point Traits) - 解决不同点云类型赋值问题
// ============================================================================

/** @brief 点属性赋值器基类模板 —— 默认不赋值任何额外属性。 */
template <typename PointT>
struct PointAssigner
{
    /** @brief 默认空实现，XYZ 类型点无需额外属性。 */
    static void apply(PointT& p, float intensity, float r, float g, float b) {}
};

/** @brief 针对 pcl::PointXYZI 的特化：赋值 intensity 字段。 */
template <>
struct PointAssigner<pcl::PointXYZI>
{
    static void apply(pcl::PointXYZI& p, float intensity, float r, float g, float b)
    {
        p.intensity = intensity;
    }
};

/** @brief 针对 pcl::PointXYZRGB 的特化：赋值 r/g/b 颜色通道。 */
template <>
struct PointAssigner<pcl::PointXYZRGB> {
    static void apply(pcl::PointXYZRGB& p, float intensity, float r, float g, float b)
    {
        p.r = static_cast<uint8_t>(r);
        p.g = static_cast<uint8_t>(g);
        p.b = static_cast<uint8_t>(b);
    }
};

// ============================================================================
// 形状生成基类 (Templated)
// ============================================================================

/**
 * @brief 形状生成基类模板，封装公共参数加载、坐标变换与属性赋值逻辑。
 *
 * 算法流水线 (processAndAddPoint)：
 *   1. 坐标变换 — 使用 Eigen 将局部坐标 (local_x, local_y, local_z) 经 transform_matrix 变换到全局坐标
 *   2. 模拟数据 — 基于局部坐标产生渐变色强度/颜色
 *   3. 属性赋值 — 通过 PointAssigner 特化将强度/颜色写入对应点类型字段
 */
template <typename PointT>
class ShapeBase
{
public:
    using Ptr = typename pcl::PointCloud<PointT>::Ptr;

    virtual ~ShapeBase() = default;

    /** @brief 纯虚函数：生成形状点云。子类必须实现。 */
    virtual void generate(Ptr cloud) = 0;

protected:
    double resolution_;             ///< 点间距/分辨率 [m]
    double length_;                 ///< 长度 (X 轴) [m]
    double width_;                  ///< 宽度 (Y 轴) [m]
    double height_;                 ///< 高度 (Z 轴) [m]
    double radius_;                 ///< 半径 [m]
    double thickness_;              ///< 壳厚度 (表面点判定阈值) [m]
    bool use_color_;                ///< 是否使用纯色填充（否则渐变）
    float r_, g_, b_;               ///< 纯色 RGB 分量 [0, 255]
    std::string intensity_dir_;     ///< 强度渐变方向 ("x"/"y"/"z"，空表示不渐变)
    TransformParams tf_params_;     ///< 6-DOF 变换参数

    /** @brief 从 ROS param 加载通用参数并更新变换矩阵。 */
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

    /**
     * @brief 处理局部坐标并添加到点云：变换、赋色、赋值、入云。
     * @param local_x  局部 X 坐标 [m]
     * @param local_y  局部 Y 坐标 [m]
     * @param local_z  局部 Z 坐标 [m]
     */
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

// ============================================================================
// 具体形状实现 (Templated)
// ============================================================================

/**
 * @brief 立方体形状 —— 六面空心壳体。
 * @details 在三轴范围 [-L/2, L/2] 内均匀采样，仅保留距离表面在 thickness 范围内的点。 */
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

/**
 * @brief 球体形状 —— 空心球壳。
 * @details 在包围立方体 [-r, r] 内均匀采样，保留距球心距离与半径之差在 thickness 范围内的点。 */
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

/**
 * @brief 方形走廊形状 —— 截面为矩形的管道，一端封闭。
 * @details X 轴从 0 到 length 延伸，YZ 截面为矩形，两端有封盖。 */
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

/**
 * @brief 圆柱走廊形状 —— 截面为圆形的管道，一端封闭。
 * @details X 轴从 0 到 length 延伸，YZ 截面为圆形，两端有圆盘封盖。 */
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

/**
 * @brief 半圆柱走廊形状 —— 上半圆 + 下半矩形截面的复合管道。
 * @details Z >= 0 区域为圆形截面，Z < 0 区域为矩形截面，一端封闭。
 *         适用于模拟隧道等顶部圆弧、底部平坦的地形。 */
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

/**
 * @brief 形状工厂 —— 根据字符串名称创建对应的形状实例。
 * @details 支持的形状类型：cube / sphere / box_corridor / cylinder_corridor / semi_cylinder_corridor。 */
template <typename PointT>
class ShapeFactory
{
public:
    /**
     * @brief 工厂方法：根据类型名创建形状。
     * @param type  形状类型字符串 (cube/sphere/box_corridor/cylinder_corridor/semi_cylinder_corridor)
     * @param nh    ROS 私有节点句柄，用于加载形状参数
     * @return      形状对象 unique_ptr，未知类型返回 nullptr
     */
    static std::unique_ptr<ShapeBase<PointT>> create(const std::string& type, ros::NodeHandle& nh)
    {
        if (type == "cube") return std::unique_ptr<ShapeBase<PointT>>(new CubeShape<PointT>(nh));
        if (type == "sphere") return std::unique_ptr<ShapeBase<PointT>>(new SphereShape<PointT>(nh));
        if (type == "box_corridor") return std::unique_ptr<ShapeBase<PointT>>(new BoxCorridorShape<PointT>(nh));
        if (type == "cylinder_corridor") return std::unique_ptr<ShapeBase<PointT>>(new CylinderCorridorShape<PointT>(nh));
        if (type == "semi_cylinder_corridor") return std::unique_ptr<ShapeBase<PointT>>(new SemiCylinderCorridorShape<PointT>(nh));
        ROS_ERROR("Unknown shape type: %s", type.c_str());
        return nullptr;
    }
};

// ============================================================================
// 后处理 (Templated)
// ============================================================================

/**
 * @brief 对生成的点云应用后处理：噪声、稀疏化、挖洞。
 * @details
 *   1. 高斯噪声 — 对每个点的 XYZ 坐标叠加 N(0, noise_std) 噪声
 *   2. 稀疏化   — 以 sparse_keep_ratio 概率随机保留点，模拟稀疏采样
 *   3. 挖洞     — 移除以 (hx, hy, hz) 为球心、hr 为半径内的所有点，模拟遮挡
 *
 * @param cloud  输入/输出点云（原地修改）
 * @param nh     ROS 私有节点句柄，读取后处理参数
 */
template <typename PointT>
void applyPostProcessing(typename pcl::PointCloud<PointT>::Ptr cloud, ros::NodeHandle& nh)
{
    bool enable_noise, enable_sparse, enable_hole;
    nh.param("enable_noise", enable_noise, false);
    nh.param("enable_sparse", enable_sparse, false);
    nh.param("enable_hole", enable_hole, false);

    double noise_std, sparse_keep;
    nh.param("noise_std", noise_std, 0.02);              ///< 噪声标准差 [m]
    nh.param("sparse_keep_ratio", sparse_keep, 0.7);     ///< 稀疏保留比例 [0, 1]

    double hx, hy, hz, hr;
    nh.param("hole_x", hx, 0.0);                         ///< 洞中心 X [m]
    nh.param("hole_y", hy, 0.0);                         ///< 洞中心 Y [m]
    nh.param("hole_z", hz, 0.0);                         ///< 洞中心 Z [m]
    nh.param("hole_radius", hr, 1.0);                    ///< 洞半径 [m]

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

// ============================================================================
// 执行引擎 (Driver)
// ============================================================================

/**
 * @brief 执行生成器主流程：创建形状 → 生成 → 后处理 → 发布 → 保存 → 统计。
 * @tparam PointT  PCL 点类型 (PointXYZ / PointXYZI / PointXYZRGB)
 * @param nh  ROS 私有节点句柄
 */
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

// ============================================================================
// 主函数
// ============================================================================

/**
 * @brief 程序入口：根据 point_type 参数分派到对应模板实例化。
 *
 * 用法：
 *   rosrun uav_util generate_pcd_map _point_type:=xyzrgb _shape_type:=cube
 */
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
