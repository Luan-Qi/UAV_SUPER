/**
 * @file generate_pcd_map_old.cpp
 * @brief 程序化点云地图生成工具（旧版）—— 仅支持 PointXYZI 类型，功能简化，已废弃，保留供参考。
 *
 * @details
 * 旧版 PCD 地图生成器，相比新版 generate_pcd_map.cpp 的差异：
 *   1. 点类型固定    — 仅支持 pcl::PointXYZI，无模板化
 *   2. 形状较少      — 仅支持 cube / sphere / box_corridor / cylinder_corridor 四种
 *   3. 无坐标变换    — 仅通过简单的 cx/cy/cz 平移，不支持旋转
 *   4. 无渐变着色    — 所有点 intensity 固定为 1.0
 *   5. 后处理独立    — 噪声/稀疏/挖洞为独立函数，而非统一模板
 *
 * 坐标系约定：
 *   - 各形状以几何中心为局部原点，通过 cx/cy/cz 平移到全局坐标
 *   - 无旋转支持
 *
 * 注意：本文件为遗留代码，新的开发请使用 generate_pcd_map.cpp。
 */

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

// ============================================================================
// 工具函数
// ============================================================================

/**
 * @brief 构造一个带 intensity 属性的 PointXYZI 点。
 * @param x          X 坐标 [m]
 * @param y          Y 坐标 [m]
 * @param z          Z 坐标 [m]
 * @param intensity  强度值（默认 1.0）
 * @return           构造好的 PointXYZI 点
 */
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

/**
 * @brief 生成 [a, b] 范围内的均匀分布随机数。
 * @param a  随机数范围下界
 * @param b  随机数范围上界
 * @return   [a, b] 内的均匀分布随机数
 */
double randUniform(double a, double b)
{
    static std::mt19937 gen{std::random_device{}()};
    std::uniform_real_distribution<double> dist(a, b);
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
    std::normal_distribution<double> dist(mean, stddev);
    return dist(gen);
}

// ============================================================================
// 生成不同形状
// ============================================================================

/**
 * @brief 形状生成基类（旧版，无模板化）。
 * @details 封装公共参数加载，子类通过覆写 generate() 实现具体形状。 */
class ShapeBase
{
public:
    virtual ~ShapeBase() = default;

    /** @brief 纯虚函数：生成形状点云。子类必须实现。
     *  @param cloud  输出点云（PointXYZI 类型） */
    virtual void generate(Cloud::Ptr cloud) = 0;

protected:
    double resolution_;     ///< 点间距/分辨率 [m]
    double length_;         ///< 长度 (X 轴) [m]
    double width_;          ///< 宽度 (Y 轴) [m]
    double height_;         ///< 高度 (Z 轴) [m]
    double radius_;         ///< 半径 [m]
    double thickness_;      ///< 壳厚度 (表面点判定阈值) [m]
    double cx, cy, cz;      ///< 平移分量 [m]（旧版不支持旋转）

    /** @brief 从 ROS param 加载通用参数。 */
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

/**
 * @brief 立方体形状 —— 六面空心壳体（旧版实现）。
 * @details 在三轴范围 [-L/2, L/2] 内均匀采样，仅保留距离表面在 thickness 范围内的点。
 *         通过 cx/cy/cz 平移，不支持旋转。 */
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

/**
 * @brief 球体形状 —— 空心球壳（旧版实现）。
 * @details 在包围立方体 [-r, r] 内均匀采样，保留距球心距离与半径之差在 thickness 范围内的点。 */
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

/**
 * @brief 方形走廊形状 —— 截面为矩形的管道（旧版实现）。
 * @details X 轴从 0 到 length 延伸，YZ 截面为矩形，两端有封盖。 */
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

/**
 * @brief 圆柱走廊形状 —— 截面为圆形的管道（旧版实现）。
 * @details X 轴从 0 到 length 延伸，YZ 截面为圆形，两端有圆盘封盖。 */
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

/**
 * @brief 形状工厂（旧版） —— 根据字符串名称创建对应的形状实例。
 * @details 支持的形状类型：cube / sphere / box_corridor / cylinder_corridor。 */
class ShapeFactory
{
public:
    /**
     * @brief 工厂方法：根据类型名创建形状。
     * @param type  形状类型字符串 (cube/sphere/box_corridor/cylinder_corridor)
     * @param nh    ROS 私有节点句柄
     * @return      形状对象 unique_ptr，未知类型返回 nullptr
     */
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

// ============================================================================
// 后处理
// ============================================================================

/**
 * @brief 对点云每个点的 XYZ 坐标叠加高斯噪声 N(0, stddev^2)。
 * @param cloud   输入/输出点云（原地修改）
 * @param stddev  噪声标准差 [m]
 */
void applyNoise(Cloud::Ptr cloud, double stddev)
{
    for (auto& p : cloud->points)
    {
        p.x += randNormal(0, stddev);
        p.y += randNormal(0, stddev);
        p.z += randNormal(0, stddev);
    }
}

/**
 * @brief 按比例随机保留点，实现稀疏化采样。
 * @param cloud       输入/输出点云（原地修改）
 * @param keep_ratio  保留比例 [0, 1]
 */
void applySparse(Cloud::Ptr cloud, double keep_ratio)
{
    Cloud tmp;
    for (auto& p : cloud->points)
        if (randUniform(0,1) < keep_ratio)
            tmp.push_back(p);
    cloud->swap(tmp);
}

/**
 * @brief 挖除以 (hx, hy, hz) 为球心、hr 为半径范围内的所有点。
 * @param cloud  输入/输出点云（原地修改）
 * @param hx     洞中心 X [m]
 * @param hy     洞中心 Y [m]
 * @param hz     洞中心 Z [m]
 * @param hr     洞半径 [m]
 */
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

// ============================================================================
// 主函数
// ============================================================================

/**
 * @brief 程序入口：加载参数 → 生成 → 后处理 → 发布 → 保存 → 统计输出。
 *
 * 用法：
 *   rosrun uav_util generate_pcd_map_old _shape_type:=cube _publish_cloud:=true
 */
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
    nh.param("noise_std", noise_std, 0.02);              ///< 噪声标准差 [m]
    nh.param("sparse_keep_ratio", sparse_keep, 0.7);     ///< 稀疏保留比例

    double hole_x, hole_y, hole_z, hole_r;
    nh.param("hole_x", hole_x, 0.0);                     ///< 洞中心 X [m]
    nh.param("hole_y", hole_y, 0.0);                     ///< 洞中心 Y [m]
    nh.param("hole_z", hole_z, 0.0);                     ///< 洞中心 Z [m]
    nh.param("hole_radius", hole_r, 1.0);                ///< 洞半径 [m]

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
