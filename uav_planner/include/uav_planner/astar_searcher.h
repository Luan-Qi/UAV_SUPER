/**
 * @file    astar_searcher.h
 * @brief   A* 3D 栅格搜索器 — 数据结构与类声明
 *
 * 核心数据结构:
 *   GridNode     — 栅格节点 (索引、坐标、代价、父指针、open set 迭代器)
 *   AstarPathFinder — A* 搜索器类 (地图管理、障碍物、搜索、路径回溯)
 *
 * 算法流程 (详见 astar_searcher.cpp):
 *   initGridMap()           → 分配栅格地图内存
 *   setObs()                → 标记障碍物
 *   AstarGraphSearch(s, g)  → A* 主搜索循环
 *   getPath()               → 回溯路径
 *   resetUsedGrids()        → 重置搜索状态
 */

#ifndef ASTAR_SEARCHER_H
#define ASTAR_SEARCHER_H

#include <ros/ros.h>
#include <Eigen/Eigen>
#include <vector>
#include <map>
#include <nav_msgs/Path.h>

using namespace Eigen;
using namespace std;

// ============================================================================
// GridNode — 栅格节点结构体
// ============================================================================

struct GridNode;
typedef GridNode* GridNodePtr;

struct GridNode
{
    /**
     * id 状态机:
     *   0  — 未访问 (unvisited)
     *   1  — 在 open set 中
     *   -1 — 在 closed set 中
     */
    int id;

    Eigen::Vector3i index;   ///< 栅格索引 (x, y, z)
    Eigen::Vector3d coord;   ///< 世界坐标 (栅格中心点, m)

    double gScore;           ///< 从起点到当前节点的实际代价
    double fScore;           ///< 总代价 f = g + h (h 为启发式估计)

    Eigen::Vector3i dir;     ///< 从父节点走到当前节点的方向向量 (调试用)
    GridNodePtr cameFrom;    ///< 父节点指针 (用于路径回溯)

    /// open set 中的迭代器 — 用于 O(log n) 删除/更新节点
    std::multimap<double, GridNodePtr>::iterator nodeMapIt;

    GridNode(Eigen::Vector3i _index, Eigen::Vector3d _coord)
    {
        id       = 0;
        index    = _index;
        coord    = _coord;
        gScore   = fScore = std::numeric_limits<double>::infinity();
        cameFrom = NULL;
        dir      = Eigen::Vector3i::Zero();
    }

    GridNode() {}
};

// ============================================================================
// AstarPathFinder — A* 搜索器类
// ============================================================================

class AstarPathFinder
{
public:
    AstarPathFinder()  = default;
    ~AstarPathFinder() = default;

    // -------- 地图管理 --------
    void initGridMap(double _resolution, Eigen::Vector3d global_xyz_l,
                     Eigen::Vector3d global_xyz_u,
                     int max_x_id, int max_y_id, int max_z_id);

    // -------- 障碍物管理 --------
    void setObs(const double coord_x, const double coord_y, const double coord_z);
    bool getObs(const int index_x, const int index_y, const int index_z);

    // -------- 搜索与路径 --------
    void AstarGraphSearch(Eigen::Vector3d start_pt, Eigen::Vector3d end_pt);
    std::vector<Eigen::Vector3d> getPath();
    std::vector<Eigen::Vector3d> getVisitedNodes();

    // -------- 状态管理 --------
    void resetGrid(GridNodePtr ptr);
    void resetUsedGrids();  ///< 重置所有节点搜索状态 (障碍物标记保留)

    // -------- 搜索保护参数 --------
    void setMaxSearchIterations(int max_iter);   ///< 设置最大搜索迭代次数
    void setMaxSearchTime(double max_time);       ///< 设置最大搜索时间 (秒)

    // -------- 坐标转换 --------
    Eigen::Vector3d gridIndex2coord(const Eigen::Vector3i &index);
    Eigen::Vector3i coord2gridIndex(const Eigen::Vector3d &pt);
    Eigen::Vector3d coordRounding(const Eigen::Vector3d &coord);

private:
    // -------- 启发式函数 (可选其一) --------
    double getHeu(GridNodePtr node1, GridNodePtr node2);      ///< 对角距离 + tie-breaker (默认)
    double getEuclHeu(GridNodePtr node1, GridNodePtr node2);  ///< 欧氏距离
    double getManhHeu(GridNodePtr node1, GridNodePtr node2);  ///< 曼哈顿距离
    double getDiagHeu(GridNodePtr node1, GridNodePtr node2);  ///< 对角距离 (纯)

    // -------- 邻域扩展 --------
    void AstarGetSucc(GridNodePtr currentPtr,
                      vector<GridNodePtr> &neighborPtrSets,
                      vector<double> &edgeCostSets);

    // -------- 占据状态查询 (inline, 高频调用) --------
    inline bool isOccupied(const Eigen::Vector3i &index) const;
    inline bool isOccupied(const int &idx_x, const int &idx_y, const int &idx_z) const;
    inline bool isFree(const Eigen::Vector3i &index) const;
    inline bool isFree(const int &idx_x, const int &idx_y, const int &idx_z) const;

private:
    // -------- 地图数据 --------
    uint8_t *data;         ///< 一维障碍物标记数组 (0=空闲, 1=障碍)
    int GLX_SIZE, GLY_SIZE, GLZ_SIZE;   ///< 各方向栅格数量
    int GLYZ_SIZE, GLXYZ_SIZE;          ///< 预计算: GLY*GLZ, GLX*GLY*GLZ
    double resolution, inv_resolution;  ///< 分辨率及其倒数
    double gl_xl, gl_yl, gl_zl;         ///< 地图下界 (世界坐标)
    double gl_xu, gl_yu, gl_zu;         ///< 地图上界 (世界坐标)

    // -------- 搜索数据结构 --------
    std::multimap<double, GridNodePtr> openSet;  ///< open set (按 fScore 排序)
    GridNodePtr ***GridNodeMap;                   ///< 三维栅格节点指针数组
    GridNodePtr terminatePtr;                     ///< 搜索终点指针 (供 getPath 回溯)
    Eigen::Vector3i goalIdx;                      ///< 终点栅格索引
    const double inf = std::numeric_limits<double>::infinity();

    // -------- 搜索保护参数 --------
    int max_search_iterations_ = 100000;          ///< 最大搜索迭代次数 (可配置)
    double max_search_time_ = 0.5;                ///< 最大搜索时间 秒 (可配置)
};

/*
 * 算法流程图:
 *
 *      ┌────────────────────┐
 *      │   initGridMap()    │  分配内存, 创建 GridNode 对象
 *      └──────────┬─────────┘
 *                 │
 *                 ▼
 *      ┌────────────────────────┐
 *      │   setObs() 设置障碍物   │  世界坐标 → 栅格索引 → data[]=1
 *      └───────────┬────────────┘
 *                  │
 *                  ▼
 *   ┌──────────────────────────────┐
 *   │ AstarGraphSearch(start,goal) │
 *   └──────────────┬───────────────┘
 *                  │
 *                  ▼
 *      ┌──────────────────────┐
 *      │ openSet ← start      │
 *      ├──────────────────────┤
 *      │  while openSet 非空: │
 *      │    取出 f 最小节点    │
 *      │    if 是终点 → break │
 *      │    AstarGetSucc()    │  26 邻域展开
 *      │    更新 g, f, cameFrom│
 *      └──────────────────────┘
 *                  │
 *                  ▼
 *      ┌─────────────────────┐
 *      │ getPath() 回溯路径   │  terminatePtr → ... → startPtr
 *      └─────────────────────┘
 */

#endif  // ASTAR_SEARCHER_H
