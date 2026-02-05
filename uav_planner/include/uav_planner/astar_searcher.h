#ifndef ASTAR_SEARCHER_H
#define ASTAR_SEARCHER_H

#include <ros/ros.h>
#include <Eigen/Eigen>
#include <vector>
#include <map>
#include <nav_msgs/Path.h>

using namespace Eigen;
using namespace std;

// ============ 节点结构体 ============
struct GridNode;
typedef GridNode* GridNodePtr;

struct GridNode
{
    int id;                     // 0: 未访问, 1: open, -1: closed
    Eigen::Vector3i index;      // 栅格索引 (x, y, z)
    Eigen::Vector3d coord;      // 世界坐标 (m)
    double gScore, fScore;      // g: 已花费代价, f: 总代价 = g + h
    Eigen::Vector3i dir;        // 走到这个节点的方向
    GridNodePtr cameFrom;       // 上一个节点
    std::multimap<double, GridNodePtr>::iterator nodeMapIt;

    GridNode(Eigen::Vector3i _index, Eigen::Vector3d _coord)
    {
        id = 0;
        index = _index;
        coord = _coord;
        gScore = fScore = std::numeric_limits<double>::infinity();
        cameFrom = NULL;
        dir = Eigen::Vector3i::Zero();
    }

    GridNode() {}
};

// ============ A* 搜索器类 ============
class AstarPathFinder
{
public:
    AstarPathFinder() = default;
    ~AstarPathFinder() = default;

    void initGridMap(double _resolution, Eigen::Vector3d global_xyz_l, Eigen::Vector3d global_xyz_u,
                    int max_x_id, int max_y_id, int max_z_id);                      // 初始化地图
    void setObs(const double coord_x, const double coord_y, const double coord_z);  // 设置障碍物（世界坐标）
    bool getObs(const int index_x, const int index_y, const int index_z);
    void resetGrid(GridNodePtr ptr);
    void resetUsedGrids();

    void AstarGraphSearch(Eigen::Vector3d start_pt, Eigen::Vector3d end_pt);    // 主要搜索函数
    std::vector<Eigen::Vector3d> getPath();                                     // 路径回溯
    std::vector<Eigen::Vector3d> getVisitedNodes();                             // 获得已访问节点

    Eigen::Vector3d gridIndex2coord(const Eigen::Vector3i &index);  // 世界坐标 → 栅格索引
    Eigen::Vector3i coord2gridIndex(const Eigen::Vector3d &pt);     // 栅格索引 → 世界坐标（中心点）
    Eigen::Vector3d coordRounding(const Eigen::Vector3d &coord);

private:
    double getHeu(GridNodePtr node1, GridNodePtr node2);        // 代价函数  （选择一下其一）
    double getEuclHeu(GridNodePtr node1, GridNodePtr node2);    // 欧氏距离  （Euclidean）
    double getManhHeu(GridNodePtr node1, GridNodePtr node2);    // 曼哈顿距离（Manhattan）
    double getDiagHeu(GridNodePtr node1, GridNodePtr node2);    // 对角距离  （Diagonal）
    void AstarGetSucc(GridNodePtr currentPtr, vector<GridNodePtr> &neighborPtrSets, vector<double> &edgeCostSets);  // 拓展邻居

    inline bool isOccupied(const Eigen::Vector3i &index) const;
    inline bool isOccupied(const int &idx_x, const int &idx_y, const int &idx_z) const;
    inline bool isFree(const Eigen::Vector3i &index) const;
    inline bool isFree(const int &idx_x, const int &idx_y, const int &idx_z) const;

private:
    uint8_t *data;                              // 三维数组（存放障碍物标记）
    int GLX_SIZE, GLY_SIZE, GLZ_SIZE;           // 地图大小（栅格数）
    int GLYZ_SIZE, GLXYZ_SIZE;                  // 地图栅格数量
    double resolution, inv_resolution;          // 栅格分辨率及其倒数
    double gl_xl, gl_yl, gl_zl;                 // 地图下界
    double gl_xu, gl_yu, gl_zu;                 // 地图上界
    std::multimap<double, GridNodePtr> openSet; // open list，用于从f最小取节点
    GridNodePtr ***GridNodeMap;                 // 三维节点指针矩阵
    GridNodePtr terminatePtr;                   // 终点指针
    Eigen::Vector3i goalIdx;                    // 目标格索引
    const double inf = std::numeric_limits<double>::infinity();
};

/*
     ┌────────────────────┐
     │   initGridMap()    │
     └──────────┬─────────┘
                │
                ▼
    ┌────────────────────────┐
    │   setObs() 设置障碍物   │
    └───────────┬────────────┘
                │
                ▼
 ┌──────────────────────────────┐
 │ AstarGraphSearch(start,goal) │
 └──────────────┬───────────────┘
                │
                ▼
    ┌──────────────────────┐
    │ openSet ← start      │
    ├──────────────────────┤
    │  while openSet非空:  │
    │    取出f最小节点      │
    │    if 是终点 → break │
    │    AstarGetSucc()    │
    │    更新g,f, cameFrom │
    └──────────────────────┘
                │
                ▼
    ┌─────────────────────┐
    │ getPath() 回溯路径   │
    └─────────────────────┘
*/

#endif
