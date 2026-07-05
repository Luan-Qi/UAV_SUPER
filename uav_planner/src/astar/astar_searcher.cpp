/**
 * @file    astar_searcher.cpp
 * @brief   A* 3D 栅格搜索算法核心实现
 *
 * 算法概述:
 *   - 搜索空间: 26 邻域 3D 均匀栅格 (dx,dy,dz ∈ {-1,0,1}, 排除 (0,0,0))
 *   - 启发式:   对角距离 (Diagonal distance) + tie-breaker (1 + 1/1000)
 *   - 数据结构: std::multimap<double, GridNodePtr> 作为 open set (按 fScore 排序)
 *   - 路径回溯: 从终点沿 cameFrom 指针回溯到起点, 然后反转
 *
 * 关键函数调用链:
 *   initGridMap()           → 分配内存, 创建 GridNode 对象
 *   setObs()                → 标记障碍物 (世界坐标 → 栅格索引)
 *   AstarGraphSearch(s, g)  → 主搜索循环
 *   getPath()               → 回溯并返回路径点序列
 *   resetUsedGrids()        → 重置已访问节点, 准备下次搜索
 */

#include "astar_searcher.h"

using namespace std;
using namespace Eigen;

// ============================================================================
// 地图初始化
// ============================================================================

/**
 * @brief 初始化 3D 栅格地图
 * @param _resolution  栅格分辨率 (m)
 * @param global_xyz_l 地图下界 (世界坐标, 最小值角)
 * @param global_xyz_u 地图上界 (世界坐标, 最大值角)
 * @param max_x_id     X 方向栅格数
 * @param max_y_id     Y 方向栅格数
 * @param max_z_id     Z 方向栅格数
 *
 * 内存分配:
 *   - data[]:      一维 uint8_t 数组, 存储障碍物标记 (0=空闲, 1=障碍)
 *   - GridNodeMap: 三维指针数组, 每个栅格预分配一个 GridNode 对象
 *
 * 索引映射: data[idx_x * GLYZ_SIZE + idx_y * GLZ_SIZE + idx_z]
 */
void AstarPathFinder::initGridMap(double _resolution, Vector3d global_xyz_l,
                                   Vector3d global_xyz_u, int max_x_id,
                                   int max_y_id, int max_z_id)
{
    // 世界坐标边界
    gl_xl = global_xyz_l(0);
    gl_yl = global_xyz_l(1);
    gl_zl = global_xyz_l(2);

    gl_xu = global_xyz_u(0);
    gl_yu = global_xyz_u(1);
    gl_zu = global_xyz_u(2);

    // 栅格数量
    GLX_SIZE = max_x_id;
    GLY_SIZE = max_y_id;
    GLZ_SIZE = max_z_id;
    GLYZ_SIZE  = GLY_SIZE * GLZ_SIZE;
    GLXYZ_SIZE = GLX_SIZE * GLYZ_SIZE;

    // 分辨率
    resolution     = _resolution;
    inv_resolution = 1.0 / _resolution;  // 预计算倒数, 加速坐标转换

    // 障碍物标记数组 (0 = 空闲, 1 = 障碍)
    data = new uint8_t[GLXYZ_SIZE];
    memset(data, 0, GLXYZ_SIZE * sizeof(uint8_t));

    // 创建栅格节点对象 — 每个格点一个 GridNode (含坐标、索引)
    GridNodeMap = new GridNodePtr **[GLX_SIZE];
    for (int i = 0; i < GLX_SIZE; ++i)
    {
        GridNodeMap[i] = new GridNodePtr *[GLY_SIZE];
        for (int j = 0; j < GLY_SIZE; ++j)
        {
            GridNodeMap[i][j] = new GridNodePtr[GLZ_SIZE];
            for (int k = 0; k < GLZ_SIZE; ++k)
            {
                Vector3i tmpIdx(i, j, k);
                Vector3d pos = gridIndex2coord(tmpIdx);  // 栅格中心世界坐标
                GridNodeMap[i][j][k] = new GridNode(tmpIdx, pos);
            }
        }
    }
}

// ============================================================================
// 节点状态管理
// ============================================================================

/**
 * @brief 重置单个 GridNode 到初始状态 (id=0, 清空搜索信息)
 */
void AstarPathFinder::resetGrid(GridNodePtr ptr)
{
    ptr->id       = 0;      // 0 = 未访问
    ptr->cameFrom = NULL;   // 清除父节点指针
    ptr->gScore   = inf;    // 重置代价
    ptr->fScore   = inf;
}

/**
 * @brief 重置所有已访问过的栅格节点 — 每次规划完成后调用
 *
 * 遍历整个 GridNodeMap, 将每个节点的搜索状态重置。
 * 注意: 障碍物标记 (data[]) 不受影响。
 */
void AstarPathFinder::resetUsedGrids()
{
    for (int i = 0; i < GLX_SIZE; ++i)
        for (int j = 0; j < GLY_SIZE; ++j)
            for (int k = 0; k < GLZ_SIZE; ++k)
                resetGrid(GridNodeMap[i][j][k]);
    terminatePtr = nullptr;  // 清除上次搜索的终点指针
}

// ============================================================================
// 障碍物管理
// ============================================================================

/**
 * @brief 标记障碍物 (世界坐标 → 栅格索引)
 * @param coord_x, coord_y, coord_z 世界坐标 (m)
 *
 * 超出地图范围的坐标会被静默忽略
 */
void AstarPathFinder::setObs(const double coord_x, const double coord_y,
                              const double coord_z)
{
    // 边界检查
    if (coord_x < gl_xl || coord_y < gl_yl || coord_z < gl_zl ||
        coord_x >= gl_xu || coord_y >= gl_yu || coord_z >= gl_zu)
        return;

    // 世界坐标 → 栅格索引
    int idx_x = static_cast<int>((coord_x - gl_xl) * inv_resolution);
    int idx_y = static_cast<int>((coord_y - gl_yl) * inv_resolution);
    int idx_z = static_cast<int>((coord_z - gl_zl) * inv_resolution);

    // 设置障碍标记
    data[idx_x * GLYZ_SIZE + idx_y * GLZ_SIZE + idx_z] = 1;
}

/**
 * @brief 查询指定栅格索引是否为障碍物 (含边界检查)
 * @return true=障碍物, false=空闲或越界
 */
bool AstarPathFinder::getObs(const int index_x, const int index_y,
                              const int index_z)
{
    return (index_x >= 0 && index_x < GLX_SIZE &&
            index_y >= 0 && index_y < GLY_SIZE &&
            index_z >= 0 && index_z < GLZ_SIZE &&
            (data[index_x * GLYZ_SIZE + index_y * GLZ_SIZE + index_z] == 1));
}

// ============================================================================
// 坐标变换
// ============================================================================

/**
 * @brief 栅格索引 → 世界坐标 (栅格中心点)
 * @param index 栅格索引 (i, j, k)
 * @return 栅格中心的世界坐标
 *
 * 公式: coord = (index + 0.5) * resolution + origin
 */
Vector3d AstarPathFinder::gridIndex2coord(const Vector3i &index)
{
    Vector3d pt;
    pt(0) = ((double)index(0) + 0.5) * resolution + gl_xl;
    pt(1) = ((double)index(1) + 0.5) * resolution + gl_yl;
    pt(2) = ((double)index(2) + 0.5) * resolution + gl_zl;
    return pt;
}

/**
 * @brief 世界坐标 → 栅格索引 (向下取整, 自动钳位到有效范围)
 * @param pt 世界坐标 (m)
 * @return 对应的栅格索引
 *
 * 公式: idx = floor((pt - origin) / resolution), clamped to [0, SIZE-1]
 */
Vector3i AstarPathFinder::coord2gridIndex(const Vector3d &pt)
{
    Vector3i idx;
    idx << min(max(int((pt(0) - gl_xl) * inv_resolution), 0), GLX_SIZE - 1),
           min(max(int((pt(1) - gl_yl) * inv_resolution), 0), GLY_SIZE - 1),
           min(max(int((pt(2) - gl_zl) * inv_resolution), 0), GLZ_SIZE - 1);
    return idx;
}

/**
 * @brief 世界坐标 → 最近栅格中心坐标 (先取索引, 再转回坐标, 实现舍入)
 */
Eigen::Vector3d AstarPathFinder::coordRounding(const Eigen::Vector3d &coord)
{
    return gridIndex2coord(coord2gridIndex(coord));
}

// ============================================================================
// 占据状态查询 (inline, 供搜索循环高频调用)
// ============================================================================

inline bool AstarPathFinder::isOccupied(const Eigen::Vector3i &index) const
{
    return isOccupied(index(0), index(1), index(2));
}

inline bool AstarPathFinder::isFree(const Eigen::Vector3i &index) const
{
    return isFree(index(0), index(1), index(2));
}

inline bool AstarPathFinder::isOccupied(const int &idx_x, const int &idx_y,
                                         const int &idx_z) const
{
    return (idx_x >= 0 && idx_x < GLX_SIZE &&
            idx_y >= 0 && idx_y < GLY_SIZE &&
            idx_z >= 0 && idx_z < GLZ_SIZE &&
            (data[idx_x * GLYZ_SIZE + idx_y * GLZ_SIZE + idx_z] == 1));
}

inline bool AstarPathFinder::isFree(const int &idx_x, const int &idx_y,
                                     const int &idx_z) const
{
    return (idx_x >= 0 && idx_x < GLX_SIZE &&
            idx_y >= 0 && idx_y < GLY_SIZE &&
            idx_z >= 0 && idx_z < GLZ_SIZE &&
            (data[idx_x * GLYZ_SIZE + idx_y * GLZ_SIZE + idx_z] < 1));
}

// ============================================================================
// 邻域扩展 — 26 连通 3D 搜索
// ============================================================================

/**
 * @brief 获取当前节点的所有有效邻居节点及其边代价
 *
 * 搜索范围: 26 邻域 (dx,dy,dz ∈ {-1,0,1}, 排除原点)
 * 边代价:   轴对齐移动 = 1, 面对角移动 = √2, 体对角移动 = √3
 *
 * @param[in]  currentPtr      当前节点
 * @param[out] neighborPtrSets 邻居节点指针列表
 * @param[out] edgeCostSets    对应边的移动代价
 */
inline void AstarPathFinder::AstarGetSucc(GridNodePtr currentPtr,
                                           vector<GridNodePtr> &neighborPtrSets,
                                           vector<double> &edgeCostSets)
{
    neighborPtrSets.clear();
    edgeCostSets.clear();

    Vector3i neighborIdx;
    for (int dx = -1; dx < 2; ++dx)
    {
        for (int dy = -1; dy < 2; ++dy)
        {
            for (int dz = -1; dz < 2; ++dz)
            {
                // 跳过自身
                if (dx == 0 && dy == 0 && dz == 0)
                    continue;

                // 计算邻居索引
                neighborIdx(0) = (currentPtr->index)(0) + dx;
                neighborIdx(1) = (currentPtr->index)(1) + dy;
                neighborIdx(2) = (currentPtr->index)(2) + dz;

                // 边界检查 — 超出地图范围的邻居直接跳过
                if (neighborIdx(0) < 0 || neighborIdx(0) >= GLX_SIZE ||
                    neighborIdx(1) < 0 || neighborIdx(1) >= GLY_SIZE ||
                    neighborIdx(2) < 0 || neighborIdx(2) >= GLZ_SIZE)
                    continue;

                // 加入邻居列表
                neighborPtrSets.push_back(
                    GridNodeMap[neighborIdx(0)][neighborIdx(1)][neighborIdx(2)]);
                // 边代价 = 欧氏距离 (轴对齐=1, 面对角=√2, 体对角=√3)
                edgeCostSets.push_back(sqrt(dx * dx + dy * dy + dz * dz));
            }
        }
    }
}

// ============================================================================
// 启发式函数
// ============================================================================

/**
 * @brief 启发式函数 — 对角距离 + tie-breaker
 *
 * tie-breaker (1 + 1/1000) 的作用:
 *   在 f 值相等时略微偏向 g 值更大 (即更靠近终点) 的节点,
 *   减少需要探索的节点数, 同时保证 ε- admissible (最优性损失 < 0.1%)
 *
 * @return h(n) = tie_breaker * diagonal_distance(n, goal)
 */
double AstarPathFinder::getHeu(GridNodePtr node1, GridNodePtr node2)
{
    double tie_breaker = 1.0 + 1.0 / 1000.0;
    return tie_breaker * getDiagHeu(node1, node2);
}

/**
 * @brief 欧氏距离启发式 — 适用于各向同性运动
 * @note  可能低估实际代价, 导致搜索节点偏多, 但保证最优
 */
double AstarPathFinder::getEuclHeu(GridNodePtr node1, GridNodePtr node2)
{
    double dx = node1->index(0) - node2->index(0);
    double dy = node1->index(1) - node2->index(1);
    double dz = node1->index(2) - node2->index(2);
    return sqrt(dx * dx + dy * dy + dz * dz);
}

/**
 * @brief 曼哈顿距离启发式 — 适用于轴对齐运动
 * @note  在 26 连通 3D 栅格中可能高估, 不保证最优
 */
double AstarPathFinder::getManhHeu(GridNodePtr node1, GridNodePtr node2)
{
    double dx = abs(node1->index(0) - node2->index(0));
    double dy = abs(node1->index(1) - node2->index(1));
    double dz = abs(node1->index(2) - node2->index(2));
    return dx + dy + dz;
}

/**
 * @brief 对角距离启发式 — 适用于 26 连通 3D 栅格 (当前默认)
 *
 * 原理: 先沿体对角线移动 min(dx,dy,dz) 步,
 *       再沿面对角线移动, 最后沿轴移动
 *
 * @note  这是 26 连通 3D 栅格中最紧的可接受启发式 (consistent heuristic)
 */
double AstarPathFinder::getDiagHeu(GridNodePtr node1, GridNodePtr node2)
{
    double dx = abs(node1->index(0) - node2->index(0));
    double dy = abs(node1->index(1) - node2->index(1));
    double dz = abs(node1->index(2) - node2->index(2));

    double h = 0.0;
    int diag = min(min(dx, dy), dz);  // 体对角线步数
    dx -= diag;
    dy -= diag;
    dz -= diag;

    // 根据哪一维先走完分情况处理
    if (dx == 0)
        h = 1.0 * sqrt(3.0) * diag           // 体对角线: √3
          + sqrt(2.0) * min(dy, dz)           // 面对角线: √2
          + 1.0 * abs(dy - dz);               // 轴对齐:   1
    if (dy == 0)
        h = 1.0 * sqrt(3.0) * diag
          + sqrt(2.0) * min(dx, dz)
          + 1.0 * abs(dx - dz);
    if (dz == 0)
        h = 1.0 * sqrt(3.0) * diag
          + sqrt(2.0) * min(dx, dy)
          + 1.0 * abs(dx - dy);

    return h;
}

// ============================================================================
// 主搜索循环 — A* Graph Search
// ============================================================================

/**
 * @brief A* 3D 栅格搜索主函数
 *
 * 算法流程:
 *   1. 将起点和终点坐标钳位到最近栅格中心
 *   2. 起点加入 open set (fScore = gScore + heuristic)
 *   3. 循环: 从 open set 取出 f 最小节点
 *      a) 若到达终点 → 记录 terminatePtr, 返回
 *      b) 将当前节点移入 closed set (id = -1)
 *      c) 展开 26 邻域:
 *         - 跳过障碍物和 closed set 中的节点
 *         - 未访问节点 (id=0):   计算 g/f, 加入 open set
 *         - open set 中节点 (id=1): 若新 g 更优则更新
 *   4. open set 为空 → 搜索失败
 *
 * @param start_pt 起点世界坐标 (自动舍入到最近栅格中心)
 * @param end_pt   终点世界坐标 (自动舍入到最近栅格中心)
 *
 * @note 搜索结果通过 terminatePtr 保存, 后续由 getPath() 回溯
 */
void AstarPathFinder::AstarGraphSearch(Vector3d start_pt, Vector3d end_pt)
{
    ros::Time time_1 = ros::Time::now();  // 计时开始

    // -------- 坐标预处理: 舍入到栅格中心 --------
    Vector3i start_idx = coord2gridIndex(start_pt);
    Vector3i end_idx   = coord2gridIndex(end_pt);
    goalIdx = end_idx;  // 保存终点索引 (用于终止判断)

    start_pt = gridIndex2coord(start_idx);
    end_pt   = gridIndex2coord(end_idx);

    // -------- 初始化起点/终点节点 --------
    GridNodePtr startPtr = new GridNode(start_idx, start_pt);
    GridNodePtr endPtr   = new GridNode(end_idx, end_pt);

    // -------- 初始化搜索数据结构 --------
    openSet.clear();  // multimap<double, GridNodePtr>: key=fScore
    GridNodePtr currentPtr  = NULL;
    GridNodePtr neighborPtr = NULL;

    // 起点入队
    startPtr->gScore = 0;
    startPtr->fScore = getHeu(startPtr, endPtr);  // f = g + h = 0 + h
    startPtr->id     = 1;  // 1 = 在 open set 中
    startPtr->coord  = start_pt;
    openSet.insert(make_pair(startPtr->fScore, startPtr));

    // 临时变量
    double tentative_gScore;
    vector<GridNodePtr> neighborPtrSets;
    vector<double> edgeCostSets;

    // ======== 主搜索循环 ========
    while (!openSet.empty())
    {
        // -------- Step 1: 取出 fScore 最小的节点 (open set 按 key 排序) --------
        currentPtr = openSet.begin()->second;

        // -------- Step 2: 到达终点检查 --------
        if (currentPtr->index == goalIdx)
        {
            ros::Time time_2 = ros::Time::now();
            terminatePtr = currentPtr;  // 记录终点, 供 getPath() 回溯
            ROS_INFO("[astar] [A*]{success} Time in A* is %.1f ms, path cost is %.1f m",
                     (time_2 - time_1).toSec() * 1000.0,
                     currentPtr->gScore * resolution);
            return;
        }

        // -------- Step 3: 移入 closed set --------
        openSet.erase(openSet.begin());
        currentPtr->id = -1;  // -1 = 在 closed set 中

        // -------- Step 4: 展开邻居 (26 邻域) --------
        AstarGetSucc(currentPtr, neighborPtrSets, edgeCostSets);

        // -------- Step 5: 遍历所有邻居 --------
        for (int i = 0; i < (int)neighborPtrSets.size(); ++i)
        {
            neighborPtr = neighborPtrSets[i];

            // 跳过障碍物和已关闭节点
            if (isOccupied(neighborPtr->index) || neighborPtr->id == -1)
                continue;

            double edge_cost = edgeCostSets[i];
            tentative_gScore = currentPtr->gScore + edge_cost;

            if (neighborPtr->id != 1)
            {
                // -------- 发现新节点 (不在 open set 中) --------
                neighborPtr->id       = 1;
                neighborPtr->cameFrom = currentPtr;
                neighborPtr->gScore   = tentative_gScore;
                neighborPtr->fScore   = tentative_gScore + getHeu(neighborPtr, endPtr);
                // 插入 open set 并保存迭代器 (用于后续高效删除/更新)
                neighborPtr->nodeMapIt =
                    openSet.insert(make_pair(neighborPtr->fScore, neighborPtr));
            }
            else if (neighborPtr->gScore >= tentative_gScore)
            {
                // -------- 已在 open set 中, 且找到更优路径 → 更新 --------
                neighborPtr->cameFrom = currentPtr;
                neighborPtr->gScore   = tentative_gScore;
                neighborPtr->fScore   = tentative_gScore + getHeu(neighborPtr, endPtr);
                // 从 open set 中删除旧记录, 插入新记录
                openSet.erase(neighborPtr->nodeMapIt);
                neighborPtr->nodeMapIt =
                    openSet.insert(make_pair(neighborPtr->fScore, neighborPtr));

                // 更新方向向量 (用于调试/可视化)
                for (int d = 0; d < 3; ++d)
                {
                    neighborPtr->dir(d) = neighborPtr->index(d) - currentPtr->index(d);
                    if (neighborPtr->dir(d) != 0)
                        neighborPtr->dir(d) /= abs(neighborPtr->dir(d));
                }
            }
        }
    }

    // -------- open set 耗尽 → 搜索失败 (无可行路径) --------
    ros::Time time_2 = ros::Time::now();
    if ((time_2 - time_1).toSec() > 0.1)
        ROS_INFO("[astar] [A*]{fail} Time cost in A* is %.1f s",
                 (time_2 - time_1).toSec());
}

// ============================================================================
// 路径回溯
// ============================================================================

/**
 * @brief 从终点沿 cameFrom 指针回溯到起点, 返回完整路径点序列
 *
 * @return vector<Vector3d> 从起点到终点的有序路径点 (世界坐标)
 *         若搜索未找到路径 (terminatePtr==nullptr), 返回空 vector
 *
 * 回溯方向: terminatePtr → ... → startPtr
 * 输出方向: startPtr → ... → terminatePtr (已反转)
 */
vector<Vector3d> AstarPathFinder::getPath()
{
    vector<Vector3d> path;
    vector<GridNodePtr> gridPath;

    if (terminatePtr == nullptr)
        return path;  // 搜索失败, 无路径

    // 从终点开始回溯
    gridPath.push_back(terminatePtr);
    while (terminatePtr->cameFrom != NULL)
    {
        terminatePtr = terminatePtr->cameFrom;
        gridPath.push_back(terminatePtr);
    }

    // 提取坐标
    for (auto ptr : gridPath)
        path.push_back(ptr->coord);

    // 反转: 使路径从起点到终点
    reverse(path.begin(), path.end());

    return path;
}

/**
 * @brief 获取所有已访问节点 (用于可视化调试)
 * @return 所有 id==-1 (closed set) 节点的世界坐标列表
 */
vector<Vector3d> AstarPathFinder::getVisitedNodes()
{
    vector<Vector3d> visited_nodes;
    for (int i = 0; i < GLX_SIZE; ++i)
        for (int j = 0; j < GLY_SIZE; ++j)
            for (int k = 0; k < GLZ_SIZE; ++k)
            {
                if (GridNodeMap[i][j][k]->id == -1)  // 仅可视化 closed set
                    visited_nodes.push_back(GridNodeMap[i][j][k]->coord);
            }

    ROS_INFO("[astar] visited_nodes size : %zu", visited_nodes.size());
    return visited_nodes;
}
