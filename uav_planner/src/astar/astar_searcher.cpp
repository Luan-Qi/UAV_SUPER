#include "astar_searcher.h"

using namespace std;
using namespace Eigen;

void AstarPathFinder::initGridMap(double _resolution, Vector3d global_xyz_l, Vector3d global_xyz_u, int max_x_id,
                                  int max_y_id, int max_z_id)
{
    // 设置边界范围
    gl_xl = global_xyz_l(0);
    gl_yl = global_xyz_l(1);
    gl_zl = global_xyz_l(2);

    gl_xu = global_xyz_u(0);
    gl_yu = global_xyz_u(1);
    gl_zu = global_xyz_u(2);

    // 设置地图大小（栅格数量）
    GLX_SIZE = max_x_id;
    GLY_SIZE = max_y_id;
    GLZ_SIZE = max_z_id;
    GLYZ_SIZE = GLY_SIZE * GLZ_SIZE;
    GLXYZ_SIZE = GLX_SIZE * GLYZ_SIZE;

    // 设置地图分辨率
    resolution = _resolution;
    inv_resolution = 1.0 / _resolution;

    // data 用来标记障碍物：0=空闲, 1=障碍
    data = new uint8_t[GLXYZ_SIZE];
    memset(data, 0, GLXYZ_SIZE * sizeof(uint8_t));

    // 创建栅格节点对象 GridNodeMap
    GridNodeMap = new GridNodePtr **[GLX_SIZE];
    for (int i = 0; i < GLX_SIZE; i++)
    {
        GridNodeMap[i] = new GridNodePtr *[GLY_SIZE];
        for (int j = 0; j < GLY_SIZE; j++)
        {
            GridNodeMap[i][j] = new GridNodePtr[GLZ_SIZE];
            for (int k = 0; k < GLZ_SIZE; k++)
            {
                Vector3i tmpIdx(i, j, k);
                Vector3d pos = gridIndex2coord(tmpIdx);
                GridNodeMap[i][j][k] = new GridNode(tmpIdx, pos);
            }
        }
    }
}

void AstarPathFinder::resetGrid(GridNodePtr ptr)
{
    ptr->id = 0;
    ptr->cameFrom = NULL;
    ptr->gScore = inf;
    ptr->fScore = inf;
}

void AstarPathFinder::resetUsedGrids()
{
    for (int i = 0; i < GLX_SIZE; i++)
        for (int j = 0; j < GLY_SIZE; j++)
            for (int k = 0; k < GLZ_SIZE; k++)
                resetGrid(GridNodeMap[i][j][k]);
    terminatePtr = nullptr;
}

void AstarPathFinder::setObs(const double coord_x, const double coord_y, const double coord_z)
{
    // 判断是否在地图范围内
    if (coord_x < gl_xl || coord_y < gl_yl || coord_z < gl_zl || coord_x >= gl_xu || coord_y >= gl_yu || coord_z >= gl_zu)
        return;

    // 将世界坐标转换为索引
    int idx_x = static_cast<int>((coord_x - gl_xl) * inv_resolution);
    int idx_y = static_cast<int>((coord_y - gl_yl) * inv_resolution);
    int idx_z = static_cast<int>((coord_z - gl_zl) * inv_resolution);

    data[idx_x * GLYZ_SIZE + idx_y * GLZ_SIZE + idx_z] = 1;
}

bool AstarPathFinder::getObs(const int index_x, const int index_y, const int index_z)
{
    return (index_x >= 0 && index_x < GLX_SIZE && index_y >= 0 && index_y < GLY_SIZE && index_z >= 0 && index_z < GLZ_SIZE &&
            (data[index_x * GLYZ_SIZE + index_y * GLZ_SIZE + index_z] == 1));
}

vector<Vector3d> AstarPathFinder::getVisitedNodes()
{
    vector<Vector3d> visited_nodes;
    for (int i = 0; i < GLX_SIZE; i++)
        for (int j = 0; j < GLY_SIZE; j++)
            for (int k = 0; k < GLZ_SIZE; k++)
            {
                // if(GridNodeMap[i][j][k]->id != 0) // visualize all nodes in open and
                // close list
                if (GridNodeMap[i][j][k]->id == -1)  // visualize nodes in close list only
                    visited_nodes.push_back(GridNodeMap[i][j][k]->coord);
            }

    ROS_INFO("[astar] visited_nodes size : %zu", visited_nodes.size());
    return visited_nodes;
}

Vector3d AstarPathFinder::gridIndex2coord(const Vector3i &index)
{
    Vector3d pt;

    pt(0) = ((double)index(0) + 0.5) * resolution + gl_xl;
    pt(1) = ((double)index(1) + 0.5) * resolution + gl_yl;
    pt(2) = ((double)index(2) + 0.5) * resolution + gl_zl;

    return pt;
}

Vector3i AstarPathFinder::coord2gridIndex(const Vector3d &pt)
{
    Vector3i idx;
    idx << min(max(int((pt(0) - gl_xl) * inv_resolution), 0), GLX_SIZE - 1),
           min(max(int((pt(1) - gl_yl) * inv_resolution), 0), GLY_SIZE - 1),
           min(max(int((pt(2) - gl_zl) * inv_resolution), 0), GLZ_SIZE - 1);

    return idx;
}

Eigen::Vector3d AstarPathFinder::coordRounding(const Eigen::Vector3d &coord)
{
    return gridIndex2coord(coord2gridIndex(coord));
}

inline bool AstarPathFinder::isOccupied(const Eigen::Vector3i &index) const
{
    return isOccupied(index(0), index(1), index(2));
}

inline bool AstarPathFinder::isFree(const Eigen::Vector3i &index) const
{
    return isFree(index(0), index(1), index(2));
}

inline bool AstarPathFinder::isOccupied(const int &idx_x, const int &idx_y, const int &idx_z) const
{
    return (idx_x >= 0 && idx_x < GLX_SIZE && idx_y >= 0 && idx_y < GLY_SIZE && idx_z >= 0 && idx_z < GLZ_SIZE &&
            (data[idx_x * GLYZ_SIZE + idx_y * GLZ_SIZE + idx_z] == 1));
}

inline bool AstarPathFinder::isFree(const int &idx_x, const int &idx_y, const int &idx_z) const
{
    return (idx_x >= 0 && idx_x < GLX_SIZE && idx_y >= 0 && idx_y < GLY_SIZE && idx_z >= 0 && idx_z < GLZ_SIZE &&
            (data[idx_x * GLYZ_SIZE + idx_y * GLZ_SIZE + idx_z] < 1));
}

inline void AstarPathFinder::AstarGetSucc(GridNodePtr currentPtr, vector<GridNodePtr> &neighborPtrSets,
                                          vector<double> &edgeCostSets)
{
    /*
    STEP 4: finish AstarPathFinder::AstarGetSucc yourself
    please write your code below
    */
    // get all neighbours of current node, calculate all the costs, and then save to the point.
    // should be a 3D search, maxumin 26 nodes, but need to remove the obstacles and boundary
    neighborPtrSets.clear();
    edgeCostSets.clear();

    Vector3i neighborIdx;
    for (int dx = -1; dx < 2; dx++)
    {
        for (int dy = -1; dy < 2; dy++)
        {
            for (int dz = -1; dz < 2; dz++)
            {
                if (dx == 0 && dy == 0 && dz == 0)
                    continue;

                neighborIdx(0) = (currentPtr->index)(0) + dx;
                neighborIdx(1) = (currentPtr->index)(1) + dy;
                neighborIdx(2) = (currentPtr->index)(2) + dz;

                if (neighborIdx(0) < 0 || neighborIdx(0) >= GLX_SIZE || neighborIdx(1) < 0 || neighborIdx(1) >= GLY_SIZE ||
                    neighborIdx(2) < 0 || neighborIdx(2) >= GLZ_SIZE)
                    continue;

                neighborPtrSets.push_back(GridNodeMap[neighborIdx(0)][neighborIdx(1)][neighborIdx(2)]);
                edgeCostSets.push_back(sqrt(dx * dx + dy * dy + dz * dz));
            }
        }
    }
}

double AstarPathFinder::getHeu(GridNodePtr node1, GridNodePtr node2)
{
    /*STEP 1
    choose possible heuristic function you want
    Manhattan, Euclidean, Diagonal, or 0 (Dijkstra)
    Remember tie_breaker learned in lecture, add it here ?
    */
    // ROS_INFO("[node] calcute Heu");
    // return getDiagHeu(node1, node2);
    double tie_breaker = 1 + 1 / 1000;
    return tie_breaker * getDiagHeu(node1, node2);
}

double AstarPathFinder::getEuclHeu(GridNodePtr node1, GridNodePtr node2)
{
    // calculate distance at each dimention
    double dx = node1->index(0) - node2->index(0);
    double dy = node1->index(1) - node2->index(1);
    double dz = node1->index(2) - node2->index(2);

    double result1 = sqrt(dx * dx + dy * dy + dz * dz);
    // double result2 = (node2->index - node1->index).norm();

    // cout.setf(ios::fixed);
    // cout << "norm1 = " << setprecision(4) << result1 << endl;
    // cout << "diff = " << (node2->index - node1->index) << endl;
    // cout << "norm2 = " << setprecision(4) << result2 << endl;

    return result1;
}

double AstarPathFinder::getManhHeu(GridNodePtr node1, GridNodePtr node2)
{
    double dx = abs(node1->index(0) - node2->index(0));
    double dy = abs(node1->index(1) - node2->index(1));
    double dz = abs(node1->index(2) - node2->index(2));

    return dx + dy + dz;
}

double AstarPathFinder::getDiagHeu(GridNodePtr node1, GridNodePtr node2)
{
    double dx = abs(node1->index(0) - node2->index(0));
    double dy = abs(node1->index(1) - node2->index(1));
    double dz = abs(node1->index(2) - node2->index(2));

    double h = 0.0;
    int diag = min(min(dx, dy), dz);
    dx -= diag;
    dy -= diag;
    dz -= diag;

    if (dx == 0)
        h = 1.0 * sqrt(3.0) * diag + sqrt(2.0) * min(dy, dz) + 1.0 * abs(dy - dz);
    if (dy == 0)
        h = 1.0 * sqrt(3.0) * diag + sqrt(2.0) * min(dx, dz) + 1.0 * abs(dx - dz);
    if (dz == 0)
        h = 1.0 * sqrt(3.0) * diag + sqrt(2.0) * min(dx, dy) + 1.0 * abs(dx - dy);

    return h;
}

void AstarPathFinder::AstarGraphSearch(Vector3d start_pt, Vector3d end_pt)
{
    ros::Time time_1 = ros::Time::now();

    // index of start_point and end_point
    Vector3i start_idx = coord2gridIndex(start_pt);  // what is coord2gridIndex mean?
    Vector3i end_idx = coord2gridIndex(end_pt);
    goalIdx = end_idx;

    // position of start_point and end_point
    start_pt = gridIndex2coord(start_idx);
    end_pt = gridIndex2coord(end_idx);

    // Initialize the pointers of struct GridNode which represent start node and
    // goal node
    GridNodePtr startPtr = new GridNode(start_idx, start_pt);
    GridNodePtr endPtr = new GridNode(end_idx, end_pt);

    // openSet is the open_list implemented through multimap in STL library
    openSet.clear();
    // currentPtr represents the node with lowest f(n) in the open_list
    GridNodePtr currentPtr = NULL;
    GridNodePtr neighborPtr = NULL;

    // put start node in open set
    startPtr->gScore = 0;
    startPtr->fScore = getHeu(startPtr, endPtr);  // f = h + g = h + 0
    startPtr->id = 1;
    startPtr->coord = start_pt;
    openSet.insert(make_pair(startPtr->fScore, startPtr));
    // startPtr->cameFrom = startPtr;

    /**
     STEP 2: some else preparatory works which should be done before while loop
    please write your code below, neighbour of start point
    **/
    double tentative_gScore;

    vector<GridNodePtr> neighborPtrSets;
    vector<double> edgeCostSets;

    // this is the main loop
    while (!openSet.empty())
    {
        /*
        step 3: Remove the node with lowest cost function from open set to closed
        set, please write your code below
        IMPORTANT NOTE!!!
        This part you should use the C++ STL: multimap,
        more details can be find in Homework description
        */
        // get the min f node from open set to current
        currentPtr = openSet.begin()->second;

        // if the current node is the goal
        if (currentPtr->index == goalIdx)
        {
            ros::Time time_2 = ros::Time::now();
            terminatePtr = currentPtr;
            ROS_INFO("[astar] [A*]{sucess} Time in A*  is %.1f ms, path cost is %.1f m", (time_2 - time_1).toSec() * 1000.0,
                    currentPtr->gScore * resolution);
            // cout << "final point" << endl << terminatePtr->coord << endl;
            // cout << "came frome" << endl << terminatePtr->cameFrom->coord << endl;
            return;
        }

        // put to close, and erase it
        openSet.erase(openSet.begin());
        currentPtr->id = -1;

        // STEP 4: finish AstarPathFinder::AstarGetSucc, get the succetion
        AstarGetSucc(currentPtr, neighborPtrSets, edgeCostSets);

        /***
        STEP 5:  For all unexpanded neigbors "m" of node "n", please finish this for
        loop please write your code below
        **/
        for (int i = 0; i < (int)neighborPtrSets.size(); i++)
        {
            /*
            Judge if the neigbors have been expanded，please write your code below
            IMPORTANT NOTE!!!
            neighborPtrSets[i]->id = -1 : in closed set
            neighborPtrSets[i]->id = 1 : in open set
            */
            neighborPtr = neighborPtrSets[i];
            if (isOccupied(neighborPtr->index) || neighborPtr->id == -1)
                continue;

            double edge_cost = edgeCostSets[i];
            tentative_gScore = currentPtr->gScore + edge_cost;

            if (neighborPtr->id != 1)
            {  // discover a new node, which is not in the open set
                /*
                STEP 6:  As for a new node, do what you need do ,and then put neighbor
                in open set and record it，please write your code below
                */
                // insert to open set
                openSet.insert(make_pair(startPtr->fScore, neighborPtrSets[i]));
                neighborPtr->id = 1;
                neighborPtr->cameFrom = currentPtr;
                neighborPtr->gScore = tentative_gScore;
                neighborPtr->fScore = neighborPtr->gScore + getHeu(neighborPtr, endPtr);
                neighborPtr->nodeMapIt =
                    openSet.insert(make_pair(neighborPtr->fScore, neighborPtr));  // put neighbor in open set and record it.
                continue;
            }
            else if (neighborPtr->gScore >= tentative_gScore)  // compare original f and new f from current
            // this node is in open set and need to judge if it needs to update,
            // he "0" should be deleted when you are coding
            {
                /*
                STEP 7:  As for a node in open set, update it , maintain the openset,
                and then put neighbor in open set and record it please write your code below
                */
                neighborPtr->cameFrom = currentPtr;
                neighborPtr->gScore = tentative_gScore;
                neighborPtr->fScore = tentative_gScore + getHeu(neighborPtr, endPtr);
                openSet.erase(neighborPtr->nodeMapIt);
                neighborPtr->nodeMapIt = openSet.insert(make_pair(neighborPtr->fScore, neighborPtr));
                // if change its parents, update the expanding direction
                for (int i = 0; i < 3; i++)
                {
                    neighborPtr->dir(i) = neighborPtr->index(i) - currentPtr->index(i);
                    if (neighborPtr->dir(i) != 0)
                        neighborPtr->dir(i) /= abs(neighborPtr->dir(i));
                }
            }
        }
    }
    // if search fails
    ros::Time time_2 = ros::Time::now();
    if ((time_2 - time_1).toSec() > 0.1)
        ROS_INFO("[astar] [A*]{fail} Time cost in Astar planner is %.1f", (time_2 - time_1).toSec());
}

vector<Vector3d> AstarPathFinder::getPath()
{
    /*
    STEP 8:  trace back from the curretnt nodePtr to get all nodes along the path
    please write your code below
    */
    vector<Vector3d> path;
    vector<GridNodePtr> gridPath;

    if (terminatePtr == nullptr)
        return path;

    gridPath.push_back(terminatePtr);

    while (terminatePtr->cameFrom != NULL)
    {
        terminatePtr = terminatePtr->cameFrom;
        gridPath.push_back(terminatePtr);
    }

    for (auto ptr : gridPath)
        path.push_back(ptr->coord);

    reverse(path.begin(), path.end());

    return path;
}

