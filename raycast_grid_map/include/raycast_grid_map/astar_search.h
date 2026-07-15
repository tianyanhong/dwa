#include <queue>
#include <unordered_set>
#include <cmath>
#include <Eigen/Dense>
#include <raycast_grid_map/grid_map.h>

struct AStarNode
{
    int x, y;
    double g, h;
    AStarNode* parent;

    AStarNode(int x_, int y_, double g_, double h_, AStarNode* p = nullptr)
    : x(x_), y(y_), g(g_), h(h_), parent(p) {}

    double f() const { return g + h; }

    bool operator>(const AStarNode& other) const
    {
        return f() > other.f();
    }
};

bool isFree(int x, int y, const GridMap& map)
{
    if (!map.inBounds(x, y))
        return false;

    int8_t cost = map.collision[map.index(x, y)];  
    // 非障碍 & 非膨胀
    return static_cast<int>(cost) != 100;
}

std::vector<Eigen::Vector3d>
AStarSearch(const GridMap& map,
            const Eigen::Vector3d& start,
            const Eigen::Vector3d& goal)
{
    int sx, sy, gx, gy;
    map.worldToIndex(start, sx, sy);
    map.worldToIndex(goal, gx, gy);

    auto cmp = [](AStarNode* a, AStarNode* b)
    { return a->f() > b->f(); };

    std::priority_queue<
        AStarNode*,
        std::vector<AStarNode*>,
        decltype(cmp)> open_set(cmp);

    std::unordered_set<int> closed_set;

    auto heuristic = [&](int x, int y)
    {
        return std::hypot(x - gx, y - gy);
    };

    open_set.push(new AStarNode(
        sx, sy, 0.0, heuristic(sx, sy)));

    const int dx[8] = {-1,-1,0,1,1,1,0,-1};
    const int dy[8] = {0,1,1,1,0,-1,-1,-1};

    for (int i = 0; i < 8; ++i)
    {
        int nx = sx + dx[i];
        int ny = sy + dy[i];
        open_set.push(new AStarNode(
            nx, ny, 0.0, heuristic(nx, ny)));
    }

    // std::cout << open_set.size() << std::endl;

    while (!open_set.empty())
    {
        AStarNode* cur = open_set.top();
        open_set.pop();

        if (std::abs(cur->x - gx) < 2 && std::abs(cur->y - gy) < 2)
        {
            // 回溯路径
            std::vector<Eigen::Vector3d> path;
            while (cur)
            {
                Eigen::Vector3d p(
                    cur->x * map.resolution + map.x_min,
                    cur->y * map.resolution + map.y_min,
                    0.0);
                path.push_back(p);
                cur = cur->parent;
            }
            std::reverse(path.begin(), path.end());
            return path;
        }

        int idx = map.index(cur->x, cur->y);
        if (closed_set.count(idx)) continue;
        closed_set.insert(idx);

        for (int i = 0; i < 8; ++i)
        {
            int nx = cur->x + dx[i];
            int ny = cur->y + dy[i];

            if (!isFree(nx, ny, map))
                continue;

            int nidx = map.index(nx, ny);
            if (closed_set.count(nidx)) continue;

            double ng = cur->g + std::hypot(dx[i], dy[i]);
            open_set.push(new AStarNode(
                nx, ny, ng, heuristic(nx, ny), cur));
        }
    }

    // 没找到路径
    return {};
}