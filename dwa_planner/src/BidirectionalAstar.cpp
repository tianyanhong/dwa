#include <iostream>
#include <vector>
#include <queue>
#include <unordered_map>
#include <cmath>
#include <algorithm>

using namespace std;

struct Node {
    int x, y;
    int g;      // 实际代价
    int f;      // f = g + h
    Node* parent;

    Node(int _x, int _y, int _g, int _f, Node* _parent = nullptr)
        : x(_x), y(_y), g(_g), f(_f), parent(_parent) {}

    bool operator<(const Node& other) const {
        return f > other.f; // 小顶堆
    }
};

//需要栅格地图  加载全局地图和局部地图  grid_map

vector<Node*> bidirectionalAStar(
    const vector<vector<int>>& grid,
    pair<int,int> start,
    pair<int,int> goal
) {
    int w = grid[0].size();
    int h = grid.size();

    unordered_map<pair<int,int>, Node*, NodeHash> forwardOpenMap, backwardOpenMap;
    unordered_map<pair<int,int>, Node*, NodeHash> forwardClosed, backwardClosed;

    priority_queue<Node> fOpen, bOpen;

    auto forwardStart = new Node(start.first, start.second, 0,
        heuristic(start.first, start.second, goal.first, goal.second));
    auto backwardStart = new Node(goal.first, goal.second, 0,
        heuristic(goal.first, goal.second, start.first, start.second));

    fOpen.push(*forwardStart);
    bOpen.push(*backwardStart);

    forwardOpenMap[start] = forwardStart;
    backwardOpenMap[goal] = backwardStart;

    const int dx[8] = {1,-1,0,0,1,1,-1,-1};
    const int dy[8] = {0,0,1,-1,1,-1,1,-1};
    const int cost[8] = {1,1,1,1,14,14,14,14}; // 10 or 14

    Node* meetNode = nullptr;

    while (!fOpen.empty() && !bOpen.empty()) {
        // Forward search
        Node cur = fOpen.top(); fOpen.pop();
        forwardOpenMap.erase({cur.x, cur.y});
        forwardClosed[{cur.x, cur.y}] = new Node(cur.x, cur.y, cur.g, cur.f, cur.parent);

        if (backwardClosed.count({cur.x, cur.y})) {
            meetNode = forwardClosed[{cur.x, cur.y}];
            break;
        }

        for (int i = 0; i < 8; ++i) {
            int nx = cur.x + dx[i];
            int ny = cur.y + dy[i];
            if (!inBounds(nx, ny, w, h) || grid[ny][nx] == 1) continue;

            int ng = cur.g + cost[i];
            if (forwardClosed.count({nx, ny}) ||
                (forwardOpenMap.count({nx, ny}) && forwardOpenMap[{nx, ny}]->g <= ng))
                continue;

            int nf = ng + heuristic(nx, ny, goal.first, goal.second);
            Node* nnode = new Node(nx, ny, ng, nf, forwardClosed[{cur.x, cur.y}]);
            fOpen.push(*nnode);
            forwardOpenMap[{nx, ny}] = nnode;
        }

        // Backward search
        Node bcur = bOpen.top(); bOpen.pop();
        backwardOpenMap.erase({bcur.x, bcur.y});
        backwardClosed[{bcur.x, bcur.y}] = new Node(bcur.x, bcur.y, bcur.g, bcur.f, bcur.parent);

        if (forwardClosed.count({bcur.x, bcur.y})) {
            meetNode = forwardClosed[{bcur.x, bcur.y}];
            break;
        }

        for (int i = 0; i < 8; ++i) {
            int nx = bcur.x + dx[i];
            int ny = bcur.y + dy[i];
            if (!inBounds(nx, ny, w, h) || grid[ny][nx] == 1) continue;

            int ng = bcur.g + cost[i];
            if (backwardClosed.count({nx, ny}) ||
                (backwardOpenMap.count({nx, ny}) && backwardOpenMap[{nx, ny}]->g <= ng))
                continue;

            int nf = ng + heuristic(nx, ny, start.first, start.second);
            Node* nnode = new Node(nx, ny, ng, nf, backwardClosed[{bcur.x, bcur.y}]);
            bOpen.push(*nnode);
            backwardOpenMap[{nx, ny}] = nnode;
        }
    }

    vector<Node*> path;
    if (!meetNode) return path;

    // Forward path
    Node* n = meetNode;
    while (n) {
        path.push_back(n);
        n = n->parent;
    }
    reverse(path.begin(), path.end());

    // Backward path
    Node* bn = backwardClosed[{meetNode->x, meetNode->y}]->parent;
    while (bn) {
        path.push_back(new Node(bn->x, bn->y, bn->g, bn->f, nullptr));
        bn = bn->parent;
    }

    return path;
}


int main() {
    vector<vector<int>> grid = {
        {0,0,0,0,0},
        {0,1,1,0,0},
        {0,1,0,0,0},
        {0,0,0,1,0},
        {0,0,0,0,0}
    };

    auto path = bidirectionalAStar(grid, {0,0}, {4,4});

    cout << "Path found: " << path.size() << " steps\n";
    for (auto& n : path)
        cout << "(" << n->x << "," << n->y << ") ";
    cout << endl;

    return 0;
}