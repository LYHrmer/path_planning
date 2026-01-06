#include "mapf/low_level_astar.h"
#include <queue>
#include <unordered_map>
#include <algorithm>

namespace mapf {

struct State {
    int x, y, t;
    bool operator==(const State& o) const { return x==o.x && y==o.y && t==o.t; }
};
struct StateHash {
    size_t operator()(const State& s) const noexcept {
        return ((size_t)s.t<<32) ^ ((size_t)s.x<<16) ^ (size_t)s.y;
    }
};

static std::vector<Pos> neighbors5(const Grid& grid, const Pos& cur) {
    const int dx[5] = {1,-1,0,0,0};
    const int dy[5] = {0,0,1,-1,0};
    std::vector<Pos> res;
    for (int k=0;k<5;k++){
        int nx = cur.x + dx[k], ny = cur.y + dy[k];
        if (grid.passable(nx,ny)) res.push_back(Pos{nx,ny});
    }
    return res;
}

static bool goalSafeToH(const ConstraintTable& ct, const Pos& goal, int t, int H) {
    for (int tau = t; tau <= H; ++tau) {
        if (violatesVertex(ct, goal.x, goal.y, tau)) return false;
        if (tau < H && violatesEdge(ct, goal.x, goal.y, goal.x, goal.y, tau)) return false; // 等待边
    }
    return true;
}

Path spaceTimeAStar(const Grid& grid, Pos start, Pos goal, int maxT, const ConstraintTable& ct) {
    struct Node { State s; int g; int f; };
    struct Cmp {
        bool operator()(const Node& a, const Node& b) const {
            if (a.f != b.f) return a.f > b.f;
            return a.g < b.g;
        }
    };

    if (violatesVertex(ct, start.x, start.y, 0)) return {};

    std::priority_queue<Node, std::vector<Node>, Cmp> open;
    std::unordered_map<State, int, StateHash> bestG;
    std::unordered_map<State, State, StateHash> parent;

    State s0{start.x, start.y, 0};
    bestG[s0] = 0;
    open.push(Node{s0, 0, manhattan(start, goal)});

    while (!open.empty()) {
        Node cur = open.top(); open.pop();
        State cs = cur.s;

        if (cur.g != bestG[cs]) continue;
        if (cs.t == maxT) continue;

        // 到达 goal：必须 goal-safe 才能返回
        if (cs.x == goal.x && cs.y == goal.y) {
            if (goalSafeToH(ct, goal, cs.t, maxT)) {
                std::vector<Pos> rev;
                State p = cs;
                while (true) {
                    rev.push_back(Pos{p.x, p.y});
                    if (p.t == 0) break;
                    p = parent[p];
                }
                std::reverse(rev.begin(), rev.end());
                while ((int)rev.size() < maxT + 1) rev.push_back(rev.back());
                return rev;
            }
            // 不安全：继续找其他到达方式/到达时刻
        }

        for (const auto& np : neighbors5(grid, Pos{cs.x, cs.y})) {
            int nt = cs.t + 1;

            if (violatesVertex(ct, np.x, np.y, nt)) continue;
            if (violatesEdge(ct, cs.x, cs.y, np.x, np.y, cs.t)) continue;

            State ns{np.x, np.y, nt};
            int ng = cur.g + 1;

            auto it = bestG.find(ns);
            if (it == bestG.end() || ng < it->second) {
                bestG[ns] = ng;
                parent[ns] = cs;
                int nf = ng + manhattan(Pos{ns.x, ns.y}, goal);
                open.push(Node{ns, ng, nf});
            }
        }
    }
    return {};
}

} // namespace mapf
