#pragma once
#include <vector>
#include <unordered_set>
#include <algorithm>
#include "grid.h"

namespace mapf {

enum class ConstraintType { Vertex, Edge };

struct Constraint {
    int agent = -1;
    ConstraintType type = ConstraintType::Vertex;
    int t = 0;
    int x1 = 0, y1 = 0;   // Vertex: forbid (x1,y1) at time t
    int x2 = 0, y2 = 0;   // Edge  : forbid (x1,y1)->(x2,y2) at time t
};

inline long long keyVertex(int x, int y, int t) {
    return ((long long)t << 40) ^ ((long long)x << 20) ^ (long long)y;
}
inline long long keyEdge(int x1, int y1, int x2, int y2, int t) {
    long long k = (long long)t;
    k = (k << 12) ^ x1; k = (k << 12) ^ y1;
    k = (k << 12) ^ x2; k = (k << 12) ^ y2;
    return k;
}

struct ConstraintTable {
    std::unordered_set<long long> forbV;
    std::unordered_set<long long> forbE;
};

inline bool violatesVertex(const ConstraintTable& ct, int x, int y, int t) {
    return ct.forbV.count(keyVertex(x, y, t)) > 0;
}
inline bool violatesEdge(const ConstraintTable& ct, int x1, int y1, int x2, int y2, int t) {
    return ct.forbE.count(keyEdge(x1, y1, x2, y2, t)) > 0;
}

inline ConstraintTable buildConstraintTable(const std::vector<Constraint>& cons, int agent) {
    ConstraintTable ct;
    for (const auto& c : cons) {
        if (c.agent != agent) continue;
        if (c.type == ConstraintType::Vertex) ct.forbV.insert(keyVertex(c.x1, c.y1, c.t));
        else ct.forbE.insert(keyEdge(c.x1, c.y1, c.x2, c.y2, c.t));
    }
    return ct;
}

inline int maxConstraintTimeForAgent(const std::vector<Constraint>& cons, int agent) {
    int mx = 0;
    for (const auto& c : cons) if (c.agent == agent) mx = std::max(mx, c.t);
    return mx;
}
inline int maxConstraintTimeAll(const std::vector<Constraint>& cons) {
    int mx = 0;
    for (const auto& c : cons) mx = std::max(mx, c.t);
    return mx;
}
inline int lowerBoundLen(const std::vector<Pos>& starts, const std::vector<Pos>& goals) {
    int lb = 0;
    for (int i = 0; i < (int)starts.size(); i++)
        lb = std::max(lb, manhattan(starts[i], goals[i]));
    return lb;
}

} // namespace mapf
