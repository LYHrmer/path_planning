#include "mapf/cbs.h"
#include "mapf/low_level_astar.h"
#include "mapf/conflict.h"

#include <queue>
#include <utility>
#include <iostream>
#include <algorithm>

namespace mapf {

struct CTNode {
    std::vector<Constraint> constraints;
    std::vector<Path> paths;
    int cost = 0;
    int id = 0;
};

struct CTNodeCmp {
    bool operator()(const CTNode& a, const CTNode& b) const {
        if (a.cost != b.cost) return a.cost > b.cost;
        return a.id > b.id;
    }
};

bool CBS(const Grid& grid,
         const std::vector<Pos>& starts,
         const std::vector<Pos>& goals,
         std::vector<Path>& solution) {

    int n = (int)starts.size();
    int nodeId = 0;

    auto replanAgent = [&](CTNode& node, int agent) -> bool {
        ConstraintTable ct = buildConstraintTable(node.constraints, agent);

        int lb    = lowerBoundLen(starts, goals);
        int curMS = (!node.paths.empty() ? makespan(node.paths) : 0);
        int mxA   = maxConstraintTimeForAgent(node.constraints, agent);
        int mxAll = maxConstraintTimeAll(node.constraints);

        int maxT = std::max({lb, curMS, mxA, mxAll}) + 10;

        // 迭代加深：防止 maxT 估计偏小误判无解
        for (int attempt = 0; attempt < 3; attempt++) {
            Path p = spaceTimeAStar(grid, starts[agent], goals[agent], maxT, ct);
            if (!p.empty()) {
                node.paths[agent] = std::move(p);
                return true;
            }
            maxT += 10;
        }
        return false;
    };

    CTNode root;
    root.id = nodeId++;
    root.paths.resize(n);

    for (int i = 0; i < n; i++) {
        if (!replanAgent(root, i)) return false;
    }
    padPathsToSameLength(root.paths);
    root.cost = sumOfCosts(root.paths);

    std::priority_queue<CTNode, std::vector<CTNode>, CTNodeCmp> open;
    open.push(root);

    while (!open.empty()) {
        CTNode cur = open.top(); open.pop();

        Conflict conf = detectFirstConflict(cur.paths);
        if (!conf.exists) {
            solution = cur.paths;
            return true;
        }

        for (int k = 0; k < 2; k++) {
            int agent = (k == 0 ? conf.a : conf.b);

            CTNode child = cur;
            child.id = nodeId++;

            // 添加约束（CBS 分裂）
            if (!conf.isEdge) {
                child.constraints.push_back(Constraint{
                    agent, ConstraintType::Vertex, conf.t,
                    conf.x, conf.y, 0, 0
                });
            } else {
                if (agent == conf.a) {
                    child.constraints.push_back(Constraint{
                        agent, ConstraintType::Edge, conf.t,
                        conf.ax1, conf.ay1, conf.ax2, conf.ay2
                    });
                } else {
                    child.constraints.push_back(Constraint{
                        agent, ConstraintType::Edge, conf.t,
                        conf.ax2, conf.ay2, conf.ax1, conf.ay1
                    });
                }
            }

            if (!replanAgent(child, agent)) continue;

            padPathsToSameLength(child.paths);
            child.cost = sumOfCosts(child.paths);
            open.push(std::move(child));
        }
    }
    return false;
}

} // namespace mapf
