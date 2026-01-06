#pragma once
#include <vector>
#include "grid.h"

namespace mapf {

struct Conflict {
    bool exists = false;
    bool isEdge = false;
    int a = -1, b = -1;
    int t = -1;
    // vertex
    int x = -1, y = -1;
    // edge（以 a 的移动为准）
    int ax1=0, ay1=0, ax2=0, ay2=0;
};

Pos posAt(const Path& p, int t);
Conflict detectFirstConflict(const std::vector<Path>& paths);

// 工具
int sumOfCosts(const std::vector<Path>& paths);
int makespan(const std::vector<Path>& paths);
void padPathsToSameLength(std::vector<Path>& paths);

} // namespace mapf
