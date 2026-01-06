#pragma once
#include <vector>
#include "grid.h"
#include "constraints.h"

namespace mapf {

// 返回是否找到无冲突解；solution 里是每个 agent 的完整路径
bool CBS(const Grid& grid,
         const std::vector<Pos>& starts,
         const std::vector<Pos>& goals,
         std::vector<Path>& solution);

} // namespace mapf
