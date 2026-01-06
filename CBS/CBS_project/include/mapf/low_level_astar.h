#pragma once
#include <vector>
#include "grid.h"
#include "constraints.h"

namespace mapf {

// 带约束的 Space-Time A*（并做 goal-safe 到 maxT，返回路径会补齐到 maxT+1）
Path spaceTimeAStar(const Grid& grid, Pos start, Pos goal, int maxT, const ConstraintTable& ct);

} // namespace mapf
