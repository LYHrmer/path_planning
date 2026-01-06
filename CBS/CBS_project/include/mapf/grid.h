#pragma once
#include <vector>
#include <string>
#include <cstdlib>

namespace mapf {

struct Pos {
    int x = 0;
    int y = 0;
    bool operator==(const Pos& o) const { return x == o.x && y == o.y; }
};

using Path = std::vector<Pos>;

struct Grid {
    int W = 0;
    int H = 0;
    std::vector<std::string> g;

    bool inBounds(int x, int y) const { return 0 <= x && x < W && 0 <= y && y < H; }
    bool passable(int x, int y) const { return inBounds(x, y) && g[y][x] != '#'; }
};

inline int manhattan(const Pos& a, const Pos& b) {
    return std::abs(a.x - b.x) + std::abs(a.y - b.y);
}

} // namespace mapf
