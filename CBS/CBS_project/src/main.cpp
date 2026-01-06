#include <iostream>
#include "mapf/cbs.h"
#include "mapf/conflict.h"

using namespace mapf;

int main() {
    Grid grid;
    grid.g = {
        "..........",
        ".####.....",
        "..........",
        ".....####.",
        ".........."
    };
    grid.H = (int)grid.g.size();
    grid.W = (int)grid.g[0].size();

    std::vector<Pos> starts = { {0,0}, {0,4} };
    std::vector<Pos> goals  = { {9,4}, {2,4} };

    std::vector<Path> sol;
    bool ok = CBS(grid, starts, goals, sol);
    if (!ok) {
        std::cout << "No solution.\n";
        return 0;
    }

    std::cout << "CBS solution found!\n";
    for (int i = 0; i < (int)sol.size(); i++) {
        std::cout << "Agent " << i << ":\n";
        int showT = 20;
        for (int t = 0; t < std::min((int)sol[i].size(), showT); t++) {
            std::cout << "  t=" << t << " (" << sol[i][t].x << "," << sol[i][t].y << ")\n";
        }
    }
    return 0;
}
