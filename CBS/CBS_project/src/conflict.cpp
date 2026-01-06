#include "mapf/conflict.h"
#include <algorithm>

namespace mapf {

Pos posAt(const Path& p, int t) {
    if (t < 0) return p.front();
    if (t < (int)p.size()) return p[t];
    return p.back();
}

Conflict detectFirstConflict(const std::vector<Path>& paths) {
    int n = (int)paths.size();
    int T = 0;
    for (const auto& p : paths) T = std::max(T, (int)p.size());

    for (int t = 0; t < T; t++) {
        for (int i = 0; i < n; i++) {
            Pos pi = posAt(paths[i], t);
            Pos pi_prev = posAt(paths[i], t - 1);

            for (int j = i + 1; j < n; j++) {
                Pos pj = posAt(paths[j], t);

                // vertex conflict
                if (pi == pj) {
                    Conflict c; c.exists = true;
                    c.isEdge = false; c.a = i; c.b = j;
                    c.t = t; c.x = pi.x; c.y = pi.y;
                    return c;
                }

                // edge conflict (swap)
                if (t > 0) {
                    Pos pj_prev = posAt(paths[j], t - 1);
                    if (pi_prev == pj && pj_prev == pi) {
                        Conflict c; c.exists = true;
                        c.isEdge = true; c.a = i; c.b = j;
                        c.t = t - 1;
                        c.ax1 = pi_prev.x; c.ay1 = pi_prev.y;
                        c.ax2 = pi.x;      c.ay2 = pi.y;
                        return c;
                    }
                }
            }
        }
    }
    return Conflict{};
}

int sumOfCosts(const std::vector<Path>& paths) {
    int s = 0;
    for (const auto& p : paths) s += std::max(0, (int)p.size() - 1);
    return s;
}

int makespan(const std::vector<Path>& paths) {
    int m = 0;
    for (const auto& p : paths) m = std::max(m, (int)p.size() - 1);
    return m;
}

void padPathsToSameLength(std::vector<Path>& paths) {
    int T = 0;
    for (auto& p : paths) T = std::max(T, (int)p.size());
    for (auto& p : paths) {
        while ((int)p.size() < T) p.push_back(p.back());
    }
}

} // namespace mapf
