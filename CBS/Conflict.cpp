#include <iostream>
#include <vector>
#include <string>
#include <cmath>

using namespace std;

struct Pos {
    int x, y;
    bool operator==(const Pos& o) const { return x == o.x && y == o.y; }
};

using Path = vector<Pos>; 

// 如果 t 超过路径长度，就返回最后一个位置（表示到达目标后停留）
Pos posAt(const Path& p, int t){
    if (t<0) return p.front();
    if (t < (int)p.size()) return p[t];
    return p.back(); 
}

struct Conflict
{
    bool exists = false;//是否存在冲突
    bool isEdge = false;//是否为边冲突
    int a = -1, b = -1;
    int t = -1;

    //vertex conflict
    int x = -1, y = -1;
    //edge conflict(以a的移动为准)
    int ax1, ay1, ax2, ay2;
};

Conflict detectFirstConflict(const vector<Path>& paths){
    int n = (int)paths.size();
    int T = 0;
    for (const auto& p : paths) T = max(T, (int)p.size());

    for (int t = 0; t < T; t++)
    {
        for (int i = 0; i < n; i++)
        {
            Pos pi = posAt(paths[i], t);
            Pos pi_prev = posAt(paths[i], t - 1);

            for (int j = i + 1; j < n; j++)
            {
                Pos pj = posAt(paths[j],t);

                //点冲突：同一时刻同一格
                if (pi == pj)
                {
                    Conflict c;
                    c.exists = true;
                    c.isEdge = false;
                    c.a = i; c.b = j; //冲突的两个智能体
                    c.t = t;
                    c.x = pi.x; c.y = pi.y;
                    return c;
                }

                if (t > 0)
                {
                    Pos pj_prev = posAt(paths[j], t-1);//上一步位置
                    if (pi_prev == pj && pj_prev == pi)
                    {
                        Conflict c;
                        c.exists = true;
                        c.isEdge = true;
                        c.a = i; c.b = j;
                        c.t = t;
                        c.ax1 = pi_prev.x; c.ay1 = pi_prev.y;
                        c.ax2 = pi.x; c.ay2 = pi.y;
                        return c;
                    }
                }    
            }
        } 
    }   
}

