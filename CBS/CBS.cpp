#include <algorithm>
#include <iostream>
#include <vector>
#include <string>
#include <queue>
#include <unordered_map>
#include <cmath>

using namespace std;

// 修正：结构体名统一为Pos（大写P），和main里一致
struct Pos{
    int x;
    int y;

    // C++ 运算符重载；以后可以直接写(a=b)
    bool operator==(const Pos& other) const{
        return x == other.x && y == other.y;
    }
};

/* 网格地图
   g 用vector<string>保存每一行
   g[y][x] 访问第y行第x列的字符（修正坐标顺序）
 */
struct Grid{
    int W;  // 宽（列数）
    int H;  // 高（行数）
    vector<string> g; // 地图字符

    // 判断坐标是否在地图边界内
    bool inBounds(int x, int y) const{
        return 0 <= x && x < W && 0 <= y && y < H;
    }

    // 判断该位置是否可走（修正坐标访问：g[y][x]）
    bool passable(int x, int y) const{
        return inBounds(x, y) && g[y][x] != '#';
    }

    // 打印地图
    void print() const{
        for (int y = 0; y < H; y++){
            cout << g[y] << "\n";
        }
    }
};

//---------关键：时空状态（x,y,t）-------
struct State{
    int x,y,t;
    bool operator==(const State& o) const { return x == o.x 
        && y == o.y && t == o.t; }
};

// unordered_map 需要 hash:我们把（x,y,t）打包到size_t
struct StateHash {
    size_t operator()(const State& s) const noexcept {
        // 简单打包：t 放高位，x,y 放低位（这里假设不会太大）
        return ((size_t)s.t << 32) ^ ((size_t)s.x << 16) ^ (size_t)s.y;
    }
};

// 计算曼哈顿距离
int manhattan(const Pos& a, const Pos& b){
    return abs(a.x - b.x) + abs(a.y - b.y);
}

/* 生成下一步候选位置（4邻接+等待,t在外层+1）
    返回值是 vector<Pos>（动态数组）
*/
vector<Pos> getNeighbors(const Grid& grid, const Pos& cur){
    //五个动作：上、下、左、右、等待
    const int dx[5] = {1,-1,0,0,0};
    const int dy[5] = {0,0,1,-1,0};

    vector<Pos> res;
    for (int k = 0; k < 5; k++){
        int nx = cur.x + dx[k];
        int ny = cur.y + dy[k];
        if (grid.passable(nx,ny)){
            res.push_back(Pos{nx,ny}); //花括号初试后一个Pos
        }
    }
    return res;
}

/*
    最小的 Space-Time BFS(无约束版)
    限制最大时间 maxT，避免无限等待
*/
vector<Pos> spaceTimeBFS(const Grid& grid, Pos start, Pos goal, int maxT){
    queue<State> q;

    // visiter / parent : 记录回溯路径
    unordered_map<State, State, StateHash> parent;
    unordered_map<State, bool, StateHash> visited;

    State s{start.x, start.y, 0};
    q.push(s);
    visited[s] = true;

    State goalState{-1,-1,-1};//记录目标状态
    bool found = false;

    while (!q.empty()){
        State cur = q.front(); q.pop();

        // 到达目标：停止
        if (cur.x == goal.x && cur.y == goal.y){
            goalState = cur;
            found = true;
            break;
        }

        if (cur.t >= maxT) continue; // 超过最大时间，跳过

        //拓展下一步（t+1）
        vector<Pos> nb = getNeighbors(grid, Pos{cur.x, cur.y});
        for(const auto& p : nb){
            State nxt{p.x, p.y, cur.t+1};
            if(!visited[nxt]){
                visited[nxt] = true;
                parent[nxt] = cur;
                q.push(nxt);
            }
        }
    }
    
    if(!found) return {}; //未找到路径返回空

    // 回溯：从goalState回到起点
    vector<Pos> rev;
    State cur = goalState;
    while (true){
        rev.push_back(Pos{cur.x,cur.y});
        if(cur.t == 0) break; //到达起点
        cur = parent[cur];
    }

    //反转得到正向路径
    reverse(rev.begin(), rev.end());
    return rev;
}

/*
  Space-Time A*（无约束版）
  - g = 走了多少步（每步+1）
  - h = 曼哈顿距离到目标
  - f = g + h
*/
vector <Pos> spaceTimeAStar(const Grid& grid, Pos start, Pos goal, int maxT){

    // 优先队列节点：保存状态以及g,f值
    struct Node{
        State s;
        int g;//实际代价
        int f;//估计总代价
    };

    //priority_queue 默认弹出"最大"的，所以比较器要写成：f 更小优先
    struct Cmp{
        bool operator()(const Node& a, const Node& b) const{
            if (a.f != b.f) return a.f > b.f; // f 小的优先
            return a.g > b.g; // 同f时可选：g大优先
        }
    };

    priority_queue<Node, vector<Node>,Cmp> open;

    unordered_map<State, int, StateHash> bestG; //记录每个state的最小G
    unordered_map<State, State, StateHash> parent; //回溯
    
    State s0{start.x, start.y, 0};
    bestG[s0] = 0;
    open.push(Node{s0,0,manhattan(start,goal)});

    return {};
}

int main(){
    // 修正：先声明Grid类型的grid变量
    Grid grid;

    // 1) 构造小地图:'.'可走，'#'障碍（每行长度统一，避免越界）
    grid.g = {
        "..........",
        ".####.....",
        "..........",
        ".....####.",
        ".........."
    };

    // 2) 计算宽高
    grid.H = (int)grid.g.size();       // 行数=高
    grid.W = (int)grid.g[0].size();    // 列数=宽


    Pos start{0,0};
    Pos goal{9,4};

    cout << "Map:\n";
    grid.print();
    cout << "\n";

    // BFS 最大时间上限，先随便给个够大的（以后做 CBS 会用更聪明的 horizon）
    int maxT = 50;
    vector<Pos> path = spaceTimeBFS(grid, start, goal, maxT);

    if (path.empty()) {
        cout << "No path within maxT=" << maxT << "\n";
        return 0;
    }

    cout << "Found path length=" << (int)path.size() << " (steps=" << (int)path.size()-1 << ")\n";
    for (int t = 0; t < (int)path.size(); t++) {
        cout << "t=" << t << " (" << path[t].x << "," << path[t].y << ")\n";
    }

    return 0;
}