#include <algorithm>
#include <iostream>
#include <vector>
#include <string>
#include <queue>
#include <unordered_map>
#include <unordered_set>
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

//----------CBS 底层需要的约束结构 -------
enum class ConstraintType{
    Vertex, //顶点约束
    Edge   //边约束
};

struct Constraint{
    int agent; //哪个 agent (本步只有1个，可先写死为0)
    ConstraintType type; //约束类型
    int t; //时间
    int x1,y1; //Vertex:禁止到达（x1,y1）
    int x2,y2; //Edge:禁止从（x1,y1）到（x2，y2）
};


// 把 (x,y,t) 打包成一个 long long，便于放进 unordered_set
long long keyVertex(int x, int y, int t){
    return ((long long)t << 40)^((long long)x << 20)^(long long)y;//假设x，y，z都不会太大
}

// 把边约束 (x1,y1,x2,y2,t) 打包
long long keyEdge(int x1, int y1, int x2, int y2, int t){
    long long k = (long long)t; //时间放高位
    k = (k<<12) ^ x1; k = (k<<12) ^ y1; 
    k = (k<<12) ^ x2; k = (k<<12) ^ y2;
    return k;
}

// 一个 agent 的“约束表”：快速查点约束/边约束
struct ConstraintTable{
    unordered_set<long long> forbV;//顶点集合约束
    unordered_set<long long> forbE;//边集合约束
};

//---------关键：时空状态（x,y,t）-------
struct State{
    int x,y,t;
    bool operator==(const State& o) const { return x == o.x 
        && y == o.y && t == o.t; }
};

// unordered_map 需要 hash:我们把（x,y,t）打包到size_t
struct StateHash {
    size_t operator()(const State& s) const noexcept  {
        // 简单打包：t 放高位，x,y 放低位（这里假设不会太大）
        return ((size_t)s.t << 32) ^ ((size_t)s.x << 16) ^ (size_t)s.y;
    }
};

// 计算曼哈顿距离
int manhattan(const Pos& a, const Pos& b){
    return abs(a.x - b.x) + abs(a.y - b.y);
}

//约束检查：点约束
bool violatesVertex(const ConstraintTable& ct, int x, int y, int t){
    return ct.forbV.count( keyVertex(x,y,t)) > 0;
}
//约束检查：边约束（注意 t 指的是这次移动的起始时间）
bool violatesEdge(const ConstraintTable& ct, int x1,int y1,int x2,int y2,int t) {
    return ct.forbE.count(keyEdge(x1, y1, x2, y2, t)) > 0;
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
  Space-Time A*（约束版）
  - g = 走了多少步（每步+1）
  - h = 曼哈顿距离到目标
  - f = g + h
  在拓展时检查 vertex/edge 约束
*/
vector <Pos> spaceTimeAStarConstrained(const Grid& grid, Pos start, Pos goal, int maxT, const ConstraintTable& ct){

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
 
    //起点也要检查点约束（t=0 时不能站在被禁格）
    if (violatesVertex(ct, start.x, start.y, 0)) return{};

    State s0{start.x, start.y, 0};
    bestG[s0] = 0;
    open.push(Node{s0,0,manhattan(start,goal)});

    while (!open.empty())
    {
        Node cur = open.top(); open.pop();// 先读取栈open的栈顶元素赋值给cur，再将栈顶元素从栈中删除
        State cs = cur.s;

        // "懒惰删除"：如果弹出的g不是当前最优，跳过
        if(cur.g != bestG[cs]) continue;

        // 到达目标就结束（注意：此时 t 不一定最小，但 A*保证第一次出队目标就是最优）
        if(cs.x == goal.x && cs.y == goal.y){
            //回溯路径
            vector<Pos> rev;
            State p =cs;
            while(true){
                rev.push_back(Pos{p.x,p.y});
                if(p.t ==0) break;
                p = parent[p];
            }
            reverse(rev.begin(), rev.end());
            return rev;
        }

        if (cs.t >= maxT) continue; // 超过最大时间，跳过

        //拓展邻居（t+1）
        vector<Pos> nb = getNeighbors(grid, Pos{cs.x, cs.y});
        for(const auto& np : nb){
            int nt = cs.t +1;

            // 1) 检查点约束：下一时刻 nt 不能到达（np.x,np.y）
            if (violatesVertex(ct, np.x, np.y, nt)) continue;
            // 2) 检查边约束：这次移动cs(t)->np(t+1) 是否被禁止
            if (violatesEdge(ct, cs.x, cs.y, np.x, np.y, cs.t)) continue;

            State ns{np.x, np.y, cs.t+1};
            int ng = cur.g +1; //每步代价＋1

            // 如果这个状态从没见过，或者找到更短的g，就更新
            auto it = bestG.find(ns);
            if(it == bestG.end() || ng < it->second){
                bestG[ns] = ng;
                parent[ns] = cs;

                int h = manhattan(Pos{ns.x, ns.y}, goal);// 估计代价
                int nf = ng + h;
                open.push(Node{ns, ng, nf});//入栈
            }
        }
    }
    

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
    grid.print(); //打印地图
    cout << "\n";

    // BFS 最大时间上限，先随便给个够大的（以后做 CBS 会用更聪明的 horizon）
    int maxT = 50;
    vector<Pos> path = spaceTimeAStarConstrained(grid, start, goal, maxT, ct);


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