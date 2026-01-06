// 包含所有标准C++库的头文件（简化写法，实际开发中不建议使用）
// bits/stdc++.h 是一个GCC扩展，包含了几乎所有标准库头文件
#include <bits/stdc++.h>
// 使用标准命名空间，这样就不用写 std::vector，可以直接写 vector
using namespace std;

/* ========== 1) 基础结构：Pos / Grid ========== */
// 定义位置结构体，表示二维平面上的一个点
struct Pos {
    int x, y;  // x坐标（列），y坐标（行）

    // C++运算符重载：重载 == 运算符，用于比较两个Pos是否相等
    // const Pos& o 表示传入一个常量引用，const 表示这个函数不会修改对象状态
    bool operator==(const Pos& o) const { return x==o.x && y==o.y; }
};

// 类型别名：Path 是 Pos 的向量（动态数组），表示一条路径
// vector<Pos> 是C++的动态数组模板，类似C语言的数组但可以自动扩容
using Path = vector<Pos>;

// 定义网格结构体，表示地图
struct Grid {
    int W, H;           // 网格宽度和高度
    vector<string> g;   // 网格内容，每行是一个字符串，'.'表示可通过，'#'表示障碍

    // 检查坐标是否在网格范围内
    // const 关键字表示这个函数不会修改Grid对象
    bool inBounds(int x,int y) const { return 0<=x && x<W && 0<=y && y<H; }

    // 检查坐标是否可通过（在范围内且不是障碍）
    bool passable(int x,int y) const { return inBounds(x,y) && g[y][x] != '#'; }
};

/* ========== 2) 约束结构（CBS 高层往低层传的东西） ========== */
// 枚举类：定义约束类型，C++11引入的强类型枚举，比C语言的enum更安全
// Vertex: 顶点约束（禁止在某个时间点出现在某个位置）
// Edge: 边约束（禁止在某个时间点从A位置移动到B位置）
enum class ConstraintType { Vertex, Edge };

// 约束结构体：CBS高层算法传递给低层路径规划器的约束信息
struct Constraint {
    int agent;           // 约束针对哪个智能体（编号）
    ConstraintType type; // 约束类型：顶点约束还是边约束
    int t;               // 时间步（约束生效的时间）
    int x1,y1;           // 位置1（对于顶点约束就是禁止的位置，对于边约束是起点）
    int x2,y2;           // 位置2（仅边约束使用，表示终点）
};

// 打包函数：将坐标和时间打包成一个64位整数，用于unordered_set快速查询
// static inline: static表示只在当前文件可见，inline建议编译器内联展开
// 顶点约束的键生成：将t、x、y分别移位后异或，确保唯一性
static inline long long keyVertex(int x,int y,int t){
    // 位运算：t左移40位，x左移20位，y不左移，然后异或合并
    return ((long long)t<<40) ^ ((long long)x<<20) ^ (long long)y;
}

// 边约束的键生成：将t、x1、y1、x2、y2分别移位后异或
static inline long long keyEdge(int x1,int y1,int x2,int y2,int t){
    long long k=(long long)t;  // 从时间t开始
    k=(k<<12)^x1;  // 左移12位后与x1异或
    k=(k<<12)^y1;  // 再左移12位后与y1异或
    k=(k<<12)^x2;  // 再左移12位后与x2异或
    k=(k<<12)^y2;  // 再左移12位后与y2异或
    return k;
}

// 约束表结构体：存储一个智能体的所有约束（从CTNode.constraints中筛选出来）
struct ConstraintTable {
    unordered_set<long long> forbV;  // 禁止的顶点集合（存储打包后的键）
    unordered_set<long long> forbE;  // 禁止的边集合（存储打包后的键）
    // unordered_set是C++的哈希集合，查找时间复杂度O(1)
};

// 检查是否违反顶点约束
static inline bool violatesVertex(const ConstraintTable& ct, int x,int y,int t){
    // count()函数返回集合中该键的数量（0或1），>0表示存在该约束
    return ct.forbV.count(keyVertex(x,y,t))>0;
}

// 检查是否违反边约束
static inline bool violatesEdge(const ConstraintTable& ct, int x1,int y1,int x2,int y2,int t){
    return ct.forbE.count(keyEdge(x1,y1,x2,y2,t))>0;
}

/* ========== 3) 低层：带约束 Space-Time A* ========== */
// 状态结构体：表示时空中的一个状态（位置+时间）
struct State {
    int x,y,t;  // 位置(x,y)和时间t

    // 重载 == 运算符，用于比较两个状态是否相等
    bool operator==(const State& o) const { return x==o.x && y==o.y && t==o.t; }
};

// 自定义哈希函数结构体：用于unordered_map/unordered_set中的State类型
// 这是C++标准库要求的，如果要使用自定义类型作为哈希表的键，必须提供哈希函数
struct StateHash {
    // 重载 () 运算符，使得这个结构体可以像函数一样调用
    // noexcept 表示这个函数不会抛出异常
    size_t operator()(State const& s) const noexcept {
        // 将t、x、y分别移位后异或，生成哈希值
        return ((size_t)s.t<<32) ^ ((size_t)s.x<<16) ^ (size_t)s.y;
    }
};

// 计算曼哈顿距离（启发式函数）
static inline int manhattan(const Pos& a, const Pos& b){
    return abs(a.x-b.x) + abs(a.y-b.y);
}

// 获取当前位置的所有邻居位置（可移动的方向）
static vector<Pos> getNeighbors(const Grid& grid, const Pos& cur){
    // 5个方向：右(1,0)、左(-1,0)、下(0,1)、上(0,-1)、等待(0,0)
    const int dx[5]={1,-1,0,0,0};
    const int dy[5]={0,0,1,-1,0};

    vector<Pos> res;  // 存储所有可行的邻居位置

    for(int k=0;k<5;k++){
        int nx=cur.x+dx[k], ny=cur.y+dy[k];
        if(grid.passable(nx,ny)) {
            // 如果位置可通过，添加到结果中
            // Pos{nx,ny} 是C++11的统一初始化语法
            res.push_back(Pos{nx,ny});
        }
    }
    return res;
}

// 从全局约束列表中构建某个特定智能体的约束表
static ConstraintTable buildCT(const vector<Constraint>& cons, int agent){
    ConstraintTable ct;  // 创建空的约束表

    // 遍历所有约束，C++11的范围for循环，类似Python的for循环
    // const auto& c: cons 表示遍历cons中的每个元素，c是常量引用
    for(const auto& c: cons){
        if(c.agent!=agent) continue;  // 如果不是当前智能体的约束，跳过

        if(c.type==ConstraintType::Vertex){
            // 顶点约束：添加到禁止顶点集合
            ct.forbV.insert(keyVertex(c.x1,c.y1,c.t));
        }else{
            // 边约束：添加到禁止边集合
            ct.forbE.insert(keyEdge(c.x1,c.y1,c.x2,c.y2,c.t));
        }
    }
    return ct;  // 返回构建好的约束表
}

//到达goal后，从t到H期间都要能合法停留（点约束/边约束都要检查）
static bool goalSafeToH(const ConstraintTable& ct,const Pos& goal,int t,int H){
    for(int tau = t; tau <= H; tau++){
        if (violatesVertex(ct, goal.x, goal.y, tau)) return false;
        // tau -> tau+1 的边约束检查
        if (tau < H && violatesEdge(ct, goal.x, goal.y, goal.x, goal.y, tau)) return false;
    }
    return true;
}

static int maxConstraintTimeForAgent(const vector<Constraint>& cons,int agent){
    int mx = 0;
    for (const auto& c : cons){
        if (c.agent == agent) mx = max(mx,c.t);
    }
    return mx;
}

static int maxConstraintTimeAll(const vector<Constraint>& cons){
    int mx = 0;
    for (const auto& c : cons) mx = max(mx, c.t);
    return mx;
}


static int lowerBoundLen(const vector<Pos>& start, const vector<Pos>& goal){
    int lb = 0;
    for(int i = 0; i < (int)start.size(); i++){
        lb = max(lb,abs(start[i].x - goal[i].x)+abs(start[i].y - goal[i].y));
    }
    return lb;
}

// 带约束的时空A*算法：在考虑约束的情况下寻找从起点到目标的最短路径
// 参数：网格地图、起点、目标点、最大时间步、约束表
// 返回：找到的路径（Pos的向量），如果找不到返回空路径
static Path spaceTimeAStar(const Grid& grid, Pos start, Pos goal, int maxT, const ConstraintTable& ct){
    // 定义搜索节点结构体：包含状态、实际代价g、估计总代价f
    struct Node { State s; int g,f; };

    // 自定义优先队列比较函数：按照f值从小到大排序（最小堆）
    // 如果f值相同，按照g值从大到小排序（优先扩展代价小的）
    struct Cmp {
        bool operator()(const Node& a, const Node& b) const {
            if(a.f!=b.f) return a.f>b.f;  // f值小的优先级高
            return a.g<b.g;  // f相同时，g值大的优先级高（实际代价小的）
        }
    };

    // 优先队列（开放列表），按照Cmp定义的顺序排序
    // priority_queue是C++的优先队列容器，默认是最大堆，这里通过Cmp改为最小堆
    priority_queue<Node, vector<Node>, Cmp> open;

    // 记录到达每个状态的最佳g值（实际代价）
    // unordered_map是C++的哈希表，键是State，值是int（g值）
    unordered_map<State,int,StateHash> bestG;

    // 记录每个状态的父状态，用于回溯路径
    unordered_map<State,State,StateHash> parent;

    // 检查起点是否违反约束（时间t=0时不能在禁止的位置）
    if(violatesVertex(ct, start.x,start.y,0)) return {};

    // 初始化起点状态
    State s0{start.x,start.y,0};
    bestG[s0]=0;  // 起点到起点的代价为0
    // 将起点加入开放列表，f = g + h = 0 + 曼哈顿距离
    open.push(Node{s0,0,manhattan(start,goal)});

    // A*主循环：当开放列表不为空时继续搜索
    while(!open.empty()){
        // 从开放列表中取出f值最小的节点
        Node cur=open.top(); open.pop();
        State cs=cur.s;  // 当前状态

        // 跳过无效节点：如果当前g值不是最佳g值，说明这个节点已经过时
        if(cur.g!=bestG[cs]) continue;

    
        if(cs.x==goal.x && cs.y==goal.y){
            //只有当 goal 从当前时刻到 maxT 都能合法等待，才算真正找到可行解
            if(goalSafeToH(ct,goal,cs.t,maxT))
            {
                // 回溯构建路径
                vector<Pos> rev;  // 反向路径
                State p=cs;  // 从目标状态开始回溯

                 while(true){
                    // 将当前位置加入路径
                    rev.push_back(Pos{p.x,p.y});
                    if(p.t==0) break;  // 回溯到时间0（起点）时停止
                    p=parent[p];  // 获取父状态
                }

                // 反转路径（从起点到目标）
                reverse(rev.begin(),rev.end());
                //返回前补齐到maxT+1长度，保证后续处理一致性
                while ((int)rev.size() < maxT + 1) rev.push_back(rev.back());
                return rev;
            }  
             // 否则不能直接返回，继续搜索其他到达方式/到达时刻
        }

        // 如果达到最大时间限制，不再扩展
        if(cs.t==maxT) continue;

        // 扩展当前节点的所有邻居
        for(const auto& np: getNeighbors(grid, Pos{cs.x,cs.y})){
            int nt=cs.t+1;  // 下一时刻

            // 检查顶点约束：下一时刻不能在禁止的位置
            if(violatesVertex(ct, np.x,np.y,nt)) continue;

            // 检查边约束：这一步移动（从cs到np）不能在禁止的边上
            if(violatesEdge(ct, cs.x,cs.y,np.x,np.y,cs.t)) continue;

            // 新状态
            State ns{np.x,np.y,nt};
            int ng=cur.g+1;  // 新代价 = 当前代价 + 1（每步移动代价为1）

            // 查找是否已经到达过这个状态
            auto it=bestG.find(ns);

            // 如果这是第一次到达该状态，或者找到了更优的路径
            if(it==bestG.end() || ng<it->second){
                bestG[ns]=ng;  // 更新最佳g值
                parent[ns]=cs;  // 记录父状态

                // 计算f值 = g + 启发式值（到目标的曼哈顿距离）
                int nf=ng+manhattan(Pos{ns.x,ns.y},goal);

                // 将新节点加入开放列表
                open.push(Node{ns,ng,nf});
            }
        }
    }

    // 开放列表为空仍未找到路径，返回空路径
    return {};
}

//把所有路径补齐到同一长度：短的用最后一个位置重复填充（表示到达后等待）
static void padPathsToSameLength(vector<Path>& paths){
    int T = 0;
    for (auto &p : paths) T = max(T,(int)p.size());
    for (auto &p : paths){
        while ((int)p.size() < T) p.push_back(p.back()); 
    }
}

/* ========== 4) 冲突检测（CBS 高层） ========== */
// 获取路径在时间t的位置（带边界检查）
// 如果t小于0，返回路径的第一个位置（起点）
// 如果t在路径长度范围内，返回路径的第t个位置
// 如果t超出路径长度，返回路径的最后一个位置（智能体到达目标后等待）
static Pos posAt(const Path& p, int t){
    if(t<0) return p.front();          // t<0：返回起点
    if(t<(int)p.size()) return p[t];   // t在路径范围内：返回对应位置
    return p.back();                   // t超出路径：返回最后位置（等待状态）
}

// 冲突结构体：描述两个智能体之间的冲突
struct Conflict{
    bool exists=false;  // 是否存在冲突
    bool isEdge=false;  // 冲突类型：false=顶点冲突，true=边冲突
    int a=-1,b=-1;      // 冲突的两个智能体编号
    int t=-1;           // 冲突发生的时间

    // 顶点冲突的字段
    int x=-1,y=-1;      // 冲突发生的位置

    // 边冲突的字段（以智能体a的移动为准）
    int ax1,ay1,ax2,ay2;  // 智能体a的移动：从(ax1,ay1)到(ax2,ay2)
};

// 检测路径集合中的第一个冲突
// 按照时间顺序检查，找到第一个发生的冲突就返回
static Conflict detectFirstConflict(const vector<Path>& paths){
    int n=(int)paths.size();  // 智能体数量
    int T=0;  // 最大路径长度（最大时间步）

    // 计算所有路径的最大长度
    for(const auto& p: paths) T=max(T,(int)p.size());

    // 按时间顺序检查每个时刻
    for(int t=0;t<T;t++){
        // 检查每对智能体
        for(int i=0;i<n;i++){
            Pos pi=posAt(paths[i],t);       // 智能体i在时间t的位置
            Pos pi_prev=posAt(paths[i],t-1); // 智能体i在时间t-1的位置

            for(int j=i+1;j<n;j++){
                Pos pj=posAt(paths[j],t);   // 智能体j在时间t的位置

                // 1. 检查顶点冲突：两个智能体在同一时间占据同一位置
                if(pi==pj){
                    Conflict c;
                    c.exists=true;   // 存在冲突
                    c.isEdge=false;  // 顶点冲突
                    c.a=i; c.b=j;    // 冲突的智能体
                    c.t=t;           // 冲突时间
                    c.x=pi.x; c.y=pi.y;  // 冲突位置
                    return c;        // 返回第一个找到的冲突
                }

                // 2. 检查边冲突：两个智能体交换位置（在相邻时间步）
                // 边冲突发生在时间t-1到t的移动过程中
                if(t>0){  // 需要至少一个时间步的移动
                    Pos pj_prev=posAt(paths[j],t-1);  // 智能体j在时间t-1的位置

                    // 检查是否交换位置：i从pi_prev移动到pi，j从pj_prev移动到pj
                    // 且 pi_prev == pj 且 pj_prev == pi
                    if(pi_prev==pj && pj_prev==pi){
                        Conflict c;
                        c.exists=true;   // 存在冲突
                        c.isEdge=true;   // 边冲突
                        c.a=i; c.b=j;    // 冲突的智能体（以i的移动方向为准）
                        c.t=t-1;         // 冲突时间（移动开始的时间）
                        c.ax1=pi_prev.x; c.ay1=pi_prev.y;  // i的起点
                        c.ax2=pi.x;      c.ay2=pi.y;       // i的终点
                        return c;
                    }
                }
            }
        }
    }

    // 没有找到冲突，返回默认的Conflict对象（exists=false）
    return Conflict{};
}

// 计算总代价：所有智能体路径长度的总和
// 路径长度 = 位置数量 - 1（因为起点不算一步移动）
static int sumOfCosts(const vector<Path>& paths){
    int s=0;
    for(const auto& p: paths) {
        // max(0, ...) 防止空路径产生负值
        s += max(0,(int)p.size()-1);
    }
    return s;
}

static int makespan(const vector<Path>& paths){
    int ms=0;
    for(const auto& p: paths) {
        ms = max(ms, (int)p.size()-1);
    }
    return ms;
}

/* ========== 5) CBS 高层：CTNode + open list ========== */
// 约束树节点（CT = Constraint Tree）
// CBS算法构建一棵约束树，每个节点包含一组约束和对应的路径集合
struct CTNode{
    vector<Constraint> constraints;  // 该节点的约束集合
    vector<Path> paths;              // 在当前约束下各智能体的路径
    int cost=0;                      // 总代价（路径长度总和）
    int id=0;                        // 节点ID（用于打破平局）
};

// 约束树节点的比较函数：用于优先队列
// 按照代价从小到大排序（最小代价优先），代价相同时按ID排序
struct CTNodeCmp{
    bool operator()(const CTNode& a, const CTNode& b) const {
        if(a.cost!=b.cost) return a.cost>b.cost;  // 代价小的优先级高
        return a.id>b.id;  // 代价相同时，ID小的优先级高
    }
};

// CBS主算法：冲突基搜索
// 参数：网格地图、所有智能体的起点、所有智能体的目标点
// 输出：solution - 存储找到的无冲突路径
// 返回值：true表示找到解，false表示无解
static bool CBS(const Grid& grid,
                const vector<Pos>& starts,
                const vector<Pos>& goals,
                vector<Path>& solution){

    int n=(int)starts.size();  // 智能体数量
    int nodeId=0;              // 节点ID计数器

    // C++11 Lambda表达式：定义重规划函数
    // [&] 表示捕获所有外部变量 by reference（引用捕获）
    // 参数：CTNode节点和智能体编号
    // 返回值：重规划是否成功
    auto replanAgent = [&](CTNode& node, int agent)->bool{
        // 为该智能体构建约束表
        ConstraintTable ct = buildCT(node.constraints, agent);

    int lb = lowerBoundLen(starts, goals);                 // 最短路下界
    int curMS = 0;
    if (!node.paths.empty()) curMS = makespan(node.paths); // 当前节点已有解的 makespan（root 初始化时可能还是空/未填满）

    int mxA = maxConstraintTimeForAgent(node.constraints, agent); // 该 agent 最大约束时间
    int mxAll = maxConstraintTimeAll(node.constraints);           // 整个节点最大约束时间（可选但很稳）

    int buffer = 10;

    // 解释：
    // - 至少要覆盖到当前解的 makespan（不然会搜太短）
    // - 至少要覆盖到约束时间（否则 goal-safe 检查没意义）
    // - 至少要覆盖到最短路下界（否则本来就到不了）
    int maxT = max({lb, curMS, mxA, mxAll}) + buffer;

    for (int attempt = 0; attempt < 3; attempt++) {   // 最多扩 3 次
        cout << "[replan] agent=" << agent << " maxT=" << maxT << "\n";
        Path p = spaceTimeAStar(grid, starts[agent], goals[agent], maxT, ct);
        if (!p.empty()) {
            node.paths[agent] = std::move(p);
            return true;
        }
        maxT += 10; // 不够就再加 10
    }

    return false;

    };
    
    // 创建根节点：没有任何约束
    CTNode root;
    root.id = nodeId++;        // 分配节点ID
    root.paths.resize(n);      // 为n个智能体预留路径空间

    root.constraints.push_back(Constraint{1, ConstraintType::Vertex, 5, 2,4, 0,0});//约束的意思是在时间5时刻，智能体1不能出现在位置(2,4)

    // 为每个智能体规划初始路径（无约束）
    for(int i=0;i<n;i++){
        if(!replanAgent(root,i)) return false;  // 如果任何一个智能体规划失败，整个问题无解
    }
    padPathsToSameLength(root.paths);
    root.cost = sumOfCosts(root.paths);  // 计算根节点的总代价

    // 创建优先队列（开放列表），存储待扩展的约束树节点
    priority_queue<CTNode, vector<CTNode>, CTNodeCmp> open;
    open.push(root);  // 将根节点加入开放列表

    // CBS主循环：扩展约束树节点直到找到解或证明无解
    while(!open.empty()){
        // 取出代价最小的节点
        CTNode cur = open.top(); open.pop();

        // 检测当前路径集合中的冲突
        Conflict conf = detectFirstConflict(cur.paths);

        // 如果没有冲突，找到解！
        if(!conf.exists){
            solution = cur.paths;  // 将当前路径集合作为解
            return true;
        }

        // 有冲突：分裂为两个子节点，分别对冲突的两个智能体添加约束
        for(int k=0;k<2;k++){
            int agent = (k==0 ? conf.a : conf.b);  // 选择要约束的智能体

            // 创建子节点（复制当前节点的状态）
            CTNode child = cur;
            child.id = nodeId++;  // 分配新ID

            // 根据冲突类型添加约束
            if(!conf.isEdge){
                // 顶点冲突：禁止智能体在时间t占据位置(x,y)
                child.constraints.push_back(Constraint{
                    agent, ConstraintType::Vertex, conf.t,
                    conf.x, conf.y, 0, 0  // 后两个参数对顶点约束无用
                });
            }else{
                // 边冲突：禁止智能体走那条边
                if(agent==conf.a){
                    // 约束智能体a：禁止从(ax1,ay1)移动到(ax2,ay2)
                    child.constraints.push_back(Constraint{
                        agent, ConstraintType::Edge, conf.t,
                        conf.ax1, conf.ay1, conf.ax2, conf.ay2
                    });
                }else{
                    // 约束智能体b：禁止从(ax2,ay2)移动到(ax1,ay1)
                    // 注意方向相反，因为边冲突是双向的
                    child.constraints.push_back(Constraint{
                        agent, ConstraintType::Edge, conf.t,
                        conf.ax2, conf.ay2, conf.ax1, conf.ay1
                    });
                }
            }

            // 只重规划被约束的智能体（其他智能体路径不变）
            if(!replanAgent(child, agent)) continue;  // 如果重规划失败，跳过这个子节点

            padPathsToSameLength(child.paths);

            // 计算子节点的总代价
            child.cost = sumOfCosts(child.paths);

            // 将子节点加入开放列表
            // std::move将child移动到队列中，避免拷贝
            open.push(std::move(child));
        }
    }

    // 开放列表为空仍未找到解，返回失败
    return false;
}

/* ========== 6) main：测试 CBS ========== */
int main(){
    volatile int dbg = 0; // 调试用的变量，防止编译器优化掉某些代码
    // 创建网格地图
    Grid grid;
    // 初始化网格内容：5行10列的网格
    // '.' 表示可通过的空地，'#' 表示障碍物
    grid.g = {
        "..........",  // 第0行：全部可通过
        ".####.....",  // 第1行：位置(1,1)到(4,1)是障碍
        "..........",  // 第2行：全部可通过
        ".....####.",  // 第3行：位置(5,3)到(8,3)是障碍
        ".........."   // 第4行：全部可通过
    };

    // 计算网格的高度和宽度
    grid.H=(int)grid.g.size();      // 高度 = 行数 = 5
    grid.W=(int)grid.g[0].size();   // 宽度 = 第一行的长度 = 10

    // 设置两个智能体的起点和目标点（对穿场景）
    // 智能体0：从左上角(0,0)到右下角(9,4)
    // 智能体1：从左下角(0,4)到右上角(9,0)
    vector<Pos> starts = { {0,0}, {0,4} };
    vector<Pos> goals  = { {9,4}, {2,4} };

    // 存储解决方案的路径
    vector<Path> sol;

    // 调用CBS算法求解
    bool ok = CBS(grid, starts, goals, sol);

    // 检查是否找到解
    if(!ok){
        cout << "No solution.\n";
        return 0;
    }

    // 输出找到的解决方案
    cout << "CBS solution found!\n";
    for(int i=0;i<(int)sol.size();i++){
        cout << "Agent " << i << ":\n";
        //输出该智能体在每个时间步的位置
        for(int t=0;t<(int)sol[i].size();t++){
            cout << "  t="<<t<<" ("<<sol[i][t].x<<","<<sol[i][t].y<<")\n";
        }
        // int showT = 15;
        // for (int t = 0; t < min((int)sol[i].size(), showT); t++) {
        //     cout << "  t=" << t << " (" << sol[i][t].x << "," << sol[i][t].y << ")\n";
        // }
    }

    return 0;
}
