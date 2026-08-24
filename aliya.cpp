#include <bits/stdc++.h>
using namespace std;
using ll = long long; using ull = unsigned long long;
const int MAXN = 2e5 + 10, MAXE = MAXN << 1;
const int INF32 = INT_MAX; const ll INF64 = LLONG_MAX;

int n, m; ll t, ans = INF64;
ll dis[2][MAXN]; int nxtEdge[MAXN];// 最优路径的下一条边

vector<bitset<MAXE>> vis;// vis[w][e]: 状态 (w, e) 是否生成
vector<vector<ull>> inCnt;// inCnt[w][u]: 记录节点 u 在权值 w 层已经接受的不同入边个数，最多为 2
// inCnt: 每个节点用 2 位记录。一个 ull 可存放 32 个节点
void addCnt(int w, int u) {
	int idx = u >> 5, shift = (u & 31) << 1; ull val = (inCnt[w][idx] >> shift) & 3;
	if (val < 2) {
		val++;
		inCnt[w][idx] = (inCnt[w][idx] & ~(3ULL << shift)) | (val << shift);
	}
}
int qryCnt(int w, int u) {
	int idx = u >> 5, shift = (u & 31) << 1;
	return (inCnt[w][idx] >> shift) & 3;
}

struct Edge {
	int v, w, nxt;
} edge[MAXE];
int head[MAXN], totEdge;
void addEdge(int u, int v, int w) {
	edge[totEdge] = Edge{v, w, head[u]};
	head[u] = totEdge++;
}

void init() {
	head[0] = -1;
	for (int i = 1; i <= n; i++) {
		head[i] = -1;
		dis[0][i] = dis[1][i] = INF64;
		nxtEdge[i] = -1;
	}
}

// 反向 dijkstra
void dijkstra() {
	using T = tuple<ll, int, int>;// (d, u, e): 当前在 u 已经走了 d，且从 u 出发到 1 第一条边为 e
	priority_queue<T, vector<T>, greater<T>> q;
	
	for (int i = head[1]; ~i; i = edge[i].nxt) {
		int v = edge[i].v; ll w = edge[i].w;
		q.emplace(w, v, i ^ 1);
	}
	while (q.size()) {
		ll d; int u, e;
		tie(d, u, e) = q.top(); q.pop();
		
		if (dis[0][u] == INF64) dis[0][u] = d, nxtEdge[u] = e;
		else if (dis[1][u] == INF64 && e != nxtEdge[u]) dis[1][u] = d;
		else continue;
		
		for (int i = head[u]; ~i; i = edge[i].nxt) {
			int v = edge[i].v; ll w = edge[i].w;
			if (v == 1 || i == e) continue;
			q.emplace(d + w, v, i ^ 1);
		}
	}
}
int main() {
	// freopen(".in", "r", stdin);
	// freopen(".out", "w", stdout);
	ios::sync_with_stdio(0); cin.tie(0); cout.tie(0);
	
	cin >> n >> m >> t;
	
	if (n == 1) {
		cout << t;
		return 0;
	}
	
	init();
	
	for (int i = 0; i < m; i++) {
		int u, v, w;
		cin >> u >> v >> w;
		addEdge(u, v, w);
		addEdge(v, u, w);
	}
	
	dijkstra();
	
	// 添加虚拟起点边
	addEdge(0, n, 0);
	int vE = totEdge - 1;
	
	// 动态分配内存
	int totInCnt = (n >> 5) + 1;
	vis.assign(t + 1, bitset<MAXE>());
	inCnt.assign(t + 1, vector<ull>(totInCnt, 0));
	
	// 将初始状态 (0, p) 入队
	vis[0][vE] = 1;
	addCnt(0, n);
	
	// 分层 bfs
	// (w, eIn): 当前总边权和为 w，最后一步是通过边 eIn 进入当前节点
	vector<int> q;
	q.reserve((n << 1) + 10);
	for (int w = 0, flag = 1; w <= t && flag; w++) {
		q.clear();
		for (int pos = vis[w]._Find_first(); pos != MAXE; pos = vis[w]._Find_next(pos)) q.emplace_back(pos);
		
		for (int hd = 0; hd < (int)q.size() && flag; hd++) {
			int eIn = q[hd];
			int u = edge[eIn].v;
			
			if (u == 1) continue;
			
			for (int eOut = head[u]; ~eOut; eOut = edge[eOut].nxt) {
				if (eOut == (eIn ^ 1)) continue;
				
				int v = edge[eOut].v; ll nxtW = w + edge[eOut].w;
				if (v == 1) {
					ll now = llabs(nxtW - t);
					ans = min(ans, now);
				}
				else {
					if (dis[0][v] != INF64 && nxtEdge[v] != (eOut ^ 1)) {
						ll d = nxtW + dis[0][v], now = llabs(d - t);
						ans = min(ans, now);
					}
					else if (dis[1][v] != INF64) {
						ll d = nxtW + dis[1][v], now = llabs(d - t);
						ans = min(ans, now);
					}
				}
				
				if (!ans) {
					flag = 0;
					break;
				}
				
				if (nxtW <= t && qryCnt(nxtW, v) < 2 && !vis[nxtW][eOut]) {
					vis[nxtW][eOut] = 1;
					addCnt(nxtW, v);
					if (nxtW == w) q.emplace_back(eOut);
				}
			}
		}
	}
	cout << ans << endl;
	return 0;
}