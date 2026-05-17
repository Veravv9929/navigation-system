#include "core/routing.h"
#include <queue>
#include <unordered_map>
#include <chrono>
#include <algorithm>
#include <cmath>

namespace nav{

    DijkstraRouter::DijkstraRouter(const RoadNetwork& network) : network_(network) {}

    Path DijkstraRouter::findShortestPath(Node* start, Node* end) {
        auto timer_start = std::chrono::steady_clock::now();
        last_stats_ = Stats{};

        //边界检查
        if (!start || !end) return Path{};   //空指针检查
        if (start == end) return Path{{start}, 0, 0, true}; //起点=终点
        
        //优先队列:按照当前已知最短距离排序（小顶堆）
        //pair<距离， 节点指针>
        using QueueItem = std::pair<double, Node*>;
        std::priority_queue<QueueItem, std::vector<QueueItem>, std::greater<>> pq;

        //距离表：记录从起点到每个节点的当前最短距离
        std::unordered_map<NodeID, double> dist;
        //前驱表：用于重建路径
        std::unordered_map<NodeID, Node*> prev;

        //初始化
        dist[start->id] = 0;
        pq.push({0, start});

        while (!pq.empty()){
            auto [current_dist, current] = pq.top();
            pq.pop();
            last_stats_.nodes_explored++;

            //如果已经处理过更短的路径，跳过
            if (current_dist > dist[current->id]) continue;
            //是否到达终点
            if (current == end) break;

            //遍历所有邻居（出边）
            for (auto [neighbor, weight] : network_.getNeighbors(current)) {
                last_stats_.edges_relaxed++;
                double new_dist = current_dist + weight;

                //如果找到更短路径，更新
                if (!dist.count(neighbor->id) || new_dist < dist[neighbor->id]) {
                    dist[neighbor->id] = new_dist;
                    prev[neighbor->id] = current;
                    pq.push({new_dist, neighbor});
                }

            }
        }

        //重建路径
        Path result;
        if (!dist.count(end->id)) {
            return result;
        }

        //从终点回溯到起点
        Node* current = end;
        while (current) { //current为空指针时停止循环
            result.nodes.push_back(current); //从终点往起点收集
            auto it = prev.find(current->id);
            current = (it != prev.end()) ? it->second : nullptr;
        }
        std::reverse(result.nodes.begin(), result.nodes.end());

        result.found = true; //我找到路径了
        result.total_time = dist[end->id]; //总时间就是边的权重的和，的最后一个节点的节点指针为终点。
        //这里应该计算一下总距离，先用时间代替吧，得遍历边呢这东西,先不做了

        auto timer_end = std::chrono::steady_clock::now();
        last_stats_.compute_time_ms = std::chrono::duration<double, std::milli>(timer_end- timer_start).count();

        return result;
    }







    AStarRouter::AStarRouter(const RoadNetwork& network) : network_(network) {}

    //启发函数
    double AStarRouter::heuristic(Node* node, Node* end) const {
        //欧几里得距离（度）转米，除以最大可能速度（120km/h = 33.3m/s）
        //这样h(n)是最理想情况下需要的时间，不会高估实际代价
        double dx = node->coord.lat - end->coord.lat;
        double dy = node->coord.lon - end->coord.lon;
        double dist_meters = std::sqrt(dx*dx + dy*dy) * 111000; //粗略转换一下
        return dist_meters / 33.3; //秒
    }

    //A*主算法
    Path AStarRouter::findShortestPath(Node* start, Node* end) {
        auto timer_start = std::chrono::steady_clock::now();
        last_stats_ = DijkstraRouter::Stats{};

        if(!start || !end) return Path{};
        if(start == end) return Path{{start}, 0, 0, true};

        //优先队列排序依据：f(n) = g(n) + h(n)
        using QueueItem = std::tuple<double, double, Node*>; //<f, g, node>
        //比较器：按f值排序（小顶堆）
        auto cmp = [](const QueueItem& a, const QueueItem& b) {
            return std::get<0>(a) > std::get<0>(b);
        };
        std::priority_queue<QueueItem, std::vector<QueueItem>, decltype(cmp)> pq(cmp);

        //和Dijkstra算法一样的写法
        std::unordered_map<NodeID, double> g_score;
        std::unordered_map<NodeID, Node*> prev;

        g_score[start->id] = 0;
        double h = heuristic(start, end); //估算起点的h值
        pq.push({h, 0, start}); //f = 0 + h

        while(!pq.empty()) {
            auto [f, g, current] = pq.top();
            pq.pop();
            last_stats_.nodes_explored++;

            //Lazy deletion:如果已经找到更优路径，跳过
            if (g > g_score[current->id]) continue;

            if (current == end) break;

            for(auto [neighbor, weight] : network_.getNeighbors(current)) {
                last_stats_.edges_relaxed++;

                double tentative_g = g + weight;

                if (!g_score.count(neighbor->id) || tentative_g < g_score[neighbor->id]) {
                    g_score[neighbor->id] = tentative_g;
                    prev[neighbor->id] = current;
                    double h = heuristic(neighbor, end);
                    pq.push({tentative_g + h, tentative_g, neighbor});
                }
            }
        }
        
        //重建路径（于Dijkstra相同）
        Path result;
        if (!g_score.count(end->id)) return result;

        Node* current = end;
        while (current) {
            result.nodes.push_back(current);
            auto it = prev.find(current->id);
            current = (it != prev.end()) ? it->second : nullptr;
        }
        std::reverse(result.nodes.begin(), result.nodes.end());

        result.found = true;
        result.total_time = g_score[end->id];

        auto timer_end = std::chrono::steady_clock::now();
        last_stats_.compute_time_ms = std::chrono::duration<double, std::milli> (timer_end - timer_start).count();
        
        return result;

    }



    Path DijkstraRouter::findShortestPathFast(Node* start, Node* end) {
        auto timer_start = std::chrono::steady_clock::now();
        last_stats_ = Stats{};

        if (!start || !end) return Path{};
        if (start == end) return Path {{start}, 0, 0, true};
        
        const size_t n = network_.indexSize();

        //预分配vector,O(1)访问且缓存友好
        std::vector<double> dist(n, std::numeric_limits<double>::infinity());
        std::vector<Node*> prev(n, nullptr);
        std::vector<bool> visited(n, false);

        //哈希表(id_to_index_)中查起始点和目的地的数组座位号
        size_t start_idx = network_.nodeIndex(start->id);
        size_t end_idx = network_.nodeIndex(end->id);

        //数组中起始点的座位号对应的值为0，因为起始点到自己的距离为0
        dist[start_idx] = 0;

        using QueueItem = std::pair<double, size_t>; //<距离，索引>
        std::priority_queue<QueueItem, std::vector<QueueItem>, std::greater<>> pq;
        pq.push({0, start_idx});

        while (!pq.empty()) {
            auto [d, idx] = pq.top();
            pq.pop();
            last_stats_.nodes_explored++;

            if (visited[idx]) continue;
            visited[idx] = true;

            if (idx == end_idx ) break;

            //第二遍看总览代码理解了，其实优化就优化在for循环里面每个节点遍历所有邻居节点的时候不用hash表而是用vector直接定点查找
            //这就快很多了，循环外多遍历一次的开销也不算什么了
            //prev, dist都是vector
            Node* current = network_.nodeByIndex(idx);
            for (auto [neighbor, weight] : network_.getNeighbors(current)) {
                last_stats_.edges_relaxed++;

                size_t nidx = network_.nodeIndex(neighbor->id);
                double new_dist = d + weight;

                if (new_dist < dist[nidx]) {
                    dist[nidx] = new_dist;
                    prev[nidx] = current;
                    pq.push({new_dist, nidx});
                }
            }

        }

        //重建路径（用索引版prev）
        Path result;
        if (dist[end_idx] == std::numeric_limits<double>::infinity()) {
            return result; //不可达
        }

        size_t idx = end_idx;
        while (idx != std::numeric_limits<size_t>::max()) {
            result.nodes.push_back(network_.nodeByIndex(idx));
            if (idx == start_idx) break;

            Node* prev_node = prev[idx];
            if (!prev_node) break;
            idx = network_.nodeIndex(prev_node->id);
        }
        std::reverse(result.nodes.begin(), result.nodes.end());

        result.found = true;
        result.total_time = dist[end_idx];

        auto timer_end = std::chrono::steady_clock::now();
        last_stats_.compute_time_ms = std::chrono::duration<double, std::milli> (timer_end - timer_start).count();
        return result;
        
    }

}
