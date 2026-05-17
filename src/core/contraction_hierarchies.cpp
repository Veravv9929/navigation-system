#include "core/contraction_hierarchies.h"
#include <iostream>
#include <chrono>
#include <queue>
#include <limits>
#include <algorithm>

namespace nav {

    CHRouter::CHRouter(const RoadNetwork& network) : network_(network) {}

    //计算节点重要性边差
    double CHRouter::computeEdgeDifference(Node* node) {
        //边差 = 需要添加的捷径边数 - 被移除的边数
        //边差越小，节点越不重要，越早收缩

        //计算被移除的边数
        int removed_edges = node->outgoing.size() + node->incoming.size();
        //这个节点收缩时需要创建的捷径数量
        int added_shortcuts = 0;

        // 检查每对邻居（u->node->v）是否需要捷径 u->v
        // 挺有意思，u是node前面的节点，v是node后面的节点哈,这么个邻居对法
        for (Edge* in : node->incoming) {
            Node* u = in->from;
            for (Edge* out : node->outgoing) {
                Node* v = out->to;
                if (u == v) continue; // 自环跳过
                
                double via_dist = in->weight() + out->weight();

                //witness search: 找是否存在不经过node的更短路径
                double direct = witnessSearch(u, v, via_dist, 5); //限制5跳
                if (direct > via_dist) {
                    added_shortcuts++; //需要添加捷径
                }

            }
        }

        return added_shortcuts - removed_edges;
    }
    

    //查找是否存在不经过node的更短路径 ,返回的是不经过node的最短路径的长度
    double CHRouter::witnessSearch(Node* from, Node* to, double max_dist, int max_hops) {
        //限制范围的Dijkstra,找from->to 不经过被收缩节点的最短距离
        //如果找到的距离 < max_dist, 说明不需要捷径

        std::unordered_map<NodeID, double> dist;
        using QueueItem = std::pair<double, Node*>;
        std::priority_queue<QueueItem, std::vector<QueueItem>, std::greater<>> pq;

        //所以这个操作是从from开始的哦要注意
        dist[from->id] = 0;
        pq.push({0, from});

        int hops = 0;
        while (!pq.empty() && hops < max_hops) {
            auto [d, current] = pq.top();
            pq.pop();

            if (current == to) return d; //找到更短路径
            if (d > max_dist) continue; //超过限制，跳过

            for (auto [neighbor, weight] : network_.getNeighbors(current)) {
                //跳过正在被收缩的节点（通过node_level_判断）
                if (node_level_.count(neighbor->id)) continue;

                double new_dist = d + weight;
                //如果没到过这个neighbor节点或者以前的到这的边消耗要比new_dist大的
                if (!dist.count(neighbor->id) || new_dist < dist[neighbor->id]) {
                    dist[neighbor->id] = new_dist;
                    pq.push({new_dist, neighbor});
                }
            }
            hops++;
        }
        return std::numeric_limits<double>::infinity(); //这说明没法达到to点,返回的是无穷大。
    }



    //进行收缩操作
    void CHRouter::contractNode(Node* node) {
        //当前收缩等级 = 已收缩节点数
        int level = static_cast<int>(node_level_.size());

        //2.标记节点等级
        node_level_[node->id] = level;


        //1.为需要捷径的邻居对添加shortcut
        for (Edge* in : node->incoming) {
            Node* u = in->from;
            for (Edge* out : node->outgoing) {
                Node* v = out->to;
                if (u == v) continue;

                double via_dist = in->weight() + out->weight();
                double direct = witnessSearch(u, v, via_dist, 5);
                if (direct > via_dist) {
                    //添加到正向图添加捷径边u->v, 权重为via_dist
                    CHEdge shortcut{v, via_dist, true};
                    ch_up_outgoing[u->id].push_back(shortcut);
                    //添加到反向图：v的向上入边（用于从终点搜索）
                    CHEdge rev_shortcut{u, via_dist, true};
                    ch_up_incoming[v->id].push_back(rev_shortcut);
                }
            }
        }

        //2.标记节点等级
        //node_level_[node->id] = level;

        //将原边标记为”向上边“
        for (Edge* e : node->outgoing) {
            NodeID nbr_id = e->to->id;
            //邻居还没收缩（没有等级）,或者等级更高
            //这是”向上边“
            if (!node_level_.count(nbr_id)) {
                //这个节点的向上边的集合
                ch_up_outgoing[node->id].push_back({e->to, e->weight(), false});
            }
            //如果邻居已经收缩了（有等级且<level）,不保留，因为查询时候不会往下走
        }

        //入边同理（反向）
        for (Edge* e : node->incoming) {
            NodeID nbr_id = e->from->id;
            if (!node_level_.count(nbr_id)) {
                //邻居未收缩，从邻居（等级高）指向node(等级低)
                ch_up_incoming[node->id].push_back({e->from, e->weight(), false});
            }
        }
    }



    //实现预处理preprocess()主流程
    void CHRouter::preprocess() {
        auto timer_start = std::chrono::steady_clock::now();

        //1.收集所有节点
        std::vector<Node*> nodes;
        for (const auto& [id, ptr] : network_.nodes()) {
            nodes.push_back(ptr.get());
        }

        //2.计算初始边差
        std::vector<std::pair<double, Node*>> priorities;
        for (Node* n : nodes) {
            priorities.push_back({computeEdgeDifference(n), n});
        }

        //3.按边差排序（小的先收缩）
        std::sort(priorities.begin(), priorities.end(), [](const auto& a, const auto& b) { return a.first < b.first; }); 

        //4.逐个收缩
        for (auto& [diff, node] : priorities) {
            if (node_level_.count(node->id)) continue; //已经收缩
            contractNode(node);
        }

        //5.为剩余未收缩节点添加向上边
        for (Node* n : nodes) {
            if (!node_level_.count(n->id)) {
                node_level_[n->id] = static_cast<int>(node_level_.size());
            }
        }

        auto timer_end = std::chrono::steady_clock::now();
        last_stats_.preprocess_time_ms = std::chrono::duration<double, std::milli>(timer_end - timer_start).count();

        std::cout << "CH preprocess complete: " << node_level_.size() << " nodes, "  << last_stats_.preprocess_time_ms << "ms\n";

    }


    Path CHRouter::findPath(Node* start, Node* end) {
        auto timer_start = std::chrono::steady_clock::now();
        last_stats_.nodes_explored = 0;


        //边界检查
        if (!isReady()) return Path{};
        if (!start || !end) return Path{};
        if (start == end) return Path{{start}, 0, 0, true};

        //===========双向Dijkstra初始化================
        //从起点向上搜索（正向图）
        using QueueItem = std::pair<double, Node*>; //<距离， 节点>
        std::priority_queue<QueueItem, std::vector<QueueItem>, std::greater<QueueItem>> pq_f;
        //从终点向上搜索（反向图）
        std::priority_queue<QueueItem, std::vector<QueueItem>, std::greater<QueueItem>> pq_b;

        //距离表
        std::unordered_map<NodeID, double> dist_f, dist_b;
        //前驱表
        std::unordered_map<NodeID, Node*> prev_f, prev_b;

        //初始化起点
        dist_f[start->id] = 0.0;
        pq_f.push({0.0, start});

        //初始化终点
        dist_b[end->id] = 0.0;
        pq_b.push({0.0, end});

        //最优相遇和相遇节点
        double best_dist = std::numeric_limits<double>::infinity();
        Node* meeting_node = nullptr;



        //================主循环：交替扩展两边============
        while (!pq_f.empty() || !pq_b.empty()) {
            //选择较小的一边扩展
            //如果一边选择空了，就扩展另一边
            bool expand_f = pq_f.empty() ? false : (pq_b.empty() ? true : pq_f.top().first <= pq_b.top().first);

            //引用别名，简化代码
            auto& pq = expand_f ? pq_f : pq_b;
            auto& dist = expand_f ? dist_f : dist_b;
            auto& prev = expand_f ? prev_f : prev_b;
            auto& other_dist = expand_f ? dist_b : dist_f;

            //取出最小距离节点
            auto [d, cur] = pq.top();
            pq.pop();
            last_stats_.nodes_explored++;

            //过期条目检查（Lazy Deletion）
            if (d > dist[cur->id]) continue;

            //===================检查是否相遇======================
            //如果另一边已经访问过这个节点，说明路径联通了
            if (other_dist.count(cur->id)) {
                double total = d + other_dist[cur->id];
                if (total < best_dist) {
                    best_dist = total;
                    meeting_node = cur;
                }
            }

            //=================扩展邻居：只走向上的边==========================
            int cur_level = node_level_[cur->id];

            //选择正向边或者反向边
            auto& edges= expand_f ? ch_up_outgoing[cur->id] : ch_up_incoming[cur->id];

            for (const auto& e : edges) {
                //核心约束：邻居等级必须>当前等级
                int nbr_level = node_level_[e.to->id];
                bool is_endPoint = (cur == start || cur == end);
                if (!is_endPoint && nbr_level < cur_level) continue;

                double nd = d + e.weight;

                //松弛操作
                if (!dist.count(e.to->id) || nd < dist[e.to->id]) {
                    dist[e.to->id] = nd;
                    prev[e.to->id] = cur;
                    pq.push({nd, e.to});
                }
            }
        }

        //============================重建路径===========================
        Path result;
        if (!meeting_node || best_dist == std::numeric_limits<double>::infinity()) {
            auto timer_end = std::chrono::steady_clock::now();
            last_stats_.query_time_ms = std::chrono::duration<double, std::milli>(timer_end - timer_start).count();
            return result; //未找到路径
        }

        //从meeting_node回溯到起点
        std::vector<Node*> path_from_start;
        Node* cur  = meeting_node;
        while (cur) {
            path_from_start.push_back(cur);
            auto it = prev_f.find(cur->id); 
            cur = (it != prev_f.end()) ? it->second : nullptr;
        }
        std::reverse(path_from_start.begin(), path_from_start.end());

        //从meeting_node回溯到终点（注意跳过meeting_node本身）
        std::vector<Node*> path_to_end;
        cur = meeting_node;
        auto it = prev_b.find(cur->id);
        cur = (it != prev_b.end()) ? it->second :nullptr; //跳过meeting_node
        while (cur) {
            path_to_end.push_back(cur);
            it = prev_b.find(cur->id);
            cur = (it != prev_b.end()) ? it->second : nullptr;
        }

        //合并：起点->meeting_node->终点
        result.nodes = path_from_start;//先放入起点到相遇点的路径
        result.nodes.insert(result.nodes.end(), path_to_end.begin(), path_to_end.end()); //在result.nodes的末尾插入path_to_end的所有元素
        
        result.found = true;
        result.total_time = best_dist;
        result.total_distance = 0; //如需可遍历边计算


        auto timer_end = std::chrono::steady_clock::now();
        last_stats_.query_time_ms = std::chrono::duration<double, std::milli>(timer_end - timer_start).count();
        return result;
    }

}
