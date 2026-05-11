#pragma once
#include "core/graph.h"
#include <vector>
#include <limits>

namespace nav {

    //路径结果
    struct Path {
        std::vector<Node*> nodes;         //经过的节点序列
        double total_distance = 0;        //总距离（米）
        double total_time = 0;            //总时间（秒）
        bool found = false;               //是否找到路径

        size_t hopCount() const { return nodes.empty() ? 0 : nodes.size() - 1;};
    };

    //Djikstra 算法实现
    class DijkstraRouter {
        public:
            explicit DijkstraRouter(const RoadNetwork& network);

            //计算从start 到 end 的最短路径
            Path findShortestPath(Node* start, Node* end);
            Path findShortestPathFast(Node* start, Node* end);

            //获取算法统计（用于性能分析）
            struct Stats {
                size_t nodes_explored = 0; //访问了多少节点
                size_t edges_relaxed = 0; //松弛了多少边
                double compute_time_ms = 0; //计算耗时
            };
            const Stats& lastStats() const {return last_stats_; }
        private:
            const RoadNetwork& network_;
            Stats last_stats_;
    };

    //A* 算法实现
    class AStarRouter {
        public:
            explicit AStarRouter(const RoadNetwork& network);

            Path findShortestPath(Node* start, Node* end);
            const DijkstraRouter::Stats& lastStats() const { return last_stats_; }

        private:
            const RoadNetwork& network_;
            DijkstraRouter::Stats last_stats_;
            
            //启发式函数：估计从node到end的代价
            //用欧几里得距离 / 最大速度 （保证可采纳，即不高估）
            double heuristic(Node* node, Node* end) const;
    };
}
