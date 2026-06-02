#pragma once
#include "core/routing.h"
#include <string>

namespace nav {

    class AlgorithmRouter {
        public:
            enum class Strategy {
                AUTO,
                DIJKSTRA,
                DIJKSTRA_FAST,
                ASTAR
            };
            
            explicit AlgorithmRouter(const RoadNetwork& network);

            //统一路径查询接口
            Path findPath(Node* start, Node* end, Strategy strategy = Strategy::AUTO, bool enable_stats = true);

            //获取上次查询统计
            struct QueryStats {
                std::string algorithm_name;
                double compute_time_ms = 0;
                size_t nodes_explored = 0;
                size_t edges_relaxed = 0;
                size_t path_hops = 0;
                double path_distance = 0;
                double path_duration = 0;
            };

            const QueryStats& lastStats() const { return last_stats_; }

            //策略转字符串（用于日志/响应）
            static std::string strategyName(Strategy s);

        private:
            const RoadNetwork& network_;
            DijkstraRouter dijkstra_;
            AStarRouter astar_;
            QueryStats last_stats_;

            Strategy selectAutoStrategy() const;

    };
}
