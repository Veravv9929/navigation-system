#include "core/algorithm_router.h" 
#include <stdexcept>

namespace nav {

    AlgorithmRouter::AlgorithmRouter(const RoadNetwork& network) : network_(network), dijkstra_(network_), astar_(network) {}

    Path AlgorithmRouter::findPath(Node* start, Node* end, Strategy strategy, bool enable_stats) {

        //空指针检查，清空上次统计
        if (!start || !end) {
            last_stats_ = QueryStats{};
            return Path();
        }

        //自动选择策略
        if (strategy == Strategy::AUTO) {
            strategy = selectAutoStrategy();
        }

        //执行对应算法
        Path result;
        switch (strategy) {
            case Strategy::DIJKSTRA:
                result = dijkstra_.findShortestPath(start, end);
                last_stats_.algorithm_name = "dijkstra";
                break;
                
            case Strategy::DIJKSTRA_FAST:
                //确保连续索引已经构建
                if (network_.indexSize() == 0) {
                    //未构建则回退到基础版
                    result = dijkstra_.findShortestPath(start, end);
                    last_stats_.algorithm_name = "dijkstra_fallback";
                } else {
                    result = dijkstra_.findShortestPathFast(start, end);
                    last_stats_.algorithm_name = "dijkstra_fast";
                }
                break;

            case Strategy::ASTAR:
                result = astar_.findShortestPath(start, end);
                last_stats_.algorithm_name = "astar";
                break;

            default:
                throw::std::runtime_error("Unknow strategy");
        }

        //收集统计
        if (enable_stats && result.found) {
            //收集统计
            if (enable_stats) {
                switch (strategy){
                    case Strategy::DIJKSTRA:
                    case Strategy::DIJKSTRA_FAST: 
                        {
                            const auto& s = dijkstra_.lastStats();
                            last_stats_.compute_time_ms = s.compute_time_ms;
                            last_stats_.nodes_explored = s.nodes_explored;
                            last_stats_.edges_relaxed = s.edges_relaxed;
                            break;
                        }
                        
                    case Strategy::ASTAR:
                        {
                            const auto& s = astar_.lastStats();
                            last_stats_.compute_time_ms = s.compute_time_ms;
                            last_stats_.nodes_explored = s.nodes_explored;
                            last_stats_.edges_relaxed = s.edges_relaxed;
                            break;
                        }

                    default:break;
                }
            }

            last_stats_.path_hops = result.hopCount();
            last_stats_.path_distance = result.total_distance;
            last_stats_.path_duration = result.total_time;
        }

        return result;
    }

    AlgorithmRouter::Strategy AlgorithmRouter::selectAutoStrategy() const {
        size_t n = network_.nodeCount();

        //策略选择逻辑
        if (n < 10000) {
            return Strategy::DIJKSTRA;
        } else if (n < 100000) {
            return Strategy::DIJKSTRA_FAST;
        } else {
            return Strategy::ASTAR;
        }
    }

    std::string AlgorithmRouter::strategyName(Strategy s) {
        switch (s) {
            case Strategy::AUTO: return "auto";
            case Strategy::DIJKSTRA: return "dijkstra";
            case Strategy::DIJKSTRA_FAST: return "dijkstra_fast";
            case Strategy::ASTAR: return "astar";
        }
        return "unknown";
    }
}
