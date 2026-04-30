#pragma once
#include "core/graph.h"
#include <string>
#include <cmath>

namespace nav{
    
class MapLoader{
public:
    //类的接口
    bool loadFromOSM(const std::string& filename, RoadNetwork& network);

    struct Stats {
        size_t nodes_parsed = 0;
        size_t ways_parsed = 0;
        size_t edges_created = 0;
        double load_time_ms = 0;
    };
    const Stats& stats() const { return stats_; }

private:
    Stats stats_;
    static uint64_t next_edge_id_;    //全局边ID计数器

    //解析辅助函数
    bool isDrivableRoad(const std::string& highway_type) const;
    double getSpeedLimit(const std::string& highway_type) const;
};

}
