#include "core/map_loader.h"
#include "core/algorithm_router.h"
#include "core/spatial_index.h"
#include <nlohmann/json.hpp>
#include <iostream>
#include <cstdlib>
#include <cmath>

using json = nlohmann::json;

int main(int argc, char* argv[]) {
    if (argc < 2) {
        std::cerr << "Usage: " << argv[0] << " <osm_file>\n";
        return 1;
    }

    std::string osm_file = argv[1];

    //加载地图
    nav::RoadNetwork network;
    nav::MapLoader loader;
    if (!loader.loadFromOSM(osm_file, network)) {
        json err{{"error", "failed_to_load_map"}};
        std::cout << err.dump() << std::endl;
        return 1;
    }
    network.buildIndex();

    //初始化算法路由器
    nav::AlgorithmRouter router(network);

    //读取stdin的JSON请求（Python通过stdin传入）
    std::string input;
    std::getline(std::cin, input);

    json req;
    try {
        req = json::parse(input);
    } catch (const std::exception& e) {
        json err{{"error", "invalid_json"}, {"detail", e.what()}};
        std::cout << err.dump() << std::endl;
        return 1;
    }

    std::string cmd = req.value("cmd", " ");

    if (cmd == "route") {
        //解析参数
        double start_lat = req["start"]["lat"];
        double start_lon = req["start"]["lon"];
        double end_lat = req["end"]["lat"];
        double end_lon = req["end"]["lon"];
        std::string algo_str = req.value("algorithm", "auto");

        //地图匹配：找最近节点(这里指的是找距离输入的位置最近的网格节点)
        nav::GridIndex index;
        index.build(network);

        auto start_nodes = index.nearest({start_lat, start_lon}, 1);
        auto end_nodes =index.nearest({end_lat, end_lon}, 1);

        if (start_nodes.empty() || end_nodes.empty()) {
            json err{{"error", "no_nearby_road"}};
            //加调试信息
            err["query_start"] = {{"lat", start_lat}, {"lon", start_lon}};
            err["query_end"] = {{"lat", end_lat}, {"lon", end_lon}};
            err["map_nodes"] = network.nodeCount();
            std::cout << err.dump() << std::endl;
            return 0;
        }

        //选择算法
        nav::AlgorithmRouter::Strategy strategy = nav::AlgorithmRouter::Strategy::AUTO;
        if (algo_str == "dijkstra") strategy = nav::AlgorithmRouter::Strategy::DIJKSTRA;
        else if (algo_str == "dijkstra_fast") strategy = nav::AlgorithmRouter::Strategy::DIJKSTRA_FAST;
        else if (algo_str == "astar") strategy = nav::AlgorithmRouter::Strategy::ASTAR;

        //计算路径
        auto path = router.findPath(start_nodes[0], end_nodes[0], strategy);
        const auto& stats = router.lastStats();

        //构建响应
        json response;
        response["status"] = path.found ? "ok" : "no_path";
        response["algorithm_used"] = stats.algorithm_name;
        response["algorithm_request"] = algo_str;
        response["compute_time_ms"] = stats.compute_time_ms;
        response["nodes_explored"] = stats.nodes_explored;
        response["edges_relaxed"] = stats.edges_relaxed;
        response["distance"] = stats.path_distance;
        response["duration"] = stats.path_duration;
        response["hop_count"] = stats.path_hops;

        json path_array = json::array();
        for (auto* node : path.nodes) {
            json point;
            point["lat"] = node->coord.lat;
            point["lon"] = node->coord.lon;
            path_array.push_back(point);
        }
        response["path"] = path_array;

        std::cout << response.dump() << std::endl;

    } else if (cmd == "match") {
        double lat = req["lat"];
        double lon = req["lon"];
        int k = req.value("k", 3);

        nav::GridIndex index;
        index.build(network);

        auto candidates = index.nearest({lat, lon}, k);
        
        json response;
        response["status"] = "ok";
        response["query"] = {{"lat", lat}, {"lon", lon}};

        json cand_array = json::array();
        for (auto* node : candidates) {
            json c;
            c["node_id"] = node->id;
            c["lat"] = node->coord.lat;
            c["lon"] = node->coord.lon;
            double dx = lat - node->coord.lat;
            double dy = lon - node->coord.lon;
            c["distance_meters"] = std::sqrt(dx*dx + dy*dy) * 111000;
            cand_array.push_back(c);
        }
        response["candidates"] = cand_array;
        response["count"] = cand_array.size();

        std::cout << response.dump() << std::endl;
    } else if (cmd == "algorithms") {
        json response;
        response["algorithms"] = {"auto", "dijkstra", "dijkstra_fast", "astar"};
        response["map_nodes"] = network.nodeCount();
        response["map_edges"] = network.edgeCount();
        std::cout << response.dump() << std::endl;
    } else {
        json err{{"error", "unknown_cmd"}, {"valid", {"route", "match", "algorithms"}}};
        std::cout << err.dump() << std::endl;
    }
    
    return 0;
}
