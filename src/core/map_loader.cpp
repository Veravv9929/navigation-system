#include "core/map_loader.h"
#include <tinyxml2.h>
#include <unordered_map>
#include <unordered_set>
#include <chrono>

namespace nav {
    
        uint64_t MapLoader::next_edge_id_ = 0;

    bool MapLoader:: loadFromOSM(const std::string& filename, RoadNetwork& network) {
        auto start = std::chrono::steady_clock::now();

        tinyxml2::XMLDocument doc;
        if (doc.LoadFile(filename.c_str()) != tinyxml2::XML_SUCCESS) {
            return false;
        }

        //第一阶段：解析所有node
        std::unordered_map<uint64_t, std::pair<double, double>> node_coords;

        auto osm = doc.RootElement();
        for (auto elem = osm->FirstChildElement(); elem; elem = elem->NextSiblingElement()) {
            if (std::string(elem->Name()) == "node") {
                uint64_t id = std::stoull(elem->Attribute("id"));
                double lat = std::stod(elem->Attribute("lat"));
                double lon = std::stod(elem->Attribute("lon"));
                node_coords[id] = {lat, lon};
                stats_.nodes_parsed++;
            }
        }
        
        //第二阶段：解析way(道路),创建edge
        for (auto elem = osm->FirstChildElement(); elem; elem = elem->NextSiblingElement()) {
            if (std::string(elem->Name()) != "way") continue;

            //提取道路属性
            std::string highway_type;
            bool oneway = false;

            for (auto tag = elem->FirstChildElement("tag"); tag; tag = tag->NextSiblingElement("tag")) {
                const char* k = tag->Attribute("k");
                const char* v = tag->Attribute("v");
                if (!k || !v) continue;

                if (std::string(k) == "highway") highway_type = v;
                if (std::string(k) == "oneway" && std::string(v) == "yes") oneway = true;
            }

            //只保留可通行道路
            if (!isDrivableRoad(highway_type)) continue;
            stats_.ways_parsed++;

            //提取node引用序列
            std::vector<uint64_t> node_refs;
            for (auto nd = elem->FirstChildElement("nd"); nd; nd = nd->NextSiblingElement("nd")) {
                node_refs.push_back(std::stoull(nd->Attribute("ref")));
            }


            //创建节点（如果不存在）和边
            double speed = getSpeedLimit(highway_type);

            for (size_t i = 0; i < node_refs.size() - 1; ++i) {
                uint64_t from_id = node_refs[i];
                uint64_t to_id = node_refs[i+1];

                if (!network.getNode(from_id)) {
                        auto& coord = node_coords[from_id];
                        network.addNode(from_id, coord.first, coord.second);
                }
                if (!network.getNode(to_id)) {
                        auto& coord = node_coords[to_id];
                        network.addNode(to_id, coord.first, coord.second);
                }

                //计算边得长度（简化版，实际上可用Haversine）
                auto& c1 = node_coords[from_id];
                auto& c2 = node_coords[to_id];
                double dist = std::sqrt(std::pow(c1.first-c2.first, 2) + std::pow(c1.second-c2.second, 2)) * 111000;

                //创建边 （ID用way_id * 1000 + seggment_index避免冲突）
                uint64_t edge_id = next_edge_id_++;
                network.addEdge(edge_id, from_id, to_id, dist, speed, oneway);
                stats_.edges_created++;
            }
        }

        auto end =  std::chrono::steady_clock::now();
        stats_.load_time_ms = std::chrono::duration<double, std::milli>(end - start).count();

        return true;
    }

    bool MapLoader::isDrivableRoad(const std::string& type) const {
        static const std::unordered_set<std::string> drivable = {
            "motorway", "trunk", "primary", "secondary", "tertiary", "residential", "unclassified", "service", "motorway_link"
        };
        return drivable.count(type);
    }

    double MapLoader::getSpeedLimit(const std::string& type) const {
        //默认速度（km/h）
        if (type == "motorway") return 120;
        if (type == "trunk") return 80;
        if (type == "primary") return 70;
        if (type == "secondary") return 60;
        if (type == "tertiary") return 50;
        if (type == "residential") return 30;
        return 40;//默认
    }

}
