#include <iostream>
#include <cstdlib>
#include "core/graph.h"
#include "core/map_loader.h"
#include "core/spatial_index.h"
#include "core/routing.h"

using namespace nav;

void printUsage(const char* name) {
    std::cout << "Usage:\n" << " "  << name << " load <osm_file> - 加载地图并显示统计\n" << " " << name << " nearest <lat> <lon> - 查找最近节点\n";
}

int main(int argc, char* argv[]) {
    if (argc < 2) {
        printUsage(argv[0]);
        return 1;
    }

    std::string cmd = argv[1];

    if (cmd == "load" && argc >= 3) {
        RoadNetwork network;
        MapLoader loader;

        std::cout << "Loading OSM file: "  <<  argv[2] << "...\n";

        if (!loader.loadFromOSM(argv[2], network)) {
            std::cerr << "Failed to load map!\n";
            return 1;
        }

        auto& s = loader.stats();
        std::cout << " Load complete!\n"
                  << " Nodes parsed: " << s.nodes_parsed << "\n"
                  << " Ways parsed: " << s.ways_parsed << "\n"
                  << " Edges created: " << s.edges_created << "\n"
                  << " Load Time: " << s.load_time_ms << "ms\n"
                  << " Memory nodes: " << network.nodeCount() << "\n";
        //构建空间索引并测试
        GridIndex index;
        index.build(network);
        std::cout << " Spatial index built.\n" ;

    } else if (cmd == "nearest" && argc >= 4) {
        //需要先加载地图，这里简化演示
        std::cout << "Nearest query at (" << argv[2] << ", " << argv[3] << ")\n";
        //实际实现需要持久化network对象。。。
    } else if (cmd == "route" && argc >= 7) {
        //1.加载地图
        RoadNetwork network;
        MapLoader loader;

        std::cout << "Loading OSM file: " << argv[2] << "...\n";

        if (!loader.loadFromOSM(argv[2], network)) {
            std::cerr << "Failed to load map!\n";
            return 1;
        }
        network.buildIndex();

        std::cout << "Load complete! Nodes: " << network.nodeCount()
                  << ", Edges: " << network.edgeCount() << "\n";

        //2.解析起终点坐标
        double start_lat = std::stod(argv[3]);
        double start_lon = std::stod(argv[4]);
        double end_lat = std::stod(argv[5]);
        double end_lon = std::stod(argv[6]);
        std::string algo = (argc >= 8) ? argv[7] : "astar";

        std::cout << "Route from (" << start_lat << ", " << start_lon << ") "
                  << "to (" << end_lat << ", " << end_lon << ")\n";

        //3.空间索引找最近道路节点
        GridIndex index;
        index.build(network);

        auto start_nodes = index.nearest({start_lat, start_lon}, 1); //其实就是给定经度纬度处的点，只不过有时候我们的节点不是那么精确而已啦。就好像上车点我们得走一点点一样。end_nodes也是这样啦。
        auto end_nodes = index.nearest({end_lat, end_lon}, 1);

        if (start_nodes.empty() || end_nodes.empty()) {
            std::cerr << "Cannot find nearby road nodes\n";
            return 1;
        }

        std::cout << "Start node: " << start_nodes[0]->id
                  << " at (" << start_nodes[0]->coord.lat << ", " << start_nodes[0]->coord.lon << ")\n";
        std::cout << "End node: " << end_nodes[0]->id
                  << "at (" <<end_nodes[0]->coord.lat << ", " << end_nodes[0]->coord.lon << ")\n";

        //4.路径计算
        Path path;
        if (algo == "dijkstra") {
            DijkstraRouter router(network);
            path = router.findShortestPath(start_nodes[0], end_nodes[0]);
            std::cout << "\nAlgorithm: Dijkstra\n";
            std::cout << "Nodes explored: " << router.lastStats().nodes_explored << "\n";
            std::cout << "Time: " << router.lastStats().compute_time_ms << "ms\n";
        } else {
            AStarRouter router(network);
            path = router.findShortestPath(start_nodes[0], end_nodes[0]);
            std::cout << "\nAlgorithm: A*\n";
            std::cout << "Nodes explored: " << router.lastStats().nodes_explored << "\n";
            std::cout << "Time: " << router.lastStats().compute_time_ms << "ms\n";
        }
        
        //5.输出结果
        if (!path.found) {
            std::cout << "No path found!\n";
            return 0;
        }

        std::cout << "\n=== Path Result ===\n"
                  << "Total distance: " << path.total_distance << "m\n"
                  << "Total time: " << path.total_time << "s\n" 
                  << "Hops: " << path.hopCount() << "\n"
                  << "Path nodes: " << path.nodes.size() << "\n\n";

        // 输出前五个和后五个坐标点
       std::cout << "First 5 points:\n";
       for (size_t i = 0; i < std::min(size_t(5), path.nodes.size()); ++i) {
           auto& c = path.nodes[i]->coord;
           std::cout << "  [" << i << "] " << c.lat << ", " << c.lon << "\n";
       }
       if (path.nodes.size() > 10) {
           std::cout << "  ... (" << path.nodes.size() - 10 << " points omitted) ...\n";
       }
       std::cout << "Last 5 points:\n";
       for (size_t i = std::max(size_t(5), path.nodes.size()) - 5; i < path.nodes.size(); ++i) {
           auto& c = path.nodes[i]->coord;
           std::cout << "  [" << i << "] " << c.lat << ", " << c.lon << "\n";
       } 

       //6.输出GeoJSON(用于网页可视化)
       std::cout << "\n=== GeoJSON ===\n";
       std::cout << "{\"type\":\"LineString\",\"coordinates\":[";
       for (size_t i = 0; i < path.nodes.size(); ++i) {
           if (i > 0) std::cout << ",";
           std::cout << "[" << path.nodes[i]->coord.lon << ","
               << path.nodes[i]->coord.lat << "]";
       }
       std::cout << "]}\n";

    
    } else {
        printUsage(argv[0]);
        return 1;
    }

    return 0;
}
