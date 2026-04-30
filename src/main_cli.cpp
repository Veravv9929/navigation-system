#include <iostream>
#include <cstdlib>
#include "core/graph.h"
#include "core/map_loader.h"
#include "core/spatial_index.h"

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
    } else {
        printUsage(argv[0]);
        return 1;
    }

    return 0;
}
