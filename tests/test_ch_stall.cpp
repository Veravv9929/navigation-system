#include "core/graph.h"
#include "core/map_loader.h"
#include "core/routing.h"
#include "core/contraction_hierarchies.h"
#include <iostream>

using namespace nav;

int main() {
    RoadNetwork network;
    MapLoader loader;
    
    std::cout << "Loading map...\n";
    if (!loader.loadFromOSM("../data/test_data/small.osm", network)) {
        std::cerr << "Load failed\n";
        return 1;
    }
    
    std::cout << "Nodes: " << network.nodeCount() << "\n";
    std::cout << "Edges: " << network.edgeCount() << "\n";
    
    CHRouter ch(network);
    std::cout << "Preprocessing...\n";
    ch.preprocess();
    
    // 收集节点
    std::vector<Node*> nodes;
    for (const auto& [id, ptr] : network.nodes()) {
        nodes.push_back(ptr.get());
    }
    
    // 测试 20 对随机节点
    AStarRouter astar(network);
    int ch_ok = 0, astar_ok = 0, both_ok = 0;
    
    for (int i = 0; i < 20 && i + 1 < (int)nodes.size(); i++) {
        Node* s = nodes[i * 100];  // 间隔取，避免太近
        Node* e = nodes[i * 100 + 50];
        
        auto p_ch = ch.findPath(s, e);
        auto p_astar = astar.findShortestPath(s, e);
        
        if (p_ch.found) ch_ok++;
        if (p_astar.found) astar_ok++;
        if (p_ch.found && p_astar.found) both_ok++;
        
        if (!p_ch.found && p_astar.found) {
            std::cout << "CH FAIL: " << s->id << " -> " << e->id 
                      << " (A* found=" << p_astar.found << ")\n";
        }
    }
    
    std::cout << "\nResults: CH=" << ch_ok << " A*=" << astar_ok 
              << " Both=" << both_ok << "/20\n";
    
    return 0;
}
