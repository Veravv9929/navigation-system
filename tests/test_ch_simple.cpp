#include "core/graph.h"
#include "core/routing.h"
#include "core/contraction_hierarchies.h"
#include <iostream>

using namespace nav;

int main() {
    RoadNetwork network;
    
    // 手动构建简单图：0-1-2-3
    network.addNode(0, 0, 0);
    network.addNode(1, 1, 0);
    network.addNode(2, 2, 0);
    network.addNode(3, 3, 0);
    
    network.addEdge(0, 0, 1, 100, 50, false);
    network.addEdge(1, 1, 2, 100, 50, false);
    network.addEdge(2, 2, 3, 100, 50, false);
    
    std::cout << "Nodes: " << network.nodeCount() << "\n";
    std::cout << "Edges: " << network.edgeCount() << "\n";
    
    // A* 对比
    AStarRouter astar(network);
    auto p_astar = astar.findShortestPath(network.getNode(0), network.getNode(3));
    std::cout << "A*: found=" << p_astar.found << " time=" << p_astar.total_time << "\n";
    
    // CH 测试
    CHRouter ch(network);
    std::cout << "Preprocessing...\n";
    ch.preprocess();
    
    auto p_ch = ch.findPath(network.getNode(0), network.getNode(3));
    std::cout << "CH: found=" << p_ch.found << " time=" << p_ch.total_time << "\n";
    
    if (p_ch.found && p_astar.found) {
        std::cout << "Match: " << (std::abs(p_ch.total_time - p_astar.total_time) < 0.01 ? "YES" : "NO") << "\n";
    }
    
    return 0;
}
