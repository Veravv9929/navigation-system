#include <gtest/gtest.h>
#include "core/graph.h"
#include "core/map_loader.h"
#include "core/routing.h"
#include <random>

using namespace nav;

TEST(Benchmark, RealMapRouting) {
    // 加载真实地图
    RoadNetwork network;
    MapLoader loader;
    
    ASSERT_TRUE(loader.loadFromOSM("../data/test_data/small.osm", network));
    
    std::cout << "\n=== Map Load Statistics ===\n"
              << "Nodes: " << network.nodeCount() << "\n"
              << "Edges: " << network.edgeCount() << "\n"
              << "Load time: " << loader.stats().load_time_ms << "ms\n";
    
    DijkstraRouter dijkstra(network);
    AStarRouter astar(network);
    
    // 随机选 100 对起终点
    std::vector<Node*> nodes;
    for (const auto& [id, ptr] : network.nodes()) {
        nodes.push_back(ptr.get());
    }
    
    std::mt19937 rng(42); // 固定种子，结果可复现
    std::uniform_int_distribution<size_t> dist(0, nodes.size() - 1);
    
    double total_dijkstra_ms = 0;
    double total_astar_ms = 0;
    size_t success_count = 0;
    size_t total_d_explored = 0;
    size_t total_a_explored = 0;
    
    const int TEST_COUNT = 100;
    
    for (int i = 0; i < TEST_COUNT; ++i) {
        Node* start = nodes[dist(rng)];
        Node* end = nodes[dist(rng)];
        
        auto path_d = dijkstra.findShortestPath(start, end);
        auto path_a = astar.findShortestPath(start, end);
        
        if (path_d.found && path_a.found) {
            success_count++;
            total_dijkstra_ms += dijkstra.lastStats().compute_time_ms;
            total_astar_ms += astar.lastStats().compute_time_ms;
            total_d_explored += dijkstra.lastStats().nodes_explored;
            total_a_explored += astar.lastStats().nodes_explored;
            
            // 验证结果一致性
            ASSERT_DOUBLE_EQ(path_d.total_time, path_a.total_time);
        }
    }
    
    std::cout << "\n=== Routing Performance ===\n"
              << "Successful queries: " << success_count << "/" << TEST_COUNT << "\n"
              << "Dijkstra avg time: " << (total_dijkstra_ms / success_count) << "ms\n"
              << "A* avg time: " << (total_astar_ms / success_count) << "ms\n"
              << "Speedup: " << (total_dijkstra_ms / total_astar_ms) << "x\n"
              << "Dijkstra avg explored: " << (total_d_explored / success_count) << "\n"
              << "A* avg explored: " << (total_a_explored / success_count) << "\n"
              << "Node reduction: " << (1.0 - (double)total_a_explored/total_d_explored) * 100 << "%\n";
}


TEST(Benchmark, OptimizedDijkstra) {
    RoadNetwork network;
    MapLoader loader;
    ASSERT_TRUE(loader.loadFromOSM("../data/test_data/small.osm", network));
    network.buildIndex();

    std::vector<Node*> nodes;
    for (const auto& [id, ptr] : network.nodes()) {
        nodes.push_back(ptr.get());
    }

    // 随机选连通的节点对
    std::mt19937 rng(42);
    std::uniform_int_distribution<size_t> dist(0, nodes.size() - 1);
    
    DijkstraRouter test_router(network);
    Node* start = nullptr;
    Node* end = nullptr;
    
    for (int i = 0; i < 1000; ++i) {
        start = nodes[dist(rng)];
        end = nodes[dist(rng)];
        auto test_path = test_router.findShortestPath(start, end);
        if (test_path.found) break;
    }

    std::cout << "Testing: " << start->id << " -> " << end->id << "\n";

    DijkstraRouter router1(network);
    DijkstraRouter router2(network);

    auto path1 = router1.findShortestPath(start, end);
    ASSERT_TRUE(path1.found);
    auto stats1 = router1.lastStats();

    auto path2 = router2.findShortestPathFast(start, end);
    ASSERT_TRUE(path2.found);
    auto stats2 = router2.lastStats();

    EXPECT_DOUBLE_EQ(path1.total_time, path2.total_time);

    std::cout << "Original: " << stats1.compute_time_ms << "ms, "
              << "Optimized: " << stats2.compute_time_ms << "ms, "
              << "Speedup: " << (stats1.compute_time_ms / stats2.compute_time_ms) << "x\n";
}
