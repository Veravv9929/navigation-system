#include <gtest/gtest.h>
#include "core/graph.h"
#include "core/routing.h"

using namespace nav;

// 构建测试用的简单网格路网
class RoutingTest : public ::testing::Test {
protected:
    void SetUp() override {
        // 3x3 网格，节点间距 100米，限速 50km/h
        // 0-1-2
        // | | |
        // 3-4-5
        // | | |
        // 6-7-8
        
        for (int i = 0; i < 9; ++i) {
            double lat = 39.9 + (i / 3) * 0.001;  // 行
            double lon = 116.4 + (i % 3) * 0.001; // 列
            network.addNode(i, lat, lon);
        }
        
        // 横向边
        for (int row = 0; row < 3; ++row) {
            for (int col = 0; col < 2; ++col) {
                int from = row * 3 + col;
                int to = row * 3 + col + 1;
                network.addEdge(row*10 + col, from, to, 100, 50, false);
            }
        }
        // 纵向边
        for (int col = 0; col < 3; ++col) {
            for (int row = 0; row < 2; ++row) {
                int from = row * 3 + col;
                int to = (row + 1) * 3 + col;
                network.addEdge(30 + col*10 + row, from, to, 100, 50, false);
            }
        }
    }
    
    RoadNetwork network;
};

TEST_F(RoutingTest, DijkstraBasicPath) {
    DijkstraRouter router(network);
    auto path = router.findShortestPath(network.getNode(0), network.getNode(8));
    
    ASSERT_TRUE(path.found);
    EXPECT_EQ(path.nodes.front()->id, 0);
    EXPECT_EQ(path.nodes.back()->id, 8);
    // 最短路径应该是 0->1->2->5->8 或 0->3->6->7->8，距离都是 400米
    EXPECT_NEAR(path.total_time, 400 / (50/3.6), 0.1); // 28.8秒
}

TEST_F(RoutingTest, AStarSameAsDijkstra) {
    DijkstraRouter dijkstra(network);
    AStarRouter astar(network);
    
    auto path_d = dijkstra.findShortestPath(network.getNode(0), network.getNode(8));
    auto path_a = astar.findShortestPath(network.getNode(0), network.getNode(8));
    
    // A* 和 Dijkstra 结果必须一致（都是最优解）
    ASSERT_TRUE(path_d.found);
    ASSERT_TRUE(path_a.found);
    EXPECT_DOUBLE_EQ(path_d.total_time, path_a.total_time);
    EXPECT_EQ(path_d.nodes.size(), path_a.nodes.size());
}

TEST_F(RoutingTest, AStarFasterThanDijkstra) {
    DijkstraRouter dijkstra(network);
    AStarRouter astar(network);
    
    // 在更大网络上测试，这里用多次查询模拟
    for (int i = 0; i < 100; ++i) {
        dijkstra.findShortestPath(network.getNode(0), network.getNode(8));
        astar.findShortestPath(network.getNode(0), network.getNode(8));
    }
    
    auto& stats_d = dijkstra.lastStats();
    auto& stats_a = astar.lastStats();
    
    std::cout << "Dijkstra explored: " << stats_d.nodes_explored 
              << ", A* explored: " << stats_a.nodes_explored << "\n";
    
    // A* 应该探索更少节点
    EXPECT_LE(stats_a.nodes_explored, stats_d.nodes_explored);
}

TEST_F(RoutingTest, UnreachableNode) {
    // 添加孤立节点
    network.addNode(100, 40.0, 117.0);
    
    DijkstraRouter router(network);
    auto path = router.findShortestPath(network.getNode(0), network.getNode(100));
    
    EXPECT_FALSE(path.found);
    EXPECT_TRUE(path.nodes.empty());
}

TEST_F(RoutingTest, SameStartEnd) {
    DijkstraRouter router(network);
    auto path = router.findShortestPath(network.getNode(4), network.getNode(4));
    
    EXPECT_TRUE(path.found);
    EXPECT_EQ(path.nodes.size(), 1);
    EXPECT_EQ(path.nodes[0]->id, 4);
}
