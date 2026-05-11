#include <gtest/gtest.h>
#include "core/graph.h"
#include "core/spatial_index.h"
#include "core/map_loader.h"

using namespace nav;

// 测试基础图构建
TEST(GraphTest, BasicConstruction) {
    RoadNetwork network;
    
    // 创建简单十字路网
    //     2
    //     |
    // 0---1---3
    //     |
    //     4
    
    auto n0 = network.addNode(0, 0, 0);
    auto n1 = network.addNode(1, 1, 0);
    auto n2 = network.addNode(2, 1, 1);
    auto n3 = network.addNode(3, 2, 0);
    auto n4 = network.addNode(4, 1, -1);
    
    network.addEdge(0, 0, 1, 100, 50, false); // 0-1
    network.addEdge(1, 1, 2, 100, 50, false); // 1-2
    network.addEdge(2, 1, 3, 100, 50, false); // 1-3
    network.addEdge(3, 1, 4, 100, 50, false); // 1-4
    
    EXPECT_EQ(network.nodeCount(), 5);
    EXPECT_EQ(network.edgeCount(), 8); // 4条双向边 = 8条有向边
    
    // 验证拓扑
    EXPECT_EQ(n1->outgoing.size(), 4); // 1号节点有4个出边
    EXPECT_EQ(n0->outgoing.size(), 1); // 0号节点只有1个出边（到1）
}

// 测试空间索引
TEST(SpatialIndexTest, NearestQuery) {
    RoadNetwork network;
    network.addNode(0, 39.9, 116.4);
    network.addNode(1, 39.901, 116.401);
    network.addNode(2, 39.91, 116.41); // 较远
    
    GridIndex index;
    index.build(network);
    
    auto nearest = index.nearest({39.9, 116.4}, 1);
    ASSERT_EQ(nearest.size(), 1);
    EXPECT_EQ(nearest[0]->id, 0); // 应该是0号节点
}

// 测试地图加载（需要准备测试数据文件）
TEST(MapLoaderTest, LoadSmallOSM) {
    // 准备一个小型测试OSM文件
    RoadNetwork network;
    MapLoader loader;
    
    // 假设有test_data/small.osm
    bool ok = loader.loadFromOSM("../data/test_data/small.osm", network);
    
    EXPECT_TRUE(ok);
    EXPECT_GT(loader.stats().nodes_parsed, 0);
    EXPECT_GT(loader.stats().edges_created, 0);
    std::cout << "Loaded " << network.nodeCount() << " nodes, " 
              << network.edgeCount() << " edges in " 
              << loader.stats().load_time_ms << "ms\n";
}

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
