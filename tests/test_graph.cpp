#include <gtest/gtest.h>
#include "core/graph.h"
#include "core/spatial_index.h"
#include "core/map_loader.h"
#include <unordered_set>

using namespace nav;

//测试1：基础图构建
TEST(GraphTest, BasicConstruction) {
    RoadNetwork network;

    //创建简单的十字路网
    //      2
    //      /
    //  0---1---3
    //      /
    //      4
    auto n0 = network.addNode(0, 0, 0);
    auto n1 = network.addNode(1, 1, 0);
    auto n2 = network.addNode(2, 1, 1);
    auto n3 = network.addNode(3, 2, 0);
    auto n4 = network.addNode(4, 1, -1);

    network.addEdge(0, 0, 1, 100, 50, false);
    network.addEdge(1, 1, 2, 100, 50, false);
    network.addEdge(2, 1, 3, 100, 50, false);
    network.addEdge(3, 1, 4, 100, 50, false);

    EXPECT_EQ(network.nodeCount(), 5);
    EXPECT_EQ(network.edgeCount(), 8);

    //验证拓扑结构
    EXPECT_EQ(n1->outgoing.size(), 4);
    EXPECT_EQ(n0->outgoing.size(), 1);
}

    //测试空间索引
TEST(SpatialIndexTest, NearestQuery) {
    RoadNetwork network;
    network.addNode(0, 39.9, 116.4);
    network.addNode(1, 39.901, 116.401);
    network.addNode(2, 39.91, 116.41);
    std::cout << "nodeCount = " << network.nodeCount() << std::endl;

    //构建空间索引
    GridIndex index;
    index.build(network);

    auto nearest = index.nearest({39.9, 116.4}, 1);
    
    ASSERT_EQ(nearest.size(), 1);
    EXPECT_EQ(nearest[0]->id, 0);

}

TEST(SpatialIndexTest, RangeQuery) {
    RoadNetwork network;
    auto n0 = network.addNode(0, 39.9, 116.4);
    auto n1 = network.addNode(1, 39.901, 116.401);
    auto n2 = network.addNode(2, 40.0, 117.0);

    GridIndex index;
    index.build(network);

    auto result = index.rangeQuery(39.89, 116.39, 39.91, 116.41);

    ASSERT_EQ(result.size(), 2);  // ASSERT 确保后续安全

    // 组合验证：ID + 坐标 + 不包含外部节点
    std::unordered_set<uint64_t> result_ids;
    for (Node* node : result) {
        result_ids.insert(node->id);
        EXPECT_GE(node->coord.lat, 39.89);
        EXPECT_LE(node->coord.lat, 39.91);
    }

    EXPECT_TRUE(result_ids.count(0));
    EXPECT_TRUE(result_ids.count(1));
    EXPECT_FALSE(result_ids.count(2));  // 节点2在范围外，不应出现
}

//测试地图加载（需要准备测试数据文件）
TEST(MapLoadeTest, LoadSmallOSM) {
    //准备一个小型测试OSM文件
    RoadNetwork network;
    MapLoader loader;

    //假设有test_data/small.osm
    bool ok = loader.loadFromOSM("../tests/test_data/small.osm", network);

    EXPECT_TRUE(ok);
    EXPECT_GT(loader.stats().nodes_parsed, 0);
    EXPECT_GT(loader.stats().edges_created, 0);
    std::cout << "Loaded" << network.nodeCount() << " nodes," << network.edgeCount() << "edges in" << loader.stats().load_time_ms  << "ms\n";
}

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}

