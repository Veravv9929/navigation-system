#include <gtest/gtest.h>
#include "core/algorithm_router.h"
#include "core/graph.h"

using namespace nav;

//构建3*3网格测试路网
RoadNetwork buildGirdNetwork() {
    RoadNetwork network;
    for (int i =0; i< 9; i++) {
        double lat = 39.9 + (i / 3)  * 0.001;
        double lon = 116.4 + (i % 3) * 0.001;
        network.addNode(i, lat, lon);
    }
    //横向边
    for (int row = 0; row < 3; ++row) {
        for(int col = 0; col < 2; ++col) {
            int from = row * 3 + col;
            int to = row *3 + col + 1;
            network.addEdge(row*10 + col, from, to, 100, 50, false);
        }
    }
    //纵向边
    for (int col = 0; col < 3; ++col) {
        for (int row = 0; row < 2; ++row) {
            int from = row *3 + col;
            int to = (row + 1) * 3 + col;
            network.addEdge(30 + col*10 + row, from, to, 100, 50, false);
        }
    }
    return network;
}

TEST(AlgorithmRouter, AutoSelectDijkstra) {
    RoadNetwork network = buildGirdNetwork();
    AlgorithmRouter router(network);

    auto* start = network.getNode(0);
    auto* end = network.getNode(8);

    auto path = router.findPath(start, end, AlgorithmRouter::Strategy::AUTO);

    EXPECT_TRUE(path.found);
    EXPECT_EQ(router.lastStats().algorithm_name, "dijkstra");
    EXPECT_GT(router.lastStats().compute_time_ms, 0);
    EXPECT_GT(router.lastStats().nodes_explored, 0);
}

TEST(AlgorithmRouter, ManualSelectAStar) {
    RoadNetwork network = buildGirdNetwork();
    AlgorithmRouter router(network);

    auto path = router.findPath(network.getNode(0), network.getNode(8), AlgorithmRouter::Strategy::ASTAR);

    EXPECT_TRUE(path.found);
    EXPECT_EQ(router.lastStats().algorithm_name, "astar");
    EXPECT_GT(router.lastStats().compute_time_ms, 0);
}

TEST(AlgorithmRouter, FastFallbackWhenNoIndex) {
    RoadNetwork network = buildGirdNetwork();
    //不调用network.buildIndex()
    AlgorithmRouter router(network);

    auto path = router.findPath(network.getNode(0), network.getNode(8), AlgorithmRouter::Strategy::DIJKSTRA_FAST);

    EXPECT_TRUE(path.found);
    EXPECT_EQ(router.lastStats().algorithm_name, "dijkstra_fallback");
}

TEST(AlgorithmRouter, FastWhenIndexBuilt){
    RoadNetwork network = buildGirdNetwork();
    network.buildIndex();
    AlgorithmRouter router(network);

    auto path = router.findPath(network.getNode(0), network.getNode(8), AlgorithmRouter::Strategy::DIJKSTRA_FAST);

    EXPECT_TRUE(path.found);
    EXPECT_EQ(router.lastStats().algorithm_name, "dijkstra_fast");
}

TEST(AlgorithmRouter, StatsArePopulated) {
    RoadNetwork network = buildGirdNetwork();
    AlgorithmRouter router(network);

    auto path = router.findPath(network.getNode(0), network.getNode(8));
    EXPECT_TRUE(path.found);
    const auto& stats = router.lastStats();

    //验证所有统计字段都有值
    EXPECT_FALSE(stats.algorithm_name.empty());
    EXPECT_GT(stats.compute_time_ms, 0);
    EXPECT_GT(stats.nodes_explored, 0);
    EXPECT_EQ(stats.path_hops, path.hopCount());
    EXPECT_DOUBLE_EQ(stats.path_duration, path.total_time );
}

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
