#pragma once
#include "core/graph.h"
#include "core/routing.h"
#include <vector>
#include <unordered_map>

namespace nav {
    struct CHEdge {
        Node* to;
        double weight;
        bool is_shortcut; //true = 预处理添加的捷径， false = 原图边
    };
    class CHRouter {
        public:
            explicit CHRouter(const RoadNetwork& network);

            //预处理：构建层次结构（离线构建，耗时较长）
            void preprocess();

            //查询：双向向上搜索（在线，极快）
            Path findPath(Node* start, Node* end);

            //统计
            struct Stats {
                double preprocess_time_ms = 0; //预处理耗时
                double query_time_ms = 0; //单词查询耗时
                size_t nodes_explored = 0; //探索节点数
            };
            const Stats& lastStats() const {return last_stats_; }

            //查询是否已经完成了预处理
            bool isReady() const {return !node_level_.empty();}

        private:
            const RoadNetwork& network_;
            Stats last_stats_;

            //节点等级（重要性） : 0 = 最低，越高越重要
            std::unordered_map<NodeID, int> node_level_;

            //预处理后的图：每个节点的向上出边
            std::unordered_map<NodeID, std::vector<CHEdge>> ch_up_outgoing;
            //每个节点的向上入边
            std::unordered_map<NodeID, std::vector<CHEdge>> ch_up_incoming;

            //辅助方法
            //判断节点重要性
            double computeEdgeDifference(Node* node);
            //从图中移除，添加必要捷径边
            void contractNode(Node* node);
            //判断是否需要捷径
            bool needShortcut(Node* u, Node* v, Node* node, double via_dist);
            //限制范围的局部搜索，找不经过被收缩节点的替代路径
            double witnessSearch(Node* from, Node* to, double max_dist, int max_hops);
    };
}

