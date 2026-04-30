#pragma once
#include <cstdint>
#include <vector>
#include <unordered_map>
#include <memory>
namespace nav {

	//前向声明
	struct Edge;
	struct Node;

	using NodeID = uint64_t;
	using EdgeID = uint64_t;
	
	//地理坐标
	struct Coordinate {
		double lat;//纬度
		double lon;//经度
		
		double distanceTo(const Coordinate& other) const;
	};

	//路网节点(交叉路口)
	struct Node{
		NodeID id;
		Coordinate coord;
		std::vector<Edge*> outgoing;//出边
		std::vector<Edge*> incoming;//入边

		Node(NodeID i,double la, double lo) : id(i), coord{la, lo} {}
	};

	//路网边（道路段）
	struct Edge{
		EdgeID id;
		Node* from;//起点
		Node* to;//终点
		double length;//长度(m)
		double speed_limit;//限速多少(km/h)
		bool oneway;//是否单行线

		//通行时间(s)
		double travelTime() const {return length / (speed_limit / 3.6); }

		//权重（用于路径算法，可扩展为动态权重）
		double weight() const {return travelTime();}
	};

	class RoadNetwork{
	public:
		RoadNetwork() = default;
		~RoadNetwork();

		//禁止拷贝（管理复杂内存）
		RoadNetwork(const RoadNetwork&) = delete;
		RoadNetwork& operator=(const RoadNetwork&) = delete;

		//构建接口
		Node* addNode(NodeID id, double lat, double lon);
		Edge* addEdge(EdgeID id, NodeID from, NodeID to, double length, double speed, bool oneway);


		//查询接口
		Node* getNode(NodeID id) const;
		//这块有个nodes的函数声明，但是cpp中没有实现它，是一个Bug我回头修改
		const std::unordered_map<NodeID, std::unique_ptr<Node>>& nodes() const;

		//获取邻居节点（用于遍历）
		std::vector<std::pair<Node*, double>> getNeighbors(Node* node) const;

		//统计信息
		size_t nodeCount() const { return nodes_.size(); }
		size_t edgeCount() const { return edges_.size(); }

	private:
		std::unordered_map<NodeID, std::unique_ptr<Node>> nodes_;
		std::unordered_map<EdgeID, std::unique_ptr<Edge>> edges_;
	};
}

