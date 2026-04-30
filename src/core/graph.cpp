#include "core/graph.h"
#include <cmath>
#include <stdexcept>

namespace nav {

	double Coordinate::distanceTo(const Coordinate& other ) const {
		//简化版欧几里得距离（后续使用Haversine计算球面距离）
		double dx = lat - other.lat;
		double dy = lon - other.lon;
		return std::sqrt(dx * dx + dy * dy) *  111000;//粗略转换1度约等于111km
	}

	RoadNetwork::~RoadNetwork(){
		//unique_ptr自动释放Node和Edge
		//但是Edge内部的from和to是裸指针，无需手动释放，真正的Node对象由nodes_中的unique_ptr管理
		//所以不需要也不应该在Edge的析构里delete这些指针	
	}

	Node* RoadNetwork::addNode(NodeID id, double lat, double lon){
		//防重复检查，如果id已存在，抛出异常
		//count返回0或者1.存在返回1不存在返回0
		if (nodes_.count(id)){
			throw std::runtime_error("Duplicate node ID: " + std::to_string(id));
		}
		auto node =  std::make_unique<Node>(id, lat, lon);
		Node* raw = node.get();
		nodes_[id] =  std::move(node);
		return raw;
	}

	Edge* RoadNetwork::addEdge(EdgeID id, NodeID from_id, NodeID to_id, double length, double speed, bool oneway){
		if(edges_.count(id)){
			throw std::runtime_error("Duplicate edge ID: " + std::to_string(id));
		}

		auto from = getNode(from_id);
		auto to = getNode(to_id);
		if(!from || !to){
			throw std::runtime_error("Invalid node reference in edge");
		}

		//又开始了,创建一个边的对象的智能指针，指向一个Edge对象
		auto edge = std::make_unique<Edge>(Edge{id, from, to ,length, speed, oneway});
		//创建裸指针，借出对象的使用权
		Edge* raw = edge.get();
		//转移对象所有权到哈希表中对应的键值上，成为哈希表的值
		edges_[id] = std::move(edge);

		//建立拓扑关系
		from->outgoing.push_back(raw);
		to->incoming.push_back(raw);

		//双向道路添加反向边
		if(!oneway){
			return addEdge(id + 1000000, to_id, from_id, length, speed, true);
		}

		//返回这个可以调用对象的裸指针
		return raw;
	}

	Node* RoadNetwork::getNode(NodeID id) const{
		//创建一个迭代器返回的是哈希表中按照id查找的智能指针
		auto it = nodes_.find(id);
		//返回迭代器是不是没到nodes_end()？是的话返回智能指针get()得到的对象，不是返回空指针。
		return (it != nodes_.end()) ? it->second.get() : nullptr;
	}

	const std::unordered_map<NodeID, std::unique_ptr<Node>>& RoadNetwork::nodes() const{
		return nodes_;
	}
	
	std::vector<std::pair<Node*, double>> RoadNetwork::getNeighbors(Node* node) const{
		//创建一个空的vector容器
		std::vector<std::pair<Node*, double>> result;
		//for(auto it = node->outgoing.begin(); it!= node->outgoing.end(); ++it)
		//	Edge* e =  *it
		//这俩for是一样的
		for(Edge* e : node->outgoing){
			result.emplace_back(e->to, e->weight());
		}
		return result;
	}

}
