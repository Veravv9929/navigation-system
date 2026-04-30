#pragma once
#include "core/graph.h"
#include <vector>
#include <map>

namespace nav {

	//简化版网格空间索引
	//将地图划分为玩过，每个网格存储其中的节点
	class GridIndex {
	public:
		//cell_size:网络边长（度），大约0.001度约等于111米
		explicit GridIndex(double cell_size = 0.001);

		//构建索引
		//遍历路网所有节点，按照坐标放入对应网络
		void build(const RoadNetwork& network);

		//查询最近邻（k个最近节点）
		std::vector<Node*> nearest(const Coordinate& coord, int k = 1 ) const;

		//查询矩阵范围内的节点
		std::vector<Node*> rangeQuery(double min_lat, double min_lon, double max_lat, double max_lon) const;
	
	private:
		//坐标->网格ID
		//把经纬度映射岛网格坐标（x,y）
		std::pair<int, int> getCell(double lat, double lon) const; 

		double cell_size_;
		std::map<std::pair<int, int>, std::vector<Node*>> grid_;
	};
}
