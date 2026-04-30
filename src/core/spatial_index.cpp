#include "core/spatial_index.h"
#include <algorithm>
#include <queue>
#include <cmath>

namespace nav {

	GridIndex::GridIndex(double cell_size) : cell_size_(cell_size){}

	void GridIndex::build(const RoadNetwork& network) {
		grid_.clear();
		for (const auto& [id, node_ptr] : network.nodes()) {
			Node* node = node_ptr.get();
			auto cell = getCell(node->coord.lat, node->coord.lon);
			grid_[cell].push_back(node);
		}
	}

	std::vector<Node*> GridIndex::nearest(const Coordinate& coord, int k) const {
		auto center_cell = getCell(coord.lat, coord.lon);

		//优先队列：按距离排序（小顶堆）
		using QueueItem = std::pair<double, Node*>;
		std::priority_queue<QueueItem, std::vector<QueueItem>, std::greater<>> candidates;
		
		for(int dx = -3; dx <=3; ++dx){
                       for(int dy = -3; dy <= 3; ++dy){
                                auto cell = std::make_pair(center_cell.first + dx, center_cell.second + dy);
                                auto it = grid_.find(cell);
                                if (it == grid_.end()) continue;

                                for(Node* node : it->second){
                                       double dist = coord.distanceTo(node->coord); 
                                       candidates.push({dist, node});
                                }
                        }
                }

                //取最近的k个（从优先队列(priority_queue)candidates中）
                std::vector<Node*> result;
                while (!candidates.empty() && result.size() < static_cast<size_t>(k)) {
                       result.push_back(candidates.top().second);
                       candidates.pop();
                }
	        return result;	
	}

	std::pair<int, int> GridIndex::getCell(double lat, double lon) const {
		return {
			static_cast<int>(std::floor(lat / cell_size_)),
			static_cast<int>(std::floor(lon / cell_size_))
		};	
	}

        std::vector<Node*> GridIndex::rangeQuery(double min_lat, double min_lon, double max_lat, double max_lon) const {
                std::vector<Node*> result;
                auto min_cell = getCell(min_lat, min_lon);
                auto max_cell = getCell(max_lat, max_lon);

                for(int x = min_cell.first; x <= max_cell.first; ++x){
                        for(int y = min_cell.second; y <= max_cell.second; ++y){
                                auto it = grid_.find({x, y});
                                if (it == grid_.end()) continue;
                                for (Node* node : it -> second){
                                    if (node->coord.lat >= min_lat && node->coord.lat <= max_lat && node->coord.lon >= min_lon && node->coord.lon <= max_lon){
                                        result.push_back(node);
                                    }
                                }
                        }
                }
                return result;
        } 

}
