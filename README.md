# Navigation System — 基于真实路网的高性能路径规划引擎

基于 OpenStreetMap 真实路网数据（北京）的 C++17 路径规划引擎，实现 Dijkstra / A* / Contraction Hierarchies 最短路算法，并提供 CLI、C++ HTTP 服务与 FastAPI Web 层三种接入方式。个人项目 / 毕业设计。

## 特性

- **真实路网数据**：解析 OSM 原始数据（tinyxml2），建模单行道、转向限制等交通规则，将 38.7 万节点 / 70.9 万边的真实路网构建为内存有向图（清洗前 184 万节点）
- **路径规划算法**：
  - Dijkstra：二叉堆优化优先队列，单次查询 O((V+E)logV)
  - A\*：基于直线距离 / 最高限速的可采纳（admissible）启发函数，较 Dijkstra 平均减少 57% 节点扩展，单次查询耗时 124.9ms → 61.9ms
  - Contraction Hierarchies：节点收缩 + shortcut 预处理 + 双向向上查询（开发验证中）
- **空间索引**：GridIndex 网格索引，经纬度 O(1) 级定位最近道路节点
- **多接入方式**：
  - `nav_cli`：命令行交互
  - `nav_server`：基于 mongoose 的 C++ 原生 HTTP 服务
  - `backend/`：FastAPI Web 层，通过桥接模块调用 C++ 计算核心，算法引擎与 Web 层解耦
- **工程化**：CMake 构建、GoogleTest 单元测试与 Benchmark、CTest 集成

## 性能数据（GoogleTest Benchmark）

| 指标 | Dijkstra | A* |
|---|---|---|
| 平均查询耗时 | 1.17 ms | 0.86 ms（1.37x 加速） |
| 平均扩展节点数 | 6820 | 4864（-28.7%） |

> 测试集：真实路网 13912 节点 / 25675 边，100 组随机点对。完整城市级路网（38.7 万节点）下 A* 单次查询 61.9ms。

## 项目结构

```
├── include/core/            # 核心头文件
│   ├── graph.h                  # RoadNetwork：节点/边存储、连续索引
│   ├── routing.h                # Dijkstra / A*
│   ├── contraction_hierarchies.h# CH 预处理与查询
│   ├── spatial_index.h          # GridIndex 空间索引
│   ├── map_loader.h             # OSM 数据解析
│   └── algorithm_router.h       # 算法统一调度
├── src/
│   ├── core/                # 核心算法实现
│   ├── cli/                 # nav_cli / nav_engine（JSON 引擎）
│   └── server/              # nav_server（mongoose HTTP 服务）
├── backend/                 # FastAPI Web 层（接口文档见 backend/API.md）
├── frontend/                # 前端展示
├── tests/                   # GoogleTest：单元测试 + Benchmark + CH 测试
├── third_party/             # mongoose、nlohmann/json
└── data/                    # 路网数据（Beijing OSM）
```

## 构建与运行

依赖：CMake ≥ 3.14、支持 C++17 的编译器、GoogleTest、tinyxml2；Web 层需 Python 3.x。

```bash
# 构建
mkdir build && cd build
cmake .. && make -j$(nproc)

# 运行测试（含 Benchmark）
ctest --output-on-failure

# CLI
./nav_cli

# C++ HTTP 服务
./nav_server
```

```bash
# FastAPI Web 层
cd backend
pip install -r requirements.txt
python main.py
```

## HTTP API（详见 backend/API.md）

| 方法 | 路径 | 说明 |
|---|---|---|
| GET | `/health` | 健康检查 |
| GET | `/api/algorithms` | 支持的算法列表 |
| POST | `/api/route` | 路径规划（dijkstra / dijkstra_fast / astar / auto） |
| POST | `/api/route/compare` | 多算法对比 |
| POST | `/api/match` | 地图匹配（GPS 坐标找最近节点） |

## 路线图

- [x] OSM 数据解析与路网建模
- [x] Dijkstra / A* + GridIndex 空间索引
- [x] FastAPI Web 层 + C++ mongoose HTTP 服务
- [ ] Contraction Hierarchies 正确性回归（1000 组随机点对 vs Dijkstra）
- [ ] CH benchmark：三算法查询耗时 / 预处理时间 / 内存对比
- [ ] 服务端性能优化：线程池 + LRU 热点缓存 + wrk 压测
