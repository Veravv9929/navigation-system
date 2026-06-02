vigation API 文档

## 服务信息

| 项目 | 内容 |
|------|------|
| 版本 | 1.0.0 |
| 引擎 | C++ 导航引擎（Dijkstra / DijkstraFast / A* / Contraction Hierarchies） |
| 地图 | Beijing Large OSM |

---

## 接口列表

| 方法 | 路径 | 说明 |
|------|------|------|
| GET | `/` | 服务信息 |
| GET | `/health` | 健康检查 |
| GET | `/api/algorithms` | 支持的算法列表 |
| POST | `/api/route` | 路径规划 |
| POST | `/api/route/compare` | 三种算法对比 |
| POST | `/api/match` | 地图匹配（GPS 找最近节点） |

---

## 详细接口

### GET /

返回服务基本信息。

**响应：**
```json
{
    "message": "Navigation API Server",
        "version": "1.0.0",
            "algorithms": ["dijkstra", "dijkstra_fast", "astar", "auto"],
                "engine_loaded": true
                }
