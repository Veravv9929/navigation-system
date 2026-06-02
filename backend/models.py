from pydantic import BaseModel, Field
from typing import Literal, List, Optional
from enum import Enum

# ========================基础类型===================================
class Coordinate(BaseModel): #集成BaseModel成为Pydantic模型
    lat: float = Field(..., ge=-90, le=90, description = "latitude, range[-90,90]")
    lon: float = Field(..., ge=-180, le=180, description = "longitude, range[-180,180]")

# ========================枚举======================================
class Algorithm(str, Enum):
    AUTO = "auto"
    DIJKSTRA = "dijkstra"
    DIJKSTRA_FAST = "dijkstra_fast"
    ASTAR = "astar"
# =======================请求模型===================================
#路径规划请求
class RouteRequest(BaseModel):
    start: Coordinate
    end: Coordinate
    algorithm: Algorithm = Algorithm.AUTO
    mode: Literal["driving", "walking", "cycling"] = "driving"

#地图匹配请求（GPS坐标找最近路网节点）
class MatchRequest(BaseModel):
    lat: float = Field(..., ge=-90, le=90, description="find_latitude")
    lon: float = Field(..., ge=-180, le= 180,  description="find_longitude")
    k: int = Field(default=3, ge=1, le=20, description="return number of candidate nodes, default is 3, limit to 20")

#=========================响应模型============================
#路径上的点
class PathPoint(BaseModel):
    lat: float
    lon: float
    instruction: Optional[str] = None #导航指令（如“前方200米右转”）

#路径规划响应
class RouteResponse(BaseModel):
    status: str #ok或者no_path
    algorithm_used: str #实际使用的算法
    algorithm_requested: str #请求的算法
    compute_time_ms: float #计算耗时
    nodes_explored: int #探索节点数
    edges_relaxed: int #松弛的边数
    distance: float #路径距离（米）
    duration: float #预计时间（秒）
    hop_count: int #跳数（边数）
    path: List[PathPoint] #路径坐标序列

#地图匹配候选节点
class MatchCandidate(BaseModel):
    node_id: int
    lat: float
    lon: float
    distance_meters: float

#地图匹配响应
class MatchResponse(BaseModel):
    status: str #ok
    query: Coordinate #原始查询坐标
    candidates: List[MatchCandidate] #候选节点列表
    count: int #候选数量
