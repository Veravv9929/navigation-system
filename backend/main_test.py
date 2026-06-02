from fastapi import FastAPI, HTTPException
from models import (
        Coordinate, Algorithm, RouteRequest, RouteResponse, PathPoint, MatchRequest, MatchResponse, MatchCandidate
        )

app = FastAPI(
        title="Navigation API",
        version="1.0.0",
        description="Backend Service of Path Routing with three algorithms Dijkstra/Dijkstra Fast/A*"
        )

@app.get("/")
def root():
    return {
            "message": "Navigation API Server",
            "version": "1.0.0",
            "algorithms": ["dijkstra", "dijkstra_fast", "astar", "auto"]
            }

@app.get("/health")
def health():
    return {
            "status": "ok",
            "service": "navigation"
            }

@app.post("/api/route", response_model=RouteResponse)
def route(request: RouteRequest):
    """
    路径规划接口
    
    - **start**: 起点坐标
    - **end**: 终点坐标  
    - **algorithm**: 算法选择（auto/dijkstra/dijkstra_fast/astar）
    - **mode**: 出行模式（driving/walking/cycling）
    """
    # 占位：后续接入 C++ 引擎
    # 现在返回模拟数据，验证模型和文档
    
    return RouteResponse(
       status="ok",
       algorithm_used="astar",
       algorithm_requested=request.algorithm.value,
       compute_time_ms=23.5,
       nodes_explored=4200,
       edges_relaxed=12800,
       distance=12500.0,
       duration=900.0,
       hop_count=45,
       path=[
           PathPoint(lat=request.start.lat, lon=request.start.lon),
           PathPoint(lat=39.95, lon=116.35),
           PathPoint(lat=request.end.lat, lon=request.end.lon),
       ]
    )

@app.post("api/match, response_model=MatchResponse")
def match(request: MatchRequest):
    """
    地图匹配接口 - GPS坐标匹配到最近路网节点
    """
    # 占位：后续接入 C++ 引擎
    return MatchResponse(
        status="ok",
        query=Coordinate(lat=request.lat, lon=request.lon),
        candidates=[
            MatchCandidate(
                node_id=12345,
                lat=request.lat + 0.0001,
                lon=request.lon + 0.0001,
                distance_meters=15.3
            )
        ],
        count=1
    )


