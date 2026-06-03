from fastapi import FastAPI, HTTPException, Request
from fastapi.staticfiles import StaticFiles
from fastapi.responses import FileResponse
from fastapi.responses import JSONResponse
from models import (
    RouteRequest, RouteResponse, MatchRequest, MatchResponse,
    PathPoint, MatchCandidate, Coordinate, Algorithm
)
from cpp_bridge import CppEngine
import os
import time
import logging

# ========== 路径配置 ==========
BACKEND_DIR = os.path.dirname(os.path.abspath(__file__))
PROJECT_ROOT = os.path.dirname(BACKEND_DIR)
ENGINE_PATH = os.path.join(PROJECT_ROOT, "build", "nav_engine")
OSM_FILE = os.path.join(PROJECT_ROOT, "data", "beijing_large.osm")

# ========== 日志配置 ==========
os.makedirs("logs", exist_ok=True)
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(levelname)s - %(message)s',
    handlers=[
        logging.FileHandler('logs/navigation.log'),
        logging.StreamHandler()
    ]
)
logger = logging.getLogger(__name__)

# ========== FastAPI 应用 ==========
app = FastAPI(
    title="Navigation API",
    version="1.0.0",
    description="Backend Service of Path Routing with C++ Engine"
)

#静态文件：前端页面
app.mount("/static", StaticFiles(directory="../frontend"), name="static")

@app.get("/")
def serve_frontend():
    """根路径返回前端页面"""
    return FileResponse("../frontend/index.html")

# ========== 请求日志中间件 ==========
@app.middleware("http")
async def log_requests(request: Request, call_next):
    start_time = time.time()
    response = await call_next(request)
    duration = (time.time() - start_time) * 1000
    
    logger.info(
        f"{request.method} {request.url.path} "
        f"- {response.status_code} "
        f"- {duration:.2f}ms"
    )
    return response

# ========== 全局异常处理 ==========
@app.exception_handler(Exception)
async def global_exception_handler(request: Request, exc: Exception):
    logger.error(f"Unhandled exception: {str(exc)}", exc_info=True)
    return JSONResponse(
        status_code=500,
        content={"error": "internal_error", "detail": str(exc)}
    )

# ========== 全局引擎实例 ==========
engine = None

@app.on_event("startup")
def startup():
    global engine
    logger.info("Starting Navigation API server...")
    logger.info(f"Engine path: {ENGINE_PATH}")
    logger.info(f"Map file: {OSM_FILE}")
    
    if not os.path.exists(ENGINE_PATH):
        logger.error(f"Engine not found: {ENGINE_PATH}")
        return
    
    if not os.path.exists(OSM_FILE):
        logger.error(f"Map file not found: {OSM_FILE}")
        return
    
    try:
        engine = CppEngine(OSM_FILE, ENGINE_PATH)
        logger.info(f"Engine ready: {engine.map_info['nodes']} nodes, {engine.map_info['edges']} edges")
    except Exception as e:
        logger.error(f"Failed to init engine: {e}")
        engine = None

@app.on_event("shutdown")
def shutdown():
    logger.info("Shutting down Navigation API server...")

@app.get("/api/info")
def root():
    return {
        "message": "Navigation API Server",
        "version": "1.0.0",
        "algorithms": ["dijkstra", "dijkstra_fast", "astar", "auto"],
        "engine_loaded": engine is not None
    }

@app.get("/health")
def health():
    if engine is None:
        raise HTTPException(status_code=503, detail="Engine not loaded")
    return {
        "status": "ok",
        "map_nodes": engine.map_info["nodes"],
        "map_edges": engine.map_info["edges"]
    }

@app.post("/api/route", response_model=RouteResponse)
def route(request: RouteRequest):
    if engine is None:
        raise HTTPException(status_code=503, detail="Engine not ready")
    
    logger.info(
        f"Route request: ({request.start.lat},{request.start.lon}) -> "
        f"({request.end.lat},{request.end.lon}), algo={request.algorithm.value}"
    )
    
    result = engine.route(
        start={"lat": request.start.lat, "lon": request.start.lon},
        end={"lat": request.end.lat, "lon": request.end.lon},
        algorithm=request.algorithm.value
    )
    
    if "error" in result:
        logger.warning(f"Route error: {result['error']}")
        raise HTTPException(status_code=400, detail=result["error"])
    
    if result.get("status") == "no_path":
        raise HTTPException(status_code=404, detail="No path found")
    
    path_points = [
        PathPoint(lat=p["lat"], lon=p["lon"]) 
        for p in result.get("path", [])
    ]
    
    logger.info(
        f"Route success: {result.get('hop_count', 0)} hops, "
        f"{result.get('compute_time_ms', 0)}ms"
    )
    
    return RouteResponse(
        status="ok",
        algorithm_used=result.get("algorithm_used", "unknown"),
        algorithm_requested=result.get("algorithm_request", request.algorithm.value),
        compute_time_ms=result.get("compute_time_ms", 0.0),
        nodes_explored=result.get("nodes_explored", 0),
        edges_relaxed=result.get("edges_relaxed", 0),
        distance=result.get("distance", 0.0),
        duration=result.get("duration", 0.0),
        hop_count=result.get("hop_count", 0),
        path=path_points
    )

@app.post("/api/match", response_model=MatchResponse)
def match(request: MatchRequest):
    if engine is None:
        raise HTTPException(status_code=503, detail="Engine not ready")
    
    logger.info(f"Match request: ({request.lat},{request.lon}), k={request.k}")
    
    result = engine.match(lat=request.lat, lon=request.lon, k=request.k)
    
    if "error" in result:
        logger.warning(f"Match error: {result['error']}")
        raise HTTPException(status_code=400, detail=result["error"])
    
    candidates = [
        MatchCandidate(
            node_id=c["node_id"],
            lat=c["lat"],
            lon=c["lon"],
            distance_meters=c["distance_meters"]
        )
        for c in result.get("candidates", [])
    ]
    
    logger.info(f"Match success: {len(candidates)} candidates")
    
    return MatchResponse(
        status="ok",
        query=Coordinate(lat=request.lat, lon=request.lon),
        candidates=candidates,
        count=len(candidates)
    )

@app.post("/api/route/compare")
def route_compare(request: RouteRequest):
    if engine is None:
        raise HTTPException(status_code=503, detail="Engine not ready")
    
    logger.info(
        f"Compare request: ({request.start.lat},{request.start.lon}) -> "
        f"({request.end.lat},{request.end.lon})"
    )
    
    algorithms = ["dijkstra", "dijkstra_fast", "astar"]
    results = []
    
    for algo in algorithms:
        start_time = time.time()
        result = engine.route(
            start={"lat": request.start.lat, "lon": request.start.lon},
            end={"lat": request.end.lat, "lon": request.end.lon},
            algorithm=algo
        )
        end_time = time.time()
        
        python_overhead = (end_time - start_time) * 1000 - result.get("compute_time_ms", 0)
        python_overhead = max(0, python_overhead)  # 防止负数
        
        results.append({
            "algorithm": algo,
            "status": result.get("status", "error"),
            "compute_time_ms": result.get("compute_time_ms", 0),
            "python_overhead_ms": round(python_overhead, 3),
            "nodes_explored": result.get("nodes_explored", 0),
            "edges_relaxed": result.get("edges_relaxed", 0),
            "distance": result.get("distance", 0),
            "duration": result.get("duration", 0),
            "hop_count": result.get("hop_count", 0)
        })
    
    logger.info(f"Compare complete: {len([r for r in results if r['status'] == 'ok'])} algorithms succeeded")
    
    return {
        "status": "ok",
        "start": {"lat": request.start.lat, "lon": request.start.lon},
        "end": {"lat": request.end.lat, "lon": request.end.lon},
        "comparisons": results
    }

@app.get("/api/algorithms")
def algorithms():
    if engine is None:
        raise HTTPException(status_code=503, detail="Engine not ready")
    
    result = engine.algorithms()
    return {
        "algorithms": result.get("algorithms", []),
        "map_nodes": result.get("map_nodes", 0),
        "map_edges": result.get("map_edges", 0)
    }
