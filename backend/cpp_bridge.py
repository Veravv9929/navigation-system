import subprocess
import json
import os
from typing import Dict, Any
class CppEngine:
    """封装C++导航引擎， 通过 subprocess 调用"""

    def __init__(self, osm_file: str, engine_path: str = "./nav_engine"):
        self.engine_path = engine_path
        self.osm_file = osm_file

        #验证引擎存在
        if not os.path.exists(engine_path):
            raise FileNotFoundError(f"C++ engine not found: {engine_path}")

        #验证地图可加载
        result = self._call({"cmd": "algorithms"})
        if "error" in result:
            raise RuntimeError(f"Failed to init engine: {result['error']}")

        self.map_info = {
                "nodes": result.get("map_nodes", 0),
                "edges": result.get("map_edges", 0)

                }

    def _call(self, request: Dict[str, Any]) -> Dict[str, Any]:
        """调用C++引擎，发送JSON，接受JSON"""
        proc = subprocess.run(
                [self.engine_path, self.osm_file],
                input=json.dumps(request),
                capture_output=True,
                text=True,
                timeout=30#超时30秒
                )

        if proc.returncode != 0:
            raise RuntimeError(f"Engine error: {proc.stderr}")

        return json.loads(proc.stdout)

    def route(self, start: Dict[str, float], end: Dict[str, float], algorithm: str = "auto") -> Dict[str, Any]:
        """路径规划"""
        return self._call({
            "cmd": "route",
            "start": start,
            "end": end,
            "algorithm": algorithm
            })

    def match(self, lat: float, lon: float, k: int = 3) -> Dict[str, Any]:
        """地图匹配"""
        return self._call({
            "cmd": "match",
            "lat": lat,
            "lon": lon,
            "k": k
            })

    def algorithms(self) -> Dict[str, Any]:
        """获取支持的算法列表"""
        return self._call({"cmd": "algorithms"})
