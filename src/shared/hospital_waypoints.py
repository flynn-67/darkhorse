import os
import time
import yaml
from typing import Dict, Any, Optional

DEFAULT_PATH = os.path.expanduser("~/.ros/hospital_waypoints.yaml")

_cache = {
    "path": None,
    "mtime": None,
    "data": None,
}

def _load_yaml(path: str) -> Dict[str, Any]:
    if not os.path.exists(path):
        return {"departments": {}, "meta": {}}
    with open(path, "r", encoding="utf-8") as f:
        data = yaml.safe_load(f) or {}
    data.setdefault("departments", {})
    data.setdefault("meta", {})
    return data

def get_waypoints(path: str = DEFAULT_PATH) -> Dict[str, Any]:
    """파일이 바뀌면 자동으로 다시 읽어서 최신 dict 반환"""
    global _cache
    mtime = os.path.getmtime(path) if os.path.exists(path) else None

    if _cache["path"] != path or _cache["mtime"] != mtime or _cache["data"] is None:
        _cache["path"] = path
        _cache["mtime"] = mtime
        _cache["data"] = _load_yaml(path)

    return _cache["data"]

def get_department(path: str, name: str) -> Optional[Dict[str, float]]:
    data = get_waypoints(path)
    dept = data.get("departments", {}).get(name)
    if not dept:
        return None
    # 필드 보정
    x = float(dept.get("x", 0.0))
    y = float(dept.get("y", 0.0))
    yaw = float(dept.get("yaw", 0.0))
    return {"x": x, "y": y, "yaw": yaw}

def save_waypoints(path: str, departments: Dict[str, Dict[str, float]]) -> None:
    os.makedirs(os.path.dirname(path), exist_ok=True)
    data = {
        "departments": departments,
        "meta": {"updated_at": time.strftime("%Y-%m-%dT%H:%M:%S%z")},
    }
    with open(path, "w", encoding="utf-8") as f:
        yaml.safe_dump(data, f, allow_unicode=True, sort_keys=False)
