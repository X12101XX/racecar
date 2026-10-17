"""配置加载：JSON 配置文件 + 点号访问 + 默认值合并。"""

from __future__ import annotations

import copy
import json
import os
from typing import Any, Dict, Optional


def deep_merge(base: Dict[str, Any], override: Dict[str, Any]) -> Dict[str, Any]:
    """递归合并字典（override 优先），返回新字典。"""
    out = copy.deepcopy(base)
    for k, v in (override or {}).items():
        if isinstance(v, dict) and isinstance(out.get(k), dict):
            out[k] = deep_merge(out[k], v)
        else:
            out[k] = copy.deepcopy(v)
    return out


def load_json(path: str) -> Dict[str, Any]:
    with open(path, "r", encoding="utf-8") as f:
        return json.load(f)


class Config:
    """薄封装：支持 cfg.get("a.b.c", default) 点号取值。"""

    def __init__(self, data: Optional[Dict[str, Any]] = None,
                 path: Optional[str] = None):
        self.data: Dict[str, Any] = data or {}
        self.path = path

    @classmethod
    def load(cls, path: str, base: Optional[Dict[str, Any]] = None) -> "Config":
        data = load_json(path)
        if base:
            data = deep_merge(base, data)
        return cls(data, path)

    def get(self, dotted: str, default: Any = None) -> Any:
        cur: Any = self.data
        for key in dotted.split("."):
            if not isinstance(cur, dict) or key not in cur:
                return default
            cur = cur[key]
        return cur

    def section(self, dotted: str) -> "Config":
        return Config(self.get(dotted, {}) or {})

    def __repr__(self) -> str:  # pragma: no cover
        return f"Config({self.path or 'inline'})"


def project_root() -> str:
    """本仓库根目录（racelib 的上一级）。"""
    return os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def default_config_dir() -> str:
    return os.path.join(project_root(), "config")


REQUIRED_FILES = ("field.json", "robot.json", "vision.json", "plan.json")


def load_all(config_dir: Optional[str] = None,
             route: Optional[str] = None) -> Dict[str, Config]:
    """加载 field / robot / vision / plan / route_x 配置。"""
    d = config_dir or default_config_dir()
    out: Dict[str, Config] = {}
    for name in REQUIRED_FILES:
        out[name.split(".")[0]] = Config.load(os.path.join(d, name))
    if route:
        name = f"route_{str(route).strip().lower()}.json"
        out["route"] = Config.load(os.path.join(d, name))
    return out
