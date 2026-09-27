"""巡逻点加载与几何工具（纯逻辑，可单元测试）。"""
import math
from dataclasses import dataclass

import yaml


@dataclass(frozen=True)
class Waypoint:
    name: str
    x: float
    y: float
    yaw: float = 0.0  # 弧度


def load_waypoints(path: str) -> list[Waypoint]:
    """从 yaml 读取巡逻点列表。

    文件不存在抛 OSError；yaml 缺少 waypoints 字段或为 null 抛 ValueError。
    """
    with open(path, "r", encoding="utf-8") as f:
        data = yaml.safe_load(f)
    pts = data.get("waypoints") if isinstance(data, dict) else None
    if pts is None:
        raise ValueError("yaml 缺少 waypoints 字段或为 null")
    return [
        Waypoint(
            name=str(w["name"]),
            x=float(w["x"]),
            y=float(w["y"]),
            yaw=float(w.get("yaw", 0.0)),
        )
        for w in pts
    ]


def load_loop(path: str) -> bool:
    """读取是否循环巡逻（默认 True）。"""
    with open(path, "r", encoding="utf-8") as f:
        data = yaml.safe_load(f)
    return bool(data.get("loop", True))


def yaw_to_quaternion(yaw: float) -> tuple:
    """绕 z 轴旋转 yaw 弧度的四元数，返回 (x, y, z, w)。"""
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))
