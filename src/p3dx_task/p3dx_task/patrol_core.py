"""巡逻状态机核心（纯逻辑，可单元测试）。"""
from enum import Enum


class PatrolStatus(Enum):
    IDLE = "idle"
    NAVIGATING = "navigating"
    ARRIVED = "arrived"
    FAILED = "failed"


def next_index(current: int, total: int, loop: bool):
    """返回下一个巡逻点索引。

    current: 当前索引（0-based）
    total: 巡逻点总数
    loop: 是否循环
    非循环模式下走完最后一点返回 None；循环模式下最后一点回 0。
    """
    if total <= 0:
        raise ValueError("total 必须 > 0")
    if not (0 <= current < total):
        raise ValueError("current 越界")
    nxt = current + 1
    if nxt < total:
        return nxt
    return 0 if loop else None
