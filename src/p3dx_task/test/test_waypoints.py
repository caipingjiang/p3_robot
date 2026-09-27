import math
import os
import tempfile

import pytest

from p3dx_task.waypoints import Waypoint, load_loop, load_waypoints, yaw_to_quaternion


def test_yaw_to_quaternion_zero():
    assert yaw_to_quaternion(0.0) == (0.0, 0.0, 0.0, 1.0)


def test_yaw_to_quaternion_pi_over_2():
    q = yaw_to_quaternion(math.pi / 2)
    assert q[2] == pytest.approx(math.sin(math.pi / 4))
    assert q[3] == pytest.approx(math.cos(math.pi / 4))


def test_load_waypoints_and_loop():
    content = """
loop: true
waypoints:
  - name: A
    x: 1.0
    y: 2.0
    yaw: 0.0
  - name: B
    x: 3.0
    y: 4.0
"""
    with tempfile.NamedTemporaryFile("w", suffix=".yaml", delete=False) as f:
        f.write(content)
        path = f.name
    try:
        wps = load_waypoints(path)
        assert len(wps) == 2
        assert wps[0] == Waypoint("A", 1.0, 2.0, 0.0)
        assert wps[1].yaw == 0.0  # 缺省 yaw 取默认值
        assert load_loop(path) is True
    finally:
        os.unlink(path)


def test_load_waypoints_default_loop():
    content = "waypoints:\n  - {name: A, x: 0.0, y: 0.0}\n"
    with tempfile.NamedTemporaryFile("w", suffix=".yaml", delete=False) as f:
        f.write(content)
        path = f.name
    try:
        assert load_loop(path) is True  # 缺省 loop 默认 True
    finally:
        os.unlink(path)


def test_load_waypoints_missing_file():
    # 文件不存在 → 抛 OSError（patrol_node 据此捕获并报清晰日志）
    with pytest.raises(OSError):
        load_waypoints("/nonexistent/patrol_waypoints.yaml")


def test_load_waypoints_missing_key():
    # yaml 缺少 waypoints 字段 → 抛清晰 ValueError，而非裸 KeyError
    content = "loop: true\n"
    with tempfile.NamedTemporaryFile("w", suffix=".yaml", delete=False) as f:
        f.write(content)
        path = f.name
    try:
        with pytest.raises(ValueError):
            load_waypoints(path)
    finally:
        os.unlink(path)


def test_load_waypoints_null_list():
    # waypoints 为 null → 抛清晰 ValueError，而非裸 TypeError
    content = "loop: true\nwaypoints:\n"
    with tempfile.NamedTemporaryFile("w", suffix=".yaml", delete=False) as f:
        f.write(content)
        path = f.name
    try:
        with pytest.raises(ValueError):
            load_waypoints(path)
    finally:
        os.unlink(path)


def test_load_loop_explicit_false():
    # 显式 loop: false 应返回 False（YAML 解析为布尔，非字符串）
    content = "loop: false\nwaypoints:\n  - {name: A, x: 0.0, y: 0.0}\n"
    with tempfile.NamedTemporaryFile("w", suffix=".yaml", delete=False) as f:
        f.write(content)
        path = f.name
    try:
        assert load_loop(path) is False
    finally:
        os.unlink(path)
