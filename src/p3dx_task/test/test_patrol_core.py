import pytest

from p3dx_task.patrol_core import PatrolStatus, next_index


def test_next_index_loop():
    assert next_index(0, 3, True) == 1
    assert next_index(2, 3, True) == 0  # 最后一点回 0


def test_next_index_no_loop():
    assert next_index(0, 3, False) == 1
    assert next_index(2, 3, False) is None  # 走完结束


def test_next_index_invalid():
    with pytest.raises(ValueError):
        next_index(0, 0, True)
    with pytest.raises(ValueError):
        next_index(3, 3, True)
    with pytest.raises(ValueError):
        next_index(-1, 3, True)


def test_patrol_status_values():
    assert PatrolStatus.NAVIGATING.value == "navigating"
    assert PatrolStatus.ARRIVED.value == "arrived"
