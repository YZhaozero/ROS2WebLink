import json
from pathlib import Path

import pytest

from web_server import route_manager


class FakeRosBridge:
    def __init__(self):
        self.goals = []

    def publish_goal(self, goal):
        self.goals.append(goal)


class FakeRobotSDK:
    def __init__(self):
        self.stair_mode = []

    def send_stair_mode(self, enable: bool) -> None:
        self.stair_mode.append(enable)


@pytest.fixture
def manager(tmp_path):
    data_dir = tmp_path / "data"
    data_dir.mkdir()
    mgr = route_manager.RouteManager(data_dir=data_dir)
    mgr._ros_bridge = FakeRosBridge()
    mgr._robot_sdk = FakeRobotSDK()
    return mgr


def test_create_route_persists_to_disk(manager):
    manager.create_route("test", [
        {"x": 1.0, "y": 2.0, "yaw": 0.0, "type": "normal"},
    ])
    data = json.loads(manager.data_file.read_text())
    assert data["routes"][0]["name"] == "test"


def test_execute_route_sends_goals_and_stair_mode(manager):
    route = manager.create_route(
        "patrol",
        [
            {"x": 0, "y": 0, "yaw": 0, "type": "normal"},
            {"x": 1, "y": 0, "yaw": 0, "type": "stair_enable"},
            {"x": 2, "y": 0, "yaw": 0, "type": "normal"},
            {"x": 3, "y": 0, "yaw": 0, "type": "stair_disable"},
        ],
    )

    manager.execute_route(route["id"])

    assert len(manager._ros_bridge.goals) == 4
    assert manager._robot_sdk.stair_mode == [True, False]
    assert manager.current_route() == route["id"]

