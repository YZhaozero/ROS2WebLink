import base64
import pytest


class DummyRobotSDK:
    def __init__(self) -> None:
        self.robot_info = {"battery": 88, "status": "IDLE"}
        self.twist_commands = []
        self.modes = []
        self.stair_modes = []
        self.started = False

    def start(self) -> None:
        self.started = True

    def send_twist(self, x: float, y: float, z: float) -> None:
        self.twist_commands.append((x, y, z))

    def send_stand_mode(self) -> None:
        self.modes.append("stand")

    def send_walk_mode(self) -> None:
        self.modes.append("walk")

    def send_sit_mode(self) -> None:
        self.modes.append("sit")

    def send_stair_mode(self, enable: bool) -> None:
        self.stair_modes.append(enable)

    def send_emergency_stop(self) -> None:
        self.modes.append("emg_stop")


class DummyRosBridge:
    def __init__(self) -> None:
        data = base64.b64encode(bytes([0, 100, 0, 0])).decode("ascii")
        self._payload = {
            "width": 2,
            "height": 2,
            "resolution": 0.05,
            "origin_x": 0.0,
            "origin_y": 0.0,
            "data": data,
        }
        self.published_goals = []

    def get_map_json(self):
        return self._payload

    def get_costmap_json(self, _kind: str):
        return self._payload

    def get_navigation_status(self):
        return {"status": "IDLE"}

    def publish_goal(self, goal):
        self.published_goals.append(goal)


class DummyMappingController:
    def __init__(self) -> None:
        self.started = False
        self.start_calls = []
        self.stop_calls = []

    def start(self, map_name=None):
        self.started = True
        self.start_calls.append(map_name)
        return {"status": "RUNNING", "pids": [1, 2]}

    def stop(self, *, save=True, map_name=None):
        self.started = False
        self.stop_calls.append({"save": save, "map_name": map_name})
        return {"status": "IDLE", "map_saved": save}

    def status(self):
        return {"status": "RUNNING" if self.started else "IDLE"}


class DummyRouteManager:
    def __init__(self) -> None:
        self.routes = []
        self.executed = []

    def create_route(self, name, waypoints):
        route = {"id": f"route-{len(self.routes) + 1}", "name": name, "waypoints": list(waypoints)}
        self.routes.append(route)
        return route

    def list_routes(self):
        return list(self.routes)

    def delete_route(self, route_id: str):
        before = len(self.routes)
        self.routes = [route for route in self.routes if route["id"] != route_id]
        return len(self.routes) != before

    def execute_route(self, route_id: str):
        if not any(route["id"] == route_id for route in self.routes):
            raise KeyError(route_id)
        self.executed.append(route_id)


@pytest.fixture
def test_client(monkeypatch):
    """Provide a FastAPI TestClient instance for API tests with stubbed services."""
    from fastapi.testclient import TestClient
    from web_server import mapping_nav_server

    robot = DummyRobotSDK()
    ros = DummyRosBridge()
    mapping = DummyMappingController()
    routes = DummyRouteManager()

    monkeypatch.setattr(mapping_nav_server, "_robot_sdk", robot)
    monkeypatch.setattr(mapping_nav_server, "_ros_bridge", ros)
    monkeypatch.setattr(mapping_nav_server, "_mapping_controller", mapping)
    monkeypatch.setattr(mapping_nav_server, "_route_manager", routes)

    app = mapping_nav_server.create_app()
    app.state.test_robot = robot
    app.state.test_mapping = mapping
    app.state.test_routes = routes
    app.state.test_ros = ros
    return TestClient(app)

