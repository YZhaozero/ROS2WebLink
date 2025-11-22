import pytest


def test_healthz_endpoint_returns_ok(test_client):
    response = test_client.get("/healthz")
    assert response.status_code == 200
    assert response.json() == {"status": "ok"}


def test_root_serves_index_html(test_client):
    response = test_client.get("/")
    assert response.status_code == 200
    assert "text/html" in response.headers["content-type"].lower()


@pytest.mark.parametrize("origin", ["http://example.com", "https://localhost:3000"])
def test_cors_headers_present(test_client, origin):
    response = test_client.options(
        "/healthz",
        headers={
            "origin": origin,
            "access-control-request-method": "GET",
        },
    )
    assert response.status_code == 200
    assert response.headers.get("access-control-allow-origin") == "*"


def test_mapping_start_stop_and_status(test_client):
    resp = test_client.post("/api/mapping/start", json={"map_name": "demo"})
    assert resp.status_code == 200
    assert resp.json()["status"] == "RUNNING"

    resp = test_client.get("/api/mapping/status")
    assert resp.json()["status"] == "RUNNING"

    resp = test_client.post("/api/mapping/stop", json={"save": True, "map_name": "demo"})
    assert resp.status_code == 200
    assert resp.json()["status"] == "IDLE"

    mapping = test_client.app.state.test_mapping
    assert mapping.start_calls[-1] == "demo"
    assert mapping.stop_calls[-1]["map_name"] == "demo"


def test_robot_map_and_costmap(test_client):
    resp = test_client.get("/api/robot/map")
    payload = resp.json()
    assert payload["width"] == 2

    resp = test_client.get("/api/costmap/local")
    payload = resp.json()
    assert payload["height"] == 2


def test_robot_twist_and_modes(test_client):
    resp = test_client.post("/api/robot/twist", json={"vel_x": 0.5, "vel_y": 0.1, "vel_theta": -0.3})
    assert resp.status_code == 200
    robot = test_client.app.state.test_robot
    assert robot.twist_commands[-1] == (0.5, 0.1, -0.3)

    for path in [
        "/api/robot/mode/stand",
        "/api/robot/mode/walk",
        "/api/robot/mode/sit",
    ]:
        assert test_client.post(path).status_code == 200

    assert test_client.post("/api/robot/mode/stair", json={"enable": True}).status_code == 200
    assert test_client.post("/api/robot/emergency_stop").status_code == 200
    assert robot.stair_modes[-1] is True
    assert "emg_stop" in robot.modes


def test_route_lifecycle(test_client):
    resp = test_client.post(
        "/api/routes",
        json={
            "name": "patrol",
            "waypoints": [
                {"x": 1.0, "y": 2.0, "yaw": 0.0, "type": "normal"},
                {"x": 2.0, "y": 3.0, "yaw": 0.0, "type": "stair_enable"},
            ],
        },
    )
    route = resp.json()
    assert route["name"] == "patrol"

    resp = test_client.get("/api/routes")
    assert resp.json()["routes"][0]["id"] == route["id"]

    assert test_client.post(f"/api/routes/{route['id']}/execute").status_code == 200
    routes_stub = test_client.app.state.test_routes
    assert routes_stub.executed[-1] == route["id"]

    assert test_client.delete(f"/api/routes/{route['id']}").status_code == 200

