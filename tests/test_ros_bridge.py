from types import SimpleNamespace

import pytest

from web_server import ros_extended_node


class DummyPublisher:
    def __init__(self):
        self.messages = []

    def publish(self, msg):
        self.messages.append(msg)


class FakeRosAdapter:
    def __init__(self):
        self.subscriptions = {}
        self.publishers = {}

    def create_subscription(self, _msg_type, topic, callback, qos):
        self.subscriptions[topic] = callback
        return object()

    def create_publisher(self, _msg_type, topic, qos):
        publisher = DummyPublisher()
        self.publishers[topic] = publisher
        return publisher


def occupancy_grid(width=2, height=2, resolution=0.1, origin_x=0.0, origin_y=0.0, data=None):
    if data is None:
        data = [0] * (width * height)
    info = SimpleNamespace(
        width=width,
        height=height,
        resolution=resolution,
        origin=SimpleNamespace(position=SimpleNamespace(x=origin_x, y=origin_y)),
    )
    return SimpleNamespace(info=info, data=data)


def string_msg(data):
    return SimpleNamespace(data=data)


@pytest.fixture
def bridge():
    adapter = FakeRosAdapter()
    bridge = ros_extended_node.ExtendedRosBridge(ros_adapter=adapter)
    bridge._adapter = adapter
    return bridge


def test_map_subscription_updates_cache(bridge):
    grid = occupancy_grid(width=3, height=1, data=[0, 100, -1])
    callback = bridge._subscriptions["/map"]
    callback(grid)
    payload = bridge.get_map_json()
    assert payload["width"] == 3
    assert payload["height"] == 1
    assert payload["data"]


def test_costmap_subscription_returns_base64(bridge):
    grid = occupancy_grid(width=2, height=2, data=[0, 1, 2, 3])
    bridge._subscriptions["/global_costmap/costmap"](grid)
    payload = bridge.get_costmap_json("global")
    assert payload["width"] == 2
    assert payload["height"] == 2
    assert isinstance(payload["data"], str)


def test_publish_goal_uses_ros_publisher(bridge):
    bridge.publish_goal({"goal_x": 1.0, "goal_y": 2.0, "goal_theta": 0.0, "goal_id": 42})
    publisher = bridge._adapter.publishers["/goal"]
    assert publisher.messages
    msg = publisher.messages[-1]
    assert msg.pose.position.x == 1.0
    assert msg.pose.position.y == 2.0


def test_nav_status_callback(bridge):
    bridge._subscriptions["/nav_status"](string_msg("SUCCEEDED"))
    assert bridge.get_navigation_status()["status"] == "SUCCEEDED"




