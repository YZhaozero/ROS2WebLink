import json
import uuid
from dataclasses import dataclass

import pytest

from web_server import robot_sdk_bridge


@dataclass
class DummyMessage:
    data: str


class DummyWebSocketApp:
    instances = []

    def __init__(self, url, on_open=None, on_message=None, on_close=None, on_error=None):
        self.url = url
        self.on_open = on_open
        self.on_message = on_message
        self.on_close = on_close
        self.on_error = on_error
        self.sent_messages = []
        self.closed = False
        DummyWebSocketApp.instances.append(self)

    def run_forever(self, *_, **__):
        if self.on_open:
            self.on_open(self)

    def send(self, payload):
        self.sent_messages.append(payload)

    def close(self):
        self.closed = True


def uuid_sequence():
    while True:
        yield uuid.UUID(int=0)


@pytest.fixture(autouse=True)
def patch_websocket(monkeypatch):
    DummyWebSocketApp.instances.clear()
    monkeypatch.setattr(robot_sdk_bridge, "WebSocketApp", DummyWebSocketApp)
    monkeypatch.setattr(robot_sdk_bridge.uuid, "uuid4", lambda: next(robot_sdk_bridge._uuid_gen))


def test_bridge_start_establishes_connection(monkeypatch):
    monkeypatch.setattr(robot_sdk_bridge, "_uuid_gen", uuid_sequence())

    bridge = robot_sdk_bridge.RobotSDKBridge(url="ws://test-server")
    bridge.start(background=False)

    assert DummyWebSocketApp.instances, "WebSocket should be instantiated"
    assert DummyWebSocketApp.instances[0].url == "ws://test-server"
    assert bridge.is_connected


def test_send_stand_mode_sends_expected_payload(monkeypatch):
    monkeypatch.setattr(robot_sdk_bridge, "_uuid_gen", uuid_sequence())
    monkeypatch.setattr(robot_sdk_bridge.time, "time", lambda: 1234567890.0)

    bridge = robot_sdk_bridge.RobotSDKBridge(url="ws://test-server")
    bridge.start(background=False)
    bridge.send_stand_mode()

    sent = DummyWebSocketApp.instances[0].sent_messages
    assert sent, "No message sent"
    payload = json.loads(sent[-1])
    assert payload["title"] == "request_stand_mode"
    assert payload["timestamp"] == 1234567890000


def test_notify_robot_info_updates_cache(monkeypatch):
    monkeypatch.setattr(robot_sdk_bridge, "_uuid_gen", uuid_sequence())

    bridge = robot_sdk_bridge.RobotSDKBridge(url="ws://test-server")
    bridge.start(background=False)

    message = json.dumps(
        {
            "accid": "TRON",
            "title": "notify_robot_info",
            "timestamp": 1,
            "guid": "abc",
            "data": {
                "status": "WALK",
                "battery": 88,
            },
        }
    )

    DummyWebSocketApp.instances[0].on_message(DummyWebSocketApp.instances[0], message)

    assert bridge.robot_info["status"] == "WALK"
    assert bridge.robot_info["battery"] == 88




