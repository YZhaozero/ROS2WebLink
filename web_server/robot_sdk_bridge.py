"""Robot SDK WebSocket bridge implementation."""

from __future__ import annotations

import json
import logging
import threading
import time
import uuid
from dataclasses import dataclass, field
from typing import Any, Callable, Dict, Optional

from websocket import WebSocketApp


logger = logging.getLogger(__name__)


def _uuid_generator():
    while True:
        yield uuid.uuid4()


_uuid_gen = _uuid_generator()


EventCallback = Callable[[Dict[str, Any]], None]


@dataclass
class RobotState:
    robot_info: Dict[str, Any] = field(default_factory=dict)
    imu: Dict[str, Any] = field(default_factory=dict)
    odom: Dict[str, Any] = field(default_factory=dict)


class RobotSDKBridge:
    """Thin wrapper around the official robot WebSocket SDK protocol."""

    def __init__(self, url: str = "ws://10.192.1.2:5000", accid: Optional[str] = None):
        self.url = url
        self.accid = accid
        self._ws: Optional[WebSocketApp] = None
        self._thread: Optional[threading.Thread] = None
        self._lock = threading.Lock()
        self._state = RobotState()
        self._callbacks: Dict[str, list[EventCallback]] = {}
        self._connected = threading.Event()
        self._stop_requested = threading.Event()
        self._nav_status: Optional[str] = None

    # ------------------------------------------------------------------
    # Lifecycle
    # ------------------------------------------------------------------
    def start(self, *, background: bool = True) -> None:
        if self._ws is not None:
            logger.debug("RobotSDKBridge already started")
            return

        self._stop_requested.clear()
        self._ws = WebSocketApp(
            self.url,
            on_open=self._on_open,
            on_message=self._on_message,
            on_close=self._on_close,
            on_error=self._on_error,
        )

        if background:
            self._thread = threading.Thread(target=self._ws.run_forever, daemon=True)
            self._thread.start()
        else:
            self._ws.run_forever()

    def stop(self) -> None:
        self._stop_requested.set()
        if self._ws:
            try:
                self._ws.close()
            except Exception as exc:  # pragma: no cover - defensive
                logger.warning("Failed to close WebSocket: %s", exc)
        self._connected.clear()
        self._ws = None

    # ------------------------------------------------------------------
    # Properties
    # ------------------------------------------------------------------
    @property
    def is_connected(self) -> bool:
        return self._connected.is_set()

    @property
    def robot_info(self) -> Dict[str, Any]:
        return dict(self._state.robot_info)

    @property
    def navigation_status(self) -> Optional[str]:
        return self._nav_status

    # ------------------------------------------------------------------
    # Command helpers
    # ------------------------------------------------------------------
    def send_stand_mode(self) -> None:
        self._send_command("request_stand_mode")

    def send_walk_mode(self) -> None:
        self._send_command("request_walk_mode")

    def send_sit_mode(self) -> None:
        self._send_command("request_sitdown")

    def send_stair_mode(self, enable: bool) -> None:
        self._send_command("request_stair_mode", {"enable": bool(enable)})

    def send_twist(self, x: float, y: float, z: float) -> None:
        self._send_command("request_twist", {"x": x, "y": y, "z": z})

    def send_emergency_stop(self) -> None:
        self._send_command("request_emgy_stop")

    def send_pause(self, resume: bool) -> None:
        self._send_command("request_pause" if not resume else "request_resume")

    def send_body_height(self, direction: int) -> None:
        """Adjust robot body height by 5cm each call.
        
        Args:
            direction: 1 for increase height, -1 for decrease height
        """
        self._send_command("request_base_height", {"direction": int(direction)})

    def send_recover(self) -> None:
        """Send recovery command (stand up from fall)."""
        self._send_command("request_recover")

    def _send_command(self, title: str, data: Optional[Dict[str, Any]] = None) -> None:
        if not self._ws:
            raise RuntimeError("WebSocket connection not established")

        payload = {
            "accid": self.accid,
            "title": title,
            "timestamp": int(time.time() * 1000),
            "guid": str(uuid.uuid4()),
            "data": data or {},
        }

        message = json.dumps(payload)
        logger.info("Sending command %s with data: %s", title, data or {})
        with self._lock:
            self._ws.send(message)

    # ------------------------------------------------------------------
    # WebSocket callbacks
    # ------------------------------------------------------------------
    def _on_open(self, _ws) -> None:
        logger.info("RobotSDKBridge connected to %s", self.url)
        self._connected.set()

    def _on_close(self, _ws, *_args) -> None:
        logger.info("RobotSDKBridge disconnected from %s", self.url)
        self._connected.clear()

    def _on_error(self, _ws, error) -> None:  # pragma: no cover - logging only
        logger.error("RobotSDKBridge error: %s", error)

    def _on_message(self, _ws, message: str) -> None:
        try:
            payload = json.loads(message)
        except json.JSONDecodeError:
            logger.warning("Ignoring invalid JSON payload: %s", message)
            return

        title = payload.get("title")
        
        # Auto-detect accid from first notify_robot_info if not set
        if self.accid is None and title == "notify_robot_info":
            robot_accid = payload.get("accid")
            if robot_accid:
                self.accid = robot_accid
                logger.info("Auto-detected robot accid: %s", self.accid)
        
        # Log all responses and notifications for debugging
        if title and (title.startswith("response_") or title.startswith("notify_")):
            logger.info("Received %s: %s", title, payload.get("data", {}))
        
        if title == "notify_robot_info":
            self._state.robot_info = payload.get("data", {})
        elif title == "notify_imu":
            self._state.imu = payload.get("data", {})
        elif title == "notify_odom":
            self._state.odom = payload.get("data", {})
        elif title == "notify_nav_status":
            self._nav_status = payload.get("data", {}).get("status")

        self._emit(title or "", payload)

    # ------------------------------------------------------------------
    # Event system
    # ------------------------------------------------------------------
    def on(self, title: str, callback: EventCallback) -> None:
        self._callbacks.setdefault(title, []).append(callback)

    def _emit(self, title: str, payload: Dict[str, Any]) -> None:
        for callback in self._callbacks.get(title, []):
            try:
                callback(payload)
            except Exception as exc:  # pragma: no cover - defensive
                logger.warning("Event handler %s raised %s", callback, exc)


__all__ = ["RobotSDKBridge"]

