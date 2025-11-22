"""Route manager for waypoint CRUD and execution."""

from __future__ import annotations

import json
import uuid
import time
import threading
import logging
from pathlib import Path
from typing import Any, Dict, Iterable, List, Optional

logger = logging.getLogger(__name__)

class RouteManager:
    def __init__(
        self,
        data_dir: Optional[Path] = None,
        ros_bridge: Any = None,
        robot_sdk: Any = None,
    ) -> None:
        self.data_dir = Path(data_dir or Path(__file__).resolve().parent / "data")
        self.data_dir.mkdir(parents=True, exist_ok=True)
        self.data_file = self.data_dir / "routes.json"
        self._routes: List[Dict[str, Any]] = []
        self._ros_bridge = ros_bridge
        self._robot_sdk = robot_sdk
        self._current_route: Optional[str] = None
        self._stop_execution_flag = False
        self._execution_thread: Optional[threading.Thread] = None
        self._load()

    # ------------------------------------------------------------------
    def _load(self) -> None:
        if self.data_file.exists():
            try:
                self._routes = json.loads(self.data_file.read_text()).get("routes", [])
            except Exception as e:
                logger.error(f"Failed to load routes: {e}")
                self._routes = []
        else:
            self._routes = []
            self._flush()

    def _flush(self) -> None:
        payload = {"routes": self._routes}
        try:
            self.data_file.write_text(json.dumps(payload, indent=2, ensure_ascii=False))
        except Exception as e:
            logger.error(f"Failed to save routes: {e}")

    # ------------------------------------------------------------------
    def create_route(self, name: str, waypoints: Iterable[Dict[str, Any]]) -> Dict[str, Any]:
        route = {
            "id": str(uuid.uuid4()),
            "name": name,
            "waypoints": list(waypoints),
        }
        self._routes.append(route)
        self._flush()
        return route

    def list_routes(self) -> List[Dict[str, Any]]:
        return list(self._routes)

    def delete_route(self, route_id: str) -> bool:
        before = len(self._routes)
        self._routes = [route for route in self._routes if route["id"] != route_id]
        changed = len(self._routes) != before
        if changed:
            self._flush()
        return changed

    def stop_execution(self) -> None:
        """Stop the current route execution."""
        if self._current_route:
            logger.info(f"Stopping route execution: {self._current_route}")
            self._stop_execution_flag = True
            # Also cancel current navigation goal via ROS
            if self._ros_bridge:
                # Publish current position as goal to stop robot immediately
                pos = self._ros_bridge.get_robot_position()
                if pos:
                    self._ros_bridge.publish_nav_goal(pos['x'], pos['y'], pos['yaw'])
            
    def execute_route(self, route_id: str) -> None:
        route = next((r for r in self._routes if r["id"] == route_id), None)
        if not route:
            raise KeyError(f"route {route_id} not found")

        if not self._ros_bridge:
            raise RuntimeError("ROS bridge not set")
        if not self._robot_sdk:
            # It's okay if robot_sdk is not set (e.g. simulation or limited mode), just warn
            logger.warning("RobotSDK bridge not set, some features (stair mode) may not work")

        # Stop any previous execution
        if self._current_route:
            self.stop_execution()
            if self._execution_thread and self._execution_thread.is_alive():
                self._execution_thread.join(timeout=1.0)

        self._current_route = route_id
        self._stop_execution_flag = False
        
        self._execution_thread = threading.Thread(target=self._execution_loop, args=(route,))
        self._execution_thread.daemon = True
        self._execution_thread.start()

    def _execution_loop(self, route: Dict[str, Any]) -> None:
        logger.info(f"Starting patrol route execution: {route['name']} ({len(route['waypoints'])} waypoints)")
        
        try:
            for index, waypoint in enumerate(route["waypoints"]):
                if self._stop_execution_flag:
                    logger.info("Patrol execution stopped by user")
                    break
                    
                wp_type = waypoint.get("type", "normal")
                if self._robot_sdk:
                    if wp_type == "stair_enable":
                        self._robot_sdk.send_stair_mode(True)
                    elif wp_type == "stair_disable":
                        self._robot_sdk.send_stair_mode(False)

                # Prepare goal
                goal_payload = {
                    "goal_x": float(waypoint.get("x", 0.0)),
                    "goal_y": float(waypoint.get("y", 0.0)),
                    "goal_theta": float(waypoint.get("yaw", 0.0)),
                    "goal_id": f"{route['id']}-{index}",  # Use string ID
                }
                
                logger.info(f"Navigating to waypoint {index+1}/{len(route['waypoints'])}: {goal_payload}")
                self._ros_bridge.publish_goal(goal_payload)
                
                # Wait for goal to be accepted/started
                time.sleep(1.0)
                
                # Monitor status
                # Wait until status is SUCCEEDED or FAILED
                timeout = 300.0 # 5 minutes timeout per waypoint
                start_time = time.time()
                
                while not self._stop_execution_flag:
                    # Check timeout
                    if time.time() - start_time > timeout:
                        logger.warning(f"Waypoint {index+1} timed out")
                        self._stop_execution_flag = True
                        break
                        
                    status = self._ros_bridge._latest_nav_status
                    
                    if status == "SUCCEEDED":
                        logger.info(f"Waypoint {index+1} reached")
                        # Optional: wait a bit at the waypoint
                        time.sleep(0.5)
                        break
                    elif status in ["FAILED", "ABORTED", "CANCELLED"]:
                        # If it failed immediately after sending, it might be because we just sent it.
                        # But we slept 1.0s above.
                        # Double check if this status corresponds to OUR goal?
                        # ros_extended_node stores _current_goal_id.
                        # But we don't easily know if the FAILED status belongs to this specific goal ID without more complex logic.
                        # For now, assume any failure stops the patrol.
                        logger.error(f"Waypoint {index+1} failed with status: {status}")
                        self._stop_execution_flag = True
                        break
                    
                    time.sleep(0.5)
                    
        except Exception as e:
            logger.error(f"Error during patrol execution: {e}")
        finally:
            logger.info(f"Patrol finished or stopped. Route: {route['name']}")
            self._current_route = None

    def current_route(self) -> Optional[str]:
        return self._current_route
