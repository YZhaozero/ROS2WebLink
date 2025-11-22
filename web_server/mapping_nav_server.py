"""FastAPI application bootstrap for the mapping & navigation WebUI."""

import asyncio
import logging
import time
from pathlib import Path

from fastapi import FastAPI, HTTPException, Response
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import FileResponse, JSONResponse
from fastapi.staticfiles import StaticFiles
from pydantic import BaseModel, Field, field_validator

from .robot_sdk_bridge import RobotSDKBridge
from .ros_extended_node import ExtendedRosBridge
from .mapping_controller import MappingController
from .navigation_controller import NavigationController
from .route_manager import RouteManager
from .dlio_health_checker import DLIOHealthChecker
from .tools.manual_relocalization import ManualRelocalizationTool

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger(__name__)


BASE_DIR = Path(__file__).resolve().parent
STATIC_DIR = BASE_DIR / "static"

_robot_sdk = RobotSDKBridge()
_ros_bridge = ExtendedRosBridge()

# Rosbag cleanup policy: "delete" (default), "archive", or "keep"
# - "delete": Delete rosbag after removert processing (saves disk space)
# - "archive": Move rosbag to archive directory (keeps for later use)
# - "keep": Keep rosbag in original location (for debugging)
import os
_rosbag_cleanup_policy = os.getenv("ROSBAG_CLEANUP_POLICY", "delete").lower()
if _rosbag_cleanup_policy not in ["delete", "archive", "keep"]:
    logger.warning(f"Invalid ROSBAG_CLEANUP_POLICY '{_rosbag_cleanup_policy}', using 'delete'")
    _rosbag_cleanup_policy = "delete"

_mapping_controller = MappingController(
    workdir=Path("/home/guest/tron_ros2"),
    rosbag_cleanup_policy=_rosbag_cleanup_policy
)
_navigation_controller = NavigationController(workdir=Path("/home/guest/tron_ros2"))
_route_manager = RouteManager(ros_bridge=_ros_bridge, robot_sdk=_robot_sdk)

# Initialize DLIO health checker
_dlio_health_checker = DLIOHealthChecker(_ros_bridge._adapter.node)

# Trajectory recording state
_trajectory_recording = {
    "active": False,
    "waypoints": [],
    "interval": 1.0,  # Recording interval in seconds
    "last_record_time": 0.0
}


class MappingStartRequest(BaseModel):
    map_name: str | None = Field(default=None, description="Target map name")


class MappingStopRequest(BaseModel):
    save: bool = Field(default=True, description="Save map when stopping")
    map_name: str | None = None


class WaypointModel(BaseModel):
    x: float
    y: float
    yaw: float = 0.0
    type: str = "normal"

    @field_validator("type")
    @classmethod
    def validate_type(cls, value: str) -> str:
        allowed = {"normal", "stair_enable", "stair_disable"}
        if value not in allowed:
            raise ValueError(f"type must be one of {allowed}")
        return value


class RouteCreateRequest(BaseModel):
    name: str
    waypoints: list[WaypointModel]


class TwistCommand(BaseModel):
    vel_x: float = 0.0
    vel_y: float = 0.0
    vel_theta: float = 0.0


class StairModeRequest(BaseModel):
    enable: bool


class WaypointRecordRequest(BaseModel):
    type: str = "normal"


class TrajectoryStartRequest(BaseModel):
    interval: float = 1.0


class NavigationStartRequest(BaseModel):
    map_name: str | None = Field(default=None, description="Map file name (without extension)")


class SetInitialPoseRequest(BaseModel):
    x: float = Field(description="X coordinate in map frame (meters)")
    y: float = Field(description="Y coordinate in map frame (meters)")
    z: float = Field(default=0.0, description="Z coordinate in map frame (meters)")
    yaw: float = Field(default=0.0, description="Yaw angle in radians")


class SetNavGoalRequest(BaseModel):
    goal_x: float = Field(description="X coordinate in map frame (meters)")
    goal_y: float = Field(description="Y coordinate in map frame (meters)")
    goal_theta: float = Field(default=0.0, description="Yaw angle in radians")
    goal_id: int = Field(default=0, description="Goal ID for tracking")
    xy_tolerance: float = Field(default=0.1, description="Position tolerance in meters")
    yaw_tolerance: float = Field(default=0.05, description="Orientation tolerance in radians")


def create_app() -> FastAPI:
    """Create and configure the FastAPI application."""

    app = FastAPI(
        title="Tron Mapping & Navigation WebUI",
        description="Backend service for robot mapping and navigation control",
        version="0.1.0",
        docs_url="/docs",
        redoc_url="/redoc",
    )

    app.add_middleware(
        CORSMiddleware,
        allow_origins=["*"],
        allow_credentials=False,
        allow_methods=["*"],
        allow_headers=["*"],
    )
    
    # HTTP request logging middleware
    @app.middleware("http")
    async def log_requests(request, call_next):
        start_time = time.time()
        response = await call_next(request)
        duration = time.time() - start_time
        logger.info(
            f"{request.method} {request.url.path} - "
            f"Status: {response.status_code} - "
            f"Duration: {duration:.3f}s"
        )
        return response

    if not STATIC_DIR.exists():
        STATIC_DIR.mkdir(parents=True, exist_ok=True)

    # --- Manual Relocalization Tool Integration ---
    try:
        _manual_reloc_tool = ManualRelocalizationTool(_ros_bridge)
        app.include_router(_manual_reloc_tool.router)
        logger.info("Manual Relocalization Tool initialized")
        
        @app.get("/manual_reloc")
        async def manual_reloc_page():
            return FileResponse(STATIC_DIR / "manual_reloc.html")
    except Exception as e:
        logger.error(f"Failed to initialize Manual Relocalization Tool: {e}")
    # ----------------------------------------------

    app.mount("/static", StaticFiles(directory=STATIC_DIR), name="static")

    if hasattr(_robot_sdk, "start"):
        try:
            _robot_sdk.start()
        except Exception:
            pass
    
    @app.on_event("startup")
    async def startup_background_tasks():
        """Start background tasks on application startup."""
        asyncio.create_task(trajectory_recording_task())
        
        # Start DLIO health monitoring
        def on_dlio_ready():
            """Callback when DLIO becomes ready."""
            logger.info("🎉 DLIO is ready! Subscribing to DLIO topics...")
            _ros_bridge.subscribe_to_dlio()
        
        _dlio_health_checker.register_ready_callback(on_dlio_ready)
        _dlio_health_checker.start_monitoring(check_interval=3.0)
        logger.info("🔍 DLIO health monitoring started (will subscribe when ready)")
    
    async def trajectory_recording_task():
        """Background task: periodically record trajectory waypoints."""
        while True:
            await asyncio.sleep(0.1)
            if _trajectory_recording["active"]:
                current_time = time.time()
                if current_time - _trajectory_recording["last_record_time"] >= _trajectory_recording["interval"]:
                    pos = _ros_bridge.get_robot_position()
                    if pos:
                        _trajectory_recording["waypoints"].append({
                            "x": pos["x"],
                            "y": pos["y"],
                            "yaw": pos["yaw"],
                            "type": "normal",
                            "timestamp": current_time
                        })
                        _trajectory_recording["last_record_time"] = current_time

    @app.get("/healthz", response_class=JSONResponse)
    async def healthz() -> dict[str, str]:
        """Lightweight liveness probe."""

        return {"status": "ok"}

    @app.post("/api/mapping/start")
    async def mapping_start(payload: MappingStartRequest):
        try:
            logger.info(f"🗺️ Starting mapping mode (map_name: {payload.map_name})")
            # Clear previous localization data to allow odom frame updates from DLIO
            _ros_bridge._robot_position = None
            logger.info("Cleared previous localization data for mapping mode")
            
            # Start mapping in background task - don't wait for it
            async def start_mapping_background():
                try:
                    result = _mapping_controller.start(payload.map_name)
                    logger.info(f"✅ Mapping processes started: {result}")
                    
                    # Wait for DLIO health check in background
                    if not _ros_bridge._dlio_subscribed:
                        import time
                        wait_start = time.time()
                        max_wait = 10.0  # Maximum wait time: 10 seconds
                        
                        logger.info("⏳ Waiting for DLIO health check (max 10s)...")
                        while time.time() - wait_start < max_wait:
                            if _dlio_health_checker.is_ready():
                                logger.info("✅ DLIO health check passed, subscribing...")
                                break
                            await asyncio.sleep(0.5)
                        
                        # Subscribe regardless of health check result (fail-safe)
                        if not _ros_bridge._dlio_subscribed:
                            _ros_bridge.subscribe_to_dlio()
                            if _dlio_health_checker.is_ready():
                                logger.info("✅ Subscribed to DLIO topics (health check passed)")
                            else:
                                logger.warning("⚠️ Subscribed to DLIO topics (health check timeout, forcing subscription)")
                except Exception as e:
                    logger.error(f"❌ Background mapping start failed: {e}")
            
            # Start background task without waiting
            asyncio.create_task(start_mapping_background())
            
            # Return immediately
            logger.info(f"✅ Mapping start initiated (background)")
            return {"status": "STARTING", "message": "Mapping is starting in background"}
        except RuntimeError as exc:  # pragma: no cover - guarded by tests
            logger.error(f"❌ Failed to start mapping: {exc}")
            raise HTTPException(status_code=409, detail=str(exc))

    @app.post("/api/mapping/stop")
    async def mapping_stop(payload: MappingStopRequest):
        logger.info(f"⏹️ Stopping mapping (save={payload.save}, map_name={payload.map_name})")
        # Stop is already async (uses background thread), but ensure we return immediately
        result = _mapping_controller.stop(save=payload.save, map_name=payload.map_name)
        logger.info(f"✅ Mapping stop initiated")
        return result

    @app.get("/api/mapping/status")
    async def mapping_status():
        return _mapping_controller.status()
    
    @app.post("/api/mapping/clear_cache")
    async def clear_mapping_cache():
        """Clear all mapping processes and reset state."""
        import subprocess
        try:
            # Stop mapping controller processes
            _mapping_controller.stop(save=False)
            
            # Kill all mapping-related processes
            subprocess.run(["pkill", "-9", "-f", "dlio|octomap|livox_ros_driver2|pointcloud2"], 
                          capture_output=True, timeout=5)
            subprocess.run(["pkill", "-9", "-f", "static_transform_publisher.*livox"], 
                          capture_output=True, timeout=5)
            
            # Reset controller state
            _mapping_controller._state = "IDLE"
            _mapping_controller._processes.clear()
            _mapping_controller._save_status = "idle"
            
            # Clear cached map data in ROS bridge
            _ros_bridge._latest_map = None
            _ros_bridge._latest_global_costmap = None
            _ros_bridge._latest_local_costmap = None
            
            return {"status": "ok", "message": "缓存已清空，所有建图进程已停止"}
        except Exception as e:
            raise HTTPException(status_code=500, detail=f"清空缓存失败: {str(e)}")

    @app.get("/api/robot/map")
    async def robot_map():
        """Get real-time map data (ros_web_server compatible)."""
        try:
            map_data = _ros_bridge.get_map_json()
            logger.debug(f"Returning real-time map: {map_data['width']}x{map_data['height']}")
            # Add ros_web_server compatible fields
            return {
                "Result": 0,
                "Error": "",
                "name": "map",
                "resolution": map_data["resolution"],
                "width": map_data["width"],
                "height": map_data["height"],
                "origin_x": map_data["origin_x"],
                "origin_y": map_data["origin_y"],
                "origin_yaw": 0.0,
                "data": map_data["data"]
            }
        except RuntimeError as exc:
            logger.debug(f"Real-time map requested but not available: {exc}")
            return {
                "Result": 1,
                "Error": str(exc)
            }
    
    @app.get("/api/maps/{map_name}/load")
    async def load_map_file(map_name: str):
        """Load a specific map file from disk and return as occupancy grid."""
        import yaml
        from PIL import Image
        import base64
        
        maps_dir = Path("/home/guest/tron_ros2/src/tron_nav/tron_navigation/maps")
        yaml_file = maps_dir / f"{map_name}.yaml"
        pgm_file = maps_dir / f"{map_name}.pgm"
        
        if not yaml_file.exists() or not pgm_file.exists():
            raise HTTPException(status_code=404, detail=f"Map '{map_name}' not found")
        
        # Read map metadata
        with open(yaml_file, 'r') as f:
            map_config = yaml.safe_load(f)
        
        # Read PGM image
        img = Image.open(pgm_file)
        width, height = img.size
        
        # PIL reads PGM files in top-to-bottom order (row 0 at top)
        # This matches Canvas format, so no flip needed
        # (ROS2 data is flipped in ros_extended_node to match this format)
        img_data = list(img.getdata())
        
        # Convert to occupancy grid format (0-100, -1 for unknown)
        occupancy_data = []
        for pixel in img_data:
            if pixel == 205:  # Unknown (gray in PGM)
                occupancy_data.append(255)  # Use 255 to represent unknown, will convert to -1 in frontend
            elif pixel >= 250:  # Free space (white)
                occupancy_data.append(0)
            elif pixel <= 5:  # Occupied (black)
                occupancy_data.append(100)
            else:
                # Linear interpolation for gray values
                occupancy_data.append(int((254 - pixel) / 254 * 100))
        
        # Encode as base64
        data_bytes = bytes(occupancy_data)
        data_b64 = base64.b64encode(data_bytes).decode('utf-8')
        
        origin = map_config.get('origin', [0.0, 0.0, 0.0])
        
        return {
            "width": width,
            "height": height,
            "resolution": float(map_config.get('resolution', 0.05)),
            "origin_x": float(origin[0]),
            "origin_y": float(origin[1]),
            "data": data_b64
        }

    class MapSaveRequest(BaseModel):
        map_name: str
        image_data: str  # Base64 encoded PNG data

    @app.post("/api/maps/save_edit")
    async def save_edited_map(payload: MapSaveRequest):
        """Save edited map image back to PGM format."""
        import base64
        import numpy as np
        import cv2
        import shutil
        from datetime import datetime
        
        maps_dir = Path("/home/guest/tron_ros2/src/tron_nav/tron_navigation/maps")
        pgm_path = maps_dir / f"{payload.map_name}.pgm"
        
        if not pgm_path.exists():
            raise HTTPException(status_code=404, detail="Original map file not found")

        try:
            # Decode Base64 image
            # format: "data:image/png;base64,iVBORw0KGgo..."
            header, encoded = payload.image_data.split(",", 1)
            binary_data = base64.b64decode(encoded)
            nparr = np.frombuffer(binary_data, np.uint8)
            edited_img = cv2.imdecode(nparr, cv2.IMREAD_GRAYSCALE)
            
            if edited_img is None:
                 raise ValueError("Failed to decode image data")

            # Read original image to compare size
            original_img = cv2.imread(str(pgm_path), cv2.IMREAD_GRAYSCALE)
            if original_img.shape != edited_img.shape:
                raise HTTPException(status_code=400, 
                    detail=f"Image size mismatch! Original: {original_img.shape}, New: {edited_img.shape}")
                
            # Backup original file
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            backup_path = pgm_path.parent / f"{payload.map_name}_backup_{timestamp}.pgm"
            shutil.copy2(pgm_path, backup_path)
            
            # Standardize pixel values
            # Free space (white) -> 254
            edited_img[edited_img >= 250] = 254
            # Unknown (gray) -> 205
            mask_gray = (edited_img >= 200) & (edited_img < 250)
            edited_img[mask_gray] = 205
            # Obstacle (black) -> 0
            edited_img[edited_img <= 10] = 0
            
            # Save overwriting original file
            cv2.imwrite(str(pgm_path), edited_img)
            
            return {
                "success": True, 
                "message": "Map saved successfully",
                "backup": backup_path.name
            }
            
        except Exception as e:
            logger.error(f"Failed to save map edit: {e}")
            raise HTTPException(status_code=500, detail=str(e))
    
    @app.get("/api/maps/{map_name}/image")
    async def get_map_image_raw(map_name: str):
        """Get raw map PGM converted to PNG for editing."""
        import cv2
        import io
        
        maps_dir = Path("/home/guest/tron_ros2/src/tron_nav/tron_navigation/maps")
        pgm_path = maps_dir / f"{map_name}.pgm"
        
        if not pgm_path.exists():
            raise HTTPException(status_code=404, detail="Map file not found")
            
        # Read PGM
        img = cv2.imread(str(pgm_path), cv2.IMREAD_GRAYSCALE)
        if img is None:
             raise HTTPException(status_code=500, detail="Could not read map image")
             
        # Encode to PNG
        _, buffer = cv2.imencode('.png', img)
        return Response(content=buffer.tobytes(), media_type="image/png")
    
    @app.get("/api/maps/list")
    async def list_maps():
        """List all available map files."""
        maps_dir = Path("/home/guest/tron_ros2/src/tron_nav/tron_navigation/maps")
        if not maps_dir.exists():
            return {"maps": []}
        
        map_files = []
        for yaml_file in maps_dir.glob("*.yaml"):
            map_name = yaml_file.stem
            pgm_file = maps_dir / f"{map_name}.pgm"
            if pgm_file.exists():
                map_files.append({
                    "name": map_name,
                    "size": pgm_file.stat().st_size,
                    "modified": pgm_file.stat().st_mtime
                })
        
        # Sort by modification time (newest first)
        map_files.sort(key=lambda x: x["modified"], reverse=True)
        return {"maps": map_files}
    
    @app.get("/api/maps/registry")
    async def get_map_registry():
        """Get all maps from map_registry.yaml with PCD+PGM/YAML pairing."""
        import yaml
        
        registry_file = Path("/home/guest/tron_ros2/src/tron_slam/localizer/config/map_registry.yaml")
        
        if not registry_file.exists():
            return {
                "available": False,
                "message": "Map registry not found",
                "maps": []
            }
        
        try:
            with open(registry_file, 'r') as f:
                registry = yaml.safe_load(f)
            
            maps = registry.get('maps', {})
            default_map = registry.get('default_map', 'default')
            
            map_list = []
            for map_name, config in maps.items():
                # Check file existence
                pcd_exists = Path(config['pcd_file']).exists() if 'pcd_file' in config else False
                yaml_exists = Path(config['yaml_file']).exists() if 'yaml_file' in config else False
                pgm_exists = Path(config['pgm_file']).exists() if 'pgm_file' in config else False
                
                map_list.append({
                    "name": map_name,
                    "description": config.get('description', ''),
                    "pcd_file": config.get('pcd_file', ''),
                    "yaml_file": config.get('yaml_file', ''),
                    "pgm_file": config.get('pgm_file', ''),
                    "enabled": config.get('enabled', True),
                    "is_default": map_name == default_map,
                    "valid": pcd_exists and yaml_exists and pgm_exists,
                    "files_status": {
                        "pcd": pcd_exists,
                        "yaml": yaml_exists,
                        "pgm": pgm_exists
                    }
                })
            
            # Sort: default first, then by name
            map_list.sort(key=lambda x: (not x['is_default'], x['name']))
            
            return {
                "available": True,
                "default_map": default_map,
                "total": len(map_list),
                "maps": map_list
            }
        except Exception as e:
            logger.error(f"Failed to load map registry: {e}")
            return {
                "available": False,
                "message": f"Error loading registry: {str(e)}",
                "maps": []
            }
    
    @app.get("/api/maps/registry/{map_name}")
    async def get_map_from_registry(map_name: str):
        """Get specific map details from registry."""
        import yaml
        
        registry_file = Path("/home/guest/tron_ros2/src/tron_slam/localizer/config/map_registry.yaml")
        
        if not registry_file.exists():
            raise HTTPException(status_code=404, detail="Map registry not found")
        
        try:
            with open(registry_file, 'r') as f:
                registry = yaml.safe_load(f)
            
            maps = registry.get('maps', {})
            
            if map_name not in maps:
                raise HTTPException(status_code=404, detail=f"Map '{map_name}' not found in registry")
            
            config = maps[map_name]
            
            # Validate files
            pcd_file = Path(config['pcd_file'])
            yaml_file = Path(config['yaml_file'])
            pgm_file = Path(config['pgm_file'])
            
            return {
                "name": map_name,
                "description": config.get('description', ''),
                "pcd_file": str(pcd_file),
                "yaml_file": str(yaml_file),
                "pgm_file": str(pgm_file),
                "enabled": config.get('enabled', True),
                "valid": pcd_file.exists() and yaml_file.exists() and pgm_file.exists(),
                "files_status": {
                    "pcd": pcd_file.exists(),
                    "yaml": yaml_file.exists(),
                    "pgm": pgm_file.exists()
                }
            }
        except HTTPException:
            raise
        except Exception as e:
            logger.error(f"Failed to get map from registry: {e}")
            raise HTTPException(status_code=500, detail=str(e))
    
    @app.delete("/api/maps/{map_name}")
    async def delete_map(map_name: str):
        """Delete a map file."""
        maps_dir = Path("/home/guest/tron_ros2/src/tron_nav/tron_navigation/maps")
        yaml_file = maps_dir / f"{map_name}.yaml"
        pgm_file = maps_dir / f"{map_name}.pgm"
        
        deleted = []
        if yaml_file.exists():
            yaml_file.unlink()
            deleted.append(f"{map_name}.yaml")
        if pgm_file.exists():
            pgm_file.unlink()
            deleted.append(f"{map_name}.pgm")
        
        if not deleted:
            raise HTTPException(status_code=404, detail="Map not found")
        
        return {"deleted": deleted}

    @app.get("/api/costmap/{kind}")
    async def costmap(kind: str):
        try:
            return _ros_bridge.get_costmap_json(kind)
        except RuntimeError as exc:
            raise HTTPException(status_code=404, detail=str(exc))

    @app.get("/api/robot/status")
    async def robot_status():
        """Get robot status (ros_web_server compatible)."""
        info = _robot_sdk.robot_info or {}
        
        # Calculate theta from odom message (ros_web_server compatible way)
        theta = 0.0
        if _ros_bridge.latest_odom:
            orientation = _ros_bridge.latest_odom.pose.pose.orientation
            theta = _ros_bridge.quaternion_to_yaw(orientation)
        
        # Get battery power (ros_web_server compatible way)
        battery_power = 0.0
        battery = info.get("battery")
        if battery is not None:
            battery_power = float(battery)
        
        return {
            "robot_info": info,  # Include full robot_info for frontend
            "battery": {
                "power": battery_power,
                "charging": False
            },
            "localization": {
                "status": 0 if _ros_bridge.latest_odom else 1,
                "x": float(_ros_bridge.latest_odom.pose.pose.position.x) 
                    if _ros_bridge.latest_odom else 0.0,
                "y": float(_ros_bridge.latest_odom.pose.pose.position.y) 
                    if _ros_bridge.latest_odom else 0.0,
                "theta": theta,
                "reliability": 0.95
            },
            "navigation": {
                "status": _ros_bridge._latest_nav_status,
                "blocked": False,
                "goal_id": _ros_bridge.current_task_id if _ros_bridge.current_task_id is not None else 0
            }
        }
    
    @app.get("/api/sensors/status")
    async def sensor_status():
        """Get sensor health status including Livox and mapping."""
        return _ros_bridge.get_sensor_status()
    
    @app.get("/api/dlio/status")
    async def dlio_status():
        """Get DLIO health status and readiness."""
        return {
            "ready": _dlio_health_checker.is_ready(),
            "subscribed": _ros_bridge._dlio_subscribed,
            "message": "DLIO is ready and subscribed" if _dlio_health_checker.is_ready() else "Waiting for DLIO to be ready..."
        }
    
    @app.get("/api/robot/navigation_status")
    async def get_navigation_status():
        """Get navigation status (API document compliant)."""
        nav_status = _ros_bridge.get_navigation_status()
        return {
            "Result": 0,
            "Error": "",
            **nav_status
        }
    
    @app.get("/api/robot/position")
    async def robot_position():
        """Get robot real-time position on the map with frame_id."""
        pos = _ros_bridge.get_robot_position()
        if pos is None:
            logger.debug("Robot position requested but not available")
            raise HTTPException(status_code=503, detail="Robot position not available")
        logger.debug(f"Robot position: x={pos['x']:.2f}, y={pos['y']:.2f}, yaw={pos['yaw']:.2f}, frame={pos.get('frame_id', 'unknown')}")
        return pos
    
    @app.get("/api/robot/scan_points")
    async def scan_points():
        """Get 2D laser scan points in map frame for visualization."""
        scan_data = _ros_bridge.get_scan_points()
        if scan_data is None:
            logger.debug("Scan points requested but not available (robot not localized or no scan data)")
            raise HTTPException(status_code=503, detail="Scan points not available")
        logger.debug(f"Returning {len(scan_data['points'])} scan points")
        return scan_data
    
    @app.get("/api/robot/navigation_goal")
    async def navigation_goal():
        """Get current navigation goal coordinates and orientation."""
        goal = _ros_bridge.get_navigation_goal()
        if goal is None:
            raise HTTPException(status_code=503, detail="Navigation goal not available")
        return goal
    
    @app.post("/api/robot/set_initial_pose")
    async def set_initial_pose(request: SetInitialPoseRequest):
        """Set initial pose for GICP localizer via /initialpose topic."""
        try:
            import math
            _ros_bridge.publish_initial_pose(
                x=request.x,
                y=request.y,
                z=request.z,
                yaw=request.yaw
            )
            return {
                "success": True,
                "message": f"Initial pose set: ({request.x:.2f}, {request.y:.2f}, yaw={math.degrees(request.yaw):.1f}°)"
            }
        except Exception as e:
            raise HTTPException(status_code=500, detail=f"Failed to set initial pose: {str(e)}")
    
    @app.post("/api/robot/navigation_goal")
    async def set_navigation_goal(request: SetNavGoalRequest):
        """Set navigation goal via /goal_pose topic (API document compliant)."""
        try:
            # Prepare goal data in the format expected by ros_bridge
            goal_data = {
                "goal_x": request.goal_x,
                "goal_y": request.goal_y,
                "goal_theta": request.goal_theta,
                "goal_id": request.goal_id,
                "xy_tolerance": request.xy_tolerance,
                "yaw_tolerance": request.yaw_tolerance
            }
            _ros_bridge.publish_goal(goal_data)
            return {
                "Result": 0,
                "Error": ""
            }
        except Exception as e:
            return {
                "Result": 4,
                "Error": str(e)
            }
    
    @app.post("/api/waypoints/record")
    async def record_waypoint(payload: WaypointRecordRequest):
        """Record robot's current position as a waypoint."""
        pos = _ros_bridge.get_robot_position()
        if pos is None:
            raise HTTPException(status_code=503, detail="Robot position not available")
        
        waypoint = {
            "x": pos["x"],
            "y": pos["y"],
            "yaw": pos["yaw"],
            "type": payload.type,
            "timestamp": time.time()
        }
        return waypoint
    
    @app.post("/api/trajectory/start")
    async def start_trajectory_recording(payload: TrajectoryStartRequest):
        """Start recording trajectory."""
        _trajectory_recording["active"] = True
        _trajectory_recording["waypoints"] = []
        _trajectory_recording["interval"] = payload.interval
        _trajectory_recording["last_record_time"] = time.time()
        return {"status": "recording"}
    
    @app.post("/api/trajectory/stop")
    async def stop_trajectory_recording():
        """Stop recording trajectory and return waypoint list."""
        _trajectory_recording["active"] = False
        waypoints = _trajectory_recording["waypoints"]
        return {"waypoints": waypoints, "count": len(waypoints)}
    
    @app.get("/api/trajectory/status")
    async def trajectory_status():
        """Get trajectory recording status."""
        return {
            "active": _trajectory_recording["active"],
            "waypoint_count": len(_trajectory_recording["waypoints"])
        }
    
    @app.post("/api/navigation/start")
    async def start_navigation(payload: NavigationStartRequest):
        """Start navigation system (DLIO + Nav2 + tron_commander)."""
        try:
            map_name = payload.map_name or "final_test_map"
            logger.info(f"🧭 Starting navigation mode (map: {map_name})")
            result = _navigation_controller.start(map_name)
            logger.info(f"✅ Navigation started successfully")
            return result
        except RuntimeError as exc:
            logger.error(f"❌ Failed to start navigation: {exc}")
            raise HTTPException(status_code=409, detail=str(exc))
    
    @app.post("/api/navigation/stop")
    async def stop_navigation():
        """Stop navigation system."""
        return _navigation_controller.stop()
    
    @app.post("/api/navigation/cancel")
    async def cancel_navigation():
        """Cancel current navigation goal (stop executing trajectory)."""
        try:
            # Stop route manager if it's running a patrol
            _route_manager.stop_execution()
            
            # Publish an empty goal to cancel current navigation
            from geometry_msgs.msg import PoseStamped
            msg = PoseStamped()
            msg.header.frame_id = "map"
            msg.header.stamp = _ros_bridge._adapter.node.get_clock().now().to_msg()
            # Use current position as goal to effectively cancel
            pos = _ros_bridge.get_robot_position()
            if pos:
                msg.pose.position.x = pos['x']
                msg.pose.position.y = pos['y']
                msg.pose.position.z = 0.0
                msg.pose.orientation.w = 1.0
                _ros_bridge._goal_pub.publish(msg)
            return {"success": True, "message": "Navigation cancelled"}
        except Exception as e:
            raise HTTPException(status_code=500, detail=f"Failed to cancel navigation: {str(e)}")
    
    @app.get("/api/navigation/status")
    async def navigation_status():
        """Get navigation system status."""
        return _navigation_controller.status()

    @app.get("/api/routes")
    async def list_routes():
        return {"routes": _route_manager.list_routes()}

    @app.post("/api/routes")
    async def create_route(payload: RouteCreateRequest):
        route = _route_manager.create_route(
            payload.name,
            [wp.model_dump() for wp in payload.waypoints],
        )
        return route

    @app.delete("/api/routes/{route_id}")
    async def delete_route(route_id: str):
        if not _route_manager.delete_route(route_id):
            raise HTTPException(status_code=404, detail="Route not found")
        return {"status": "deleted"}

    @app.post("/api/routes/{route_id}/execute")
    async def execute_route(route_id: str):
        try:
            _route_manager.execute_route(route_id)
        except KeyError:
            raise HTTPException(status_code=404, detail="Route not found")
        return {"status": "queued"}

    @app.post("/api/robot/cmd_vel")
    async def robot_cmd_vel(command: TwistCommand):
        """Send velocity command (API document compliant)."""
        try:
            _robot_sdk.send_twist(command.vel_x, command.vel_y, command.vel_theta)
            return {
                "Result": 0,
                "Error": ""
            }
        except Exception as e:
            return {
                "Result": 4,
                "Error": str(e)
            }
    
    @app.post("/api/robot/control")
    async def robot_control(command: TwistCommand):
        """Send velocity command to /cmd_vel topic (ros_web_server compatible)."""
        try:
            _ros_bridge.publish_cmd_vel({
                "vel_x": command.vel_x,
                "vel_y": command.vel_y,
                "vel_theta": command.vel_theta
            })
            return {
                "Result": 0,
                "Error": ""
            }
        except Exception as e:
            return {
                "Result": 4,
                "Error": str(e)
            }

    @app.post("/api/robot/mode/stand")
    async def robot_mode_stand():
        _robot_sdk.send_stand_mode()
        return {"status": "ok"}

    @app.post("/api/robot/mode/walk")
    async def robot_mode_walk():
        _robot_sdk.send_walk_mode()
        return {"status": "ok"}

    @app.post("/api/robot/mode/sit")
    async def robot_mode_sit():
        _robot_sdk.send_sit_mode()
        return {"status": "ok"}

    @app.post("/api/robot/mode/stair")
    async def robot_mode_stair(payload: StairModeRequest):
        _robot_sdk.send_stair_mode(payload.enable)
        return {"status": "ok"}

    @app.post("/api/robot/emergency_stop")
    async def robot_emergency_stop():
        _robot_sdk.send_emergency_stop()
        return {"status": "ok"}

    @app.post("/api/robot/body_height")
    async def robot_body_height(payload: dict):
        """Adjust robot body height by 5cm increments.
        
        Args:
            direction: 1 for increase (+5cm), -1 for decrease (-5cm)
        """
        try:
            direction = payload.get("direction", 1)
            if direction not in [1, -1]:
                return {"status": "error", "message": "Direction must be 1 (up) or -1 (down)"}
            
            _robot_sdk.send_body_height(direction)
            return {"status": "ok", "direction": direction, "change_cm": direction * 5}
        except Exception as e:
            logger.error(f"Failed to adjust body height: {e}")
            return {"status": "error", "message": str(e)}

    @app.post("/api/robot/mode/recover")
    async def robot_mode_recover():
        """Recovery mode - stand up from fall.
        
        When robot falls over, call this to make it automatically stand up
        and return to walk mode.
        """
        try:
            _robot_sdk.send_recover()
            return {"status": "ok"}
        except Exception as e:
            logger.error(f"Failed to send recovery command: {e}")
            return {"status": "error", "message": str(e)}
    
    @app.post("/api/robot/pause_navigation")
    async def pause_navigation():
        """Pause navigation (ros_web_server compatible)."""
        try:
            _ros_bridge.publish_pause()
            return {
                "Result": 0,
                "Error": ""
            }
        except Exception as e:
            return {
                "Result": 4,
                "Error": str(e)
            }
    
    @app.post("/api/robot/resume_navigation")
    async def resume_navigation():
        """Resume navigation (ros_web_server compatible)."""
        try:
            _ros_bridge.publish_resume()
            return {
                "Result": 0,
                "Error": ""
            }
        except Exception as e:
            return {
                "Result": 4,
                "Error": str(e)
            }
    
    @app.post("/api/inspection/callback")
    async def inspection_callback(callback_data: dict = None):
        """Handle inspection task callback (ros_web_server compatible)."""
        try:
            # If no callback_data provided, get from current navigation status
            if callback_data is None:
                nav_status = _ros_bridge._latest_nav_status
                if nav_status in ["SUCCEEDED", "FAILED"]:
                    # Use current_task_id for ros_web_server compatibility
                    task_id = _ros_bridge.current_task_id if _ros_bridge.current_task_id is not None else int(time.time())
                    callback_data = {
                        "robot_id": 1,  # Default robot ID
                        "task_id": task_id,
                        "execution_status": nav_status,
                        "execution_time": int(time.time() * 1000)  # 13-digit timestamp
                    }
                else:
                    return {
                        "code": 400,
                        "msg": f"Current nav_status '{nav_status}' is not a completion status"
                    }
            
            # Validate required fields
            required_fields = ["robot_id", "task_id", "execution_status", "execution_time"]
            for field in required_fields:
                if field not in callback_data:
                    return {
                        "code": 400,
                        "msg": f"Missing required field: {field}"
                    }
            
            robot_id = callback_data["robot_id"]
            task_id = callback_data["task_id"]
            execution_status = callback_data["execution_status"]
            execution_time = callback_data["execution_time"]
            
            # Validate execution_status
            valid_statuses = ["SUCCEEDED", "FAILED", "success"]
            if execution_status not in valid_statuses:
                return {
                    "code": 400,
                    "msg": f"Invalid execution_status. Must be one of: {valid_statuses}"
                }
            
            # Validate execution_time (13-digit timestamp)
            if not isinstance(execution_time, int) or execution_time < 1000000000000 or execution_time > 9999999999999:
                return {
                    "code": 400,
                    "msg": "Invalid execution_time. Must be a 13-digit UNIX timestamp in milliseconds"
                }
            
            # Log callback info
            logger.info(f"收到导航结果回调:")
            logger.info(f"  机器人ID: {robot_id}")
            logger.info(f"  任务ID: {task_id}")
            logger.info(f"  执行状态: {execution_status}")
            logger.info(f"  执行时间: {execution_time}")
            
            return {
                "code": 200,
                "msg": "success"
            }
            
        except Exception as e:
            logger.error(f"处理导航结果回调时发生错误: {str(e)}")
            return {
                "code": 500,
                "msg": f"Internal server error: {str(e)}"
            }
    
    @app.get("/api/inspection/nav_status")
    async def get_nav_status_for_callback():
        """Get navigation status for inspection callback (ros_web_server compatible)."""
        nav_status = _ros_bridge._latest_nav_status
        if nav_status:
            return {
                "code": 200,
                "msg": "success",
                "nav_status": nav_status,
                "can_trigger_callback": nav_status in ["SUCCEEDED", "FAILED"],
                "current_task_id": _ros_bridge.current_task_id,  # Use current_task_id for compatibility
                "timestamp": int(time.time() * 1000)
            }
        else:
            return {
                "code": 400,
                "msg": "No nav_status data available from ROS"
            }

    @app.get("/api/localizer/logs")
    async def get_localizer_logs():
        """Get recent localizer logs from the navigation system."""
        import subprocess
        try:
            log_file = "/tmp/navigation_3_localizer.log"
            # 读取最后10000行，过滤出关键信息，保留更多日志记录
            # 包含SC、Scan Context、图匹配、TEASER++等关键词
            result = subprocess.run(
                f"tail -10000 {log_file} 2>/dev/null | grep -E '(粗匹配|精匹配|ICP匹配|全局配准|评分|SC|Scan Context|图匹配|TEASER|对应关系|Match Found|定位成功|定位失败)' | tail -5000",
                shell=True,
                capture_output=True,
                text=True
            )
            if result.returncode == 0 and result.stdout.strip():
                logs = result.stdout.strip().split('\n')
                # 清理日志格式
                cleaned_logs = []
                for log in logs:
                    # 移除ROS2日志前缀，保留核心信息
                    if '[localizer_node-1]' in log:
                        log = log.split('[localizer_node-1]')[1].strip()
                    # 移除时间戳前缀
                    if '] [' in log:
                        parts = log.split('] ', 1)
                        if len(parts) > 1:
                            log = parts[1]
                    # 保留所有匹配的日志行（包括没有[localizer_node-1]前缀的）
                    if log.strip():  # 只要不是空行就保留
                        cleaned_logs.append(log)
                return {"logs": cleaned_logs}  # 返回所有过滤后的日志，不再限制行数
            else:
                return {"logs": ["等待定位数据..."]}
        except Exception as e:
            logger.error(f"Failed to read localizer logs: {e}")
            return {"logs": [f"读取日志失败: {str(e)}"]}

    @app.get("/api/robot/matching_clouds")
    async def get_matching_clouds():
        """Get matching process visualization: source & target point clouds."""
        try:
            clouds = _ros_bridge.get_matching_clouds()
            return {
                "Result": 0,
                "Error": "",
                **clouds
            }
        except Exception as e:
            logger.error(f"Failed to get matching clouds: {e}")
            return {
                "Result": 1,
                "Error": str(e),
                "teaser_source": None,
                "teaser_target": None,
                "rough_source": None,
                "rough_target": None,
                "refine_source": None,
                "refine_target": None
            }

    @app.get("/api/docs", include_in_schema=False)
    async def api_documentation() -> FileResponse:
        """Serve the API documentation page."""
        docs_file = STATIC_DIR / "api_docs.html"
        if not docs_file.exists():
            raise HTTPException(status_code=404, detail="api_docs.html not found")
        return FileResponse(docs_file)

    @app.get("/", include_in_schema=False)
    async def index() -> FileResponse:
        """Serve the single page application entrypoint."""

        index_file = STATIC_DIR / "index.html"
        if not index_file.exists():
            raise HTTPException(status_code=404, detail="index.html not found")
        return FileResponse(index_file)

    return app


app = create_app()


if __name__ == "__main__":
    import uvicorn

    uvicorn.run("web_server.mapping_nav_server:app", host="0.0.0.0", port=8800)

