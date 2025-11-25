import asyncio
import json
import logging
import math
import threading
import time
import struct
import base64
from pathlib import Path
from typing import Any, Dict, List, Optional

import numpy as np
import pypcd4
import transforms3d
from fastapi import APIRouter, Response
from pydantic import BaseModel

logger = logging.getLogger(__name__)

class PoseModel(BaseModel):
    x: float
    y: float
    z: float
    roll: float
    pitch: float
    yaw: float

class ConfirmPoseRequest(BaseModel):
    pose: PoseModel
    ref_odom: Optional[PoseModel] = None

class ManualRelocalizationTool:
    def __init__(self, ros_bridge):
        self.ros_bridge = ros_bridge
        self.router = APIRouter()
        
        # Get map path from ROS parameter (same as localizer uses)
        self.map_path = self._normalize_map_path(self._get_map_path_from_ros())
        logger.info(f"Using map path from ROS: {self.map_path}")
        
        # Data State
        self.map_points: Optional[bytes] = None # Downsampled, float32 bytes
        self.scan_points: Optional[np.ndarray] = None # Raw scan points
        self.scan_frame_id: Optional[str] = None # Frame ID of scan points
        self.scan_lock = threading.Lock()
        self._map_reload_lock = threading.Lock()
        self._map_file_mtime: Optional[float] = None
        self._loaded_map_path: Optional[str] = None
        
        # TF Buffer for coordinate transformation
        self.tf_buffer = None
        self.tf_listener = None
        self._setup_tf()
        
        # Load Map asynchronously
        threading.Thread(target=self._load_map, kwargs={"map_path": self.map_path}, daemon=True).start()
        
        # Setup ROS Subscription
        self._setup_ros()
        
        # Setup Routes
        self.router.add_api_route("/api/manual_reloc/map", self.get_map, methods=["GET"])
        self.router.add_api_route("/api/manual_reloc/capture", self.get_capture, methods=["GET"])
        self.router.add_api_route("/api/manual_reloc/confirm", self.confirm_pose, methods=["POST"])
    
    def _setup_tf(self):
        """Setup TF buffer and listener for coordinate transformation."""
        try:
            import rclpy
            from tf2_ros import Buffer, TransformListener
            
            # Get the ROS node from ros_bridge
            if hasattr(self.ros_bridge, "_adapter") and hasattr(self.ros_bridge._adapter, "node"):
                node = self.ros_bridge._adapter.node
                self.tf_buffer = Buffer()
                self.tf_listener = TransformListener(self.tf_buffer, node, spin_thread=True)
                logger.info("TF Buffer and Listener initialized")
            else:
                logger.warning("Cannot setup TF: ROS adapter node not available")
        except Exception as e:
            logger.error(f"Failed to setup TF: {e}")
            self.tf_buffer = None
            self.tf_listener = None
    
    def _get_map_path_from_ros(self) -> str:
        """Get current map path from ROS parameter (same as localizer node uses)."""
        try:
            # Try to get from ROS parameter server
            if hasattr(self.ros_bridge, "_adapter") and hasattr(self.ros_bridge._adapter, "node"):
                node = self.ros_bridge._adapter.node
                # Declare and get parameter
                if not node.has_parameter("default_map_path"):
                    node.declare_parameter("default_map_path", "")
                map_path = node.get_parameter("default_map_path").get_parameter_value().string_value
                if map_path:
                    logger.info(f"Got map path from ROS parameter: {map_path}")
                    return map_path
        except Exception as e:
            logger.warning(f"Failed to get map path from ROS parameter: {e}")
        
        # Fallback: read from config file
        try:
            import yaml
            config_path = Path("/home/guest/tron_ros2/src/tron_slam/localizer/config/localizer.yaml")
            if config_path.exists():
                with open(config_path, 'r') as f:
                    config = yaml.safe_load(f)
                    if config and "default_map_path" in config:
                        map_path = config["default_map_path"]
                        logger.info(f"Got map path from config file: {map_path}")
                        return map_path
        except Exception as e:
            logger.warning(f"Failed to read config file: {e}")
        
        # Final fallback
        default_path = "/home/guest/tron_ros2/src/tron_slam/localizer/PCD/map_1121_test_004.pcd"
        logger.warning(f"Using fallback map path: {default_path}")
        return default_path
    
    def _normalize_map_path(self, map_path: str) -> str:
        if not map_path:
            return ""
        try:
            return str(Path(map_path.strip()).expanduser().resolve(strict=False))
        except Exception:
            return map_path.strip()

    def _load_map(self, map_path: Optional[str] = None):
        target_path = self._normalize_map_path(map_path or self.map_path)
        if not target_path:
            logger.error("No map path available, cannot load map")
            return
        with self._map_reload_lock:
            logger.info(f"Loading map from {target_path}...")
            try:
                path = Path(target_path)
                if not path.exists():
                    logger.error(f"Map file not found at {path}!")
                    return
                    
                # Load PCD
                # pypcd4 is fast
                pc = pypcd4.PointCloud.from_path(path)
                points_ros = pc.numpy(("x", "y", "z"))  # ROS coordinates
                
                # Coordinate transformation: ROS -> Three.js
                # ROS: X forward, Y left, Z up (right-handed)
                # Three.js: X right, Y up, Z backward (right-handed, but different convention)
                # Transform: Three.js X = ROS X, Three.js Y = ROS Z, Three.js Z = -ROS Y
                points_threejs = np.zeros_like(points_ros)
                points_threejs[:, 0] = points_ros[:, 0]   # X: forward -> right (same)
                points_threejs[:, 1] = points_ros[:, 2]   # Y: up -> up (same)
                points_threejs[:, 2] = -points_ros[:, 1]  # Z: left -> backward (negate)
                
                # Downsample to ~500k points for web visualization
                # This is critical for browser performance
                target_size = 500000
                if len(points_threejs) > target_size:
                    indices = np.random.choice(len(points_threejs), target_size, replace=False)
                    points_threejs = points_threejs[indices]
                    
                # Convert to flat float32 buffer: [x, y, z, x, y, z, ...]
                self.map_points = points_threejs.astype(np.float32).tobytes()
                self.map_path = target_path
                try:
                    self._map_file_mtime = path.stat().st_mtime
                except Exception:
                    self._map_file_mtime = None
                self._loaded_map_path = target_path
                logger.info(f"Map loaded: {len(points_threejs)} points (transformed to Three.js coordinates)")
                
            except Exception as e:
                logger.error(f"Failed to load map: {e}")
                import traceback
                logger.error(traceback.format_exc())
                # Keep previous map_points if available

    async def _ensure_latest_map_loaded(self):
        """Reload the downsampled map if the source PCD has changed."""
        new_map_path = self._normalize_map_path(self._get_map_path_from_ros())
        if not new_map_path:
            return
        need_reload = False
        if self._loaded_map_path != new_map_path:
            logger.info(f"Detected map path change: {self._loaded_map_path} -> {new_map_path}")
            need_reload = True
        else:
            try:
                current_mtime = Path(new_map_path).stat().st_mtime
                if self._map_file_mtime is None or current_mtime > self._map_file_mtime:
                    logger.info("Detected map file timestamp update, reloading map cloud")
                    need_reload = True
            except FileNotFoundError:
                logger.error(f"Map file not found at {new_map_path}")
                self.map_points = None
                self._loaded_map_path = None
                return
        
        if need_reload:
            # Clear existing data so callers do not receive stale maps
            self.map_points = None
            self._loaded_map_path = None
            loop = asyncio.get_event_loop()
            await loop.run_in_executor(None, self._load_map, new_map_path)

    def _setup_ros(self):
        # Subscribe to Livox points
        try:
            from sensor_msgs.msg import PointCloud2
            from rclpy.qos import QoSProfile, ReliabilityPolicy
            import sensor_msgs_py.point_cloud2 as pc2
            
            def scan_callback(msg):
                # Parse PointCloud2
                try:
                    # Store frame_id for coordinate transformation
                    frame_id = msg.header.frame_id
                    
                    # Use read_points iterator (more compatible with different field types)
                    points_list = []
                    for point in pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True):
                        # point is a tuple: (x, y, z)
                        points_list.append([float(point[0]), float(point[1]), float(point[2])])
                    
                    if len(points_list) == 0:
                        return
                    
                    points = np.array(points_list, dtype=np.float32)
                    
                    with self.scan_lock:
                        self.scan_points = points
                        self.scan_frame_id = frame_id
                    
                    # Downsample live scan (keep ~5000 points)
                    if len(points) > 5000:
                        indices = np.random.choice(len(points), 5000, replace=False)
                        points = points[indices]
                    
                    with self.scan_lock:
                        self.scan_points = points
                    logger.debug(f"Received scan: {len(points)} points")
                except Exception as e:
                    logger.error(f"Error processing scan: {e}")
                    import traceback
                    logger.error(traceback.format_exc())
            
            # Use BEST_EFFORT for Livox
            qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT)
            
            # Assuming ros_bridge has the internal adapter
            if hasattr(self.ros_bridge, "_adapter"):
                # Try to subscribe to the same point cloud topic as localizer uses
                # This ensures we use the same coordinate system
                localizer_topic = "/dlio/odom_node/pointcloud/deskewed"
                try:
                    # Use RELIABLE QoS for DLIO topics
                    dlio_qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.RELIABLE)
                    self.ros_bridge._adapter.create_subscription(
                        PointCloud2, localizer_topic, scan_callback, dlio_qos
                    )
                    logger.info(f"Subscribed to {localizer_topic} for manual reloc (same as localizer)")
                except Exception as e:
                    logger.warning(f"Failed to subscribe to {localizer_topic}: {e}, trying /livox/points")
                    # Fallback to /livox/points
                    try:
                        self.ros_bridge._adapter.create_subscription(
                            PointCloud2, "/livox/points", scan_callback, qos
                        )
                        logger.info("Subscribed to /livox/points for manual reloc")
                    except:
                        # Final fallback
                        self.ros_bridge._adapter.create_subscription(
                            PointCloud2, "/livox/lidar/pointcloud", scan_callback, qos
                        )
                        logger.info("Subscribed to /livox/lidar/pointcloud for manual reloc")
            else:
                logger.error("ROS Bridge does not have _adapter attribute!")
                
        except ImportError:
            logger.error("ROS messages not available, cannot subscribe")
        except Exception as e:
            logger.error(f"Setup ROS error: {e}")

    async def get_map(self):
        """HTTP endpoint to get the map point cloud."""
        await self._ensure_latest_map_loaded()
        if not self.map_points:
            return Response(status_code=503, content="Map not loaded yet")
        
        headers = {"Cache-Control": "no-store, max-age=0"}
        return Response(content=self.map_points, media_type="application/octet-stream", headers=headers)

    async def get_capture(self):
        """
        HTTP endpoint to capture current scan and odom.
        Returns JSON with:
        - scan: Base64 encoded float32 buffer (transformed to Three.js coords)
        - odom: Current Odom pose (odom -> base_link)
        - pose: Current Map pose (map -> base_link) for initial guess
        """
        # 1. Get latest scan (with timeout)
        scan = None
        scan_frame_id = None
        
        # Try to get scan for up to 2 seconds
        for _ in range(20):
            with self.scan_lock:
                if self.scan_points is not None:
                    scan = self.scan_points.copy()
                    scan_frame_id = self.scan_frame_id
                    break
            await asyncio.sleep(0.1)
        
        if scan is None:
             logger.warning("Capture failed: No scan data received after timeout")
             return Response(status_code=404, content="No scan data available yet (timeout)")
        
        # 2. Transform scan to map frame using TF (if needed)
        # Note: We want the scan in the robot's LOCAL frame (base_link) usually, 
        # but for the visualizer, we might want it in map frame relative to where the robot IS.
        # Actually, the visualizer logic was:
        # Transformed Scan = (Scan in Map) * R_pose.T + T_pose
        # Wait, the visualizer logic was confusing.
        # Let's simplify: Return scan in ROBOT frame (base_link).
        # The frontend will rotate/translate it based on user's manual pose.
        
        # BUT, the scan we receive might be in 'livox_frame' or 'lidar_frame'.
        # We should transform it to 'base_link' first.
        scan_in_base = scan
        
        # Critical: Ensure we are transforming to base_link, otherwise calibration is wrong
        if self.tf_buffer:
            try:
                # If frame_id not provided, assume livox_frame (fallback)
                source_frame = scan_frame_id if scan_frame_id else "livox_frame"
                
                if source_frame != "base_link":
                    import rclpy
                    from rclpy.duration import Duration
                    
                    # Get transform from source_frame to base_link
                    transform = self.tf_buffer.lookup_transform(
                        "base_link",
                        source_frame,
                        rclpy.time.Time(),
                        timeout=Duration(seconds=0.2)
                    )
                    t = transform.transform.translation
                    q = transform.transform.rotation
                    R_tf = transforms3d.quaternions.quat2mat([q.w, q.x, q.y, q.z])
                    T_tf = np.array([t.x, t.y, t.z])
                    
                    # P_base = R * P_source + T
                    scan_in_base = np.dot(scan, R_tf.T) + T_tf
                    logger.info(f"Transformed scan from {source_frame} to base_link")
            except Exception as e:
                logger.error(f"CRITICAL: TF transform failed ({scan_frame_id}->base_link): {e}")
                return Response(status_code=500, content=f"TF Error: Could not transform scan to base_link. Check TF tree. Error: {e}")
        else:
             logger.warning("TF Buffer not ready, using raw scan (might be inaccurate if not in base_link)")
        
        # 3. Convert to Three.js coordinates (for visualization convenience)
        # ROS: X forward, Y left, Z up
        # Three.js: X right, Y up, Z backward (Standard Three.js)
        # OUR Three.js setup in frontend:
        # Map points were converted: X->X, Y->Z, Z->-Y
        # So we should do the same for scan points
        scan_threejs = np.zeros_like(scan_in_base)
        scan_threejs[:, 0] = scan_in_base[:, 0]   # X: same
        scan_threejs[:, 1] = scan_in_base[:, 2]   # Y: Z -> Y
        scan_threejs[:, 2] = -scan_in_base[:, 1]  # Z: -Y -> Z
        
        scan_b64 = base64.b64encode(scan_threejs.astype(np.float32).tobytes()).decode('utf-8')
        
        # 4. Get Odom and Map Pose
        odom_pose = self._get_current_odom_pose()
        map_pose = self._get_current_robot_pose()
        
        if not map_pose:
            # Default to 0
            map_pose = {"x": 0, "y": 0, "z": 0, "roll": 0, "pitch": 0, "yaw": 0}
            
        return {
            "scan": scan_b64,
            "odom": odom_pose,
            "pose": map_pose
        }

    async def confirm_pose(self, req: ConfirmPoseRequest):
        """HTTP endpoint to confirm pose."""
        pose_dict = req.pose.dict()
        ref_odom_dict = req.ref_odom.dict() if req.ref_odom else None
        
        self._publish_pose(pose_dict, ref_odom_dict)
        return {"status": "ok", "message": "Pose published"}

    def _get_current_odom_pose(self) -> Optional[Dict[str, float]]:
        """
        Get current robot pose in ODOM frame (odom -> base_link).
        """
        # Try to get from TF
        if self.tf_buffer:
            try:
                import rclpy
                from rclpy.duration import Duration
                transform = self.tf_buffer.lookup_transform(
                    "odom",
                    "base_link",
                    rclpy.time.Time(),
                    timeout=Duration(seconds=0.05)
                )
                t = transform.transform.translation
                q = transform.transform.rotation
                quat = [q.w, q.x, q.y, q.z]
                roll, pitch, yaw = transforms3d.euler.quat2euler(quat, axes='sxyz')
                return {
                    "x": float(t.x), "y": float(t.y), "z": float(t.z),
                    "roll": float(roll), "pitch": float(pitch), "yaw": float(yaw)
                }
            except Exception:
                pass
        
        # Fallback: ros_bridge.latest_odom
        if hasattr(self.ros_bridge, 'latest_odom') and self.ros_bridge.latest_odom:
            try:
                msg = self.ros_bridge.latest_odom
                if msg.header.frame_id == "odom":
                    x = msg.pose.pose.position.x
                    y = msg.pose.pose.position.y
                    z = msg.pose.pose.position.z
                    q = msg.pose.pose.orientation
                    quat = [q.w, q.x, q.y, q.z]
                    roll, pitch, yaw = transforms3d.euler.quat2euler(quat, axes='sxyz')
                    return {
                        "x": float(x), "y": float(y), "z": float(z),
                        "roll": float(roll), "pitch": float(pitch), "yaw": float(yaw)
                    }
            except Exception:
                pass
        return None

    def _get_current_robot_pose(self) -> Optional[Dict[str, float]]:
        """Get current robot pose from TF (map -> base_link)."""
        if self.tf_buffer:
            try:
                import rclpy
                from rclpy.duration import Duration
                transform = self.tf_buffer.lookup_transform(
                    "map",
                    "base_link",
                    rclpy.time.Time(),
                    timeout=Duration(seconds=0.5)
                )
                t = transform.transform.translation
                q = transform.transform.rotation
                quat = [q.w, q.x, q.y, q.z]
                roll, pitch, yaw = transforms3d.euler.quat2euler(quat, axes='sxyz')
                return {
                    "x": float(t.x), "y": float(t.y), "z": float(t.z),
                    "roll": float(roll), "pitch": float(pitch), "yaw": float(yaw)
                }
            except Exception:
                pass
        return None

    def _compute_transform_matrix(self, pose: Dict[str, float]) -> tuple:
        T = np.array([pose["x"], pose["y"], pose["z"]])
        R = transforms3d.euler.euler2mat(pose["roll"], pose["pitch"], pose["yaw"], axes='sxyz')
        return R, T
    
    def _apply_transform(self, R: np.ndarray, T: np.ndarray, pose: Dict[str, float]) -> Dict[str, float]:
        pose_T = np.array([pose["x"], pose["y"], pose["z"]])
        pose_R = transforms3d.euler.euler2mat(pose["roll"], pose["pitch"], pose["yaw"], axes='sxyz')
        new_R = np.dot(R, pose_R)
        new_T = np.dot(R, pose_T) + T
        roll, pitch, yaw = transforms3d.euler.mat2euler(new_R, axes='sxyz')
        return {
            "x": float(new_T[0]), "y": float(new_T[1]), "z": float(new_T[2]),
            "roll": float(roll), "pitch": float(pitch), "yaw": float(yaw)
        }

    def _publish_pose(self, pose: Dict[str, float], ref_odom: Optional[Dict[str, float]] = None):
        target_pose = pose
        final_pose = target_pose
        
        try:
            if ref_odom is not None:
                current_odom = self._get_current_odom_pose()
                if current_odom:
                    # P_final = P_target * inv(O_ref) * O_current
                    R_target, T_target = self._compute_transform_matrix(target_pose)
                    R_ref, T_ref = self._compute_transform_matrix(ref_odom)
                    R_curr, T_curr = self._compute_transform_matrix(current_odom)
                    
                    R_ref_inv = R_ref.T
                    T_ref_inv = -np.dot(R_ref_inv, T_ref)
                    
                    R_rel = np.dot(R_ref_inv, R_curr)
                    T_rel = np.dot(R_ref_inv, T_curr) + T_ref_inv
                    
                    R_final = np.dot(R_target, R_rel)
                    T_final = np.dot(R_target, T_rel) + T_target
                    
                    roll, pitch, yaw = transforms3d.euler.mat2euler(R_final, axes='sxyz')
                    final_pose = {
                        "x": float(T_final[0]), "y": float(T_final[1]), "z": float(T_final[2]),
                        "roll": float(roll), "pitch": float(pitch), "yaw": float(yaw)
                    }
                    logger.info("Freeze Mode Update Applied")
                else:
                    logger.warning("No current odom, using target pose directly")

            self.ros_bridge.publish_initial_pose(
                final_pose["x"], final_pose["y"], final_pose["z"], final_pose["yaw"], 
                roll=final_pose["roll"], pitch=final_pose["pitch"]
            )
            logger.info(f"Published pose: x={final_pose['x']:.2f}, y={final_pose['y']:.2f}")
            
        except Exception as e:
            logger.error(f"Failed to publish pose: {e}")
