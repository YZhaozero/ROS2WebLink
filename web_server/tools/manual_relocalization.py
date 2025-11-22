import asyncio
import json
import logging
import math
import threading
import time
import struct
from pathlib import Path
from typing import Any, Dict, List, Optional

import numpy as np
import pypcd4
import transforms3d
from fastapi import APIRouter, WebSocket, WebSocketDisconnect
from starlette.websockets import WebSocketState

logger = logging.getLogger(__name__)

class ManualRelocalizationTool:
    def __init__(self, ros_bridge):
        self.ros_bridge = ros_bridge
        self.router = APIRouter()
        
        # Get map path from ROS parameter (same as localizer uses)
        self.map_path = self._get_map_path_from_ros()
        logger.info(f"Using map path from ROS: {self.map_path}")
        
        # Pose State (World Frame)
        self.pose = {
            "x": 0.0, "y": 0.0, "z": 0.0,
            "roll": 0.0, "pitch": 0.0, "yaw": 0.0
        }
        
        # Data State
        self.map_points: Optional[bytes] = None # Downsampled, float32 bytes
        self.scan_points: Optional[np.ndarray] = None # Raw scan points
        self.scan_frame_id: Optional[str] = None # Frame ID of scan points
        self.scan_lock = threading.Lock()
        
        # TF Buffer for coordinate transformation
        self.tf_buffer = None
        self.tf_listener = None
        self._setup_tf()
        
        # WebSocket Connections
        self.active_connections: List[WebSocket] = []
        
        # Load Map asynchronously
        threading.Thread(target=self._load_map, daemon=True).start()
        
        # Setup ROS Subscription
        self._setup_ros()
        
        # Setup Routes
        self.router.add_api_websocket_route("/ws/manual_reloc", self.websocket_endpoint)
    
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

    def _load_map(self):
        logger.info(f"Loading map from {self.map_path}...")
        try:
            path = Path(self.map_path)
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
            
            # Downsample to ~100k points for web visualization
            # This is critical for browser performance
            target_size = 100000
            if len(points_threejs) > target_size:
                indices = np.random.choice(len(points_threejs), target_size, replace=False)
                points_threejs = points_threejs[indices]
                
            # Convert to flat float32 buffer: [x, y, z, x, y, z, ...]
            self.map_points = points_threejs.astype(np.float32).tobytes()
            logger.info(f"Map loaded: {len(points_threejs)} points (transformed to Three.js coordinates)")
            
        except Exception as e:
            logger.error(f"Failed to load map: {e}")
            import traceback
            logger.error(traceback.format_exc())

    def _setup_ros(self):
        # Subscribe to Livox points
        try:
            from sensor_msgs.msg import PointCloud2
            from rclpy.qos import QoSProfile, ReliabilityPolicy
            import sensor_msgs_py.point_cloud2 as pc2
            from tf2_ros import Buffer, TransformListener
            import tf2_geometry_msgs
            
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

    async def websocket_endpoint(self, websocket: WebSocket):
        await websocket.accept()
        self.active_connections.append(websocket)
        logger.info("Manual Reloc Client Connected")
        
        try:
            # Wait for map to load (max 5 seconds)
            max_wait = 5.0
            waited = 0.0
            while not self.map_points and waited < max_wait:
                await asyncio.sleep(0.1)
                waited += 0.1
            
            # Send Map if available
            if self.map_points:
                # Ensure header is exactly 4 bytes
                header = b'MAP\x00'  # 4 bytes: 'M', 'A', 'P', null
                logger.info(f"Sending map to client ({len(self.map_points)//12} points, {len(self.map_points)} bytes)")
                await websocket.send_bytes(header + self.map_points)
            else:
                logger.warning("Map not loaded yet, client will not see map")
            
            # Send Initial Pose
            await websocket.send_json({"type": "pose", "data": self.pose})
            
            # Start broadcast loop for this client
            # We use a task to push updates while waiting for input
            broadcast_task = asyncio.create_task(self._client_broadcast_loop(websocket))
            
            while True:
                data = await websocket.receive_text()
                msg = json.loads(data)
                
                if msg["type"] == "move":
                    self._handle_move(msg)
                elif msg["type"] == "set_pose":
                    # Allow setting absolute pose
                    new_pose = msg.get("data", {})
                    for k, v in new_pose.items():
                        if k in self.pose:
                            self.pose[k] = float(v)
                elif msg["type"] == "reset":
                    self.pose = {k: 0.0 for k in self.pose}
                elif msg["type"] == "confirm":
                    self._publish_pose()
                    
        except WebSocketDisconnect:
            logger.info("Manual Reloc Client Disconnected")
        except Exception as e:
            logger.error(f"WebSocket error: {e}")
        finally:
            if websocket in self.active_connections:
                self.active_connections.remove(websocket)
            # broadcast_task will be cancelled when websocket closes and loop raises error

    def _handle_move(self, msg):
        # msg: {axis: 'x', value: 0.1}
        axis = msg.get("axis")
        value = float(msg.get("value", 0.0))
        
        # Handle relative movement
        if axis in self.pose:
            self.pose[axis] += value

    def _get_current_robot_pose(self) -> Optional[Dict[str, float]]:
        """
        Get current robot pose from TF or odom.
        Returns pose in map frame: {x, y, z, roll, pitch, yaw} or None if unavailable.
        """
        # Try to get from TF first (most accurate)
        if self.tf_buffer:
            try:
                import rclpy
                from rclpy.duration import Duration
                
                # Try to get transform from map to base_link
                transform = self.tf_buffer.lookup_transform(
                    "map",
                    "base_link",
                    rclpy.time.Time(),
                    timeout=Duration(seconds=0.5)
                )
                
                t = transform.transform.translation
                q = transform.transform.rotation
                
                # Convert quaternion to euler angles
                quat = [q.w, q.x, q.y, q.z]
                roll, pitch, yaw = transforms3d.euler.quat2euler(quat, axes='sxyz')
                
                current_pose = {
                    "x": float(t.x),
                    "y": float(t.y),
                    "z": float(t.z),
                    "roll": float(roll),
                    "pitch": float(pitch),
                    "yaw": float(yaw)
                }
                
                logger.info(f"Got current pose from TF: x={current_pose['x']:.3f}, y={current_pose['y']:.3f}, z={current_pose['z']:.3f}, yaw={current_pose['yaw']:.3f}")
                return current_pose
                
            except Exception as e:
                logger.debug(f"TF lookup failed: {e}, trying odom")
        
        # Fallback: get from odom message
        if hasattr(self.ros_bridge, 'latest_odom') and self.ros_bridge.latest_odom:
            try:
                msg = self.ros_bridge.latest_odom
                
                # Only use if it's in map frame
                if msg.header.frame_id == "map":
                    x = msg.pose.pose.position.x
                    y = msg.pose.pose.position.y
                    z = msg.pose.pose.position.z
                    
                    q = msg.pose.pose.orientation
                    quat = [q.w, q.x, q.y, q.z]
                    roll, pitch, yaw = transforms3d.euler.quat2euler(quat, axes='sxyz')
                    
                    current_pose = {
                        "x": float(x),
                        "y": float(y),
                        "z": float(z),
                        "roll": float(roll),
                        "pitch": float(pitch),
                        "yaw": float(yaw)
                    }
                    
                    logger.info(f"Got current pose from odom: x={current_pose['x']:.3f}, y={current_pose['y']:.3f}, z={current_pose['z']:.3f}, yaw={current_pose['yaw']:.3f}")
                    return current_pose
            except Exception as e:
                logger.debug(f"Failed to get pose from odom: {e}")
        
        logger.warning("Could not get current robot pose, will use target pose directly")
        return None
    
    def _compute_transform_matrix(self, pose: Dict[str, float]) -> tuple:
        """
        Convert pose to 4x4 transformation matrix.
        Returns (R, T) where R is 3x3 rotation matrix and T is 3x1 translation vector.
        """
        T = np.array([pose["x"], pose["y"], pose["z"]])
        R = transforms3d.euler.euler2mat(
            pose["roll"], pose["pitch"], pose["yaw"],
            axes='sxyz'
        )
        return R, T
    
    def _apply_transform(self, R: np.ndarray, T: np.ndarray, pose: Dict[str, float]) -> Dict[str, float]:
        """
        Apply transformation matrix to a pose.
        Returns new pose after transformation.
        """
        # Convert pose to matrix form
        pose_T = np.array([pose["x"], pose["y"], pose["z"]])
        pose_R = transforms3d.euler.euler2mat(
            pose["roll"], pose["pitch"], pose["yaw"],
            axes='sxyz'
        )
        
        # Apply transformation: new_pose = R * pose + T
        # For pose transformation: new_R = R * pose_R, new_T = R * pose_T + T
        new_R = np.dot(R, pose_R)
        new_T = np.dot(R, pose_T) + T
        
        # Convert back to euler angles
        roll, pitch, yaw = transforms3d.euler.mat2euler(new_R, axes='sxyz')
        
        return {
            "x": float(new_T[0]),
            "y": float(new_T[1]),
            "z": float(new_T[2]),
            "roll": float(roll),
            "pitch": float(pitch),
            "yaw": float(yaw)
        }
    
    def _publish_pose(self):
        """
        Publish pose using transform matrix approach.
        Instead of directly setting initial pose, we:
        1. Get current robot pose
        2. Compute transform matrix from current to target
        3. Apply transform to get final pose
        4. Publish final pose
        """
        # Target pose (what user wants)
        target_pose = {
            "x": self.pose["x"],
            "y": self.pose["y"],
            "z": self.pose["z"],
            "roll": self.pose["roll"],
            "pitch": self.pose["pitch"],
            "yaw": self.pose["yaw"]
        }
        
        try:
            # Get current robot pose
            current_pose = self._get_current_robot_pose()
            
            if current_pose is None:
                # Fallback: if we can't get current pose, use target pose directly
                logger.warning("Using target pose directly (could not get current pose)")
                final_pose = target_pose
            else:
                # Compute transform matrix from current to target
                # We want: T_final = T_target * inv(T_current) * T_current = T_target
                # So the relative transform is: T_relative = T_target * inv(T_current)
                
                # Get current pose transform (map -> base_link)
                R_current, T_current = self._compute_transform_matrix(current_pose)
                
                # Get target pose transform (map -> base_link)
                R_target, T_target = self._compute_transform_matrix(target_pose)
                
                # Compute relative transform: T_relative = T_target * inv(T_current)
                # For SE(3): inv([R, T]) = [R^T, -R^T * T]
                R_current_inv = R_current.T
                T_current_inv = -np.dot(R_current_inv, T_current)
                
                # Compose transforms: T_relative = T_target * T_current_inv
                # [R_relative, T_relative] = [R_target * R_current_inv, R_target * T_current_inv + T_target]
                R_relative = np.dot(R_target, R_current_inv)
                T_relative = np.dot(R_target, T_current_inv) + T_target
                
                # Apply relative transform to current pose to get final pose
                # This compensates for robot movement during delay
                # final_pose = T_relative * current_pose = T_target * inv(T_current) * T_current = T_target
                final_pose = self._apply_transform(R_relative, T_relative, current_pose)
                
                logger.info(
                    f"Transform matrix computed: "
                    f"current=({current_pose['x']:.3f}, {current_pose['y']:.3f}, {current_pose['z']:.3f}, yaw={current_pose['yaw']:.3f}), "
                    f"target=({target_pose['x']:.3f}, {target_pose['y']:.3f}, {target_pose['z']:.3f}, yaw={target_pose['yaw']:.3f}), "
                    f"final=({final_pose['x']:.3f}, {final_pose['y']:.3f}, {final_pose['z']:.3f}, yaw={final_pose['yaw']:.3f})"
                )
            
            # Publish final pose
            self.ros_bridge.publish_initial_pose(
                final_pose["x"], 
                final_pose["y"], 
                final_pose["z"], 
                final_pose["yaw"], 
                roll=final_pose["roll"], 
                pitch=final_pose["pitch"]
            )
            logger.info(
                f"Published pose via transform matrix: x={final_pose['x']:.3f}, y={final_pose['y']:.3f}, "
                f"z={final_pose['z']:.3f}, roll={final_pose['roll']:.3f}, pitch={final_pose['pitch']:.3f}, "
                f"yaw={final_pose['yaw']:.3f}"
            )
            
        except Exception as e:
            logger.error(f"Failed to publish pose with transform: {e}")
            import traceback
            logger.error(traceback.format_exc())
            # Fallback: try direct publish
            try:
                self.ros_bridge.publish_initial_pose(
                    target_pose["x"], 
                    target_pose["y"], 
                    target_pose["z"], 
                    target_pose["yaw"], 
                    roll=target_pose["roll"], 
                    pitch=target_pose["pitch"]
                )
                logger.info("Fallback: Published target pose directly")
            except Exception as e2:
                logger.error(f"Fallback publish also failed: {e2}")

    async def _client_broadcast_loop(self, ws: WebSocket):
        """Push live scan updates to client."""
        try:
            while ws.client_state == WebSocketState.CONNECTED:
                start_time = time.time()
                
                # 1. Get latest scan
                with self.scan_lock:
                    scan = self.scan_points.copy() if self.scan_points is not None else None
                    scan_frame_id = self.scan_frame_id
                
                if scan is not None:
                    # 2. Transform scan to map frame using TF (if needed)
                    scan_in_map = scan.copy()
                    
                    # If scan is not in map frame, transform it using TF
                    if scan_frame_id and scan_frame_id != "map" and self.tf_buffer:
                        try:
                            import rclpy
                            from rclpy.duration import Duration
                            
                            # Get transform from scan_frame to map
                            transform = self.tf_buffer.lookup_transform(
                                "map",
                                scan_frame_id,
                                rclpy.time.Time(),
                                timeout=Duration(seconds=0.2)
                            )
                            
                            # Extract transform matrix
                            t = transform.transform.translation
                            q = transform.transform.rotation
                            
                            # Convert quaternion to rotation matrix
                            R_tf = transforms3d.quaternions.quat2mat([q.w, q.x, q.y, q.z])
                            T_tf = np.array([t.x, t.y, t.z])
                            
                            # Apply TF transform: P_map = R_tf * P_scan + T_tf
                            scan_in_map = np.dot(scan, R_tf.T) + T_tf
                            
                        except Exception as e:
                            logger.debug(f"TF transform failed ({scan_frame_id}->map): {e}, using direct transform")
                            # If TF fails, assume scan is already close to map frame
                            scan_in_map = scan
                    
                    # 3. Apply user-adjusted pose transform to preview alignment
                    # The pose is map->base_link (initial pose user wants to set)
                    # The scan is now in map frame (after TF conversion)
                    # We apply the pose transform to show how scan aligns with map at this pose
                    # Note: This is for preview only - the actual pose will be sent to ROS
                    
                    # Get translation (map frame)
                    T_pose = np.array([self.pose["x"], self.pose["y"], self.pose["z"]])
                    
                    # Get rotation matrix (map frame, roll-pitch-yaw)
                    R_pose = transforms3d.euler.euler2mat(
                        self.pose["roll"], self.pose["pitch"], self.pose["yaw"], 
                        axes='sxyz'
                    )
                    
                    # Apply pose transform to preview alignment
                    # This shows where the scan would be when robot is at this pose
                    transformed_scan_map = np.dot(scan_in_map, R_pose.T) + T_pose
                    
                    # 3. Convert to Three.js coordinates for visualization
                    # Use the SAME transformation as pointcloud-viewer.js:
                    # Three.js X = ROS X
                    # Three.js Y = ROS Z (Z轴向上)
                    # Three.js Z = -ROS Y (Y轴反向)
                    transformed_scan_threejs = np.zeros_like(transformed_scan_map)
                    transformed_scan_threejs[:, 0] = transformed_scan_map[:, 0]   # X: same
                    transformed_scan_threejs[:, 1] = transformed_scan_map[:, 2]   # Y: Z -> Y (Z轴向上)
                    transformed_scan_threejs[:, 2] = -transformed_scan_map[:, 1]  # Z: -Y -> Z (Y轴反向)
                    
                    # 4. Send
                    scan_bytes = transformed_scan_threejs.astype(np.float32).tobytes()
                    
                    # Send Pose Update (for UI numbers)
                    await ws.send_json({
                        "type": "update", 
                        "pose": self.pose,
                        "scan_count": len(scan)
                    })
                    
                    # Send Binary Scan (ensure header is 4 bytes)
                    scan_header = b'SCAN'  # Already 4 bytes
                    await ws.send_bytes(scan_header + scan_bytes)
                
                # Cap at 10Hz
                elapsed = time.time() - start_time
                delay = max(0.1 - elapsed, 0.01)
                await asyncio.sleep(delay)
                
        except Exception as e:
            # WebSocket closed or other error
            pass

