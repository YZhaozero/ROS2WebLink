"""Extended ROS bridge for mapping & navigation WebUI."""

from __future__ import annotations

import base64
import math
from dataclasses import dataclass
from typing import Any, Dict, Optional

try:  # pragma: no cover - optional dependency
    import rclpy
    from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist
    from nav_msgs.msg import OccupancyGrid, Odometry
    from std_msgs.msg import String
except ImportError:  # pragma: no cover - fallback definitions for tests
    rclpy = None
    Odometry = None
    PoseWithCovarianceStamped = None
    
    @dataclass
    class PoseStamped:  # type: ignore
        @dataclass
        class Header:
            frame_id: str = ""
            stamp: Any = None
        
        @dataclass
        class Pose:
            @dataclass
            class Position:
                x: float = 0.0
                y: float = 0.0
                z: float = 0.0

            @dataclass
            class Orientation:
                x: float = 0.0
                y: float = 0.0
                z: float = 0.0
                w: float = 1.0

            position: "PoseStamped.Pose.Position" = Position()
            orientation: "PoseStamped.Pose.Orientation" = Orientation()

        header: "PoseStamped.Header" = Header()
        pose: "PoseStamped.Pose" = Pose()

    @dataclass
    class Twist:  # type: ignore
        @dataclass
        class Linear:
            x: float = 0.0
            y: float = 0.0
            z: float = 0.0

        @dataclass
        class Angular:
            x: float = 0.0
            y: float = 0.0
            z: float = 0.0

        linear: "Twist.Linear" = Linear()
        angular: "Twist.Angular" = Angular()

    @dataclass
    class OccupancyGrid:  # type: ignore
        info: Any
        data: Any

    @dataclass
    class String:  # type: ignore
        data: str = ""


class RosAdapter:
    """Adapter around rclpy Node. Allows injection for tests."""

    def __init__(self):  # pragma: no cover - not exercised in tests
        if rclpy is None:
            raise RuntimeError("rclpy is not available in this environment")
        
        # Initialize rclpy only once
        # Force ROS_DOMAIN_ID=0 to match mapping processes
        # (Mapping processes run in domain 0, so we must use domain 0 to receive map data)
        import os
        os.environ["ROS_DOMAIN_ID"] = "0"
        print(f"[DEBUG] RosAdapter: Setting ROS_DOMAIN_ID=0 (was: {os.getenv('ROS_DOMAIN_ID', 'not set')})")
        
        try:
            if not rclpy.ok():
                rclpy.init()
        except Exception as e:
            print(f"rclpy init warning: {e}")
        
        from rclpy.node import Node
        import threading

        self.node = Node("extended_ros_bridge")
        
        # Start spinning in background thread to process callbacks
        self._spin_thread = threading.Thread(target=self._spin_loop, daemon=True)
        self._spin_thread.start()
        print(f"ROS2 node started, spinning in background thread")
    
    def _spin_loop(self):
        """Spin the ROS node in a background thread."""
        try:
            rclpy.spin(self.node)
        except Exception as e:
            print(f"ROS spin error: {e}")

    def create_subscription(self, msg_type, topic: str, callback, qos: int):
        return self.node.create_subscription(msg_type, topic, callback, qos)

    def create_publisher(self, msg_type, topic: str, qos: int):
        return self.node.create_publisher(msg_type, topic, qos)


class ExtendedRosBridge:
    """Collects ROS data and exposes helper utilities for the WebUI."""

    def __init__(self, ros_adapter: Optional[RosAdapter] = None):
        self._adapter = ros_adapter or RosAdapter()
        self._subscriptions: Dict[str, Any] = {}

        self._latest_map: Optional[OccupancyGrid] = None
        self._latest_global_costmap: Optional[OccupancyGrid] = None
        self._latest_local_costmap: Optional[OccupancyGrid] = None
        self._latest_nav_status: str = "IDLE"
        self._map_origin: Optional[Dict[str, float]] = None
        
        # Sensor status tracking
        self._last_pointcloud_time: float = 0.0
        self._pointcloud_count: int = 0
        self._last_map_time: float = 0.0
        
        # Robot position tracking (from tron_commander/odom in map frame, or dlio in odom frame)
        self._robot_position: Optional[Dict[str, Any]] = None
        
        # Store raw odometry message (ros_web_server compatible)
        self.latest_odom: Optional[Any] = None  # Raw Odometry message
        
        # Navigation goal tracking (from /goal_pose topic)
        self._navigation_goal: Optional[Dict[str, float]] = None
        
        # Goal ID tracking for API compliance
        self._current_goal_id: int = 0
        self.current_task_id: Optional[int] = None  # Alias for ros_web_server compatibility
        
        # Laser scan tracking for 2D point cloud visualization
        self._latest_scan: Optional[Any] = None
        self._last_scan_time: float = 0.0
        
        # Matching result point clouds
        # Matching process visualization: source & target clouds
        self._teaser_source_cloud: Optional[Any] = None
        self._teaser_target_cloud: Optional[Any] = None
        self._teaser_correspondences: Optional[Any] = None  # 对应关系连线
        self._rough_source_cloud: Optional[Any] = None
        self._rough_target_cloud: Optional[Any] = None
        self._refine_source_cloud: Optional[Any] = None
        self._refine_target_cloud: Optional[Any] = None

        # Publishers
        self._goal_pub = self._adapter.create_publisher(PoseStamped, "/goal", 10)
        self._goal_id_pub = self._adapter.create_publisher(String, "/goal_id", 10)
        self._cmd_vel_pub = self._adapter.create_publisher(Twist, "/cmd_vel_web", 10)
        self._pause_pub = self._adapter.create_publisher(String, "/pause_navigation", 10)
        self._resume_pub = self._adapter.create_publisher(String, "/resume_navigation", 10)
        self._initialpose_pub = self._adapter.create_publisher(PoseWithCovarianceStamped, "/initialpose", 10)

        # Subscriptions
        # Use TRANSIENT_LOCAL QoS for /map to match map_server's QoS
        from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
        map_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE
        )
        self._subscriptions["/map"] = self._map_callback
        self._adapter.create_subscription(OccupancyGrid, "/map", self._map_callback, map_qos)
        
        self._register_subscription("/global_costmap/costmap", OccupancyGrid, self._global_costmap_callback)
        self._register_subscription("/local_costmap/costmap", OccupancyGrid, self._local_costmap_callback)
        self._register_subscription("/nav_status", String, self._nav_status_callback)
        self._register_subscription("/navigation_status", String, self._nav_status_callback)
        
        # Subscribe to Livox point cloud for sensor monitoring
        try:
            from sensor_msgs.msg import PointCloud2, LaserScan
            from rclpy.qos import QoSProfile, ReliabilityPolicy
            # Livox uses BEST_EFFORT QoS
            best_effort_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
            self._adapter.create_subscription(PointCloud2, "/livox/lidar/pointcloud", self._pointcloud_callback, best_effort_qos)
            self._subscriptions["/livox/lidar/pointcloud"] = self._pointcloud_callback
            # Subscribe to /scan for 2D visualization
            self._register_subscription("/scan", LaserScan, self._scan_callback)
            # Subscribe to matching process visualization clouds (source & target)
            # Note: localizer uses nested namespace /localizer/localizer/*
            self._register_subscription("/localizer/localizer/teaser_source_cloud", PointCloud2, self._teaser_source_callback)
            self._register_subscription("/localizer/localizer/teaser_target_cloud", PointCloud2, self._teaser_target_callback)
            # Subscribe to correspondences
            from visualization_msgs.msg import MarkerArray
            self._register_subscription("/localizer/localizer/teaser_correspondences", MarkerArray, self._teaser_corr_callback)
            self._register_subscription("/localizer/localizer/rough_source_cloud", PointCloud2, self._rough_source_callback)
            self._register_subscription("/localizer/localizer/rough_target_cloud", PointCloud2, self._rough_target_callback)
            self._register_subscription("/localizer/localizer/refine_source_cloud", PointCloud2, self._refine_source_callback)
            self._register_subscription("/localizer/localizer/refine_target_cloud", PointCloud2, self._refine_target_callback)
        except ImportError:
            pass  # PointCloud2/LaserScan not available in test environment
        
        # Subscribe to robot position from tron_commander (navigation mode)
        # Use RELIABLE QoS to match tron_commander's publisher
        if Odometry is not None:
            from rclpy.qos import QoSProfile, ReliabilityPolicy
            odom_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
            self._adapter.create_subscription(Odometry, "/tron_commander/odom", self._robot_odom_callback, odom_qos)
            self._subscriptions["/tron_commander/odom"] = self._robot_odom_callback
            # DLIO odom subscription is deferred - will be subscribed after DLIO health check
        
        # Subscribe to navigation goal
        if PoseStamped is not None:
            self._register_subscription("/goal_pose", PoseStamped, self._goal_pose_callback)
        
        # Flag to track if DLIO subscriptions are active
        self._dlio_subscribed = False

    # ------------------------------------------------------------------
    # Subscription registration helper
    # ------------------------------------------------------------------
    def _register_subscription(self, topic: str, msg_type, callback) -> None:
        self._subscriptions[topic] = callback
        self._adapter.create_subscription(msg_type, topic, callback, 10)

    # ------------------------------------------------------------------
    # Callbacks
    # ------------------------------------------------------------------
    def _map_callback(self, msg: OccupancyGrid) -> None:
        self._latest_map = msg
        import time
        self._last_map_time = time.time()
        print(f"[DEBUG] Map callback received: {msg.info.width}x{msg.info.height}, frame_id={msg.header.frame_id}")
        origin_pose = msg.info.origin
        self._map_origin = {
            "x": float(origin_pose.position.x),
            "y": float(origin_pose.position.y),
            "yaw": self.quaternion_to_yaw(origin_pose.orientation),
            "frame_id": msg.header.frame_id
        }

    def _global_costmap_callback(self, msg: OccupancyGrid) -> None:
        self._latest_global_costmap = msg
    
    def _pointcloud_callback(self, msg) -> None:
        """Track point cloud messages for sensor health monitoring."""
        import time
        self._last_pointcloud_time = time.time()
        self._pointcloud_count += 1
    
    def _scan_callback(self, msg) -> None:
        """Store latest laser scan for 2D visualization."""
        import time
        self._latest_scan = msg
        self._last_scan_time = time.time()
    
    def _teaser_source_callback(self, msg) -> None:
        """Store TEASER++ source point cloud."""
        self._teaser_source_cloud = msg
    
    def _teaser_target_callback(self, msg) -> None:
        """Store TEASER++ target point cloud."""
        self._teaser_target_cloud = msg
    
    def _teaser_corr_callback(self, msg) -> None:
        """Store TEASER++ correspondences."""
        self._teaser_correspondences = msg
    
    def _rough_source_callback(self, msg) -> None:
        """Store GICP rough source point cloud."""
        self._rough_source_cloud = msg
    
    def _rough_target_callback(self, msg) -> None:
        """Store GICP rough target point cloud."""
        self._rough_target_cloud = msg
    
    def _refine_source_callback(self, msg) -> None:
        """Store GICP refined source point cloud."""
        self._refine_source_cloud = msg
    
    def _refine_target_callback(self, msg) -> None:
        """Store GICP refined target point cloud."""
        self._refine_target_cloud = msg

    def _local_costmap_callback(self, msg: OccupancyGrid) -> None:
        self._latest_local_costmap = msg

    def _nav_status_callback(self, msg: String) -> None:
        self._latest_nav_status = msg.data
    
    def _robot_odom_callback(self, msg) -> None:
        """Process robot position updates with frame priority: map > odom."""
        # Store raw odometry message (ros_web_server compatible)
        # Priority: map frame > odom frame
        current_frame = self._robot_position.get("frame_id", None) if self._robot_position else None
        frame_id = msg.header.frame_id
        
        # Update latest_odom only if it's map frame, or we don't have map frame yet
        if frame_id == "map" or current_frame != "map":
            self.latest_odom = msg
        
        # Skip if incoming is odom but we already have map
        if frame_id == "odom" and current_frame == "map":
            print(f"[DEBUG] Skipping odom frame update (already have map frame)")
            return
        
        # Extract position
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        
        # Calculate yaw from quaternion
        q = msg.pose.pose.orientation
        yaw = math.atan2(2.0*(q.w*q.z + q.x*q.y), 1.0 - 2.0*(q.y*q.y + q.z*q.z))
        
        # DO NOT subtract map origin here - frontend will do it when rendering
        # Otherwise we subtract twice and robot appears off-canvas
        # Update position with frame_id
        self._robot_position = {
            "x": x,
            "y": y,
            "yaw": yaw,
            "frame_id": frame_id
        }
        print(f"[DEBUG] Robot position updated: x={x:.3f}, y={y:.3f}, yaw={yaw:.3f}, frame={frame_id}")

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------
    def quaternion_to_yaw(self, q) -> float:
        """
        Convert quaternion to yaw angle (ros_web_server compatible).
        q: geometry_msgs.msg.Quaternion
        Returns: float - yaw angle in radians
        """
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny, cosy)
    
    def get_map_json(self) -> Dict[str, Any]:
        if not self._latest_map:
            # Debug: Check if subscription exists
            print(f"[DEBUG] get_map_json: _latest_map is None, _last_map_time={self._last_map_time}")
            raise RuntimeError("map not received yet")
        return self._occupancy_grid_to_json(self._latest_map)

    def get_costmap_json(self, kind: str) -> Dict[str, Any]:
        source = {
            "global": self._latest_global_costmap,
            "local": self._latest_local_costmap,
        }.get(kind)
        if source is None:
            raise RuntimeError(f"{kind} costmap not received yet")
        return self._occupancy_grid_to_json(source)

    def get_navigation_status(self) -> Dict[str, Any]:
        """Return navigation status in API document format."""
        return {
            "status": self._latest_nav_status,
            "blocked": False,  # TODO: Implement blocked detection from ROS topics
            "goal_id": self._current_goal_id
        }
    
    def get_robot_position(self) -> Optional[Dict[str, Any]]:
        """Return robot position with frame_id (x, y, yaw, frame_id)."""
        return self._robot_position
    
    def get_scan_points(self) -> Optional[Dict[str, Any]]:
        """
        Convert latest laser scan to 2D points in map frame.
        Returns: {
            "points": [[x, y], ...],  # points in map frame
            "timestamp": float,
            "frame_id": str
        }
        """
        if self._latest_scan is None or self._robot_position is None:
            return None
        
        import numpy as np
        
        scan = self._latest_scan
        robot_pos = self._robot_position
        
        # Only return scan if it's in map frame (navigation mode)
        if robot_pos.get("frame_id") != "map":
            return None
        
        robot_x = robot_pos["x"]
        robot_y = robot_pos["y"]
        robot_yaw = robot_pos["yaw"]
        
        # Convert scan ranges to points in robot frame, then to map frame
        points = []
        angle = scan.angle_min
        for r in scan.ranges:
            if scan.range_min < r < scan.range_max:
                # Point in base_link frame (relative to robot)
                px_robot = r * math.cos(angle)
                py_robot = r * math.sin(angle)
                
                # Transform to map frame
                cos_yaw = math.cos(robot_yaw)
                sin_yaw = math.sin(robot_yaw)
                px_map = robot_x + (px_robot * cos_yaw - py_robot * sin_yaw)
                py_map = robot_y + (px_robot * sin_yaw + py_robot * cos_yaw)
                
                points.append([float(px_map), float(py_map)])
            
            angle += scan.angle_increment
        
        return {
            "points": points,
            "timestamp": self._last_scan_time,
            "frame_id": "map"
        }
    
    def _goal_pose_callback(self, msg) -> None:
        """Process navigation goal updates from /goal_pose topic."""
        x = msg.pose.position.x
        y = msg.pose.position.y
        
        # Calculate yaw from quaternion
        q = msg.pose.orientation
        yaw = math.atan2(2.0*(q.w*q.z + q.x*q.y), 1.0 - 2.0*(q.y*q.y + q.z*q.z))
        
        self._navigation_goal = {"x": x, "y": y, "yaw": yaw}
    
    def get_navigation_goal(self) -> Optional[Dict[str, float]]:
        """Return current navigation goal (x, y, yaw) in map frame."""
        return self._navigation_goal
    
    def get_sensor_status(self) -> Dict[str, Any]:
        """Get sensor health status for monitoring."""
        import time
        current_time = time.time()
        
        # Check if sensors are active (received data in last 2 seconds)
        pointcloud_active = (current_time - self._last_pointcloud_time) < 2.0 if self._last_pointcloud_time > 0 else False
        map_active = (current_time - self._last_map_time) < 5.0 if self._last_map_time > 0 else False
        
        return {
            "livox_active": pointcloud_active,
            "livox_count": self._pointcloud_count,
            "livox_last_time": self._last_pointcloud_time,
            "map_active": map_active,
            "map_last_time": self._last_map_time,
            "map_size": {
                "width": self._latest_map.info.width if self._latest_map else 0,
                "height": self._latest_map.info.height if self._latest_map else 0,
            } if self._latest_map else None
        }
    
    def get_matching_clouds(self) -> Dict[str, Any]:
        """Get matching process visualization: source & target clouds + correspondences."""
        result = {
            "teaser_source": None,
            "teaser_target": None,
            "teaser_correspondences": None,
            "rough_source": None,
            "rough_target": None,
            "refine_source": None,
            "refine_target": None
        }
        
        try:
            import sensor_msgs_py.point_cloud2 as pc2
            
            # Helper function to convert PointCloud2 to list of points
            def convert_cloud(cloud_msg):
                if not cloud_msg:
                    return None
                points = []
                for point in pc2.read_points(cloud_msg, field_names=("x", "y", "z"), skip_nans=True):
                    points.append({"x": float(point[0]), "y": float(point[1]), "z": float(point[2])})
                return {
                    "points": points,
                    "count": len(points),
                    "frame_id": cloud_msg.header.frame_id
                }
            
            # Convert TEASER++ source & target
            result["teaser_source"] = convert_cloud(self._teaser_source_cloud)
            result["teaser_target"] = convert_cloud(self._teaser_target_cloud)
            
            # Convert TEASER++ correspondences
            if self._teaser_correspondences:
                correspondences = []
                for marker in self._teaser_correspondences.markers:
                    if len(marker.points) >= 2:
                        correspondences.append({
                            "src": {
                                "x": float(marker.points[0].x),
                                "y": float(marker.points[0].y),
                                "z": float(marker.points[0].z)
                            },
                            "tgt": {
                                "x": float(marker.points[1].x),
                                "y": float(marker.points[1].y),
                                "z": float(marker.points[1].z)
                            },
                            "color": {
                                "r": marker.color.r,
                                "g": marker.color.g,
                                "b": marker.color.b,
                                "a": marker.color.a
                            }
                        })
                result["teaser_correspondences"] = correspondences
            
            # Convert rough source & target
            result["rough_source"] = convert_cloud(self._rough_source_cloud)
            result["rough_target"] = convert_cloud(self._rough_target_cloud)
            
            # Convert refine source & target
            result["refine_source"] = convert_cloud(self._refine_source_cloud)
            result["refine_target"] = convert_cloud(self._refine_target_cloud)
                
        except Exception as e:
            print(f"Error processing point clouds: {e}")
        
        return result

    def publish_goal(self, goal_json: Dict[str, Any]) -> None:
        """Publish navigation goal with complete message fields."""
        msg = PoseStamped()
        msg.header.frame_id = goal_json.get("frame_id", "map")
        msg.header.stamp = self._adapter.node.get_clock().now().to_msg()
        
        # Set position (all three coordinates)
        msg.pose.position.x = float(goal_json["goal_x"])
        msg.pose.position.y = float(goal_json["goal_y"])
        msg.pose.position.z = 0.0

        # Convert yaw to quaternion (complete quaternion)
        yaw = float(goal_json.get("goal_theta", 0.0))
        msg.pose.orientation.x = 0.0
        msg.pose.orientation.y = 0.0
        msg.pose.orientation.z = math.sin(yaw / 2.0)
        msg.pose.orientation.w = math.cos(yaw / 2.0)

        self._goal_pub.publish(msg)
        
        # Log for debugging
        print(f"Published navigation goal: x={msg.pose.position.x:.2f}, y={msg.pose.position.y:.2f}, yaw={yaw:.2f} rad")

        # Store and publish goal_id
        if "goal_id" in goal_json:
            self._current_goal_id = int(goal_json["goal_id"])
            self.current_task_id = self._current_goal_id  # Sync for ros_web_server compatibility
            goal_id_msg = String(data=str(goal_json["goal_id"]))
            self._goal_id_pub.publish(goal_id_msg)
            print(f"Published goal_id: {self._current_goal_id}")

    def publish_cmd_vel(self, nav_json: Dict[str, Any]) -> None:
        msg = Twist()
        msg.linear.x = float(nav_json.get("vel_x", 0.0))
        msg.linear.y = float(nav_json.get("vel_y", 0.0))
        msg.angular.z = float(nav_json.get("vel_theta", 0.0))
        self._cmd_vel_pub.publish(msg)

    def publish_pause(self) -> None:
        self._pause_pub.publish(String(data="pause"))

    def publish_resume(self) -> None:
        self._resume_pub.publish(String(data="resume"))
    
    def publish_initial_pose(self, x: float, y: float, z: float, yaw: float, roll: float = 0.0, pitch: float = 0.0) -> None:
        """Publish initial pose for GICP localizer.
        
        Args:
            x, y, z: Position in map frame
            yaw: Yaw angle in radians (rotation around Z axis)
            roll: Roll angle in radians (rotation around X axis), default 0.0
            pitch: Pitch angle in radians (rotation around Y axis), default 0.0
        """
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = "map"
        msg.header.stamp = self._adapter.node.get_clock().now().to_msg()
        
        # Set position
        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y
        msg.pose.pose.position.z = z
        
        # Convert Euler angles (roll, pitch, yaw) to quaternion
        # Using standard ROS convention: roll (X), pitch (Y), yaw (Z)
        # Formula: q = q_roll * q_pitch * q_yaw
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)
        
        msg.pose.pose.orientation.w = cr * cp * cy + sr * sp * sy
        msg.pose.pose.orientation.x = sr * cp * cy - cr * sp * sy
        msg.pose.pose.orientation.y = cr * sp * cy + sr * cp * sy
        msg.pose.pose.orientation.z = cr * cp * sy - sr * sp * cy
        
        # Set covariance (diagonal matrix with position and orientation uncertainty)
        msg.pose.covariance = [
            0.25, 0.0, 0.0, 0.0, 0.0, 0.0,  # x variance
            0.0, 0.25, 0.0, 0.0, 0.0, 0.0,  # y variance
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,   # z variance (not used in 2D)
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,   # roll variance (not used in 2D)
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,   # pitch variance (not used in 2D)
            0.0, 0.0, 0.0, 0.0, 0.0, 0.06853891945200942  # yaw variance
        ]
        
        self._initialpose_pub.publish(msg)
        print(f"Published initial pose: x={x:.2f}, y={y:.2f}, z={z:.2f}, yaw={yaw:.2f} rad")
    
    def publish_nav_goal(self, x: float, y: float, yaw: float) -> None:
        """Publish navigation goal to /goal_pose topic."""
        msg = PoseStamped()
        msg.header.frame_id = "map"
        msg.header.stamp = self._adapter.node.get_clock().now().to_msg()
        
        # Set position
        msg.pose.position.x = x
        msg.pose.position.y = y
        msg.pose.position.z = 0.0
        
        # Convert yaw to quaternion
        qz = math.sin(yaw / 2.0)
        qw = math.cos(yaw / 2.0)
        msg.pose.orientation.x = 0.0
        msg.pose.orientation.y = 0.0
        msg.pose.orientation.z = qz
        msg.pose.orientation.w = qw
        
        self._goal_pub.publish(msg)
        print(f"Published navigation goal: x={x:.2f}, y={y:.2f}, yaw={yaw:.2f} rad")
    
    def subscribe_to_dlio(self) -> None:
        """
        Subscribe to DLIO-related topics after DLIO is confirmed to be ready.
        This method should only be called once, after DLIO health check passes.
        """
        if self._dlio_subscribed:
            print("⚠️ DLIO topics already subscribed, skipping")
            return
        
        print("🔗 Subscribing to DLIO topics...")
        
        # Subscribe to DLIO odom (mapping mode)
        # Use RELIABLE QoS to match DLIO's publisher
        if Odometry is not None:
            from rclpy.qos import QoSProfile, ReliabilityPolicy
            dlio_odom_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
            self._adapter.create_subscription(Odometry, "/dlio/odom_node/odom", self._robot_odom_callback, dlio_odom_qos)
            self._subscriptions["/dlio/odom_node/odom"] = self._robot_odom_callback
            print("✅ Subscribed to /dlio/odom_node/odom with RELIABLE QoS")
        
        self._dlio_subscribed = True
        print("✅ DLIO topic subscriptions complete")

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------
    @staticmethod
    def _occupancy_grid_to_json(msg: OccupancyGrid) -> Dict[str, Any]:
        info = msg.info
        width = int(info.width)
        height = int(info.height)
        
        # ROS2 OccupancyGrid is bottom-to-top (row 0 at bottom)
        # But frontend Canvas expects top-to-bottom (row 0 at top)
        # So we flip the data here to match Canvas/PGM format
        data = list(msg.data)
        flipped_data = []
        for y in range(height - 1, -1, -1):  # Iterate from bottom to top
            row_start = y * width
            row_end = row_start + width
            flipped_data.extend(data[row_start:row_end])
        
        payload = {
            "width": width,
            "height": height,
            "resolution": float(info.resolution),
            "origin_x": float(info.origin.position.x),
            "origin_y": float(info.origin.position.y),
            "data": base64.b64encode(bytes([(cell + 256) % 256 for cell in flipped_data])).decode("utf-8"),
        }
        return payload


__all__ = ["ExtendedRosBridge", "RosAdapter"]


