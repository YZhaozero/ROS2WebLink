"""DLIO health checker - monitors DLIO startup status before subscribing."""

import time
import threading
from typing import Callable, Optional
import logging

logger = logging.getLogger(__name__)


class DLIOHealthChecker:
    """
    Monitors DLIO node health and readiness.
    
    Checks:
    1. DLIO odom topic exists and publishes data
    2. Livox point cloud topic exists and publishes data
    3. DLIO nodes are running
    """
    
    def __init__(self, ros_node):
        """
        Args:
            ros_node: rclpy Node instance for topic/node discovery
        """
        self.node = ros_node
        self._is_ready = False
        self._monitor_thread: Optional[threading.Thread] = None
        self._stop_monitor = False
        self._ready_callbacks = []
        
        # Monitoring state
        self._last_odom_time = 0.0
        self._last_pointcloud_time = 0.0
        self._odom_count = 0
        self._pointcloud_count = 0
        
        # Temporary subscriptions for health check
        self._temp_subs = []
    
    def is_ready(self) -> bool:
        """Check if DLIO is ready."""
        return self._is_ready
    
    def register_ready_callback(self, callback: Callable[[], None]) -> None:
        """Register a callback to be called when DLIO becomes ready."""
        self._ready_callbacks.append(callback)
    
    def start_monitoring(self, check_interval: float = 2.0) -> None:
        """
        Start monitoring DLIO health in background thread.
        
        Args:
            check_interval: Seconds between health checks
        """
        if self._monitor_thread and self._monitor_thread.is_alive():
            logger.warning("DLIO monitor already running")
            return
        
        self._stop_monitor = False
        self._monitor_thread = threading.Thread(
            target=self._monitor_loop,
            args=(check_interval,),
            daemon=True
        )
        self._monitor_thread.start()
        logger.info("🔍 DLIO health monitor started")
    
    def stop_monitoring(self) -> None:
        """Stop the monitoring thread."""
        self._stop_monitor = True
        if self._monitor_thread:
            self._monitor_thread.join(timeout=5.0)
        
        # Clean up temporary subscriptions
        for sub in self._temp_subs:
            try:
                self.node.destroy_subscription(sub)
            except Exception as e:
                logger.warning(f"Failed to destroy temp subscription: {e}")
        self._temp_subs.clear()
    
    def _monitor_loop(self, check_interval: float) -> None:
        """Background monitoring loop."""
        logger.info("⏳ Waiting for DLIO to be ready...")
        
        while not self._stop_monitor:
            try:
                ready = self._check_health()
                
                if ready and not self._is_ready:
                    self._is_ready = True
                    logger.info("✅ DLIO is ready! Notifying subscribers...")
                    
                    # Notify all registered callbacks
                    for callback in self._ready_callbacks:
                        try:
                            callback()
                        except Exception as e:
                            logger.error(f"Error in ready callback: {e}")
                    
                    # Stop monitoring once ready
                    break
                
                elif not ready:
                    logger.debug("⏳ DLIO not ready yet, continuing to monitor...")
                
            except Exception as e:
                logger.error(f"Error in DLIO health check: {e}")
            
            time.sleep(check_interval)
        
        logger.info("DLIO health monitor stopped")
    
    def _check_health(self) -> bool:
        """
        Check if DLIO is healthy and ready.
        
        Returns:
            True if DLIO is ready, False otherwise
        """
        # Step 1: Check if critical topics exist
        topics = self.node.get_topic_names_and_types()
        topic_dict = {name: types for name, types in topics}
        
        required_topics = [
            "/dlio/odom_node/odom",
            "/livox/lidar/pointcloud",
        ]
        
        missing_topics = []
        for topic in required_topics:
            if topic not in topic_dict:
                missing_topics.append(topic)
        
        if missing_topics:
            logger.debug(f"Missing topics: {missing_topics}")
            return False
        
        # Step 2: Check if topics are actively publishing
        # We need to verify data is flowing
        current_time = time.time()
        
        # Create temporary subscriptions if not already done
        if not self._temp_subs:
            self._setup_temp_subscriptions()
            # Give subscriptions time to receive first messages
            logger.debug("Created temp subscriptions, waiting for data...")
            return False
        
        # Check if we received recent data (within last 5 seconds)
        # Use 5 seconds to be more tolerant of timing issues
        odom_active = (current_time - self._last_odom_time) < 5.0 if self._last_odom_time > 0 else False
        pointcloud_active = (current_time - self._last_pointcloud_time) < 5.0 if self._last_pointcloud_time > 0 else False
        
        if not odom_active or not pointcloud_active:
            logger.info(
                f"Topics not publishing: odom_active={odom_active} "
                f"(odom_count={self._odom_count}, last_time={self._last_odom_time:.2f}, current={current_time:.2f}), "
                f"pointcloud_active={pointcloud_active} "
                f"(pc_count={self._pointcloud_count}, last_time={self._last_pointcloud_time:.2f})"
            )
            return False
        
        # Step 3: Check if DLIO nodes are running
        nodes = self.node.get_node_names()
        # Note: get_node_names() returns names WITHOUT leading slash
        required_nodes = [
            "dlio_odom_node",
            "dlio_map_node",
        ]
        
        missing_nodes = []
        for node_name in required_nodes:
            if node_name not in nodes:
                missing_nodes.append(node_name)
        
        if missing_nodes:
            logger.info(f"❌ Missing nodes: {missing_nodes}, available nodes: {nodes}")
            return False
        
        logger.info(
            f"🎉 ✅ DLIO health check PASSED! "
            f"odom_count={self._odom_count}, "
            f"pointcloud_count={self._pointcloud_count}, "
            f"nodes_ok={len(nodes)}"
        )
        return True
    
    def _setup_temp_subscriptions(self) -> None:
        """Set up temporary subscriptions for health monitoring."""
        try:
            from nav_msgs.msg import Odometry
            from sensor_msgs.msg import PointCloud2
            from rclpy.qos import QoSProfile, ReliabilityPolicy
            
            # Use RELIABLE QoS to match DLIO's publisher
            reliable_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
            
            # Subscribe to DLIO odom
            odom_sub = self.node.create_subscription(
                Odometry,
                "/dlio/odom_node/odom",
                self._temp_odom_callback,
                reliable_qos
            )
            self._temp_subs.append(odom_sub)
            
            # Subscribe to Livox point cloud with BEST_EFFORT QoS (Livox uses BEST_EFFORT)
            best_effort_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
            pc_sub = self.node.create_subscription(
                PointCloud2,
                "/livox/lidar/pointcloud",
                self._temp_pointcloud_callback,
                best_effort_qos
            )
            self._temp_subs.append(pc_sub)
            
            logger.info("✅ Temporary health check subscriptions created for /dlio/odom_node/odom and /livox/lidar/pointcloud")
            
        except Exception as e:
            logger.error(f"Failed to create temp subscriptions: {e}")
    
    def _temp_odom_callback(self, msg) -> None:
        """Temporary callback for DLIO odom topic."""
        self._last_odom_time = time.time()
        self._odom_count += 1
        if self._odom_count == 1 or self._odom_count % 50 == 0:
            logger.info(f"📊 Received DLIO odom #{self._odom_count}")
    
    def _temp_pointcloud_callback(self, msg) -> None:
        """Temporary callback for Livox point cloud topic."""
        self._last_pointcloud_time = time.time()
        self._pointcloud_count += 1
        if self._pointcloud_count == 1 or self._pointcloud_count % 10 == 0:
            logger.info(f"📊 Received Livox pointcloud #{self._pointcloud_count}")


__all__ = ["DLIOHealthChecker"]




