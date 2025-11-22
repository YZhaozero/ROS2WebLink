#!/usr/bin/env python3
"""
Test script to verify DLIO health checker functionality.

This script can be run to test the DLIO health monitoring without starting the full web server.
"""

import time
import rclpy
from rclpy.node import Node
from web_server.dlio_health_checker import DLIOHealthChecker


def main():
    print("=" * 60)
    print("DLIO Health Checker Test")
    print("=" * 60)
    
    # Initialize ROS2
    rclpy.init()
    
    # Create a test node
    test_node = Node("dlio_health_test_node")
    
    # Create health checker
    health_checker = DLIOHealthChecker(test_node)
    
    # Register callback
    def on_ready():
        print("✅ CALLBACK TRIGGERED: DLIO is ready!")
    
    health_checker.register_ready_callback(on_ready)
    
    # Start monitoring
    health_checker.start_monitoring(check_interval=2.0)
    
    print("\n⏳ Monitoring DLIO status...")
    print("   (Press Ctrl+C to stop)")
    print("\nExpected behavior:")
    print("  - If DLIO is NOT running: Will keep checking every 2 seconds")
    print("  - If DLIO IS running: Will detect it and trigger callback")
    print()
    
    try:
        # Keep the node spinning
        while rclpy.ok():
            rclpy.spin_once(test_node, timeout_sec=0.1)
            time.sleep(0.1)
            
            # Show status updates
            if health_checker.is_ready():
                print("✅ Status: DLIO is ready!")
                break
    
    except KeyboardInterrupt:
        print("\n\n⚠️ Test interrupted by user")
    
    finally:
        # Cleanup
        health_checker.stop_monitoring()
        test_node.destroy_node()
        rclpy.shutdown()
        print("\n✅ Test completed")


if __name__ == "__main__":
    main()




