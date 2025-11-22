import sys
import os
from pathlib import Path

# Add project root to python path
sys.path.append("/home/guest/ROS2WebLink")

from web_server.mapping_controller import MappingController

def main():
    # Set environment
    os.environ["ROS_DOMAIN_ID"] = "0"
    
    workdir = Path("/home/guest/tron_ros2")
    rosbag_path = workdir / "rosbags/mapping_20251121_115750"
    map_name = "map_1121_test_001"
    
    print(f"Initializing controller...")
    controller = MappingController(workdir=workdir)
    controller._rosbag_path = rosbag_path
    
    print(f"Starting manual processing...")
    print(f"Rosbag: {rosbag_path}")
    print(f"Map Name: {map_name}")
    
    controller._process_rosbag_with_removert(map_name)
    print("Done!")

if __name__ == "__main__":
    main()

