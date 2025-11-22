import sys
import os
import shutil
from pathlib import Path

# Add project root to python path
sys.path.append("/home/guest/ROS2WebLink")

from web_server.mapping_controller import MappingController

def main():
    # Set environment
    os.environ["ROS_DOMAIN_ID"] = "0"
    
    workdir = Path("/home/guest/tron_ros2")
    map_name = "map_1121_test_001"
    removert_map_name = f"{map_name}_removert"
    
    print(f"Initializing controller...")
    controller = MappingController(workdir=workdir)
    # Set _map_name manually for description in registry
    controller._map_name = map_name
    
    kitti_data_dir = workdir / "removert_data" / map_name
    removert_result_dir = kitti_data_dir / "results"
    
    print(f"Checking results in {removert_result_dir}")
    
    # Step 4: Find and Copy Cleaned PCD (Logic from updated MappingController)
    cleaned_pcd = removert_result_dir / "map_static" / "StaticMapScansideMapGlobal.pcd"
    
    if not cleaned_pcd.exists():
        # Try map-side removal result (intermediate result)
        candidates = list(removert_result_dir.rglob("StaticMapMapsideGlobal*.pcd"))
        if candidates:
            cleaned_pcd = sorted(candidates, key=lambda x: x.stat().st_mtime)[-1]
            print(f"⚠️ Using intermediate map-side result: {cleaned_pcd.name}")

    if not cleaned_pcd.exists():
        print(f"❌ Could not find cleaned PCD map")
        return

    final_pcd_dir = workdir / "src/tron_slam/localizer/PCD"
    final_pcd_dir.mkdir(parents=True, exist_ok=True)
    final_pcd = final_pcd_dir / f"{removert_map_name}.pcd"
    
    print(f"Copying {cleaned_pcd} -> {final_pcd}")
    shutil.copy2(cleaned_pcd, final_pcd)
    print(f"✅ Copied cleaned PCD: {final_pcd}")
    
    # Step 5: Generate 2D Map
    print(f"Step 5: Generating 2D map from cleaned PCD...")
    nav_maps_dir = workdir / "src/tron_nav/tron_navigation/maps"
    nav_maps_dir.mkdir(parents=True, exist_ok=True)
    
    if controller._generate_2d_map_from_pcd(final_pcd, nav_maps_dir / removert_map_name):
        print(f"✅ Generated 2D map: {removert_map_name}")
    else:
        print(f"❌ Failed to generate 2D map")
        
    # Step 6: Update registry
    print(f"Step 6: Updating registry...")
    removert_pgm_file = nav_maps_dir / f"{removert_map_name}.pgm"
    removert_yaml_file = nav_maps_dir / f"{removert_map_name}.yaml"
    
    try:
        controller._update_map_registry(removert_map_name, final_pcd, removert_pgm_file, removert_yaml_file)
        print(f"✅ Registry updated for {removert_map_name}")
    except Exception as e:
        print(f"❌ Failed to update registry: {e}")
        
    print("Done!")

if __name__ == "__main__":
    main()
