"""Mapping controller to orchestrate ROS mapping launch pipelines."""

from __future__ import annotations

import subprocess
import threading
import time
import os
import shutil
from pathlib import Path
from typing import Iterable, List, Optional


class MappingController:
    def __init__(
        self,
        launch_sequence: Optional[Iterable[Iterable[str]]] = None,
        map_save_script: Optional[List[str]] = None,
        workdir: Optional[Path] = None,
        rosbag_cleanup_policy: str = "delete",  # delete, archive, keep
    ) -> None:
        self.launch_sequence = [list(cmd) for cmd in (launch_sequence or self._default_launch())]
        self.map_save_script = map_save_script or [
            "ros2", "run", "nav2_map_server", "map_saver_cli", "-f"
        ]
        self.workdir = Path(workdir or ".").resolve()
        self.rosbag_cleanup_policy = rosbag_cleanup_policy

        self._processes: List[subprocess.Popen] = []
        self._rosbag_proc: Optional[subprocess.Popen] = None
        self._rosbag_path: Optional[Path] = None
        
        self._state = "IDLE"
        self._start_time: Optional[float] = None
        self._map_name: Optional[str] = None
        self._save_thread: Optional[threading.Thread] = None
        self._save_status: str = "idle"  # idle, saving, success, failed
        self._save_stage: str = ""       # description of current save stage
        self._save_progress: int = 0     # 0-100 progress percentage

    # ------------------------------------------------------------------
    def start(self, map_name: Optional[str] = None) -> dict:
        if self._state not in {"IDLE", "ERROR"}:
            raise RuntimeError("mapping already running")

        self._state = "STARTING"
        self._map_name = map_name or time.strftime("map_%Y%m%d_%H%M%S")
        self._processes.clear()
        self._rosbag_proc = None
        self._rosbag_path = None

        # Start rosbag recording first
        self._start_rosbag_recording(self._map_name)

        # Start processes in background thread to avoid blocking
        def start_processes():
            for idx, command in enumerate(self.launch_sequence):
                # Wrap command in bash to source ROS2 environment
                # Use ROS_DOMAIN_ID from environment or default to 0
                ros_domain_id = os.getenv("ROS_DOMAIN_ID", "0")
                bash_cmd = [
                    "bash", "-c",
                    f"export ROS_DOMAIN_ID={ros_domain_id} && "
                    f"export FASTRTPS_DEFAULT_PROFILES_FILE=/home/guest/.config/fastdds/fastdds.xml && "
                    f"source /opt/ros/humble/setup.bash && "
                    f"source {self.workdir}/install/setup.bash && "
                    f"{' '.join(command)}"
                ]
                # Log output to files for debugging
                log_file = f"/tmp/mapping_{idx}_{command[2] if len(command) > 2 else 'unknown'}.log"
                with open(log_file, "w") as log:
                    proc = subprocess.Popen(
                        bash_cmd,
                        cwd=self.workdir,
                        stdout=log,
                        stderr=subprocess.STDOUT
                    )
                self._processes.append(proc)
                print(f"Started process {idx}: {' '.join(command)} (PID: {proc.pid}, log: {log_file})")
                time.sleep(3)  # Give each process time to start
            
            self._state = "RUNNING"
            self._start_time = time.time()
            print("✅ All mapping processes started")
        
        # Start processes in background thread
        start_thread = threading.Thread(target=start_processes, daemon=True)
        start_thread.start()
        
        # Return immediately with STARTING status
        return {"status": "STARTING", "message": "Mapping processes are starting in background"}

    def _start_rosbag_recording(self, map_name: str) -> None:
        """Start recording relevant topics to a rosbag."""
        rosbag_root = self.workdir / "rosbags"
        rosbag_root.mkdir(parents=True, exist_ok=True)
        rosbag_dir = rosbag_root / f"mapping_{time.strftime('%Y%m%d_%H%M%S')}"
        self._rosbag_path = rosbag_dir
        
        # Topics required for removert (Livox points) and DLIO odometry
        topics = [
            "/livox/lidar",
            "/livox/points", 
            "/dlio/odom_node/odom", 
            "/dlio/odom_node/pointcloud/deskewed",
            "/tf_static" # Record static TF for extrinsic calibration
        ]
        
        cmd = ["ros2", "bag", "record", "-o", str(rosbag_dir)] + topics
        
        # Start recording
        print(f"Starting rosbag recording to {rosbag_dir}")
        ros_domain_id = os.getenv("ROS_DOMAIN_ID", "0")
        bash_cmd = [
            "bash", "-c",
            f"export ROS_DOMAIN_ID={ros_domain_id} && "
            f"source /opt/ros/humble/setup.bash && "
            f"{' '.join(cmd)}"
        ]
        
        try:
            self._rosbag_proc = subprocess.Popen(
                bash_cmd,
                cwd=self.workdir,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL
            )
            print(f"Rosbag recording started (PID: {self._rosbag_proc.pid})")
        except Exception as e:
            print(f"Failed to start rosbag recording: {e}")
            self._rosbag_proc = None

    def _stop_rosbag_recording(self) -> None:
        """Stop the rosbag recording process."""
        if self._rosbag_proc:
            print("Stopping rosbag recording...")
            try:
                self._rosbag_proc.terminate()
                try:
                    self._rosbag_proc.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    self._rosbag_proc.kill()
                print("Rosbag recording stopped")
            except Exception as e:
                print(f"Error stopping rosbag: {e}")
            finally:
                self._rosbag_proc = None
                
            # Also force kill ros2 bag record to be sure
            subprocess.run(["pkill", "-f", "ros2 bag record"], capture_output=True)

    def stop(self, *, save: bool = True, map_name: Optional[str] = None) -> dict:
        # Check if there are actual mapping processes running (even if _processes list is empty)
        # This handles the case where processes were started but _processes list was lost
        actual_processes_running = self._check_mapping_processes_running()
        
        if self._state not in {"RUNNING", "ERROR"} and not actual_processes_running:
            return {"status": self._state, "save_status": self._save_status}

        self._state = "STOPPING"
        if map_name:
            self._map_name = map_name
        
        # Recover rosbag path if lost
        if not self._rosbag_path:
            self._recover_rosbag_path()
        
        # Set save status BEFORE starting thread to ensure it's visible immediately
        if save and self.map_save_script:
            self._save_status = "saving"
            self._save_stage = "Stopping processes..."
            self._save_progress = 5
        
        # If not saving, stop rosbag immediately and terminate all processes
        if not save:
            self._stop_rosbag_recording()
            self._terminate_all_mapping_processes()
            self._processes.clear()
            # Cleanup unused rosbag
            self._cleanup_rosbag()
        
        # Start map saving in background thread
        if save and self.map_save_script:
            final_map_name = self._map_name or map_name or "map"
            self._save_thread = threading.Thread(
                target=self._save_map_async,
                args=(final_map_name,),
                daemon=True
            )
            self._save_thread.start()
        else:
            # If not saving, set state to IDLE immediately
            self._state = "IDLE"
        
        return {
            "status": self._state,
            "save_status": self._save_status,
            "message": "Map saving in background" if save else "Stopped without saving"
        }

    def _check_mapping_processes_running(self) -> bool:
        """Check if mapping processes are actually running."""
        import subprocess
        try:
            result = subprocess.run(
                ["pgrep", "-f", "dlio|octomap|livox_ros_driver2|pointcloud2"],
                capture_output=True,
                text=True,
                timeout=2
            )
            return result.returncode == 0 and len(result.stdout.strip()) > 0
        except Exception:
            return False
    
    def _recover_rosbag_path(self) -> None:
        """Recover rosbag path by finding the most recent rosbag directory."""
        rosbag_dir = self.workdir / "rosbags"
        if rosbag_dir.exists():
            # Find most recent rosbag directory
            rosbag_dirs = sorted(
                [d for d in rosbag_dir.iterdir() if d.is_dir() and d.name.startswith("mapping_")],
                key=lambda x: x.stat().st_mtime,
                reverse=True
            )
            if rosbag_dirs:
                self._rosbag_path = rosbag_dirs[0]
                print(f"🔍 Recovered rosbag path: {self._rosbag_path}")

    def _terminate_all_mapping_processes(self) -> None:
        """Terminate all mapping processes, even if _processes list is empty."""
        import subprocess
        # First try to terminate processes in _processes list
        for proc in self._processes:
            try:
                proc.terminate()
                proc.wait(timeout=5)
            except subprocess.TimeoutExpired:
                proc.kill()
            except Exception:
                pass
        
        # Also kill processes by name (in case _processes list is empty)
        # Specifically kill mapping nodes but NOT the save script processes if they are running
        try:
            subprocess.run(
                ["pkill", "-TERM", "-f", "dlio|octomap|livox_ros_driver2|pointcloud2"],
                capture_output=True,
                timeout=5
            )
            time.sleep(2)
            # Force kill if still running
            subprocess.run(
                ["pkill", "-9", "-f", "dlio|octomap|livox_ros_driver2|pointcloud2"],
                capture_output=True,
                timeout=5
            )
        except Exception as e:
            print(f"⚠️ Error terminating processes: {e}")

    def _save_map_async(self, map_name: str):
        """Save map in background thread to avoid blocking."""
        # Ensure map_name is absolute path
        map_file = self.workdir / map_name
        cmd = self.map_save_script + [str(map_file)]
        # Wrap in bash to source ROS2 environment
        ros_domain_id = os.getenv("ROS_DOMAIN_ID", "0")
        bash_cmd = [
            "bash", "-c",
            f"export ROS_DOMAIN_ID={ros_domain_id} && "
            f"source /opt/ros/humble/setup.bash && "
            f"source {self.workdir}/install/setup.bash && "
            f"{' '.join(cmd)}"
        ]
        
        self._save_stage = "Saving 3D PCD Map..."
        self._save_progress = 10
        
        try:
            # Save 3D PCD map FIRST (while DLIO is still running and publishing /dlio_points)
            print("Saving 3D PCD map first (while DLIO is running)...")
            saved_pcd_file = self._save_pcd_map(map_name)
            self._save_progress = 30
            
            # Stop rosbag before saving 2D map? No, continue recording until end
            
            self._save_stage = "Saving 2D Map..."
            print(f"Executing 2D map save command: {' '.join(cmd)}")
            result = subprocess.run(
                bash_cmd, 
                check=False, 
                capture_output=True, 
                text=True,
                timeout=180,  # 3 minutes timeout for map saving
                cwd=self.workdir
            )
            self._save_progress = 50
            
            # Now stop rosbag and other processes
            self._stop_rosbag_recording()
            self._terminate_all_mapping_processes()
            self._processes.clear()
            
            if result.returncode == 0:
                print(f"2D Map saved successfully: {map_name}")
                
                # Auto-copy map to navigation directory
                try:
                    import shutil
                    nav_maps_dir = self.workdir / "src/tron_nav/tron_navigation/maps"
                    if nav_maps_dir.exists():
                        for ext in ['.pgm', '.yaml']:
                            src = self.workdir / f"{map_name}{ext}"
                            dst = nav_maps_dir / f"{map_name}{ext}"
                            if src.exists():
                                shutil.copy2(src, dst)
                                print(f"Copied {src} to {dst}")
                except Exception as e:
                    print(f"Failed to copy map to navigation directory: {e}")
                
                # Process with remover
                base_pcd_dir = self.workdir / "src/tron_slam/localizer/PCD"
                base_pcd_file = saved_pcd_file if saved_pcd_file and saved_pcd_file.exists() else base_pcd_dir / f"{map_name}.pcd"
                if base_pcd_file.exists():
                    try:
                        self._save_stage = "Updating map registry..."
                        self._update_map_registry(
                            map_name,
                            base_pcd_file,
                            nav_maps_dir / f"{map_name}.pgm",
                            nav_maps_dir / f"{map_name}.yaml"
                        )
                        print(f"✅ Registered base map: {map_name}")
                    except Exception as e:
                        print(f"⚠️ Failed to update registry for {map_name}: {e}")
                else:
                    print(f"⚠️ Base PCD map missing, skipping registry update for {map_name}")

                self._save_stage = "Processing with remover..."
                self._save_progress = 60
                removert_success = self._process_rosbag_with_removert(map_name)
                if removert_success:
                    self._save_progress = 100
                    self._save_stage = "Completed"
                else:
                    print("⚠️ Removert processing failed; base map saved without cleaned version.")
                    self._save_progress = 90
                    self._save_stage = "Completed (Removert failed)"
                
                self._save_status = "success"
            else:
                self._save_status = "failed"
                self._save_stage = "Map Save Failed"
                print(f"Map save failed: {result.stderr}")
                
        except subprocess.TimeoutExpired:
            self._save_status = "failed"
            self._save_stage = "Timeout"
            print("Map save timed out after 3 minutes")
        except Exception as e:
            self._save_status = "failed"
            self._save_stage = "Error"
            print(f"Map save error: {e}")
            import traceback
            traceback.print_exc()
        finally:
            # Ensure everything is stopped
            self._stop_rosbag_recording()
            self._terminate_all_mapping_processes()
            self._processes.clear()
            self._state = "IDLE"
            print("All mapping processes terminated.")
            
            # Handle rosbag cleanup based on final status
            if self._save_status == "success":
                # If success, apply normal cleanup policy (delete/archive/keep)
                self._cleanup_rosbag()
            elif self._save_status == "failed":
                # If failed, FORCE KEEP rosbag for debugging, unless policy is archive
                if self.rosbag_cleanup_policy == "archive":
                    print("⚠️ Map save failed, but archiving rosbag as configured.")
                    self._cleanup_rosbag()
                else:
                    print(f"⚠️ Map save failed. RETAINING rosbag for debugging: {self._rosbag_path}")
                    # Do NOT call _cleanup_rosbag() here to prevent deletion
    
    def _save_pcd_map(self, map_name: str) -> Optional[Path]:
        """Save 3D point cloud map for GICP localizer using DLIO service."""
        try:
            pcd_dir = self.workdir / "src/tron_slam/localizer/PCD"
            pcd_dir.mkdir(parents=True, exist_ok=True)
            pcd_file = pcd_dir / f"{map_name}.pcd"
            
            print(f"Saving 3D PCD map to {pcd_file} via DLIO service...")
            
            # Call DLIO save_pcd service
            # The service saves 'dlio_map.pcd' in the specified directory
            ros_domain_id = os.getenv("ROS_DOMAIN_ID", "0")
            
            # Using leaf_size 0.05 for high quality map
            service_args = f"{{leaf_size: 0.05, save_path: '{str(pcd_dir)}'}}"
            
            bash_cmd = [
                "bash", "-c",
                f"export ROS_DOMAIN_ID={ros_domain_id} && "
                f"source /opt/ros/humble/setup.bash && "
                f"source {self.workdir}/install/setup.bash && "
                f"ros2 service call /save_pcd direct_lidar_inertial_odometry/srv/SavePCD \"{service_args}\""
            ]
            
            # Run with 30 second timeout
            result = subprocess.run(
                bash_cmd,
                capture_output=True,
                text=True,
                timeout=30,
                cwd=self.workdir
            )
            
            print(f"Service call output: {result.stdout}")
            
            # Check for the generated file (default name is dlio_map.pcd)
            generated_file = pcd_dir / "dlio_map.pcd"
            
            if result.returncode == 0 and generated_file.exists():
                # Rename to requested map name
                if generated_file != pcd_file:
                    if pcd_file.exists():
                        pcd_file.unlink()
                    generated_file.rename(pcd_file)
                
                file_size = pcd_file.stat().st_size
                print(f"✅ 3D PCD map saved: {pcd_file} ({file_size / 1024:.1f} KB)")
                self._update_localizer_config(pcd_file)
                return pcd_file
            else:
                print(f"⚠️ PCD save failed. Service output: {result.stdout}")
                print(f"Command error: {result.stderr}")
                
                # Fallback: try using topic subscription if service fails
                print("🔄 Trying fallback: saving from topic dlio/map...")
                save_script = self.workdir / "tools/save_pointcloud_to_pcd.py"
                bash_cmd_fallback = [
                    "bash", "-c",
                    f"export ROS_DOMAIN_ID={ros_domain_id} && "
                    f"source /opt/ros/humble/setup.bash && "
                    f"source {self.workdir}/install/setup.bash && "
                    f"python3 {save_script} dlio/map {pcd_file} 20"
                ]
                subprocess.run(bash_cmd_fallback, capture_output=True, timeout=25)
                
                if pcd_file.exists():
                    print(f"✅ Fallback saved: {pcd_file}")
                    self._update_localizer_config(pcd_file)
                    return pcd_file

        except subprocess.TimeoutExpired:
            print(f"⚠️ PCD save timeout")
        except Exception as e:
            print(f"❌ PCD save exception: {e}")
        
        return pcd_file if pcd_file.exists() else None
    
    def _update_localizer_config(self, pcd_path: Path) -> None:
        """Update localizer config with new map path."""
        import re
        config_files = [
            self.workdir / "src/tron_slam/localizer/config/localizer.yaml",
            self.workdir / "install/localizer/share/localizer/config/localizer.yaml"
        ]
        for config_file in config_files:
            try:
                if not config_file.exists():
                    continue
                with open(config_file, 'r') as f:
                    content = f.read()
                new_content = re.sub(r'default_map_path:.*', f'default_map_path: {pcd_path}', content)
                with open(config_file, 'w') as f:
                    f.write(new_content)
                print(f"✅ Updated localizer config: {config_file}")
            except Exception as e:
                print(f"⚠️ Failed to update {config_file}: {e}")
    
    def _process_rosbag_with_removert(self, map_name: str) -> bool:
        """Process recorded rosbag with removert and save cleaned map with _removert suffix.
        Returns True if the cleaned map and registry entry were produced."""
        try:
            if not self._rosbag_path or not self._rosbag_path.exists():
                print(f"⚠️ Rosbag not found, skipping removert processing")
                return False
            
            print(f"Processing rosbag with removert: {self._rosbag_path}")
            removert_map_name = f"{map_name}_removert"
            
            # Step 1: Extract rosbag to KITTI format
            kitti_data_dir = self.workdir / "removert_data" / map_name
            print(f"Step 1: Extracting rosbag to KITTI format...")
            self._save_stage = "Extracting Rosbag..."
            if not self._extract_rosbag_to_kitti(self._rosbag_path, kitti_data_dir):
                print(f"❌ Failed to extract rosbag to KITTI format")
                return False
            
            # Step 2: Query TF for Extrinsic
            # This replaces the need for removert to be a ROS node
            print(f"Step 2: Querying TF for radar extrinsic...")
            extrinsic_matrix = self._get_radar_extrinsic()
            
            # Step 3: Generate removert config
            removert_config_file = kitti_data_dir / "removert_config.yaml"
            removert_result_dir = kitti_data_dir / "results"
            print(f"Step 3: Generating removert config...")
            self._generate_removert_config(
                kitti_data_dir / "scans",
                kitti_data_dir / "poses.txt",
                removert_result_dir,
                removert_config_file,
                extrinsic_matrix
            )
            
            # Step 4: Run removert
            print(f"Step 4: Running removert...")
            self._save_stage = "Running Removert..."
            if not self._run_removert(removert_config_file):
                print(f"❌ Removert processing failed")
                return False
            
            # Step 5: Find and Copy Cleaned PCD
            cleaned_pcd = removert_result_dir / "map_static" / "StaticMapScansideMapGlobal.pcd"
            
            if not cleaned_pcd.exists():
                # Try map-side removal result (intermediate result)
                candidates = list(removert_result_dir.rglob("StaticMapMapsideGlobal*.pcd"))
                if candidates:
                    cleaned_pcd = sorted(candidates, key=lambda x: x.stat().st_mtime)[-1]
                    print(f"⚠️ Using intermediate map-side result: {cleaned_pcd.name}")
            
            if not cleaned_pcd.exists():
                print(f"❌ Could not find cleaned PCD map")
                return False

            final_pcd_dir = self.workdir / "src/tron_slam/localizer/PCD"
            final_pcd_dir.mkdir(parents=True, exist_ok=True)
            final_pcd = final_pcd_dir / f"{removert_map_name}.pcd"
            
            shutil.copy2(cleaned_pcd, final_pcd)
            print(f"✅ Copied cleaned PCD: {final_pcd}")
            
            # Step 6: Generate 2D Map
            print(f"Step 6: Generating 2D map from cleaned PCD...")
            self._save_stage = "Generating 2D Map..."
            nav_maps_dir = self.workdir / "src/tron_nav/tron_navigation/maps"
            nav_maps_dir.mkdir(parents=True, exist_ok=True)
            
            if not self._generate_2d_map_from_pcd(final_pcd, nav_maps_dir / removert_map_name):
                print(f"❌ Failed to generate 2D map from cleaned PCD")
                return False
            
            # Step 7: Update map registry
            try:
                removert_pgm_file = nav_maps_dir / f"{removert_map_name}.pgm"
                removert_yaml_file = nav_maps_dir / f"{removert_map_name}.yaml"
                self._update_map_registry(removert_map_name, final_pcd, removert_pgm_file, removert_yaml_file)
                print(f"✅ Cleaned map (removert) registered: {removert_map_name}")
            except Exception as e:
                print(f"Failed to update map registry: {e}")
                return False
            
            print(f"✅ Removert processing complete: {removert_map_name}")
            return True
        except Exception as e:
            print(f"❌ Removert processing failed: {e}")
            import traceback
            traceback.print_exc()
            return False


    def _extract_rosbag_to_kitti(self, rosbag_path: Path, output_dir: Path) -> bool:
        """Run rosbag2_to_kitti tool."""
        tool_path = self.workdir / "install/tools/lib/tools/rosbag2_to_kitti"
        if not tool_path.exists():
            # Try build directory
             tool_path = self.workdir / "build/tools/rosbag2_to_kitti"
        
        if not tool_path.exists():
            print(f"❌ rosbag2_to_kitti tool not found")
            return False
            
        output_dir.mkdir(parents=True, exist_ok=True)
        
        # Usage: rosbag2_to_kitti <bag_path> <output_dir> <pointcloud_topic> <odom_topic>
        cmd = [
            str(tool_path),
            str(rosbag_path),
            str(output_dir),
            "/dlio/odom_node/pointcloud/deskewed",
            "/dlio/odom_node/odom"
        ]
        
        print(f"Running: {' '.join(cmd)}")
        try:
            result = subprocess.run(cmd, capture_output=True, text=True, timeout=600) # 10 min timeout for large bags
            if result.returncode != 0:
                print(f"Extraction failed: {result.stderr}")
                return False
            return True
        except Exception as e:
            print(f"Extraction error: {e}")
            return False

    def _get_radar_extrinsic(self) -> List[float]:
        """Get the static transform from base_link to livox_frame.
        Returns a 16-element list representing the 4x4 matrix (row-major).
        """
        try:
            # Use python to query tf directly since we are in a ROS environment
            import rclpy
            from rclpy.node import Node
            from tf2_ros import Buffer, TransformListener
            from geometry_msgs.msg import TransformStamped
            import numpy as np
            from scipy.spatial.transform import Rotation as R
            
            # We need a temporary node to query TF
            # Note: rclpy.init() should already be called in the main process
            if not rclpy.ok():
                rclpy.init()
                
            node = rclpy.create_node('tf_query_temp')
            tf_buffer = Buffer()
            tf_listener = TransformListener(tf_buffer, node)
            
            # Wait for TF buffer to fill (we need static tf)
            # Spin the node briefly to receive TF messages
            start_time = time.time()
            while time.time() - start_time < 2.0:
                rclpy.spin_once(node, timeout_sec=0.1)
                
            # Try to lookup transform
            # Target: base_link (or base_footprint), Source: livox_frame
            # We want T_base_lidar, which transforms points from Lidar frame to Base frame
            try:
                # Try multiple common base frames
                base_frames = ['base_link', 'base_footprint', 'odom'] # odom is fallback
                trans = None
                
                for base in base_frames:
                    if tf_buffer.can_transform(base, 'livox_frame', rclpy.time.Time()):
                        trans = tf_buffer.lookup_transform(base, 'livox_frame', rclpy.time.Time())
                        print(f"Found transform: {base} -> livox_frame")
                        break
                
                if trans:
                    # Convert transform to 4x4 matrix
                    t = trans.transform.translation
                    r = trans.transform.rotation
                    
                    # Rotation
                    rot = R.from_quat([r.x, r.y, r.z, r.w])
                    mat = np.eye(4)
                    mat[:3, :3] = rot.as_matrix()
                    
                    # Translation
                    mat[0, 3] = t.x
                    mat[1, 3] = t.y
                    mat[2, 3] = t.z
                    
                    # Flatten to list
                    flat_mat = mat.flatten().tolist()
                    node.destroy_node()
                    return flat_mat
                    
            except Exception as e:
                print(f"TF lookup failed: {e}")
            
            node.destroy_node()
            
        except ImportError:
            print("Failed to import ROS libraries for TF query")
        except Exception as e:
            print(f"Error getting extrinsic: {e}")
            
        # Fallback: Return identity matrix if lookup fails
        print("⚠️ Using identity matrix for extrinsic (fallback)")
        return [1.0, 0.0, 0.0, 0.0, 
                0.0, 1.0, 0.0, 0.0, 
                0.0, 0.0, 1.0, 0.0, 
                0.0, 0.0, 0.0, 1.0]

    def _generate_removert_config(self, scan_dir: Path, pose_file: Path, result_dir: Path, config_file: Path, extrinsic: List[float]) -> None:
        """Generate YAML config for removert."""
        
        # Format extrinsic matrix as string list
        extrinsic_str = "[" + ", ".join(map(str, extrinsic)) + "]"
        
        config_content = f"""
        patchwork:
            mode: "czm" 
            verbose: true 
            visualize: false 
            ground_plane_estimation_method: "RANSAC" 

        removert:
            isScanFileKITTIFormat: true
            saveMapPCD: true
            
            # Input
            sequence_scan_dir: "{scan_dir}/"
            sequence_pose_path: "{pose_file}"
            
            # Output
            save_pcd_directory: "{result_dir}/"
            
            # Extrinsic (Lidar to Base)
            # This fixes the "messy map" issue by aligning scans correctly to the robot base
            ExtrinsicLiDARtoPoseBase: {extrinsic_str}
            
            # Keyframe
            use_keyframe_gap: true
            keyframe_gap: 20
            
            # Resolution
            downsample_voxel_size: 0.1
            remove_resolution_list: [2.5, 2.0, 1.5]
            revert_resolution_list: [1.0, 0.5]
            
            # FOV (Livox MID-360: 360 Horizontal, 59 Vertical)
            fov_up: 52.0
            fov_down: -7.0
            fov_left: 180.0
            fov_right: -180.0
            
            # Sensitivity
            num_nn_points_within: 2
            dist_nn_points_within: 0.1
        """
        with open(config_file, "w") as f:
            f.write(config_content)

    def _run_removert(self, config_file: Path) -> bool:
        """Run removert executable with real-time progress tracking."""
        tool_path = self.workdir / "src/removert/build/removert_removert"
        if not tool_path.exists():
            print(f"❌ removert tool not found at {tool_path}")
            return False
            
        cmd = [str(tool_path), str(config_file)]
        print(f"Running: {' '.join(cmd)}")
        
        try:
            # Use Popen to capture output in real-time
            process = subprocess.Popen(
                cmd,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
                bufsize=1,
                universal_newlines=True
            )
            
            # Read stdout line by line
            for line in process.stdout:
                line = line.strip()
                if not line:
                    continue
                    
                # Check for progress tags
                if "[REMOVERT_PROGRESS]" in line:
                    try:
                        # Expected format: [REMOVERT_PROGRESS] 25% - Map-side removal...
                        parts = line.split("[REMOVERT_PROGRESS]")
                        if len(parts) > 1:
                            content = parts[1].strip()
                            # Extract percentage
                            percent_str = content.split("%")[0].strip()
                            if percent_str.isdigit():
                                progress = int(percent_str)
                                description = content.split("-")[-1].strip() if "-" in content else content
                                
                                # Update controller status
                                self._save_progress = progress
                                self._save_stage = f"Removert: {description}"
                                print(f"🔄 Removert Progress: {progress}% - {description}")
                    except Exception as e:
                        print(f"Error parsing progress: {e}")
                else:
                    # Print other logs occasionally or to file?
                    # For now just print to console for debugging
                    if "Error" in line or "Warning" in line:
                        print(f"Removert: {line}")

            # Wait for process to complete
            returncode = process.wait()
            
            if returncode != 0:
                print(f"Removert failed with return code {returncode}")
                return False
                
            print(f"Removert finished successfully.")
            self._save_progress = 100
            return True
            
        except Exception as e:
            print(f"Removert execution error: {e}")
            return False

    def _generate_2d_map_from_pcd(self, pcd_file: Path, output_name: Path) -> bool:
        """Generate 2D map from PCD using pcd_to_2d_map tool."""
        tool_path = self.workdir / "install/tools/lib/tools/pcd_to_2d_map"
        if not tool_path.exists():
             tool_path = self.workdir / "build/tools/pcd_to_2d_map"
        
        if not tool_path.exists():
            print(f"❌ pcd_to_2d_map tool not found")
            return False
        
        # Usage: pcd_to_2d_map <pcd_file> <output_name_prefix> <resolution> <z_min> <z_max>
        cmd = [
            str(tool_path),
            str(pcd_file),
            str(output_name), # tool appends .pgm and .yaml
            "0.05", # resolution
            "0.2",  # z_min (filter ground)
            "2.0"   # z_max
        ]
        
        print(f"Running: {' '.join(cmd)}")
        try:
            result = subprocess.run(cmd, capture_output=True, text=True, timeout=60)
            if result.returncode != 0:
                print(f"2D map generation failed: {result.stderr}")
                return False
            return True
        except Exception as e:
            print(f"2D map generation error: {e}")
            return False

    def _update_map_registry(self, map_name: str, pcd_file: Path, pgm_file: Path, yaml_file: Path) -> None:
        """Update map_registry.yaml with the new map."""
        import yaml
        registry_file = self.workdir / "src/tron_slam/localizer/config/map_registry.yaml"
        
        if not registry_file.exists():
            print(f"Registry file not found: {registry_file}")
            return
        
        try:
            with open(registry_file, 'r') as f:
                registry = yaml.safe_load(f) or {}
            
            if 'maps' not in registry:
                registry['maps'] = {}
            
            registry['maps'][map_name] = {
                'description': f"Generated by removert from {self._map_name}",
                'pcd_file': str(pcd_file),
                'pgm_file': str(pgm_file),
                'yaml_file': str(yaml_file),
                'enabled': True
            }
            
            with open(registry_file, 'w') as f:
                yaml.dump(registry, f, default_flow_style=False)
            
            print(f"Updated registry for {map_name}")
        except Exception as e:
            print(f"Error updating registry: {e}")

    def _cleanup_rosbag(self) -> None:
        """Cleanup rosbag based on policy."""
        if not self._rosbag_path or not self._rosbag_path.exists():
            return
            
        if self.rosbag_cleanup_policy == "delete":
            print(f"🗑️ Deleting rosbag: {self._rosbag_path}")
            shutil.rmtree(self._rosbag_path, ignore_errors=True)
        elif self.rosbag_cleanup_policy == "archive":
            archive_dir = self.workdir / "archive/rosbags"
            archive_dir.mkdir(parents=True, exist_ok=True)
            print(f"📦 Archiving rosbag to: {archive_dir}")
            shutil.move(str(self._rosbag_path), str(archive_dir))
            
            # Cleanup old archives (> 7 days)
            try:
                current_time = time.time()
                for bag_dir in archive_dir.iterdir():
                    if bag_dir.is_dir() and (current_time - bag_dir.stat().st_mtime) > (7 * 86400):
                        shutil.rmtree(bag_dir, ignore_errors=True)
            except Exception:
                pass

    def status(self) -> dict:
        uptime = time.time() - self._start_time if self._start_time else 0.0
        # Check process status and clean up zombies
        alive_count = 0
        process_pids = []
        for proc in self._processes:
            ret = proc.poll()
            if ret is None:
                alive_count += 1
                process_pids.append(proc.pid)
        
        if self._state == "RUNNING" and alive_count == 0:
            self._state = "ERROR"

        rosbag_pid = None
        if self._rosbag_proc and self._rosbag_proc.poll() is None:
            rosbag_pid = self._rosbag_proc.pid
            process_pids.append(rosbag_pid)
        
        return {
            "status": self._state,
            "processes": process_pids,
            "uptime": uptime,
            "save_status": self._save_status,
            "save_in_progress": self._save_thread is not None and self._save_thread.is_alive(),
            "save_stage": getattr(self, "_save_stage", ""),
            "save_progress": getattr(self, "_save_progress", 0),
            "rosbag_pid": rosbag_pid,
            "rosbag_recording": rosbag_pid is not None,
            "rosbag_path": str(self._rosbag_path) if self._rosbag_path else None
        }

    @staticmethod
    def _default_launch() -> List[List[str]]:
        """Default launch sequence for 2D mapping with DLIO + Octomap."""
        return [
            # 1. Start Livox LiDAR driver
            ["ros2", "launch", "livox_ros_driver2", "msg_MID360_launch.py"],
            # 2. Convert Livox custom message to PointCloud2
            ["ros2", "launch", "tron_navigation", "livox_to_pointcloud2_launch.py"],
            # 3. Start DLIO for odometry and point cloud processing
            ["ros2", "launch", "direct_lidar_inertial_odometry", "dlio.launch.py", "rviz:=false"],
            # 4. Start Octomap to generate 2D occupancy grid from 3D point cloud
            ["ros2", "launch", "slam_2d", "octomap_launch.py"],
        ]
