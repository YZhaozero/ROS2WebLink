"""Navigation controller to orchestrate ROS navigation launch pipelines."""

from __future__ import annotations

import subprocess
import time
from pathlib import Path
from typing import List, Optional


class NavigationController:
    """Manages navigation system lifecycle (DLIO + GICP Localizer + pointcloud_to_laserscan + Nav2 + tron_commander)."""
    
    def __init__(self, workdir: Optional[Path] = None, map_name: Optional[str] = None) -> None:
        self.workdir = Path(workdir or ".").resolve()
        self.map_name = map_name or "final_test_map"
        
        self._processes: List[subprocess.Popen] = []
        self._state = "IDLE"  # IDLE, STARTING, RUNNING, ERROR
        self._start_time: Optional[float] = None
    
    def _validate_and_get_map_info(self, map_name: str) -> dict:
        """Validate map exists in registry and return map information."""
        import yaml
        
        registry_file = self.workdir / "src/tron_slam/localizer/config/map_registry.yaml"
        
        # Check if registry file exists
        if not registry_file.exists():
            return {
                "valid": False,
                "error": f"地图注册表文件不存在: {registry_file}",
                "map_name": map_name
            }
        
        # Load map registry
        try:
            with open(registry_file, 'r', encoding='utf-8') as f:
                registry = yaml.safe_load(f)
        except Exception as e:
            return {
                "valid": False,
                "error": f"无法读取地图注册表: {e}",
                "map_name": map_name
            }
        
        maps = registry.get('maps', {})
        
        # Check if map exists in registry
        if map_name not in maps:
            available_maps = list(maps.keys())
            return {
                "valid": False,
                "error": f"地图 '{map_name}' 未在注册表中找到",
                "map_name": map_name,
                "available_maps": available_maps,
                "hint": f"可用地图: {', '.join(available_maps)}"
            }
        
        map_entry = maps[map_name]
        
        # Check if map is enabled
        if not map_entry.get('enabled', True):
            return {
                "valid": False,
                "error": f"地图 '{map_name}' 已禁用",
                "map_name": map_name
            }
        
        # Validate PCD file exists
        pcd_file = Path(map_entry['pcd_file'])
        if not pcd_file.exists():
            return {
                "valid": False,
                "error": f"PCD地图文件不存在: {pcd_file}",
                "map_name": map_name,
                "pcd_file": str(pcd_file)
            }
        
        # Validate 2D map files exist
        pgm_file = Path(map_entry.get('pgm_file', ''))
        yaml_file = Path(map_entry.get('yaml_file', ''))
        
        missing_files = []
        if not pgm_file.exists():
            missing_files.append(f"PGM文件: {pgm_file}")
        if not yaml_file.exists():
            missing_files.append(f"YAML文件: {yaml_file}")
        
        if missing_files:
            return {
                "valid": False,
                "error": f"2D地图文件缺失: {', '.join(missing_files)}",
                "map_name": map_name,
                "pcd_file": str(pcd_file),
                "missing_files": missing_files
            }
        
        # All validation passed
        return {
            "valid": True,
            "map_name": map_name,
            "description": map_entry.get('description', ''),
            "pcd_file": str(pcd_file),
            "pgm_file": str(pgm_file),
            "yaml_file": str(yaml_file),
            "pcd_size_kb": pcd_file.stat().st_size / 1024
        }
    
    def _update_localizer_config(self, map_name: str) -> None:
        """Update localizer.yaml with the PCD map path for the given map name using map_registry."""
        import yaml
        
        # Load map registry
        registry_file = self.workdir / "src/tron_slam/localizer/config/map_registry.yaml"
        pcd_map_path = None
        
        if registry_file.exists():
            try:
                with open(registry_file, 'r') as f:
                    registry = yaml.safe_load(f)
                
                maps = registry.get('maps', {})
                default_map = registry.get('default_map', 'default')
                
                # Try to find map in registry
                if map_name in maps:
                    pcd_map_path = Path(maps[map_name]['pcd_file'])
                    print(f"✅ Found map '{map_name}' in registry: {pcd_map_path}")
                elif default_map in maps:
                    pcd_map_path = Path(maps[default_map]['pcd_file'])
                    print(f"⚠️  Map '{map_name}' not in registry, using default '{default_map}': {pcd_map_path}")
            except Exception as e:
                print(f"⚠️  Failed to load map registry: {e}")
        
        # Fallback: original logic if registry not available or map not found
        if not pcd_map_path or not pcd_map_path.exists():
            pcd_map_path = self.workdir / "src/tron_slam/localizer/PCD" / f"{map_name}.pcd"
            
            if not pcd_map_path.exists():
                pcd_map_path = self.workdir / "src/tron_slam/localizer/PCD/map.pcd"
                print(f"⚠️  PCD map for '{map_name}' not found, using default: {pcd_map_path}")
        
        # Update both source and install config files (same as mapping_controller)
        config_files = [
            self.workdir / "src/tron_slam/localizer/config/localizer.yaml",
            self.workdir / "install/localizer/share/localizer/config/localizer.yaml"
        ]
        
        import re
        for config_path in config_files:
            if config_path.exists():
                try:
                    with open(config_path, 'r') as f:
                        content = f.read()
                    
                    # Replace the default_map_path line
                    new_content = re.sub(
                        r'default_map_path:.*',
                        f'default_map_path: {pcd_map_path}',
                        content
                    )
                    
                    with open(config_path, 'w') as f:
                        f.write(new_content)
                    
                    print(f"✅ Updated localizer config: {config_path}")
                except Exception as e:
                    print(f"⚠️ Failed to update {config_path}: {e}")
            else:
                print(f"⚠️ Localizer config not found: {config_path}")

    def _get_current_nodes(self) -> set:
        """Return set of current ROS2 nodes."""
        try:
            out = subprocess.check_output(
                ["ros2", "node", "list"], encoding="utf-8", stderr=subprocess.DEVNULL
            )
            return set(out.strip().splitlines())
        except Exception:
            return set()

    def _wait_for_new_nodes(self, old_nodes: set, timeout: int = 20) -> bool:
        """Wait until new ROS2 nodes appear (indicating launch is running)."""
        start = time.time()
        while time.time() - start < timeout:
            new_nodes = self._get_current_nodes()
            if len(new_nodes) > len(old_nodes):
                print("   ✅ 节点启动成功")
                return True
            time.sleep(0.5)
        print("   ❌ 节点启动超时，可能启动失败")
        return False

    def start(self, map_name: Optional[str] = None) -> dict:
        """Start navigation system with all required nodes (auto-detect success)."""

        # ------ 状态检查 ------
        if self._state not in {"IDLE", "ERROR"}:
            raise RuntimeError("navigation already running")
        
        if map_name:
            self.map_name = map_name

        # ------ 地图验证 ------
        map_info = self._validate_and_get_map_info(self.map_name)
        if not map_info["valid"]:
            error_msg = f"❌ 地图验证失败: {map_info['error']}"
            print(error_msg)
            self._state = "ERROR"
            return {
                "status": "ERROR",
                "error": map_info["error"],
                "map_name": self.map_name,
                "map_info": map_info
            }

        # 打印地图信息
        print(f"✅ 地图验证成功:")
        print(f"   地图名称: {self.map_name}")
        print(f"   PCD地图: {map_info['pcd_file']}")
        print(f"   2D地图: {map_info['pgm_file']}")
        print(f"   描述: {map_info['description']}")

        # ------ 更新 localizer 配置 ------
        self._update_localizer_config(self.map_name)

        self._state = "STARTING"
        self._processes.clear()

        # ------ 启动序列 ------
        launch_sequence = [
            ["ros2", "launch", "livox_ros_driver2", "msg_MID360_launch.py"],
            ["ros2", "launch", "tron_navigation", "livox_to_pointcloud2_launch.py"],
            ["ros2", "launch", "direct_lidar_inertial_odometry", "dlio.launch.py", "rviz:=false"],
            ["ros2", "launch", "localizer", "localizer_launch.py"],
            ["ros2", "launch", "pointcloud_to_laserscan", "pointcloud_to_laserscan_launch.py"],
            ["ros2", "launch", "tron_navigation", "tron_bringup.launch.py",
                f"map:={self.workdir}/src/tron_nav/tron_navigation/maps/{self.map_name}.yaml",
                "use_sim_time:=false"],
            ["ros2", "launch", "cmd_bridge", "cmd_vel_bridge.launch.py"],
            ["ros2", "launch", "pid_parking", "pid_parking.launch.py"],
            ["python3", f"{self.workdir}/tools/tron_commander_pid.py"],
            # ["python3", f"{self.workdir}/tools/pointcloud_filter.py"],
        ]

        # -------------------------------------
        # 关键增强：逐个启动 + 自动检测 ROS 节点是否成功启动
        # -------------------------------------

        for idx, command in enumerate(launch_sequence):

            # 1. 获取启动前的 ROS 节点列表
            old_nodes = self._get_current_nodes()
            print(f"\n⏳ 当前 ROS 节点数: {len(old_nodes)}，准备启动第 {idx} 个节点...")
            print("   命令:", " ".join(command))

            # 2. 启动进程
            bash_cmd = [
                "bash", "-c",
                f"export FASTRTPS_DEFAULT_PROFILES_FILE=/home/guest/.config/fastdds/fastdds.xml && "
                f"source /opt/ros/humble/setup.bash && "
                f"source {self.workdir}/install/setup.bash && "
                f"{' '.join(command)}"
            ]
            
            log_file = f"/tmp/navigation_{idx}_{command[2] if len(command) > 2 else 'commander'}.log"
            with open(log_file, "w") as log:
                proc = subprocess.Popen(
                    bash_cmd,
                    cwd=self.workdir,
                    stdout=log,
                    stderr=subprocess.STDOUT
                )

            self._processes.append(proc)
            print(f"   ✔ 已启动进程 PID={proc.pid}, 日志文件: {log_file}")

            # 3. 自动检测该节点是否正确启动
            print(f"   ⏳ 等待节点启动中...")
            ok = self._wait_for_new_nodes(old_nodes, timeout=25)

            if not ok:
                print(f"❌ 第 {idx} 个 launch 启动失败，正在终止所有进程...")
                self.stop()
                self._state = "ERROR"
                return {
                    "status": "ERROR",
                    "error": f"Launch {command} failed to start",
                    "failed_launch": command,
                    "map_name": self.map_name,
                }

            print(f"   ✅ 第 {idx} 个 launch 启动成功！")

            # 可选：给每个进程 1s 缓冲
            time.sleep(1)

        # ------ 完成 ------
        self._state = "RUNNING"
        self._start_time = time.time()

        return {
            "status": self._state,
            "pids": [p.pid for p in self._processes],
            "map_name": self.map_name,
            "map_info": map_info
        }

    
    def stop(self) -> dict:
        """Stop all navigation processes."""
        if self._state not in {"RUNNING", "ERROR"}:
            return {"status": self._state}
        
        self._state = "STOPPING"
        
        # Terminate all processes in self._processes list
        for proc in self._processes:
            try:
                proc.terminate()
                proc.wait(timeout=5)
            except subprocess.TimeoutExpired:
                proc.kill()
            except Exception:
                pass
        
        self._processes.clear()
        
        # Additional cleanup: Force kill all navigation-related processes (including orphaned ones)
        # This ensures complete cleanup even if previous stop() calls failed
        import subprocess
        try:
            print("🧹 Force cleanup: killing all navigation-related processes...")
            subprocess.run([
                "pkill", "-9", "-f",
                "dlio|livox_ros_driver2|livox_to_pointcloud2|localizer_node|pointcloud_to_laserscan|tron_commander|nav2|pid_parking|pid_controller"
            ], capture_output=True)
            subprocess.run([
                "pkill", "-9", "-f", "static_transform_publisher.*livox"
            ], capture_output=True)
            print("✅ Force cleanup completed")
        except Exception as e:
            print(f"⚠️ Force cleanup exception: {e}")
        
        self._state = "IDLE"
        return {"status": self._state}
    
    def status(self) -> dict:
        """Get navigation system status."""
        uptime = time.time() - self._start_time if self._start_time else 0.0
        
        # Check process status
        alive_count = 0
        for proc in self._processes:
            ret = proc.poll()
            if ret is None:
                alive_count += 1
        
        if self._state == "RUNNING" and alive_count == 0:
            self._state = "ERROR"
        
        # Get current map info
        map_info = self._validate_and_get_map_info(self.map_name) if self._state != "IDLE" else None
        
        result = {
            "status": self._state,
            "processes": [proc.pid for proc in self._processes],
            "alive_count": alive_count,
            "total_count": len(self._processes),
            "uptime": uptime,
            "map_name": self.map_name,
        }
        
        # Add map info if available
        if map_info:
            result["map_info"] = map_info
            if map_info.get("valid"):
                result["pcd_file"] = map_info.get("pcd_file")
                result["pcd_size_kb"] = map_info.get("pcd_size_kb")
        
        return result


