#!/usr/bin/env python3
import base64
import random
import time
import threading
import math
from typing import Optional, Dict, Any

# 尝试导入ROS2相关模块，如果失败则使用模拟模式
try:
    import rclpy
    from rclpy.node import Node
    from nav_msgs.msg import OccupancyGrid, Odometry
    from geometry_msgs.msg import Twist, PoseStamped, Point, Quaternion
    from sensor_msgs.msg import BatteryState
    from std_msgs.msg import String
    ROS_AVAILABLE = True
except ImportError:
    ROS_AVAILABLE = False
    print("ROS2 not available, running in simulation mode")

from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
import uvicorn

# ---------------- 模拟数据类 ----------------
class SimulatedData:
    def __init__(self):
        self.map_width = 100
        self.map_height = 100
        self.resolution = 0.1
        self.origin_x = -5.0
        self.origin_y = -5.0
        
        # 生成模拟地图数据
        self.map_data = self._generate_simulated_map()
        
        # 模拟机器人状态
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_theta = 0.0
        self.battery_level = 85.0
        self.charging = False
        self.nav_status = "IDLE"
        
        # 添加导航状态
        self.navigation_status = "IDLE"
        
        # 移动控制参数
        self.current_vel_x = 0.0
        self.current_vel_y = 0.0
        self.current_vel_theta = 0.0
        self.target_x = None
        self.target_y = None
        self.target_theta = None
        self.movement_mode = "IDLE"  # IDLE, NAVIGATION, VELOCITY_CONTROL
        
        # BLOCKED状态恢复参数
        self.blocked_recovery_attempts = 0
        self.max_recovery_attempts = 3
        self.last_blocked_time = 0
        self.last_status_update = time.time()  # 添加这一行
        
        # 启动模拟数据更新线程
        self.running = True
        self.update_thread = threading.Thread(target=self._update_simulated_data)
        self.update_thread.daemon = True
        self.update_thread.start()
    def _update_navigation_status(self, status):
        """更新导航状态和时间戳"""
        self.navigation_status = status
        self.nav_status = status
        self.last_status_update = time.time()
        print(f"导航状态更新为: {status}")

    def _generate_simulated_map(self):
        """生成模拟的栅格地图数据"""
        data = []
        for y in range(self.map_height):
            for x in range(self.map_width):
                # 创建一个简单的室内地图
                # 边界是墙
                if x == 0 or x == self.map_width - 1 or y == 0 or y == self.map_height - 1:
                    data.append(100)  # 占用
                # 内部的一些障碍物
                elif (20 <= x <= 30 and 20 <= y <= 30) or \
                     (60 <= x <= 70 and 40 <= y <= 50) or \
                     (40 <= x <= 50 and 70 <= y <= 80):
                    data.append(100)  # 占用
                # 未知区域
                elif (x < 15 or x > 85) and (y < 15 or y > 85):
                    data.append(-1)  # 未知
                else:
                    data.append(0)  # 空闲
        return data
    
    def _is_position_valid(self, x, y):
        """检查位置是否有效（不碰撞障碍物）"""
        # 转换为地图坐标
        map_x = int((x - self.origin_x) / self.resolution)
        map_y = int((y - self.origin_y) / self.resolution)
        
        # 检查边界
        if map_x < 0 or map_x >= self.map_width or map_y < 0 or map_y >= self.map_height:
            return False
        
        # 检查障碍物
        index = map_y * self.map_width + map_x
        if index < len(self.map_data) and self.map_data[index] == 100:
            return False
        
        return True
    
    def _attempt_blocked_recovery(self):
        """尝试从BLOCKED状态恢复"""
        self.blocked_recovery_attempts += 1
        
        if self.blocked_recovery_attempts >= self.max_recovery_attempts:
            # 超过最大尝试次数，重置为IDLE状态
            self._update_navigation_status("FAILED")
            self.movement_mode = "IDLE"
            self.blocked_recovery_attempts = 0
            self.target_x = None
            self.target_y = None
            self.target_theta = None
            self.current_vel_x = 0
            self.current_vel_y = 0
            self.current_vel_theta = 0
            print("BLOCKED状态恢复：超过最大尝试次数，导航失败")
            return
        
        # 尝试后退一小段距离
        backup_distance = 0.2
        backup_x = self.robot_x - backup_distance * math.cos(self.robot_theta)
        backup_y = self.robot_y - backup_distance * math.sin(self.robot_theta)
        
        if self._is_position_valid(backup_x, backup_y):
            self.robot_x = backup_x
            self.robot_y = backup_y
            self._update_navigation_status("RECOVERING")
            print(f"BLOCKED状态恢复：尝试后退，第{self.blocked_recovery_attempts}次")
        else:
            # 如果后退也不行，尝试旋转
            self.robot_theta += 0.5  # 旋转约30度
            self._update_navigation_status("RECOVERING")
            print(f"BLOCKED状态恢复：尝试旋转，第{self.blocked_recovery_attempts}次")
    
    def reset_blocked_status(self):
        """手动重置BLOCKED状态"""
        self._update_navigation_status("IDLE")
        self.movement_mode = "IDLE"
        self.blocked_recovery_attempts = 0
        self.target_x = None
        self.target_y = None
        self.target_theta = None
        self.current_vel_x = 0
        self.current_vel_y = 0
        self.current_vel_theta = 0
        return {"Result": 0, "Error": "BLOCKED状态已重置"}
    
    def _update_navigation_movement(self):
        """更新导航移动"""
        if self.target_x is None or self.target_y is None:
            return
        
        # 计算到目标的距离和角度
        dx = self.target_x - self.robot_x
        dy = self.target_y - self.robot_y
        distance = (dx**2 + dy**2)**0.5
        
        # 如果距离很小，认为到达目标
        if distance < 0.1:
            self.robot_x = self.target_x
            self.robot_y = self.target_y
            if self.target_theta is not None:
                self.robot_theta = self.target_theta
            self._update_navigation_status("SUCCEEDED")
            self.movement_mode = "IDLE"
            self.target_x = None
            self.target_y = None
            self.target_theta = None
            print("导航成功到达目标")
            return
        
        # 计算目标角度
        target_angle = math.atan2(dy, dx)
        angle_diff = target_angle - self.robot_theta
        
        # 角度归一化到[-pi, pi]
        while angle_diff > math.pi:
            angle_diff -= 2 * math.pi
        while angle_diff < -math.pi:
            angle_diff += 2 * math.pi
        
        # 先调整角度，再移动
        if abs(angle_diff) > 0.1:
            # 旋转
            angular_vel = 0.5 if angle_diff > 0 else -0.5
            self.robot_theta += angular_vel * 0.1
        else:
            # 前进
            speed = min(0.2, distance * 0.5)  # 速度与距离成比例
            new_x = self.robot_x + speed * math.cos(self.robot_theta) * 0.1
            new_y = self.robot_y + speed * math.sin(self.robot_theta) * 0.1
            
            # 检查新位置是否有效
            if self._is_position_valid(new_x, new_y):
                self.robot_x = new_x
                self.robot_y = new_y
            else:
                # 如果路径被阻挡，停止导航
                self._update_navigation_status("FAILED")
                self.movement_mode = "IDLE"
                print("导航失败：路径被阻挡")
    
    def _update_velocity_movement(self):
        """更新速度控制移动"""
        if abs(self.current_vel_x) < 0.01 and abs(self.current_vel_y) < 0.01 and abs(self.current_vel_theta) < 0.01:
            self._update_navigation_status("IDLE")
            self.movement_mode = "IDLE"
            return
        
        # 计算新位置
        dt = 0.1  # 时间步长
        
        # 更新角度
        self.robot_theta += self.current_vel_theta * dt
        
        # 归一化角度
        self.robot_theta = self.robot_theta % (2 * math.pi)
        
        # 计算新位置
        new_x = self.robot_x + self.current_vel_x * dt * math.cos(self.robot_theta) - self.current_vel_y * dt * math.sin(self.robot_theta)
        new_y = self.robot_y + self.current_vel_x * dt * math.sin(self.robot_theta) + self.current_vel_y * dt * math.cos(self.robot_theta)
        
        # 检查新位置是否有效
        if self._is_position_valid(new_x, new_y):
            self.robot_x = new_x
            self.robot_y = new_y
        else:
            # 如果碰撞，停止移动
            self.current_vel_x = 0
            self.current_vel_y = 0
            self.current_vel_theta = 0
            self._update_navigation_status("FAILED")
            self.movement_mode = "IDLE"
            print("速度控制移动失败：碰撞检测")
    
    def _update_simulated_data(self):
        """更新模拟数据"""
        import math
        
        while self.running:
            # 根据移动模式更新机器人位置
            if self.movement_mode == "NAVIGATION":
                self._update_navigation_movement()
            elif self.movement_mode == "VELOCITY_CONTROL":
                self._update_velocity_movement()
            
            # 模拟电池消耗
            if self.movement_mode != "IDLE":
                self.battery_level = max(0, self.battery_level - 0.02)
            else:
                self.battery_level = max(0, self.battery_level - 0.001)
            
            # 模拟充电
            if self.charging:
                self.battery_level = min(100, self.battery_level + 0.1)
            
            # 低电量自动停止
            if self.battery_level < 5:
                self._update_navigation_status("FAILED")
                self.movement_mode = "IDLE"
                self.current_vel_x = 0
                self.current_vel_y = 0
                self.current_vel_theta = 0
                self.target_x = None
                self.target_y = None
                print("低电量自动停止，导航失败")
            
            time.sleep(0.1)
    
    def get_map_json(self):
        """获取模拟地图的JSON数据"""
        return {
            "Result": 0,
            "Error": "",
            "name": "simulated_map",
            "resolution": self.resolution,
            "width": self.map_width,
            "height": self.map_height,
            "origin_x": self.origin_x,
            "origin_y": self.origin_y,
            "origin_yaw": 0.0,
            "data": base64.b64encode(bytes([d & 0xFF for d in self.map_data])).decode("utf-8")
        }
    
    def get_status_json(self):
        """获取模拟状态的JSON数据"""
        return {
            "battery": {
                "power": self.battery_level,
                "charging": self.charging
            },
            "localization": {
                "status": 0,
                "x": self.robot_x,
                "y": self.robot_y,
                "theta": self.robot_theta,
                "reliability": 0.95
            },
            "navigation": {
                "status": self.nav_status,
                "blocked": self.nav_status == "BLOCKED",
                "goal_id": 0
            }
        }
    
    def get_navigation_status_json(self):
        """获取导航状态的JSON数据"""
        # 添加调试信息
        current_time = time.time()
        time_since_update = current_time - self.last_status_update
        
        status_info = {
            "status": self.navigation_status,
            "movement_mode": self.movement_mode,
            "last_update": time_since_update,
            "has_target": self.target_x is not None and self.target_y is not None,
            "robot_position": {"x": self.robot_x, "y": self.robot_y, "theta": self.robot_theta}
        }
        
        if self.target_x is not None and self.target_y is not None:
            status_info["target_position"] = {"x": self.target_x, "y": self.target_y, "theta": self.target_theta}
            distance = ((self.target_x - self.robot_x)**2 + (self.target_y - self.robot_y)**2)**0.5
            status_info["distance_to_target"] = distance
        
        return status_info
    
    def handle_goal(self, goal_json):
        """处理导航目标"""
        try:
            print(f"收到导航目标: {goal_json}")
            
            # 如果当前是BLOCKED状态，先重置
            if self.nav_status == "BLOCKED":
                self.nav_status = "IDLE"
                self.movement_mode = "IDLE"
                self.blocked_recovery_attempts = 0
            
            self.target_x = float(goal_json["goal_x"])
            self.target_y = float(goal_json["goal_y"])
            self.target_theta = float(goal_json.get("goal_theta", 0.0))
            
            # 检查目标位置是否有效
            if self._is_position_valid(self.target_x, self.target_y):
                self.movement_mode = "NAVIGATION"
                self._update_navigation_status("GOING")
                print(f"开始导航到目标: ({self.target_x}, {self.target_y})")
                return {"Result": 0, "Error": ""}
            else:
                # 目标位置无效，导航失败
                self._update_navigation_status("FAILED")
                print(f"目标位置无效: ({self.target_x}, {self.target_y})")
                return {"Result": 1, "Error": "Target position is not valid (collision with obstacle)"}
        except (KeyError, ValueError) as e:
            # 数据错误，导航失败
            self._update_navigation_status("FAILED")
            print(f"导航目标数据错误: {e}")
            return {"Result": 1, "Error": f"Invalid goal data: {str(e)}"}
    
    def handle_cmd_vel(self, nav_json):
        """处理速度指令"""
        try:
            print(f"收到速度指令: {nav_json}")
            
            # 如果当前是BLOCKED状态，先重置
            if self.nav_status == "BLOCKED":
                self.nav_status = "IDLE"
                self.movement_mode = "IDLE"
                self.blocked_recovery_attempts = 0
            
            self.current_vel_x = float(nav_json.get("vel_x", 0.0))
            self.current_vel_y = float(nav_json.get("vel_y", 0.0))
            self.current_vel_theta = float(nav_json.get("vel_theta", 0.0))
            
            # 如果有速度指令，进入速度控制模式
            if abs(self.current_vel_x) > 0.01 or abs(self.current_vel_y) > 0.01 or abs(self.current_vel_theta) > 0.01:
                self.movement_mode = "VELOCITY_CONTROL"
                self._update_navigation_status("GOING")
                # 清除导航目标
                self.target_x = None
                self.target_y = None
                self.target_theta = None
                print(f"开始速度控制: vx={self.current_vel_x}, vy={self.current_vel_y}, vtheta={self.current_vel_theta}")
            else:
                self.movement_mode = "IDLE"
                self._update_navigation_status("IDLE")
                print("停止速度控制")
            
            return {"Result": 0, "Error": ""}
        except (ValueError, TypeError) as e:
            # 数据错误，导航失败
            self._update_navigation_status("FAILED")
            print(f"速度指令数据错误: {e}")
            return {"Result": 1, "Error": f"Invalid velocity data: {str(e)}"}
    
    def pause_navigation(self):
        """暂停导航"""
        if self.navigation_status == "GOING":
            self._update_navigation_status("PAUSED")
            self.movement_mode = "IDLE"
            print("导航已暂停")
            return {"Result": 0, "Error": "Navigation paused"}
        else:
            print(f"无法暂停导航，当前状态: {self.navigation_status}")
            return {"Result": 1, "Error": "Navigation is not in progress"}
    
    def resume_navigation(self):
        """恢复导航"""
        if self.navigation_status == "PAUSED":
            if self.target_x is not None and self.target_y is not None:
                self.movement_mode = "NAVIGATION"
                self._update_navigation_status("GOING")
                print("导航已恢复")
                return {"Result": 0, "Error": "Navigation resumed"}
            else:
                print("无法恢复导航：没有导航目标")
                return {"Result": 1, "Error": "No navigation target to resume"}
        else:
            print(f"无法恢复导航，当前状态: {self.navigation_status}")
            return {"Result": 1, "Error": "Navigation is not paused"}
    
    def stop(self):
        """停止模拟数据更新"""
        self.running = False

# ---------------- ROS2 Node ----------------
class RosBridge(Node):
    def __init__(self):
        super().__init__("ros_web_server")

        # 最新数据缓存
        self.latest_map = None
        self.latest_odom = None
        self.latest_battery = None
        self.latest_nav_status = None
        self.latest_navigation_status = None

        # 订阅话题
        self.create_subscription(
            OccupancyGrid, "/map", self.map_callback, 10)
        self.create_subscription(
            Odometry, "/odom", self.odom_callback, 10)
        self.create_subscription(
            BatteryState, "/battery", self.battery_callback, 10)
        self.create_subscription(
            String, "/nav_status", self.nav_status_callback, 10)
        self.create_subscription(
            String, "/navigation_status", self.navigation_status_callback, 10)

        # 发布器
        self.goal_pub = self.create_publisher(PoseStamped, "/goal", 10)
        self.cmd_vel_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.pause_pub = self.create_publisher(String, "/pause_navigation", 10)
        self.resume_pub = self.create_publisher(String, "/resume_navigation", 10)

    # -------- 回调函数 --------
    def map_callback(self, msg: OccupancyGrid):
        self.latest_map = msg

    def odom_callback(self, msg: Odometry):
        self.latest_odom = msg

    def battery_callback(self, msg: BatteryState):
        self.latest_battery = msg

    def nav_status_callback(self, msg: String):
        self.latest_nav_status = msg

    def navigation_status_callback(self, msg: String):
        self.latest_navigation_status = msg

    # -------- 工具函数：获取地图 JSON --------
    def get_map_json(self):
        if not self.latest_map:
            return {"Result": 1, "Error": "map not received yet"}
        m = self.latest_map
        return {
            "Result": 0,
            "Error": "",
            "name": "map",
            "resolution": float(m.info.resolution),
            "width": int(m.info.width),
            "height": int(m.info.height),
            "origin_x": float(m.info.origin.position.x),
            "origin_y": float(m.info.origin.position.y),
            "origin_yaw": 0.0,
            "data": base64.b64encode(bytes([d & 0xFF for d in m.data])).decode("utf-8")
        }

    def get_navigation_status_json(self):
        """获取导航状态的JSON数据"""
        if self.latest_navigation_status:
            status = self.latest_navigation_status.data
        elif self.latest_nav_status:
            # 如果没有专门的导航状态话题，根据导航状态推断
            nav_status = self.latest_nav_status.data
            if nav_status in ["IDLE", "SUCCEEDED"]:
                status = "IDLE"
            elif nav_status in ["MOVING", "GOING"]:
                status = "GOING"
            elif nav_status == "PAUSED":
                status = "PAUSED"
            else:
                status = "FAILED"
        else:
            status = "IDLE"
        
        return {
            "status": status
        }

    # -------- 发布目标点 --------
    def publish_goal(self, goal_json):
        msg = PoseStamped()
        msg.header.frame_id = "map"
        msg.pose.position.x = float(goal_json["goal_x"])
        msg.pose.position.y = float(goal_json["goal_y"])

        from math import sin, cos
        yaw = float(goal_json["goal_theta"])
        msg.pose.orientation.z = sin(yaw / 2.0)
        msg.pose.orientation.w = cos(yaw / 2.0)

        self.goal_pub.publish(msg)

    # -------- 发布速度指令 --------
    def publish_cmd_vel(self, nav_json):
        msg = Twist()
        msg.linear.x = float(nav_json.get("vel_x", 0.0))
        msg.linear.y = float(nav_json.get("vel_y", 0.0))
        msg.angular.z = float(nav_json.get("vel_theta", 0.0))
        self.cmd_vel_pub.publish(msg)

    # -------- 发布暂停导航指令 --------
    def publish_pause_navigation(self):
        msg = String()
        msg.data = "pause"
        self.pause_pub.publish(msg)

    # -------- 发布恢复导航指令 --------
    def publish_resume_navigation(self):
        msg = String()
        msg.data = "resume"
        self.resume_pub.publish(msg)

# ---------------- FastAPI ----------------
app = FastAPI()

app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# 全局变量
ros_node: Optional[RosBridge] = None
simulated_data: Optional[SimulatedData] = None
current_mode = "simulation"  # 默认模式

def get_current_mode():
    """获取当前模式"""
    return current_mode

def set_mode(mode: str):
    """设置运行模式"""
    global current_mode, ros_node, simulated_data
    
    if mode not in ["simulation", "ros"]:
        return {"Result": 1, "Error": "Invalid mode. Use 'simulation' or 'ros'"}
    
    current_mode = mode
    
    if mode == "simulation":
        # 停止ROS节点（如果存在）
        if ros_node:
            rclpy.shutdown()
            ros_node = None
        
        # 启动模拟数据
        if not simulated_data:
            simulated_data = SimulatedData()
    
    elif mode == "ros":
        # 停止模拟数据
        if simulated_data:
            simulated_data.stop()
            simulated_data = None
        
        # 启动ROS节点
        if not ros_node and ROS_AVAILABLE:
            rclpy.init()
            ros_node = RosBridge()
            
            # 启动ROS2独立线程
            t = threading.Thread(target=lambda: rclpy.spin(ros_node), daemon=True)
            t.start()
        elif not ROS_AVAILABLE:
            return {"Result": 1, "Error": "ROS2 not available"}
    
    return {"Result": 0, "Error": "", "mode": mode}

@app.get("/api/mode")
async def get_mode():
    """获取当前模式"""
    return {"mode": current_mode, "ros_available": ROS_AVAILABLE}

@app.post("/api/mode")
async def change_mode(mode_data: Dict[str, str]):
    """切换模式"""
    mode = mode_data.get("mode", "simulation")
    return set_mode(mode)

@app.get("/api/robot/map")
async def get_map():
    """获取地图数据"""
    if current_mode == "simulation":
        if simulated_data:
            return simulated_data.get_map_json()
        else:
            return {"Result": 1, "Error": "Simulation data not initialized"}
    else:  # ros mode
        if ros_node:
            return ros_node.get_map_json()
        else:
            return {"Result": 1, "Error": "ROS node not initialized"}

@app.get("/api/robot/status")
async def get_status():
    """获取机器人状态"""
    if current_mode == "simulation":
        if simulated_data:
            return simulated_data.get_status_json()
        else:
            return {"Result": 1, "Error": "Simulation data not initialized"}
    else:  # ros mode
        if ros_node:
            return {
                "battery": {
                    "power": float(ros_node.latest_battery.percentage)
                    if ros_node.latest_battery else 0.0,
                    "charging": ros_node.latest_battery.power_supply_status == 1
                    if ros_node.latest_battery else False
                },
                "localization": {
                    "status": 0 if ros_node.latest_odom else 1,
                    "x": float(ros_node.latest_odom.pose.pose.position.x)
                    if ros_node.latest_odom else 0.0,
                    "y": float(ros_node.latest_odom.pose.pose.position.y)
                    if ros_node.latest_odom else 0.0,
                    "theta": 0.0,
                    "reliability": 0.95
                },
                "navigation": {
                    "status": ros_node.latest_nav_status.data
                    if ros_node.latest_nav_status else "IDLE",
                    "blocked": False,
                    "goal_id": 0
                }
            }
        else:
            return {"Result": 1, "Error": "ROS node not initialized"}

@app.get("/api/robot/navigation_status")
async def get_navigation_status():
    """获取导航状态"""
    if current_mode == "simulation":
        if simulated_data:
            nav_data = simulated_data.get_navigation_status_json()
            return {"Result": 0, "Error": "", **nav_data}
        else:
            return {"Result": 1, "Error": "Simulation data not initialized"}
    else:  # ros mode
        if ros_node:
            nav_data = ros_node.get_navigation_status_json()
            return {"Result": 0, "Error": "", **nav_data}
        else:
            return {"Result": 1, "Error": "ROS node not initialized"}

@app.post("/api/robot/navigation_goal")
async def navigation_goal(goal: dict):
    """发送导航目标"""
    if current_mode == "simulation":
        if simulated_data:
            return simulated_data.handle_goal(goal)
        else:
            return {"Result": 1, "Error": "Simulation data not initialized"}
    else:  # ros mode
        if ros_node:
            try:
                ros_node.publish_goal(goal)
                return {"Result": 0, "Error": ""}
            except Exception as e:
                return {"Result": 4, "Error": str(e)}
        else:
            return {"Result": 1, "Error": "ROS node not initialized"}

@app.post("/api/robot/cmd_vel")
async def navigation_cmd(nav: dict):
    """发送速度指令"""
    if current_mode == "simulation":
        if simulated_data:
            return simulated_data.handle_cmd_vel(nav)
        else:
            return {"Result": 1, "Error": "Simulation data not initialized"}
    else:  # ros mode
        if ros_node:
            try:
                ros_node.publish_cmd_vel(nav)
                return {"Result": 0, "Error": ""}
            except Exception as e:
                return {"Result": 4, "Error": str(e)}
        else:
            return {"Result": 1, "Error": "ROS node not initialized"}

@app.post("/api/robot/pause_navigation")
async def pause_navigation():
    """暂停导航"""
    if current_mode == "simulation":
        if simulated_data:
            return simulated_data.pause_navigation()
        else:
            return {"Result": 1, "Error": "Simulation data not initialized"}
    else:  # ros mode
        if ros_node:
            try:
                ros_node.publish_pause_navigation()
                return {"Result": 0, "Error": ""}
            except Exception as e:
                return {"Result": 4, "Error": str(e)}
        else:
            return {"Result": 1, "Error": "ROS node not initialized"}

@app.post("/api/robot/resume_navigation")
async def resume_navigation():
    """恢复导航"""
    if current_mode == "simulation":
        if simulated_data:
            return simulated_data.resume_navigation()
        else:
            return {"Result": 1, "Error": "Simulation data not initialized"}
    else:  # ros mode
        if ros_node:
            try:
                ros_node.publish_resume_navigation()
                return {"Result": 0, "Error": ""}
            except Exception as e:
                return {"Result": 4, "Error": str(e)}
        else:
            return {"Result": 1, "Error": "ROS node not initialized"}

# ---------------- Main ----------------
def main():
    import argparse
    
    parser = argparse.ArgumentParser(description="ROS2 Web Server with Simulation Mode")
    parser.add_argument("--mode", choices=["simulation", "ros"], default="simulation",
                       help="Run mode: simulation (default) or ros")
    parser.add_argument("--host", default="0.0.0.0", help="Host to bind to")
    parser.add_argument("--port", type=int, default=8000, help="Port to bind to")
    
    args = parser.parse_args()
    
    # 设置初始模式
    result = set_mode(args.mode)
    if result["Result"] != 0:
        print(f"Failed to set mode: {result['Error']}")
        return
    
    print(f"Starting server in {args.mode} mode")
    print(f"ROS2 available: {ROS_AVAILABLE}")
    print(f"Server running on http://{args.host}:{args.port}")
    
    # 启动 FastAPI
    uvicorn.run(app, host=args.host, port=args.port)

if __name__ == "__main__":
    main()