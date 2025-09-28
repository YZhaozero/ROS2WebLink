#!/usr/bin/env python3
import base64
import random
import time
import threading
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
        
        # 启动模拟数据更新线程
        self.running = True
        self.update_thread = threading.Thread(target=self._update_simulated_data)
        self.update_thread.daemon = True
        self.update_thread.start()
    
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
    
    def _update_simulated_data(self):
        """更新模拟数据"""
        while self.running:
            # 模拟机器人移动
            if self.nav_status == "MOVING":
                self.robot_x += random.uniform(-0.1, 0.1)
                self.robot_y += random.uniform(-0.1, 0.1)
                self.robot_theta += random.uniform(-0.1, 0.1)
                
                # 模拟电池消耗
                self.battery_level = max(0, self.battery_level - 0.01)
                
                # 随机改变导航状态
                if random.random() < 0.05:
                    self.nav_status = "IDLE"
            
            # 模拟充电
            if self.charging:
                self.battery_level = min(100, self.battery_level + 0.1)
            
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
                "blocked": False,
                "goal_id": 0
            }
        }
    
    def handle_goal(self, goal_json):
        """处理导航目标"""
        self.nav_status = "MOVING"
        return {"Result": 0, "Error": ""}
    
    def handle_cmd_vel(self, nav_json):
        """处理速度指令"""
        if nav_json.get("vel_x", 0) != 0 or nav_json.get("vel_y", 0) != 0:
            self.nav_status = "MOVING"
        return {"Result": 0, "Error": ""}
    
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

        # 订阅话题
        self.create_subscription(
            OccupancyGrid, "/map", self.map_callback, 10)
        self.create_subscription(
            Odometry, "/odom", self.odom_callback, 10)
        self.create_subscription(
            BatteryState, "/battery", self.battery_callback, 10)
        self.create_subscription(
            String, "/nav_status", self.nav_status_callback, 10)

        # 发布器
        self.goal_pub = self.create_publisher(PoseStamped, "/goal", 10)
        self.cmd_vel_pub = self.create_publisher(Twist, "/cmd_vel", 10)

    # -------- 回调函数 --------
    def map_callback(self, msg: OccupancyGrid):
        self.latest_map = msg

    def odom_callback(self, msg: Odometry):
        self.latest_odom = msg

    def battery_callback(self, msg: BatteryState):
        self.latest_battery = msg

    def nav_status_callback(self, msg: String):
        self.latest_nav_status = msg

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