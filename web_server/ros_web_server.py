#!/usr/bin/env python3
import base64
import time
from typing import Optional, Dict, Any

# 导入ROS2相关模块
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, Odometry
from geometry_msgs.msg import Twist, PoseStamped, Point, Quaternion
from sensor_msgs.msg import BatteryState
from std_msgs.msg import String

from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
import uvicorn

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
        self.goal_id_pub = self.create_publisher(String, "/goal_id", 10)
        self.cmd_vel_pub = self.create_publisher(Twist, "/cmd_vel_web", 10)
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
        # 发布目标位置
        msg = PoseStamped()
        msg.header.frame_id = "map"
        msg.pose.position.x = float(goal_json["goal_x"])
        msg.pose.position.y = float(goal_json["goal_y"])

        from math import sin, cos
        yaw = float(goal_json["goal_theta"])
        msg.pose.orientation.z = sin(yaw / 2.0)
        msg.pose.orientation.w = cos(yaw / 2.0)

        self.goal_pub.publish(msg)
        
        # 发布目标点ID
        if "goal_id" in goal_json:
            goal_id_msg = String()
            goal_id_msg.data = str(goal_json["goal_id"])
            self.goal_id_pub.publish(goal_id_msg)
            print(f"发布目标点ID: {goal_id_msg.data}")

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

def init_ros_node():
    """初始化ROS节点"""
    global ros_node
    if not ros_node:
        rclpy.init()
        ros_node = RosBridge()
        
        # 启动ROS2独立线程
        import threading
        t = threading.Thread(target=lambda: rclpy.spin(ros_node), daemon=True)
        t.start()
        print("ROS节点已初始化")

@app.get("/api/robot/map")
async def get_map():
    """获取地图数据"""
    if ros_node:
        return ros_node.get_map_json()
    else:
        return {"Result": 1, "Error": "ROS node not initialized"}

@app.get("/api/robot/status")
async def get_status():
    """获取机器人状态"""
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
    if ros_node:
        nav_data = ros_node.get_navigation_status_json()
        return {"Result": 0, "Error": "", **nav_data}
    else:
        return {"Result": 1, "Error": "ROS node not initialized"}

@app.post("/api/robot/navigation_goal")
async def navigation_goal(goal: dict):
    """发送导航目标"""
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
    if ros_node:
        try:
            ros_node.publish_resume_navigation()
            return {"Result": 0, "Error": ""}
        except Exception as e:
            return {"Result": 4, "Error": str(e)}
    else:
        return {"Result": 1, "Error": "ROS node not initialized"}

@app.post("/api/inspection/callback")
async def inspection_callback(callback_data: dict):
    """
    导航结果回调通知接口
    导航任务执行完毕后，导航系统通知巡检平台任务执行状态
    """
    try:
        # 验证必需的参数
        required_fields = ["robot_id", "task_id", "execution_status", "execution_time"]
        for field in required_fields:
            if field not in callback_data:
                return {
                    "code": 400,
                    "msg": f"Missing required field: {field}"
                }
        
        robot_id = callback_data["robot_id"]
        task_id = callback_data["task_id"]
        execution_status = callback_data["execution_status"]
        execution_time = callback_data["execution_time"]
        
        # 验证execution_status的值
        valid_statuses = ["SUCCEEDED", "FAILED", "success"]
        if execution_status not in valid_statuses:
            return {
                "code": 400,
                "msg": f"Invalid execution_status. Must be one of: {valid_statuses}"
            }
        
        # 验证execution_time是否为有效的13位时间戳
        if not isinstance(execution_time, int) or execution_time < 1000000000000 or execution_time > 9999999999999:
            return {
                "code": 400,
                "msg": "Invalid execution_time. Must be a 13-digit UNIX timestamp in milliseconds"
            }
        
        # 记录回调信息
        print(f"收到导航结果回调:")
        print(f"  机器人ID: {robot_id}")
        print(f"  任务ID: {task_id}")
        print(f"  执行状态: {execution_status}")
        print(f"  执行时间: {execution_time}")
        
        # 返回成功响应
        return {
            "code": 200,
            "msg": "success"
        }
        
    except Exception as e:
        # 处理异常情况
        print(f"处理导航结果回调时发生错误: {str(e)}")
        return {
            "code": 500,
            "msg": f"Internal server error: {str(e)}"
        }

# ---------------- Main ----------------
def main():
    import argparse
    
    parser = argparse.ArgumentParser(description="ROS2 Web Server")
    parser.add_argument("--host", default="10.240.6.173", help="Host to bind to")
    parser.add_argument("--port", type=int, default=8000, help="Port to bind to")
    
    args = parser.parse_args()
    
    # 初始化ROS节点
    init_ros_node()
    
    print(f"Starting ROS2 Web Server")
    print(f"Server running on http://{args.host}:{args.port}")
    
    # 启动 FastAPI
    uvicorn.run(app, host=args.host, port=args.port)

if __name__ == "__main__":
    main()