#!/usr/bin/env python3
import base64
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, Odometry
from geometry_msgs.msg import Twist, PoseStamped
from sensor_msgs.msg import BatteryState
from std_msgs.msg import String

from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
import uvicorn
import threading

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

# ✅ 跨域允许
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],  # 如果想只允许本地前端访问，可改成 ["http://localhost:8080"]
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

ros_node: RosBridge = None


@app.get("/api/robot/map")
async def get_map():
    return ros_node.get_map_json()


@app.get("/api/robot/status")
async def get_status():
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


@app.post("/api/robot/navigation_goal")
async def navigation_goal(goal: dict):
    try:
        ros_node.publish_goal(goal)
        return {"Result": 0, "Error": ""}
    except Exception as e:
        return {"Result": 4, "Error": str(e)}


@app.post("/api/robot/cmd_vel")
async def navigation_cmd(nav: dict):
    try:
        ros_node.publish_cmd_vel(nav)
        return {"Result": 0, "Error": ""}
    except Exception as e:
        return {"Result": 4, "Error": str(e)}


# ---------------- Main ----------------
def ros_spin():
    rclpy.spin(ros_node)


def main():
    global ros_node
    rclpy.init()
    ros_node = RosBridge()

    # ROS2 独立线程 spin
    t = threading.Thread(target=ros_spin, daemon=True)
    t.start()

    # 启动 FastAPI
    uvicorn.run(app, host="0.0.0.0", port=8000)


if __name__ == "__main__":
    main()
