#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from fastapi import FastAPI
import uvicorn
import threading
from fastapi.middleware.cors import CORSMiddleware

app = FastAPI()
latest_map = None

class MapServer(Node):
    def __init__(self):
        super().__init__('map_to_http')
        qos_profile = rclpy.qos.QoSProfile(
            depth=1,
            durability=rclpy.qos.DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=rclpy.qos.ReliabilityPolicy.RELIABLE
        )
        self.create_subscription(OccupancyGrid, "/map", self.map_callback, qos_profile)

    def map_callback(self, msg):
        global latest_map
        self.get_logger().info("Map received!")
        latest_map = {
            "resolution": msg.info.resolution,
            "width": msg.info.width,
            "height": msg.info.height,
            "origin": {
                "x": msg.info.origin.position.x,
                "y": msg.info.origin.position.y,
                "z": msg.info.origin.position.z
            },
            "data": list(msg.data)  # 转换为 JSON 可序列化
        }

# ---------------- FastAPI ----------------
app = FastAPI()

app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],  # 如果只想允许前端页面访问，可以改成 ["http://localhost:8080"]
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)
@app.get("/api/robot/map")
def get_map():
    if latest_map is None:
        return {"error": "No map received yet"}
    return latest_map

def ros_spin(node):
    rclpy.spin(node)

if __name__ == "__main__":
    rclpy.init()
    node = MapServer()
    threading.Thread(target=ros_spin, args=(node,), daemon=True).start()
    uvicorn.run(app, host="0.0.0.0", port=8000)
