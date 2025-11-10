"""模拟机器人WebSocket服务器，主动推送电池电量等信息"""

import asyncio
import json
import logging
import time
import uuid
from dataclasses import dataclass, field
from typing import Dict, Any, Optional, Set
import websockets

logger = logging.getLogger(__name__)


@dataclass
class MockRobotState:
    """模拟机器人状态"""
    battery_level: float = 100.0  # 电量百分比
    battery_voltage: float = 24.5  # 电压
    battery_temperature: float = 25.0  # 温度
    is_charging: bool = False
    robot_mode: str = "stand"  # 机器人模式
    robot_position: Dict[str, float] = field(default_factory=lambda: {"x": 0.0, "y": 0.0, "z": 0.0})
    robot_orientation: Dict[str, float] = field(default_factory=lambda: {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0})
    navigation_status: str = "idle"
    imu_data: Dict[str, float] = field(default_factory=lambda: {
        "linear_acceleration": {"x": 0.0, "y": 0.0, "z": 9.8},
        "angular_velocity": {"x": 0.0, "y": 0.0, "z": 0.0},
        "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0}
    })
    odom_data: Dict[str, Any] = field(default_factory=lambda: {
        "position": {"x": 0.0, "y": 0.0, "z": 0.0},
        "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0},
        "linear_velocity": {"x": 0.0, "y": 0.0, "z": 0.0},
        "angular_velocity": {"x": 0.0, "y": 0.0, "z": 0.0}
    })


class MockRobotServer:
    """模拟机器人WebSocket服务器"""
    
    def __init__(self, host: str = "localhost", port: int = 5000):
        self.host = host
        self.port = port
        self.state = MockRobotState()
        self.connected_clients: Set[websockets.WebSocketServerProtocol] = set()
        self.server: Optional[websockets.Server] = None
        self._running = False
        
    async def start(self):
        """启动服务器"""
        self._running = True
        self.server = await websockets.serve(
            self._handle_client,
            self.host,
            self.port
        )
        logger.info(f"模拟机器人服务器启动在 ws://{self.host}:{self.port}")
        
        # 启动状态更新任务
        asyncio.create_task(self._update_robot_state())
        
    async def stop(self):
        """停止服务器"""
        self._running = False
        if self.server:
            self.server.close()
            await self.server.wait_closed()
        logger.info("模拟机器人服务器已停止")
    
    async def _handle_client(self, websocket):
        """处理客户端连接"""
        self.connected_clients.add(websocket)
        logger.info(f"新客户端连接: {websocket.remote_address}")
        
        try:
            # 发送初始机器人信息
            await self._send_robot_info(websocket)
            
            # 处理客户端消息
            async for message in websocket:
                await self._handle_message(websocket, message)
                
        except websockets.exceptions.ConnectionClosed:
            logger.info(f"客户端断开连接: {websocket.remote_address}")
        except Exception as e:
            logger.error(f"处理客户端连接时出错: {e}")
        finally:
            if websocket in self.connected_clients:
                self.connected_clients.remove(websocket)
    
    async def _handle_message(self, websocket, message: str):
        """处理客户端消息"""
        try:
            payload = json.loads(message)
            title = payload.get("title")
            data = payload.get("data", {})
            
            logger.info(f"收到客户端命令: {title}, 数据: {data}")
            
            # 处理各种命令
            if title == "request_stand_mode":
                self.state.robot_mode = "stand"
                await self._send_response(websocket, title, {"status": "success"})
                
            elif title == "request_walk_mode":
                self.state.robot_mode = "walk"
                await self._send_response(websocket, title, {"status": "success"})
                
            elif title == "request_sitdown":
                self.state.robot_mode = "sit"
                await self._send_response(websocket, title, {"status": "success"})
                
            elif title == "request_stair_mode":
                enable = data.get("enable", False)
                self.state.robot_mode = "stair" if enable else "walk"
                await self._send_response(websocket, title, {"status": "success"})
                
            elif title == "request_twist":
                # 模拟移动
                x, y, z = data.get("x", 0), data.get("y", 0), data.get("z", 0)
                self.state.odom_data["linear_velocity"] = {"x": x, "y": y, "z": z}
                await self._send_response(websocket, title, {"status": "success"})
                
            elif title == "request_emgy_stop":
                self.state.robot_mode = "emergency_stop"
                await self._send_response(websocket, title, {"status": "success"})
                
            elif title == "request_pause":
                self.state.navigation_status = "paused"
                await self._send_response(websocket, title, {"status": "success"})
                
            elif title == "request_resume":
                self.state.navigation_status = "active"
                await self._send_response(websocket, title, {"status": "success"})
                
            else:
                await self._send_response(websocket, title, {"status": "unknown_command"})
                
        except json.JSONDecodeError:
            logger.warning(f"无效的JSON消息: {message}")
        except Exception as e:
            logger.error(f"处理消息时出错: {e}")
    
    async def _update_robot_state(self):
        """定期更新机器人状态并推送"""
        while self._running:
            try:
                # 模拟电池消耗（每10秒消耗1%）
                if not self.state.is_charging:
                    self.state.battery_level = max(0, self.state.battery_level - 0.1)
                    self.state.battery_voltage = max(20.0, self.state.battery_voltage - 0.01)
                else:
                    # 充电时增加电量
                    self.state.battery_level = min(100, self.state.battery_level + 0.5)
                    self.state.battery_voltage = min(25.0, self.state.battery_voltage + 0.05)
                
                # 模拟温度变化
                self.state.battery_temperature = 20 + (100 - self.state.battery_level) * 0.1
                
                # 模拟位置变化（如果正在移动）
                if self.state.robot_mode == "walk":
                    vel = self.state.odom_data["linear_velocity"]
                    self.state.robot_position["x"] += vel["x"] * 0.1
                    self.state.robot_position["y"] += vel["y"] * 0.1
                    self.state.odom_data["position"] = self.state.robot_position.copy()
                
                # 推送状态更新到所有连接的客户端
                await self._broadcast_robot_info()
                await self._broadcast_imu_data()
                await self._broadcast_odom_data()
                await self._broadcast_navigation_status()
                
                # 每10秒推送一次电池低电量警告
                if self.state.battery_level < 20 and int(time.time()) % 10 == 0:
                    await self._broadcast_low_battery_warning()
                
            except Exception as e:
                logger.error(f"状态更新错误: {e}")
            
            await asyncio.sleep(1)  # 每秒更新一次
    
    async def _send_robot_info(self, websocket):
        """发送机器人信息 - 适配robot_sdk_bridge.py和mapping_nav_server.py的数据结构"""
        payload = {
            "accid": "mock_robot_001",
            "title": "notify_robot_info",
            "timestamp": int(time.time() * 1000),
            "guid": str(uuid.uuid4()),
            "data": {
                # 适配mapping_nav_server.py期望的数据结构
                "battery": round(self.state.battery_level, 1),  # 直接数值，不是嵌套对象
                "status": self.state.robot_mode,
                "mode": self.state.robot_mode,
                "position": self.state.robot_position,
                "orientation": self.state.robot_orientation
            }
        }
        await websocket.send(json.dumps(payload))
    
    async def _broadcast_robot_info(self):
        """广播机器人信息给所有客户端"""
        if not self.connected_clients:
            return
            
        payload = {
            "accid": "mock_robot_001",
            "title": "notify_robot_info",
            "timestamp": int(time.time() * 1000),
            "guid": str(uuid.uuid4()),
            "data": {
                # 适配mapping_nav_server.py期望的数据结构
                "battery": round(self.state.battery_level, 1),  # 直接数值，不是嵌套对象
                "status": self.state.robot_mode,
                "mode": self.state.robot_mode,
                "position": self.state.robot_position,
                "orientation": self.state.robot_orientation
            }
        }
        
        message = json.dumps(payload)
        disconnected_clients = []
        for client in self.connected_clients:
            try:
                await client.send(message)
            except websockets.exceptions.ConnectionClosed:
                disconnected_clients.append(client)
            except Exception as e:
                logger.error(f"发送消息到客户端时出错: {e}")
                disconnected_clients.append(client)
        
        # 清理断开的客户端
        for client in disconnected_clients:
            if client in self.connected_clients:
                self.connected_clients.remove(client)
    
    async def _broadcast_imu_data(self):
        """广播IMU数据"""
        if not self.connected_clients:
            return
            
        payload = {
            "accid": "mock_robot_001",
            "title": "notify_imu",
            "timestamp": int(time.time() * 1000),
            "guid": str(uuid.uuid4()),
            "data": self.state.imu_data
        }
        
        message = json.dumps(payload)
        disconnected_clients = []
        for client in self.connected_clients:
            try:
                await client.send(message)
            except websockets.exceptions.ConnectionClosed:
                disconnected_clients.append(client)
            except Exception as e:
                logger.error(f"发送IMU数据时出错: {e}")
                disconnected_clients.append(client)
        
        # 清理断开的客户端
        for client in disconnected_clients:
            if client in self.connected_clients:
                self.connected_clients.remove(client)
    
    async def _broadcast_odom_data(self):
        """广播里程计数据"""
        if not self.connected_clients:
            return
            
        payload = {
            "accid": "mock_robot_001",
            "title": "notify_odom",
            "timestamp": int(time.time() * 1000),
            "guid": str(uuid.uuid4()),
            "data": self.state.odom_data
        }
        
        message = json.dumps(payload)
        disconnected_clients = []
        for client in self.connected_clients:
            try:
                await client.send(message)
            except websockets.exceptions.ConnectionClosed:
                disconnected_clients.append(client)
            except Exception as e:
                logger.error(f"发送里程计数据时出错: {e}")
                disconnected_clients.append(client)
        
        # 清理断开的客户端
        for client in disconnected_clients:
            if client in self.connected_clients:
                self.connected_clients.remove(client)
    
    async def _broadcast_navigation_status(self):
        """广播导航状态"""
        if not self.connected_clients:
            return
            
        payload = {
            "accid": "mock_robot_001",
            "title": "notify_nav_status",
            "timestamp": int(time.time() * 1000),
            "guid": str(uuid.uuid4()),
            "data": {
                "status": self.state.navigation_status,
                "goal": {"x": 5.0, "y": 3.0, "z": 0.0}
            }
        }
        
        message = json.dumps(payload)
        disconnected_clients = []
        for client in self.connected_clients:
            try:
                await client.send(message)
            except websockets.exceptions.ConnectionClosed:
                disconnected_clients.append(client)
            except Exception as e:
                logger.error(f"发送导航状态时出错: {e}")
                disconnected_clients.append(client)
        
        # 清理断开的客户端
        for client in disconnected_clients:
            if client in self.connected_clients:
                self.connected_clients.remove(client)
    
    async def _broadcast_low_battery_warning(self):
        """广播低电量警告"""
        if not self.connected_clients:
            return
            
        payload = {
            "accid": "mock_robot_001",
            "title": "notify_battery_warning",
            "timestamp": int(time.time() * 1000),
            "guid": str(uuid.uuid4()),
            "data": {
                "level": round(self.state.battery_level, 1),
                "warning": "low_battery",
                "message": f"电池电量低: {self.state.battery_level}%"
            }
        }
        
        message = json.dumps(payload)
        disconnected_clients = []
        for client in self.connected_clients:
            try:
                await client.send(message)
            except websockets.exceptions.ConnectionClosed:
                disconnected_clients.append(client)
            except Exception as e:
                logger.error(f"发送电池警告时出错: {e}")
                disconnected_clients.append(client)
        
        # 清理断开的客户端
        for client in disconnected_clients:
            if client in self.connected_clients:
                self.connected_clients.remove(client)
    
    async def _send_response(self, websocket, command: str, data: Dict[str, Any]):
        """发送命令响应"""
        payload = {
            "accid": "mock_robot_001",
            "title": f"response_{command}",
            "timestamp": int(time.time() * 1000),
            "guid": str(uuid.uuid4()),
            "data": data
        }
        await websocket.send(json.dumps(payload))


async def main():
    """主函数"""
    server = MockRobotServer(host="0.0.0.0", port=5000)
    
    try:
        await server.start()
        # 保持服务器运行
        while True:
            await asyncio.sleep(1)
    except KeyboardInterrupt:
        logger.info("收到中断信号，正在停止服务器...")
    finally:
        await server.stop()


if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO)
    asyncio.run(main())