# ros_web_server 与 mapping_nav_server 兼容性总结

## 字段获取方式对比

### 1. `/api/robot/status` 接口

| 字段 | ros_web_server.py | mapping_nav_server.py | 状态 |
|------|-------------------|----------------------|------|
| **battery.power** | `ros_node.robot_sdk.robot_info.get("battery")` | `_robot_sdk.robot_info.get("battery")` | ✅ 一致 |
| **battery.charging** | `False` | `False` | ✅ 一致 |
| **localization.status** | `0 if ros_node.latest_odom else 1` | `0 if _ros_bridge.latest_odom else 1` | ✅ 一致 |
| **localization.x** | `ros_node.latest_odom.pose.pose.position.x` | `_ros_bridge.latest_odom.pose.pose.position.x` | ✅ 一致 |
| **localization.y** | `ros_node.latest_odom.pose.pose.position.y` | `_ros_bridge.latest_odom.pose.pose.position.y` | ✅ 一致 |
| **localization.theta** | `ros_node.quaternion_to_yaw(orientation)` | `_ros_bridge.quaternion_to_yaw(orientation)` | ✅ 一致 |
| **localization.reliability** | `0.95` | `0.95` | ✅ 一致 |
| **navigation.status** | `ros_node.latest_nav_status.data if ros_node.latest_nav_status else "IDLE"` | `_ros_bridge._latest_nav_status` | ✅ 一致 |
| **navigation.blocked** | `False` | `False` | ✅ 一致 |
| **navigation.goal_id** | `ros_node.current_task_id if ros_node.current_task_id is not None else 0` | `_ros_bridge.current_task_id if _ros_bridge.current_task_id is not None else 0` | ✅ 一致 |

### 2. `/api/robot/map` 接口

| 字段 | ros_web_server.py | mapping_nav_server.py | 状态 |
|------|-------------------|----------------------|------|
| **Result** | `0` (成功) / `1` (失败) | `0` (成功) / `1` (失败) | ✅ 一致 |
| **Error** | 错误信息字符串 | 错误信息字符串 | ✅ 一致 |
| **name** | `"map"` | `"map"` | ✅ 一致 |
| **resolution** | `float(m.info.resolution)` | `map_data["resolution"]` | ✅ 一致 |
| **width** | `int(m.info.width)` | `map_data["width"]` | ✅ 一致 |
| **height** | `int(m.info.height)` | `map_data["height"]` | ✅ 一致 |
| **origin_x** | `float(m.info.origin.position.x)` | `map_data["origin_x"]` | ✅ 一致 |
| **origin_y** | `float(m.info.origin.position.y)` | `map_data["origin_y"]` | ✅ 一致 |
| **origin_yaw** | `0.0` | `0.0` | ✅ 一致 |
| **data** | base64 编码 | base64 编码 | ✅ 一致 |

### 3. ROS Bridge 内部实现对比

#### ros_web_server.py (RosBridge 类)
```python
class RosBridge(Node):
    def __init__(self):
        # 原始数据缓存
        self.latest_map = None
        self.latest_odom = None  # 原始 Odometry 消息
        self.latest_nav_status = None  # String 消息对象
        self.current_task_id = None  # 当前任务 ID
        self.robot_sdk = RobotSDKBridge(...)
    
    def odom_callback(self, msg: Odometry):
        self.latest_odom = msg  # 直接保存原始消息
    
    def quaternion_to_yaw(self, q):
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny, cosy)
```

#### mapping_nav_server.py (ExtendedRosBridge 类)
```python
class ExtendedRosBridge:
    def __init__(self, ros_adapter: Optional[RosAdapter] = None):
        # 原始数据缓存
        self._latest_map: Optional[OccupancyGrid] = None
        self.latest_odom: Optional[Any] = None  # 原始 Odometry 消息（新增）
        self._latest_nav_status: str = "IDLE"  # 字符串格式
        self.current_task_id: Optional[int] = None  # 当前任务 ID（新增）
        self._current_goal_id: int = 0
    
    def _robot_odom_callback(self, msg) -> None:
        # 保存原始消息 + 处理后的数据
        if frame_id == "map" or current_frame != "map":
            self.latest_odom = msg  # 保存原始消息
        # ... 同时处理并保存到 self._robot_position
    
    def quaternion_to_yaw(self, q) -> float:
        # 完全相同的实现
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny, cosy)
```

### 4. 新增的兼容性属性和方法

在 `ExtendedRosBridge` 中新增以下属性和方法以确保与 `ros_web_server.py` 完全兼容：

| 属性/方法 | 用途 | 兼容性 |
|----------|------|--------|
| `latest_odom` | 保存原始 Odometry 消息 | ✅ 新增 |
| `current_task_id` | 当前任务 ID（与 _current_goal_id 同步） | ✅ 新增 |
| `quaternion_to_yaw(q)` | 四元数转 yaw 角度 | ✅ 新增 |

### 5. 订阅话题对比

| 话题 | ros_web_server.py | mapping_nav_server.py | 状态 |
|------|-------------------|----------------------|------|
| `/map` | ✅ | ✅ | 一致 |
| `/tron_commander/odom` | ✅ | ✅ | 一致 |
| `/dlio/odom_node/odom` | ❌ | ✅ | mapping 增强 |
| `/nav_status` | ✅ | ✅ | 一致 |
| `/navigation_status` | ✅ | ✅ | 一致 |
| `/goal` (发布) | ✅ | ✅ | 一致 |
| `/goal_id` (发布) | ✅ | ✅ | 一致 |
| `/cmd_vel` (发布) | ✅ | ✅ | 一致 |

### 6. 新增兼容接口

| 接口 | 功能 | 状态 |
|------|------|------|
| `POST /api/robot/control` | 发送速度指令（兼容接口） | ✅ 已添加 |
| `POST /api/robot/pause_navigation` | 暂停导航 | ✅ 已添加 |
| `POST /api/robot/resume_navigation` | 恢复导航 | ✅ 已添加 |
| `POST /api/inspection/callback` | 巡检任务回调 | ✅ 已添加 |
| `GET /api/inspection/nav_status` | 获取导航状态（用于巡检） | ✅ 已添加 |

## 关键改进点

### 1. 数据存储方式统一
- **之前**：`ExtendedRosBridge` 在回调中处理数据，只保存处理后的字典
- **现在**：同时保存原始 ROS 消息和处理后的数据，确保与 `ros_web_server` 获取方式一致

### 2. 字段计算逻辑统一
- **theta 计算**：都使用 `quaternion_to_yaw(orientation)` 方法
- **定位状态**：都基于 `latest_odom` 是否存在来判断
- **任务 ID**：都使用 `current_task_id` 属性

### 3. 帧优先级处理
`ExtendedRosBridge` 增强了帧优先级处理（`map` > `odom`），这比 `ros_web_server` 更智能，但保持了接口兼容性。

## 测试建议

1. **定位状态测试**
   ```bash
   curl http://localhost:8800/api/robot/status | jq '.localization.status'
   # 应返回 0（已定位）或 1（未定位）
   ```

2. **角度计算测试**
   ```bash
   curl http://localhost:8800/api/robot/status | jq '.localization.theta'
   # 应返回相同的角度值（使用相同的四元数转换方法）
   ```

3. **任务 ID 同步测试**
   ```bash
   # 发送导航目标
   curl -X POST http://localhost:8800/api/robot/navigation_goal \
     -H "Content-Type: application/json" \
     -d '{"goal_x": 1.0, "goal_y": 2.0, "goal_theta": 0.0, "goal_id": 123}'
   
   # 检查任务 ID
   curl http://localhost:8800/api/robot/status | jq '.navigation.goal_id'
   # 应返回 123
   ```

4. **地图数据格式测试**
   ```bash
   curl http://localhost:8800/api/robot/map | jq 'keys'
   # 应包含: Result, Error, name, resolution, width, height, origin_x, origin_y, origin_yaw, data
   ```

## 总结

✅ **完全兼容**：`mapping_nav_server.py` 现在使用与 `ros_web_server.py` 完全一致的字段获取方式
✅ **接口一致**：所有 API 接口的返回格式和字段完全相同
✅ **增强功能**：保留了 `mapping_nav_server.py` 的增强特性（如帧优先级、建图/导航分离）
✅ **向后兼容**：所有现有的前端代码无需修改即可使用


