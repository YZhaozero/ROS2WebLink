# ROS2WebLink API 完整文档

## 📋 目录

- [系统概览](#系统概览)
- [建图 API](#建图-api)
- [地图管理 API](#地图管理-api)
- [导航 API](#导航-api)
- [机器人控制 API](#机器人控制-api)
- [航点路线 API](#航点路线-api)
- [传感器状态 API](#传感器状态-api)
- [巡检回调 API](#巡检回调-api)
- [WebSocket 协议](#websocket-协议)
- [前端页面说明](#前端页面说明)

---

## 系统概览

### 基础信息

| 项目 | 说明 |
|------|------|
| **Base URL** | `http://localhost:8800` |
| **版本** | 0.1.0 |
| **协议** | HTTP/REST + WebSocket |
| **数据格式** | JSON |
| **文档地址** | `/static/api_docs.html` (Web) 或 `/docs` (Swagger) |

### 技术栈

- **后端框架**: FastAPI (Python)
- **ROS2**: Humble
- **SLAM**: DLIO + Octomap (建图) / GICP Localizer (定位)
- **导航**: Nav2
- **硬件**: Livox Mid-360 LiDAR + 四足机器人

### 错误码约定

| 错误码 | 说明 |
|--------|------|
| `0` | 成功 |
| `1` | 参数错误或资源不存在 |
| `4` | 执行失败 |
| `409` | 冲突（如已经在运行） |
| `503` | 服务不可用 |

### 通用响应格式

大多数 API 返回以下 JSON 格式：

```json
{
  "Result": 0,       // 0=成功, 非0=错误
  "Error": ""        // 错误信息（成功时为空）
}
```

---

## 建图 API

### 1. 启动建图

**端点**: `POST /api/mapping/start`

**描述**: 启动建图流程，依次启动 Livox 驱动、DLIO、点云转换、Octomap 节点。

**请求体**:
```json
{
  "map_name": "office_floor1"  // 可选，默认为 map_YYYYMMDD_HHMMSS
}
```

**响应**:
```json
{
  "status": "RUNNING",
  "pids": [12345, 12346, 12347, 12348]
}
```

**状态说明**:
- `IDLE`: 未运行
- `STARTING`: 启动中
- `RUNNING`: 运行中
- `STOPPING`: 停止中
- `ERROR`: 错误

**cURL 示例**:
```bash
curl -X POST http://localhost:8800/api/mapping/start \
  -H "Content-Type: application/json" \
  -d '{"map_name": "office_floor1"}'
```

---

### 2. 停止建图

**端点**: `POST /api/mapping/stop`

**描述**: 停止建图并可选保存地图。地图将保存为 PGM + YAML (2D) 和 PCD (3D)。

**请求体**:
```json
{
  "save": true,              // 可选，是否保存地图，默认 true
  "map_name": "office_final" // 可选，覆盖启动时的地图名称
}
```

**响应**:
```json
{
  "status": "IDLE",
  "save_status": "saving",   // idle | saving | success | failed
  "message": "Map saving in background"
}
```

**说明**:
- 地图保存在后台异步进行，包含 2D (PGM/YAML) 和 3D (PCD) 地图
- 2D 地图保存到 `src/tron_nav/tron_navigation/maps/`
- 3D 地图保存到 `src/tron_slam/localizer/PCD/`

---

### 3. 查询建图状态

**端点**: `GET /api/mapping/status`

**描述**: 查询当前建图状态和进程信息。

**响应**:
```json
{
  "status": "RUNNING",
  "processes": [12345, 12346, 12347],
  "uptime": 125.6,
  "save_status": "idle",
  "save_in_progress": false
}
```

---

### 4. 清空建图缓存

**端点**: `POST /api/mapping/clear_cache`

**描述**: 强制停止所有建图进程，清除缓存数据，重置系统状态。

**响应**:
```json
{
  "status": "ok",
  "message": "缓存已清空，所有建图进程已停止"
}
```

**警告**: 此操作会强制终止所有建图相关进程（pkill -9），请谨慎使用。

---

## 地图管理 API

### 5. 获取实时地图

**端点**: `GET /api/robot/map`

**描述**: 获取实时建图数据（占用栅格地图）。建图模式下可用。

**响应**:
```json
{
  "Result": 0,
  "Error": "",
  "resolution": 0.05,
  "width": 384,
  "height": 384,
  "origin_x": -10.0,
  "origin_y": -10.0,
  "origin_yaw": 0.0,
  "data": "AAAA..."  // Base64 编码的地图数据
}
```

**数据格式**:
- `data`: Base64 编码的 uint8 数组
- 每个字节表示一个栅格的占用概率
  - `0`: 空闲
  - `100`: 占用
  - `255` (-1): 未知

---

### 6. 列出所有地图

**端点**: `GET /api/maps/list`

**描述**: 列出所有已保存的地图文件。

**响应**:
```json
{
  "maps": [
    {
      "name": "office_floor1",
      "size": 147456,
      "modified": 1700000000.0
    }
  ]
}
```

---

### 7. 加载地图文件

**端点**: `GET /api/maps/{map_name}/load`

**描述**: 从磁盘加载指定的地图文件。

**路径参数**:
- `map_name`: 地图名称（不含扩展名）

**示例**: `GET /api/maps/office_floor1/load`

**响应**:
```json
{
  "width": 384,
  "height": 384,
  "resolution": 0.05,
  "origin_x": -10.0,
  "origin_y": -10.0,
  "data": "AAAA..."
}
```

---

### 8. 获取地图注册表

**端点**: `GET /api/maps/registry`

**描述**: 获取地图注册表，包含 PCD + PGM/YAML 配对信息。

**响应**:
```json
{
  "available": true,
  "default_map": "default",
  "total": 3,
  "maps": [
    {
      "name": "office",
      "description": "Office building map",
      "pcd_file": "/path/to/office.pcd",
      "yaml_file": "/path/to/office.yaml",
      "pgm_file": "/path/to/office.pgm",
      "enabled": true,
      "is_default": true,
      "valid": true,
      "files_status": {
        "pcd": true,
        "yaml": true,
        "pgm": true
      }
    }
  ]
}
```

**说明**: 地图注册表位于 `src/tron_slam/localizer/config/map_registry.yaml`

---

### 9. 获取特定地图详情

**端点**: `GET /api/maps/registry/{map_name}`

**描述**: 从注册表获取特定地图的详细信息。

**响应**:
```json
{
  "name": "office",
  "description": "Office building map",
  "pcd_file": "/path/to/office.pcd",
  "yaml_file": "/path/to/office.yaml",
  "pgm_file": "/path/to/office.pgm",
  "enabled": true,
  "valid": true,
  "files_status": {
    "pcd": true,
    "yaml": true,
    "pgm": true
  }
}
```

---

### 10. 删除地图

**端点**: `DELETE /api/maps/{map_name}`

**描述**: 删除指定的地图文件（PGM + YAML）。

**响应**:
```json
{
  "deleted": ["office.yaml", "office.pgm"]
}
```

---

## 导航 API

### 11. 启动导航系统

**端点**: `POST /api/navigation/start`

**描述**: 启动导航系统（DLIO + GICP Localizer + Nav2 + tron_commander）。

**请求体**:
```json
{
  "map_name": "office_floor1"  // 可选，默认 final_test_map
}
```

**响应**:
```json
{
  "status": "RUNNING",
  "pids": [23456, 23457, ...]
}
```

**启动的节点**:
1. Livox LiDAR 驱动
2. Livox to PointCloud2 转换
3. DLIO (里程计)
4. GICP Localizer (定位)
5. PointCloud to LaserScan (用于导航避障)
6. Nav2 导航栈 + Map Server
7. cmd_vel_bridge (WebSocket 桥接)
8. tron_commander_pid (导航控制)

---

### 12. 停止导航系统

**端点**: `POST /api/navigation/stop`

**描述**: 停止导航系统，终止所有导航相关进程。

**响应**:
```json
{
  "status": "IDLE"
}
```

---

### 13. 取消导航目标

**端点**: `POST /api/navigation/cancel`

**描述**: 取消当前导航目标，停止执行当前轨迹。

**响应**:
```json
{
  "success": true,
  "message": "Navigation cancelled"
}
```

---

### 14. 查询导航状态

**端点**: `GET /api/navigation/status`

**描述**: 获取导航系统状态。

**响应**:
```json
{
  "status": "RUNNING",
  "processes": [23456, 23457, ...],
  "alive_count": 8,
  "total_count": 8,
  "uptime": 234.5,
  "map_name": "office_floor1"
}
```

---

## 机器人控制 API

### 15. 获取机器人状态

**端点**: `GET /api/robot/status`

**描述**: 获取机器人实时状态，包括电池、定位、导航状态。

**响应**:
```json
{
  "robot_info": {
    "status": "WALK",
    "battery": 85.5,
    ...
  },
  "battery": {
    "power": 85.5,
    "charging": false
  },
  "localization": {
    "status": 0,
    "x": 1.234,
    "y": 5.678,
    "theta": 0.785,
    "reliability": 0.95
  },
  "navigation": {
    "status": "GOING",
    "blocked": false,
    "goal_id": 0
  }
}
```

**导航状态值**:
- `IDLE`: 空闲
- `GOING`: 导航中
- `PAUSED`: 暂停
- `SUCCEEDED`: 成功
- `FAILED`: 失败
- `BLOCKED`: 阻塞

---

### 16. 获取机器人位置

**端点**: `GET /api/robot/position`

**描述**: 获取机器人在地图坐标系中的实时位置。

**响应**:
```json
{
  "x": 1.234,
  "y": 5.678,
  "yaw": 0.785,
  "frame_id": "map"
}
```

**坐标系说明**:
- 建图模式: `odom` 坐标系（DLIO 提供）
- 导航模式: `map` 坐标系（GICP Localizer 提供 map→odom 变换）

---

### 17. 设置初始位姿

**端点**: `POST /api/robot/set_initial_pose`

**描述**: 设置机器人初始位姿（用于 GICP 定位器）。

**请求体**:
```json
{
  "x": 0.0,
  "y": 0.0,
  "z": 0.0,
  "yaw": 0.0
}
```

**参数说明**:
- `x`, `y`, `z`: 位置坐标（米）
- `yaw`: 偏航角（弧度）

**响应**:
```json
{
  "success": true,
  "message": "Initial pose set: (0.00, 0.00, yaw=0.0°)"
}
```

**ROS 话题**: 发布到 `/initialpose` (PoseWithCovarianceStamped)

---

### 18. 设置导航目标

**端点**: `POST /api/robot/navigation_goal`

**描述**: 设置导航目标点，机器人将自动规划路径并导航。

**请求体**:
```json
{
  "goal_x": 5.0,
  "goal_y": 3.0,
  "goal_theta": 1.57,
  "goal_id": 1,
  "xy_tolerance": 0.1,
  "yaw_tolerance": 0.05
}
```

**参数说明**:
- `goal_x`, `goal_y`: 目标位置（米）
- `goal_theta`: 目标朝向（弧度）
- `goal_id`: 目标 ID（用于跟踪）
- `xy_tolerance`: 位置容差（米）
- `yaw_tolerance`: 角度容差（弧度）

**响应**:
```json
{
  "Result": 0,
  "Error": ""
}
```

**ROS 话题**: 发布到 `/goal_pose` (PoseStamped)

---

### 19. 发送速度命令 (SDK)

**端点**: `POST /api/robot/cmd_vel`

**描述**: 通过机器人 SDK (WebSocket) 发送速度控制命令。

**请求体**:
```json
{
  "vel_x": 0.5,
  "vel_y": 0.0,
  "vel_theta": 0.0
}
```

**参数说明**:
- `vel_x`: 前进速度 (m/s)
- `vel_y`: 侧向速度 (m/s)
- `vel_theta`: 旋转速度 (rad/s)

**响应**:
```json
{
  "Result": 0,
  "Error": ""
}
```

---

### 20. 发送速度命令 (ROS)

**端点**: `POST /api/robot/control`

**描述**: 发送速度命令到 ROS `/cmd_vel_web` 话题。

**请求体**: 同上

**ROS 话题**: 发布到 `/cmd_vel_web` (Twist)

---

### 21. 机器人模式控制

**端点**: `POST /api/robot/mode/{mode}`

**描述**: 切换机器人运动模式。

**可用模式**:
| 路径 | 说明 |
|------|------|
| `/api/robot/mode/stand` | 站立模式 |
| `/api/robot/mode/walk` | 行走模式 |
| `/api/robot/mode/sit` | 坐下模式 |
| `/api/robot/mode/recover` | 恢复模式（从跌倒站起） |

**响应**:
```json
{
  "status": "ok"
}
```

**WebSocket 消息**: 发送 `request_stand_mode` / `request_walk_mode` / `request_sitdown` / `request_recover` 到机器人 SDK

---

### 22. 楼梯模式

**端点**: `POST /api/robot/mode/stair`

**描述**: 开启/关闭楼梯模式。

**请求体**:
```json
{
  "enable": true  // true=开启, false=关闭
}
```

**响应**:
```json
{
  "status": "ok"
}
```

**说明**: 楼梯模式下速度上限提高一倍。

---

### 23. 紧急停止

**端点**: `POST /api/robot/emergency_stop`

**描述**: 紧急停止机器人所有运动。

**响应**:
```json
{
  "status": "ok"
}
```

---

### 24. 调整机身高度

**端点**: `POST /api/robot/body_height`

**描述**: 调整机器人身体高度（每次 ±5cm）。

**请求体**:
```json
{
  "direction": 1  // 1=升高, -1=降低
}
```

**响应**:
```json
{
  "status": "ok",
  "direction": 1,
  "change_cm": 5
}
```

---

### 25. 暂停/恢复导航

**端点**:
- `POST /api/robot/pause_navigation` - 暂停导航
- `POST /api/robot/resume_navigation` - 恢复导航

**响应**:
```json
{
  "Result": 0,
  "Error": ""
}
```

**ROS 话题**: 发布到 `/pause_navigation` 或 `/resume_navigation` (String)

---

### 26. 获取导航状态 (详细)

**端点**: `GET /api/robot/navigation_status`

**描述**: 获取详细的导航状态信息。

**响应**:
```json
{
  "Result": 0,
  "Error": "",
  "status": "GOING",
  "goal_id": 1,
  "distance_remaining": 2.5,
  "error_code": 0
}
```

---

### 27. 获取2D扫描点云

**端点**: `GET /api/robot/scan_points`

**描述**: 获取 2D 激光扫描点云（在地图坐标系）用于可视化。

**响应**:
```json
{
  "points": [
    [1.2, 3.4],
    [1.3, 3.5],
    ...
  ]
}
```

**说明**: 从 `/scan` 话题获取 LaserScan 数据，转换到地图坐标系。

---

### 28. 获取匹配点云可视化

**端点**: `GET /api/robot/matching_clouds`

**描述**: 获取点云匹配过程可视化数据（TEASER++、粗匹配、精匹配）。

**响应**:
```json
{
  "Result": 0,
  "Error": "",
  "teaser_source": {
    "count": 1234,
    "points": [{"x": 0.1, "y": 0.2, "z": 0.3}, ...]
  },
  "teaser_target": {
    "count": 5678,
    "points": [...]
  },
  "teaser_correspondences": [
    {
      "src": {"x": 0.1, "y": 0.2, "z": 0.3},
      "tgt": {"x": 0.15, "y": 0.25, "z": 0.35},
      "color": {"r": 1.0, "g": 0.0, "b": 0.0}
    }
  ],
  "rough_source": {...},
  "rough_target": {...},
  "refine_source": {...},
  "refine_target": {...}
}
```

**ROS 话题**:
- `/localizer/teaser_source_cloud` - TEASER++ 源点云
- `/localizer/teaser_target_cloud` - TEASER++ 目标点云
- `/localizer/teaser_correspondences` - 对应关系连线
- `/localizer/rough_source_cloud` - GICP 粗匹配源点云
- `/localizer/rough_target_cloud` - GICP 粗匹配目标点云
- `/localizer/refine_source_cloud` - GICP 精匹配源点云
- `/localizer/refine_target_cloud` - GICP 精匹配目标点云

---

## 航点路线 API

### 29. 记录航点

**端点**: `POST /api/waypoints/record`

**描述**: 记录机器人当前位置为航点。

**请求体**:
```json
{
  "type": "normal"  // normal | stair_enable | stair_disable
}
```

**响应**:
```json
{
  "x": 1.234,
  "y": 5.678,
  "yaw": 0.785,
  "type": "normal",
  "timestamp": 1700000000.0
}
```

---

### 30. 开始/停止轨迹录制

**端点**:
- `POST /api/trajectory/start` - 开始录制
- `POST /api/trajectory/stop` - 停止录制
- `GET /api/trajectory/status` - 查询状态

**开始录制请求体**:
```json
{
  "interval": 1.0  // 记录间隔（秒）
}
```

**停止录制响应**:
```json
{
  "waypoints": [...],
  "count": 25
}
```

---

### 31. 路线管理

**端点**:
- `GET /api/routes` - 列出所有路线
- `POST /api/routes` - 创建路线
- `DELETE /api/routes/{route_id}` - 删除路线
- `POST /api/routes/{route_id}/execute` - 执行路线

**创建路线请求体**:
```json
{
  "name": "Floor 1 Patrol",
  "waypoints": [
    {
      "x": 1.0,
      "y": 2.0,
      "yaw": 0.0,
      "type": "normal"
    },
    {
      "x": 3.0,
      "y": 4.0,
      "yaw": 1.57,
      "type": "stair_enable"
    }
  ]
}
```

**列出路线响应**:
```json
{
  "routes": [
    {
      "id": "route_001",
      "name": "Floor 1 Patrol",
      "waypoints": [...]
    }
  ]
}
```

**数据存储**: `web_server/data/routes.json`

---

## 传感器状态 API

### 32. 获取传感器状态

**端点**: `GET /api/sensors/status`

**描述**: 获取传感器健康状态，包括 Livox 激光雷达和建图系统。

**响应**:
```json
{
  "livox_active": true,
  "livox_count": 125,
  "map_active": true,
  "map_size": {
    "width": 384,
    "height": 384
  }
}
```

---

### 33. 获取DLIO状态

**端点**: `GET /api/dlio/status`

**描述**: 获取 DLIO (Direct LiDAR-Inertial Odometry) 系统状态。

**响应**:
```json
{
  "ready": true,
  "subscribed": true,
  "message": "DLIO is ready and subscribed"
}
```

**健康检查条件**:
1. `/dlio/odom_node/odom` 话题存在且发布数据
2. `/livox/lidar/pointcloud` 话题存在且发布数据
3. DLIO 节点 (`/dlio_odom`, `/dlio_map`) 正在运行

---

### 34. 获取代价地图

**端点**: `GET /api/costmap/{kind}`

**描述**: 获取代价地图数据（global 或 local）。

**路径参数**:
- `kind`: `global` 或 `local`

**响应**:
```json
{
  "width": 200,
  "height": 200,
  "resolution": 0.05,
  "origin_x": -5.0,
  "origin_y": -5.0,
  "data": "AAAA..."  // Base64 编码
}
```

**ROS 话题**:
- `/global_costmap/costmap` (OccupancyGrid)
- `/local_costmap/costmap` (OccupancyGrid)

---

### 35. 获取定位器日志

**端点**: `GET /api/localizer/logs`

**描述**: 获取定位器最近的日志信息。

**响应**:
```json
{
  "logs": [
    "粗匹配分数: 0.85",
    "精匹配分数: 0.92",
    "ICP匹配成功"
  ]
}
```

**日志来源**: `/tmp/navigation_3_localizer.log` (GICP Localizer 输出)

---

## 巡检回调 API

### 36. 导航结果回调

**端点**: `POST /api/inspection/callback`

**描述**: 导航任务完成后的回调通知接口。用于巡检系统接收导航结果。

**请求体**:
```json
{
  "robot_id": 1,
  "task_id": 12345,
  "execution_status": "SUCCEEDED",  // SUCCEEDED | FAILED | success
  "execution_time": 1700000000000   // 13位时间戳（毫秒）
}
```

**参数说明**:
- `robot_id`: 机器人 ID
- `task_id`: 任务 ID
- `execution_status`: 执行状态
- `execution_time`: 13 位 UNIX 时间戳（毫秒）

**响应**:
```json
{
  "code": 200,
  "msg": "success"
}
```

**错误响应**:
```json
{
  "code": 400,
  "msg": "Missing required field: task_id"
}
```

---

### 37. 获取导航状态（用于回调）

**端点**: `GET /api/inspection/nav_status`

**描述**: 获取导航状态，判断是否可以触发回调。

**响应**:
```json
{
  "code": 200,
  "msg": "success",
  "nav_status": "SUCCEEDED",
  "can_trigger_callback": true,
  "current_task_id": 12345,
  "timestamp": 1700000000000
}
```

---

## WebSocket 协议

### 连接信息

**URL**: `ws://10.192.1.2:5000` (机器人 SDK)

**说明**: ROS2WebLink 通过 `robot_sdk_bridge.py` 连接到机器人的 WebSocket SDK。

### 消息格式

所有 WebSocket 消息遵循以下格式：

```json
{
  "accid": "PF_TRON1A_042",
  "title": "request_walk_mode",
  "timestamp": 1672373633989,
  "guid": "uuid",
  "data": {}
}
```

### 请求指令

| Title | Data | 说明 |
|-------|------|------|
| `request_stand_mode` | `{}` | 站立模式 |
| `request_walk_mode` | `{}` | 行走模式 |
| `request_sitdown` | `{}` | 坐下模式 |
| `request_recover` | `{}` | 恢复模式 |
| `request_twist` | `{"x": float, "y": float, "z": float}` | 速度控制 |
| `request_stair_mode` | `{"enable": bool}` | 楼梯模式 |
| `request_emgy_stop` | `{}` | 紧急停止 |
| `request_base_height` | `{"direction": int}` | 调整高度 |
| `request_pause` | `{}` | 暂停 |
| `request_resume` | `{}` | 恢复 |
| `request_enable_odom` | `{}` | 启用里程计 |
| `request_enable_imu` | `{}` | 启用 IMU |

### 推送消息

| Title | Data | 说明 |
|-------|------|------|
| `notify_robot_info` | 状态、诊断、电量等 | 机器人状态推送 |
| `notify_twist` | 行走失败反馈 | 速度命令反馈 |
| `notify_stand_mode` | 模式切换结果 | 站立模式反馈 |
| `notify_walk_mode` | 模式切换结果 | 行走模式反馈 |
| `notify_imu` | IMU 数据 | IMU 推送 |
| `notify_odom` | 里程计数据 | 里程计推送 |
| `notify_nav_status` | 导航状态 | 导航状态推送 |

### 自动识别 accid

系统会自动从第一条 `notify_robot_info` 消息中提取 `accid`，无需手动配置。

---

## 前端页面说明

### 主页面

**路径**: `/static/index.html` 或 `/`

**功能**:
- 地图可视化（Canvas）
- 建图/导航控制
- 机器人模式切换
- 虚拟摇杆控制
- 航点录制和路线管理
- 实时日志显示（定位、导航、传感器）
- 3D 点云可视化

### API 文档页面

**路径**: `/static/api_docs.html`

**功能**:
- 交互式 API 文档
- 按模块分类
- 搜索功能
- cURL 示例
- 响应示例

### 前端技术栈

- **地图渲染**: HTML Canvas
- **3D 可视化**: Three.js + OrbitControls
- **样式**: CSS Grid + Flexbox
- **通信**: Fetch API (RESTful)
- **实时更新**: 轮询 (200ms - 10s 间隔)

### 前端直接操作

前端通过 JavaScript 直接调用后端 API，主要操作包括：

1. **地图交互**
   - 点击地图添加航点
   - 点击设置初始位姿
   - 点击设置导航目标
   - 拖动设置方向（支持 2 次点击）

2. **键盘控制**
   - `W/S`: 前进/后退
   - `Q/E`: 左移/右移
   - `A/D`: 左转/右转

3. **虚拟摇杆**
   - 上下: 前进/后退
   - 左右: 左转/右转
   - 自动适配楼梯模式速度

4. **日志自动刷新**
   - 定位日志: 2 秒
   - 导航日志: 3 秒
   - 传感器日志: 3 秒
   - 机器人位置: 200ms (5Hz)

### 坐标转换

前端在多处进行坐标转换：

**地图点击 → 世界坐标**:
```javascript
const mapX = origin_x + canvasX * resolution;
const mapY = origin_y + (height - canvasY) * resolution;  // Y轴翻转
```

**世界坐标 → Canvas 坐标**:
```javascript
const pixelX = (worldX - origin_x) / resolution;
const pixelY = height - (worldY - origin_y) / resolution;  // Y轴翻转
```

**说明**:
- ROS2 坐标系: 原点在左下角，Y 轴向上
- Canvas 坐标系: 原点在左上角，Y 轴向下
- 需要翻转 Y 轴

---

## 健康检查

**端点**: `GET /healthz`

**描述**: 轻量级存活探测。

**响应**:
```json
{
  "status": "ok"
}
```

---

## Swagger 文档

FastAPI 自动生成的交互式文档：

- **Swagger UI**: `http://localhost:8800/docs`
- **ReDoc**: `http://localhost:8800/redoc`

---

## 常见问题

### Q1: 地图数据为何没有显示？

**A**: 检查以下几点：
1. 建图是否已启动 (`/api/mapping/status`)
2. DLIO 是否就绪 (`/api/dlio/status`)
3. Livox 点云是否发布 (`/api/sensors/status`)
4. 等待 3-5 秒让系统初始化

### Q2: 导航无法启动？

**A**: 可能原因：
1. 建图进程未停止 → 调用 `/api/mapping/clear_cache`
2. 地图文件不存在 → 检查 `/api/maps/list`
3. PCD 地图未配置 → 检查 `map_registry.yaml`

### Q3: 机器人位置不更新？

**A**: 检查：
1. 建图模式: DLIO odom 是否发布 (`/dlio/odom_node/odom`)
2. 导航模式: GICP Localizer 是否运行，初始位姿是否设置

### Q4: 前端键盘控制无效？

**A**: 确保：
1. 焦点不在输入框中
2. 机器人 WebSocket 已连接
3. 机器人处于 WALK 模式

---

## 文件结构

```
ROS2WebLink/
├── web_server/
│   ├── mapping_nav_server.py      # 主服务器（FastAPI）
│   ├── mapping_controller.py      # 建图控制器
│   ├── navigation_controller.py   # 导航控制器
│   ├── robot_sdk_bridge.py        # 机器人 SDK WebSocket 桥接
│   ├── ros_extended_node.py       # ROS2 扩展节点
│   ├── route_manager.py           # 路线管理器
│   ├── dlio_health_checker.py     # DLIO 健康检查
│   ├── static/
│   │   ├── index.html             # 主页面
│   │   ├── api_docs.html          # API 文档
│   │   ├── js/
│   │   │   ├── app.js             # 主应用逻辑
│   │   │   └── pointcloud-viewer.js  # 3D 点云查看器
│   │   └── css/style.css          # 样式
│   └── data/
│       └── routes.json            # 路线数据
├── docs/                          # 文档目录
│   ├── mapping_nav_api.md
│   └── mapping_control.md
├── API_DOCUMENTATION.md           # 本文档
└── README.md
```

---

## 版本历史

### v0.1.0 (2024-11-18)

**新增功能**:
- ✅ DLIO 建图系统
- ✅ GICP 3D 定位
- ✅ Nav2 导航
- ✅ WebSocket 机器人控制
- ✅ 航点路线管理
- ✅ 3D 点云可视化
- ✅ 完整 API 文档

**已知问题**:
- 地图保存偶尔超时（已设置 180s 超时）
- DLIO 启动需要 3-5 秒预热

---

## 技术支持

**项目位置**: `/home/guest/ROS2WebLink`

**日志位置**:
- 建图: `/tmp/mapping_*.log`
- 导航: `/tmp/navigation_*.log`
- 定位: `/tmp/navigation_3_localizer.log`

**配置文件**:
- FastDDS: `/home/guest/.config/fastdds/fastdds.xml`
- Localizer: `src/tron_slam/localizer/config/localizer.yaml`
- Map Registry: `src/tron_slam/localizer/config/map_registry.yaml`

---

**文档生成时间**: 2024-11-18  
**文档版本**: 1.0.0  
**作者**: ROS2WebLink Team


