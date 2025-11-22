# ROS2WebLink API 速查表

> **快速访问**: http://localhost:8800/api/docs (Web 文档) | http://localhost:8800/docs (Swagger)

---

## 🗂️ API 总览

### 基础
- `GET /healthz` - 健康检查

### 🗺️ 建图 (4)
```
POST   /api/mapping/start          启动建图
POST   /api/mapping/stop           停止建图并保存
GET    /api/mapping/status         查询建图状态
POST   /api/mapping/clear_cache    清空缓存
```

### 📁 地图管理 (6)
```
GET    /api/robot/map                获取实时地图
GET    /api/maps/list                列出所有地图
GET    /api/maps/{name}/load         加载指定地图
GET    /api/maps/registry            获取地图注册表
GET    /api/maps/registry/{name}     获取指定地图详情
DELETE /api/maps/{name}              删除地图
```

### 🧭 导航 (4)
```
POST   /api/navigation/start        启动导航系统
POST   /api/navigation/stop         停止导航系统
POST   /api/navigation/cancel       取消导航目标
GET    /api/navigation/status       查询导航状态
```

### 🤖 机器人控制 (17)
```
GET    /api/robot/status                  获取机器人状态
GET    /api/robot/position                获取机器人位置
POST   /api/robot/set_initial_pose        设置初始位姿
POST   /api/robot/navigation_goal         设置导航目标
GET    /api/robot/navigation_goal         获取当前目标
GET    /api/robot/navigation_status       获取导航状态详情
POST   /api/robot/cmd_vel                 发送速度命令(SDK)
POST   /api/robot/control                 发送速度命令(ROS)
POST   /api/robot/mode/stand              站立模式
POST   /api/robot/mode/walk               行走模式
POST   /api/robot/mode/sit                坐下模式
POST   /api/robot/mode/stair              楼梯模式
POST   /api/robot/mode/recover            恢复模式
POST   /api/robot/emergency_stop          紧急停止
POST   /api/robot/body_height             调整高度
POST   /api/robot/pause_navigation        暂停导航
POST   /api/robot/resume_navigation       恢复导航
GET    /api/robot/scan_points             获取2D扫描点云
GET    /api/robot/matching_clouds         获取3D匹配点云
```

### 📍 航点路线 (7)
```
POST   /api/waypoints/record        记录航点
POST   /api/trajectory/start        开始轨迹录制
POST   /api/trajectory/stop         停止轨迹录制
GET    /api/trajectory/status       查询轨迹状态
GET    /api/routes                  列出所有路线
POST   /api/routes                  创建路线
DELETE /api/routes/{id}             删除路线
POST   /api/routes/{id}/execute     执行路线
```

### 🔍 传感器状态 (4)
```
GET    /api/sensors/status          获取传感器状态
GET    /api/dlio/status             获取DLIO状态
GET    /api/costmap/{kind}          获取代价地图(global/local)
GET    /api/localizer/logs          获取定位器日志
```

### ✅ 巡检回调 (2)
```
POST   /api/inspection/callback     导航结果回调
GET    /api/inspection/nav_status   获取导航状态(用于回调)
```

### 📄 文档页面 (2)
```
GET    /api/docs                    API 文档 (交互式)
GET    /                            主控制台
```

---

## 📊 统计

| 类别 | API 数量 |
|------|---------|
| 建图 | 4 |
| 地图管理 | 6 |
| 导航 | 4 |
| 机器人控制 | 19 |
| 航点路线 | 7 |
| 传感器状态 | 4 |
| 巡检回调 | 2 |
| 文档 | 2 |
| **总计** | **48** |

---

## 🔗 WebSocket

**连接**: `ws://10.192.1.2:5000`

**请求指令** (通过 `robot_sdk_bridge`):
- `request_stand_mode` - 站立
- `request_walk_mode` - 行走
- `request_sitdown` - 坐下
- `request_recover` - 恢复
- `request_twist` - 速度控制
- `request_stair_mode` - 楼梯模式
- `request_emgy_stop` - 紧急停止
- `request_base_height` - 高度调节
- `request_pause` / `request_resume` - 暂停/恢复

**推送消息**:
- `notify_robot_info` - 机器人状态
- `notify_twist` - 速度反馈
- `notify_imu` - IMU数据
- `notify_odom` - 里程计数据
- `notify_nav_status` - 导航状态

---

## 🎯 常用场景

### 场景1: 启动建图
```bash
# 1. 启动建图
curl -X POST http://localhost:8800/api/mapping/start \
  -H "Content-Type: application/json" \
  -d '{"map_name": "office"}'

# 2. 查询状态
curl http://localhost:8800/api/mapping/status

# 3. 停止并保存
curl -X POST http://localhost:8800/api/mapping/stop \
  -H "Content-Type: application/json" \
  -d '{"save": true, "map_name": "office"}'
```

### 场景2: 启动导航
```bash
# 1. 启动导航系统
curl -X POST http://localhost:8800/api/navigation/start \
  -H "Content-Type: application/json" \
  -d '{"map_name": "office"}'

# 2. 设置初始位姿
curl -X POST http://localhost:8800/api/robot/set_initial_pose \
  -H "Content-Type: application/json" \
  -d '{"x": 0.0, "y": 0.0, "z": 0.0, "yaw": 0.0}'

# 3. 设置导航目标
curl -X POST http://localhost:8800/api/robot/navigation_goal \
  -H "Content-Type: application/json" \
  -d '{"goal_x": 5.0, "goal_y": 3.0, "goal_theta": 1.57}'
```

### 场景3: 创建巡逻路线
```bash
# 1. 记录航点
curl -X POST http://localhost:8800/api/waypoints/record \
  -H "Content-Type: application/json" \
  -d '{"type": "normal"}'

# 2. 创建路线
curl -X POST http://localhost:8800/api/routes \
  -H "Content-Type: application/json" \
  -d '{
    "name": "Floor 1 Patrol",
    "waypoints": [
      {"x": 1.0, "y": 2.0, "yaw": 0.0, "type": "normal"},
      {"x": 3.0, "y": 4.0, "yaw": 1.57, "type": "normal"}
    ]
  }'

# 3. 执行路线
curl -X POST http://localhost:8800/api/routes/{route_id}/execute
```

### 场景4: 机器人控制
```bash
# 切换模式
curl -X POST http://localhost:8800/api/robot/mode/walk

# 发送速度命令
curl -X POST http://localhost:8800/api/robot/cmd_vel \
  -H "Content-Type: application/json" \
  -d '{"vel_x": 0.5, "vel_y": 0.0, "vel_theta": 0.0}'

# 调整高度
curl -X POST http://localhost:8800/api/robot/body_height \
  -H "Content-Type: application/json" \
  -d '{"direction": 1}'
```

---

## ⚠️ 重要提示

1. **建图/导航互斥**: 建图和导航系统不能同时运行，启动前需停止另一个
2. **初始位姿**: 导航前必须设置初始位姿（建图模式不需要）
3. **地图格式**: 
   - 2D: PGM + YAML (Nav2)
   - 3D: PCD (GICP Localizer)
4. **坐标系**:
   - 建图模式: `odom` 坐标系
   - 导航模式: `map` 坐标系
5. **异步操作**: 地图保存在后台进行，通过 `save_status` 查看进度

---

## 📚 完整文档

- **Web 文档**: http://localhost:8800/api/docs
- **Swagger**: http://localhost:8800/docs
- **Markdown**: `/home/guest/ROS2WebLink/API_DOCUMENTATION.md`

---

**版本**: 1.0.0 | **更新**: 2024-11-18


