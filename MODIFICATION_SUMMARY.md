# API接口修改总结

**修改日期**: 2025-11-10  
**目的**: 使 mapping_nav_server.py 完全符合 API 文档规范

---

## ✅ 已完成的修改

### 1. 后端 - ros_extended_node.py

#### 修改点：
- **新增字段**: 添加 `_current_goal_id` 用于跟踪目标点ID
- **增强方法**: `get_navigation_status()` 现在返回完整格式：
  ```python
  {
    "status": "IDLE/GOING/PAUSED/SUCCEEDED/FAILED",
    "blocked": False,
    "goal_id": 0
  }
  ```
- **更新方法**: `publish_goal()` 现在会存储并发布 goal_id

---

### 2. 后端 - mapping_nav_server.py

#### 2.1 接口路径修改

| 原接口路径 | 新接口路径 | 状态 |
|-----------|-----------|------|
| `POST /api/navigation/set_goal` | `POST /api/robot/navigation_goal` | ✅ 已修改 |
| `POST /api/robot/twist` | `POST /api/robot/cmd_vel` | ✅ 已修改 |
| - | `GET /api/robot/navigation_status` | ✅ 新增 |

#### 2.2 请求参数修改

**POST /api/robot/navigation_goal**
- 原参数: `{x, y, yaw}`
- 新参数: `{goal_x, goal_y, goal_theta, goal_id, xy_tolerance, yaw_tolerance}`

#### 2.3 返回格式修改

**所有修改的接口现在返回 API 文档格式**：
```json
{
  "Result": 0,  // 0=成功, 1-4=各种错误码
  "Error": ""   // 错误信息
}
```

**GET /api/robot/status** 现在返回完整格式：
```json
{
  "battery": {
    "power": 0.87,
    "charging": false
  },
  "localization": {
    "x": 1.23,
    "y": 4.56,
    "theta": 0.78,
    "reliability": 0.95
  },
  "navigation": {
    "status": "GOING",
    "blocked": false,
    "goal_id": 0
  }
}
```

---

### 3. 前端 - app.js

#### 修改点：

**setNavGoalAtPosition() 函数** (第1102行)
- 旧 API: `POST /api/navigation/set_goal`
- 新 API: `POST /api/robot/navigation_goal`
- 参数变化: `{x, y, yaw}` → `{goal_x, goal_y, goal_theta}`

**pushTwistCommand() 函数** (第584行)
- 旧 API: `POST /api/robot/twist`
- 新 API: `POST /api/robot/cmd_vel`
- 参数保持不变: `{vel_x, vel_y, vel_theta}`

---

## 📋 API 接口对比表

### 符合 API 文档的接口

| API 文档接口 | 实现状态 | 参数符合 | 返回格式符合 |
|-------------|---------|---------|------------|
| `GET /api/robot/map` | ✅ | ✅ | ✅ |
| `POST /api/robot/navigation_goal` | ✅ | ✅ | ✅ |
| `POST /api/robot/cmd_vel` | ✅ | ✅ | ✅ |
| `GET /api/robot/status` | ✅ | N/A | ✅ |
| `GET /api/robot/navigation_status` | ✅ | N/A | ✅ |

---

## 🔄 向后兼容性

**已删除的接口**（需要更新任何外部调用）：
- ❌ `POST /api/navigation/set_goal` → 改用 `POST /api/robot/navigation_goal`
- ❌ `POST /api/robot/twist` → 改用 `POST /api/robot/cmd_vel`

---

## 🧪 测试建议

### 1. 测试导航目标设置
```bash
curl -X POST http://localhost:8800/api/robot/navigation_goal \
  -H "Content-Type: application/json" \
  -d '{
    "goal_x": 1.0,
    "goal_y": 2.0,
    "goal_theta": 0.5,
    "goal_id": 123
  }'
```

预期返回：
```json
{
  "Result": 0,
  "Error": ""
}
```

### 2. 测试速度控制
```bash
curl -X POST http://localhost:8800/api/robot/cmd_vel \
  -H "Content-Type: application/json" \
  -d '{
    "vel_x": 0.2,
    "vel_y": 0.0,
    "vel_theta": 0.1
  }'
```

### 3. 测试状态获取
```bash
curl http://localhost:8800/api/robot/status
```

预期返回包含 `battery`, `localization`, `navigation` 三个字段。

### 4. 测试导航状态
```bash
curl http://localhost:8800/api/robot/navigation_status
```

预期返回：
```json
{
  "Result": 0,
  "Error": "",
  "status": "IDLE",
  "blocked": false,
  "goal_id": 0
}
```

---

## 📝 注意事项

1. **前端缓存**: 浏览器可能缓存旧的 app.js，建议强制刷新（Ctrl+Shift+R）
2. **服务重启**: 修改后需要重启 web 服务：
   ```bash
   cd /home/guest/ROS2WebLink
   ./start_web_ui.sh
   ```
3. **兼容性**: 如果有其他系统调用旧接口，需要同步更新

---

## ✨ 改进点

1. **规范化**: 所有接口现在完全符合 API 文档规范
2. **一致性**: 返回格式统一使用 `{Result, Error}` 结构
3. **完整性**: 状态接口现在包含完整的 localization 信息
4. **可追踪**: 支持 goal_id 跟踪导航任务

---

## 🔍 问题排查

如果遇到问题：

1. **检查服务是否运行**
   ```bash
   lsof -i :8800
   ```

2. **查看服务日志**
   ```bash
   tail -f /tmp/webui.log
   ```

3. **验证 ROS2 连接**
   ```bash
   ros2 topic list | grep -E "(goal|cmd_vel|nav_status)"
   ```

---

**修改完成！** 🎉

