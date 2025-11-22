# API 参考文档

## 机器人状态与位置 API

### 1. 获取地图数据
```
GET /api/robot/map
```

**描述**: 获取当前地图的 JSON 数据

**返回示例**:
```json
{
  "width": 219,
  "height": 135,
  "resolution": 0.05,
  "origin": [-5.5, -3.375, 0.0],
  "data": [0, 0, 0, ...]
}
```

**状态码**:
- `200` - 成功返回地图数据
- `503` - 地图数据不可用（需要先建图或启动导航）

---

### 2. 获取导航目标点
```
GET /api/robot/navigation_goal
```

**描述**: 获取当前导航目标点的坐标及方向角度

**返回示例**:
```json
{
  "x": 1.23,
  "y": 4.56,
  "yaw": 0.78
}
```

**字段说明**:
- `x` (float): 目标点 X 坐标（米）
- `y` (float): 目标点 Y 坐标（米）
- `yaw` (float): 目标方向角度（弧度，范围 -π 到 π）

**状态码**:
- `200` - 成功返回目标点数据
- `503` - 目标点不可用（未设置导航目标）

**数据来源**: 订阅 ROS2 `/goal_pose` 话题

---

### 3. 获取机器人当前位置
```
GET /api/robot/position
```

**描述**: 获取机器人当前位置的坐标及方向角度

**返回示例**:
```json
{
  "x": 0.12,
  "y": -0.34,
  "yaw": 1.57
}
```

**字段说明**:
- `x` (float): 机器人 X 坐标（米）
- `y` (float): 机器人 Y 坐标（米）
- `yaw` (float): 机器人朝向角度（弧度，范围 -π 到 π）

**状态码**:
- `200` - 成功返回位置数据
- `503` - 位置不可用（需要启动导航系统）

**数据来源**: 订阅 ROS2 `/tron_commander/odom` 话题

**更新频率**: 实时更新（取决于 tron_commander 发布频率）

---

## 使用示例

### JavaScript/TypeScript

```javascript
// 获取地图
async function getMap() {
  const response = await fetch('/api/robot/map');
  if (response.ok) {
    const mapData = await response.json();
    console.log(`地图大小: ${mapData.width}x${mapData.height}`);
  }
}

// 获取导航目标
async function getNavigationGoal() {
  try {
    const response = await fetch('/api/robot/navigation_goal');
    if (response.ok) {
      const goal = await response.json();
      console.log(`目标点: (${goal.x}, ${goal.y}), 角度: ${goal.yaw}`);
    }
  } catch (error) {
    console.log('未设置导航目标');
  }
}

// 获取机器人位置（轮询）
setInterval(async () => {
  try {
    const response = await fetch('/api/robot/position');
    if (response.ok) {
      const pos = await response.json();
      updateRobotMarker(pos.x, pos.y, pos.yaw);
    }
  } catch (error) {
    console.log('机器人位置不可用');
  }
}, 200);  // 5Hz 更新
```

### Python

```python
import requests

# 获取地图
response = requests.get('http://localhost:8800/api/robot/map')
if response.status_code == 200:
    map_data = response.json()
    print(f"地图大小: {map_data['width']}x{map_data['height']}")

# 获取导航目标
response = requests.get('http://localhost:8800/api/robot/navigation_goal')
if response.status_code == 200:
    goal = response.json()
    print(f"目标点: ({goal['x']}, {goal['y']}), 角度: {goal['yaw']}")
else:
    print("未设置导航目标")

# 获取机器人位置
response = requests.get('http://localhost:8800/api/robot/position')
if response.status_code == 200:
    pos = response.json()
    print(f"机器人位置: ({pos['x']}, {pos['y']}), 角度: {pos['yaw']}")
else:
    print("机器人位置不可用（需要启动导航系统）")
```

### cURL

```bash
# 获取地图
curl http://localhost:8800/api/robot/map

# 获取导航目标
curl http://localhost:8800/api/robot/navigation_goal

# 获取机器人位置
curl http://localhost:8800/api/robot/position
```

---

## 坐标系说明

所有坐标数据使用 **map 坐标系**：
- **原点**: 地图左下角（或地图配置的 origin）
- **X 轴**: 向右为正
- **Y 轴**: 向上为正
- **Yaw 角度**: 
  - 0 弧度 = 向右（东）
  - π/2 弧度 = 向上（北）
  - π 或 -π 弧度 = 向左（西）
  - -π/2 弧度 = 向下（南）

---

## 前置条件

| API | 前置条件 |
|-----|---------|
| `/api/robot/map` | 已完成建图 或 导航系统已启动 |
| `/api/robot/navigation_goal` | 已设置导航目标（通过 Web UI 或 ROS2 发布） |
| `/api/robot/position` | 导航系统已启动（tron_commander 运行中） |

---

## 相关 API

- `POST /api/navigation/start` - 启动导航系统
- `POST /api/navigation/stop` - 停止导航系统
- `GET /api/navigation/status` - 查询导航状态
- `POST /api/waypoints/record` - 录制航点
- `POST /api/routes/{route_id}/execute` - 执行路线导航

完整 API 文档请参考 `USAGE.md`

---

**更新时间**: 2025-11-04
