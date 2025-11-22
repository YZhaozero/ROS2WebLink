# Tron Robot Web UI - 使用说明

## 快速启动

### 1. 启动 Web 界面

```bash
cd /home/guest/ROS2WebLink
./start_web_ui.sh
```

启动后访问: `http://<机器人IP>:8800`

### 2. 操作流程

#### A. 建图模式（首次使用）

1. **启动建图**
   - 在 Web 界面点击 `开始建图`
   - 系统自动启动：Livox 雷达 → DLIO → Octomap → 2D 地图生成
   - 等待传感器状态显示 `雷达: ✓ 正常`

2. **控制机器人移动**
   - 点击 `站立` 让机器人站起来
   - 点击 `行走` 进入行走模式
   - 使用 **WASD 键盘控制**移动机器人：
     - `W` - 前进
     - `S` - 后退
     - `A` - 左转
     - `D` - 右转
   - 遍历需要建图的区域

3. **保存地图**
   - 输入地图名称（如 `office_floor1`）
   - 点击 `停止建图`
   - 系统自动保存地图到 `tron_navigation/maps/` 目录

#### B. 导航模式（使用已有地图）

1. **启动导航系统**
   - 输入地图名称（如 `office_floor1`）
   - 点击 `启动导航`
   - 系统自动启动：
     - Livox 雷达
     - DLIO 定位
     - pointcloud_to_laserscan（障碍物检测）
     - Nav2 导航栈
     - tron_commander（机器人位置发布）
   - 等待导航状态显示 `导航状态：RUNNING | 进程: 6/6`

2. **录制航点**

   **方式 1：手动录制单个航点**
   - 使用 WASD 控制机器人移动到目标位置
   - 点击 `录制当前位置`
   - 重复以上步骤添加多个航点
   - 点击 `保存航点`，输入路线名称

   **方式 2：自动录制轨迹**
   - 点击 `开始录制轨迹`
   - 使用 WASD 控制机器人沿期望路径移动
   - 点击 `停止录制轨迹`
   - 系统自动记录完整路径（默认每秒 1 个航点）
   - 点击 `保存航点`，输入路线名称

3. **执行导航**
   - 在路线列表中找到已保存的路线
   - 点击 `执行` 按钮
   - 机器人将自动按航点顺序导航
   - 如果路线包含楼梯航点，系统自动切换楼梯模式

## 功能说明

### 系统状态显示

- **后端状态**: 显示 Web 服务器连接状态
- **建图状态**: 显示建图系统运行状态（IDLE / RUNNING / ERROR）
- **导航状态**: 显示导航系统运行状态和进程数量
- **传感器状态**: 显示雷达和地图数据流状态
- **机器人状态**: 显示机器人当前模式（IDLE / STAND / WALK / SIT）

### 机器人控制

- **站立**: 让机器人站起来
- **行走**: 进入行走模式（可移动）
- **蹲下**: 让机器人蹲下
- **楼梯模式**: 启用楼梯行走模式
- **退出楼梯**: 退出楼梯模式
- **紧急停止**: 立即停止所有运动

### 地图显示

- **刷新地图**: 重新加载 2D 地图
- **刷新代价地图**: 更新障碍物检测层
- **代价地图类型**:
  - `global_costmap/costmap` - 全局静态地图
  - `local_costmap/costmap` - 局部动态障碍物

### 航点管理

- **录制当前位置**: 记录机器人当前位置为航点
- **开始录制轨迹**: 持续记录机器人移动路径
- **停止录制轨迹**: 停止录制并保存轨迹
- **清空航点**: 清除当前未保存的航点
- **保存航点**: 将航点列表保存为路线文件

## 技术架构

### 建图数据流

```
Livox Mid-360 雷达
    ↓
livox_to_pointcloud2 (转换为标准 PointCloud2)
    ↓
DLIO (3D LiDAR-Inertial Odometry)
    ↓
Octomap Server (3D → 2D 投影)
    ↓
/map (OccupancyGrid 2D 地图)
```

### 导航数据流

```
Livox Mid-360 雷达
    ↓
livox_to_pointcloud2
    ↓
DLIO (定位)
    ├─→ pointcloud_to_laserscan → Nav2 (障碍物避障)
    └─→ tron_commander → /tron_commander/odom (机器人位置)
                              ↓
                          Web UI (位置显示)
```

### 目录结构

```
/home/guest/
├── ROS2WebLink/              # Web UI 项目
│   ├── web_server/
│   │   ├── mapping_nav_server.py      # FastAPI 主服务
│   │   ├── mapping_controller.py      # 建图控制器
│   │   ├── navigation_controller.py   # 导航控制器（新增）
│   │   ├── ros_extended_node.py       # ROS2 桥接
│   │   ├── robot_sdk_bridge.py        # 机器人 SDK 桥接
│   │   └── static/
│   │       ├── index.html             # 前端页面
│   │       └── js/app.js              # 前端逻辑
│   ├── start_web_ui.sh       # 启动脚本
│   └── routes/               # 保存的路线文件
│
└── tron_ros2/                # ROS2 工作空间
    └── src/tron_nav/
        └── tron_navigation/
            └── maps/         # 保存的地图文件
```

## API 端点

### 系统状态

- `GET /healthz` - 健康检查
- `GET /api/robot/status` - 机器人状态
- `GET /api/sensors/status` - 传感器状态

### 建图控制

- `POST /api/mapping/start` - 启动建图
- `POST /api/mapping/stop` - 停止建图并保存
- `GET /api/mapping/status` - 建图状态

### 导航控制（新增）

- `POST /api/navigation/start` - 启动导航系统
  ```json
  {"map_name": "office_floor1"}
  ```
- `POST /api/navigation/stop` - 停止导航系统
- `GET /api/navigation/status` - 导航状态

### 机器人控制

- `POST /api/robot/mode/stand` - 站立
- `POST /api/robot/mode/walk` - 行走
- `POST /api/robot/mode/sit` - 蹲下
- `POST /api/robot/mode/stair` - 楼梯模式
  ```json
  {"enable": true}
  ```
- `POST /api/robot/emergency_stop` - 紧急停止

### 位置与航点（新增）

- `GET /api/robot/position` - 获取机器人位置
  ```json
  {"x": 1.23, "y": 4.56, "yaw": 0.78}
  ```
- `POST /api/waypoints/record` - 录制当前位置
  ```json
  {"type": "normal"}
  ```
- `POST /api/trajectory/start` - 开始录制轨迹
  ```json
  {"interval": 1.0}
  ```
- `POST /api/trajectory/stop` - 停止录制轨迹
- `GET /api/trajectory/status` - 轨迹录制状态

### 路线管理

- `GET /api/routes` - 列出所有路线
- `POST /api/routes` - 创建新路线
- `POST /api/routes/{name}/execute` - 执行路线
- `DELETE /api/routes/{name}` - 删除路线

### 地图数据

- `GET /api/map` - 获取 2D 地图（PNG）
- `GET /api/costmap/{kind}` - 获取代价地图（PNG）

## 常见问题

### Q: 建图时传感器显示 "无数据"？

**A**: 检查以下项目：
1. 确认 Livox 雷达已连接并通电
2. 查看建图状态是否为 `RUNNING`
3. 等待 3-5 秒让系统完全启动
4. 检查 `/tmp/mapping_*.log` 日志文件

### Q: 导航系统启动失败？

**A**: 确保：
1. 已经完成建图并保存地图文件
2. 地图名称输入正确（不含 `.yaml` 后缀）
3. 地图文件存在于 `tron_navigation/maps/` 目录
4. 建图系统已停止（不能同时运行建图和导航）

### Q: 机器人位置不显示？

**A**: 需要满足：
1. 导航系统已启动（`导航状态：RUNNING`）
2. tron_commander 进程正在运行
3. DLIO 定位系统正常工作
4. 等待几秒让系统初始化

### Q: WASD 键盘控制不响应？

**A**: 确保：
1. 机器人处于 `WALK` 模式
2. 浏览器窗口已聚焦（点击地图区域）
3. 没有输入框处于激活状态

### Q: 航点录制失败？

**A**: 检查：
1. 导航系统是否正在运行
2. 机器人位置是否可用（`/api/robot/position` 返回数据）
3. tron_commander 是否正常发布 `/tron_commander/odom`

## 注意事项

1. **建图和导航不能同时运行** - 它们使用不同的 ROS2 节点配置
2. **首次使用必须先建图** - 导航需要已保存的地图文件
3. **地图名称一致性** - 建图保存和导航启动时使用相同的地图名称
4. **进程清理** - 如果系统异常，使用 `pkill -9 -f ros2` 清理所有 ROS2 进程
5. **日志位置** - 所有日志保存在 `/tmp/` 目录，便于调试

## 更新日志

### 2025-11-04 - 全自动导航系统

**新增功能**:
- ✅ 导航系统一键启动/停止
- ✅ 自动管理所有 ROS2 节点（DLIO + Nav2 + tron_commander）
- ✅ 实时显示机器人位置（地图上红色三角形）
- ✅ 航点录制功能（单点 + 轨迹）
- ✅ 导航状态监控

**改进**:
- 用户无需手动启动任何 ROS2 节点
- 所有进程由 Web 服务器自动管理
- 地图文件自动复制到导航目录

---

**技术支持**: 如有问题请查看 `/tmp/webui.log` 和 `/tmp/navigation_*.log`

