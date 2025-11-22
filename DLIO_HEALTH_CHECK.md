# DLIO Health Check 功能说明

## 概述

ros2weblink 现在包含了智能的 DLIO 健康检查机制，确保只在 DLIO 完全启动并准备就绪后才订阅相关的 ROS2 话题。这避免了启动时序问题和数据丢失。

## 功能特性

### 1. 自动检测 DLIO 状态

系统会自动检测以下指标：

- ✅ DLIO 关键话题是否存在
  - `/dlio/odom_node/odom` - DLIO 里程计输出
  - `/livox/lidar/pointcloud` - 激光雷达点云数据

- ✅ DLIO 节点是否在运行
  - `/dlio_odom` - DLIO 里程计节点
  - `/dlio_map` - DLIO 地图节点

- ✅ 话题是否有活跃的数据发布
  - 检查最近 3 秒内是否有数据更新

### 2. 延迟订阅机制

- **启动阶段**: ros2weblink 启动时不会立即订阅 DLIO 话题
- **监控阶段**: 后台线程每 3 秒检查一次 DLIO 状态
- **就绪阶段**: 一旦 DLIO 完全启动，自动订阅相关话题

### 3. 状态 API

新增 API 端点用于查看 DLIO 健康状态：

```bash
GET /api/dlio/status
```

响应示例：

```json
{
  "ready": true,
  "subscribed": true,
  "message": "DLIO is ready and subscribed"
}
```

或者 DLIO 未就绪时：

```json
{
  "ready": false,
  "subscribed": false,
  "message": "Waiting for DLIO to be ready..."
}
```

## 工作流程

```
┌─────────────────────────────────────────────────────────┐
│  1. ros2weblink 服务启动                                 │
│     - 初始化 ROS2 桥接                                   │
│     - 启动 DLIO 健康检查器                               │
└────────────────┬────────────────────────────────────────┘
                 │
                 ▼
┌─────────────────────────────────────────────────────────┐
│  2. 后台监控 (每 3 秒检查一次)                           │
│     ⏳ 检查 DLIO 话题是否存在                            │
│     ⏳ 检查 DLIO 节点是否运行                            │
│     ⏳ 检查话题是否有活跃数据                            │
└────────────────┬────────────────────────────────────────┘
                 │
                 ▼
          ┌──────┴──────┐
          │ DLIO 是否   │
          │   ready?    │
          └──────┬──────┘
                 │
        ┌────────┴────────┐
        │                 │
       NO                YES
        │                 │
        │                 ▼
        │    ┌────────────────────────────┐
        │    │ 3. DLIO 就绪                │
        │    │    ✅ 触发就绪回调          │
        │    │    ✅ 订阅 DLIO 话题        │
        │    │    ✅ 停止健康检查          │
        │    └────────────────────────────┘
        │
        └──────► 继续监控...
```

## 使用方式

### 作为服务运行

DLIO 健康检查已集成到 ros2weblink 服务中，无需额外配置：

```bash
# 启动 ros2weblink (systemd service)
sudo systemctl start ros2weblink

# 查看日志
sudo journalctl -u ros2weblink -f
```

日志输出示例：

```
🔍 DLIO health monitor started
⏳ Waiting for DLIO to be ready...
⏳ DLIO not ready yet, continuing to monitor...
✅ DLIO health check passed: odom_count=15, pointcloud_count=127
🎉 DLIO is ready! Subscribing to DLIO topics...
🔗 Subscribing to DLIO topics...
✅ Subscribed to /dlio/odom_node/odom
✅ DLIO topic subscriptions complete
```

### 独立测试

使用测试脚本验证健康检查功能：

```bash
cd /home/guest/ROS2WebLink
source /opt/ros/humble/setup.bash
source /home/guest/tron_ros2/install/setup.bash
python3 test_dlio_health.py
```

## 配置参数

可以在 `mapping_nav_server.py` 中调整监控参数：

```python
# 检查间隔（秒）
_dlio_health_checker.start_monitoring(check_interval=3.0)
```

在 `dlio_health_checker.py` 中调整健康检查阈值：

```python
# 数据活跃判断阈值（秒）
odom_active = (current_time - self._last_odom_time) < 3.0
pointcloud_active = (current_time - self._last_pointcloud_time) < 3.0
```

## 架构设计

### 核心组件

1. **DLIOHealthChecker** (`dlio_health_checker.py`)
   - 独立的健康检查模块
   - 后台监控线程
   - 回调机制通知就绪状态

2. **ExtendedRosBridge** (`ros_extended_node.py`)
   - 新增 `subscribe_to_dlio()` 方法
   - 延迟订阅 DLIO 话题
   - 防止重复订阅

3. **FastAPI Server** (`mapping_nav_server.py`)
   - 启动时初始化健康检查
   - 提供状态查询 API
   - 自动触发订阅

### 关键类和方法

```python
# DLIOHealthChecker 主要方法
class DLIOHealthChecker:
    def is_ready() -> bool                          # 检查是否就绪
    def start_monitoring(check_interval: float)     # 开始监控
    def register_ready_callback(callback: Callable) # 注册回调
    def stop_monitoring()                           # 停止监控

# ExtendedRosBridge 新增方法
class ExtendedRosBridge:
    def subscribe_to_dlio()  # 延迟订阅 DLIO 话题
```

## 优势

1. **避免启动时序问题**
   - 不会在 DLIO 未就绪时订阅话题
   - 避免丢失初始数据或订阅失败

2. **自动恢复**
   - 即使 DLIO 延迟启动也能自动连接
   - 无需手动干预

3. **状态可见**
   - 通过 API 查看 DLIO 状态
   - 日志详细记录检查过程

4. **解耦设计**
   - 健康检查独立模块，易于测试和维护
   - 不影响其他功能

## 故障排查

### DLIO 一直显示 not ready

检查以下几点：

1. **确认 DLIO 是否运行**
   ```bash
   ros2 node list | grep dlio
   # 应该看到: /dlio_odom 和 /dlio_map
   ```

2. **检查话题是否发布**
   ```bash
   ros2 topic list | grep dlio
   ros2 topic hz /dlio/odom_node/odom
   ros2 topic hz /livox/lidar/pointcloud
   ```

3. **查看健康检查日志**
   ```bash
   sudo journalctl -u ros2weblink -f | grep -i dlio
   ```

4. **检查 ROS_DOMAIN_ID**
   ```bash
   # ros2weblink 和 DLIO 必须在同一个 domain
   echo $ROS_DOMAIN_ID
   ```

### 测试脚本无响应

1. **确认 ROS2 环境已配置**
   ```bash
   source /opt/ros/humble/setup.bash
   source /home/guest/tron_ros2/install/setup.bash
   ```

2. **手动启动 DLIO 进行测试**
   ```bash
   ros2 launch tron_navigation 2d_nav_bringup_dlio_launch.py
   ```

## 未来改进

- [ ] 添加配置文件支持自定义检查参数
- [ ] 支持检查更多 DLIO 相关话题
- [ ] 提供 WebSocket 实时状态推送
- [ ] 添加健康检查超时机制

## 参考

- DLIO launch file: `/home/guest/tron_ros2/src/tron_nav/tron_navigation/launch/2d_nav_bringup_dlio_launch.py`
- DLIO package: `direct_lidar_inertial_odometry`
- Health checker implementation: `/home/guest/ROS2WebLink/web_server/dlio_health_checker.py`




