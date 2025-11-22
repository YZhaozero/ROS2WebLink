# 系统架构设计

## 1. 总体架构

```
┌───────────────┐    WebSocket    ┌─────────────────────┐
│ 前端 Web UI   │◀──────────────▶│ RobotSDKBridge      │
│  HTML/CSS/JS  │   HTTP REST    │ (ws://10.192.1.2)    │
└──────▲────────┘                └──────────▲──────────┘
       │                                      │
       │ HTTP API                             │内网协议
       │                                      │
┌──────┴────────┐                ┌───────────┴──────────┐
│ FastAPI 应用  │───ROS2订阅────│ ExtendedRosBridge   │
│ mapping_nav   │◀──────────────│ (rclpy Node)         │
└──────▲────────┘                └───────▲──────────────┘
       │ HTTP API                              │ ROS 话题
       ▼                                       ▼
┌────────────────────────┐         ┌─────────────────────────┐
│ MappingController      │         │ RouteManager             │
│ - Launch 管理          │         │ - 航点 CRUD              │
│ - 地图保存             │         │ - 巡逻执行队列           │
└────────────────────────┘         └─────────────────────────┘
```

## 2. 模块职责

- **FastAPI (mapping_nav_server.py)**
  - 暴露 HTTP 接口与静态资源
  - 注入 RobotSDKBridge、ExtendedRosBridge、MappingController、RouteManager 实例
  - 处理请求校验与响应格式

- **RobotSDKBridge**
  - 封装机器人 SDK WebSocket 协议
  - 自动重连、心跳检测
  - 暴露模式切换、速度控制、状态订阅方法

- **ExtendedRosBridge**
  - 订阅 `/map`、`/global_costmap/costmap`、`/local_costmap/costmap`、`/nav_status` 等话题
  - 发布 `/goal`、`/goal_id`、`/cmd_vel_web`、`/pause_navigation`、`/resume_navigation`
  - 提供最新数据的线程安全缓存

- **MappingController**
  - 通过 subprocess 启动/停止建图相关 launch
  - 监控进程状态，处理异常退出
  - 调用 `map_save.sh` 保存地图

- **RouteManager**
  - 航点 CRUD 操作，JSON 持久化
  - 执行巡逻：顺序发送目标点，等待 Nav2 反馈
  - 支持特殊航点（楼梯模式开关）

- **前端 Web UI**
  - Canvas 地图渲染：底图 + costmap + 航点
  - 控制面板：建图、导航、模式切换、WASD 遥控
  - WebSocket 状态显示：机器人姿态、电量、当前模式

## 3. 数据流

1. **建图流程**：
   - 用户点击“开始建图” → POST `/api/mapping/start`
   - MappingController 启动 ROS launch → ExtendedRosBridge 获取数据 → 前端轮询状态
   - 用户点击“停止保存” → POST `/api/mapping/stop` → 保存地图文件 → 返回路径

2. **导航巡逻**：
   - 用户在前端标注航点并保存 → RouteManager 写入 `routes.json`
   - 执行巡逻 → RouteManager 逐点调用 ExtendedRosBridge.publish_goal → 监听 `/nav_status`
   - 特殊航点触发 RobotSDKBridge 切换楼梯模式

3. **遥控与状态**：
   - 前端 W/A/S/D → POST `/api/robot/twist` → RobotSDKBridge `request_twist`
   - RobotSDKBridge 接收 `notify_robot_info` → 推送给前端 → UI 更新状态

## 4. 部署结构

- 后端服务运行在机器人侧 Jetson 主机
- 端口规划：
  - FastAPI：`0.0.0.0:8800`
  - 前端静态资源：同端口 `/`
  - WebSocket：Robot SDK 固定 `10.192.1.2:5000`

## 5. 线程模型

- RobotSDKBridge：后台线程运行 WebSocket 客户端
- ExtendedRosBridge：`rclpy` spin 在线程池（daemon 线程）
- MappingController：每个 launch 使用单独 subprocess，主线程监控
- FastAPI：Uvicorn ASGI 事件循环

## 6. 失败恢复策略

- WebSocket 断开：指数退避重连（1s, 2s, 4s, ... 上限 60s）
- 建图进程崩溃：MappingController 自动检测，状态变为 `error`
- 巡逻失败：RouteManager 记录失败原因，自动停止队列并通知前端





