# RobotSDKBridge 设计说明

## 1. 目标
- 提供与机器人官方 WebSocket SDK 协议完全兼容的 Python 封装
- 封装连接管理、心跳保活、断线重连
- 提供同步/异步指令发送接口与状态缓存
- 为 FastAPI 层、航点执行器提供统一能力

## 2. 功能点

| 功能 | 描述 |
| ---- | ---- |
| 连接管理 | 建立至 `ws://10.192.1.2:5000` 的长连接，支持指数退避重连 |
| 心跳机制 | 周期性发送 `request_enable_odom`/`request_enable_imu` 或 ping 包 |
| 指令封装 | 所有 `request_*` 命令（站立/行走/楼梯/灯光/紧急停等） |
| 推送解析 | 解析 `notify_*` 消息并更新本地状态 |
| 状态缓存 | 缓存 `notify_robot_info`、IMU、odom 数据，供 REST 查询 |
| 订阅回调 | 允许注册回调以处理特定推送事件 |

## 3. 类图概览

```
RobotSDKBridge
 ├── connect()
 ├── disconnect()
 ├── send_command(title, data)
 ├── set_mode(mode)
 ├── send_twist(x, y, z)
 ├── on(event, callback)
 └── properties: robot_info, imu_data, last_twist

ReconnectStrategy
 └── next_delay()
```

## 4. 线程模型
- WebSocket 使用 `websocket-client` 库在后台线程执行 `run_forever`
- 接收线程解析消息并通过事件回调通知主线程
- 发送指令支持线程安全（使用 `threading.Lock`）

## 5. 日志 & 监控
- 使用 Python `logging`，分级：INFO（连接、模式切换），DEBUG（详细 payload）
- 错误自动重试，超过阈值时向 FastAPI 抛出异常

## 6. 配置项
- `websocket_url`: 默认 `ws://10.192.1.2:5000`
- `accid`: 机器人序列号（可在 notify_robot_info 中动态获取）
- `reconnect_base`, `reconnect_max`: 重连策略参数
- `heartbeat_interval`: ms

## 7. 对外 API

```python
bridge = RobotSDKBridge()
bridge.start()
bridge.send_stand_mode()
bridge.send_walk_mode()
bridge.send_twist(x=0.5, y=0.0, z=0.1)
bridge.send_stair_mode(enable=True)
info = bridge.robot_info
```

## 8. 错误处理
- 发送指令失败抛出自定义异常 `RobotSDKCommandError`
- 推送解析失败记录 warning 并继续
- 重连次数达到上限后切换到 `disconnected` 状态并供 REST 查询

## 9. 单元测试覆盖
- 连接与重连流程（使用虚拟 WebSocket 服务器模拟）
- 指令编码正确性
- 推送解析与状态缓存
- 多次调用 send_twist 保证节流频率控制




