## Mapping & Navigation Web API 规范

本文档描述建图与导航 Web 控制系统的 HTTP 接口与 WebSocket 消息格式。

### 1. HTTP 接口概览

| 方法 | 路径 | 功能 | 说明 |
| ---- | ---- | ---- | ---- |
| POST | `/api/mapping/start` | 启动建图流程 | 触发 Livox 驱动、FAST-LIO2、点云转换、Cartographer launch |
| POST | `/api/mapping/stop` | 停止建图并保存 | 调用 `map_save.sh`，返回地图文件路径 |
| GET | `/api/mapping/status` | 查询建图状态 | 返回建图模式是否运行、各进程状态 |
| POST | `/api/routes` | 创建/更新巡逻轨迹 | 保存航点列表（JSON） |
| GET  | `/api/routes` | 获取所有轨迹 | 返回轨迹元数据与航点 |
| DELETE | `/api/routes/{route_id}` | 删除轨迹 | 软删除或物理删除 |
| POST | `/api/routes/{route_id}/execute` | 执行轨迹 | 逐个发送航点并监听导航状态 |
| POST | `/api/navigation/pause` | 暂停导航 | 转发到 `/pause_navigation` |
| POST | `/api/navigation/resume` | 恢复导航 | 转发到 `/resume_navigation` |
| POST | `/api/navigation/cancel` | 取消当前目标 | 终止当前巡逻队列 |
| POST | `/api/robot/mode/stand` | 站立模式 | WebSocket `request_stand_mode` |
| POST | `/api/robot/mode/walk` | 行走模式 | WebSocket `request_walk_mode` |
| POST | `/api/robot/mode/sit` | 蹲下模式 | WebSocket `request_sitdown` |
| POST | `/api/robot/mode/stair` | 楼梯模式开关 | WebSocket `request_stair_mode` |
| POST | `/api/robot/twist` | 速度控制 | WebSocket `request_twist`（30Hz） |
| POST | `/api/robot/emergency_stop` | 紧急停止 | WebSocket `request_emgy_stop` |
| GET  | `/api/robot/status` | 机器人状态 | 结合 ROS 与 SDK 推送数据 |
| GET  | `/api/map/current` | 当前地图 | `/map` 话题 Base64 PGM |
| GET  | `/api/costmap/global` | 全局 costmap | `/global_costmap/costmap` Base64 |
| GET  | `/api/costmap/local` | 局部 costmap | `/local_costmap/costmap` Base64 |

### 2. WebSocket 消息格式

所有消息遵循机器人 SDK 协议：

```json
{
  "accid": "PF_TRON1A_042",
  "title": "request_walk_mode",
  "timestamp": 1672373633989,
  "guid": "uuid",
  "data": { }
}
```

#### 常用指令

- `request_stand_mode`
- `request_walk_mode`
- `request_twist` (`data: {"x": -1~1, "y": -1~1, "z": -1~1}`)
- `request_stair_mode` (`data: {"enable": true/false}`)
- `request_emgy_stop`
- `request_enable_odom` / `request_enable_imu`

#### 推送消息

- `notify_robot_info`：状态、诊断、电量
- `notify_twist`：行走失败反馈
- `notify_stand_mode` / `notify_walk_mode`：模式切换结果

### 3. 数据存储

- `data/routes.json`：保存轨迹列表
- `data/mode_history.json`：记录模式切换历史
- `data/patrol_logs.json`：巡逻执行日志

### 4. 错误码约定

| Code | 含义 |
| ---- | ---- |
| `0`  | 成功 |
| `1`  | 请求参数错误 |
| `2`  | 依赖资源不可用（ROS 节点/WS 未连接） |
| `3`  | 内部执行失败（异常、进程启动失败） |





