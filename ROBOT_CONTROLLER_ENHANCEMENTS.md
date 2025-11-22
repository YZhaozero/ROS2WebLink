# Robot Controller 功能增强总结

## 概述
基于机器人官方SDK文档，为 `robot_controller.html` 添加了完整的机器人控制功能。

## 新增功能

### 1. 机器人模式控制
- ✅ **站立模式** (`request_stand_mode`) - 让机器人站起
- ✅ **行走模式** (`request_walk_mode`) - 切换到行走状态
- ✅ **蹲下模式** (`request_sitdown`) - 让机器人蹲下
- ✅ **楼梯模式** (`request_stair_mode`) - 开启/关闭楼梯模式（适用于TRON1双轮足机器人）
- ✅ **倒地爬起** (`request_recover`) - 当机器人摔倒后自动爬起并恢复到行走模式

### 2. 高度调节
- ✅ **升高** (`request_base_height` with direction=1) - 每次升高机身 5cm
- ✅ **降低** (`request_base_height` with direction=-1) - 每次降低机身 5cm

### 3. 其他已有功能
- 方向控制（前进、后退、左转、右转）
- 速度调节（线速度、角速度）
- 紧急停止
- 键盘控制支持

## 技术实现

### 前端 (robot_controller.html)
```html
新增控件：
- 机器人模式按钮组（站立、行走、蹲下、倒地爬起、楼梯模式开/关）
- 高度调节按钮（升高+5cm、降低-5cm）
- 实时通知系统（成功/错误/警告提示）
```

### 后端 API (mapping_nav_server.py)
新增端点：
- `POST /api/robot/body_height` - 调节机身高度
  - 参数: `direction` (1=升高, -1=降低)
  - 返回: 成功/失败状态及变化量
  
- `POST /api/robot/mode/recover` - 倒地爬起
  - 功能: 让摔倒的机器人自动站起

### SDK Bridge (robot_sdk_bridge.py)
新增方法：
- `send_body_height(direction: int)` - 发送高度调节指令
- `send_recover()` - 发送倒地恢复指令

## SDK 协议对应关系

| 功能 | SDK指令 | 参数 | 说明 |
|------|---------|------|------|
| 站立 | `request_stand_mode` | - | 让机器人站起 |
| 行走 | `request_walk_mode` | - | 切换到行走模式 |
| 蹲下 | `request_sitdown` | - | 让机器人蹲下 |
| 楼梯模式 | `request_stair_mode` | `enable: true/false` | 开启/关闭楼梯模式 |
| 高度调节 | `request_base_height` | `direction: 1/-1` | 升高/降低 5cm |
| 倒地爬起 | `request_recover` | - | 自动站起并恢复行走 |
| 紧急停止 | `request_emgy_stop` | - | 立即停止所有动作 |
| 行走控制 | `request_twist` | `x, y, z: [-1, 1]` | 控制移动速度和方向 |

## 文件修改清单

1. **robot_controller.html** (全新增强)
   - 添加机器人模式控制区域
   - 添加高度调节控制区域
   - 实现API调用方法
   - 添加通知提示系统

2. **web_server/robot_sdk_bridge.py**
   - 新增 `send_body_height()` 方法
   - 新增 `send_recover()` 方法

3. **web_server/mapping_nav_server.py**
   - 新增 `/api/robot/body_height` 端点
   - 新增 `/api/robot/mode/recover` 端点

## 使用说明

### 启动服务
```bash
cd /home/guest/ROS2WebLink
# 确保机器人WebSocket服务运行在 ws://10.192.1.2:5000
# 启动Web服务器（默认端口8000）
python3 web_server/mapping_nav_server.py
```

### 访问控制界面
```
http://127.0.0.1:8000/robot_controller.html
```

### 功能操作
1. **模式切换**: 点击对应模式按钮（站立、行走、蹲下等）
2. **高度调节**: 点击"升高"或"降低"按钮，每次调整5cm
3. **倒地恢复**: 当机器人摔倒时，点击"倒地爬起"按钮
4. **楼梯模式**: 点击"楼梯模式"开启，"退出楼梯"关闭
5. **方向控制**: 使用方向按钮或键盘(WASD/方向键)
6. **紧急停止**: 点击红色"紧急停止"按钮

## 注意事项

1. **连接状态**: 
   - 确保机器人已开机并连接到相同网络
   - 页面顶部显示连接状态（已连接/未连接）

2. **模式限制**:
   - 高度调节仅在特定状态下可用（站立或行走模式）
   - 楼梯模式仅适用于TRON1双轮足机器人
   - 倒地爬起仅在机器人摔倒后有效

3. **速度控制**:
   - `request_twist` 指令需要以 ≥30Hz 频率发送
   - 已实现50ms循环发送（20Hz），建议根据需要调整

4. **状态反馈**:
   - 所有操作都有实时通知提示
   - 成功操作显示绿色提示
   - 错误操作显示红色提示
   - 警告信息显示黄色提示

## 机器人状态
根据SDK文档，机器人可能处于以下状态：
- `STAND` - 站立
- `WALK` - 行走
- `SIT` - 蹲下
- `DAMPING` - 阻尼模式
- `ROTATE` - 旋转
- `STAIR` - 楼梯模式
- `ERROR_FALLOVER` - 摔倒
- `RECOVER` - 摔倒恢复中
- `ERROR_RECOVER` - 摔倒恢复失败

## 未来扩展建议

1. 添加机器人状态实时显示（使用 `notify_robot_info` 推送）
2. 添加IMU数据显示（使用 `notify_imu` 推送）
3. 添加里程计数据显示（使用 `notify_odom` 推送）
4. 添加灯光效果控制（使用 `request_light_effect`）
5. 添加踏步模式控制（使用 `request_marktime_mode`，适用于TRON1双足机器人）

## 参考文档
- 机器人SDK官方文档 (Section 5.5)
- WebSocket通信协议 (ws://10.192.1.2:5000)

---

**最后更新**: 2025-11-18
**作者**: AI Assistant
**版本**: 1.0



