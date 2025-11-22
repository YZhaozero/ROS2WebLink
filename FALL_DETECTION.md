# 机器人摔倒检测指南

## 📋 摔倒状态说明

根据机器人SDK官方文档，`notify_robot_info` 消息中的 `status` 字段包含以下与摔倒相关的状态：

| 状态值 | 说明 | 应对措施 |
|--------|------|----------|
| `ERROR_FALLOVER` | ⚠️ **机器人摔倒了** | 立即调用恢复功能 |
| `RECOVER` | 🔄 **摔倒恢复中** | 等待恢复完成 |
| `ERROR_RECOVER` | ❌ **摔倒恢复失败** | 需要人工干预 |

## 🔍 检测方法

### 方法1: Web界面自动检测（推荐）

访问 http://localhost:8800/，系统会自动检测并显示摔倒状态：

- **正常状态**: 显示 `模式：WALK ｜ 电量：85`
- **摔倒状态**: 显示 `⚠️ 机器人摔倒了！ ｜ 电量：85`（红色背景）
- **恢复中**: 显示 `🔄 恢复中... ｜ 电量：85`（黄色背景）
- **恢复失败**: 显示 `❌ 恢复失败 ｜ 电量：85`（红色背景）

### 方法2: API查询

```bash
# 获取机器人完整状态
curl -s http://localhost:8800/api/robot/status | jq '.'

# 仅查看status字段
curl -s http://localhost:8800/api/robot/status | jq '.robot_info.status'

# 示例返回：
# "WALK"           - 正常行走
# "ERROR_FALLOVER" - 摔倒了！
# "RECOVER"        - 恢复中
```

### 方法3: 实时监控脚本

创建一个监控脚本：

```bash
#!/bin/bash
# fall_monitor.sh - 实时监控机器人摔倒状态

while true; do
    STATUS=$(curl -s http://localhost:8800/api/robot/status | jq -r '.robot_info.status')
    
    case "$STATUS" in
        "ERROR_FALLOVER")
            echo "⚠️  [$(date)] 警告：机器人摔倒了！"
            # 可以添加告警通知（邮件、短信等）
            ;;
        "RECOVER")
            echo "🔄 [$(date)] 机器人正在恢复中..."
            ;;
        "ERROR_RECOVER")
            echo "❌ [$(date)] 错误：恢复失败，需要人工干预！"
            ;;
        "WALK"|"STAND"|"SIT")
            echo "✅ [$(date)] 机器人状态正常: $STATUS"
            ;;
    esac
    
    sleep 2
done
```

## 🛠️ 恢复操作

### 自动恢复

当检测到 `ERROR_FALLOVER` 状态时，点击Web界面的 **"倒地爬起"** 按钮，或调用API：

```bash
# 通过API触发恢复
curl -X POST http://localhost:8800/api/robot/mode/recover
```

系统会发送 `request_recover` 指令到机器人，机器人将自动执行站起动作。

### 状态转换流程

```
正常运行 (WALK/STAND)
    ↓
摔倒发生 (ERROR_FALLOVER)  ← 这里会自动检测到
    ↓
调用恢复 (request_recover)
    ↓
恢复中 (RECOVER)
    ↓
    ├── 成功 → 返回 STAND 或 WALK
    └── 失败 → ERROR_RECOVER（需人工干预）
```

## 💻 代码示例

### Python 监控示例

```python
import requests
import time

def check_robot_fall():
    """检查机器人是否摔倒"""
    try:
        response = requests.get('http://localhost:8800/api/robot/status')
        data = response.json()
        status = data.get('robot_info', {}).get('status', 'UNKNOWN')
        
        if status == 'ERROR_FALLOVER':
            print("⚠️  机器人摔倒了！正在触发恢复...")
            # 自动触发恢复
            requests.post('http://localhost:8800/api/robot/mode/recover')
            return True
        elif status == 'RECOVER':
            print("🔄 机器人正在恢复中...")
            return False
        elif status == 'ERROR_RECOVER':
            print("❌ 恢复失败，请人工检查！")
            return False
        else:
            print(f"✅ 机器人状态正常: {status}")
            return False
            
    except Exception as e:
        print(f"❌ 检查失败: {e}")
        return False

# 持续监控
while True:
    check_robot_fall()
    time.sleep(2)
```

### JavaScript 前端示例

```javascript
// 在Web页面中实时监控
async function monitorFallStatus() {
    try {
        const response = await fetch('/api/robot/status');
        const data = await response.json();
        const status = data.robot_info?.status || 'UNKNOWN';
        
        if (status === 'ERROR_FALLOVER') {
            // 显示警告弹窗
            if (confirm('⚠️ 机器人摔倒了！是否立即触发恢复？')) {
                await fetch('/api/robot/mode/recover', {method: 'POST'});
            }
        }
    } catch (error) {
        console.error('监控失败:', error);
    }
}

// 每2秒检查一次
setInterval(monitorFallStatus, 2000);
```

## 📊 数据结构

### notify_robot_info 完整示例

```json
{
  "accid": "PF_TRON1A_042",
  "title": "notify_robot_info",
  "timestamp": 1672373633989,
  "guid": "746d937cd8094f6a98c9577aaf213d98",
  "data": {
    "accid": "PF_TRON1A_042",
    "sw_version": "robot-tron1-2.0.10.20241111103012",
    "imu": "OK",
    "camera": "OK",
    "motor": "OK",
    "battery": 95,
    "status": "ERROR_FALLOVER"  ← 这里表示摔倒
  }
}
```

### API 返回格式

```json
{
  "robot_info": {
    "status": "ERROR_FALLOVER",
    "battery": 85,
    "imu": "OK",
    "camera": "OK",
    "motor": "OK"
  },
  "battery": {
    "power": 85.0,
    "charging": false
  },
  "localization": {...},
  "navigation": {...}
}
```

## 🎯 所有可能的状态值

| Status 值 | 中文 | 说明 |
|-----------|------|------|
| `STAND` | 站立 | 正常站立模式 |
| `WALK` | 行走 | 正常行走模式 |
| `SIT` | 坐下 | 正常坐下模式 |
| `DAMPING` | 阻尼 | 阻尼模式 |
| `ROTATE` | 旋转 | 旋转模式 |
| `STAIR` | 楼梯 | 楼梯模式 |
| `ERROR_FALLOVER` | 摔倒 | ⚠️ 机器人摔倒 |
| `RECOVER` | 恢复中 | 🔄  摔倒恢复中 |
| `ERROR_RECOVER` | 恢复失败 | ❌ 恢复失败 |

## ⚠️ 注意事项

1. **实时性**: `notify_robot_info` 每秒推送一次，检测延迟最多1秒
2. **WebSocket连接**: 需要机器人SDK WebSocket连接正常（`ws://10.192.1.2:5000`）
3. **自动恢复**: 可以配置自动调用恢复功能，但建议先人工确认
4. **多次失败**: 如果多次 `ERROR_RECOVER`，请检查机器人硬件和环境

## 🔧 故障排查

### 问题1: 检测不到摔倒状态

**原因**: 
- WebSocket未连接
- robot_info数据为空

**解决**:
```bash
# 检查WebSocket连接状态
tail -f /tmp/ros2weblink.log | grep "RobotSDKBridge"

# 检查robot_info是否有数据
curl -s http://localhost:8800/api/robot/status | jq '.robot_info'
```

### 问题2: 恢复功能无响应

**原因**:
- WebSocket连接断开
- 机器人SDK未响应

**解决**:
```bash
# 检查WebSocket日志
tail -f /tmp/ros2weblink.log | grep "request_recover"

# 手动重启服务
cd /home/guest/ROS2WebLink
python3 -m uvicorn web_server.mapping_nav_server:app --reload
```

## 📚 相关文档

- [API 完整文档](API_DOCUMENTATION.md)
- [机器人控制增强](ROBOT_CONTROLLER_ENHANCEMENTS.md)
- [WebSocket 协议](API_DOCUMENTATION.md#websocket-协议)

---

**更新时间**: 2024-11-19  
**版本**: 1.0.0

