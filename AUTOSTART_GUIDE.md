# ROS2WebLink 自启动服务管理指南

## ✅ 服务已配置

ROS2WebLink 已配置为开机自启动服务，使用 systemd 管理。

---

## 📋 服务信息

- **服务名称**: `ros2weblink.service`
- **服务描述**: ROS2 Web Link Service - Web UI for Tron Robot Navigation
- **启动用户**: guest
- **工作目录**: `/home/guest/ROS2WebLink`
- **服务端口**: 8800
- **配置文件**: `/etc/systemd/system/ros2weblink.service`
- **启动延迟**: 10 秒（等待网络和依赖服务）

---

## 🎮 常用管理命令

### 查看服务状态
```bash
sudo systemctl status ros2weblink
```

### 启动服务
```bash
sudo systemctl start ros2weblink
```

### 停止服务
```bash
sudo systemctl stop ros2weblink
```

### 重启服务
```bash
sudo systemctl restart ros2weblink
```

### 禁用开机自启（如果需要）
```bash
sudo systemctl disable ros2weblink
```

### 重新启用开机自启
```bash
sudo systemctl enable ros2weblink
```

---

## 📊 查看日志

### 实时查看日志
```bash
sudo journalctl -u ros2weblink -f
```

### 查看最近 100 行日志
```bash
sudo journalctl -u ros2weblink -n 100
```

### 查看今天的日志
```bash
sudo journalctl -u ros2weblink --since today
```

### 查看某个时间段的日志
```bash
sudo journalctl -u ros2weblink --since "2025-11-17 08:00:00" --until "2025-11-17 20:00:00"
```

---

## 🔄 修改配置后重载

如果修改了 `/etc/systemd/system/ros2weblink.service` 文件：

```bash
# 1. 重载 systemd 配置
sudo systemctl daemon-reload

# 2. 重启服务
sudo systemctl restart ros2weblink

# 3. 检查状态
sudo systemctl status ros2weblink
```

---

## 🚀 服务特性

### 1. 开机自启动
- 系统启动后自动启动 ROS2WebLink
- 延迟 10 秒启动，确保网络和 ROS2 环境就绪

### 2. 自动重启
- 如果服务崩溃，10 秒后自动重启
- 配置: `Restart=always` + `RestartSec=10`

### 3. ROS2 环境加载
自动加载以下环境：
- `/opt/ros/humble/setup.bash` (ROS2 Humble)
- `/home/guest/tron_ros2/install/setup.bash` (Tron 项目)

### 4. 环境变量
- `ROS_DOMAIN_ID=0`
- `ROS_LOCALHOST_ONLY=0`
- `PYTHONUNBUFFERED=1`

---

## 🔍 健康检查

### Web UI 访问
```
http://192.168.100.88:8800
```

### API 文档
```
http://192.168.100.88:8800/docs
```

### 健康检查接口
```bash
curl http://localhost:8800/healthz
# 预期输出: {"status":"ok"}
```

---

## ⚙️ 服务配置文件位置

- **系统服务**: `/etc/systemd/system/ros2weblink.service`
- **源文件备份**: `/home/guest/ROS2WebLink/ros2weblink.service`

---

## 🛠️ 故障排查

### 1. 服务启动失败
```bash
# 查看详细状态
sudo systemctl status ros2weblink -l

# 查看最近的错误日志
sudo journalctl -u ros2weblink -n 50 --no-pager
```

### 2. 端口被占用
```bash
# 检查端口 8800 占用情况
sudo lsof -i:8800

# 如果需要杀掉占用进程
sudo kill -9 $(sudo lsof -ti:8800)

# 重启服务
sudo systemctl restart ros2weblink
```

### 3. ROS2 环境问题
检查服务是否正确加载了 ROS2 环境：
```bash
# 查看服务的环境变量
sudo systemctl show ros2weblink --property=Environment
```

### 4. 权限问题
确保 guest 用户有权限访问：
```bash
ls -la /home/guest/ROS2WebLink
ls -la /home/guest/tron_ros2
```

---

## 📝 使用 PID 版本的 tron_commander

当前配置使用 `tron_commander_pid.py`，具有以下特性：

- ✅ 双重导航成功判定（Nav2 + PID 停车状态）
- ✅ 监听 `/pid_parking_status` 话题
- ✅ 更精准的停车控制

配置位置: `web_server/navigation_controller.py` 第 92 行

---

## 🎯 开机自启动测试

要验证开机自启动是否正常工作：

```bash
# 1. 重启系统
sudo reboot

# 2. 重启后等待 1-2 分钟，然后检查服务状态
sudo systemctl status ros2weblink

# 3. 测试 Web 服务
curl http://localhost:8800/healthz
```

---

## 📞 常见问题

### Q: 如何临时禁用自启动？
```bash
sudo systemctl disable ros2weblink
```

### Q: 如何手动启动而不是用 systemd？
```bash
# 1. 停止 systemd 服务
sudo systemctl stop ros2weblink

# 2. 手动启动
cd /home/guest/ROS2WebLink
bash start_web_ui.sh
```

### Q: 修改代码后需要重启服务吗？
```bash
# 是的，需要重启服务以加载新代码
sudo systemctl restart ros2weblink
```

---

## 🎉 总结

✅ ROS2WebLink 已成功配置为开机自启动服务  
✅ 使用 systemd 管理，稳定可靠  
✅ 支持自动重启和日志记录  
✅ 集成 ROS2 环境，无需手动 source  
✅ 使用 PID 版本的 tron_commander  

**开机后服务会自动启动，无需手动操作！** 🚀


