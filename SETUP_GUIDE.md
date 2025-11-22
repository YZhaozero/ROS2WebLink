# ROS2WebLink 配置指南

## 📋 缺失依赖检查结果

### ✅ 已具备：
- ✓ Python 3.10+
- ✓ ROS2 Humble 环境
- ✓ rclpy (ROS2 Python客户端)
- ✓ websocket-client
- ✓ routes 目录
- ✓ 地图存储目录

### ❌ 需要安装的Python依赖：

1. **fastapi** - Web框架核心
2. **uvicorn** - ASGI异步服务器
3. **pydantic** - 数据验证框架

## 🚀 快速安装

**注意：ROS2项目无需使用虚拟环境，直接安装到系统即可**

### 方法1：使用requirements.txt（推荐）

```bash
cd /home/guest/ROS2WebLink
pip3 install -i https://pypi.tuna.tsinghua.edu.cn/simple -r requirements.txt
```

### 方法2：手动安装

```bash
pip3 install -i https://pypi.tuna.tsinghua.edu.cn/simple \
    fastapi \
    uvicorn[standard] \
    pydantic
```

**为什么不用虚拟环境？**
- ROS2项目需要访问系统级的 `rclpy` 包
- 依赖少，不会污染系统环境
- 配置简单，启动方便

## ✅ 验证安装

运行以下命令验证所有依赖：

```bash
python3 -c "
import fastapi
import uvicorn
import pydantic
import rclpy
import websocket
print('✓ 所有依赖已成功安装！')
"
```

## 🎯 启动服务

安装完成后，使用以下命令启动：

```bash
cd /home/guest/ROS2WebLink
./start_web_ui.sh
```

然后在浏览器中访问：`http://<机器人IP>:8800`

## 📂 目录结构

```
ROS2WebLink/
├── web_server/          # 服务器代码
│   ├── ros_web_server.py
│   ├── mapping_nav_server.py
│   └── ...
├── routes/              # 路线文件存储 ✓ 已创建
├── requirements.txt     # Python依赖 ✓ 已创建
├── start_web_ui.sh     # 启动脚本
└── index.html          # Web界面
```

## 🔗 关联目录

- **地图文件**: `/home/guest/tron_ros2/src/tron_nav/tron_navigation/maps/` ✓
- **ROS2工作空间**: `/home/guest/tron_ros2/` ✓

## 📝 注意事项

1. 确保ROS2环境已source：
   ```bash
   source /opt/ros/humble/setup.bash
   source /home/guest/tron_ros2/install/setup.bash
   ```

2. 如果端口8800被占用，可以修改 `start_web_ui.sh` 中的端口配置

3. 日志文件位置：`/tmp/webui.log`

## 🆘 故障排除

### 问题：导入 fastapi 失败
**解决**：重新安装依赖
```bash
pip3 install --force-reinstall -i https://pypi.tuna.tsinghua.edu.cn/simple fastapi uvicorn pydantic
```

### 问题：端口已被占用
**解决**：杀死占用进程
```bash
pkill -9 -f uvicorn
```

### 问题：ROS2话题无法连接
**解决**：确保ROS2环境已正确配置
```bash
source /opt/ros/humble/setup.bash
source /home/guest/tron_ros2/install/setup.bash
```

---

**完成以上步骤后，ROS2WebLink 即可正常使用！**

