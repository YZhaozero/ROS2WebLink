# ROS2WebLink

ROS2WebLink是一个用于连接ROS2机器人操作系统与Web界面的桥梁项目，提供了简单易用的HTTP接口和Web界面，方便用户通过浏览器与ROS2系统进行交互。

## 功能特点

- **HTTP接口服务**：提供RESTful API接口，支持ROS2话题发布、订阅、服务调用
- **Web界面**：内置简洁的Web界面，支持实时数据展示和交互操作
- **多模式运行**：支持仅服务器模式和服务器+界面模式


## 快速开始

### 环境要求

- Python 3.7+
- ROS2 (Foxy或更高版本)
- 现代Web浏览器（Chrome、Firefox、Safari等）

### 安装依赖

```bash
# 安装Python依赖
pip install flask flask-cors

# 确保ROS2环境已配置
source /opt/ros/<your-ros2-version>/setup.bash
```

### 启动服务器

```bash
# 进入项目目录
cd /home/zy/ws/ROS2WebLink

# 启动服务器（仅服务器模式）
python3 web_server/ros_web_server.py

# 启动服务器并打开Web界面
python3 web_server/ros_web_server.py --open-browser
```

## 使用说明

### 服务器参数

```bash
python3 web_server/ros_web_server.py [选项]

选项：
  --port PORT       指定服务器端口（默认：8000）
  --host HOST       指定服务器主机（默认：localhost）
  --mode MODE       运行模式（simulation/ros）（默认：simulation）
```

### index.html

index.html是Web界面的主文件，位于web_server目录下。它包含了用户交互的HTML元素和JavaScript代码，用于与服务器进行通信并展示数据。


### 调试方法
 
```bash
# 启用模拟数据模式
python3 web_server/ros_web_server.py --mode simulation

# 打开index.html
xdg-open index.html
```