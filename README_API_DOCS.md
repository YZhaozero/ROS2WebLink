# ROS2WebLink API 文档访问指南

## 📚 文档位置

ROS2WebLink 提供了三种方式访问 API 文档：

### 1. 🌐 Web 交互式文档（推荐）

**访问地址**: http://localhost:8800/api/docs

**特点**:
- ✨ 精美的交互式界面
- 🔍 强大的搜索功能
- 📝 详细的请求/响应示例
- 💻 cURL 命令示例
- 📱 响应式设计，支持移动端

**使用方法**:
1. 启动 ROS2WebLink 服务器
2. 在浏览器中打开 http://localhost:8800/api/docs
3. 使用顶部标签页切换不同模块
4. 点击 API 项目展开查看详情
5. 使用搜索框快速查找 API

### 2. 📖 Markdown 完整文档

**文件位置**: `/home/guest/ROS2WebLink/API_DOCUMENTATION.md`

**特点**:
- 📄 完整的 Markdown 格式
- 🔗 支持目录跳转
- 📋 包含所有 API 详细说明
- 🎯 包含 WebSocket 协议说明
- 💡 包含前端页面说明和常见问题

**使用方法**:
```bash
# 在终端中查看
cat /home/guest/ROS2WebLink/API_DOCUMENTATION.md

# 使用 Markdown 阅读器
code /home/guest/ROS2WebLink/API_DOCUMENTATION.md
```

### 3. 🔧 Swagger/OpenAPI 文档

**访问地址**: 
- Swagger UI: http://localhost:8800/docs
- ReDoc: http://localhost:8800/redoc

**特点**:
- 🤖 FastAPI 自动生成
- 🧪 支持在线测试 API
- 📊 显示请求/响应模型
- 🔐 支持认证测试

---

## 🚀 快速开始

### 启动服务器

```bash
# 进入项目目录
cd /home/guest/ROS2WebLink

# 激活虚拟环境（如果有）
source .venv/bin/activate

# 启动服务器
python -m uvicorn web_server.mapping_nav_server:app --host 0.0.0.0 --port 8800

# 或使用启动脚本
./start_web_ui.sh
```

### 访问文档

服务器启动后，在浏览器中访问：

1. **主控制台**: http://localhost:8800/
2. **API 文档**: http://localhost:8800/api/docs
3. **Swagger**: http://localhost:8800/docs

---

## 📋 API 分类概览

### 1. 建图 API (Mapping)
- 启动/停止建图
- 查询建图状态
- 清空缓存

### 2. 地图管理 API (Maps)
- 列出/加载/删除地图
- 地图注册表管理
- 实时地图数据

### 3. 导航 API (Navigation)
- 启动/停止导航系统
- 取消导航目标
- 查询导航状态

### 4. 机器人控制 API (Robot Control)
- 位姿设置
- 速度控制
- 模式切换
- 高度调节

### 5. 航点路线 API (Waypoints & Routes)
- 记录航点
- 轨迹录制
- 路线管理和执行

### 6. 传感器状态 API (Sensors)
- 传感器健康状态
- DLIO 状态
- 代价地图
- 定位器日志

### 7. 巡检回调 API (Inspection)
- 导航结果回调
- 导航状态查询

### 8. WebSocket 协议
- 机器人 SDK 通信
- 实时状态推送
- 速度控制

---

## 🎯 常用 API 示例

### 启动建图
```bash
curl -X POST http://localhost:8800/api/mapping/start \
  -H "Content-Type: application/json" \
  -d '{"map_name": "my_map"}'
```

### 设置导航目标
```bash
curl -X POST http://localhost:8800/api/robot/navigation_goal \
  -H "Content-Type: application/json" \
  -d '{
    "goal_x": 5.0,
    "goal_y": 3.0,
    "goal_theta": 1.57,
    "goal_id": 1
  }'
```

### 获取机器人状态
```bash
curl http://localhost:8800/api/robot/status
```

---

## 📝 文档维护

### 更新文档

如果添加了新的 API 端点，请更新：

1. **Markdown 文档**: `/home/guest/ROS2WebLink/API_DOCUMENTATION.md`
2. **HTML 文档**: `/home/guest/ROS2WebLink/web_server/static/api_docs.html`
3. **OpenAPI 注解**: 在代码中添加 FastAPI 注解

### 文档生成

FastAPI 的 Swagger 文档会自动生成，确保每个端点都有：

```python
@app.post("/api/example")
async def example_endpoint(payload: RequestModel):
    """
    端点描述
    
    - 详细说明
    - 使用场景
    """
    pass
```

---

## ❓ 常见问题

### Q: 文档无法访问？

**A**: 检查以下几点：
1. 服务器是否已启动
2. 端口 8800 是否被占用
3. 防火墙是否允许访问
4. 静态文件路径是否正确

### Q: Swagger 文档不完整？

**A**: FastAPI 的 Swagger 文档基于代码注解自动生成，如果某些端点没有显示：
1. 检查是否添加了 `include_in_schema=False`
2. 检查是否有类型注解
3. 检查是否有 docstring

### Q: 如何贡献文档？

**A**: 
1. 更新 `API_DOCUMENTATION.md`
2. 更新 `api_docs.html`（如需）
3. 在代码中添加详细注释
4. 提交 Pull Request

---

## 📞 联系方式

如有问题或建议，请：
- 查看项目 README
- 查看日志文件 `/tmp/navigation_*.log` 和 `/tmp/mapping_*.log`
- 检查 FastAPI 日志输出

---

**文档版本**: 1.0.0  
**最后更新**: 2024-11-18  
**维护者**: ROS2WebLink Team


