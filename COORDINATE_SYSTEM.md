# 坐标系统统一说明文档

## 概述

本文档详细说明了 ROS2 SLAM 系统中所有坐标转换的统一实现，确保建图、导航、地图显示、机器人定位等功能使用一致的坐标系统。

---

## 1. 坐标系统定义

### 1.1 ROS2 世界坐标系（World Frame）
- **原点位置**：地图的左下角（由 `origin_x`, `origin_y` 定义）
- **X 轴**：向右为正
- **Y 轴**：向上为正
- **单位**：米（meters）
- **用途**：机器人位置、导航目标、航点等

```
  Y ↑
    │
    │  机器人 (world_x, world_y)
    │
    └──────────→ X
   (origin_x, origin_y)
```

### 1.2 ROS2 OccupancyGrid 数据格式
- **数据结构**：一维数组（`uint8[]`）
- **存储顺序**：row-major（按行存储）
- **起始位置**：`data[0]` 对应地图的**左下角**（origin）
- **索引公式**：`index = y * width + x`（其中 y=0 是底部）
- **数值含义**：
  - `0` = 空闲空间（free space）
  - `100` = 障碍物（obstacle）
  - `-1` 或其他 = 未知（unknown）

```
ROS2 地图数据布局:
  data[N-1] ─┐
  data[...] ─┤  从下到上
  data[0]   ─┘  (左下角)
```

### 1.3 Canvas 像素坐标系
- **原点位置**：画布的左上角
- **X 轴**：向右为正
- **Y 轴**：向下为正
- **单位**：像素（pixels）

```
Canvas 坐标系:
  (0,0)────────→ X
    │
    │  显示区域
    │
    ↓ Y
```

---

## 2. 坐标转换公式

### 2.1 地图渲染：ROS2 数据 → Canvas 图像

在 `renderOccupancyGrid()` 函数中，我们需要**垂直翻转**地图数据：

```javascript
function renderOccupancyGrid(rawData, width, height) {
  const image = ctx.createImageData(width, height);
  
  for (let y = 0; y < height; y++) {
    for (let x = 0; x < width; x++) {
      const srcIdx = y * width + x;           // ROS2 数据索引
      const dstY = height - 1 - y;            // 翻转 Y 坐标
      const dstIdx = (dstY * width + x) * 4;  // Canvas 数据索引
      
      // 根据 rawData[srcIdx] 的值设置颜色
      // ...
    }
  }
  return image;
}
```

**关键点**：
- ROS2 的 y=0（底部）→ Canvas 的 y=height-1（底部显示在下方）
- ROS2 的 y=height-1（顶部）→ Canvas 的 y=0（顶部显示在上方）

### 2.2 世界坐标 → Canvas 像素坐标

用于绘制机器人位置、导航目标、航点等。

```javascript
// 输入：世界坐标 (worldX, worldY)
// 输出：Canvas 像素坐标 (pixelX, pixelY)

const pixelX = (worldX - originX) / resolution;
const pixelY = (worldY - originY) / resolution;
```

**说明**：
- 因为地图在 `renderOccupancyGrid` 中已经翻转，所以这里**不需要再翻转** Y 坐标
- 直接计算相对于 origin 的偏移，再除以分辨率即可

**应用位置**：
- `drawRobot()` - 绘制机器人位置
- `drawInitialPose()` - 绘制初始位姿
- `drawNavGoal()` - 绘制导航目标
- `recordWaypoint()` - 录制航点
- `stopTrajectoryRecording()` - 轨迹录制

### 2.3 Canvas 像素坐标 → 世界坐标

用于鼠标点击地图时，将点击位置转换为世界坐标。

```javascript
// 输入：Canvas 像素坐标 (canvasX, canvasY)
// 输出：世界坐标 (worldX, worldY)

const worldX = originX + canvasX * resolution;
const worldY = originY + canvasY * resolution;
```

**说明**：
- 这是上述转换的反向操作
- 同样**不需要翻转** Y 坐标

**应用位置**：
- `handleCanvasClick()` - 处理地图点击事件
  - 设置初始位姿
  - 设置导航目标
  - 添加航点

---

## 3. 关键参数说明

### 3.1 地图元数据（Map Metadata）

```javascript
{
  width: 164,           // 地图宽度（栅格数）
  height: 263,          // 地图高度（栅格数）
  resolution: 0.05,     // 分辨率（米/栅格）
  origin_x: -3.1,       // 地图原点 X 坐标（米）
  origin_y: -3.7        // 地图原点 Y 坐标（米）
}
```

### 3.2 坐标转换示例

假设：
- 地图原点：`(-3.1, -3.7)`
- 分辨率：`0.05 m/格`
- 地图尺寸：`164 x 263 格`

**示例 1：机器人位置**
```
世界坐标：(-0.121, -0.036)
↓ 转换
Canvas 坐标：(59.6, 73.3)
```

**示例 2：地图四个角**
```
左下角（origin）: (-3.10, -3.70) → (0, 0)
右下角:          ( 5.10, -3.70) → (164, 0)
左上角:          (-3.10,  9.45) → (0, 263)
右上角:          ( 5.10,  9.45) → (164, 263)
```

---

## 4. 实现验证

### 4.1 正向转换 + 反向转换 = 恒等

```python
# 正向：世界 → 像素
pixel_x = (world_x - origin_x) / resolution
pixel_y = (world_y - origin_y) / resolution

# 反向：像素 → 世界
back_world_x = origin_x + pixel_x * resolution
back_world_y = origin_y + pixel_y * resolution

# 验证
assert abs(back_world_x - world_x) < 1e-6
assert abs(back_world_y - world_y) < 1e-6
```

### 4.2 边界测试

所有世界坐标在地图范围内时，转换后的像素坐标应该满足：
```
0 <= pixelX <= width
0 <= pixelY <= height
```

---

## 5. 常见问题

### Q1: 为什么地图看起来上下颠倒？
**A**: 如果地图看起来颠倒，说明 `renderOccupancyGrid` 中的翻转逻辑有问题。确保使用 `dstY = height - 1 - y`。

### Q2: 为什么机器人位置不对？
**A**: 检查以下几点：
1. 地图是否已加载（`state.mapImage` 不为空）
2. 机器人位置数据是否正常（`/api/robot/position` 返回有效数据）
3. 地图元数据是否正确（`origin_x`, `origin_y`, `resolution`）

### Q3: 为什么点击地图后导航目标位置不对？
**A**: 确保 `handleCanvasClick` 中的坐标转换**不翻转** Y 坐标，因为地图已经在渲染时翻转过了。

### Q4: 建图和导航的坐标系统是否一致？
**A**: 是的。建图（DLIO）、导航（Nav2）、地图显示（WebUI）都使用相同的 ROS2 世界坐标系，只是在前端显示时需要处理 Canvas 坐标系的差异。

---

## 6. 修改历史

### 2025-11-05
- ✅ 统一了所有坐标转换公式
- ✅ 修复了 `handleCanvasClick` 中的 Y 坐标翻转问题
- ✅ 验证了正向和反向转换的一致性
- ✅ 添加了 FastDDS 配置以解决 DLIO 崩溃问题

---

## 7. 代码位置

### 前端（JavaScript）
- **地图渲染**：`app.js` → `renderOccupancyGrid()`
- **机器人绘制**：`app.js` → `drawRobot()`
- **初始位姿绘制**：`app.js` → `drawInitialPose()`
- **导航目标绘制**：`app.js` → `drawNavGoal()`
- **鼠标点击处理**：`app.js` → `handleCanvasClick()`

### 后端（Python）
- **地图数据提供**：`mapping_nav_server.py` → `/api/maps/{map_name}/load`
- **机器人位置**：`ros_extended_node.py` → `_robot_odom_callback()`
- **坐标转换**：所有转换在前端完成，后端只提供原始数据

---

## 8. 测试脚本

测试脚本位于：`/tmp/full_coord_test.py`

运行测试：
```bash
python3 /tmp/full_coord_test.py
```

---

**文档维护者**：AI Assistant  
**最后更新**：2025-11-05  
**版本**：1.0

