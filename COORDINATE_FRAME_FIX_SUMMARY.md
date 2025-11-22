# 坐标系优先级修复总结

## 问题描述
之前系统同时订阅了两个 odometry topic：
- `/tron_commander/odom` (frame_id: **map**)
- `/dlio/odom_node/odom` (frame_id: **odom**)

由于没有区分 frame_id，导致 odom frame 的数据覆盖了 map frame 的数据，使得用 `/api/robot/position` 获取的位置（odom frame）发送给导航目标（需要 map frame）时，机器人去了错误的位置。

## 解决方案

### 1. 实现坐标系优先级逻辑
在 `ros_extended_node.py` 中实现：
- **优先使用 map frame** 的数据（来自 tron_commander）
- **仅在没有 map frame 时使用 odom frame**（来自 dlio，建图模式）
- 存储并返回 `frame_id` 信息

### 2. 修改内容

**`ros_extended_node.py`**
- 第 131 行：`_robot_position` 类型改为 `Dict[str, Any]` 以存储 frame_id
- 第 201-232 行：重写 `_robot_odom_callback`，实现优先级逻辑
  - 如果已有 map frame 数据，跳过 odom frame 更新
  - 存储 frame_id 到位置字典中
- 第 259-261 行：`get_robot_position` 返回类型包含 frame_id

**`mapping_nav_server.py`**
- 第 397-405 行：`/api/robot/position` 返回包含 frame_id
- 第 355-372 行：`/api/robot/status` 的 localization 字段包含 frame_id

## 验证结果

### ✅ 坐标系验证
```bash
/tron_commander/odom:
  frame_id: map
  position: x=3.44, y=7.19

/dlio/odom_node/odom:
  frame_id: odom
  position: x=1.25, y=1.32 (不同的坐标系！)
```

### ✅ API 返回验证
```json
// /api/robot/position
{
    "x": 3.444,
    "y": 7.198,
    "yaw": 1.774,
    "frame_id": "map"  ← 新增
}

// /api/robot/status
{
    "localization": {
        "x": 3.446,
        "y": 7.198,
        "theta": 1.760,
        "reliability": 0.95,
        "frame_id": "map"  ← 新增
    }
}
```

### ✅ 优先级逻辑验证
日志显示：
```
[DEBUG] Robot position updated: x=3.445, y=7.194, yaw=1.768, frame=map
[DEBUG] Skipping odom frame update (already have map frame)
[DEBUG] Skipping odom frame update (already have map frame)
[DEBUG] Skipping odom frame update (already have map frame)
...
```

## 预期效果

### 导航模式（有 tron_commander）
- ✅ 使用 map frame 位置
- ✅ 导航目标和当前位置在同一坐标系
- ✅ 发送当前位置作为目标时，机器人应该原地不动或移动很小距离

### 建图模式（没有 tron_commander）
- ✅ Fallback 到 odom frame 位置
- ✅ 前端至少能显示机器人位置（相对于启动点）

## 下一步测试（需要用户确认）
1. ✅ 验证 API 返回正确的 frame_id
2. ✅ 验证优先级逻辑正常工作
3. ⏸️ **待测试**：实际发送导航目标，验证机器人导航到正确位置
   - 获取当前位置（应该是 map frame）
   - 发送该位置作为导航目标
   - 验证机器人是否回到原点或移动很小距离

## 注意事项
- 前端可能需要根据 frame_id 显示不同的提示
- 建图模式下 frame_id 是 odom，不应该用于导航
- 导航目标应该只在 frame_id=map 时发送

