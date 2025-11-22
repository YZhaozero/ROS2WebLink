# ExtendedRosBridge 设计说明

## 1. 职责
- 作为 FastAPI 与 ROS2 世界的桥梁
- 订阅并缓存关键话题：地图、代价地图、导航状态
- 提供目标发布、速度控制、暂停/恢复等操作
- 为外部模块提供线程安全的数据访问接口

## 2. 订阅话题

| 话题 | 类型 | 描述 |
| ---- | ---- | ---- |
| `/map` | `nav_msgs/msg/OccupancyGrid` | 全局栅格地图 |
| `/global_costmap/costmap` | `nav_msgs/msg/OccupancyGrid` | Nav2 全局代价地图 |
| `/local_costmap/costmap` | `nav_msgs/msg/OccupancyGrid` | Nav2 局部代价地图 |
| `/nav_status` | `std_msgs/msg/String` | 任务状态（IDLE/MOVING/SUCCEEDED等） |
| `/navigation_status` | `std_msgs/msg/String` | 扩展导航状态 |
| `/navigate_to_pose/_action/feedback` | `nav2_msgs/action/NavigateToPose_FeedbackMessage` | 实时位姿反馈 |

## 3. 发布话题

| 话题 | 类型 | 描述 |
| ---- | ---- | ---- |
| `/goal` | `geometry_msgs/msg/PoseStamped` | Nav2 目标点 |
| `/goal_id` | `std_msgs/msg/String` | 目标任务 ID |
| `/cmd_vel_web` | `geometry_msgs/msg/Twist` | Web手动控制速度 |
| `/pause_navigation` | `std_msgs/msg/String` | Nav2 暂停指令 |
| `/resume_navigation` | `std_msgs/msg/String` | Nav2 恢复指令 |

## 4. 数据接口
- `get_map_json()`：返回 Base64 编码地图信息
- `get_costmap_json(kind="global"/"local")`：返回 costmap 数据
- `get_navigation_status()`：返回最新 nav_status 与 feedback 位姿
- `publish_goal(goal: dict)`：下发导航目标
- `publish_cmd_vel(cmd: dict)`：发送速度指令

## 5. 线程安全
- 使用 `threading.Lock` 保护共享数据
- 提供 `copy.deepcopy` 或 dict 拷贝，防止外部修改内部缓存

## 6. 错误处理
- topic 尚未接收时返回 `Result:1` + 错误信息
- 发布异常捕获后向调用者抛出 RuntimeError

## 7. 单元测试
- costmap 订阅后能正确转换成 JSON（包含 width/height/resolution/data）
- 发布函数正确调用 rclpy publisher 的 `publish`
- nav_status 回调更新缓存




