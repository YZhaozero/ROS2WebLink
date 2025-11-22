# MappingController 设计说明

## 1. 目标
- 一键控制建图流程：启动/停止/状态查询
- 管理多个 ROS launch 进程，确保按顺序启动与销毁
- 标准化地图保存流程，支持命名与目录配置

## 2. 进程拓扑

启动建图时依次拉起：
1. `livox_ros_driver2 msg_MID360_launch.py`
2. `fastlio2 lio_launch.py`
3. `tron_navigation pointcloud_to_laserscan_launch.py`
4. `slam_2d cartographer.launch.py`

每个进程使用 `subprocess.Popen` 启动并记录 PID。

## 3. 状态机

```
IDLE ── start() ──▶ STARTING ──(全部成功)──▶ RUNNING
  ▲                         │
  │ stop()                  │任一进程退出
  └────────── STOPPING ◀────┘
```

- **IDLE**：未运行
- **STARTING**：进程启动中
- **RUNNING**：全部进程启动完毕
- **STOPPING**：停止中
- **ERROR**：异常（进程意外退出）

## 4. 接口

| 方法 | 描述 |
| ---- | ---- |
| `start(map_name: str)` | 启动建图，返回状态与PID列表 |
| `stop(save: bool = True)` | 停止建图；可触发 map_save.sh |
| `status()` | 返回当前状态、运行时长、每个进程健康状况 |

## 5. 地图保存
- 调用 `/home/guest/tron_ros2/src/tron_slam/slam_2d/map_save.sh map_name`
- 捕获 stdout/stderr，返回生成的 `.pgm/.yaml` 路径
- 支持配置输出目录

## 6. 错误处理
- 任一进程退出即进入 `ERROR` 状态并记录日志
- stop() 清理所有进程并回到 `IDLE`
- 若 map_save.sh 失败，返回对应错误码

## 7. 单元测试
- `test_start_spawns_processes`：mock subprocess，验证启动顺序
- `test_stop_invokes_terminate`：确保 stop() 终止所有进程
- `test_status_reports_error`：模拟进程退出
- `test_stop_triggers_map_save`：验证地图保存命令调用




