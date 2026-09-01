# ROS 2 接口矩阵与 Nav2 落地清单

日期：2026-08-23

> 当前口径（2026-09-01）：本文前半部分仍作为 ROS 2 接口背景保留；早期
> `nav2_costmap_2d` costmap-only 记录已不是正式运行路线。正式路线只保留
> Nav2 full-stack 合并 harness：以论文 callback-trace 方法为核心，输入面覆盖
> topic / service / action / safe parameter profile。当前完整输入清单见
> `docs/research/nav2_fuzzer_current_inputs.md`。

## 1. 官方 ROS 2 接口矩阵

ROS 2 官方文档把通信接口分成三类：

| 类别 | 官方定义 | 接口文件 |
|---|---|---|
| Topic | 一对多、异步流式消息 | `.msg` |
| Service | 一问一答、请求/响应 | `.srv` |
| Action | 长时任务、可反馈、可取消 | `.action` |

官方来源：

- ROS 2 Topics / Services / Actions: <https://docs.ros.org/en/ros2_documentation/rolling/Concepts/Basic/Interfaces-Topics-Services-Actions.html>
- ROS 2 About Interfaces: <https://docs.ros.org/en/jazzy/Concepts/Basic/About-Interfaces.html>

另外，官方接口语言明确支持：

- 普通字段
- 常量
- 默认值
- 变长数组 / 定长数组

这直接影响本仓库的接口提取器，因为 Jazzy 官方接口文件里真实使用了常量和默认值，例如：

- `sensor_msgs/msg/PointField.msg` 使用常量
- `geometry_msgs/msg/Quaternion.msg` 使用默认值

因此这次实现先把 `FileExtractor` 改到可以接受官方 `.msg/.srv` 语法，而不是只接受仓库里裁剪过的 fixture。

## 2. 本轮实现边界

当前正式运行边界不再拆成“论文路径”和“Jazzy full-stack 路径”，也不再停留在
costmap-only topic/service 子集。合并路线的含义是：

1. 按 ROS 2 官方文档确认接口类别包含 topic / service / action。
2. 在同一个 Nav2 full-stack harness 中使用论文的 dry run / benchmark /
   callback-trace oracle / new-state pool 主循环。
3. 输入面扩展为真实可发送的 topic / service / action，以及经过 safe profile
   限定并轮后 restore 的 parameter。
4. coverage attribution 只进入报告，不进入调度器。

## 3. 官方 Nav2 文档可支撑的 costmap 输入面

### 3.1 Topic 面

官方 Nav2 文档表明：

- Obstacle Layer 接收 `LaserScan` 或 `PointCloud2`
- Static Layer 接收 `map`，并可选接收 `map_updates`

官方来源：

- Obstacle Layer: <https://docs.nav2.org/configuration/packages/costmap-plugins/obstacle.html>
- Static Layer: <https://docs.nav2.org/configuration/packages/costmap-plugins/static.html>
- Mapping / Localization setup guide: <https://docs.nav2.org/setup_guides/sensors/mapping_localization.html>

据此，本轮在真实闭环里实现的 topic 输入为：

| 端点 | 类型 | 依据 |
|---|---|---|
| `/scan` | `sensor_msgs/msg/LaserScan` | Obstacle Layer 官方文档 |
| `/points` | `sensor_msgs/msg/PointCloud2` | Obstacle Layer 官方文档 |
| `/map` | `nav_msgs/msg/OccupancyGrid` | Static Layer 官方文档 |
| `/map_updates` | `map_msgs/msg/OccupancyGridUpdate` | Static Layer 官方文档 + `map_topic + "_updates"` 订阅实现 |

### 3.2 Service 面

官方 `nav2_msgs` 接口文档覆盖了本轮使用的 costmap services：

- `nav2_msgs/srv/GetCost`
- `nav2_msgs/srv/GetCostmap`
- `nav2_msgs/srv/ClearCostmapExceptRegion`
- `nav2_msgs/srv/ClearCostmapAroundRobot`
- `nav2_msgs/srv/ClearCostmapAroundPose`
- `nav2_msgs/srv/ClearEntireCostmap`

官方来源：

- GetCost: <https://docs.ros.org/en/jazzy/p/nav2_msgs/srv/GetCost.html>
- GetCostmap: <https://docs.ros.org/en/jazzy/p/nav2_msgs/srv/GetCostmap.html>
- ClearCostmapAroundPose: <https://docs.ros.org/en/jazzy/p/nav2_msgs/srv/ClearCostmapAroundPose.html>
- ClearEntireCostmap: <https://docs.ros.org/en/iron/p/nav2_msgs/interfaces/srv/ClearEntireCostmap.html>
- ClearCostmapAroundRobot: <https://docs.ros.org/en/iron/p/nav2_msgs/interfaces/srv/ClearCostmapAroundRobot.html>
- ClearCostmapExceptRegion: 本机 Jazzy 安装树接口文件 `/opt/ros/jazzy/share/nav2_msgs/srv/ClearCostmapExceptRegion.srv`

据此，本轮在真实闭环里实现的 service 输入为：

| 端点 | 类型 | 当前实现来源 |
|---|---|---|
| `/get_cost_costmap` | `nav2_msgs/srv/GetCost` | `costmap_2d_ros.cpp` |
| `/get_costmap` | `nav2_msgs/srv/GetCostmap` | `costmap_2d_publisher.cpp` |
| `/clear_except_costmap` | `nav2_msgs/srv/ClearCostmapExceptRegion` | `clear_costmap_service.cpp` |
| `/clear_around_costmap` | `nav2_msgs/srv/ClearCostmapAroundRobot` | `clear_costmap_service.cpp` |
| `/clear_around_pose_costmap` | `nav2_msgs/srv/ClearCostmapAroundPose` | `clear_costmap_service.cpp` |
| `/clear_entirely_costmap` | `nav2_msgs/srv/ClearEntireCostmap` | `clear_costmap_service.cpp` |

## 4. 本仓这次落地的代码点

### 4.1 接口提取

- `src/interface_extractor.rs`
- 改动：忽略常量，接受默认值，直接解析 Jazzy 官方安装树接口文件

### 4.2 发送矩阵

- `src/runtime/ros2_sender.rs`
- 保留 `Ros2LaserScanSender`
- 新增：
  - `Ros2TopicSender`
  - `Ros2ServiceSender`
  - `Ros2ActionSender`
  - safe parameter sender helpers

### 4.3 e2e harness

- `examples/nav2_costmap_e2e.rs`
- 改动：
  - dry run 从 `/opt/ros/jazzy/share` 提取官方接口
  - benchmark 前先执行真实 ready barrier：等待 `/costmap` 出现在 ROS graph，等待 lifecycle 可查询，必要时依次 `configure -> activate`，再等待必需 service 与参数服务 ready，最后等待 registration trace 收敛
  - 只有 ready barrier 通过后，才执行 dry run 绑定提取和 benchmark
  - 每轮按接口类型分发到 topic/service/action/parameter sender
  - readiness sequence 只发布基础 pose/odom/scan 来维持合法状态，不写固定
    `/goal_pose` 或 coverage-path scripting

### 4.4 nav2 真正接线

- `nav2_ws/costmap_params.yaml`
- 改动：
  - 开启 `static_layer`
  - 开启 `subscribe_to_updates`
  - 把 `observation_sources` 从 `scan` 扩成 `scan pointcloud`
  - 新增 `/points` 的 `PointCloud2` 输入

### 4.5 tracer 覆盖补齐

- `nav2_ws/src/navigation2/nav2_costmap_2d/src/costmap_2d_publisher.cpp`
- 改动：
  - 为 `GetCostmap` service 补 `register_callback`
  - 为 `GetCostmap` service callback 补 `CallbackScope`

## 5. 本轮最终接口实现清单

### Topic

1. `/scan` -> `sensor_msgs/msg/LaserScan`
2. `/points` -> `sensor_msgs/msg/PointCloud2`
3. `/map` -> `nav_msgs/msg/OccupancyGrid`
4. `/map_updates` -> `map_msgs/msg/OccupancyGridUpdate`

### Service

1. `/get_cost_costmap` -> `nav2_msgs/srv/GetCost`
2. `/get_costmap` -> `nav2_msgs/srv/GetCostmap`
3. `/clear_except_costmap` -> `nav2_msgs/srv/ClearCostmapExceptRegion`
4. `/clear_around_costmap` -> `nav2_msgs/srv/ClearCostmapAroundRobot`
5. `/clear_around_pose_costmap` -> `nav2_msgs/srv/ClearCostmapAroundPose`
6. `/clear_entirely_costmap` -> `nav2_msgs/srv/ClearEntireCostmap`

这份清单是早期 costmap-only 子集。当前 full-stack dry run 的正式输入面是
67 个 binding：23 topic、26 service、11 action、7 safe parameter profile。
完整端点列表以 `docs/research/nav2_fuzzer_current_inputs.md` 和运行日志里的
`dry run: extracted ... full-stack interfaces` 为准。

## 6. 2026-08-23 实机验证

验证命令：

```bash
ROS_DOMAIN_ID=191 cargo run --example nav2_costmap_e2e -- --benchmark-seconds 5 --rounds 0
```

实测顺序：

1. `launch_stack.sh` 只负责拉起栈，不再固定 `sleep 3` 后盲目执行 lifecycle。
2. harness 在真实 ROS graph 中等待 `/costmap` 可见。
3. harness 轮询 lifecycle state，必要时执行 `configure` 与 `activate`。
4. harness 继续等待六个 costmap services、`/costmap` 参数服务以及 registration trace 收敛。
5. ready barrier 通过后才执行 dry run、`/map` bootstrap 和 benchmark。

实测结果：

- ready barrier 通过，打印 `startup: ready barrier passed`
- dry run 成功抽取当时目标配置下的真实接口
- benchmark 5 秒内成功完成真实 ROS 发送
- 本轮未再出现 `Node not found`
- 运行摘要为 `crashes=0 new_states=0 invalid_traces=0 empty_rounds=0`
