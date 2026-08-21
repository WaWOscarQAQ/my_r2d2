# Nav2 Jazzy 插桩与实测记录（应用层插桩，R2D2 闭环在真实 ROS 上跑通）

> 日期：2026-08-19。
> 依据：`r2d2_strict_reproduction_plan.md` 阶段 B/D 与 `tracer_reproduction_plan.md`；
> 用户约束：**只插桩 nav2**，不修改 rclcpp/rcl/rmw 源码，不动系统 ROS 2 环境
> （`/opt/ros/jazzy` 只读使用，构建全部在隔离工作区进行）。
> 状态：已完成并实测通过（见第 6 节数据）。

## 1. 目标与结论

把 R2D2 的 trace 采集管线接到真实 Jazzy nav2 上：在 nav2_costmap_2d 应用层插入
tracer 调用，用 Rust 核心生成 LaserScan payload、驱动插桩节点、读取 shm trace、
做新状态判定并回池。实测闭环完整跑通：

- 12 条注册记录、6 个完整回调（`/scan` 订阅 + 5 个 clear/get_cost 服务）。
- 每轮 18–37 次真实回调执行，execution latency 为真实纳秒值（23–64 μs）。
- 8 轮测试中 2 轮触发 new-state（新执行边 + 延迟/吞吐偏离），payload 正常入池变异，
  `invalid_traces = 0`，无 crash（普通构建无 sanitizer，属预期）。

## 2. 工作区布局（全部在 my_r2d2/nav2_ws，已 gitignore，可整体删除回滚）

```text
my_r2d2/nav2_ws/
  src/navigation2/            # 从 nav2-_fuzz/nav2_ws 复制的源码副本（已插桩）
  src/r2d2_tracer/            # 我们的 tracer 打包为 ament 包（core + hooks 头）
  src/r2d2_scan_bridge/       # 文本 payload -> /scan LaserScan 发布器
  costmap_params.yaml         # costmap 独立运行的精简参数（obstacle + inflation）
  launch_stack.sh             # 静态 TF + costmap 节点 + lifecycle configure/activate
  build/ install/ log/        # 独立 colcon 前缀
```

原 nav2 工作区 `/home/ocsar/ROS/nav2-_fuzz` 与 `/opt/ros/jazzy` 全程未改动。

## 3. 插桩点清单（只改 nav2_costmap_2d 副本，每处 1–3 行）

| 文件 | 位置 | 改动 |
|---|---|---|
| `src/costmap_2d_node.cpp` | `main()` rclcpp::init 之后 | `tracer::init("r2d2_nav2")`（应用层替代论文 RCL 层 shm 初始化） |
| `plugins/obstacle_layer.cpp` | LaserScan 订阅注册（~:256） | `register_callback(observation_buffer.get(), topic, Subscription)` |
| `plugins/obstacle_layer.cpp` | PointCloud2 订阅注册（~:283） | 同上 |
| `plugins/obstacle_layer.cpp` | `laserScanCallback` / `laserScanValidInfCallback` / `pointCloud2Callback` 入口 | `CallbackScope<Msg>(buffer.get(), msg, header.stamp)` |
| `plugins/static_layer.cpp` | `map_sub_` / `map_update_sub_` 注册 | `register_callback(...)` |
| `plugins/static_layer.cpp` | `incomingMap` / `incomingUpdate` 入口 | `CallbackScope`（map 用 header.stamp；update 无 header 用无消息 scope） |
| `src/clear_costmap_service.cpp` | 4 个 clear 服务注册 + 回调入口 | `register_callback(..., Service)` + 无消息 `CallbackScope` |
| `src/costmap_2d_ros.cpp` | `get_cost` 服务注册 + `getCostCallback` 入口 | 同上 |
| `package.xml` / `CMakeLists.txt` | 依赖 | 增加 `r2d2_tracer`（nav2 的 `${dependencies}` 是显式列表，必须手动加） |

## 4. 应用层插桩相对论文的偏差（reproduction choices，逐条记录）

1. **无 executor_execute**：invoke 时间戳产生于 rclcpp 执行器内部，应用层不可见。
   scheduling latency 恒为 unknown；execution latency 按论文 start/end 计算。
   为此 `CallbackTrace` 诊断新增 `missing_invokes` 计数：invoke 缺失**本身不取消**
   状态分析资格（应用层常态）；记录真正丢失仍由 `lossy`（drain missed 计数）拦截。
   语义调整在 `src/callback_profile.rs` 与 `tests/callback_profile.rs` 中落地并回归。
2. **handler 无 RCLCPP/RCL 之分**：两个 handler 字段携带同一个 nav2 侧稳定指针
   （订阅对象 / ObservationBuffer 对象地址）；`CallbackInfo` 按此指针关联两层注册记录。
3. **时间戳域**：hooks 用 rclcpp 系统时钟（与消息 header.stamp 同域），而非核心
   tracer 的 CLOCK_MONOTONIC；`rcl_take` 的 pub_timestamp 取消息 header.stamp。
4. **buffer size**：用 `rclcpp::Serialization<MsgT>` 在回调入口序列化得到；
   服务回调无消息可 take，只记 start/end。
5. **shm 初始化**：在 nav2 节点 main() 调用 `tracer::init()`，而非论文的 RCL 层。
6. 消息吞吐单位仍为 bytes/ns（数值量级 1e-3～1e-2，打印按比例放大阅读）。

## 5. 构建步骤与踩坑记录

```bash
# 在 nav2_ws 下，全部用 bash 显式包裹（zsh 直接 source setup.bash 不可靠）：
bash -c 'source /opt/ros/jazzy/setup.bash && colcon build --symlink-install \
  --parallel-workers 12 \
  --packages-select r2d2_tracer r2d2_scan_bridge nav2_msgs nav2_common \
                     nav2_util nav2_voxel_grid nav2_costmap_2d \
  --cmake-args -DBUILD_TESTING=OFF -DCMAKE_BUILD_TYPE=RelWithDebInfo'
```

踩坑（均有解决记录，供后续阶段参考）：

1. 用户 shell 配置残留 `COLCON_CURRENT_PREFIX` / zsh 直接 source bash 脚本失败
   → 一律 `bash -c` 包裹；启动脚本 source 前 `set +u`（`setup.bash` 引用未绑定变量）。
2. `package.xml` maintainer email 必须为合法格式（`localhost` 无点号会被 catkin_pkg 拒绝）。
3. `--symlink-install` 下 ament 拒绝安装 interface library → hooks 改为带一个
   头文件编译单元的静态库。
4. 导出目标引用 `Threads::Threads` → `ament_export_dependencies(... Threads)`。
5. 静态库默认无 PIC，链接 nav2 共享库报 relocation 错 → `POSITION_INDEPENDENT_CODE ON`。
6. nav2 的 `nav2_package()` 宏不自动收集 package.xml 依赖，`${dependencies}` 是
   CMakeLists 显式列表 → 必须手动加 `r2d2_tracer`。
7. 僵尸子进程会让 `kill -0` 误判存活 → Rust 侧改用 `Child::try_wait()` 判定。

## 6. 运行与实测结果

```bash
cargo run --example nav2_costmap_e2e -- --rounds 8 --seed 42
```

```text
startup: 12 registration records, 6 complete callbacks:
  clear_around_pose [Service], /scan [Subscription], get_cost [Service],
  clear_except [Service], clear_around [Service], clear_entirely [Service]
round 01..02  baseline（只积累基准）
round 03 | calls=35 | exec=[42us,25us,37us] | decision=new-state | pool=1
round 07 | calls=37 | decision=new-state | pool=2
summary: rounds=8 crashes=0 new_states=2 invalid_traces=0 pool_size=2
         callback_graph_edges=1 distinct_callbacks=1
```

说明：单测场景只有 `/scan` 一个消息回调被执行，call_trace 内相邻对为
`(scan, scan)` 自环，因此图边只出现一条；latency/throughput 判据在 round 7
触发了一次偏离。8 轮后进程组、shm、payload 文件全部清理（已核查）。

回归：`cargo test` 87 项全过（新增 1 项 missing_invokes 语义测试）；
`cargo clippy --all-targets` 仅剩 tests/ 中 6 个既有风格警告。

## 7. 已知限制与后续

- 目前只插桩 costmap_2d 的消息/服务回调；`mapUpdateLoop`（std::thread）、
  TF 回调、参数回调未插桩。后续可扩展到 bt_navigator/planner/controller/amcl
  与 action 输入面（用户 fuzzer 的 `/navigate_to_pose` 等），届时回调图将出现
  真正多回调边。
- 普通构建无 sanitizer，crash 检出依赖进程退出码；ASAN/TSAN 构建是下一步
  （用户的 `scripts/build_nav2.sh tsan` 思路可直接迁移到本工作区）。
- Rust 读者仍是"轮末读取"而非与写入并发的实时 drain（对应 C7 缺口），
  高频率长轮次下需要并发读取验证。
- `ros2 lifecycle`/`ros2 topic` CLI 均来自 /opt/ros 二进制，瞬态 shell 使用，
  无任何写入。
