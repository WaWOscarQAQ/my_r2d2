# ros2_tracing Overlay 落地状态
日期：2026-08-25

## 1. 目标
- 用 `ros2_tracing + rcl + rclcpp` 源码 overlay 替换应用层 interpose。
- 保持真实 ROS topic/service/action/safe-parameter-profile 闭环。
- 让 benchmark/fuzz 从 live runtime trace 中得到可分析的 callback/message profile。

## 2. 已完成
- overlay 源码已导入：
  - `overlay_ws/src/ros2_tracing`
  - `overlay_ws/src/rcl`
  - `overlay_ws/src/rclcpp`
- runtime 插桩已改到源码层：
  - `rcl` 负责 entity/name 注册与 `rcl_take`
  - `rclcpp` 负责 callback registration、`executor_execute`、`callback_start/end`
- 启动路径已改为 source overlay：
  - `nav2_ws/launch_stack.sh` source `overlay_ws/install/setup.bash`
  - `LD_PRELOAD` 已删除
- 目标应用重建已补齐：
  - overlay runtime build 完后，再重建 `nav2_costmap_2d`
- Rust 侧 round 有效性口径已修正：
  - `incomplete_registrations` 只保留为诊断
  - round 是否 invalid 由本轮实际出现的 `unknown/unmatched/lossy/...` 决定

## 3. 实测结果
- 修复前：
  - `benchmark: elapsed=61s rounds=29 analyzed=0 empty=0 invalid=29`
- 修复后：
  - `benchmark: rounds=4 analyzed=4 empty=0 invalid=0 edges=1 callbacks=4`
- 说明：
  - tracer 事件齐全
  - callback/message profile 已能进入 benchmark reference

## 4. 当前构建顺序
1. 构建基础 `nav2_ws`
2. 构建 `overlay_ws` 中的 `tracetools/rcl/rclcpp`
3. source `overlay_ws/install/setup.bash`
4. 重建 `nav2_costmap_2d`

## 5. 当前成功判据
- 不再依赖 `LD_PRELOAD`
- startup 能看到 `/scan`、`/map`、`/clear_*`、`/get_costmap` 进入 complete callbacks
- benchmark 出现 `analyzed > 0`
- benchmark 中 `invalid=0`

## 6. 剩余工作
- coverage 实时反馈已按两处根因收敛：
  - overlay 重建继承 nav2 当前 coverage/tsan 模式
  - stack 改为直启真实 `nav2_costmap_2d` 二进制，`SIGUSR1` 不再打在 `ros2 run`
- 新的实时观测面：
  - `nav2_ws/results/coverage/status.json`：benchmark baseline 与最新一轮累计分支覆盖
  - `nav2_ws/results/rounds/round_XXXXXX/summary.json`：逐轮覆盖增量
- 剩余只需要继续验证“长时间全量 fuzz 下是否还会出现栈死亡或 lcov 偶发失败”。
