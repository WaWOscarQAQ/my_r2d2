# ros2_tracing 源码扩展研究
日期：2026-08-25

> 历史研究记录：本文描述的源码扩展方案已停用。当前实现是目标源码零改动的
> LLVM 双层插桩，见 `docs/plan/non_invasive_llvm_instrumentation.md`。

## 1. 目的
- 把当前 `LD_PRELOAD` runtime interpose 进一步推进到“源码 overlay 扩展 `ros2_tracing + rcl + rclcpp`”。
- 对齐论文表述：
  - 基于 `Ros2Trace/ros2_tracing`
  - 在 `RCLCPP` 和 `RCL` 层增加 tracepoint
  - 扩展 tracer 字段
  - 用 shared memory 做实时反馈

## 2. 官方基础事实
- `ros2_tracing` 是官方 tracing 基座，不是论文独创。
- 官方 README 说明：
  - tracing 依赖 `LTTng`
  - 需要随 ROS distro 分支使用对应源码
  - `tracetools` 暴露 tracepoint 宏与调用入口
- 官方 design 文档说明：
  - 现有事件覆盖 callback、executor、publisher/subscription、rmw 等多层
  - 目标是运行时行为观测，不是 fuzzing feedback

官方来源：
- `ros2_tracing` README
  - <https://github.com/ros2/ros2_tracing/blob/rolling/README.md>
- `ros2_tracing` design
  - <https://github.com/ros2/ros2_tracing/blob/rolling/doc/design_ros_2.md>

## 3. 论文为什么还要扩展
- 论文原文不是“直接用原版 ros2_tracing 就够了”。
- 论文公开说法是：
  - 扩展 `Ros2Trace`
  - 在 `RCLCPP` / `RCL` 增加更多 tracepoint
  - 扩展 tracer 记录：
    - `buffer size`
    - `publish timestamp`
    - `subscribe timestamp`
  - 用 shared memory 实时导出

本地论文依据：
- [docs/paper/r2d2.pdf](/home/ocsar/ROS/my_r2d2/docs/paper/r2d2.pdf)
- [docs/study/paper_summary.md](/home/ocsar/ROS/my_r2d2/docs/study/paper_summary.md)

## 4. 当前官方现成 tracepoint 的边界
- 当前官方链路能稳定拿到：
  - callback registration
  - `rclcpp_executor_execute`
  - `callback_start/end`
- 当前官方链路不能直接给出论文 message half 需要的完整字段组合：
  - `buffer_size`
  - `pub_timestamp`
  - `sub_timestamp`

这也是我们之前只用内建事件时 `msg_trace` 为空的原因。

## 5. 本机源码现状
- 本机有已安装头文件与库：
  - `/opt/ros/jazzy/include/tracetools`
  - `/opt/ros/jazzy/include/rcl`
  - `/opt/ros/jazzy/include/rclcpp`
- 本机没有可直接 patch 的源码树：
  - 没有本地 `ros2_tracing`
  - 没有本地 `rcl`
  - 没有本地 `rclcpp`

因此，真正的源码扩展必须先导入 overlay 源码工作区。

## 6. 最小 overlay 包集合
- `ros2_tracing`
  - 提供 `tracetools` tracepoint 定义与接口
- `rcl`
  - 负责 `rcl_take`、底层 subscription/service/timer 初始化
- `rclcpp`
  - 负责 callback registration、executor execute、callback 生命周期

当前仓库已新增导入清单：
- [config/ros_runtime_overlay.repos](/home/ocsar/ROS/my_r2d2/config/ros_runtime_overlay.repos)

## 7. 建议的扩展策略
- 不直接改现有 `ros2:*` 事件签名。
- 新增 `r2d2_*` tracepoint，避免破坏现有 ABI 与官方工具兼容性。
- 调用点同时保留原官方事件，再补发新的 `r2d2_*` 事件。

建议新增事件集合：
- `r2d2_callback_registration`
  - `callback_type`
  - `rclcpp_handler`
  - `rcl_handler`
  - `callback_name`
  - `callback_namespace`
- `r2d2_executor_execute`
  - `rclcpp_handler`
  - `invoke_timestamp`
- `r2d2_callback_start`
  - `rclcpp_handler`
  - `start_timestamp`
- `r2d2_callback_end`
  - `rclcpp_handler`
  - `end_timestamp`
- `r2d2_rcl_take`
  - `rcl_handler`
  - `buffer_size`
  - `pub_timestamp`
  - `sub_timestamp`

## 8. 预期源码落点
- `ros2_tracing/tracetools`
  - `include/tracetools/tp_call.h`
  - `include/tracetools/tracetools.h`
- `rcl`
  - `src/rcl/subscription.c`
  - 以及 callback registration 相关初始化路径
- `rclcpp`
  - `src/rclcpp/executor.cpp`
  - callback registration / subscription / service / timer 相关源文件

## 9. 本次真实落地
- overlay 源码已导入并参与构建：
  - `overlay_ws/src/ros2_tracing`
  - `overlay_ws/src/rcl`
  - `overlay_ws/src/rclcpp`
- 已在源码层补上 live 所需运行时事件：
  - `rcl` 注册 `rcl_handler -> callback name`
  - `rclcpp` 注册 `rclcpp_handler -> rcl_handler`
  - `executor_execute`
  - `callback_start/end`
  - `rcl_take(buffer_size, pub_timestamp, sub_timestamp)`
- `LD_PRELOAD` runtime interpose 已移除；当前 live 链路改为：
  - `overlay_ws/install/setup.bash`
  - `R2D2_TRACER_MODE=runtime`
  - `/dev/shm/r2d2_nav2`

## 10. 这次踩到的真实问题
- 只重建 `ros2_tracing/rcl/rclcpp` 不够。
- 因为 subscription/service/timer 的注册 hook 落在模板头文件里，目标应用包也必须在 overlay 头文件上重编。
- 本仓目标是 `nav2_costmap_2d`，所以额外补了第二阶段重建：
  - 先 build overlay runtime
  - 再在已 source overlay 的环境里重建 `nav2_costmap_2d`

## 11. benchmark 全 invalid 的根因
- overlay 重建后，真实 runtime event 已经齐全，`/scan`、`/map`、`/clear_*`、`/get_costmap` 都进入了 complete callback 集合。
- 但 Rust 侧 `CallbackTrace::valid_for_state_analysis()` 仍把“系统里存在未补全但本轮未参与的 registration”当成整轮 invalid。
- 这会导致：
  - `complete callbacks` 已恢复
  - benchmark 仍然 `analyzed=0`
  - 但失败原因其实是 `invalid>0`，不是 empty trace
- 修复方式：
  - 保留 `incomplete_registrations` 作为诊断项
  - 不再把它作为全局致死条件
  - 真正参与本轮但未补全的 handler，仍由 `unknown_handlers` 拦截

## 12. 实际验证证据
- 修复前真实 run：
  - `startup: ready barrier passed; 57 registration records, 25 complete callbacks`
  - `benchmark: elapsed=61s rounds=29 analyzed=0 empty=0 invalid=29`
- 修复后真实 run：
  - `startup: ready barrier passed; 57 registration records, 25 complete callbacks`
  - `benchmark: rounds=4 analyzed=4 empty=0 invalid=0 edges=1 callbacks=4`
- 额外 live ring 复核：
  - runtime shared memory 中可还原 `351` 条 `call_trace`
  - 可还原 `329` 条 `msg_trace`
  - 说明问题不在 tracer 缺事件，而在 Rust 侧 round 有效性口径

## 13. 当前结论
- 插桩层级已经从应用层 hook 换到 `ros2_tracing + rcl + rclcpp` 源码 overlay。
- 真实 Nav2 costmap live benchmark 已经能产出非零 `analyzed_traces`。
- `LD_PRELOAD` 路线已经退出真实闭环；live path 只依赖 overlay runtime + shared memory。

## 14. coverage 实时反馈这次为什么会失败
- 根因 1：`scripts/build_overlay_ws.sh` 之前只给 overlay 包和 `nav2_costmap_2d` 传
  `-DBUILD_TESTING=OFF`，没有继承 `build_nav2_ws.sh --coverage/--tsan` 的
  `--coverage` / `COVERAGE_RUN` / linker flags。
- 结果：
  - overlay 重建后 `nav2_costmap_2d` 又被普通 `RelWithDebInfo` 重编
  - `flags.make` / `link.txt` 里没有 `--coverage`
  - benchmark/fuzz 触发 `SIGUSR1` 后没有 `.gcda`
  - `lcov` 只会报 `no .gcda files found`
- 根因 2：`launch_stack.sh` 用的是 `setarch ... ros2 run nav2_costmap_2d nav2_costmap_2d`。
- 结果：
  - `/dev/shm/<name>.pid` 中记录的是真实 costmap 进程 PID
  - 但用户态观察到的前台进程是 `ros2 run`
  - 发 `SIGUSR1` 时日志里会出现 `[ros2run]: User defined signal 1`
  - 轮间 flush 可能把包装进程/整组栈提前打死

## 15. 本次覆盖率修正
- `scripts/build_overlay_ws.sh`
  - 读取 `nav2_ws/build/.r2d2_last_mode`
  - 继承 `plain / coverage / tsan` 三种模式
  - 用和 `build_nav2_ws.sh` 一致的 gcc/gcov/sanitizer CMake flags 重建
    `tracetools rcl rclcpp nav2_costmap_2d`
- `nav2_ws/launch_stack.sh`
  - 改为直启真实二进制：
    - `install/nav2_costmap_2d/lib/nav2_costmap_2d/nav2_costmap_2d`
  - `SIGUSR1` 现在直接命中安装了 `__gcov_dump()` handler 的目标进程
- `examples/nav2_costmap_e2e.rs`
  - benchmark baseline capture 后立即写
    `nav2_ws/results/coverage/status.json`
  - fuzz 每轮 capture 后覆盖写同一路径
  - 字段：
    - `phase`
    - `round`
    - `coverage_ok`
    - `branch_covered_total`
    - `branch_total`
    - `branch_covered_increase`

## 16. 现在的口径
- 覆盖率仍然只是反馈与观测，不参与论文 state oracle 判定。
- 真实判定链路仍然是：
  - payload -> 真实 ROS 执行 -> runtime trace -> callback/msg profile ->
    benchmark reference / global state -> crash/new state
- 这次改动只修正“真实闭环运行时如何稳定拿到每轮 lcov 反馈”，不改论文机制。
