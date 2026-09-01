# 论文式 Tracer 落地状态
日期：2026-08-25

> 历史状态：本文记录的 `LD_PRELOAD` interpose 已由非侵入式 LLVM 双层插桩
> 取代；当前方案见 `docs/plan/non_invasive_llvm_instrumentation.md`。

## 1. 目标
- 按论文实现 callback trace，不再停留在“只用当前官方内建 tracepoint”的口径。
- 保留插桩层级在 `rclcpp/rcl` 运行时层。
- 去掉应用层 hook 依赖。

## 2. 论文要求的关键机制
- 基于 `Ros2Trace/ros2_tracing` 思路扩展 tracer。
- 在 `RCLCPP` 和 `RCL` 层补 callback registration / runtime / message passing 信息。
- `rcl_take()` 需要提供：
  - `buffer_size`
  - `publish timestamp`
  - `subscribe timestamp`
- 运行时数据实时写入 shared memory，由 fuzzer 轮内读取。

## 3. 当前实现
- live backend 已切回论文式 shared-memory tracer。
- 真实链路现在是：
  1. `launch_stack.sh` 只给 `nav2_costmap_2d` 注入 `LD_PRELOAD=libr2d2_tracer_runtime.so`
  2. `r2d2_tracer_runtime` 在 `rclcpp/rcl` 层拦截：
     - `CallbackGroup::add_subscription/add_service/add_timer`
     - `NodeTimers::add_timer`
     - `Executor::execute_any_executable`
     - `rcl_take`
  3. C++ tracer 将 registration/runtime/message 数据写入 `/dev/shm/r2d2_nav2`
  4. Rust `TraceReader` 逐轮 drain shared memory
  5. `profile_trace` 计算 callback latency 与 message throughput
- 应用层 `nav2_costmap_2d` hook 已不再参与 live 路径。

## 4. 本次代码变更
- `src/trace_buffer.rs`
  - 从 `LTTng + babeltrace` reader 改回 shared-memory ABI reader
  - `TraceSession` 改为管理 `/dev/shm` 产物清理
- `nav2_ws/launch_stack.sh`
  - 恢复 `R2D2_SHM_PATH`
  - 只对 `nav2_costmap_2d` 注入 `LD_PRELOAD`
  - 避免 tracer 扩散到静态 TF 进程
- `scripts/build_nav2_ws.sh`
  - 重新构建 `r2d2_tracer`
- `config/r2d2_env.yaml`
  - tracing 配置键改回 `R2D2_SHM_PATH=/dev/shm/r2d2_nav2`
- `examples/nav2_costmap_e2e.rs`
  - startup 后清空 startup runtime 噪声
  - bootstrap `/map` 后再清空一次 runtime，避免污染 benchmark 首轮
  - coverage flush 改为读取 `/dev/shm/<name>.pid`
- `nav2_ws/src/r2d2_tracer/src/runtime_interpose.cpp`
  - subscription/service namespace 不再硬编码 `/`
  - 从真实 topic/service 全名推导 namespace
- `nav2_ws/src/r2d2_tracer/CMakeLists.txt`
  - 不再构建应用层 hooks 和 `round_marker`

## 5. 实际验证
- clean build：
  - `./scripts/build_nav2_ws.sh --clean`
  - 结果：`Summary: 7 packages finished [2min 4s]`
- live run：
  - `ROS_DOMAIN_ID=232 cargo run --example nav2_costmap_e2e -- --benchmark-seconds 5 --rounds 1 --seed-dir config/nav2_seeds`
- 关键输出：
  - startup：`64 registration records, 32 complete callbacks`
  - benchmark：`rounds=2 analyzed=2 empty=0 invalid=0`
  - fuzz 第 1 轮：
    - `calls=2`
    - `msgs=1`
    - `thr=[3.03] MB/s`
  - summary：
    - `invalid_traces=0`
    - `empty_rounds=0`

## 6. 现在能确认的结论
- `msg_trace` 已恢复，不再是空。
- message throughput 已重新进入 live `benchmark/oracle`。
- 当前实现已经比“只用 Jazzy 官方内建 tracepoint”更接近论文。

## 7. 仍然保留的差距
- 当前是 `LD_PRELOAD` interpose，不是直接修改 `/opt/ros` 中的 `rcl/rclcpp` 源码。
- 当前目标仍是 Jazzy 上的单个 `nav2_costmap_2d` 受限复现，不是论文四应用全集。
- 新状态里的显著偏离阈值公式仍未公开，`latency_factor/throughput_floor` 仍是 reproduction choice。
