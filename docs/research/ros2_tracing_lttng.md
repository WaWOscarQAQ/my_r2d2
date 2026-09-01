# ros2_tracing 论文扩展与本仓实现
日期：2026-08-25

## 1. 官方链路和论文链路不是一回事
- 官方 `ros2_tracing` / `Ros2Trace` 提供的是 ROS 2 runtime tracing 基座。
- 论文没有声称“直接用原版现成 tracepoint 就拿到了全部 callback trace 字段”。
- 论文明确写的是：
  - 在 `RCLCPP` 和 `RCL` 层手工增加更多 tracepoint
  - 扩展原 tracer，使其额外记录：
    - `message buffer size`
    - `publish timestamp`
    - `subscribe timestamp`
  - 用 shared memory 实时导出数据

本地论文依据：
- [docs/paper/r2d2.pdf](/home/ocsar/ROS/my_r2d2/docs/paper/r2d2.pdf)
- [docs/study/paper_summary.md](/home/ocsar/ROS/my_r2d2/docs/study/paper_summary.md)

## 2. 为什么纯官方内建 tracepoint 不够
- Jazzy 当前官方内建事件能稳定拿到：
  - callback registration
  - executor scheduling
  - callback start/end
- 但不能直接拿到论文 Figure 5 所需的完整 message 字段：
  - `buffer_size`
  - `pub_timestamp`
  - `sub_timestamp`
- 所以只靠 `LTTng + babeltrace` 的官方内建事件，`msg_trace` 会是空。

这不是猜测，是我们已经实测过的结果：
- 纯官方链路阶段，live run 的 `msgs=0`，`thr=[]`
- 切回论文式 runtime tracer 后，`msgs` 和 `thr` 恢复为非零

## 3. 当前仓库怎么按论文补
- 运行时层级：
  - 只在 `rclcpp/rcl` 层拦截
  - 不再使用 `nav2_costmap_2d` 应用层 hook
- 当前 live tracer 注入点：
  - `CallbackGroup::add_subscription`
  - `CallbackGroup::add_service`
  - `CallbackGroup::add_timer`
  - `NodeTimers::add_timer`
  - `Executor::execute_any_executable`
  - `rcl_take`
- 导出介质：
  - `/dev/shm/r2d2_nav2`
- 读取方式：
  - Rust `TraceReader` 直接按 shared-memory ABI drain ring buffer

## 4. 现在真实能拿到哪些数据
- registration side：
  - callback type
  - `rclcpp_handler`
  - `rcl_handler`
  - callback name
  - callback namespace
- runtime side：
  - `executor_execute` timestamp
  - `callback_start` timestamp
  - `callback_end` timestamp
  - `rcl_take.buffer_size`
  - `rcl_take.pub_timestamp`
  - `rcl_take.sub_timestamp`

这已经满足论文 Figure 5 里 message throughput 的直接计算条件：
```text
throughput = buffer_size / (sub_timestamp - pub_timestamp)
```

## 5. 本次真实验证
命令：
```bash
./scripts/build_nav2_ws.sh --clean
ROS_DOMAIN_ID=232 cargo run --example nav2_costmap_e2e -- --benchmark-seconds 5 --rounds 1 --seed-dir config/nav2_seeds
```

关键结果：
- clean build 成功：
  - `Summary: 7 packages finished [2min 4s]`
- startup：
  - `64 registration records`
  - `32 complete callbacks`
- benchmark：
  - `rounds=2 analyzed=2 empty=0 invalid=0`
- fuzz 第 1 轮：
  - `calls=2`
  - `msgs=1`
  - `thr=[3.03] MB/s`

结论：
- `msg_trace` 已恢复
- throughput 已重新进入 live 反馈链路

## 6. 现在还剩什么差距
- 当前做法是 `LD_PRELOAD` runtime interpose。
- 论文文字口径更接近“扩展运行时 tracer / tracepoint”，但没有公开完整补丁。
- 因此当前实现已经在机制上对齐论文：
  - 插桩层级对
  - 字段对
  - shared memory 导出对
- 但在“是否直接改 ROS 源码而不是 interpose”这一点上，仍属于实现策略差异。
