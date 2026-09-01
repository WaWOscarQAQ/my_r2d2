# R2D2 复现合同（冻结版）

> 实现更新（2026-08-27）：本文冻结时记录的 source overlay/interpose 已被
> 非侵入式 LLVM 双层插桩取代；当前机制见
> `docs/plan/non_invasive_llvm_instrumentation.md`。论文口径边界仍以本文为准。

> 冻结日期：2026-08-23。
> 论文：Yuheng Shen et al., *Enhancing ROS System Fuzzing through Callback Tracing*, ISSTA 2024, DOI `10.1145/3650212.3652111`。
> 本地 PDF：`docs/paper/r2d2.pdf`，SHA-256 `12cf322e15f4ddad870522bc2704027f21a2aff329c5c9e3f5bcbe61f802cbee`。
>
> 本合同是 `r2d2_strict_reproduction_plan.md` 阶段 A 的落地：固定论文依据、记录环境事实、逐项区分 **paper setting** / **reproduction choice** / **unresolved gap**。任何后续实现与报告不得把后两栏写成第一栏。

## 1. 复现层级声明

按 `r2d2_strict_reproduction_plan.md` 第 6 节的层级定义，当前仓库只能声明：

- **L1 结构复现**：Rust core + C++ tracer、registration/runtime 双 ring buffer、论文六个 tracer 与字段、callback trace 数据结构齐全。
- **L2 功能复现（受限）**：当前只保留 full-stack Nav2 live 合并路线；live 输入面为
  67 个 topic/service/action/safe-parameter-profile binding，其中 34 个来自
  manifest、7 个来自 safe parameter profile、26 个来自 full-stack runtime extension。
  live 插桩已切到 `rclcpp/rcl` 运行时层，但仍不是论文四目标完整实验环境。

**不得声明** L3/L4：论文实验环境为 Ubuntu 22.04 + ROS 2 Humble/Rolling 源码构建 + 四个目标应用；本仓当前是在本机新版 ROS 环境上对 Nav2 full-stack 做受限复现。

## 2. 环境事实（实测值）

| 项 | 值 |
|---|---|
| OS | Ubuntu 24.04.4 LTS (noble) |
| ROS | ROS 2 Jazzy 二进制（rclcpp 28.1.18，rcl 9.2.9），无源码树 |
| 目标应用 | Nav2 full-stack（`nav2_ws/src_lyrical/navigation2` 为 vendor 副本，**上游 commit 未记录，属缺口**） |
| 编译器 | gcc/g++ 13.3.0；clang 15.0.7；cmake 3.28.3 |
| Rust | rustc 1.96.0 |
| 构建模式 | `scripts/build_nav2_ws.sh` 的 plain / coverage(gcc+gcov) / tsan 三种口径 |
| 论文硬件 | 64-core AMD EPYC 7742 + Ubuntu 22.04 —— **本机不满足**，性能数据不可比 |

## 3. 三栏登记

### 3.1 Paper setting（论文明确，已逐节核对 PDF）

| 项 | 论文出处 | 本仓库实现 |
|---|---|---|
| 六个 tracer：rclcpp_callback_init / rcl_callback_init / executor_execute / callback_start / callback_end / rcl_take | §4.1.1 | `tracer/src/tracers.cpp` |
| 注册采集属性含 **namespace**、两层 handler、callback type | §4.1.1 | ABI v2 `RegistrationRecord`（本轮补齐 namespace） |
| runtime 采集：RCLCPP handler + invoke/start/end；RCL handler + buffer size + pub/sub 时间戳 | §4.1.1 | `RuntimeRecord` |
| Callback ID = Hash(callback name, callback type) | §4.1.2 Figure 5 | `src/callback_profile.rs`（namespace **不参与** ID） |
| Execution/Scheduling latency、Throughput = buffer size / Duration(pub, sub) | §4.1.2 Figure 5 | `profile_trace` |
| CallTrace/MsgTrace 保留重复元素与时序 | §4.1.2 | `Vec<CallbackLatency>` / `Vec<MessageLatency>` |
| 新状态三判据：新 execution sequence edge、latency 显著偏离、throughput 显著偏低 | §4.2.1 | `BenchmarkStateOracle` 只输出一个 active verdict：`new_state = 新 edge/callback/message 或 latency/throughput deviation`；阈值是本仓对“显著偏离”的显式复现参数 |
| benchmark 采样 2 小时 | §5.1 | `BenchmarkBuilder` 默认独立采样 7200 秒；也可加载已保存的 model 跳过重采样 |
| shm 初始化在 RCL 层初始化路径；每个 shm object 为带 mutex 的 circular buffer；实时写出、测试期间读取 | §4.3 | `tracer::init` + `RingBuffer`（live 路径由 `runtime_interpose.cpp` 在 `rcl` 初始化链上触发；历史 app-hook 阶段曾在 costmap main 初始化） |
| Rust core / C++ tracer 分工；真实 ROS 输入面；不使用 coverage 作 guidance | §4.3、§1.2 边界 | 成立；合并路线使用 topic/service/action/safe-parameter-profile 统一 fuzz 输入面；coverage attribution 只进入报告，不进入调度器 |

### 3.2 Reproduction choice（论文未披露，本仓库自选，已标注）

| 项 | 取值 | 位置 |
|---|---|---|
| Callback ID hash 算法 | FNV-1a 64，name ∥ 0x1f ∥ type | `src/callback_profile.rs` |
| shm 布局 | SharedHeader(48) + 双 RingHeader(64，含 40B mutex slot) + 记录数组 | `tracer/include/tracer/` |
| ring 容量 | registration 1024 / runtime 4096（`init` 参数可调） | `tracer/src/tracers.cpp` |
| overflow 行为 | 覆盖最旧并计 `overflow_count`；reader 侧计 `missed` | `circular_buffer.h`、`trace_buffer.rs` |
| RegistrationRecord ABI v2 | 232 字节：既有字段 + `callback_namespace_len`@160 + `callback_namespace[64]`@164（4B 尾部对齐）；flags bit1 = namespace 截断 | `trace_records.h` |
| live 轮次分段 | harness 每轮发送前先 drain 当前 runtime 游标，发送后持续读取直到事件静默收敛；parameter round restore 后重复同一收敛过程。不再依赖仓库自定义 `RoundBoundary` marker 作为 live 判定边界。 | `examples/nav2_costmap_e2e.rs` |
| 时钟源 | live runtime tracer 用 `CLOCK_MONOTONIC`；历史 app-hook fallback 仍用 ROS system time | `tracers.cpp`、`runtime_interpose.cpp`、`nav2_hooks.hpp` |
| publish timestamp 来源 | live runtime tracer 优先取 `rmw_message_info_t.source_timestamp`；fallback 为消息 `header.stamp` | `runtime_interpose.cpp`、`nav2_hooks.hpp` |
| throughput 单位 | bytes/ns；`sub <= pub` 跳过并计数 | `callback_profile.rs` |
| handler 关联 | live runtime tracer 记录真实 `rclcpp` object / `rcl` handle；历史 app-hook fallback 曾用同一 nav2 指针近似 | `runtime_interpose.cpp`、`nav2_hooks.hpp` |
| 新状态阈值 | `latency_factor`、`throughput_floor` 可配乘数；驱动统一 callback-trace oracle，不能写成论文公开公式 | `state_oracle.rs` |
| 输入归因 | `input_kind` 与 `input_source` 只写入日志/summary，用于报告阶段拆分覆盖增长，不进入 `PayloadGenerator` 或 `BenchmarkStateOracle` | `examples/nav2_costmap_e2e.rs` |
| live 启动屏障 | 进入 benchmark 前必须等待 full-stack Nav2 关键 lifecycle 节点进入 `active`，必要时只通过 lifecycle manager 做 STARTUP 恢复，并等待 runtime registration trace 收敛；未通过则直接退出，不提前发 fuzz payload | `examples/nav2_costmap_e2e.rs`、`nav2_ws/launch_nav2_full_stack.sh` |
| readiness/warm-up 边界 | 只允许 lifecycle active、topic/service 可见性等待、costmap 初始化等待，以及基础 pose/map/scan 合法上下文维持；不得写 coverage-path scripting、每轮固定 goal 或覆盖率驱动 warm-up | `examples/nav2_costmap_e2e.rs`、`config/nav2_sequences/` |
| parameter 边界 | 先落 safe parameter profile：已知动态且低风险参数、生成值 clamp、轮后 restore；restore 失败时判本轮 stack unhealthy 并重启 | `examples/nav2_costmap_e2e.rs`、`docs/research/nav2_costmap_parameter_interfaces.md` |
| live stack 重启 | parameter restore / 执行失败时，按进程组发送 `SIGTERM`/`SIGKILL` 后重新走 ready barrier；要求旧 `/costmap` 不得残留在 ROS graph 中 | `examples/nav2_costmap_e2e.rs` |
| 覆盖口径 | gcc+gcov + lcov（RQ2 评估近似，非 guidance） | `scripts/build_nav2_ws.sh` |

### 3.3 Unresolved gap（论文未披露且本仓库未定/未实现）

1. R2D2 官方源码与 artifact（论文未给 URL）。
2. Humble/Rolling 精确 commit；四个目标应用的 commit 与场景配置。
3. 真实 rclcpp/rcl 插桩点、参数 ABI、事件格式在 Jazzy 二进制 interposer 上已落地；与论文源码 patch 版是否逐点等价仍属 gap。
4. `rmw_message_info_t.source_timestamp` 在本机 Jazzy 上已用于 live runtime publish timestamp；不同 rmw 实现下的一致性仍未验证。
5. "significant deviation" 统计公式、阈值、样本数、outlier 处理。
6. payload 生成/变异算子、概率、递归深度、选择策略。
7. SanitizerCoverage 模式与跨组件 coverage 合并方式。
8. nav2 vendor 副本的上游 commit（本仓库自身记录缺口）。
9. namespace 在论文 ID 公式中的唯一性角色：Figure 5 明确只哈希 (name, type)，跨 namespace 同名同 type 会撞 ID；本仓库只做检测与拦截（`callback_id_collisions`），不改公式。

## 4. 本轮（B3）决策记录

- **C1 namespace**：论文 §4.1.1 明确把 namespace 列为注册属性，故采集进 `RegistrationRecord`（ABI v2，version 1→2）；Figure 5 明确 ID 只含 (name, type)，故 namespace 不进 ID；registry 检测同 (name, type) 跨 namespace 的碰撞并使 trace 失去反馈资格。截断 namespace 与截断 name 同政策（不计完整注册）。
- **C2 publish timestamp**：不新增论文未描述的 pub 侧 tracer。应用层维持 `header.stamp` 来源并记录时钟域限制（ROS system time；跨进程与 sim time 下可比性有限）。未来 rcl 层候选机制登记为 gap 4。
- **C3 live 轮次分段**：live harness 改为“发送前清空当前游标 + 发送后持续 drain 到静默收敛”的纯运行时分段，不再把仓库自定义 `RoundBoundary` marker 当作真实闭环判定边界。
- **C8 oracle 合并**：`state_oracle` 只输出一个 active verdict。coverage attribution 改由 `input_kind` / `input_source` 在报告阶段拆分，不进入调度器。
- **C7 并发验证**：新增 `mock_writer --stress` 与 Rust 并发 drain 测试，证明无锁读在高事件量 + 持续 overflow 下"只丢不错"（事件数 + missed 与写入总数严格对账，handler 值域合法）。

## 5. 合并路线

当前只保留一条路线：在本机新版 ROS 环境上运行 full-stack Nav2，以论文方法为核心
（dry run/interface extraction、benchmark reference、callback-trace oracle、new-state
pool），并在同一条 harness 中扩展 topic/service/action/safe-parameter-profile 输入面。
版本差异和输入来源差异只进入报告说明，不再作为运行时调度 mode。

插桩 ABI 以本合同的 ABI v2 为准；冻结前不得再改 record 布局而不升 version。
