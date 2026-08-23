# R2D2 复现合同（冻结版）

> 冻结日期：2026-08-23。
> 论文：Yuheng Shen et al., *Enhancing ROS System Fuzzing through Callback Tracing*, ISSTA 2024, DOI `10.1145/3650212.3652111`。
> 本地 PDF：`docs/paper/r2d2.pdf`，SHA-256 `12cf322e15f4ddad870522bc2704027f21a2aff329c5c9e3f5bcbe61f802cbee`。
>
> 本合同是 `r2d2_strict_reproduction_plan.md` 阶段 A 的落地：固定论文依据、记录环境事实、逐项区分 **paper setting** / **reproduction choice** / **unresolved gap**。任何后续实现与报告不得把后两栏写成第一栏。

## 1. 复现层级声明

按 `r2d2_strict_reproduction_plan.md` 第 6 节的层级定义，当前仓库只能声明：

- **L1 结构复现**：Rust core + C++ tracer、registration/runtime 双 ring buffer、论文六个 tracer 与字段、callback trace 数据结构齐全。
- **L2 功能复现（受限）**：仅 nav2_costmap_2d 单目标；live 输入面为当前
  `costmap` 真实 topic/service/parameter 矩阵（`/scan`、`/points`、`/map`、
  `/map_updates`、六个 costmap services 与 `/costmap` 动态参数）；live
  插桩已切到 `rclcpp/rcl` 运行时层，但仍是 Jazzy 二进制上的单程序复现。

**不得声明** L3/L4：论文实验环境为 Ubuntu 22.04 + ROS 2 Humble/Rolling 源码构建 + 四个目标应用，本机为 Ubuntu 24.04 + Jazzy 二进制 + 单一 nav2 组件。

## 2. 环境事实（实测值）

| 项 | 值 |
|---|---|
| OS | Ubuntu 24.04.4 LTS (noble) |
| ROS | ROS 2 Jazzy 二进制（rclcpp 28.1.18，rcl 9.2.9），无源码树 |
| 目标应用 | nav2_costmap_2d 1.3.12（`nav2_ws/src/navigation2` 为 vendor 副本，**上游 commit 未记录，属缺口**） |
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
| 新状态三判据：新 execution sequence edge、latency 显著偏离、throughput 显著偏低 | §4.2.1 | `BenchmarkStateOracle` 当前先输出 `paper-supported` 与 `jazzy-reproduction` 两套 verdict；前者只把论文已公开可直接落地的证据计入 active verdict，后者额外启用本仓阈值策略 |
| benchmark 采样 2 小时 | §5.1 | `BenchmarkBuilder` 默认独立采样 7200 秒；也可加载已保存的 model 跳过重采样 |
| shm 初始化在 RCL 层初始化路径；每个 shm object 为带 mutex 的 circular buffer；实时写出、测试期间读取 | §4.3 | `tracer::init` + `RingBuffer`（live 路径由 `runtime_interpose.cpp` 在 `rcl` 初始化链上触发；历史 app-hook 阶段曾在 costmap main 初始化） |
| Rust core / C++ tracer 分工；topic 与 service 输入面；不使用 coverage 作 guidance | §4.3、§1.2 边界 | 成立；parameter 为仓库扩展已落地，action 仍属于扩展目标，二者都不得写成论文原设 |

### 3.2 Reproduction choice（论文未披露，本仓库自选，已标注）

| 项 | 取值 | 位置 |
|---|---|---|
| Callback ID hash 算法 | FNV-1a 64，name ∥ 0x1f ∥ type | `src/callback_profile.rs` |
| shm 布局 | SharedHeader(48) + 双 RingHeader(64，含 40B mutex slot) + 记录数组 | `tracer/include/tracer/` |
| ring 容量 | registration 1024 / runtime 4096（`init` 参数可调） | `tracer/src/tracers.cpp` |
| overflow 行为 | 覆盖最旧并计 `overflow_count`；reader 侧计 `missed` | `circular_buffer.h`、`trace_buffer.rs` |
| RegistrationRecord ABI v2 | 232 字节：既有字段 + `callback_namespace_len`@160 + `callback_namespace[64]`@164（4B 尾部对齐）；flags bit1 = namespace 截断 | `trace_records.h` |
| 轮次边界协议 | `RuntimeEventType::RoundBoundary=4`，offset 4 的 `aux` 存 round id；harness 每轮 settle 300ms 后由 `round_marker` CLI 写入；reader 按 marker 分段，下一轮 preflight 主动丢弃边界后的残留事件；parameter round 额外执行 restore + 清理 | `tracers.cpp`、`round_marker.cpp`、`examples/nav2_costmap_e2e.rs` |
| 时钟源 | live runtime tracer 用 `CLOCK_MONOTONIC`；历史 app-hook fallback 仍用 ROS system time | `tracers.cpp`、`runtime_interpose.cpp`、`nav2_hooks.hpp` |
| publish timestamp 来源 | live runtime tracer 优先取 `rmw_message_info_t.source_timestamp`；fallback 为消息 `header.stamp` | `runtime_interpose.cpp`、`nav2_hooks.hpp` |
| throughput 单位 | bytes/ns；`sub <= pub` 跳过并计数 | `callback_profile.rs` |
| handler 关联 | live runtime tracer 记录真实 `rclcpp` object / `rcl` handle；历史 app-hook fallback 曾用同一 nav2 指针近似 | `runtime_interpose.cpp`、`nav2_hooks.hpp` |
| 新状态阈值 | `latency_factor`、`throughput_floor` 可配乘数；仅驱动 `jazzy-reproduction` verdict，不能冒充论文公开公式 | `state_oracle.rs` |
| oracle 口径模式 | `paper-supported` / `jazzy-reproduction`；两套 verdict 同时输出，payload pool 只跟随当前 mode 的 active verdict | `state_oracle.rs`、`examples/nav2_costmap_e2e.rs`、`examples/end_to_end.rs` |
| live 启动屏障 | 进入 benchmark 前必须等待 `/costmap` 出现在 ROS graph、lifecycle 进入 `active`、六个 costmap services 与 `/costmap` 参数服务 ready、registration trace 收敛；未通过则直接退出，不提前发 payload | `examples/nav2_costmap_e2e.rs`、`nav2_ws/launch_stack.sh` |
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
- **C3 轮次边界**：新增 RoundBoundary marker 事件（不改变 56 字节 record 布局，复用原 padding 为 `aux`）。harness 每轮 settle 后写 marker，reader 按 marker 分段；marker 丢失时回退游标语义。
- **C8 oracle 口径拆分**：`state_oracle` 同时输出 `paper-supported` 与 `jazzy-reproduction` verdict。前者当前只把新 edge 视为论文已公开且可直接判新的证据；latency/throughput 阈值只进入 `jazzy-reproduction` verdict，不再伪装成 paper verdict。
- **C7 并发验证**：新增 `mock_writer --stress` 与 Rust 并发 drain 测试，证明无锁读在高事件量 + 持续 overflow 下"只丢不错"（事件数 + missed 与写入总数严格对账，handler 值域合法）。

## 5. B2 环境分叉（下次实施前必须决策）

论文要求 Humble/Rolling 源码构建（Ubuntu 22.04）。本机只有 Jazzy 二进制。两条路线：

1. **Jazzy rcl/rclcpp 源码 overlay**：clone Jazzy 分支源码、按 `tracer_reproduction_plan.md` 第 6 节接入点插桩、overlay 到二进制 Jazzy 之上。属功能复现，版本与论文不一致，报告须标注。
2. **等待 Humble/Rolling 环境**（如容器或 Ubuntu 22.04 机器）：才谈得上向 L4 逼近。

两条路线的插桩 ABI 都以本合同的 ABI v2 为准；冻结前不得再改 record 布局而不升 version。
