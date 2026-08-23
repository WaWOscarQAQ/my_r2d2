# Callback Trace Profile 问题报告（论文对照 + 实际运行验证）

> 分支：`callback-trace-profile` @ `be7beec`（feat: 实现 Callback Trace Profile 并补齐测试覆盖）。
> 验证日期：2026-08-19。
> 依据：Yuheng Shen et al., *Enhancing ROS System Fuzzing through Callback Tracing*, ISSTA 2024，DOI: `10.1145/3650212.3652111`，第 4.1.1 节（Runtime Temporal Behaviors Tracing）、第 4.1.2 节（Callback Trace Profile）、第 4.3 节（Implementation），以及 `r2d2_strict_reproduction_plan.md` 阶段 E 验收条件、`tracer_reproduction_plan.md`。
> 验证方法：完整跑通 `cargo test` / `cargo clippy` / `cmake --build tracer` / `mock_writer` fixture 再生对比，另编写 3 个临时探针测试（已删除，结论记录于第 2 节）与 1 个临时 C++ 探针程序。

## 1. 结论摘要

- 现有测试全部通过（83 个），golden/overflow fixture 可再生且与仓库内文件 byte-identical，C++ 侧布局 static_assert 全部成立，clippy 仅风格警告。
- 但对照论文并经针对性运行，发现 **3 个已证实的运行时缺陷**（P1–P3，均有探针输出为证）与 **7 个论文一致性缺口**（C1–C7）。
- P1–P3 会直接污染论文 §4.2.1 的 latency/throughput 新状态判据，建议在阶段 F 实施前修复；C1/C2 需要在阶段 B 冻结源码 commit 前定案。

## 2. 已证实缺陷（有运行证据）

### P1：runtime ring 溢出导致 profile 静默错配、钳零与丢数据

**现象（探针输出）**：模拟容量 4 的 runtime ring 溢出、`execute` 记录被覆盖后的事件流 `[start1, execute2, start2, end1, end2]`（真实两次执行各应产出 `{sched:100, exec:100}`）：

```text
latencies after mispairing: [CallbackLatency { callback_id: ..., execution_latency: 0, scheduling_latency: 100 }]
```

- 只产出 1 条记录（应 2 条）：execution#1 被静默丢弃。
- `end1`（ts=300）错配 `start2`（ts=1100），`300 - 1100` 为负，被 `saturating_sub` 钳成 `execution_latency = 0`——一条错误数据被当作合法 latency 输出。

**第二个探针**：`execute` 丢失但 `start`/`end` 均在时，`profile_trace` 输出 `call_trace len = 0`——execution latency 本可完整计算，却整条丢弃。

**根因**：

- `trace_buffer.rs` 的 `drain_*()` 已返回 `missed`/`overflow_count`，但 `profile_trace(infos, runtime)` 只接收事件列表，profile 层无法感知数据缺口。
- 栈配对（`pending_invoke`/`pending_start`）在丢记录时会跨执行错配；`saturating_sub` 把负值静默钳为 0，掩盖了错配。

**影响**：论文 §4.2.1 以 latency 显著偏差作为新状态判据之一；错配/钳零/缺条目会直接污染阶段 F 的 Global Callback Latency 基准与判定。ROS 高事件量下 runtime ring（默认 4096 条）溢出并非罕见场景。

**建议**：

1. `profile_trace` 增加 `missed`/overflow 入参，存在缺口时在 `CallbackTrace` 上输出显式标记（如 `lossy: bool` 或按区间返回错误），阶段 F 据此把该轮 trace 判为不可信。
2. `start`/`end` 完整时应独立计算 execution latency；`scheduling_latency` 缺失时用 `Option<u64>` 表示 unknown，而不是丢弃整条。
3. 负值 latency 一律按数据异常处理（标记或丢弃并计数），不得钳为 0。

### P2：callback ID 跨批次 drain 不稳定

**现象（探针输出）**：增量 drain 场景——先 drain 到 RCLCPP 注册记录时，回调 ID 为 `0xef14f2556cafa36e`；随后 RCL 记录到达（带来 name），同一回调 ID 变为 `0x45846e760df3b401`：

```text
stable across drains: false
```

**根因**：`Callback ID = Hash(name, type)`，而 name 来自另一条注册记录。`build_callback_infos` 每次按当前已 drain 的记录重算 hash；RCL 记录未到时 name 为空串，空串参与了 hash。

**影响**：

- 违反阶段 E 验收条件“callback ID 在同一实验配置下保持稳定”。
- 论文规定 R2D2 在测试期间实时读取 buffer（§4.1.1、§4.3）；流式使用下同一回调在不同时刻 ID 不同，阶段 F 的 Global Callback Graph 与 latency 键会分裂成多个伪回调。
- 现有测试之所以全绿，是因为测试都是一次性 drain 全部注册记录，未覆盖流式读取。

**建议**：

1. 不采用 `rclcpp_handler` 派生 ID。论文 Figure 5 明确规定 Callback ID 由 callback name 与 callback type 哈希得到，handler 只用于关联，改用地址会偏离论文。
2. profile 层增加跨 drain 保留状态的注册表：分别缓存 RCLCPP 与 RCL 记录，只有 name、type、RCLCPP handler、RCL handler 全部就绪时才发布 `CallbackInfo`，不完整记录不得产生临时 ID。
3. 对同一 handler 的冲突注册、registration ring 丢失和超长名称截断进行显式计数；存在这些情况时，后续 trace 不得进入论文 §4.2.1 的 Global Callback Graph 或 Global Callback Latency。

### P3：C++ tracer 抛异常，真实插桩会炸掉 ROS 进程

**现象（探针输出）**：`rcl_callback_init()` 传入 200 字节 name：

```text
exception thrown inside tracer: callback name exceeds record capacity
```

另有 `require_init()`：任何 tracer 在未 `init()` 时调用会抛 `std::logic_error`。

**影响**：论文规定 tracer 插在 RCLCPP/RCL 热路径（executor 调度、消息 take，§4.1.1）。这些函数在真实源码里多为 `noexcept` 上下文，异常会直接 `terminate` 掉 ROS 节点，等于插桩本身引入 crash——与论文“低开销、对系统透明”的定位相悖。

**建议**：tracer 永不抛异常：

- name 超长：截断到 `kCallbackNameCapacity` 并置 truncate 标志（或直接丢弃该字段）。
- 未 init：静默 no-op 或原子 early-out（真实插桩时 `init()` 在 RCL 层初始化路径执行，先于一切回调，但防御仍必要）。

## 3. 论文一致性缺口（对照 §4.1.1 / §4.1.2 / §4.3）

### C1：namespace 缺失（代码注释已标注，未补齐）

论文 §4.1.1 明确把 namespace 列为注册属性之一。当前 `RegistrationEvent` 不携带 namespace，`CallbackInfo` 亦无。后果：两个节点各有一个 `/cmd_vel_callback`（相同 name + type）会 hash 成同一 ID。补齐需同步修改 C++ record 布局与 Rust 解析，属阶段 B/C 的 ABI 变更，须在冻结 Humble/Rolling 插桩点之前定案。

### C2：`rcl_take` 的 publish timestamp 来源未定义

论文要求 `rcl_take()` 记录 “the timestamps associated with the publishing and subscribing activities”。publish 时间戳发生在**发布侧**，当前六个 tracer 中没有任何 pub 侧 tracer，也没有把 pub 记录与 take 记录关联的 message key；`rcl_take(pub_timestamp)` 的 pub 时间只能由调用方凭空传入（mock 用合成值）。阶段 D 验收条件“每条被观测消息都能恢复 publish/subscribe 时间关系”目前只对 mock 成立。需要在阶段 B 定案：是增加 pub 侧 tracer，还是在消息内携带 publish 时间戳，还是接受该字段为不可得值。

### C3：trace 无 payload 轮次边界

论文在每个 payload 执行后生成“当前 callback trace”（§4.2.1）。当前 runtime ring 无轮次/迭代标记，`CallbackTrace` 无 epoch 字段。阶段 F 只能靠 drain 游标快照自行分段，且必须处理事件跨轮次归属与溢出丢事件（与 P1 叠加）。建议在阶段 F 设计时同步确定分段协议（如 payload id 写入 ring 或发送前后各一次边界事件）。

### C4：throughput 单位与跳过规则

论文公式 `Throughput = buffer size / (sub - pub)` 未定义单位；当前实现为 bytes/ns。`sub <= pub` 时跳过该条，属 reproduction choice（代码已标注）。阶段 F 用吞吐偏离判新状态时，需明确基准单位与跳过条目的计数方式，避免把“跳过”误计为“低吞吐”。

### C5：重复注册记录 last-wins

`build_callback_infos` 用 `HashMap` 插入，同一 `rcl_handler` 的旧/新记录静默覆盖。节点重建回调（destroy/create 复用地址）时，旧运行时事件会错配到新档案。建议注册层检测 handler 复用并计数（或按 epoch 分段）。

### C6：注册与运行时事件的时序假设未强制

`profile_trace` 用 `rclcpp_handler`/`rcl_handler` 查 ID；若注册事件 drain 晚于运行时事件（读者游标以两个 ring 独立推进），对应 latency 被静默丢弃。与 P2 同源，需要明确定义“先注册后运行”的读取协议。

### C7：live 往返测试无并发读写覆盖

`tests/trace_buffer.rs` 的 live 往返测试在 writer 进程退出后才打开 reader；无锁读在 writer 绕环覆盖读者起点槽位时理论上可读到 torn record（计划 4.3 已声明该取舍，但无测试兜底）。建议补充并发压力测试（writer 持续写、reader 持续 drain，断言只出现“丢失”而不出现“损坏”）。

## 4. 其他运行结果（改动前基线，记录备查）

- `cargo test`：83 通过、0 失败（callback_profile 11、trace_buffer 4、payload 18、mutation 20、payload_generator 8、interface_extractor 4、interface_file_extractor 18）。
- fixture 再生：`mock_writer` 重新生成的 `trace_golden.bin` 与 `trace_overflow.bin` 与仓库内文件 byte-identical。
- `cmake --build tracer`：通过，`sizeof`/原子性 static_assert 全部成立。
- `cargo clippy --all-targets`：仅风格警告（collapsible if、可省略 lifetime、`Default::default()` 后字段赋值、复杂类型建议拆分），无正确性问题。
- 环境：ROS 2 Jazzy 二进制安装（无 Humble/Rolling 源码树），真实插桩接入点仍待阶段 B。

## 5. 处理建议优先级

| 编号 | 类型 | 建议时机 |
|---|---|---|
| P1、P2 | 运行时缺陷 | 阶段 F 实施前必须修复，否则新状态判据建立在错误数据上 |
| P3 | 运行时缺陷 | 阶段 B 插桩前必须修复，否则插桩自身引入 crash |
| C1、C2 | 论文一致性 | 阶段 B 冻结源码 commit 前定案（涉及 ABI 与 tracer 数量） |
| C3 | 设计缺口 | 阶段 F 设计时一并确定分段协议 |
| C4–C7 | 一致性/测试缺口 | 随各自阶段补齐，低阻塞 |

## 6. 本轮改进计划（论文一致性约束下）

### 6.1 P1：把数据质量纳入 Callback Trace Profile

改动位置为 `src/callback_profile.rs`、`tests/callback_profile.rs`，并复用 `src/trace_buffer.rs` 已有的 drain 丢失计数。

1. profile 入口直接接收带 `missed` 信息的 runtime drain，避免调用方只传事件数组而遗失溢出事实。
2. `CallbackTrace` 增加质量诊断，至少记录 registration/runtime 丢失数、时间戳逆序数、未配对事件数、未知 handler 数和无效 message duration 数；提供“能否用于状态分析”的统一判定。
3. `CallbackStart` 与 `CallbackEnd` 独立配对。只要二者完整且时间顺序有效，就保留论文定义的 execution latency；缺少 invoke 时 scheduling latency 记为 unknown，不再丢弃整次 callback 执行。
4. 所有 duration 使用有序检查后再相减；时间戳逆序时丢弃受污染的度量并计数，不再用钳零把异常伪装成合法零延迟。
5. runtime ring 出现丢失时仍可保留原始可解释度量供诊断，但整轮 trace 标记为不可用于 Global Callback Graph 与 Global Callback Latency，防止错误反馈进入论文 §4.2.1 的引导闭环。

### 6.2 P2、C5、C6：建立论文定义的有状态 Callback Registry

改动位置为 `src/callback_profile.rs` 与 `tests/callback_profile.rs`。

1. 新增跨 drain 累积注册记录的 registry，按 RCL handler 关联两层记录；只有论文 Figure 5 所需的 name、type 与两个 handler 完整时才构造 `CallbackInfo`。
2. ID 继续严格采用 `Hash(callback name, callback type)`；FNV-1a 64 位仍是论文未披露 hash 算法下的 reproduction choice，不改为 handler 地址。
3. 已发布条目的 ID 不因后续不完整批次变化。同一 handler 出现内容冲突时保留首个完整定义、记录冲突并使 trace 失去反馈资格，避免 last-wins 静默改写历史含义。
4. registration ring 丢失或存在不完整注册时，将质量问题传递给 callback trace；运行时事件找不到完整注册条目时显式计数，不再静默忽略。
5. 保留一次性 `build_callback_infos` 便捷入口，但它同样只返回完整条目；实时读取路径以有状态 registry 为准。

### 6.3 P3：使热路径 tracer 失败安全

改动位置为 `tracer/include/tracer/trace_records.h`、`tracer/include/tracer/tracers.h`、`tracer/src/tracers.cpp` 及相应测试。

1. 六个论文规定的注册/runtime tracer 全部改为不抛异常；shared memory 尚未初始化时直接 no-op，避免插桩改变 ROS 进程控制流。
2. callback name 超过定长 record 容量时截断存储并写入明确的 truncated 标志。Rust reader 将该标志解析出来，registry 不使用截断名称生成论文 Callback ID，而是把该注册保留为不完整诊断项。
3. `init()` 仍可在 RCL 初始化阶段报告资源创建错误，因为静默忽略初始化失败会让整次 fuzzing 产生无反馈假象；“热路径不抛异常”与“初始化失败可见”分开处理。
4. 不在本轮添加论文未描述的额外 publish tracer、payload boundary tracer 或自选 namespace ABI；这些项目继续受 C1–C3 的源码版本冻结与论文信息缺口约束。

### 6.4 验证与验收

1. 新增 P1 原始探针对应的回归测试：runtime 丢失显式标记、缺 invoke 仍保留 execution latency、逆序时间戳不产生零延迟。
2. 新增 P2 流式注册测试：第一批只有 RCLCPP 记录时不生成 ID，第二批补齐 RCL 记录后只生成一个稳定 ID。
3. 新增 P3 C++ 回归探针：未初始化调用六个 tracer 不抛异常，超长 name 不抛异常且携带 truncated 标志。
4. 完整运行 Rust 测试、Clippy、C++ 构建与 CTest；重新生成 golden/overflow fixture 并确认 ABI 兼容或同步更新 fixture。
5. 只有无数据丢失、无注册冲突、无截断注册、无时间异常、无未知 handler 和无未配对事件的 trace 才能进入后续论文状态判定。

本报告同时作为本轮实施计划；实际完成项与验证结果应在代码修改后回填。

## 7. 本轮实施结果

### 7.1 已完成

1. P1 已修复：`profile_trace` 直接接收 `RuntimeDrain`，`CallbackTrace` 新增 `lossy` 与质量诊断；时间戳逆序不再钳零，缺 invoke 时仍保留 execution latency，并把 scheduling latency 标为 unknown。
2. P2 已按论文约束修复：新增跨 drain 的 `CallbackRegistry`，不完整注册不生成 ID；补齐 name 与 type 后才按论文 Figure 5 的 `Hash(callback name, callback type)` 生成单一稳定 ID。
3. C5/C6 已做保守防护：同一 RCL handler 的冲突注册采用 first-wins 并显式计数，未知 handler、不完整注册和 registration ring 丢失都会使 trace 失去状态反馈资格。
4. P3 已修复：六个热路径 tracer 均为 `noexcept`，未初始化时 no-op；超长 callback name 截断并置标志，Rust registry 不用截断名称生成 callback ID。`init()` 的资源错误仍保持可见。
5. C++ record 总大小与既有字段偏移未改变；原 padding 位改作 flags。重新生成的 golden 与 overflow fixture 均与仓库文件 byte-identical。

### 7.2 验证结果

- `cargo test`：86 通过、0 失败，其中 callback profile 回归测试由 11 个增至 14 个。
- `cargo clippy --all-targets`：通过；只报告 6 个既有非本轮文件的风格警告，本轮新增代码无 Clippy 警告。
- `cmake --build tracer/build`：通过。
- `ctest --test-dir tracer/build --output-on-failure`：1 个 tracer safety test 通过、0 失败。
- fixture 再生对比：golden 与 overflow 两份文件均 byte-identical；SHA-256 分别为 `7eb6b5ee081c2dbc9f0cb4c4af235a277b58abc15ca91591c38482fd2814e129` 与 `222fdb6ed920e9c8df9e1c46338a0b94e67e8627d1faf7e96453b5992f648805`。

### 7.3 本轮未改动及原因

- C1 namespace：论文把 namespace 列为采集属性，但 Figure 5 又只规定 name 与 type 参与 ID；其精确 ABI 与唯一性角色不明确，且需要真实 Humble/Rolling 插桩点，暂不自创字段语义。
- C2 publish timestamp 来源：论文没有公开发布侧关联机制或 message key；在冻结 ROS 源码和确认数据来源前，不添加论文未描述的 tracer。
- C3 payload 边界：论文要求每个 payload 后分析当前 trace，但没有披露边界事件格式；留待阶段 F 的发送器与 Feedback Controller 同步设计。
- C4 throughput 单位：继续标为 bytes/ns reproduction choice，所有无效 duration 已显式计数。
- C7 并发读取：本轮未改变 reader 的无锁实现；它仍是下一项高优先级验证工作，因为论文 §4.3 明确强调 mutex 提供线程与内存安全。真实实时接入前应改为读写双方共享同一进程间 mutex，或用等价且经证明的快照协议。

## 8. 第二轮回填（2026-08-22，ABI v2）

### 8.1 本轮关闭的缺口

- **C1 namespace 已补齐**：论文 §4.1.1 明确把 namespace 列为注册属性。`RegistrationRecord` 追加 `callback_namespace_len`@160 与 `callback_namespace[64]`@164（sizeof 160→232，shm version 1→2），由 `rcl_callback_init(name, namespace, handler)` 携带；nav2 hooks 在各调用点传 `node->get_namespace()`。Figure 5 明确 ID 只哈希 (name, type)，namespace 不进 ID；registry 新增 `callback_id_collisions` 诊断：同 (name, type) 跨 namespace 时计数并使 trace 失去反馈资格。namespace 截断与 name 截断同政策（`truncated_callback_namespaces`，不计完整注册）。
- **C3 payload 轮次边界已定义**：`RuntimeEventType::RoundBoundary=4`，record 56 字节布局不变（原 offset 4 padding 命名 `aux` 存 round id）。新增 `tracer::attach()` 与 `round_marker` CLI，harness（`nav2_costmap_e2e`）每轮 settle 300ms 后写 marker 并按 marker 分段，straggler 事件带入下一轮；marker 缺失时回退游标语义。`end_to_end` mock 轮经 `--mark-round` 演练同一路径。profile 层把 marker 当 framing 跳过，不计诊断。
- **C7 并发读写已验证**：`mock_writer --stress --stress-rounds N --threads T`（小容量 ring 强制持续 overflow），Rust 测试 `concurrent_reader_never_sees_torn_records` 在 writer 双线程全速写入时并发 drain：解析零 Malformed，事件数 + missed 与写入总数（2×3000×4）严格对账，handler 值域合法——无锁读"只丢不错"。

### 8.2 C2 publish timestamp（合同化，不改 ABI）

不新增论文未描述的 pub 侧 tracer。应用层维持现状并在 `docs/plan/r2d2_reproduction_contract.md` 冻结：pub = 消息 `header.stamp`（缺失置 0、非正 duration 跳过），时钟域 ROS system time；未来 rcl 层候选 `rmw_message_info_t.source_timestamp` 登记为未验证 gap。

### 8.3 验证结果（本轮）

- `cargo test`：104 通过、0 失败（trace_buffer 6 含并发压力测试；callback_profile 18 含 namespace 截断/碰撞/marker 跳过测试）。
- `cargo clippy --all-targets`：仅剩 6 个既有风格警告（tests/mutation.rs、tests/interface_file_extractor.rs），本轮新增代码零警告。
- `cmake --build tracer/build` + `ctest`：通过，safety test 覆盖 round_boundary 未 init no-op 与 namespace 截断标志。
- fixture 重生成（golden 容量调整为 reg 8 / runtime 16 以容纳新增 marker 事件），再生对比 byte-identical；SHA-256：golden `2a3d2b3dd44c458cbaa23a304345543ea68db1c1ae6dfdedbd21e90a9b9d8905`，overflow `c377bc79483eb1dfdae68efd2d087bd3ed20d086b6091942d78b99b5198857b5`。
- `nav2_ws/src/r2d2_tracer` 已同步 ABI v2（headers/mock_writer/round_marker/safety test 与 standalone 一致；tracers.cpp 保留 nav2 侧覆盖率钩子差异）。
