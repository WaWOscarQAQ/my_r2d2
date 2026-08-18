# R2D2 Rust core

按 `docs/plan/payload_generator_reproduction_plan.md` 落地了 payload generator 复现计划的 P1 至 P6，并按 `docs/plan/tracer_reproduction_plan.md` 落地了 tracer 复现计划的 T1 至 T5。

## 当前模块

- `interface_extractor`：interface 提取边界与递归类型树（`TypeNode`），只覆盖 topic 与 service。
- `payload`：`Payload` 数据模型、`ValueTree` 值树与 `Serializer` 序列化边界（`SimpleSerializer` 为 reproduction choice）。
- `payload_pool`：interesting payload pool，只允许 crash 或 new-state payload 入池。
- `mutation`：递归变异器（`Mutator`）与类型正确生成（`generate_value`）。
- `payload_generator`：每轮分支决策（空池生成 / 非空池变异）、`GeneratorConfig`，以及 `Sender`、`StateOracle` trait 边界。
- `trace_buffer`：shared memory reader，解析 C++ tracer 写入的 registration 与 runtime 环形缓冲（阶段 C/D 的 Rust 侧读取端）。

## tracer 模块（阶段 C/D，C++ 侧）

位于 `tracer/`：六个论文规定 tracer（`rclcpp_callback_init()`、`rcl_callback_init()`、`executor_execute()`、`callback_start()`、`callback_end()`、`rcl_take()`）、双 mutex 保护环形 shared memory buffer，以及 mock 事件源 `mock_writer`。

构建与 fixture 生成：

```text
cmake -S tracer -B tracer/build
cmake --build tracer/build
tracer/build/mock_writer fixture_golden --fixture tests/fixtures/trace_golden.bin --reg-capacity 8 --runtime-capacity 8 --cleanup
tracer/build/mock_writer fixture_overflow --fixture tests/fixtures/trace_overflow.bin --reg-capacity 4 --runtime-capacity 4 --overflow --cleanup
```

Rust 侧 `src/trace_buffer.rs` 读取 `/dev/shm/<name>` 或 fixture 镜像；`tests/trace_buffer.rs` 含 golden 解析、溢出与真实 C++ 往返测试（`tracer/build/mock_writer` 缺失时往返测试打印提示并跳过）。真实 ROS 插桩接入点见 `docs/plan/tracer_reproduction_plan.md` 第 6 节，待阶段 B 冻结源码 commit 后实施。

## 当前边界

- 论文明确描述：R2D2 提取 ROS interfaces，按 interface specification 生成 payload，并基于 data files 递归变异曾触发新状态的 payload；只有 crash 或 new-state payload 才进入 pool。
- 工程占位实现：尚未连接 ROS 2 graph，不解析 `.msg`、`.srv` 文件，不发送真实 topic/service 消息；`Sender` 与 `StateOracle` 只有 trait 定义，真实实现待阶段 H 接入。
- tracer 侧以独立模块与 mock 事件源验证 C++/Rust ABI，尚未对真实 rclcpp/rcl 源码插桩（依赖阶段 B 的 Humble/Rolling 源码构建）。
- 测试替身：mock 只存在于 `tests/`，不会进入正式库的公开 API。

## Reproduction choice 记录

论文未披露以下参数，当前实现的自选值全部标注为 reproduction choice，不得在复现报告中写成 paper setting：

| 项目 | 当前实现 | 配置位置 |
|---|---|---|
| interface 与 pool item 选择概率 | 均匀分布 | `interface_select_probability` 与 `pool_item_select_probability` 仅记录缺口，不参与计算 |
| 变异算子集合与权重 | 见 `OperatorWeights` / `OperatorsPerType` 默认值 | `mutation` 模块 |
| mutation energy 与递归深度 | 8 / 8 | `GeneratorConfig::mutation_energy` / `max_recursion_depth` |
| 数组长度分布 | 0 到 8 | `GeneratorConfig::array_len_range` |
| 各基础类型取值分布 | 有符号整数与浮点 -1000 到 1000，无符号整数 0 到 1000，字符串与字节序列长度 0 到 64 | `ValueRanges` |
| 序列化实现 | `SimpleSerializer` 自定义确定格式 | `payload` 模块 |
| 随机数 | 每轮以 `base_seed` 加轮次派生 `rng_seed` 重新播种 `StdRng` | `PayloadGenerator` |
| shm 布局与容量 | 单对象内 `SharedHeader` 加双 ring，默认 registration 1024 / runtime 4096 条 | `tracer` 模块 |
| 环形溢出行为 | 覆盖最旧记录并递增 `overflow_count` | `RingHeader` |
| 时间戳时钟源 | `CLOCK_MONOTONIC` | `tracer::now_ns()` |
| 记录来源区分 | `RegistrationSource`（Rclcpp / Rcl） | `trace_records.h` |
| 平台假设 | 小端、x86-64 下 `pthread_mutex_t` 为 40 字节（C++ static_assert 兜底）、对齐 8 字节读取按实践原子 | `trace_buffer` 模块 |

## 运行测试

```text
cargo test
```

后续接入真实 ROS 2 时，只需新增实现 `Extractor`、`Sender`、`StateOracle` 与 `Serializer` 的类型，并按 tracer 计划接入点对 rclcpp/rcl 源码插桩。
