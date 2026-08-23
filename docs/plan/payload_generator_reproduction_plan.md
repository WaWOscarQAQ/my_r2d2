# Payload Generator 复现计划

> 唯一技术依据：Yuheng Shen et al., *Enhancing ROS System Fuzzing through Callback Tracing*, ISSTA 2024, DOI: `10.1145/3650212.3652111`，第 4.2.2 节 Guided Payload Synthesis 与第 4.3 节 Implementation。
>
> 本文档与 `r2d2_strict_reproduction_plan.md` 阶段 G、阶段 H 保持一致。论文未披露的信息统一列为缺口并标注为 reproduction choice，不擅自补齐。

## 1. 依据与范围

### 1.1 依据

- 论文第 4.2.2 节：定义 dry run、空池生成、非空池变异、发送执行、crash/new-state 入池的完整行为。
- 论文第 4.3 节与 Figure 7：Payload Generator 是 Rust core 六组件之一，与 Interface Extractor、Payload Pool、Feedback Controller、Process Monitor、System Logger 并列。
- 论文第 5.1 节：interface extractor 提取全部 interface specification 后，payload generator 才能为 SUT 构造 payload。
- `r2d2_strict_reproduction_plan.md` 阶段 G：给出 6 条 payload 生成规则、5 条验收条件与未披露参数清单。
- `r2d2_strict_reproduction_plan.md` 阶段 H：规定 Rust 组件的每轮执行顺序。
- `r2d2_strict_reproduction_plan.md` 第 1.2 节：输入接口只覆盖 topic 与 service，不扩展 action；fuzzing guidance 只用 callback trace，不使用 code coverage。

### 1.2 范围

本计划覆盖：

- Payload Generator 组件本身。
- 它直接依赖的 Payload Pool 与 interface 模型扩展。

本计划不覆盖，但以 trait 边界形式预留接口：

- Feedback Collector / Controller（新状态判定）。
- Process Monitor（crash 检测）。
- System Logger。
- C++ tracers 与 shared memory。

## 2. 论文规定的行为基线

依据论文第 4.2.2 节，payload generator 必须复现以下行为，与 `r2d2_strict_reproduction_plan.md` 阶段 G 的 6 条规则一一对应：

1. dry run：启动系统但不发送任何输入，提取全部 interface specification，包括 topics、services、associated data files、message types 与 data formats。
2. payload pool 为空时，随机选择一个已提取的 interface，严格按 interface specification 生成 payload。
3. payload pool 非空时，选择一个曾触发新状态的 payload。
4. 按 interface data file 递归变异结构化字段。
5. 将 payload 发送给 ROS system under test 执行。
6. 执行结束后检查 crash 与新状态；触发 crash 或新状态的 payload 保存到 pool，用于后续 mutation。

论文原句对应关系，供审查对照：

- "R2D2 conducts a dry run, which only boosts the system without sending any input, to extract all interface specifications" 对应第 1 条。
- "if the pool is empty, R2D2 selects an interface randomly from the extracted specifications and generates payloads accordingly" 对应第 2 条。
- "else, R2D2 selects a payload that previously triggered new state for mutation. The mutation is conducted recursively based on data files from the interface specification" 对应第 3、4 条。
- "These prepared payloads are then sent to the ROS system for execution" 对应第 5 条。
- "If a new state or crash is triggered, the payload is preserved in the pool for future iterations" 对应第 6 条。

## 3. 改动文件

### 3.1 新建

- `docs/plan/payload_generator_reproduction_plan.md`：本计划文档。
- `src/payload.rs`：`Payload` 数据模型、`ValueTree` 结构值树、序列化与反序列化接口。
- `src/payload_pool.rs`：interesting payload pool，实现入池与选择语义。
- `src/mutation.rs`：递归变异器与参数化变异配置。
- `src/payload_generator.rs`：生成主循环决策，编排 pool、mutation 与 trait 边界。
- `tests/payload_generator.rs`：基于内存替身的单元与端到端测试。

### 3.2 修改

- `src/interface_extractor.rs`：移除 `Kind::Action`；将 `Field` 的平面 `ty` 字符串扩展为递归类型树；`Interface` 增加关联 data files 信息。
- `src/lib.rs`：注册新增模块。
- `tests/interface_extractor.rs`：随 interface 模型扩展更新既有断言。
- `README.md`：更新模块边界说明。

## 4. 实现原理

分条说明，行内代码仅用于类型与变量名。

### 4.1 interface 模型扩展

- 现有 `Field` 只有 `name` 与 `ty` 两个字符串字段，无法表达嵌套消息与数组，也就无法支撑论文要求的"按 data files 递归变异"。
- 将字段类型改为引用类型树节点 `TypeNode`：叶子节点为基础类型 `Primitive`（bool、有符号与无符号整数族、浮点、string、byte array），内部节点为复合类型 `Composite`（嵌套消息与定长/变长数组）。
- `Interface` 增加 `data_files` 字段，保存该 interface 关联的 data file 定义（.msg 或 .srv 源与解析后的类型树），作为递归生成与变异的依据。
- 移除 `Kind::Action`，与 `r2d2_strict_reproduction_plan.md` 第 1.2 节"只覆盖 topic 与 service"保持一致。
- `Extractor` trait 的签名保持"返回 `Vec<Interface>`"不变，使既有 mock 测试只需做最小更新。

### 4.2 Payload 数据模型

- `Payload` 至少持有四类信息：目标 interface 标识 `interface_id`、目标种类 `kind`、结构化值树 `value`、本轮随机种子 `rng_seed`。
- `ValueTree` 是与 `TypeNode` 同构的值树：`Leaf` 承载基础类型值，`Nested` 承载嵌套消息字段序列，`Array` 承载数组元素序列。值树与类型树同构是类型正确生成的保证。
- `Payload` 增加 `serialized` 字节缓冲，保存序列化结果；重放时可通过 `interface_id` 与 `rng_seed` 重新生成，或直接反序列化 `serialized` 恢复输入。
- 每个 payload 可保存到磁盘并确定性重放，满足阶段 G 验收条件。

### 4.3 空池生成路径

- `PayloadGenerator` 在每轮开始时调用 `PayloadPool::is_empty()` 判断分支。
- 池为空时，用当前 RNG 从提取的 `Vec<Interface>` 中按 `interface_selection` 策略选择一个 interface；当前唯一已实现策略是 `uniform`。
- 按所选 interface 的 `TypeNode` 树递归生成值树：基础类型按其取值范围分布采样，数组先按配置的 `array_len_range` 采样长度再逐元素生成，嵌套消息逐字段递归；字段或其嵌套成员若带默认值，先落默认值，再对其余位置生成。
- 生成结果必须保证类型正确，即 `ValueTree` 与 `TypeNode` 逐节点匹配，这是向 topic publisher 或 service client 发送的前提。

### 4.4 非空池变异路径

- 池非空时，调用 `PayloadPool::pick_for_mutation()` 按 `pool_selection` 策略选择一个曾触发新状态的 payload；当前唯一已实现策略是 `uniform`。
- 被选 payload 的 `interface_id` 决定了变异所依据的 data files 与类型树，与论文"mutation is conducted recursively based on data files"一致。
- 变异产出的新 payload 保留原 interface 绑定，只改值树内容，不跨 interface 变换。

### 4.5 递归变异器

- 变异以 `MutationContext` 贯穿：记录剩余 mutation energy `mutation_energy`、当前递归深度与深度上限 `max_recursion_depth`。
- 从值树根节点开始，按 `TypeNode` 类型选择候选变异算子；算子命中后用 energy 递减，递归下钻嵌套字段或数组元素。
- 各类型可用算子通过 `operator_weights` 配置权重：基础类型支持翻转、边界值替换、随机重采样等；字符串支持长度伸缩、字节级替换等；数组支持长度增减、元素级变异；嵌套消息支持字段级递归变异。
- 论文未披露具体算子集合，全部算子作为 reproduction choice 参数化，不伪装成论文算法。

### 4.6 序列化与确定性重放

- 发送前调用 `Serializer` 将 `ValueTree` 按 `TypeNode` 序列化为 wire 格式字节，写入 `Payload::serialized`。
- 序列化实现（CDR 或自定义格式）论文未披露，标记为 reproduction choice；`Serializer` 以 trait 定义，允许后续替换。
- 重放要求：同一 `interface_id`、同一 `rng_seed`、同一 pool 状态下生成的 payload 必须字节级一致；已保存 payload 可通过反序列化直接重放。
- topic payload 与 service request 共用同一序列化边界，service response 不在生成范围内。

### 4.7 trait 边界

- `Sender`：抽象把 payload 送入 ROS SUT，占位实现用内存队列替身，真实 ROS 2 发送（`rclcpp` publisher / service client）在阶段 H 接入。
- `StateOracle`：抽象查询执行结果，接口包括 `is_new_state()` 与 `crashed()`；generator 只消费判定结果决定是否入池，不实现判定逻辑本身。
- 入池语义集中在 `PayloadPool::push()`：只有 oracle 判定 crash 或 new-state 的 payload 才允许入池，generator 不得绕过。
- 这一分层保证本模块可在无 tracer、无真实 ROS 的环境下测试，同时不依赖阶段 F 的实现细节。

### 4.8 随机性与可复现性

- generator、变异器与 pool 选择共用同一个可播种 RNG（如 `StdRng`），每轮开始记录 `rng_seed`。
- 配置对象 `GeneratorConfig` 聚合全部参数化项，与 `rng_seed` 一起写入 payload 元数据，使每一轮生成决策可追溯、可重放。

## 5. 论文未披露参数的参数化处理

以下项目论文未给出具体定义，必须保留为可配置缺口并标注 reproduction choice：

- 各 ROS 基础类型取值分布：配置项 `per_type_value_ranges`。
- 字符串、数组、嵌套消息与边界值的具体变异算子：配置项 `operator_weights` 与 `operators_per_type`。
- interface 与 pool item 的选择分布：配置项 `interface_selection` 与 `pool_selection`；当前只实现 `uniform`，不伪装成论文概率模型。
- mutation energy 与递归深度：配置项 `mutation_energy` 与 `max_recursion_depth`。
- payload 序列化实现：`Serializer` 的默认实现。
- 数组长度分布：配置项 `array_len_range`。

不属本计划的缺口，交叉引用：callback ID hash 算法、significant deviation 阈值与统计方法见 `r2d2_strict_reproduction_plan.md` 阶段 E 与阶段 F；fuzzing round 的启动、重置与超时规则见阶段 H。

## 6. 与其他阶段的依赖与测试策略

### 6.1 依赖关系

- 上游依赖：`interface_extractor` 提供 `Vec<Interface>` 作为生成依据；在模型扩展（P1）完成前，generator 的其余部分不启动。
- 下游依赖：阶段 H 的主循环调用 `PayloadGenerator::next_payload()` 获得输入，调用 `StateOracle` 判定后把结果交给 pool。
- 与阶段 C/D（tracer）无直接耦合：运行期行为经 `StateOracle` 隔离。
- 与阶段 F（状态基线）无直接耦合：新状态判定逻辑属于 Feedback Controller，不进入本模块。

### 6.2 测试策略

- 沿用 `tests/interface_extractor.rs` 的 mock 风格：`MockExtractor` 提供手写 interface；`MockSender` 收集发送记录；`MockOracle` 返回预设判定。
- 核心测试用例：空池时按 spec 生成类型正确 payload；非空池时选择池内 payload 变异；嵌套消息与数组可递归生成与变异；同 seed 生成结果字节一致；只有 crash 或 new-state payload 才入池。
- 所有测试不依赖真实 ROS 2 环境，保证在无 ROS 的 CI 上可运行。

## 7. 验收条件

逐条对齐 `r2d2_strict_reproduction_plan.md` 阶段 G：

1. dry run 不发送任何 fuzz input。
2. 提取结果足以生成类型正确的 topic message 与 service request。
3. 可对嵌套结构递归生成与变异。
4. 每个 payload 可序列化、保存并确定性重放。
5. 只有 crash 或 new-state payload 才进入 interesting pool。

## 8. 实施步骤

- P1：扩展 interface 模型，移除 `Kind::Action`，为 `Field` 引入 `TypeNode` 类型树，为 `Interface` 增加 data files；更新 `tests/interface_extractor.rs`。
- P2：实现 `src/payload.rs` 的 `ValueTree`、`Payload` 与 `Serializer` trait 及内存替身实现。
- P3：实现 `src/payload_pool.rs`，包括入池、判空与 `pick_for_mutation()`。
- P4：实现 `src/payload_generator.rs` 的每轮分支决策与 `GeneratorConfig` 聚合。
- P5：实现 `src/mutation.rs` 递归变异器与算子权重配置。
- P6：落地 `Sender` 与 `StateOracle` trait，编写 `tests/payload_generator.rs` 端到端内存测试。
- P7：在复现报告中记录全部 reproduction choice 与 unresolved gap 的分栏对照。

## 9. 复现报告要求

- 每轮生成与变异记录：`interface_id`、`rng_seed`、是否来自 pool、变异能量与深度、算子命中记录。
- 报告分栏区分 paper setting（论文明确规定的行为）与 reproduction choice（本计划的自选参数），未披露项不得写成论文配置。
- 与 `r2d2_strict_reproduction_plan.md` 第 7、8 节的缺口清单与交付物清单保持一致。
