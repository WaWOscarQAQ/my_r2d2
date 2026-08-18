# R2D2 严格论文复现与部署计划

> 唯一技术依据：Yuheng Shen et al., *Enhancing ROS System Fuzzing through Callback Tracing*, ISSTA 2024, DOI: `10.1145/3650212.3652111`。
>
> 本计划不引用本机环境、现有工程、第三方复现或论文之外的实现经验。论文未披露的信息统一列为缺口，不擅自补齐。

## 1. 复现目标与边界

### 1.1 目标

按论文实现并部署一个面向整个 ROS 2 系统的 callback trace-guided fuzzer，复现以下三类结果：

1. 功能结果：R2D2 能对 ROS runtime 与 ROS application 生成并执行结构化输入，记录 callback trace，并以新状态反馈维护 payload pool。
2. 缺陷检测结果：使用 ASAN 与 TSAN 检测内存和并发问题。
3. 实验结果：比较 R2D2、R2D2-、Ros2Fuzz 与 RoboFuzz 的分支覆盖率，并测量 R2D2 插桩的内存与延迟开销。

### 1.2 严格复现边界

- 核心 fuzzing framework 使用 Rust。
- tracer 使用 C++。
- 输入接口只按论文明确描述覆盖 topic 与 service；不自行扩展 action。
- R2D2 的 fuzzing guidance 只使用 callback trace，不使用 code coverage guidance。
- code coverage 仅用于论文 RQ2 的评估与横向比较。
- 检测目标只按论文覆盖 memory-related bugs 与 concurrency-related bugs。
- 论文明确指出当前 R2D2 不能识别 real-time constraint 相关 timing bugs，因此本轮复现不扩展 timing-bug oracle。

## 2. 论文规定的部署基线

### 2.1 硬件与操作系统

严格对齐论文第 5.1 节：

- CPU：64-core AMD EPYC 7742，2.25 GHz。
- 操作系统：Ubuntu 22.04。
- 无物理显示器时使用 X virtual frame buffer（Xvfb）。
- 所有对比实验在同一硬件上执行。

若无法取得同型号硬件，只能将实验标记为“功能复现”或“非同硬件实验复现”，不能声称严格复现论文性能数据。

### 2.2 ROS 版本与实验对象

部署两套相互隔离的 ROS runtime：

- ROS 2 Humble。
- ROS 2 Rolling，版本点应固定在论文实验时间附近；论文未给出具体 commit。

部署四个论文目标应用：

- Navigator2。
- TurtleBot3。
- Turtlesim。
- Autoware。

论文没有披露四个应用的精确 commit、完整依赖锁定文件和目标场景配置，这些属于复现缺口。取得作者工件前，不应随意选择版本后宣称结果等价。

### 2.3 编译与观测工具

- 使用 Clang 编译 ROS runtime 与 ROS applications。
- RQ1 构建启用 ASAN 与 TSAN。
- RQ2 构建对 ROS runtime 和 ROS applications 启用 SanitizerCoverage。
- callback tracing 以 Ros2Trace 为基础。
- Ros2Trace 依赖 LTTng；论文引用 `ros2_tracing` 的 tracepoint 方案。
- RQ3 使用 `performance_test` 测量 runtime latency 与 memory usage。

论文未给出 Clang 版本、编译参数、SanitizerCoverage 具体模式、ASAN/TSAN 是否分离构建，以及 coverage 数据合并命令。部署时必须记录这些缺口，不能把自选参数写成论文配置。

## 3. 论文架构的部署拓扑

R2D2 分为 callback trace collection 与 callback trace-guided generation 两个阶段。

```text
ROS applications
        |
        v
instrumented ROS runtime (RCLCPP + RCL)
        |
        v
shared memory
  - callback registration circular buffer
  - runtime execution circular buffer
        |
        v
Rust R2D2 core
  - interface extractor
  - payload generator
  - payload pool
  - feedback collector/controller
  - process monitor
  - system logger
        |
        +--> callback trace profile
        +--> new-state identification
        +--> guided payload synthesis
        +--> crash / sanitizer artifacts
```

每个 shared-memory object 必须是带 mutex 的 circular buffer，以适应 ROS 的异步执行与高事件量。论文明确要求实时写出并由 R2D2 在测试期间读取，而不是仅在进程结束后离线解析。

## 4. 分阶段部署计划

## 阶段 A：冻结论文复现合同

### 工作项

1. 固定论文 PDF、DOI、实验声明和所有表格原始值。
2. 建立“论文明确参数”和“论文未披露参数”两张清单。
3. 为每次构建记录 ROS 发行版、源码 commit、应用 commit、DDS 实现、编译器和编译选项。
4. 禁止在未标注的情况下用自选默认值填补论文缺口。

### 验收条件

- 能从部署记录追溯任一二进制对应的源码和构建参数。
- 所有非论文参数均明确标记为 reproduction choice，而不是 paper setting。

## 阶段 B：部署原始 ROS 目标与三类构建

### 工作项

1. 在 Ubuntu 22.04 上分别部署 ROS 2 Humble 与 Rolling。
2. 部署 Navigator2、TurtleBot3、Turtlesim 和 Autoware。
3. 为 RQ3 生成三类 runtime：
   - R2D2 instrumentation build。
   - Ros2Trace instrumentation build。
   - Vanilla ROS 2 build，移除全部 tracer。
4. 为 RQ1 生成 sanitizer build：ROS runtime 与应用由 Clang 构建并启用 ASAN/TSAN。
5. 为 RQ2 生成 coverage build：ROS runtime 与应用全部启用 SanitizerCoverage。
6. 配置 Xvfb，确保依赖 GUI 的应用能在无显示器环境启动。

### 验收条件

- 四个目标在未 fuzz 的情况下均可启动并完成基本任务。
- 三类 RQ3 runtime 可以执行同一个 `performance_test` workload。
- sanitizer build 能产生可解析的 ASAN/TSAN 报告。
- coverage build 能分别归集 ROSIDL、RCL_*、RCL、RMW 和 DDS 层的 branch coverage。

## 阶段 C：实现 callback registration tracing

### 论文规定的 tracer

在 RCLCPP 与 RCL 层增加：

- `rclcpp_callback_init()`
- `rcl_callback_init()`

### 必须采集的数据

`rclcpp_callback_init()` 记录：

- RCLCPP handler。
- RCL handler。
- callback type：subscription、timer 或 service。

`rcl_callback_init()` 记录：

- callback name。
- RCL handler。

采集结果写入 callback registration buffer。

### 验收条件

- 应用启动后，可以把同一个 callback 在 RCLCPP 与 RCL 层的 handler 关联起来。
- 能获得 callback name 和 callback type。
- registration buffer 可由 Rust core 在运行期间读取。

## 阶段 D：实现 runtime behavior tracing

### 论文规定的 tracer

在 RCLCPP/RCL 层增加或扩展：

- `executor_execute()`
- `callback_start()`
- `callback_end()`
- `rcl_take()`

### 必须采集的数据

callback scheduling/execution：

- 目标 callback 的 RCLCPP handler。
- executor invoke timestamp。
- callback start timestamp。
- callback end timestamp。

message passing：

- RCL handler。
- incoming message buffer size。
- publish timestamp。
- subscribe timestamp。

采集结果写入 runtime execution buffer。

### 验收条件

- 每次 callback 执行都能恢复 invoke -> start -> end 时间关系。
- 每条被观测消息都能恢复 buffer size 与 publish/subscribe 时间关系。
- 高事件量下 circular buffer 不破坏结构完整性。
- 注册数据与运行时数据能通过 handler 关联。

## 阶段 E：实现 callback trace profile

### 论文规定的数据结构

#### CallbackInfo

```text
Callback ID = Hash(callback name, callback type)
Callback Handlers = Union(RCLCPP handler, RCL handler)
```

#### Callback Latency

```text
Callback ID = CallbackInfo.find(RCLCPP handler)
Execution Latency = end timestamp - start timestamp
Scheduling Latency = start timestamp - invoke timestamp
```

#### Message Latency / Throughput

```text
Callback ID = CallbackInfo.find(RCL handler)
Throughput = buffer size / (subscribe timestamp - publish timestamp)
```

#### Callback Trace

```text
CallTrace = Vector<Callback Latency>
MsgTrace  = Vector<Message Latency>
```

同一个 callback 在一次 payload 执行中可能出现多次，因此两个 vector 必须保留重复元素与时间顺序。

### 验收条件

- 给定一轮原始 registration/runtime events，可稳定生成 CallbackInfo、CallTrace 和 MsgTrace。
- callback ID 在同一实验配置下保持稳定。
- trace 中重复 callback 不会被错误去重。

## 阶段 F：建立论文规定的状态基线

### 工作项

1. 对目标系统随机生成输入。
2. 连续采集 callback trace，论文实验使用 2 小时 sampling period。
3. 为每个 callback 与 message 建立 average benchmark value。
4. 维护两类全局状态：
   - Global Callback Graph：callback 的时序执行图。
   - Global Callback Latency：每个 callback 的总体 latency 与每个 message 的 throughput benchmark。

### 论文规定的新状态判定

当前 payload 满足任一条件即触发新状态：

1. callback trace 引入新的 execution sequence，即 Global Callback Graph 中出现新 edge。
2. 某 callback latency 相对 average benchmark 出现 significant deviation。
3. 某 message throughput 显著低于 average benchmark。

### 严格限制

论文没有披露：

- “significant deviation”的公式或阈值。
- 统计方法的具体名称。
- 每个输入重复执行的确切次数。
- outlier 的处理方式。

因此，本阶段只能先完成可配置接口和原始统计输出。获得作者配置前，不得把任一自选阈值称为论文阈值。

### 验收条件

- 新 callback edge 能被确定性识别并加入全局图。
- latency/throughput 判定器可以从外部配置阈值。
- 每次判定都保存原始观测值、基线值和判定结果。

## 阶段 G：实现 interface extraction 与 payload synthesis

### Dry run

按论文先启动系统但不发送输入，提取：

- 所有 topic interfaces。
- 所有 service interfaces。
- associated data files。
- message types。
- data formats。

### Payload 生成规则

1. payload pool 为空时，随机选择一个已提取 interface，并严格按 interface specification 生成 payload。
2. payload pool 非空时，选择一个曾触发新状态的 payload。
3. 按 interface data file 递归变异结构化字段。
4. 将 payload 发送给 ROS system under test。
5. 执行结束后检查 crash 与新状态。
6. 触发 crash 或新状态的 payload 保存到 pool，用于后续 mutation。

### 严格限制

论文没有披露：

- 各 ROS 基础类型的取值分布。
- 字符串、数组、嵌套消息和边界值的具体变异算子。
- interface 与 pool item 的选择概率。
- mutation energy、递归深度和 payload 序列化实现。

以上内容必须保留为参数化缺口，不得伪装成论文算法。

### 验收条件

- dry run 不发送任何 fuzz input。
- 提取结果足以生成类型正确的 topic message 和 service request。
- 可对嵌套结构递归生成与变异。
- 每个 payload 可序列化、保存并确定性重放。
- 只有 crash 或 new-state payload 才进入 interesting pool。

## 阶段 H：闭合 Rust fuzzing loop

### 论文规定的 Rust 组件

- Interface Extractor。
- Payload Generator。
- Payload Pool。
- Feedback Collector / Controller。
- Process Monitor。
- System Logger。

### 每轮执行顺序

```text
选择或生成 payload
  -> 启动/驱动 ROS SUT
  -> C++ tracers 实时写 shared memory
  -> Rust collector 构造 callback trace
  -> 识别新 callback edge / latency deviation / throughput degradation
  -> process monitor 检查异常退出
  -> 检查 ASAN / TSAN 结果
  -> 保存 crash 或 new-state payload
  -> 写 crash log 与 system statistics
  -> 进入下一轮
```

### 验收条件

- 不读取 code coverage 也能完成 seed selection 与 mutation guidance。
- 关闭 callback trace guidance 后得到 R2D2-，且 R2D2- 不含其他 guidance mechanism。
- 每个 bug candidate 都包含 payload、目标配置、运行日志、sanitizer 报告与可重放步骤。

## 阶段 I：按论文顺序部署目标

论文未规定实现时的目标接入先后顺序。为避免把自选顺序写成论文设置，本计划只规定最终覆盖范围：

1. Turtlesim。
2. TurtleBot3。
3. Navigator2。
4. Autoware。

以上编号仅用于部署清单，不代表论文规定的优先级。每个目标必须完成：

- dry-run interface extraction。
- topic/service payload execution。
- callback registration/runtime trace。
- callback trace profile。
- new-state feedback。
- ASAN/TSAN bug oracle。
- deterministic replay。

## 5. 严格评估计划

## 5.1 RQ1：缺陷检测能力

### 配置

- 目标：Navigator2、TurtleBot3、Turtlesim、Autoware，以及对应 ROS runtime。
- ROS：Humble 与 Rolling。
- 编译：Clang + ASAN/TSAN。
- 每个实验运行 24 小时。
- 每个实验重复 5 次。
- 所有实验使用同一硬件。

### 采集结果

- ROS runtime bugs 与 ROS application bugs 分开统计。
- memory-related 与 concurrency-related 分开统计。
- 每个报告保留触发 payload、stack、目标模块、操作名、复现率和 sanitizer 类型。

### 论文参照值

- 总计 39 个 previously unknown bugs。
- 24 个位于 ROS runtime，15 个位于 applications。
- 23 个 memory-related，16 个 concurrency-related。
- Humble 9 个，Rolling 15 个，TurtleBot3 6 个，Navigator2 2 个，Autoware 7 个。
- 8 个得到确认，6 个得到修复。

这些数字是比较基线，不是强制“造出相同数量缺陷”的验收门槛；只有相同源码版本、配置与 workload 下才可做严格数值比较。

## 5.2 RQ2：callback trace guidance 有效性

### 对比对象

- R2D2。
- R2D2-：移除 callback trace guidance，且不引入其他 guidance。
- Ros2Fuzz。
- RoboFuzz。

### 对比目标

- Navigator2：R2D2、R2D2-、Ros2Fuzz；论文说明 RoboFuzz 未适配 Navigator2。
- TurtleBot3：四种 fuzzer。
- Turtlesim：四种 fuzzer。

### Coverage 配置

- ROS runtime 与 ROS applications 均启用 SanitizerCoverage。
- 统计 branch coverage。
- 分层统计 ROSIDL、RCL_*、RCL、RMW 与 DDS。
- 每个实验 24 小时，重复 5 次，硬件相同。
- 论文指出 coverage 在第一小时趋于饱和，因此单独生成 first-hour growth curve，同时仍保留完整 24 小时结果。

### 论文 Table 2 参照原始值

| Fuzzer | Navigator2 | TurtleBot3 | Turtlesim | Average |
|---|---:|---:|---:|---:|
| R2D2 | 259111.2 | 111102.8 | 44576.2 | 138263.4 |
| R2D2- | 202274.4 | 90843.4 | 33846.4 | 108988.1 |
| RoboFuzz | - | 21867.6 | 21827.0 | 21847.3 |
| Ros2Fuzz | 29199.4 | 29394.8 | 25965.2 | 28186.5 |

论文报告 R2D2 相对 Ros2Fuzz 平均提升 3.91×，相对 RoboFuzz 平均提升 2.56×；R2D2 相对 R2D2- 的覆盖提升平均约 0.27×。

## 5.3 RQ3：插桩开销

### 三组 runtime

1. R2D2 instrumentation。
2. Ros2Trace instrumentation。
3. Vanilla ROS 2，无 tracer。

### 测量工具与指标

- 使用 `performance_test`。
- 测量 memory usage。
- 测量 execution latency。

### 论文 Table 3 参照原始值

| 指标 | R2D2 | Ros2Trace | Vanilla ROS 2 | 论文表中 overhead |
|---|---:|---:|---:|---:|
| Memory Usage (MB) | 47276.0 | 47136.0 | 46480.0 | 0.3% / 1.7% |
| Latency (ms) | 0.16226 | 0.15512 | 0.13962 | 4.6% / 16.2% |

### 报告一致性注意事项

论文摘要与结论写出约 10.4% execution overhead 和 1.0% memory overhead，但第 5.4 节 Table 3 给出相对 Ros2Trace/Vanilla 的两组值：latency 4.6%/16.2%，memory 0.3%/1.7%。严格复现时应报告原始测量值、比较分母和自行计算公式，不应只复述汇总百分比。

## 6. 复现验收层级

### L1：结构复现

- Rust core 与 C++ tracer 分工成立。
- registration/runtime 双 buffer 成立。
- 论文规定的 tracer 与数据字段齐全。
- callback trace 数据结构齐全。

### L2：功能复现

- 四个目标均可 dry-run extract interface。
- topic/service payload 可执行与重放。
- 新 callback edge、latency deviation、throughput degradation 可反馈到 payload pool。
- R2D2- 可通过关闭唯一 guidance 得到。

### L3：缺陷检测复现

- ASAN/TSAN 能报告可重放的 memory/concurrency bug candidate。
- crash、hang 或异常退出由 process monitor 与 system logger 记录。

### L4：论文实验复现

- 使用论文硬件与 Ubuntu 22.04。
- Humble/Rolling 与四个目标版本可追溯。
- 24 小时 × 5 次实验完成。
- RQ1/RQ2/RQ3 原始数据、均值、方差和计算脚本完整保存。

只有通过 L4，才可声明进行了严格实验复现；否则应准确标注为结构、功能或缺陷检测复现。

## 7. 论文未披露、必须向作者或 artifact 补齐的信息

1. R2D2 源码与构建脚本；论文未给出 artifact URL。
2. ROS Humble/Rolling 的精确 commit。
3. Navigator2、TurtleBot3、Turtlesim、Autoware 的精确 commit 与场景配置。
4. 全部 custom tracepoint 的源码位置、参数 ABI 和事件格式。
5. shared-memory layout、容量、overflow 行为与锁粒度。
6. callback ID 的具体 hash 算法。
7. significant latency deviation 与 low throughput 的精确统计公式、阈值和样本数。
8. payload 类型生成、mutation operator、概率、递归深度和选择策略。
9. fuzzing round 的启动、重置、超时、停止和状态清理规则。
10. ASAN/TSAN 的具体编译与运行参数，以及是否使用独立构建。
11. SanitizerCoverage 模式、branch 计数和分布式组件 coverage 合并方式。
12. `performance_test` 的 topology、message size、frequency、duration 与统计方法。
13. 五次重复实验的随机种子和原始结果。
14. bug 去重、确认与复现率判定规则。

在这些信息未补齐时，复现报告必须同时给出所采用的 reproduction choice，并与 paper setting 分栏记录。

## 8. 最终交付物

严格复现应最终交付：

- 两套可追溯 ROS runtime 源码与构建清单（Humble、Rolling）。
- R2D2 instrumentation、Ros2Trace instrumentation、Vanilla 三类 runtime。
- Rust fuzzing core 与 C++ tracers。
- 四个目标应用的固定版本与启动场景。
- interface specifications 与 dry-run 结果。
- registration/runtime 原始事件、callback traces、global callback graph 和基线数据。
- payload pool、crash corpus 与 deterministic replay 材料。
- ASAN/TSAN 原始报告。
- SanitizerCoverage 原始数据与 RQ2 图表。
- `performance_test` 原始数据与 RQ3 计算表。
- 24 小时 × 5 次的完整实验记录。
- 一份逐项区分 paper setting、reproduction choice 与 unresolved gap 的复现报告。
