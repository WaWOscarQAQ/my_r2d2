# Tracer 复现计划（阶段 C/D：callback registration 与 runtime behavior tracing）

> 唯一技术依据：Yuheng Shen et al., *Enhancing ROS System Fuzzing through Callback Tracing*, ISSTA 2024, DOI: `10.1145/3650212.3652111`，第 4.1.1 节 Runtime Temporal Behaviors Tracing 与第 4.3 节 Implementation。
>
> 本文档与 `r2d2_strict_reproduction_plan.md` 阶段 C、阶段 D 保持一致。论文未披露的信息统一列为缺口并标注为 reproduction choice，不擅自补齐。

## 1. 依据与范围

### 1.1 依据

- 论文第 4.1.1 节：registration tracer 与 runtime tracer 的部署位置、六个 tracer 函数与采集字段。
- 论文第 4.3 节：shared memory 初始化在 RCL 层完成，分配不同 shared memory buffer 记录不同结构信息；每个 shared memory object 是带 mutex 的 circular buffer，实时写出，测试期间由 R2D2 读取。
- `r2d2_strict_reproduction_plan.md` 阶段 C：`rclcpp_callback_init()`、`rcl_callback_init()` 与 callback registration buffer。
- `r2d2_strict_reproduction_plan.md` 阶段 D：`executor_execute()`、`callback_start()`、`callback_end()`、`rcl_take()` 与 runtime execution buffer。
- `r2d2_strict_reproduction_plan.md` 第 7 节缺口清单第 4、5 条。

### 1.2 范围

本计划覆盖：

- 独立 C++ tracer 模块：六个 tracer 函数、双环形 shared memory buffer、mutex 保护、事件 record 结构。
- Rust 侧 shared memory reader，供后续 Feedback Collector 使用。
- mock 事件源与端到端验证（C++ 写、Rust 读）。

本计划不覆盖：

- 真实 ROS 2 源码插桩（依赖阶段 B 冻结 Humble/Rolling 源码 commit，本计划只给出接入点清单）。
- callback trace profile（阶段 E）、新状态判定（阶段 F）。

### 1.3 环境事实

- 本机只有 ROS 2 Jazzy 二进制安装（含头文件、无源码树），论文目标为 Humble/Rolling 源码构建。
- 因此本轮以独立模块加 mock 事件源验证 tracer 逻辑与 C++/Rust ABI；真实插桩待阶段 B 落地后按接入点清单实施。

## 2. 论文规定的行为基线

- 在两个层部署两类 tracer：registration tracer 与 runtime tracer。
- `rclcpp_callback_init()` 记录 RCLCPP handler、RCL handler、callback type（subscription、timer 或 service）。
- `rcl_callback_init()` 记录 callback name、RCL handler。
- 以上写入 callback registration buffer。
- `executor_execute()`、`callback_start()`、`callback_end()` 记录目标 callback 的 RCLCPP handler、invoke timestamp、start timestamp、end timestamp。
- `rcl_take()` 记录 RCL handler、incoming message buffer size、publish timestamp、subscribe timestamp。
- 以上写入 runtime execution buffer。
- 每个 shared memory object 是带 mutex 的 circular buffer，实时写出，R2D2 在测试期间读取，不做事后离线解析。

## 3. 改动文件

### 3.1 新建（C++，位于 `tracer/`）

- `tracer/CMakeLists.txt`：构建静态库 `tracer` 与可执行程序 `mock_writer`。
- `tracer/include/tracer/trace_records.h`：record 结构体与事件枚举，字段严格按论文，附布局 static_assert。
- `tracer/include/tracer/circular_buffer.h`：mutex 保护、无锁读的环形缓冲模板。
- `tracer/include/tracer/shared_memory.h`：POSIX `shm_open`/`mmap` 封装与 shm 头部布局。
- `tracer/src/tracers.cpp`：六个 tracer 函数与 `init()`。
- `tracer/src/mock_writer.cpp`：模拟注册与运行事件的写入程序，支持输出 golden fixture 字节文件。

### 3.2 新建（Rust）

- `src/trace_buffer.rs`：shared memory reader，解析 header 与两个环形缓冲，增量游标与覆盖计数。
- `tests/trace_buffer.rs`：golden fixture 解析测试、溢出测试、可选的真实 C++ 往返测试。
- `tests/fixtures/trace_golden.bin`、`tests/fixtures/trace_overflow.bin`：C++ mock writer 生成的固定事件序列字节文件。

### 3.3 修改

- `src/lib.rs`：注册 `trace_buffer` 模块。
- `README.md`：模块说明与 reproduction choice 记录。

## 4. 实现原理

分条说明，行内代码仅用于类型与变量名。

### 4.1 共享内存布局

- 单个 shm 文件内依次放置 `SharedHeader`、registration ring、runtime ring。
- `SharedHeader` 固定 48 字节：`magic`、`version`、`shm_size`、`registration_capacity`、`registration_records_offset`、`runtime_capacity`、`runtime_records_offset`。
- 每个 ring 由 `RingHeader` 与记录数组组成：`RingHeader` 含 `pthread_mutex_t`（40 字节）、`write_index`、`overflow_count`、`capacity`，共 64 字节；记录数组紧随其后。
- Rust reader 跳过 mutex 区域（常量 `RING_MUTEX_SLOT`），从固定偏移读取 `write_index`、`overflow_count`、`capacity`。
- 布局与容量均为自选值，标注 reproduction choice；论文未披露 shm layout、容量与锁粒度。

### 4.2 record 结构

- `RegistrationRecord` 固定 160 字节：`source`（区分 `Rclcpp` 与 `Rcl` 两个 tracer）、`callback_type`、`rclcpp_handler`、`rcl_handler`、`callback_name_len`、`callback_name`（128 字节定长区）。
- `RuntimeRecord` 固定 56 字节：`event_type`、`rclcpp_handler`、`timestamp`，以及仅对 `rcl_take` 有意义的 `rcl_handler`、`buffer_size`、`pub_timestamp`、`sub_timestamp`。
- 同一次注册由 `rclcpp_callback_init()` 与 `rcl_callback_init()` 各写一条记录，两者通过 `rcl_handler` 关联，满足阶段 C"通过 handler 关联两层数据"的验收条件。
- 事件格式与参数 ABI 论文未披露，属缺口；`source` 字段是本计划为区分记录来源而设的 framing choice。

### 4.3 写入路径

- `push()` 加 mutex，把记录 memcpy 到 `write_index % capacity` 槽位，随后以 release 语义递增 `write_index`。
- 读者不取锁，先以 acquire 语义读 `write_index`，再读槽位内容；记录先写、索引后发，保证读者只看到完整记录。
- 环形覆盖最旧记录时递增 `overflow_count`，读者通过两次读取之间的计数差检测数据丢失；论文未披露 overflow 行为，属缺口。

### 4.4 六个 tracer 函数

- `rclcpp_callback_init()`：写入 `source = Rclcpp` 的注册记录，携带 `rclcpp_handler`、`rcl_handler`、`callback_type`。
- `rcl_callback_init()`：写入 `source = Rcl` 的注册记录，携带 `callback_name`、`rcl_handler`。
- `executor_execute()`：写入 `ExecutorExecute` 运行时记录，携带 `rclcpp_handler` 与 invoke timestamp。
- `callback_start()`：写入 `CallbackStart` 记录，携带 `rclcpp_handler` 与 start timestamp。
- `callback_end()`：写入 `CallbackEnd` 记录，携带 `rclcpp_handler` 与 end timestamp。
- `rcl_take()`：写入 `RclTake` 记录，携带 `rcl_handler`、`buffer_size`、`pub_timestamp`、`sub_timestamp`。
- 时间戳由调用点传入（真实插桩时调用 `now_ns()` 获取 `CLOCK_MONOTONIC`）；时钟源为 reproduction choice。

### 4.5 Rust reader

- `TraceReader::open()` 打开 `/dev/shm/<name>` 对应的文件（或 fixture 文件），校验 `magic` 与 `version`，读取容量与偏移。
- `drain_registration()` 与 `drain_runtime()` 从当前游标读到 `write_index`，返回事件列表与被覆盖跳过的 `missed` 计数，并推进游标。
- `registration_overflow()` 与 `runtime_overflow()` 返回 ring header 中的覆盖计数。
- 解析全部使用标准库 `read_at`，不新增依赖；字节序为小端，x86-64 对齐 8 字节读取按实践原子处理，标注为平台假设。

### 4.6 时间关系与关联验证

- mock 事件序列使用固定 handler 地址与固定时间戳，使 invoke/start/end 关系与 pub/sub 关系可精确断言。
- golden fixture 为 shm 完整镜像（mutex 区域清零以保证跨构建稳定），Rust 测试直接解析，验证 C++/Rust 两侧布局一致。

## 5. 论文未披露参数的参数化处理

以下项目论文未给出具体定义，必须保留为可配置缺口并标注 reproduction choice：

- shared memory 布局、容量与锁粒度：布局见第 4.1 节，容量由 `init()` 参数指定，默认 registration 1024 条、runtime 4096 条。
- overflow 行为：覆盖最旧记录并计数，见第 4.3 节。
- 各 tracepoint 在 rclcpp/rcl 源码中的精确位置、参数 ABI 与事件格式：见第 6 节接入点清单。
- 时间戳时钟源：`CLOCK_MONOTONIC`。
- 记录来源区分：`RegistrationRecord::source` 字段。
- 平台假设：小端字节序、`pthread_mutex_t` 为 40 字节（C++ 侧 static_assert 兜底）。

不属本计划的缺口，交叉引用：callback ID hash 算法见 `r2d2_strict_reproduction_plan.md` 阶段 E；shared memory 初始化 tracer 与真实 RCL 层的对接位置见阶段 B 与第 6 节。

## 6. 真实 ROS 接入点清单（仅文档，待阶段 B 实施）

- rclcpp 层：executor 调度与回调执行路径（`executor_execute()`、`callback_start()`、`callback_end()`）；subscription、timer、service 创建路径（`rclcpp_callback_init()`）。
- rcl 层：subscription/service 初始化路径（`rcl_callback_init()`）；消息 take 路径（`rcl_take()`）。
- shared memory 初始化位于 RCL 层初始化流程，与论文"instrument the shared memory initialization tracer during the initialization of the RCL layer"一致。
- 精确 patch 位置依赖阶段 B 冻结的 Humble/Rolling 源码 commit，逐项标注 reproduction choice，不得在冻结前声称与论文一致。

## 7. 测试策略

- golden fixture 解析：纯 Rust 测试，验证注册事件字段、运行时事件字段与固定时间戳。
- 溢出测试：容量 4 的 ring 写入 6 条记录，断言 `overflow_count` 与 `missed` 均为 2，幸存记录为最后 4 条且顺序正确。
- 往返测试：若 `tracer/build/mock_writer` 存在（或通过 `TRACER_MOCK_WRITER` 指定），spawn 真实 C++ 进程写入 shm，Rust 侧 `TraceReader` 读取并断言；二进制缺失时打印提示并跳过。
- handler 关联测试：同一 callback 的 `Rclcpp` 与 `Rcl` 注册记录 `rcl_handler` 一致，满足阶段 C 验收条件。

## 8. 验收条件

逐条对齐 `r2d2_strict_reproduction_plan.md` 阶段 C、D：

1. 应用启动后，可以把同一个 callback 在 RCLCPP 与 RCL 层的 handler 关联起来。
2. 能获得 callback name 和 callback type。
3. registration buffer 可由 Rust core 在运行期间读取。
4. 每次 callback 执行都能恢复 invoke -> start -> end 时间关系。
5. 每条被观测消息都能恢复 buffer size 与 publish/subscribe 时间关系。
6. 高事件量下 circular buffer 不破坏结构完整性。
7. 注册数据与运行时数据能通过 handler 关联。

## 9. 实施步骤

- T1：编写本计划文档。
- T2：C++ record 结构、环形缓冲、shm 封装与 CMake。
- T3：六个 tracer 函数与 `mock_writer`，生成 golden fixture。
- T4：Rust `trace_buffer.rs` reader。
- T5：golden fixture 测试、溢出测试与 C++ 往返测试。
- T6：更新 `README.md` 与 `src/lib.rs`。

## 10. 复现报告要求

- 每次运行记录：shm 名称、容量、时钟源、平台与编译器版本。
- 报告分栏区分 paper setting（论文明确规定的 tracer 函数与字段）与 reproduction choice（布局、容量、溢出行为、时钟源、接入点选择），未披露项不得写成论文配置。
- 与 `r2d2_strict_reproduction_plan.md` 第 7、8 节的缺口清单与交付物清单保持一致。
