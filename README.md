# R2D2 Rust core

按 `docs/plan/payload_generator_reproduction_plan.md` 落地了 payload generator 复现计划的 P1 至 P6，并按 `docs/plan/tracer_reproduction_plan.md` 落地了 tracer 复现计划的 T1 至 T5。

## Quickstart：在 Jazzy 上跑通 nav2_costmap_e2e

在 ROS 2 Jazzy 环境里跑通 R2D2 闭环（Rust 核心 + 插桩 costmap + /scan 驱动）的最小路径。当前闭环的边界见文末「当前边界」；本文档只保证下面的命令在本仓库当前状态下可跑。

### 前提

- 系统已安装 `/opt/ros/jazzy`（含 `setup.bash`），且 `colcon`、`ros2`、`setarch` 可用（`setarch` 用于关闭 ASLR，TSAN/coverage 构建必需）。
- Rust 工具链：`cargo`（edition 2024，需 Rust 1.85+）。
- 仅支持 Linux，且依赖 `/dev/shm`（tracer 环形缓冲与 payload 文件都放这里）。
- 可选：`lcov`（只有加 `--lcov-dir` 抓分支覆盖时才需要）。

### 0. 环境预检（可选）

```bash
scripts/check_env.sh
```

检查 ROS setup、ros2/colcon/setarch/Rust 工具链、`/dev/shm` 可写性以及 nav2_ws 是否已搭建构建；必需项不过会以非零退出码收尾并给出排查方向（lcov 缺失只是 WARN，不加 `--lcov-dir` 就不需要）。

### 1. 构建 nav2_ws

`nav2_ws/` 是 gitignore 的工作区（插桩后的 navigation2 副本 + `r2d2_tracer` + `r2d2_scan_bridge` + 启动脚本）。本机已搭建完成；新机器需按 `docs/plan/nav2_jazzy_instrumentation_plan.md` 第 2、5 节先搭好目录结构再构建。

```bash
scripts/build_nav2_ws.sh
```

一键脚本会先检查工作区与依赖是否就位，再做普通 RelWithDebInfo 构建。构建口径切换（需干净重建，约 2-3 分钟）：

```bash
scripts/build_nav2_ws.sh --clean --coverage   # 纯 coverage 构建（论文覆盖口径近似）
scripts/build_nav2_ws.sh --clean --tsan       # TSAN + coverage（并发检测战役）
```

构建成功后 `nav2_ws/install/setup.bash` 存在，`launch_stack.sh` 与 `costmap_params.yaml` 位于 `nav2_ws/` 下。路径不依赖固定目录：仓库挪位置后重编译即可，或用下面的环境变量显式指定（空值视为未设置）。

### 2. 跑测试

```bash
cargo test
```

预期全部通过；`tests/trace_buffer.rs` 里的 C++ 往返测试在 `tracer/build/mock_writer` 不存在时会打印提示并跳过，不影响通过。

### 3. 跑最小闭环

```bash
cargo run --example nav2_costmap_e2e -- --rounds 4 --seed 42
```

首轮会先拉起 costmap 栈（静态 TF + lifecycle configure/activate，约 10 秒），之后每轮生成/变异一个 LaserScan payload，由 `r2d2_scan_bridge` 发布到 /scan，Rust 侧读 shm trace 做新状态判定。4 轮约一分钟。

成功时输出形如（2026-08-21 按本文档命令实跑截取，省略 ROS 启动日志）：

```text
dry run: extracted interface LaserScan (topic) with 10 top-level fields
startup: 12 registration records, 6 complete callbacks: get_cost [Service],
  clear_except [Service], clear_around [Service], clear_around_pose [Service],
  clear_entirely [Service], /scan [Subscription]
loop: rounds=4 seed=42 baseline_rounds=2 latency_factor=2 throughput_floor=0.5
round 01 | len= 111 | calls=23 msgs=23 | exec=[294010ns,...] thr=[0.83,1.48] MB/s | decision=none      | pool=0 | sched=-
round 02 | len=  89 | calls=38 msgs=38 | exec=[207431ns,...] thr=[1.21,2.04] MB/s | decision=none      | pool=0 | sched=-
round 03 | len=  96 | calls=29 msgs=29 | exec=[90551ns,...]  thr=[1.63,1.91] MB/s | decision=new-state | pool=1 | sched=-
round 04 | len= 134 | calls=35 msgs=35 | exec=[119903ns,...] thr=[1.88,1.36] MB/s | decision=new-state | pool=2 | sched=-
=== summary ===
rounds=4 crashes=0 new_states=2 invalid_traces=0 empty_rounds=0 pool_size=2
callback_graph_edges=1 distinct_callbacks=1
```

- `calls/msgs`：本轮记录到的回调执行数与消息数；`thr` 为消息吞吐（MB/s）；`pool` 为变异池大小（前两轮 baseline 只积累基准，pool 从 0 起）。
- `crashes=0` 且进程退出码为 0 即成功；若 costmap 栈崩溃，summary 的 `crashes` 会非零。
- 结束时进程组、/dev/shm 文件与 payload 文件都会自动清理。

### 4. 常用进阶参数

| 参数 | 作用 |
|---|---|
| `--rounds N` / `--seed N` | 轮数与随机种子（默认 10 / 42） |
| `--seed-dir tests/fixtures/nav2_seeds` | 加载 nav2-_fuzz 种子语料：scans 预填变异池，schedules 决定每轮发布时序（rate/duration/burst/stamp_mode） |
| `--lcov-dir nav2_ws/results` | 每轮结束后用 lcov 抓累计分支覆盖落盘（需 lcov；覆盖口径构建见 `docs/plan/nav2_lcov_full_run.md`） |
| `--tsan-log-dir 目录` | TSAN 构建下把报告写入该目录 |
| `--round-duration 秒` / `--bridge-rate Hz` | 无 seed-dir 时每轮发布时长与速率 |

### 5. 环境变量（默认值已可跑，仓库挪位置或想换 shm 名时用）

| 变量 | 默认值 | 作用 |
|---|---|---|
| `R2D2_WS_ROOT` | 编译期仓库根（`CARGO_MANIFEST_DIR`） | 仓库根，`nav2_ws` 默认取 `<根>/nav2_ws` |
| `R2D2_NAV2_WS` | `<R2D2_WS_ROOT>/nav2_ws` | nav2 工作区路径（launch 脚本、params、payload 文件都在其下） |
| `R2D2_ROS_SETUP` | `/opt/ros/jazzy/setup.bash` | ROS 2 setup 脚本（launch 栈与 bridge 都 source 它） |
| `R2D2_COSTMAP_PARAMS` | `<R2D2_NAV2_WS>/costmap_params.yaml` | costmap 参数文件 |
| `R2D2_SHM_PATH` | `/dev/shm/r2d2_nav2` | tracer 共享内存路径（C++ tracer 对象名取 basename，Rust 侧同步使用） |
| `ROS_DOMAIN_ID` | `190` | DDS domain，栈与 bridge 保持一致 |

## 当前模块

- `interface_extractor`：递归类型树（`TypeNode`）以及 `FileExtractor`，可从真实 `.msg`/`.srv` 文件提取 topic/service 接口、嵌套消息依赖和 service request/response。
- `payload`：`Payload` 数据模型、`ValueTree` 值树与 `Serializer` 序列化边界（`SimpleSerializer` 为 reproduction choice）。
- `payload_pool`：interesting payload pool，只允许 crash 或 new-state payload 入池。
- `mutation`：递归变异器（`Mutator`）与类型正确生成（`generate_value`）。
- `payload_generator`：每轮分支决策（空池生成 / 非空池变异）、`GeneratorConfig`，以及 `Sender`、`StateOracle` trait 边界。
- `trace_buffer`：shared memory reader，解析 C++ tracer 写入的 registration 与 runtime 环形缓冲（阶段 C/D 的 Rust 侧读取端）。
- `callback_profile`：跨 drain 合并完整注册信息，按论文 Figure 5 生成 callback ID、latency 与 throughput，并阻止丢失、冲突、截断或时序异常的 trace 进入后续状态反馈。

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
- 本仓库在 Jazzy 上只打通了最小闭环（`nav2_costmap_e2e`）：目标仅为单个 `nav2_costmap_2d` 程序，主输入面只有 `/scan`（LaserScan，经 `r2d2_scan_bridge` 发布）；输入面与回调面远小于论文的四个目标系统 + topic/service 全接口矩阵。
- 插桩在 nav2 应用层（`nav2_ws/src/navigation2` 副本），不是论文的 RCL 层：`executor_execute` 语义不完整、handler 关联为应用层近似、shm 初始化在节点 main。逐条偏差见 `docs/plan/nav2_jazzy_instrumentation_plan.md` 第 4 节。
- state oracle（`BaselineOracle`）仍是阶段 F 雏形：新执行边 + 延迟/吞吐偏离判据；其中 latency/throughput 判据超出论文边界（论文明确不检测 timing bug）。
- 覆盖率为 gcc+gcov/lcov 近似，非论文的 clang SanitizerCoverage；叠加 Jazzy 版本、单程序目标、TSAN+coverage 合并构建等口径差异，本仓库的覆盖数字与缺陷统计不能直接对齐论文表格。
- `FileExtractor` 当前支持基础类型、嵌套消息、无界序列和固定数组；bounded type、常量和字段默认值会明确返回错误，因为现有 `TypeNode` 尚未保存这些约束。
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
| 超长 callback name | 定长区截断并置标志；截断名称不参与 callback ID 生成 | `trace_records.h` / `callback_profile` |
| 平台假设 | 小端、x86-64 下 `pthread_mutex_t` 为 40 字节（C++ static_assert 兜底）、对齐 8 字节读取按实践原子 | `trace_buffer` 模块 |

## 运行测试

```text
cargo test
```

后续接入真实 ROS 2 时，只需新增实现 `Extractor`、`Sender`、`StateOracle` 与 `Serializer` 的类型，并按 tracer 计划接入点对 rclcpp/rcl 源码插桩。
