# Jazzy 差距清单与执行顺序

> 日期：2026-08-21。
> 目标：把“当前仓库在 Jazzy 上能跑到什么程度”与“距离论文意义上的 R2D2 还差什么”拆成两条可执行路线。
> 适用范围：当前仓库 `my_r2d2`，以及其中的 `nav2_ws` Jazzy costmap 闭环。

## 1. 当前状态

### 1.1 已经具备的能力

- Rust 核心模块、C++ tracer、`nav2_costmap_e2e` example、`nav2_ws` 工作区均已存在。
- `cargo test` 与 `cargo build --examples` 可通过，说明仓库内核心代码自洽。
- `nav2_ws` 下已有 `r2d2_tracer`、`r2d2_scan_bridge`、插桩后的 `navigation2` 副本，以及已有构建产物。
- 当前可以在 **Jazzy + nav2_costmap_2d + /scan 输入** 这一缩减场景下跑闭环。

### 1.2 需要避免的误解

- 这不是论文原义上的完整 R2D2 复现。
- 当前不是 `rclcpp/rcl` runtime 层插桩，而是 nav2 应用层插桩。
- 当前不是完整 Nav2，更不是论文中的四个目标系统矩阵。

## 2. 路线 A：先把 Jazzy 最小可跑版做扎实

### A1. 补 Quickstart 文档

优先级：P0

状态：已完成（2026-08-22）。落地为 `README.md` 顶部「Quickstart：在 Jazzy 上跑通 nav2_costmap_e2e」一节：写明前提、YAML 配置、nav2_ws 构建命令、`cargo test`、最小运行命令、成功输出样例和常用进阶参数。

目标：

- 让新用户能按一页文档从零跑通 `nav2_costmap_e2e`。

建议修改文件：

- `README.md`

应写清的前提：

- 需要系统已有 `/opt/ros/jazzy/setup.bash`
- 需要 `colcon`、`ros2`、`setarch`
- 若要抓覆盖率，需要 `lcov`
- 当前只支持 Linux / `/dev/shm`

应给出的最小步骤：

1. 构建 `nav2_ws`
2. 运行 `cargo test`
3. 运行 `cargo run --example nav2_costmap_e2e -- --rounds ...`
4. 说明成功时会看到什么输出

### A2. 去掉路径硬编码

优先级：P0

状态：已完成（2026-08-22）。运行配置统一放在 `config/r2d2_env.yaml`，由 `src/utils/yaml_reader.rs` 直接读取；`examples/nav2_costmap_e2e.rs`、构建脚本、启动脚本和种子导入脚本均从该文件取值。C++ costmap 节点所需的 `R2D2_SHM_PATH` 由启动脚本从 YAML 读取后传入。配置缺失或空白时只报告对应键为空，不执行额外路径或环境校验。

目标：

- 让仓库不依赖 `/home/ocsar/ROS/my_r2d2` 这一固定目录。

建议修改文件：

- `nav2_ws/launch_stack.sh`
- `examples/nav2_costmap_e2e.rs`

优先抽出的配置项：

- `R2D2_ROS_SETUP`
- `R2D2_NAV2_WS`
- `R2D2_PYTHON_EXECUTABLE`
- `R2D2_SHM_PATH`
- `ROS_DOMAIN_ID`
- `R2D2_FUZZ_SOURCE`

验收标准：

- 仓库挪到新路径后，YAML 中的相对路径仍按配置文件目录解析并能启动。

### A3. 增加一键构建脚本

优先级：P0

状态：已完成（2026-08-22）。落地 `scripts/build_nav2_ws.sh`：从 YAML 读取工作区、ROS setup 和 Python 路径，收口 plain / coverage(1a) / tsan(1b) 三种构建口径；记录上次模式并在未 `--clean` 切换时拒绝，`--clean` 清理对应包产物。脚本不重复做依赖和路径预检，实际缺失项由对应命令直接报错。

目标：

- 把当前散落在文档里的 `colcon build` 命令收口成一个入口。

建议新增文件：

- `scripts/build_nav2_ws.sh`

脚本职责：

- 从 `config/r2d2_env.yaml` 读取工作区、ROS setup 和 Python 路径
- `source` YAML 指定的 ROS setup
- 调用 `colcon build --packages-select ...`
- 支持 plain、coverage 和 tsan 三种构建模式

### A4. 增加 YAML 配置读取检查脚本

优先级：P1

状态：已完成（2026-08-22）。落地 `scripts/check_env.sh`：逐项读取并打印 `config/r2d2_env.yaml` 中的全部运行配置。键缺失、值为空或仅含空白时，直接报告具体键名并以非零状态退出；不执行系统、依赖、路径、版本或可写性校验。

目标：

- 在真正运行 example 前，确认全部运行配置都能从 YAML 读出。

建议新增文件：

- `scripts/check_env.sh`

检查规则：逐项打印 YAML 配置；键缺失、空值或纯空白值时报出键名，不做其他校验。

### A5. 明确当前 demo 边界

优先级：P1

状态：已完成（2026-08-22）。`examples/nav2_costmap_e2e.rs` 头部注释新增边界段（仅 nav2_costmap_2d、主输入面 /scan、应用层插桩、阶段 F 雏形 oracle、gcov 近似覆盖口径），构建命令同步改为 `scripts/build_nav2_ws.sh`；README「当前边界」整节改写，删除「尚未连接 ROS 2 graph」等过时描述，改为按四类边界（目标范围/插桩层/oracle/覆盖口径）逐条说明，均指向对应文档。

目标：

- 避免后续把当前 Jazzy costmap 闭环误写成完整论文复现。

建议修改文件：

- `README.md`
- `examples/nav2_costmap_e2e.rs`

应明确写出的边界：

- 仅支持 `nav2_costmap_2d`
- 当前主输入面是 `/scan`
- 当前 state oracle 仍是阶段 F 雏形
- 当前结果不能直接对齐论文覆盖率与缺陷统计

## 3. 路线 B：向论文复现逼近

### B1. 先冻结“论文复现合同”

优先级：P0

状态：已完成（2026-08-22）。落地 `docs/plan/r2d2_reproduction_contract.md`：固定论文身份（DOI + PDF SHA-256）、环境实测值（Ubuntu 24.04 / Jazzy 二进制 / gcc 13.3 / nav2_costmap_2d 1.3.12 vendor 副本）、paper setting / reproduction choice / unresolved gap 三栏登记、本轮 C1–C3 决策记录与 B2 环境分叉决策点。复现层级声明上限为 L1/L2（受限），不得声称 L3/L4。

目标：

- 明确哪些配置来自论文，哪些只是 reproduction choice。

基线文档：

- `docs/plan/r2d2_strict_reproduction_plan.md`

必须继续维持的原则：

- 不把自选参数写成 paper setting
- 记录 ROS 版本、源码 commit、应用 commit、编译器和编译参数

### B2. 迁移到 runtime 层插桩

优先级：P0

目标：

- 从“只改 nav2 应用层”推进到论文要求的 `rclcpp/rcl` 层插桩。

基线文档：

- `docs/plan/tracer_reproduction_plan.md`
- `docs/plan/r2d2_strict_reproduction_plan.md`

核心差距：

- 当前 `executor_execute()` 语义不完整
- 当前 handler 关联是应用层近似
- 当前 shm 初始化位置不在论文要求的 runtime 初始化路径

### B3. 补 trace 语义缺口

优先级：P0

状态：部分完成（2026-08-22）。namespace 已按论文 §4.1.1 补入注册采集（ABI v2，ID 仍只哈希 name+type，跨 namespace 碰撞显式拦截）；payload 轮次边界已定义为 RoundBoundary marker 事件并在 e2e 落地分段；并发读写已由 stress writer + 并发 drain 测试验证“只丢不错”。publish timestamp 未改 ABI：应用层来源（header.stamp）已在复现合同中冻结，rcl 层候选机制登记为未验证 gap，待 B2 定案。详见 `callback_trace_profile_issue_report.md` 第 8 节。

基线文档：

- `docs/plan/callback_trace_profile_issue_report.md`

未闭环事项：

- ~~namespace 缺失~~（已补齐，ABI v2）
- publish timestamp 来源未定（已合同化应用层口径；rcl 层真实来源待 B2）
- ~~payload 轮次边界未定义~~（RoundBoundary marker）
- ~~并发读写验证未完成~~（压力测试通过对账）

影响：

- 若这些语义不定，后续 Global Callback Graph 与 latency / throughput benchmark 都可能带歧义。

### B4. 把 Sender / StateOracle 变成真实实现

优先级：P1

状态：已完成（2026-08-23）。`src/runtime/ros2_sender.rs` 提供真实 `Ros2LaserScanSender`，负责把生成出的 LaserScan payload 渲染到 bridge 文本文件并调用 `r2d2_scan_bridge` 注入 `/scan`；`src/runtime/state_oracle.rs` 现提供共享的 `BenchmarkBuilder` / `BenchmarkStateOracle`，按论文结构先独立采样 benchmark model，再在 fuzz phase 比较 trace、缓存 crash 状态并做新状态判定。`examples/nav2_costmap_e2e.rs`、`examples/end_to_end.rs` 都已切到该实现，`PayloadGenerator::retain_if_interesting()` 负责把 crash/new-state payload 收回 pool。

目标：

- 把当前 example 里的特化逻辑收敛成可复用运行时组件。

涉及文件：

- `src/payload_generator.rs`
- `examples/nav2_costmap_e2e.rs`

建议方向：

- `Sender` 负责真实 ROS topic / service 注入
- `StateOracle` 负责 trace 判定、crash 判定、sanitizer 结果归档
- 把 example 中的专用调度逻辑尽量下沉到库层或独立 harness 模块

当前剩余边界：

- 真实 sender 现在已覆盖 topic/service/action/safe parameter profile；剩余工作是把
  当前 Nav2 full-stack binding 进一步泛化为完整 ROS graph 自动发现。
- crash/sanitizer 结果仍主要由 example/harness 汇总，未形成统一归档接口。
- 更大范围的输入面扩展已顺延到 B5。

### B5. 扩大输入面和回调面

优先级：P1

目标：

- 让 callback graph 不再只是单一 `/scan` 自环。

建议顺序：

1. 扩 costmap 相关回调
2. 接入 clear/get_cost 之外更多 service / timer / thread 行为
3. 扩到 bt_navigator / planner / controller / amcl
4. 再考虑 action 输入面

### B6. 最后再做论文实验矩阵

优先级：P2

目标：

- 对齐论文中的目标系统、构建矩阵和评估方式。

需要补齐的维度：

- ROS 2 Humble
- ROS 2 Rolling
- Navigator2
- TurtleBot3
- Turtlesim
- Autoware
- ASAN / TSAN
- coverage build
- `performance_test`

说明：

- 这是成本最高的一步，不应先做。
- 在 B1 至 B5 未基本收敛前，做大矩阵只会放大不确定性。

## 4. 推荐执行顺序

### 第一阶段：让别人稳定跑起来

1. A1 Quickstart 文档
2. A2 去硬编码
3. A3 一键构建脚本
4. A4 环境预检脚本
5. A5 明确 demo 边界

### 第二阶段：把当前 demo 变成更像框架的东西

1. B4 Sender / StateOracle 实装
2. B5 扩输入面和回调面

### 第三阶段：回到论文主线

1. B1 冻结复现合同
2. B2 runtime 层插桩
3. B3 trace 语义缺口收敛
4. B6 论文实验矩阵

## 5. 我对当前项目的判断

- **如果目标是“Jazzy 上可运行的验证版”**：最缺的是工程化，不是算法主体。
- **如果目标是“论文级复现”**：最缺的是 runtime 层插桩、语义闭环和实验矩阵，不是再往当前 costmap demo 上堆更多局部补丁。
- **短期最值钱的工作**：A1、A2、A3。这三项完成后，仓库的可交接性会明显提升。
- **中期最值钱的工作**：B2、B3。没有这两项，论文一致性始终站不住。

## 6. 后续可直接落地的文件清单

优先建议按下面顺序改：

1. `README.md`
2. `nav2_ws/launch_stack.sh`
3. `examples/nav2_costmap_e2e.rs`
4. `scripts/build_nav2_ws.sh`
5. `scripts/check_env.sh`

若进入论文逼近阶段，再继续：

1. `docs/plan/tracer_reproduction_plan.md`
2. `src/callback_profile.rs`
3. `src/trace_buffer.rs`
4. `src/payload_generator.rs`
5. `tracer/` 与 `nav2_ws/src/r2d2_tracer/`
