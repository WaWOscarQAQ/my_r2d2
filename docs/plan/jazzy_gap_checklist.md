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

状态：已完成（2026-08-21）。落地为 `README.md` 顶部「Quickstart：在 Jazzy 上跑通 nav2_costmap_e2e」一节：前提（/opt/ros/jazzy、colcon/ros2/setarch、Rust 1.85+、/dev/shm、可选 lcov）、nav2_ws 构建命令、`cargo test`、最小运行命令与成功输出样例、常用进阶参数表，并注明 launch_stack.sh 路径硬编码与 nav2_ws 新机器搭建（分别待 A2/A3 收口）。

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

状态：已完成（2026-08-21）。`examples/nav2_costmap_e2e.rs` 的 SHM 路径、ROS setup、workspace、params、ROS_DOMAIN_ID 全部改为环境变量解析（默认值等价原硬编码）；`nav2_ws/launch_stack.sh` 以 `$0` 所在目录为默认工作区并接受 `R2D2_*` 覆盖；C++ 侧 `costmap_2d_node.cpp` 的 tracer shm 名取 `R2D2_SHM_PATH` basename；`scripts/import_nav2_seeds.py` 默认源支持 `R2D2_FUZZ_SOURCE` 覆盖。README Quickstart 第 5 节收录环境变量表。已用默认方式与显式环境变量方式各实跑验证（见验证记录）。

目标：

- 让仓库不依赖 `/home/ocsar/ROS/my_r2d2` 这一固定目录。

建议修改文件：

- `nav2_ws/launch_stack.sh`
- `examples/nav2_costmap_e2e.rs`

优先抽出的配置项：

- `R2D2_WS_ROOT`
- `R2D2_ROS_SETUP`
- `R2D2_NAV2_WS`
- `R2D2_SHM_PATH`
- `ROS_DOMAIN_ID`
- `R2D2_COSTMAP_PARAMS`

验收标准：

- 仓库挪到新路径后，只靠环境变量或默认相对路径仍能启动。

### A3. 增加一键构建脚本

优先级：P0

状态：已完成（2026-08-22）。落地 `scripts/build_nav2_ws.sh`：收口 plain / coverage(1a) / tsan(1b) 三种构建口径；构建前检查 ROS setup、工作区、`r2d2_tracer`/`r2d2_scan_bridge`/插桩 navigation2/params 是否就位与 colcon 是否可用；记录上次模式并在未 `--clean` 切换时拒绝；`--clean` 清理对应包产物；覆盖/TSAN 构建后提示清 `.gcda`；踩坑处理（setup.bash 的 set -u、COLCON_CURRENT_PREFIX）内建。已验证：plain 与 tsan 两种模式各全量构建并实跑 e2e 通过，模式切换守卫生效。README Quickstart 第 1 步改为脚本入口。

目标：

- 把当前散落在文档里的 `colcon build` 命令收口成一个入口。

建议新增文件：

- `scripts/build_nav2_ws.sh`

脚本职责：

- `source /opt/ros/jazzy/setup.bash`
- 调用 `colcon build --packages-select ...`
- 提前检查 `nav2_ws/src/r2d2_tracer`、`nav2_ws/src/r2d2_scan_bridge` 是否存在
- 构建失败时输出下一步排查点

### A4. 增加环境预检脚本

优先级：P1

状态：已完成（2026-08-22）。落地 `scripts/check_env.sh`：只读预检 Linux 系统、ROS setup 与 ros2（含「不在 PATH 但 setup 内可 source」的判定）、colcon/setarch/Rust 1.85+/lcov（lcov 仅 WARN）、/dev/shm 挂载与可写（写探针文件后删除）、nav2_ws 是否搭建/构建/含 params 与 launch 脚本；必需项失败退出码 1 并附排查方向。已验证：正常环境全绿退出 0，缺失工作区/ROS setup/cargo 场景均正确 FAIL。README Quickstart 新增第 0 步。

目标：

- 在真正运行 example 前，把常见缺依赖或路径问题提前暴露。

建议新增文件：

- `scripts/check_env.sh`

建议检查项：

- `/opt/ros/jazzy/setup.bash` 是否存在
- `nav2_ws/install/setup.bash` 是否存在
- `ros2`、`colcon`、`setarch` 是否在 PATH
- `/dev/shm` 是否可用
- `nav2_ws/costmap_params.yaml` 是否存在

### A5. 明确当前 demo 边界

优先级：P1

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

基线文档：

- `docs/plan/callback_trace_profile_issue_report.md`

未闭环事项：

- namespace 缺失
- publish timestamp 来源未定
- payload 轮次边界未定义
- 并发读写验证未完成

影响：

- 若这些语义不定，后续 Global Callback Graph 与 latency / throughput benchmark 都可能带歧义。

### B4. 把 Sender / StateOracle 变成真实实现

优先级：P1

目标：

- 把当前 example 里的特化逻辑收敛成可复用运行时组件。

涉及文件：

- `src/payload_generator.rs`
- `examples/nav2_costmap_e2e.rs`

建议方向：

- `Sender` 负责真实 ROS topic / service 注入
- `StateOracle` 负责 trace 判定、crash 判定、sanitizer 结果归档
- 把 example 中的专用调度逻辑尽量下沉到库层或独立 harness 模块

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
