# my_r2d2

## 运行命令配置

项目不再在 Rust 代码中逐项拼装外部命令。固定的可执行文件、参数顺序和选项
位于以下两个文件：

- `config/runtime_commands.yaml`：Nav2 栈启动、进程管理以及 lcov/genhtml；
- `config/ros2_senders.yaml`：LaserScan bridge、ROS 2 CLI 和 topic/service/action 前缀。

模板中的 `{{name}}` 接收一个运行时参数，`{{name...}}` 接收零个或多个独立
argv 参数。参数不会重新交给 shell 拆词，因此路径中的空格和 JSON payload 会原样
保留。若要使用另一份配置，可分别设置 `R2D2_RUNTIME_COMMANDS_CONFIG` 和
`R2D2_ROS2_SENDERS_CONFIG`；未设置时读取仓库内上述文件。修改命令时只需编辑
YAML，无需重新修改 Rust 的命令构造逻辑。

## 非侵入式 LLVM 双层插桩

当前 callback trace 使用 LLVM 18 pass 在编译期完成，不修改
`rclcpp`、`rcl`、`ros2_tracing` 或九个目标 Nav2 包的源码：

- RCL 层收集 node、subscription、service 的名字和底层 handle；
- RCLCPP 层关联 callback 对象，并收集 executor/callback 的运行事件；
- 独立 runtime 把 registration/runtime 记录写入已有共享内存双 ring buffer。

`scripts/build_overlay_ws.sh` 会先从三个 ROS 仓库的 `HEAD` 生成只读意义上的
干净编译副本，再对 RCL/RCLCPP 和九个目标包统一加载 LLVM pass。完整设计、
边界和验证方法见
[非侵入式 LLVM 插桩说明](docs/plan/non_invasive_llvm_instrumentation.md)。

单独验证 LLVM pass：

```bash
scripts/build_llvm_instrumentation.sh
```

覆盖率运行方法：

```bash
R2D2_PROFILE=coverage scripts/build_nav2_ws.sh --coverage --clean
R2D2_PROFILE=coverage scripts/build_overlay_ws.sh --clean
R2D2_PROFILE=coverage ROS_DOMAIN_ID=200 cargo run --example nav2_costmap_e2e -- --rounds 1000 --seed-dir config/nav2_seeds --fresh-generation-period 1 --fresh-selection shuffle-cycle --lcov-dir outputs/nav2_lcov_results
```

TSan 运行方法：

```bash
R2D2_PROFILE=tsan scripts/build_nav2_ws.sh --tsan --clean
R2D2_PROFILE=tsan scripts/build_overlay_ws.sh --clean
R2D2_PROFILE=tsan ROS_DOMAIN_ID=201 cargo run --example nav2_costmap_e2e -- --rounds 1000 --seed-dir config/nav2_seeds --fresh-generation-period 1 --fresh-selection shuffle-cycle --tsan-log-dir outputs/nav2_tsan_reports
```

TSan + SanitizerCoverage 运行方法：

```bash
R2D2_PROFILE=sancov scripts/build_nav2_ws.sh --sancov --clean
R2D2_PROFILE=sancov scripts/build_overlay_ws.sh --clean
R2D2_PROFILE=sancov ROS_DOMAIN_ID=202 cargo run --example nav2_costmap_e2e -- --rounds 1000 --seed-dir config/nav2_seeds --fresh-generation-period 1 --fresh-selection shuffle-cycle --tsan-log-dir outputs/nav2_tsan_reports_sancov --sancov-dir outputs/nav2_sancov_results
```

若要严格观察原始 R2D2 pool-first 行为，可以去掉
`--fresh-generation-period 1 --fresh-selection shuffle-cycle`。全栈覆盖率实验建议保留
这两个参数，否则第一个触发 new-state 的接口容易长期占住 payload pool，降低 67 个
Nav2 topic/service/action/parameter binding 的轮转质量。
