# Nav2 全量运行 + 每轮 lcov 分支覆盖指南

> 日期：2026-08-20。
> 前置：`examples/nav2_costmap_e2e.rs` 已支持 `--lcov-dir` 参数（每轮结束发 SIGUSR1 →
> costmap 进程 `__gcov_dump()` → lcov 抓取落盘）；`r2d2_tracer` 在 COVERAGE_RUN 构建下
> 安装该信号 handler 并写 PID 文件到 `/dev/shm/r2d2_nav2.pid`。
> 约束：全程只 `source /opt/ros/jazzy/setup.bash` 读取环境，不写 /opt。

## 一、覆盖率重建 nav2_ws（一次性；换机器或普通构建后重跑）

先重建 `nav2_ws`，再重建 overlay；否则 overlay 第二阶段会把 `nav2_costmap_2d`
重新编回普通模式，实时 lcov 会再次失效。

两种构建必须分开：**coverage** 用于 lcov/gcov 覆盖率战役；**TSAN-only** 用于
并发检测战役。不要把 `--coverage` 与 `-fsanitize=thread` 混用，否则 `__gcov0.*`
计数器会成为 TSan 噪声来源。

当前实现用 `R2D2_PROFILE` 固定选择构建产物，避免同一个 build 目录在两种插桩之间
反复覆盖：

- `R2D2_PROFILE=coverage`：`nav2_ws/build`、`nav2_ws/install`、
  `overlay_ws/build`、`overlay_ws/install`。
- `R2D2_PROFILE=tsan`：`nav2_ws/build_tsan`、`nav2_ws/install_tsan`、
  `overlay_ws/build_tsan`、`overlay_ws/install_tsan`。
- `R2D2_PROFILE=sancov`：`nav2_ws/build_sancov`、`nav2_ws/install_sancov`、
  `overlay_ws/build_sancov`、`overlay_ws/install_sancov`。

运行时 `nav2_costmap_e2e` 会从同一份 YAML 读取当前 profile 的
`R2D2_NAV2_BUILD_BASE` 和 `R2D2_NAV2_INSTALL_SETUP`，所以 coverage campaign 只从
coverage build 抓 `.gcda`，TSan campaign 只加载 TSan build。`sancov` campaign
加载 `TSan + LLVM SanitizerCoverage` build，用 edge counter 替代 lcov。

### 1a. 纯 coverage 构建

```bash
cd /home/ocsar/ROS/my_r2d2
R2D2_PROFILE=coverage scripts/build_nav2_ws.sh --coverage --clean
R2D2_PROFILE=coverage scripts/build_overlay_ws.sh --clean
```

等价的底层命令如下。

```bash
cd /home/ocsar/ROS/my_r2d2/nav2_ws
bash -c 'source /opt/ros/jazzy/setup.bash && colcon build --symlink-install --parallel-workers 12 \
  --packages-select r2d2_tracer r2d2_scan_bridge nav2_msgs nav2_common nav2_util nav2_voxel_grid nav2_amcl nav2_behaviors nav2_bt_navigator nav2_controller nav2_costmap_2d nav2_lifecycle_manager nav2_map_server nav2_planner nav2_smoother nav2_bringup \
  --cmake-clean-cache \
  --cmake-args -DBUILD_TESTING=OFF -DCMAKE_BUILD_TYPE=RelWithDebInfo \
  -DCMAKE_C_COMPILER=/usr/bin/cc -DCMAKE_CXX_COMPILER=/usr/bin/c++ \
  -DCMAKE_C_FLAGS="--coverage -DCOVERAGE_RUN=1 -fprofile-update=atomic -w -Wno-error" \
  -DCMAKE_CXX_FLAGS="--coverage -DCOVERAGE_RUN=1 -fprofile-update=atomic -w -Wno-error" \
  -DCMAKE_EXE_LINKER_FLAGS="--coverage" -DCMAKE_SHARED_LINKER_FLAGS="--coverage"'
```

### 1b. TSAN-only 构建（并发检测战役用）

```bash
cd /home/ocsar/ROS/my_r2d2
R2D2_PROFILE=tsan scripts/build_nav2_ws.sh --tsan --clean
R2D2_PROFILE=tsan scripts/build_overlay_ws.sh --clean
```

要点：

- `--cmake-clean-cache` + 显式 gcc：清掉历史 CMakeCache，避免 clang/gcc 混用导致
  lcov 版本戳冲突（`408*` vs `B33*`）。
- `-w -Wno-error`：GCC 13 在 coverage 构建下对 std::regex 触发
  `-Werror=null-dereference` 误报，必须压制（与 nav2-_fuzz 的 build_nav2.sh 一致）。
- `-fprofile-update=atomic`：仅用于纯 coverage 构建，防止收尾 SIGKILL 撕裂写导致的
  "Unexpected negative count"
  finalize 失败（示例的 ignore-errors 已含 `negative` 兜底）。
- 踩坑：**不要做 TSAN+coverage 合并构建**。gcov 计数器竞争会污染 TSan 报告，
  clang profile 运行时与 TSAN 组合还会让 costmap 执行器停摆。
- 桥接节点也被 TSAN 插桩，`setarch -R` 已由示例内建（内核 6.x 高熵 ASLR 下
  TSAN 会 FATAL/漏报；costmap 侧由 launch_stack.sh 处理）。
- 覆盖口径战役用 1a，TSAN 战役用 1b；两者现在使用不同 build/install/log 目录，
  不再互相覆盖。输出仍建议分开归档：`results/` 放 lcov，`tsan_reports/` 放 TSan。
- 如需完全干净，直接使用脚本：
  `scripts/build_nav2_ws.sh --coverage --clean && scripts/build_overlay_ws.sh --clean`

## 二、清空历史覆盖计数

`nav2_costmap_e2e` 现在会在战役启动时自动清掉 `nav2_ws/build` 下的旧 `.gcda`，
避免 `libgcov profiling error: ... different checksum` 污染真实运行日志。
如果你是**不经过 harness** 手动起 ROS 栈，再单独跑 lcov，仍然要自己先清。

```bash
cd /home/ocsar/ROS/my_r2d2/nav2_ws
find build -name "*.gcda" -delete
rm -rf results
```

不清 `.gcda` 会让上一轮战役的计数污染本轮 `branch_covered_increase`。

## 三、全量运行

当前 fuzz harness 仍是 `nav2_costmap_e2e`；完整 Nav2 bringup 的启动入口已补到
`nav2_ws/launch_nav2_full_stack.sh`，用于下一步把输入用例扩到 AMCL、planner、
controller、BT navigator、smoother、behaviors 等节点。该脚本依赖第一节的
`nav2_bringup` 安装产物，并继承同一套 rcl/rclcpp tracer 与 coverage 构建。

```bash
cd /home/ocsar/ROS/my_r2d2
R2D2_PROFILE=coverage cargo run --example nav2_costmap_e2e -- --rounds 10 --seed 42 \
  --lcov-dir nav2_ws/results \
  --seed-dir config/nav2_seeds
```

- `--seed-dir`：引入 nav2-_fuzz 的时序语料（见第五节）。当前只使用
  `schedules/` 决定每轮发布时序（rate/duration/burst/stamp_mode），
  不再把单一 `scans/` 样例预填到 payload pool。不传该参数则退化为纯随机
  生成 + 固定 20Hz×2s。
- `--lcov-dir`：每轮结束后抓取 lcov 分支覆盖落盘。
- `--tsan-log-dir`：只在 TSAN-only 构建下使用，报告写入该目录（`tsan.<pid>`）。
  不允许与 `--lcov-dir` 同时传。

TSAN 战役（1b 构建）全量指令：

```bash
cd /home/ocsar/ROS/my_r2d2
R2D2_PROFILE=tsan cargo run --example nav2_costmap_e2e -- --rounds 10 --seed 42 \
  --tsan-log-dir nav2_ws/tsan_reports \
  --seed-dir config/nav2_seeds
```

`nav2_costmap_e2e` 显式拒绝同时传 `--lcov-dir` 与 `--tsan-log-dir`。coverage 的
每轮反馈来自 `lcov`；TSan 的每轮反馈来自 `tsan.<pid>` 报告文件，两者不在同一进程
插桩组合里混用。

TSan + SanitizerCoverage 战役：

```bash
cd /home/ocsar/ROS/my_r2d2
R2D2_PROFILE=sancov scripts/build_nav2_ws.sh --sancov --clean
R2D2_PROFILE=sancov scripts/build_overlay_ws.sh --clean
R2D2_PROFILE=sancov cargo run --example nav2_costmap_e2e -- --rounds 10 --seed 42 \
  --tsan-log-dir nav2_ws/tsan_reports_sancov \
  --sancov-dir nav2_ws/sancov_results \
  --seed-dir config/nav2_seeds
```

`--sancov-dir` 与 `--lcov-dir` 互斥。输出不是 lcov `.info`，而是原始 `.sancov`
文件、每轮 `coverage/status.json`、按包 `coverage/packages.json` 和最终
`summary.json`。反馈单位是 SanitizerCoverage PC，不是 lcov branch。

## 四、实测证据（profile 隔离）

2026-08-26 实测：

- `R2D2_PROFILE=tsan scripts/build_nav2_ws.sh --tsan --clean` 完成，随后
  `R2D2_PROFILE=tsan scripts/build_overlay_ws.sh --clean` 完成。
- `find nav2_ws/build_tsan nav2_ws/install_tsan overlay_ws/build_tsan overlay_ws/install_tsan -name '*.gcno' -o -name '*.gcda' | wc -l`
  输出 `0`，证明 TSan build 不含 gcov 产物。
- coverage profile 的 1 轮真实 fuzz 完成：
  `branches 26029/150160 (+11)`，final `26050/150160`，输出目录为
  `nav2_ws/results_profile_cov_probe/`。
- TSan profile 的 1 轮真实 fuzz 完成：全栈启动、benchmark `analyzed=1`、
  round 1 执行完成，`nav2_ws/tsan_reports_profile_probe/` 生成 5 个
  `tsan.<pid>` 报告。
- `rg "__gcov|gcov" nav2_ws/tsan_reports_profile_probe` 无匹配，说明 TSan 报告不再被
  覆盖率计数器污染。

已知发现（2026-08-21 战役）：costmap 关闭路径存在真实 data race：
`mapUpdateLoop` 线程读 `active_`（costmap_2d_ros.cpp:531）与主线程
`on_deactivate` 写 `active_`（:353）无同步；影响良性（线程随后被 join），
但属 nav2 上游真实竞争，见 `nav2_tsan_report.md`。

## 五、每轮结果

每轮结果落盘到 `nav2_ws/results/rounds/round_XXXXXX/`：

- `coverage.info`：本轮到当前为止的累计 lcov 分支覆盖
- `packages/`：按 9 个目标包拆出的 `*.info`、`*.json` 和 `packages.json`
- `trace.json`：本轮完整 `CallbackTrace`（call/message trace、lossy、diagnostics）
- `summary.json`：`round` / `decision` / `calls` / `pool_size` / `crash_or_hang` /
  `coverage_ok` / `branch_covered_total` / `branch_covered_increase` / `packages`
- `payload.txt`：当轮变异 payload（可复现）
- `round.txt`：当轮打印行（含本轮 sched 名）

`callbacks.json` 保存 callback ID 到名称、类型、namespace、RCLCPP/RCL handler 的映射；
`benchmark/traces/trace_XXXXXX.json` 保存 benchmark 每次成功执行的完整 trace。

## 六、查看结果

```bash
cd /home/ocsar/ROS/my_r2d2/nav2_ws/results
cat coverage/status.json                                      # 最新实时覆盖状态
cat summary.json                                              # 战役总览
cat callbacks.json                                            # callback ID 与名称映射
jq . rounds/round_000001/trace.json                           # 第一轮完整 callback trace
jq . rounds/round_000001/packages/packages.json                # 第一轮 9 包分支覆盖
jq . packages/packages.json                                    # final 9 包分支覆盖
grep -hE '"branch_covered_(total|increase)"' rounds/round_*/summary.json   # 每轮累计与增量
lcov --summary coverage_total.info --rc branch_coverage=1     # 总体行/函数/分支覆盖
# HTML 报告：lcov_html/index.html（浏览器打开）
```

当前自动拆分的 9 个重点包来自图片中的目标集合：

`nav2_amcl`、`nav2_behaviors`、`nav2_bt_navigator`、`nav2_controller`、
`nav2_costmap_2d`、`nav2_lifecycle_manager`、`nav2_map_server`、`nav2_planner`、
`nav2_smoother`。

每个 `*.json` 都记录 `branch_covered_total`、`branch_total`、
`expected_branch_total` 和 `matches_expected_branch_total`。`expected_branch_total`
来自图片中的 9 包源码过滤口径，用于检查本机 lcov/gcov 口径是否一致。

## 七、输入样例语料（nav2-_fuzz 种子）

语料位于 `config/nav2_seeds/`，由 `scripts/import_nav2_seeds.py`
从 nav2-_fuzz 的 `seed_pool`/`local_seed_pool` 一次性转换而来：

- `scans/*.txt`：两行文本扫描（7 标量 + ranges，即 bridge 的 payload 格式）。
  含 canonical 规范扫描与曾触发 crash/TSAN 的 bug_candidates 变异扫描
  （如 range 0.1863、angle_increment 取负、angle_max 3.57）。
- `schedules/*.sched`：每行
  `duration_sec period_ms burst_count burst_gap_ms max_publishes stamp_mode`，
  来自 74 份 event 时序种子与 bug_candidates。
- `provenance.csv`：来源路径 / round / sanitizer_kind / 去重记录。

旧 fuzzer 的语料按内容去重后高度集中（4 份唯一扫描 + 10 份唯一 /scan 时序），
属正常现象：其变异集中在其它 topic 与 runtime 值域。重新导入：

```bash
python3 scripts/import_nav2_seeds.py \
  --source /home/ocsar/ROS/nav2-_fuzz \
  --output config/nav2_seeds
```

## 八、回归

```bash
cd /home/ocsar/ROS/my_r2d2 && cargo test          # 87 项全过
```

## 常用变体参数

| 参数 | 默认 | 说明 |
|---|---|---|
| `--rounds N` | 10 | fuzzing 轮数 |
| `--seed S` | 42 | 随机种子 |
| `--baseline N` | 2 | 前 N 轮只积累基准不做判定 |
| `--round-duration 秒` | 2.0 | 每轮向 /scan 发布的时长 |
| `--bridge-rate Hz` | 20 | 发布频率 |
| `--latency-factor` | 2.0 | new-state 延迟判据 |
| `--throughput-floor` | 0.5 | new-state 吞吐判据 |
| `--tsan-log-dir 路径` | 无 | 只用于 TSAN-only 战役，不可与 `--lcov-dir` 同时传 |
| `--seed-dir 路径` | 无 | nav2-_fuzz 语料目录（当前只使用 `schedules/` 驱动每轮时序） |

## 注意事项

- lcov 2.0 已废弃 `lcov_branch_coverage`，一律用 `--rc branch_coverage=1`。
- `coverage_total.info` 的总分支（约 6.6 万）含 nav2_msgs 生成代码与系统头内联，
  与论文 Table 4 的"单程序已覆盖分支数"不可直接比较；优先看 `packages/`
  下的 9 包源码过滤结果。
- 实时 coverage flush 已改为对 full-stack Nav2 进程组发送 `SIGUSR1`，让多个目标包同时 dump `.gcda`。
- 单接口（仅 /scan）场景下，obstacle_layer 可达分支在第 1 轮即基本饱和，
  后续轮 `branch_covered_increase` 多为 0 属正常现象；扩展插桩面（static_layer、
  PointCloud2、action 输入等）后覆盖会随轮次继续增长。
- 本文档当前命令仍运行 `nav2_costmap_e2e`，但该 harness 已固定为 full-stack
  Nav2 合并路线；输入面来自 67 个 topic/service/action/safe-parameter-profile
  binding，并按 9 个 Nav2 目标包做覆盖分包统计。
