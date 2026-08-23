# Nav2 全量运行 + 每轮 lcov 分支覆盖指南

> 日期：2026-08-20。
> 前置：`examples/nav2_costmap_e2e.rs` 已支持 `--lcov-dir` 参数（每轮结束发 SIGUSR1 →
> costmap 进程 `__gcov_dump()` → lcov 抓取落盘）；`r2d2_tracer` 在 COVERAGE_RUN 构建下
> 安装该信号 handler 并写 PID 文件到 `/dev/shm/r2d2_nav2.pid`。
> 约束：全程只 `source /opt/ros/jazzy/setup.bash` 读取环境，不写 /opt。

## 一、覆盖率重建 nav2_ws（一次性；换机器或普通构建后重跑）

两种构建二选一：**coverage**（覆盖口径战役，分支总数 ~6.6 万，与历史结果可比）或
**TSAN+coverage**（并发检测战役；其 lcov 分支总数因 TSAN 插桩膨胀约 2 倍，只作
战役内部反馈，不与纯 coverage 数字对比——见方案 A 说明）。

### 1a. 纯 coverage 构建

```bash
cd /home/ocsar/ROS/my_r2d2/nav2_ws
bash -c 'source /opt/ros/jazzy/setup.bash && colcon build --symlink-install --parallel-workers 12 \
  --packages-select r2d2_tracer r2d2_scan_bridge nav2_msgs nav2_common nav2_util nav2_voxel_grid nav2_costmap_2d \
  --cmake-clean-cache \
  --cmake-args -DBUILD_TESTING=OFF -DCMAKE_BUILD_TYPE=RelWithDebInfo \
  -DCMAKE_C_COMPILER=/usr/bin/cc -DCMAKE_CXX_COMPILER=/usr/bin/c++ \
  -DCMAKE_C_FLAGS="--coverage -DCOVERAGE_RUN=1 -fprofile-update=atomic -w -Wno-error" \
  -DCMAKE_CXX_FLAGS="--coverage -DCOVERAGE_RUN=1 -fprofile-update=atomic -w -Wno-error" \
  -DCMAKE_EXE_LINKER_FLAGS="--coverage" -DCMAKE_SHARED_LINKER_FLAGS="--coverage"'
```

### 1b. TSAN + coverage 构建（并发检测战役用）

```bash
cd /home/ocsar/ROS/my_r2d2/nav2_ws
bash -c 'source /opt/ros/jazzy/setup.bash && colcon build --symlink-install --parallel-workers 12 \
  --packages-select r2d2_tracer r2d2_scan_bridge nav2_msgs nav2_common nav2_util nav2_voxel_grid nav2_costmap_2d \
  --cmake-clean-cache \
  --cmake-args -DBUILD_TESTING=OFF -DCMAKE_BUILD_TYPE=RelWithDebInfo \
  -DCMAKE_C_COMPILER=/usr/bin/cc -DCMAKE_CXX_COMPILER=/usr/bin/c++ \
  "-DCMAKE_C_FLAGS=-O1 -g -w -Wno-error -fno-omit-frame-pointer -fsanitize=thread --coverage -fprofile-update=atomic -DCOVERAGE_RUN=1" \
  "-DCMAKE_CXX_FLAGS=-O1 -g -w -Wno-error -fno-omit-frame-pointer -fsanitize=thread --coverage -fprofile-update=atomic -DCOVERAGE_RUN=1" \
  "-DCMAKE_EXE_LINKER_FLAGS=-fsanitize=thread --coverage" \
  "-DCMAKE_SHARED_LINKER_FLAGS=-fsanitize=thread --coverage"'
```

要点：

- `--cmake-clean-cache` + 显式 gcc：清掉历史 CMakeCache，避免 clang/gcc 混用导致
  lcov 版本戳冲突（`408*` vs `B33*`）。
- `-w -Wno-error`：GCC 13 在 coverage 构建下对 std::regex 触发
  `-Werror=null-dereference` 误报，必须压制（与 nav2-_fuzz 的 build_nav2.sh 一致）。
- `-fprofile-update=atomic`：计数器更新原子化。TSAN 构建下消除计数器伪报；
  纯 coverage 构建下防止收尾 SIGKILL 撕裂写导致的 "Unexpected negative count"
  finalize 失败（示例的 ignore-errors 已含 `negative` 兜底）。
- 踩坑：**不要用 clang-18 做 TSAN+coverage**——clang 的 profile 运行时与 TSAN 组合
  会让 costmap 执行器在首个扫描后停摆（回调不再执行）；clang 只适合 TSAN-only。
- 桥接节点也被 TSAN 插桩，`setarch -R` 已由示例内建（内核 6.x 高熵 ASLR 下
  TSAN 会 FATAL/漏报；costmap 侧由 launch_stack.sh 处理）。
- 方案 A（业界标准做法，GCC 官方建议 sanitizer 与 gcov 分开）：覆盖口径战役用 1a，
  TSAN 战役用 1b；同一 workspace 内切换需干净重建（约 2–3 分钟），切换前归档
  `results/` 与 `tsan_reports/`（如 `mv results results_tsan_YYYYMMDD`）。
- 如需完全干净，先删旧产物再构建：
  `rm -rf build/{nav2_common,nav2_msgs,nav2_util,nav2_voxel_grid,nav2_costmap_2d,r2d2_tracer,r2d2_scan_bridge}`

## 二、清空历史覆盖计数（每次全量前必做）

```bash
cd /home/ocsar/ROS/my_r2d2/nav2_ws
find build -name "*.gcda" -delete
rm -rf results
```

不清 `.gcda` 会让上一轮战役的计数污染本轮 `branch_covered_increase`。

## 三、全量运行

```bash
cd /home/ocsar/ROS/my_r2d2
cargo run --example nav2_costmap_e2e -- --rounds 10 --seed 42 \
  --lcov-dir nav2_ws/results \
  --seed-dir tests/fixtures/nav2_seeds
```

- `--seed-dir`：引入 nav2-_fuzz 的输入样例（见第五节）。scans 预填 pool，
  第 1 轮起即从真实扫描变异；schedules 决定每轮发布时序（rate/duration/
  burst/stamp_mode）。不传该参数则退化为纯随机生成 + 固定 20Hz×2s。
- `--lcov-dir`：每轮结束后抓取 lcov 分支覆盖落盘。
- `--tsan-log-dir`：TSAN 构建下加此参数，报告写入该目录（`tsan.<pid>`）；
  每轮 SIGUSR1 触发 gcov dump 的 signal-unsafe 告警已通过 `launch_stack.sh`
  导出的 `TSAN_OPTIONS=report_signal_unsafe=0` 关闭（本地脚本属 gitignore
  的 nav2_ws，换机重建时需保留该导出）。

TSAN 战役（1b 构建）全量指令：

```bash
cd /home/ocsar/ROS/my_r2d2/nav2_ws
find build -name "*.gcda" -delete && rm -rf results tsan_reports && mkdir -p tsan_reports
cd /home/ocsar/ROS/my_r2d2
cargo run --example nav2_costmap_e2e -- --rounds 10 --seed 42 \
  --lcov-dir nav2_ws/results \
  --tsan-log-dir nav2_ws/tsan_reports \
  --seed-dir tests/fixtures/nav2_seeds
```

已知发现（2026-08-21 战役）：costmap 关闭路径存在真实 data race——
`mapUpdateLoop` 线程读 `active_`（costmap_2d_ros.cpp:531）与主线程
`on_deactivate` 写 `active_`（:353）无同步；影响良性（线程随后被 join），
但属 nav2 上游真实竞争，见 `nav2_tsan_report.md`。

每轮结果落盘到 `nav2_ws/results/rounds/round_XXXXXX/`：

- `coverage.info`：本轮到当前为止的累计 lcov 分支覆盖
- `summary.json`：`round` / `decision` / `calls` / `pool_size` / `crash_or_hang` /
  `coverage_ok` / `branch_covered_total` / `branch_covered_increase`
- `payload.txt`：当轮变异 payload（可复现）
- `round.txt`：当轮打印行（含本轮 sched 名）

## 四、查看结果

```bash
cd /home/ocsar/ROS/my_r2d2/nav2_ws/results
cat summary.json                                              # 战役总览
grep -hE '"branch_covered_(total|increase)"' rounds/round_*/summary.json   # 每轮累计与增量
lcov --summary coverage_total.info --rc branch_coverage=1     # 总体行/函数/分支覆盖
# HTML 报告：lcov_html/index.html（浏览器打开）
```

仅统计目标包（对齐论文 Table 4 的"单程序"口径，排除生成代码与系统头）：

```bash
cd /home/ocsar/ROS/my_r2d2/nav2_ws/results
lcov --extract coverage_total.info '*/navigation2/nav2_costmap_2d/*' \
  --rc branch_coverage=1 -o costmap_only.info
lcov --summary costmap_only.info --rc branch_coverage=1
```

## 五、输入样例语料（nav2-_fuzz 种子）

语料位于 `tests/fixtures/nav2_seeds/`，由 `scripts/import_nav2_seeds.py`
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
  --output tests/fixtures/nav2_seeds
```

## 六、回归

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
| `--tsan-log-dir 路径` | 无 | 与 lcov 可同时开，TSAN 报告按轮落盘 |
| `--seed-dir 路径` | 无 | nav2-_fuzz 语料目录（scans/ 预填 pool，schedules/ 驱动每轮时序） |

## 注意事项

- lcov 2.0 已废弃 `lcov_branch_coverage`，一律用 `--rc branch_coverage=1`。
- `coverage_total.info` 的总分支（约 6.6 万）含 nav2_msgs 生成代码与系统头内联，
  与论文 Table 4 的"单程序已覆盖分支数"不可直接比较；对齐口径见第四节。
- 单接口（仅 /scan）场景下，obstacle_layer 可达分支在第 1 轮即基本饱和，
  后续轮 `branch_covered_increase` 多为 0 属正常现象；扩展插桩面（static_layer、
  PointCloud2、action 输入等）后覆盖会随轮次继续增长。
