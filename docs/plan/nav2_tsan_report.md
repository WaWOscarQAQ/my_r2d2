# Nav2 Jazzy TSAN fuzz 报告（第一轮战役，2026-08-19/20）

> 目标：在只插桩 nav2（应用层）的约束下，用 ThreadSanitizer 当并发 bug oracle，
> 跑通「TSAN 构建 → trace 引导 fuzz → 报告收集」全流程。
> 结论：**管线已验证可用（阳性对照产出完整 race 报告），但本轮 25 轮战役在
> costmap 扫描路径上未检测到 data race**；原因分析见第 5 节，扩大刺激面是下一步。

## 1. 构建配置

- 编译器：`clang-18` / `clang++-18`，`-O1 -g -fno-omit-frame-pointer -fsanitize=thread`
  （编译与链接 flags 均带），`CMAKE_BUILD_TYPE=RelWithDebInfo`。
- 范围：隔离工作区 `nav2_ws` 内全部 7 个包（r2d2_tracer、r2d2_scan_bridge、
  nav2_msgs、nav2_common、nav2_util、nav2_voxel_grid、nav2_costmap_2d）。
- 验证：`nm` 确认对象文件含 `__tsan_func_entry/exit`、`__tsan_read/write*` 全套
  插桩；主可执行文件含 `__tsan_init` 运行时（共享库按 TSAN 设计不链 libtsan，
  符号由主程序解析——这是正常形态，不是构建问题）。

## 2. 关键修复：高熵 ASLR 与 TSAN 不兼容

第一次 20 轮跑完报告目录为空，排查发现启动日志有：

```text
WARNING: ThreadSanitizer: memory layout is incompatible, possibly due to high-entropy ASLR.
```

内核 6.x + `vm.mmap_rnd_bits=32` 下 TSAN 影子内存布局被 ASLR 破坏，此状态下
**不报错但漏报**。修复：`launch_stack.sh` 中 costmap 节点改为
`setarch x86_64 -R ros2 run ...`（关闭该进程的 ASLR）。这与用户 fuzzer 的
`launch.command` 中 `setarch x86_64 -R ros2 launch ...` 的做法一致。
修复后启动仅剩正常横幅 `***** Running under ThreadSanitizer v3 *****`。

## 3. 阳性对照（证明报告管线可用）

用同一 clang/TSAN_OPTIONS/setarch 组合编译并运行一个已知 race 的双线程累加程序：

```text
WARNING: ThreadSanitizer: data race (pid=70465)
  Write of size 4 at 0x555556a60ae8 by thread T2:
    #0 main::$_1::operator()() const /tmp/tsan_race_control.cpp:5:62
  Previous write of size 4 at 0x555556a60ae8 by thread T1:
```

报告文件 `tsan.<pid>` 正常写入 `log_path` 目录。**因此战役中报告目录为空是
"未检测到 race"的真实结果，而非管线失效。**

## 4. 战役参数与结果

```text
cargo run --example nav2_costmap_e2e -- --rounds 25 --seed 7 \
  --round-duration 3.5 --bridge-rate 100 --tsan-log-dir nav2_ws/tsan_reports
```

- 流量：100 Hz × 3.5 s/轮 × 25 轮 ≈ 8700 条 LaserScan；每轮 215–329 次
  laserScanCallback 执行（回调 + rcl_take + start/end 三事件/条）。
- trace 反馈：23/25 轮触发 new-state（latency 判据在 TSAN 开销放大抖动后
  频繁命中），pool 增长到 23，`invalid_traces=0`，无 crash。
- TSAN：**0 份报告**。`tsan_reports/` 为空。

## 5. 为什么没有 race（分析，含局限）

1. **并发面小**：costmap 节点是单线程执行器，laserScanCallback 串行执行；
   唯一并发线程是 5 Hz 的 `mapUpdateLoop`。两者经由 `Costmap2D::mutex_t` 与
   ObservationBuffer 锁同步——该路径是 nav2 中被充分锁保护的成熟代码。
2. **刺激单一**：只有 `/scan` 一个输入。用户 fuzzer 的 TSAN 报告来自全栈
   bringup（9 个生命周期节点、action/param/service 并发刺激、amcl/planner/
   bt_navigator），与本轮单节点场景不可比。
3. **跨层 race 不可见**：/opt/ros 的 rclcpp/message_filters/DDS 二进制未插桩，
   nav2 代码与这些库之间若存在共享内存的 race，TSAN 只看得见一侧（插桩侧），
   检测能力受限。论文方案对整个 runtime 源码构建插桩，正是为了覆盖这一层。
4. 不排除 race 需要更长时战役/特定交错才触发——本轮仅 25 轮（约 5 分钟净流量）。

## 6. 下一步建议（按优先级）

1. 插桩扩展 + 全栈 bringup：bt_navigator / planner / controller / amcl /
   behavior_server，复用用户 fuzzer 的 launch 结构与输入面
   （/initialpose、/goal_pose、navigate 动作、clear 服务并发）。
2. 运行时层插桩（若获得许可/阶段 B 推进）：对 rclcpp/rcl 源码构建 TSAN，
   使跨层 race 可见——对应论文 RQ1 的核心配置。
3. 长时间战役：多 seed × 数百轮；TSAN 发现的 race 按用户 fuzzer 的
   `triage_tsan.py` 思路分类归档。
4. 期间把 trace 反馈与 TSAN 报告关联：记录"哪一 payload 触发的哪份报告"，
   为 interesting payload 保留复现材料（对应论文 payload pool 语义）。

## 7. 复现命令

```bash
# 构建（隔离工作区，一次完成）
cd nav2_ws && bash -c 'source /opt/ros/jazzy/setup.bash && colcon build \
  --symlink-install --parallel-workers 12 --cmake-clean-cache \
  --packages-select r2d2_tracer r2d2_scan_bridge nav2_msgs nav2_common \
    nav2_util nav2_voxel_grid nav2_costmap_2d \
  --cmake-args -DBUILD_TESTING=OFF -DCMAKE_BUILD_TYPE=RelWithDebInfo \
    -DCMAKE_C_COMPILER=/usr/bin/clang-18 -DCMAKE_CXX_COMPILER=/usr/bin/clang++-18 \
    "-DCMAKE_C_FLAGS=-O1 -g -w -Wno-error -fno-omit-frame-pointer -fsanitize=thread" \
    "-DCMAKE_CXX_FLAGS=-O1 -g -w -Wno-error -fno-omit-frame-pointer -fsanitize=thread" \
    "-DCMAKE_EXE_LINKER_FLAGS=-fsanitize=thread" \
    "-DCMAKE_SHARED_LINKER_FLAGS=-fsanitize=thread"'

# 战役
cargo run --example nav2_costmap_e2e -- --rounds 25 --seed 7 \
  --round-duration 3.5 --bridge-rate 100 --tsan-log-dir nav2_ws/tsan_reports
```

## 8. 第二轮战役（2026-08-21）：TSAN+lcov 合并构建

在「TSAN 与每轮 lcov 分支覆盖同时生效」的目标下重新验证了构建矩阵：

- **clang-18 + TSAN + --coverage 不可用**：clang 的 profile 运行时与 TSAN 组合会
  让 costmap 执行器在首个扫描后停摆（10s 内 200 条扫描仅 2 个回调执行，之后
  全部线程 futex 等待；外部观测 /scan 正常在线上）。二分定位到 --coverage 标志
  本身（TSAN-only 构建 10s 167 回调正常）。COVERAGE_RUN 宏确认只影响 tracer。
- **gcc + TSAN + --coverage 可用**，但需两个配套修复：
  1. `-fprofile-update=atomic`：libgcov 计数器默认非原子自增，TSAN 会把计数器
     更新本身报成 data race（首轮 64MB 报告全是 `__gcov0.*` 全局位置）；
     原子化后计数器误报清零。
  2. TSAN_OPTIONS 加 `report_signal_unsafe=0`：轮间 SIGUSR1 → `__gcov_dump()`
     是有意为之的覆盖落盘模式，默认开启的 signal-unsafe 检查每轮产生数千条
     保守告警；关闭该类别后报告干净。
- 报告质量：10 轮战役（语料 + 每轮 lcov）TSAN 报告 16KB、恰好 1 条 data race：

  **真实发现（本轮唯一 race，良性）**：costmap 关闭路径上
  `Costmap2DROS::mapUpdateLoop`（线程 T17）读 `active_`（costmap_2d_ros.cpp:531）
  与主线程 `on_deactivate` 写 `active_ = false`（:353）无同步。更新线程随后被
  join，实际影响良性，但属 nav2 上游真实竞争（未插桩的 /opt 版本同样存在）。

- 战役结果：10 轮、每轮 44–112 回调（TSAN 开销下吞吐下降）、new_states=3、
  pool=7、crashes=0、invalid_traces=0；最终分支覆盖 8864/129333（6.9%，
  总分支因 TSAN 插桩膨胀约一倍，与纯 coverage 构建口径不同，不可直接对比）。

## 9. 方案 B 评估（2026-08-21）：clang source-based coverage + TSAN

目标：一次战役同时拿到 TSAN 检测与"源码口径"精确分支覆盖（llvm-cov 只统计
源码分支，sanitizer 插桩分支不进计数）。评估结论：**本机工具链下不可行，
维持方案 A（分开构建）**。两个硬限制：

1. **clang profile 插桩 × TSAN 使执行器停摆**：`-fprofile-instr-generate
   -fcoverage-mapping` + `-fsanitize=thread`（gcov-mode 同理）下，costmap 执行器
   在激活后不再处理扫描（10s 内 200 条仅 2–5 个回调；strace 显示数据持续到达
   UDP 层、执行器 waitset 永不被唤醒）。二分定位：问题出在 **nav2_msgs 的
   rosidl 生成 typesupport**（运行于 FastDDS 接收线程）被插桩——将其排除插桩后
   174 回调正常（排除生成胶水代码也是合理的产品选择）。gcc TSAN + libgcov 无
   此问题。
2. **静态 profile 运行时限制（排除生成代码后仍不可行）**：系统仅有静态
   libclang_rt.profile（无共享版），每个二进制各带一份运行时。玩具程序证实：
   进程内 `__llvm_profile_dump()`（无论信号处理器还是辅助线程）只 dump 主映像，
   **DSO 数据只在各自运行时的进程退出钩子写盘**——而目标代码
   （nav2_costmap_2d_core.so 等）全部在 DSO 里。因此进程常驻 + 每轮 dump 的
   模式拿不到目标覆盖；唯一可行形态是每轮重启 costmap（对齐旧 fuzzer 的
   restart_nav2_each_round=true），经用户决策不采纳，回到方案 A。
