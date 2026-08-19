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
