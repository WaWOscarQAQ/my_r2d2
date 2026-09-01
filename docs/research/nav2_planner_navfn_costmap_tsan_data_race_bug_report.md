# Nav2 planner_server / NavfnPlanner costmap TSan data race bug report

## 结论

ThreadSanitizer 在 Nav2 full-stack 运行中报告了 `planner_server` 内的真实 data race 候选。问题发生在 `nav2_navfn_planner::NavfnPlanner` 调用 `clearRobotCell()` 修改全局 costmap 时，该写操作发生在获取 `costmap_->getMutex()` 之前；同时，`global_costmap` 的 map update / publish 线程正在读取或复制同一块 costmap 内存。

建议报告定级：

```text
Severity: Medium-High
Type: Data race on Costmap2D backing buffer
Confidence: High
Affected process: planner_server
Affected components: nav2_planner + nav2_navfn_planner + nav2_costmap_2d
```

这不是 fuzzer 自身的 race，也不是单纯的 SanitizerCoverage 输出噪声。TSan 报告中的冲突地址位于 `Costmap2D` 的 heap buffer，两个访问栈均符号化到 Nav2 源码。

## 发现环境

工作目录：

```text
/home/oscar/ROS/my_r2d2
```

全量运行输出：

```text
/home/oscar/ROS/my_r2d2/outputs/fullstack_signal_full_20260901_2355_domain158
```

TSan 原始报告：

```text
/home/oscar/ROS/my_r2d2/outputs/fullstack_signal_full_20260901_2355_domain158/tsan/tsan.314667
```

目标进程：

```text
/home/oscar/ROS/my_r2d2/nav2_ws/install_sancov/nav2_planner/lib/nav2_planner/planner_server
```

对应二进制 / BuildId：

```text
/home/oscar/ROS/my_r2d2/nav2_ws/build_sancov/nav2_planner/planner_server
BuildId: c9eab7150b3b6d3907032315649ba3052a56c2bd
```

触发时运行命令核心参数：

```bash
export R2D2_PROFILE=sancov
export R2D2_SANCOV_FLUSH=signal
export ROS_DOMAIN_ID=158
export ROS2CLI_NO_DAEMON=1
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTDDS_BUILTIN_TRANSPORTS=UDPv4

cargo run --example nav2_costmap_e2e -- \
  --rounds 1000 \
  --benchmark-seconds 7200 \
  --seed 46 \
  --seed-dir config/nav2_seeds \
  --fresh-generation-period 1 \
  --fresh-selection shuffle-cycle \
  --round-duration 2.0 \
  --bridge-rate 20 \
  --tsan-log-dir /home/oscar/ROS/my_r2d2/outputs/fullstack_signal_full_20260901_2355_domain158/tsan \
  --sancov-dir /home/oscar/ROS/my_r2d2/outputs/fullstack_signal_full_20260901_2355_domain158/sancov
```

## TSan 报告摘要

原始报告中有多处同类 race。核心冲突如下：

```text
WARNING: ThreadSanitizer: data race (pid=314667)

Write of size 8 at 0x7fffd1589a58 by thread T21:
  planner_server+0x5f81f
  libnav2_costmap_2d_core.so+0x20ca45
  libnav2_costmap_2d_core.so+0x217c00
  libnav2_costmap_2d_core.so+0x257e4f
  libnav2_costmap_2d_core.so+0x24a7fa
  libnav2_costmap_2d_core.so+0x3269ef

Previous write of size 1 at 0x7fffd1589a5c by thread T25:
  libnav2_costmap_2d_core.so+0x20d3d8
  libnav2_navfn_planner.so+0x3222c
  libnav2_navfn_planner.so+0x304a2
  libplanner_server_core.so+0x10bdd8
  libplanner_server_core.so+0xfb613

Location is heap block of size 1684044 at 0x7fffd14c7000 allocated by thread T21.

SUMMARY: ThreadSanitizer: data race
```

符号化后的关键栈：

```text
Thread T21, costmap update / copy path:
  nav2_costmap_2d::Costmap2D::copyMapRegion<unsigned char>
    nav2_costmap_2d/include/nav2_costmap_2d/costmap_2d.hpp:441
  nav2_costmap_2d::Costmap2D::copyWindow
    nav2_costmap_2d/src/costmap_2d.cpp:203
  nav2_costmap_2d::LayeredCostmap::updateMap
    nav2_costmap_2d/src/layered_costmap.cpp:237
  nav2_costmap_2d::Costmap2DROS::updateMap
    nav2_costmap_2d/src/costmap_2d_ros.cpp:625
  nav2_costmap_2d::Costmap2DROS::mapUpdateLoop
    nav2_costmap_2d/src/costmap_2d_ros.cpp:568

Thread T25/T31, planner action path:
  nav2_costmap_2d::Costmap2D::setCost
    nav2_costmap_2d/src/costmap_2d.cpp:279
  nav2_navfn_planner::NavfnPlanner::clearRobotCell
    nav2_navfn_planner/src/navfn_planner.cpp:532
  nav2_navfn_planner::NavfnPlanner::makePlan
    nav2_navfn_planner/src/navfn_planner.cpp:247
  nav2_navfn_planner::NavfnPlanner::createPlan
    nav2_navfn_planner/src/navfn_planner.cpp:196
  nav2_planner::PlannerServer::getPlan
    nav2_planner/src/planner_server.cpp:655
  nav2_planner::PlannerServer::computePlanThroughPoses
    nav2_planner/src/planner_server.cpp:424
```

## 具体问题代码

`NavfnPlanner::makePlan()` 先写 costmap，再加 costmap mutex：

```cpp
// nav2_navfn_planner/src/navfn_planner.cpp

unsigned int mx, my;
worldToMap(wx, wy, mx, my);

// clear the starting cell within the costmap because we know it can't be an obstacle
clearRobotCell(mx, my);

std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> lock(*(costmap_->getMutex()));
```

`clearRobotCell()` 内部直接调用 `setCost()`：

```cpp
void
NavfnPlanner::clearRobotCell(unsigned int mx, unsigned int my)
{
  costmap_->setCost(mx, my, nav2_costmap_2d::FREE_SPACE);
}
```

`Costmap2D::setCost()` 直接写底层数组，没有内部加锁：

```cpp
void Costmap2D::setCost(unsigned int mx, unsigned int my, unsigned char cost)
{
  costmap_[getIndex(mx, my)] = cost;
}
```

另一边，costmap 更新线程在 `mapUpdateLoop()` 中持续调用 `updateMap()`，并可能在 `Costmap2D::copyMapRegion()` 中复制同一块 costmap buffer：

```cpp
// nav2_costmap_2d/include/nav2_costmap_2d/costmap_2d.hpp
memcpy(dm_index, sm_index, region_size_x * sizeof(data_type));
```

因此，核心问题不是 planner 自己的 `mutex_`，而是 `clearRobotCell()` 修改的是共享 `Costmap2D` 内存；这个修改应当和 costmap update / publish 线程使用同一把 `costmap_->getMutex()` 同步。

## 为什么认为是真 bug

1. TSan 冲突地址位于 `Costmap2D` 分配的 heap block，而不是测试框架内存。
2. 两个访问栈都符号化到 Nav2：
   - costmap update / publish 线程读取或复制 costmap；
   - planner action 线程通过 `NavfnPlanner::clearRobotCell()` 写 costmap。
3. `Costmap2D::setCost()` 本身不加锁，依赖调用者在外层持有 costmap mutex。
4. `NavfnPlanner::makePlan()` 的锁位置在 `clearRobotCell()` 之后，所以这一次写入没有受到 `costmap_->getMutex()` 保护。
5. 该路径可以由正常 ROS action 触发，即 `/compute_path_to_pose` 或 `/compute_path_through_poses`，不是非 ROS 内部调用。

## 影响分析

实际风险主要有三类：

- 规划结果不稳定：起点 cell 被 planner action 线程改写时，costmap update 线程可能正在复制或发布 costmap，导致 planner 看到非一致快照。
- 导航行为偶现异常：race 通常不会每次崩溃，但可能表现为偶发规划失败、路径为空、局部不可重复问题。
- 与其他 costmap 并发问题叠加：本项目同一轮运行里还观察到 controller/local_costmap 的 lock-order-inversion，说明 costmap 动态参数、插件初始化、更新线程之间已经存在较复杂的锁交互。

暂未观察到由该 race 直接导致的 crash；所以建议定为 Medium-High，而不是 Critical。若能进一步复现 hang、崩溃、或稳定错误路径，则严重性可以上调。

## 不依赖本项目的 ROS2 复现思路

下面命令用于给其他人复核。它不依赖 R2D2/fuzzer，但要求 Nav2 本身已经用 Clang + ThreadSanitizer 编译，并且保留 debug info。

### 1. 环境准备

```bash
source /opt/ros/<ros-distro>/setup.bash
source <nav2_tsan_ws>/install/setup.bash

export ROS_DOMAIN_ID=42
export ROS2CLI_NO_DAEMON=1
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTDDS_BUILTIN_TRANSPORTS=UDPv4

mkdir -p /tmp/nav2_planner_tsan_race
export TSAN_OPTIONS="halt_on_error=0:exitcode=0:history_size=7:second_deadlock_stack=1:detect_deadlocks=1:report_signal_unsafe=0:symbolize=0:log_path=/tmp/nav2_planner_tsan_race/tsan"
```

### 2. 创建最小 planner 参数文件

```bash
cat >/tmp/nav2_planner_navfn_race.yaml <<'YAML'
planner_server:
  ros__parameters:
    use_sim_time: false
    expected_planner_frequency: 20.0
    costmap_update_timeout: 1.0
    planner_plugins: ["GridBased"]
    GridBased:
      plugin: "nav2_navfn_planner/NavfnPlanner"
      tolerance: 0.5
      use_astar: false
      allow_unknown: true

global_costmap:
  global_costmap:
    ros__parameters:
      use_sim_time: false
      update_frequency: 10.0
      publish_frequency: 5.0
      global_frame: map
      robot_base_frame: base_link
      robot_radius: 0.22
      resolution: 0.05
      track_unknown_space: false
      plugins: ["static_layer", "obstacle_layer", "inflation_layer"]
      static_layer:
        plugin: "nav2_costmap_2d::StaticLayer"
        map_subscribe_transient_local: true
      obstacle_layer:
        plugin: "nav2_costmap_2d::ObstacleLayer"
        enabled: true
        observation_sources: scan
        scan:
          topic: scan
          max_obstacle_height: 2.0
          clearing: true
          marking: true
          data_type: "LaserScan"
          raytrace_max_range: 3.0
          raytrace_min_range: 0.0
          obstacle_max_range: 2.5
          obstacle_min_range: 0.0
      inflation_layer:
        plugin: "nav2_costmap_2d::InflationLayer"
        cost_scaling_factor: 3.0
        inflation_radius: 0.7
      always_send_full_costmap: true
YAML
```

### 3. 启动基础 ROS 输入

```bash
ros2 run tf2_ros static_transform_publisher \
  --x 0 --y 0 --z 0 --roll 0 --pitch 0 --yaw 0 \
  --frame-id map --child-frame-id odom &

ros2 run tf2_ros static_transform_publisher \
  --x 0 --y 0 --z 0 --roll 0 --pitch 0 --yaw 0 \
  --frame-id odom --child-frame-id base_link &

ros2 topic pub -r 5 /map nav_msgs/msg/OccupancyGrid \
"{header: {frame_id: 'map'},
  info: {resolution: 0.05, width: 100, height: 100,
    origin: {position: {x: -2.5, y: -2.5, z: 0.0},
      orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}},
  data: [0]}" &

ros2 topic pub -r 20 /scan sensor_msgs/msg/LaserScan \
"{header: {frame_id: 'base_link'},
  angle_min: -0.5, angle_max: 0.5, angle_increment: 0.25,
  time_increment: 0.0, scan_time: 0.05,
  range_min: 0.05, range_max: 10.0,
  ranges: [2.0, 2.0, 2.0, 2.0, 2.0],
  intensities: []}" &
```

如果本地 ROS CLI 不接受 `OccupancyGrid.data: [0]` 自动扩展，需要换成一个真实 10000 个元素的 map publisher，或直接用 `nav2_map_server map_server` 加载 yaml/pgm 地图。

### 4. 直接启动 TSan 版 planner_server

```bash
ros2 run nav2_planner planner_server \
  --ros-args \
  --params-file /tmp/nav2_planner_navfn_race.yaml \
  -p use_sim_time:=false
```

另开一个终端激活 lifecycle：

```bash
source /opt/ros/<ros-distro>/setup.bash
source <nav2_tsan_ws>/install/setup.bash
export ROS_DOMAIN_ID=42
export ROS2CLI_NO_DAEMON=1

ros2 lifecycle set /planner_server configure
ros2 lifecycle set /planner_server activate
```

### 5. 触发 planner action

循环发送 `/compute_path_to_pose` 或 `/compute_path_through_poses`。race 需要和 costmap update 线程重叠，因此建议连续发送多次：

```bash
for i in $(seq 1 200); do
  ros2 action send_goal /compute_path_to_pose nav2_msgs/action/ComputePathToPose \
  "{goal: {header: {frame_id: 'map'},
    pose: {position: {x: 1.0, y: 1.0, z: 0.0},
      orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}},
    start: {header: {frame_id: 'map'},
    pose: {position: {x: 0.0, y: 0.0, z: 0.0},
      orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}},
    planner_id: 'GridBased',
    use_start: true}"
done
```

成功触发后，TSan 文件会出现在：

```text
/tmp/nav2_planner_tsan_race/tsan.*
```

## 建议修复方向

最直接的修复方向是把 costmap mutex 覆盖到 `clearRobotCell()` 之前：

```cpp
worldToMap(wx, wy, mx, my);

std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> lock(*(costmap_->getMutex()));

// clear the starting cell within the costmap because we know it can't be an obstacle
clearRobotCell(mx, my);
```

但实际提交前需要确认：

- `worldToMap()` 是否也应在同一锁保护下读取 costmap metadata；
- `clearRobotCell()` 是否真的应该修改共享 master costmap，还是应该只修改 planner 的局部副本；
- 锁范围扩大后是否会和 planner 自身 `mutex_`、costmap update loop、dynamic parameter callback 产生新的锁顺序问题。

更稳妥的方向可能是：在持有 costmap mutex 时复制一份规划所需 costmap 快照，随后所有 Navfn 修改都只发生在局部副本上，避免 planner action 线程直接写共享 costmap。

