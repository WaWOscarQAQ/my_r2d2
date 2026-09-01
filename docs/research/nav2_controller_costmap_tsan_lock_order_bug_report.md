# Nav2 controller_server / local_costmap TSan lock-order-inversion bug report

## 结论

ThreadSanitizer 在 Nav2 full-stack 运行中报告了一个 `lock-order-inversion (potential deadlock)`。该问题发生在 `controller_server` 进程内，涉及 `local_costmap` 的 costmap 更新线程、lifecycle configure / plugin initialize 路径，以及 rclcpp 参数服务路径。

建议报告定级：

```text
Severity: High
Type: Potential deadlock / lock-order inversion
Confidence: Medium-High
Affected process: controller_server
Affected component: nav2_controller + nav2_costmap_2d + rclcpp parameter service
```

这是一个潜在死锁问题，不是普通日志噪声。TSan 观察到三条真实锁顺序边，并组成闭环：

```text
_dynamic_parameter_mutex
  -> Costmap2D / LayeredCostmap mutex
  -> rclcpp::NodeParameters::mutex_
  -> _dynamic_parameter_mutex
```

如果三个执行路径在运行时重叠，`controller_server` 可能卡死，从而影响 Nav2 导航主链路。

## 发现环境

项目工作目录：

```text
/home/oscar/ROS/my_r2d2
```

TSan 原始报告：

```text
outputs/current_sancov_postclean_probe_20260901_domain145/tsan/tsan.94624
```

对应进程：

```text
/home/oscar/ROS/my_r2d2/nav2_ws/install_sancov/nav2_controller/lib/nav2_controller/controller_server
```

对应二进制：

```text
/home/oscar/ROS/my_r2d2/nav2_ws/build_sancov/nav2_controller/controller_server
BuildId: 698cb0d3057db266160b3886cf690e8ba4b391c2
```

## 脱离本项目的 ROS2 复现命令

下面这组命令不依赖 R2D2/fuzzer 项目，只依赖一个已经用 ThreadSanitizer 编译过的 Nav2 工作区。重点是直接用 `ros2 run nav2_controller controller_server` 启动目标进程，然后通过 ROS2 lifecycle / parameter service 触发。

前提：

- `nav2_controller`、`nav2_costmap_2d`、`rclcpp`、`rclcpp_lifecycle` 需要用同一套 Clang TSan 编译，并且保留 debug info。
- 如果使用普通 release Nav2 二进制，通常只能尝试观察 hang，不能稳定得到 TSan 报告。
- `ROS_DOMAIN_ID` 建议使用小于 232 的值，例如 `42`。
- 如果 `controller_server` 已经在链接阶段带入 TSan runtime，不要再对它额外 `LD_PRELOAD=libclang_rt.tsan...so`，否则可能出现启动后立刻退出或崩溃。
- 如果 `ros2` Python CLI 因为加载了 TSan 版 `libtracetools.so` 报 `undefined symbol: __tsan_read4`，让 CLI 终端使用普通 ROS 环境；目标 `controller_server` 进程使用 TSan 环境即可。

### 已验证方案：直接 `ros2 run nav2_controller controller_server`

这个节点内部会创建 `local_costmap`，因此不需要启动完整 Nav2 bringup 也能覆盖本报告中的关键锁顺序路径。

#### 1. 创建最小参数文件

```bash
cat >/tmp/nav2_controller_costmap_loi.yaml <<'YAML'
controller_server:
  ros__parameters:
    use_sim_time: false
    controller_frequency: 20.0
    costmap_update_timeout: 0.30
    min_x_velocity_threshold: 0.001
    min_y_velocity_threshold: 0.5
    min_theta_velocity_threshold: 0.001
    failure_tolerance: 0.3
    use_realtime_priority: false
    odom_topic: odom
    speed_limit_topic: speed_limit

    progress_checker_plugins: ["progress_checker"]
    goal_checker_plugins: ["general_goal_checker"]
    controller_plugins: ["FollowPath"]
    path_handler_plugins: ["PathHandler"]

    progress_checker:
      plugin: "nav2_controller::SimpleProgressChecker"
      required_movement_radius: 0.5
      movement_time_allowance: 10.0

    general_goal_checker:
      plugin: "nav2_controller::SimpleGoalChecker"
      xy_goal_tolerance: 0.25
      yaw_goal_tolerance: 0.25
      stateful: true

    PathHandler:
      plugin: "nav2_controller::FeasiblePathHandler"

    FollowPath:
      plugin: "nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController"

local_costmap:
  local_costmap:
    ros__parameters:
      use_sim_time: false
      update_frequency: 20.0
      publish_frequency: 5.0
      global_frame: odom
      robot_base_frame: base_link
      rolling_window: true
      width: 3
      height: 3
      resolution: 0.05
      robot_radius: 0.22
      always_send_full_costmap: true
      plugins: ["voxel_layer", "inflation_layer"]

      voxel_layer:
        plugin: "nav2_costmap_2d::VoxelLayer"
        enabled: true
        publish_voxel_map: true
        origin_z: 0.0
        z_resolution: 0.05
        z_voxels: 16
        max_obstacle_height: 2.0
        mark_threshold: 0
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
        inflation_radius: 0.70
YAML
```

#### 2. 启动合法 TF / odom / scan 输入

```bash
source /opt/ros/<ros-distro>/setup.bash

export ROS_DOMAIN_ID=42
export ROS2CLI_NO_DAEMON=1
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

ros2 run tf2_ros static_transform_publisher \
  --x 0 --y 0 --z 0 --roll 0 --pitch 0 --yaw 0 \
  --frame-id odom --child-frame-id base_footprint &

ros2 run tf2_ros static_transform_publisher \
  --x 0 --y 0 --z 0 --roll 0 --pitch 0 --yaw 0 \
  --frame-id base_footprint --child-frame-id base_link &

ros2 run tf2_ros static_transform_publisher \
  --x 0 --y 0 --z 0 --roll 0 --pitch 0 --yaw 0 \
  --frame-id base_link --child-frame-id base_scan &

ros2 topic pub -r 20 /odom nav_msgs/msg/Odometry \
"{header: {frame_id: 'odom'}, child_frame_id: 'base_footprint',
  pose: {pose: {position: {x: 0.0, y: 0.0, z: 0.0},
  orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}},
  twist: {twist: {linear: {x: 0.0, y: 0.0, z: 0.0},
  angular: {x: 0.0, y: 0.0, z: 0.0}}}}" &

ros2 topic pub -r 20 /scan sensor_msgs/msg/LaserScan \
"{header: {frame_id: 'base_scan'},
  angle_min: -0.2, angle_max: 0.2, angle_increment: 0.1,
  time_increment: 0.0, scan_time: 0.1,
  range_min: 0.05, range_max: 10.0,
  ranges: [1.0, 1.0, 1.0, 1.0, 1.0],
  intensities: []}" &
```

#### 3. 启动 TSan 版 `controller_server`

```bash
source /opt/ros/<ros-distro>/setup.bash
source <nav2_tsan_ws>/install/setup.bash

export ROS_DOMAIN_ID=42
export ROS2CLI_NO_DAEMON=1
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

mkdir -p /tmp/nav2_tsan_loi
export TSAN_OPTIONS="halt_on_error=0:exitcode=0:history_size=7:second_deadlock_stack=1:detect_deadlocks=1:report_signal_unsafe=0:symbolize=0:log_path=/tmp/nav2_tsan_loi/tsan"

ros2 run nav2_controller controller_server \
  --ros-args \
  --params-file /tmp/nav2_controller_costmap_loi.yaml \
  -p use_sim_time:=false \
  -r cmd_vel:=cmd_vel_nav
```

#### 4. 配置、激活 lifecycle，然后扰动参数服务

```bash
source /opt/ros/<ros-distro>/setup.bash

export ROS_DOMAIN_ID=42
export ROS2CLI_NO_DAEMON=1

ros2 lifecycle set /controller_server configure
ros2 lifecycle set /controller_server activate

ros2 lifecycle get /controller_server
ros2 lifecycle get /local_costmap/local_costmap

for i in $(seq 1 1000); do
  ros2 param set /local_costmap/local_costmap footprint_padding 0.01 >/dev/null
  ros2 param set /local_costmap/local_costmap footprint_padding 0.02 >/dev/null
  ros2 param set /local_costmap/local_costmap publish_frequency 5.0 >/dev/null
  ros2 param set /local_costmap/local_costmap publish_frequency 2.0 >/dev/null
done
```

完成后在 `controller_server` 终端中 `Ctrl-C` 退出，然后检查：

```bash
grep -R "WARNING: ThreadSanitizer\\|lock-order-inversion\\|potential deadlock" /tmp/nav2_tsan_loi
```

预期看到类似：

```text
WARNING: ThreadSanitizer: lock-order-inversion (potential deadlock)
Cycle in lock order graph: M0 => M1 => M2 => M0
```

### 本机独立验证记录

在本机 2026-09-01 的独立 `ros2 run` 复现中，未使用 fuzzer，只执行上面的 lifecycle + parameter 顺序，第 1 轮参数扰动即生成 TSan 报告：

```text
Transitioning successful
Transitioning successful
active [3]
active [3]
param-loop 1
tsan.149325 13869 bytes
```

报告路径：

```text
/tmp/nav2_tsan_loi_rosrun_overlay2/tsan.149325
```

核心内容：

```text
WARNING: ThreadSanitizer: lock-order-inversion (potential deadlock) (pid=149325)
Cycle in lock order graph: M0 (0x727000000ce8) => M1 (0x720c0001e660) => M2 (0x72480009af88) => M0
SUMMARY: ThreadSanitizer: lock-order-inversion (potential deadlock)
```

本机工作区布局中，TSan 版 rclcpp 位于单独 overlay。为了避免 `ros2` Python CLI 加载 TSan 版 `libtracetools.so` 后出现 `undefined symbol: __tsan_read4`，本机验证时采用了如下原则：

```text
目标进程 controller_server：source /opt/ros + nav2_tsan_ws + rclcpp_tsan_overlay
辅助 CLI / topic pub / lifecycle / param set：只 source 普通 ROS 环境，不加载 rclcpp_tsan_overlay
```

### 备选方案：full-stack bringup + 参数服务扰动

如果希望使用更接近完整导航系统的形态，也可以启动 full-stack Nav2，然后持续对 `local_costmap` 动态参数发 `ros2 param set`。命令结构如下：

```bash
source /opt/ros/<ros-distro>/setup.bash
source <nav2_tsan_ws>/install/setup.bash

export ROS_DOMAIN_ID=42
export ROS2CLI_NO_DAEMON=1
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

mkdir -p /tmp/nav2_tsan_loi
export TSAN_OPTIONS="halt_on_error=0:exitcode=0:history_size=7:second_deadlock_stack=1:detect_deadlocks=1:report_signal_unsafe=0:symbolize=0:log_path=/tmp/nav2_tsan_loi/tsan"

MAP="$(ros2 pkg prefix nav2_bringup)/share/nav2_bringup/maps/tb3_sandbox.yaml"
PARAMS="$(ros2 pkg prefix nav2_bringup)/share/nav2_bringup/params/nav2_params.yaml"

ros2 launch nav2_bringup bringup_launch.py \
  map:="$MAP" \
  params_file:="$PARAMS" \
  use_sim_time:=False \
  use_composition:=False \
  use_keepout_zones:=False \
  use_speed_zones:=False \
  autostart:=True
```

然后在另一个普通 ROS CLI 环境中执行：

```bash
export ROS_DOMAIN_ID=42
export ROS2CLI_NO_DAEMON=1

ros2 lifecycle get /controller_server
ros2 lifecycle get /local_costmap/local_costmap

for i in $(seq 1 1000); do
  ros2 param set /local_costmap/local_costmap footprint_padding 0.01 >/dev/null
  ros2 param set /local_costmap/local_costmap footprint_padding 0.02 >/dev/null
  ros2 param set /local_costmap/local_costmap publish_frequency 5.0 >/dev/null
  ros2 param set /local_costmap/local_costmap publish_frequency 2.0 >/dev/null
done
```

### 复现注意事项

该问题是 lock-order-inversion，触发依赖线程调度；不是每次运行都一定在同一秒打印。若第一次没有看到 TSan 报告，建议：

- 把参数扰动循环从 `1000` 增加到 `5000`；
- 保持 `/scan` 和 `/odom` 发布频率在 20Hz 左右；
- 保持 `local_costmap.update_frequency` 较高，例如 20Hz；
- 确保 `use_composition:=False`，让 `controller_server` 是独立 TSan 进程；
- 多运行 2 到 3 次。

## 本项目内复现运行命令

该报告来自如下 full-stack fuzz / sancov / tsan 运行：

```bash
cd /home/oscar/ROS/my_r2d2

RUN_OUT=outputs/current_sancov_postclean_probe_20260901_domain145
mkdir -p "$RUN_OUT/tsan" "$RUN_OUT/sancov"

R2D2_PROFILE=sancov \
ROS_DOMAIN_ID=145 \
R2D2_STACK_STARTUP_TIMEOUT_SEC=600 \
timeout --signal=INT --kill-after=30s 25m \
cargo run --example nav2_costmap_e2e -- \
  --rounds 8 \
  --benchmark-seconds 5 \
  --seed-dir config/nav2_seeds \
  --fresh-generation-period 1 \
  --fresh-selection shuffle-cycle \
  --round-duration 1.0 \
  --bridge-rate 20 \
  --tsan-log-dir "$RUN_OUT/tsan" \
  --sancov-dir "$RUN_OUT/sancov" \
  2>&1 | tee "$RUN_OUT/run.log"
```

运行本身没有观测到 crash：

```text
rounds=8
crashes=0
active_new_states=8
invalid_traces=0
empty_rounds=0
sancov_covered_pcs=49775
```

但是 `controller_server` 在 TSan 日志中报告了潜在死锁。

## TSan 原始报告核心段

下面是 `tsan.94624` 中的核心 warning 段。原文件前后还有大量 `SanitizerCoverage` 输出；这里保留与 lock-order-inversion 直接相关的部分。

```text
==================
WARNING: ThreadSanitizer: lock-order-inversion (potential deadlock) (pid=94624)
  Cycle in lock order graph: M0 (0x727000000ce8) => M1 (0x720c00029130) => M2 (0x72480009ae08) => M0

  Mutex M1 acquired here while holding mutex M0 in thread T21:
    #0 <null> <null> (controller_server+0x6541e) (BuildId: 698cb0d3057db266160b3886cf690e8ba4b391c2)
    #1 <null> <null> (libnav2_costmap_2d_core.so+0x214ca1) (BuildId: 835a316ddefb413d7236b8f0d3dead43965147cf)
    #2 <null> <null> (libnav2_costmap_2d_core.so+0x257e4f) (BuildId: 835a316ddefb413d7236b8f0d3dead43965147cf)
    #3 <null> <null> (libnav2_costmap_2d_core.so+0x24a7fa) (BuildId: 835a316ddefb413d7236b8f0d3dead43965147cf)
    #4 <null> <null> (libnav2_costmap_2d_core.so+0x3269ef) (BuildId: 835a316ddefb413d7236b8f0d3dead43965147cf)
    #5 <null> <null> (libstdc++.so.6+0xf5ef8) (BuildId: 3a7a8c6bd922e6314edb7a710fdc0293265c2c42)

  Mutex M0 previously acquired by the same thread here:
    #0 <null> <null> (controller_server+0x6541e) (BuildId: 698cb0d3057db266160b3886cf690e8ba4b391c2)
    #1 <null> <null> (libnav2_costmap_2d_core.so+0x24a7e0) (BuildId: 835a316ddefb413d7236b8f0d3dead43965147cf)
    #2 <null> <null> (libnav2_costmap_2d_core.so+0x3269ef) (BuildId: 835a316ddefb413d7236b8f0d3dead43965147cf)
    #3 <null> <null> (libstdc++.so.6+0xf5ef8) (BuildId: 3a7a8c6bd922e6314edb7a710fdc0293265c2c42)

  Mutex M2 acquired here while holding mutex M1 in main thread:
    #0 <null> <null> (controller_server+0x6541e) (BuildId: 698cb0d3057db266160b3886cf690e8ba4b391c2)
    #1 <null> <null> (librclcpp.so+0x469851) (BuildId: d32cc7f664b628d72410a5dc80a7aafbb271fa7d)
    #2 <null> <null> (librclcpp_lifecycle.so+0x51548) (BuildId: 87a73a3b940596ff7daa670aaddda2bebd8fb5b4)
    #3 <null> <null> (libcontroller_server_core.so+0x2af09d) (BuildId: 51be9d48d0910e991f11c3256ae6426bc5fef123)
    #4 <null> <null> (liblayers.so+0x3aa587) (BuildId: 99ea2073396387bf22d1cdf0e3e77850f63bc0fc)
    #5 <null> <null> (liblayers.so+0x43038b) (BuildId: 99ea2073396387bf22d1cdf0e3e77850f63bc0fc)
    #6 <null> <null> (libnav2_costmap_2d_core.so+0x211141) (BuildId: 835a316ddefb413d7236b8f0d3dead43965147cf)
    #7 <null> <null> (libnav2_costmap_2d_core.so+0x226fa8) (BuildId: 835a316ddefb413d7236b8f0d3dead43965147cf)
    #8 <null> <null> (librclcpp_lifecycle.so+0x586ce) (BuildId: 87a73a3b940596ff7daa670aaddda2bebd8fb5b4)
    #9 <null> <null> (librclcpp_lifecycle.so+0x646e7) (BuildId: 87a73a3b940596ff7daa670aaddda2bebd8fb5b4)
    #10 <null> <null> (librclcpp_lifecycle.so+0x60bd7) (BuildId: 87a73a3b940596ff7daa670aaddda2bebd8fb5b4)
    #11 <null> <null> (librclcpp_lifecycle.so+0x651af) (BuildId: 87a73a3b940596ff7daa670aaddda2bebd8fb5b4)
    #12 <null> <null> (librclcpp_lifecycle.so+0x5516a) (BuildId: 87a73a3b940596ff7daa670aaddda2bebd8fb5b4)
    #13 <null> <null> (libcontroller_server_core.so+0x20d8dc) (BuildId: 51be9d48d0910e991f11c3256ae6426bc5fef123)
    #14 <null> <null> (librclcpp_lifecycle.so+0x586ce) (BuildId: 87a73a3b940596ff7daa670aaddda2bebd8fb5b4)
    #15 <null> <null> (librclcpp_lifecycle.so+0x646e7) (BuildId: 87a73a3b940596ff7daa670aaddda2bebd8fb5b4)
    #16 <null> <null> (librclcpp_lifecycle.so+0x60bd7) (BuildId: 87a73a3b940596ff7daa670aaddda2bebd8fb5b4)
    #17 <null> <null> (librclcpp_lifecycle.so+0x5eeb5) (BuildId: 87a73a3b940596ff7daa670aaddda2bebd8fb5b4)
    #18 <null> <null> (librclcpp_lifecycle.so+0x69617) (BuildId: 87a73a3b940596ff7daa670aaddda2bebd8fb5b4)
    #19 <null> <null> (librclcpp_lifecycle.so+0x69359) (BuildId: 87a73a3b940596ff7daa670aaddda2bebd8fb5b4)
    #20 <null> <null> (librclcpp_lifecycle.so+0x6f6ef) (BuildId: 87a73a3b940596ff7daa670aaddda2bebd8fb5b4)
    #21 <null> <null> (librclcpp_lifecycle.so+0x6c45c) (BuildId: 87a73a3b940596ff7daa670aaddda2bebd8fb5b4)
    #22 <null> <null> (librclcpp.so+0x337ff3) (BuildId: d32cc7f664b628d72410a5dc80a7aafbb271fa7d)
    #23 <null> <null> (librclcpp.so+0x33345e) (BuildId: d32cc7f664b628d72410a5dc80a7aafbb271fa7d)
    #24 <null> <null> (librclcpp.so+0x3cc558) (BuildId: d32cc7f664b628d72410a5dc80a7aafbb271fa7d)
    #25 <null> <null> (librclcpp.so+0x36c4b4) (BuildId: d32cc7f664b628d72410a5dc80a7aafbb271fa7d)
    #26 <null> <null> (controller_server+0xe3c8c) (BuildId: 698cb0d3057db266160b3886cf690e8ba4b391c2)
    #27 <null> <null> (libc.so.6+0x2a600) (BuildId: 240c8909736b31f963346aca80667fd00c551e32)

  Mutex M1 previously acquired by the same thread here:
    #0 <null> <null> (controller_server+0x6541e) (BuildId: 698cb0d3057db266160b3886cf690e8ba4b391c2)
    #1 <null> <null> (libnav2_costmap_2d_core.so+0x226a88) (BuildId: 835a316ddefb413d7236b8f0d3dead43965147cf)
    #2 <null> <null> (librclcpp_lifecycle.so+0x586ce) (BuildId: 87a73a3b940596ff7daa670aaddda2bebd8fb5b4)
    #3 <null> <null> (librclcpp_lifecycle.so+0x646e7) (BuildId: 87a73a3b940596ff7daa670aaddda2bebd8fb5b4)
    #4 <null> <null> (librclcpp_lifecycle.so+0x60bd7) (BuildId: 87a73a3b940596ff7daa670aaddda2bebd8fb5b4)
    #5 <null> <null> (librclcpp_lifecycle.so+0x651af) (BuildId: 87a73a3b940596ff7daa670aaddda2bebd8fb5b4)
    #6 <null> <null> (librclcpp_lifecycle.so+0x5516a) (BuildId: 87a73a3b940596ff7daa670aaddda2bebd8fb5b4)
    #7 <null> <null> (libcontroller_server_core.so+0x20d8dc) (BuildId: 51be9d48d0910e991f11c3256ae6426bc5fef123)
    #8 <null> <null> (librclcpp_lifecycle.so+0x586ce) (BuildId: 87a73a3b940596ff7daa670aaddda2bebd8fb5b4)
    #9 <null> <null> (librclcpp_lifecycle.so+0x646e7) (BuildId: 87a73a3b940596ff7daa670aaddda2bebd8fb5b4)
    #10 <null> <null> (librclcpp_lifecycle.so+0x60bd7) (BuildId: 87a73a3b940596ff7daa670aaddda2bebd8fb5b4)
    #11 <null> <null> (librclcpp_lifecycle.so+0x5eeb5) (BuildId: 87a73a3b940596ff7daa670aaddda2bebd8fb5b4)
    #12 <null> <null> (librclcpp_lifecycle.so+0x69617) (BuildId: 87a73a3b940596ff7daa670aaddda2bebd8fb5b4)
    #13 <null> <null> (librclcpp_lifecycle.so+0x69359) (BuildId: 87a73a3b940596ff7daa670aaddda2bebd8fb5b4)
    #14 <null> <null> (librclcpp_lifecycle.so+0x6f6ef) (BuildId: 87a73a3b940596ff7daa670aaddda2bebd8fb5b4)
    #15 <null> <null> (librclcpp_lifecycle.so+0x6c45c) (BuildId: 87a73a3b940596ff7daa670aaddda2bebd8fb5b4)
    #16 <null> <null> (librclcpp.so+0x337ff3) (BuildId: d32cc7f664b628d72410a5dc80a7aafbb271fa7d)
    #17 <null> <null> (librclcpp.so+0x33345e) (BuildId: d32cc7f664b628d72410a5dc80a7aafbb271fa7d)
    #18 <null> <null> (librclcpp.so+0x3cc558) (BuildId: d32cc7f664b628d72410a5dc80a7aafbb271fa7d)
    #19 <null> <null> (librclcpp.so+0x36c4b4) (BuildId: d32cc7f664b628d72410a5dc80a7aafbb271fa7d)
    #20 <null> <null> (controller_server+0xe3c8c) (BuildId: 698cb0d3057db266160b3886cf690e8ba4b391c2)
    #21 <null> <null> (libc.so.6+0x2a600) (BuildId: 240c8909736b31f963346aca80667fd00c551e32)

  Mutex M0 acquired here while holding mutex M2 in thread T18:
    #0 <null> <null> (controller_server+0x6541e) (BuildId: 698cb0d3057db266160b3886cf690e8ba4b391c2)
    #1 <null> <null> (libnav2_costmap_2d_core.so+0x24e1e5) (BuildId: 835a316ddefb413d7236b8f0d3dead43965147cf)
    #2 <null> <null> (libnav2_costmap_2d_core.so+0x326a9e) (BuildId: 835a316ddefb413d7236b8f0d3dead43965147cf)
    #3 <null> <null> (librclcpp.so+0x46474f) (BuildId: d32cc7f664b628d72410a5dc80a7aafbb271fa7d)
    #4 <null> <null> (librclcpp.so+0x4652ef) (BuildId: d32cc7f664b628d72410a5dc80a7aafbb271fa7d)
    #5 <null> <null> (librclcpp.so+0x46c052) (BuildId: d32cc7f664b628d72410a5dc80a7aafbb271fa7d)
    #6 <null> <null> (librclcpp.so+0x54a2c2) (BuildId: d32cc7f664b628d72410a5dc80a7aafbb271fa7d)
    #7 <null> <null> (librclcpp.so+0x5523e1) (BuildId: d32cc7f664b628d72410a5dc80a7aafbb271fa7d)
    #8 <null> <null> (librclcpp.so+0x54d7cc) (BuildId: d32cc7f664b628d72410a5dc80a7aafbb271fa7d)
    #9 <null> <null> (librclcpp.so+0x337ff3) (BuildId: d32cc7f664b628d72410a5dc80a7aafbb271fa7d)
    #10 <null> <null> (librclcpp.so+0x33345e) (BuildId: d32cc7f664b628d72410a5dc80a7aafbb271fa7d)
    #11 <null> <null> (librclcpp.so+0x3cc558) (BuildId: d32cc7f664b628d72410a5dc80a7aafbb271fa7d)
    #12 <null> <null> (libcontroller_server_core.so+0x2a3af3) (BuildId: 51be9d48d0910e991f11c3256ae6426bc5fef123)
    #13 <null> <null> (libstdc++.so.6+0xf5ef8) (BuildId: 3a7a8c6bd922e6314edb7a710fdc0293265c2c42)

  Mutex M2 previously acquired by the same thread here:
    #0 <null> <null> (controller_server+0x6541e) (BuildId: 698cb0d3057db266160b3886cf690e8ba4b391c2)
    #1 <null> <null> (librclcpp.so+0x46a41d) (BuildId: d32cc7f664b628d72410a5dc80a7aafbb271fa7d)
    #2 <null> <null> (librclcpp.so+0x54a2c2) (BuildId: d32cc7f664b628d72410a5dc80a7aafbb271fa7d)
    #3 <null> <null> (librclcpp.so+0x5523e1) (BuildId: d32cc7f664b628d72410a5dc80a7aafbb271fa7d)
    #4 <null> <null> (librclcpp.so+0x54d7cc) (BuildId: d32cc7f664b628d72410a5dc80a7aafbb271fa7d)
    #5 <null> <null> (librclcpp.so+0x337ff3) (BuildId: d32cc7f664b628d72410a5dc80a7aafbb271fa7d)
    #6 <null> <null> (librclcpp.so+0x33345e) (BuildId: d32cc7f664b628d72410a5dc80a7aafbb271fa7d)
    #7 <null> <null> (librclcpp.so+0x3cc558) (BuildId: d32cc7f664b628d72410a5dc80a7aafbb271fa7d)
    #8 <null> <null> (libcontroller_server_core.so+0x2a3af3) (BuildId: 51be9d48d0910e991f11c3256ae6426bc5fef123)
    #9 <null> <null> (libstdc++.so.6+0xf5ef8) (BuildId: 3a7a8c6bd922e6314edb7a710fdc0293265c2c42)

SUMMARY: ThreadSanitizer: lock-order-inversion (potential deadlock) (/home/oscar/ROS/my_r2d2/nav2_ws/build_sancov/nav2_controller/controller_server+0x6541e) (BuildId: 698cb0d3057db266160b3886cf690e8ba4b391c2)
==================
```

## 符号化结果

使用本地带 debug info 的 sancov 构建产物进行符号化后，关键 offset 对应关系如下。

### `M0 -> M1`: costmap 更新线程

TSan 段落：

```text
Mutex M1 acquired here while holding mutex M0 in thread T21
```

符号化后：

```text
libnav2_costmap_2d_core.so+0x24a7e0
  nav2_costmap_2d::Costmap2DROS::mapUpdateLoop(double)
  nav2_costmap_2d/src/costmap_2d_ros.cpp:564

libnav2_costmap_2d_core.so+0x257e4f
  nav2_costmap_2d::Costmap2DROS::updateMap()
  nav2_costmap_2d/src/costmap_2d_ros.cpp:625

libnav2_costmap_2d_core.so+0x214ca1
  nav2_costmap_2d::LayeredCostmap::updateMap(double, double, double)
  nav2_costmap_2d/src/layered_costmap.cpp:140
```

源码：

```cpp
// nav2_costmap_2d/src/costmap_2d_ros.cpp
void Costmap2DROS::mapUpdateLoop(double frequency)
{
  ...
  if (!stopped_) {
    // Lock while modifying layered costmap and publishing values
    std::scoped_lock<std::mutex> lock(_dynamic_parameter_mutex);

    timer.start();
    updateMap();
    timer.end();
    ...
  }
}
```

```cpp
// nav2_costmap_2d/src/costmap_2d_ros.cpp
void Costmap2DROS::updateMap()
{
  ...
  if (getRobotPose(pose)) {
    ...
    layered_costmap_->updateMap(x, y, yaw);
    ...
  }
}
```

```cpp
// nav2_costmap_2d/src/layered_costmap.cpp
void LayeredCostmap::updateMap(double robot_x, double robot_y, double robot_yaw)
{
  // Lock for the remainder of this function, some plugins
  // implement thread unsafe updateBounds() functions.
  std::unique_lock<Costmap2D::mutex_t> lock(*(combined_costmap_.getMutex()));
  ...
}
```

因此该路径形成：

```text
M0 -> M1
_dynamic_parameter_mutex -> Costmap2D / LayeredCostmap mutex
```

### `M1 -> M2`: lifecycle configure / plugin initialize 路径

TSan 段落：

```text
Mutex M2 acquired here while holding mutex M1 in main thread
```

符号化后：

```text
libnav2_costmap_2d_core.so+0x226a88
  nav2_costmap_2d::Costmap2DROS::on_configure(...)
  nav2_costmap_2d/src/costmap_2d_ros.cpp:164

libnav2_costmap_2d_core.so+0x226fa8
  nav2_costmap_2d::Costmap2DROS::on_configure(...)
  nav2_costmap_2d/src/costmap_2d_ros.cpp:169

liblayers.so+0x3aa587
  nav2_costmap_2d::ObstacleLayer::onInitialize()
  nav2_costmap_2d/plugins/obstacle_layer.cpp:87

liblayers.so+0x43038b
  nav2_costmap_2d::VoxelLayer::onInitialize()
  nav2_costmap_2d/plugins/voxel_layer.cpp:63

librclcpp.so+0x469851
  rclcpp::node_interfaces::NodeParameters::has_parameter(...)
  rclcpp/src/rclcpp/node_interfaces/node_parameters.cpp:715
```

源码：

```cpp
// nav2_costmap_2d/src/costmap_2d_ros.cpp
for (unsigned int i = 0; i < plugin_names_.size(); ++i) {
  std::shared_ptr<Layer> plugin = plugin_loader_.createSharedInstance(plugin_types_[i]);

  // lock the costmap because no update is allowed until the plugin is initialized
  std::unique_lock<Costmap2D::mutex_t> lock(*(layered_costmap_->getCostmap()->getMutex()));

  layered_costmap_->addPlugin(plugin);

  plugin->initialize(
    layered_costmap_.get(), plugin_names_[i], tf_buffer_.get(),
    shared_from_this(), callback_group_);

  lock.unlock();
}
```

```cpp
// nav2_costmap_2d/plugins/voxel_layer.cpp
void VoxelLayer::onInitialize()
{
  ObstacleLayer::onInitialize();
  ...
}
```

```cpp
// nav2_costmap_2d/plugins/obstacle_layer.cpp
void ObstacleLayer::onInitialize()
{
  ...
  allow_parameter_qos_overrides_ = nav2::declare_or_get_parameter(node,
    "allow_parameter_qos_overrides", true);
  ...
}
```

```cpp
// nav2_ros_common/nav2_ros_common/node_utils.hpp
inline void declare_parameter_if_not_declared(...)
{
  if (!node->has_parameter(parameter_name)) {
    node->declare_parameter(parameter_name, default_value, parameter_descriptor);
  }
}
```

```cpp
// rclcpp/src/rclcpp/node_interfaces/node_parameters.cpp
bool NodeParameters::has_parameter(const std::string & name) const
{
  std::lock_guard<std::recursive_mutex> lock(mutex_);

  return __lockless_has_parameter(parameters_, name);
}
```

因此该路径形成：

```text
M1 -> M2
Costmap2D / LayeredCostmap mutex -> rclcpp::NodeParameters::mutex_
```

### `M2 -> M0`: 运行时参数服务路径

TSan 段落：

```text
Mutex M0 acquired here while holding mutex M2 in thread T18
```

符号化后：

```text
librclcpp.so+0x46a41d
  rclcpp::node_interfaces::NodeParameters::set_parameters_atomically(...)
  rclcpp/src/rclcpp/node_interfaces/node_parameters.cpp:749

librclcpp.so+0x4652ef
  __set_parameters_atomically_common(...)
  rclcpp/src/rclcpp/node_interfaces/node_parameters.cpp:478

libnav2_costmap_2d_core.so+0x24e1e5
  nav2_costmap_2d::Costmap2DROS::updateParametersCallback(...)
  nav2_costmap_2d/src/costmap_2d_ros.cpp:843
```

源码：

```cpp
// rclcpp/src/rclcpp/node_interfaces/node_parameters.cpp
rcl_interfaces::msg::SetParametersResult
NodeParameters::set_parameters_atomically(const std::vector<rclcpp::Parameter> & parameters)
{
  std::lock_guard<std::recursive_mutex> lock(mutex_);

  ParameterMutationRecursionGuard guard(parameter_modification_enabled_);
  ...
}
```

```cpp
// rclcpp/src/rclcpp/node_interfaces/node_parameters.cpp
if (result.successful) {
  ...
  // Call the user post set parameter callback
  __call_post_set_parameters_callbacks(parameters, post_set_callback_container);
}
```

```cpp
// nav2_costmap_2d/src/costmap_2d_ros.cpp
void
Costmap2DROS::updateParametersCallback(const std::vector<rclcpp::Parameter> & parameters)
{
  bool resize_map = false;
  std::lock_guard<std::mutex> lock_reinit(_dynamic_parameter_mutex);

  for (const auto & parameter : parameters) {
    ...
  }
}
```

因此该路径形成：

```text
M2 -> M0
rclcpp::NodeParameters::mutex_ -> _dynamic_parameter_mutex
```

## 锁顺序闭环

三条边合并后得到：

```text
1. costmap update thread:
   _dynamic_parameter_mutex -> Costmap2D mutex

2. lifecycle configure / plugin initialize:
   Costmap2D mutex -> rclcpp parameter mutex

3. parameter service / post-set callback:
   rclcpp parameter mutex -> _dynamic_parameter_mutex
```

也就是：

```text
M0 -> M1 -> M2 -> M0
```

一种可能的死锁 interleaving：

```text
Thread A:
  holds _dynamic_parameter_mutex
  waits for Costmap2D mutex

Thread B:
  holds Costmap2D mutex
  waits for rclcpp::NodeParameters::mutex_

Thread C:
  holds rclcpp::NodeParameters::mutex_
  waits for _dynamic_parameter_mutex
```

三者互相等待时，`controller_server` 将无法继续推进。

## 影响分析

影响范围：

- `controller_server`；
- `local_costmap`；
- costmap update loop；
- lifecycle configure / activate；
- dynamic parameter service；
- costmap plugin initialization，例如 `VoxelLayer` / `ObstacleLayer`。

潜在影响：

- `controller_server` 卡住；
- `local_costmap` 不再更新；
- 参数服务调用不返回；
- lifecycle transition 卡住；
- Nav2 导航链路失去响应。

该问题不要求恶意输入才能触发。只要运行时参数更新、costmap 更新线程、lifecycle 或插件初始化路径发生并发重叠，就可能满足死锁条件。fuzzer 的价值在于提高了这些路径重叠的概率。

## 为什么不是 fuzzer 自身问题

TSan 报告中的锁全部来自 Nav2 / rclcpp C++ 运行时：

- `nav2_costmap_2d::Costmap2DROS::mapUpdateLoop`
- `nav2_costmap_2d::LayeredCostmap::updateMap`
- `nav2_costmap_2d::Costmap2DROS::on_configure`
- `nav2_costmap_2d::ObstacleLayer::onInitialize`
- `rclcpp::node_interfaces::NodeParameters::has_parameter`
- `rclcpp::node_interfaces::NodeParameters::set_parameters_atomically`
- `nav2_costmap_2d::Costmap2DROS::updateParametersCallback`

Rust fuzzer 只是通过合法 ROS2 topic / service / action / parameter 输入驱动 Nav2 full stack；它没有直接访问这些 C++ mutex，也没有在自身代码中创建该锁环。

## 修复方向建议

建议从 Nav2 / rclcpp 交互层面统一锁顺序，避免在持有一个子系统锁时调用会进入另一个锁域的函数。

可能方向：

1. 避免在持有 `Costmap2D` mutex 时调用 `plugin->initialize()`。
   - 现在 `Costmap2DROS::on_configure()` 在 costmap mutex 内调用插件初始化。
   - 插件初始化会声明 / 查询参数，从而进入 rclcpp parameter mutex。

2. 避免在 `rclcpp::NodeParameters::mutex_` 持有期间调用用户 post-set callback。
   - 当前 `set_parameters_atomically()` 路径会在参数锁内调用 post-set callback。
   - Nav2 post-set callback 又会进入 `_dynamic_parameter_mutex`。

3. 缩小 `_dynamic_parameter_mutex` 的持有范围。
   - `mapUpdateLoop()` 当前在持有 `_dynamic_parameter_mutex` 的情况下执行 `updateMap()`。
   - `updateMap()` 会进入 `LayeredCostmap::updateMap()` 并拿 costmap mutex。

4. 明确全局锁顺序。
   - 例如强制所有路径都遵守：

```text
rclcpp parameter mutex -> _dynamic_parameter_mutex -> Costmap2D mutex
```

或其他固定顺序，但不能出现反向获取。

## 当前状态

该报告目前应作为：

```text
Confirmed lock-order inversion
Potential deadlock
Not yet a confirmed runtime hang
```

也就是说，TSan 已确认源码路径形成死锁环；但本次运行没有直接观测到 `controller_server` 永久挂死。因此报告措辞建议使用：

```text
ThreadSanitizer detected a real lock-order cycle that may lead to deadlock.
```

而不要写成：

```text
The program always deadlocks.
```
