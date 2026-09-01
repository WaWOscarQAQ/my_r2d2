# ROS 2 Data Race 代表案例

## 1. Navigation2：动态参数 callback 与 lifecycle teardown

[Issue #4496](https://github.com/ros-navigation/navigation2/issues/4496) 的 AMCL ASan UAF 表明，reset callback handle 并不等于从 parameter registry 中注销 callback。维护者随后要求全仓审计，[PR #4521](https://github.com/ros-navigation/navigation2/pull/4521) 在 20 个独立组件/handler 位置先调用 `remove_on_set_parameters_callback`，再 reset handle 和销毁资源。

主根因为 R5。AMCL 实例严格判定为 yes；其他 19 个位置有维护者接受的代码审计证据，但缺少逐对象运行时 trace，因此严格性标为 uncertain。这一族也构成最重要的计数敏感性：对象级为 20，讨论族级为 1。

## 2. rclcpp：middleware listener 与对象析构

[PR #2024](https://github.com/ros2/rclcpp/pull/2024) 修复 ClientBase、ServiceBase 和 SubscriptionBase 三个独立 rmw callback/function/handle 对。旧成员析构顺序允许 middleware listener 在 `std::function` target 或 rcl handle 已进入析构时继续调用。

主根因为 R5，三项均为 strict=yes、callback-lifecycle。修复原则是先从 middleware 清除 callback，再销毁 callback target 和 handle。EventHandler 的 [PR #2102](https://github.com/ros2/rclcpp/pull/2102) 与 [PR #2349](https://github.com/ros2/rclcpp/pull/2349) 是同一对象族的早期修复和后续补漏，因此去重为一个实例。

## 3. rosbag2：一个修复覆盖数据库对象与容器

[Issue #602](https://github.com/ros2/rosbag2/issues/602) / [PR #603](https://github.com/ros2/rosbag2/pull/603) 同时修复：

1. SQLite connection/write statement 的并发数据库操作；
2. `topics_` unordered_map 的查找与插入/删除。

两者共享讨论和 mutex，但属于不同共享对象和冲突位置，因此拆为两个 strict data race。对应修复增加数据库/容器锁，并用 thread-safety annotations 标明 `GUARDED_BY`。

## 4. SLAM Toolbox：service callback 漏用已有锁

[Issue #850](https://github.com/SteveMacenski/slam_toolbox/issues/850) / [PR #853](https://github.com/SteveMacenski/slam_toolbox/pull/853) 中，queue push/pop 已使用 `q_mutex_`，但 `clearQueueCallback` 漏锁，与后台 `run()` 的 pop 并发，ASan 报告 `LaserScan.ranges` double-free。

主根因为 R2，而不是 R1：项目已经设计了正确 mutex，只是 service callback 路径没有覆盖。这个边界用于统一区分“完全无同步设计”和“单边/漏路径加锁”。

## 5. robot_localization：callback race 不等于 data race

[Issue #820](https://github.com/cra-ros-pkg/robot_localization/issues/820) / [PR #821](https://github.com/cra-ros-pkg/robot_localization/pull/821) 中，manual datum 模式的 timer callback 可先于 odometry callback，导致 frame 状态未初始化就计算 transform。

开发者明确称其为 race，但公开证据只支持 callback 顺序违反，因此标为 R4、strict=no、callback=yes。这个案例说明必须同时保留“开发者 race condition”与 C++ 严格 data race 两条统计轴。

## 6. Autoware Universe：批量成员变量修复的粒度选择

Autoware Universe 多个 PR 将 callback 与规划/控制线程共享的成员变量改为 atomic 或加入 mutex。主统计按逻辑状态对象计数，而不是机械按 diff 每一行计数。特别是 PR #6718，主口径把 GoalPlanner 状态按 7 个逻辑对象计；字段级替代口径会使项目总数从 32 增至 36。

这种敏感性记录避免了两种偏差：按 issue 计数导致低估；按每个字段机械计数导致虚假精度。
