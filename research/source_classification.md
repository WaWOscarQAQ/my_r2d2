# ROS 2 项目的 race 并发来源分类

查询与复核截止日期：2026-08-28。

## 1. 分类目的与口径

本文件对最初指定的八个项目以及扩展纳入的 Navigation2、rclcpp、rosbag2 做第二层分类。原有 R1–R7 继续描述“同步、原子性、顺序、生命周期”等技术缺陷；这里的 `source_class` 描述“哪一种并发执行来源使缺陷实际形成”。下方主汇总完整纳入十一个项目，共 148 个严格 data race。所有占比以严格 data race 总数为分母。

学术文献和标准没有把 ROS callback、项目自建 worker thread、lifecycle 固定为一套通用互斥 race taxonomy。C++ `[intro.races]`、CWE-362 以及 Lu 等人的并发缺陷研究用于判定和描述 race，但本文件的四类是针对 ROS 2 研究问题建立的可操作分类。为使项目汇总可以相加，采用以下互斥优先级：

1. `Lifecycle / Object-Lifetime Race`：启动、停止、取消、cleanup、deactivate、析构、对象替换或内存回收是 race 的必要条件；
2. `Project-Owned Worker-Thread Race`：排除 lifecycle 后，项目代码显式创建或拥有的线程/线程型调度器是必要参与者；
3. `Callback-Induced Race`：排除前两类后，ROS/Rx/action callback 的并发是主要触发源；
4. `Other Concurrency Race`：其余跨进程、普通 API 调用线程或无法归入前三类的 race。

`worker_thread` 的严格边界是项目主动创建或拥有执行线程，典型证据包括 `pthread_create`、`std::thread`、`boost::thread`、`std::jthread`、`std::async(std::launch::async)`，以及项目显式创建并持有的专用 event-loop/worker。ROS Executor 线程、DDS/middleware 内部线程、普通外部调用线程不自动计入 worker thread；ROS timer 仍是 callback。

为了不丢失 callback 专项信息，`callback_race` 继续作为独立、允许重叠的标签。下表中的 `Callback 主类` 是互斥分类；`Callback 参与` 则包含被 lifecycle 或 worker_thread 优先吸收、但仍有 callback 参与的实例。

## 2. 汇总结果

| 项目 | 严格 Data Race 总数 | Lifecycle / Object-Lifetime Race | Project-Owned Worker-Thread Race | Callback-Induced Race | Other Concurrency Race | Callback 参与（可重叠） |
|---|---:|---:|---:|---:|---:|---:|
| ros2_control | 7 | 3（42.9%） | 4（57.1%） | 0（0%） | 0（0%） | 7（100%） |
| Autoware Universe | 31 | 1（3.2%） | 1（3.2%） | 29（93.5%） | 0（0%） | 31（100%） |
| MoveIt 2 | 4 | 1（25.0%） | 2（50.0%） | 1（25.0%） | 0（0%） | 4（100%） |
| SLAM Toolbox | 8 | 3（37.5%） | 3（37.5%） | 2（25.0%） | 0（0%） | 8（100%） |
| robot_localization | 0 | 0（—） | 0（—） | 0（—） | 0（—） | 0（—） |
| rmf_ros2 | 9 | 4（44.4%） | 4（44.4%） | 1（11.1%） | 0（0%） | 6（66.7%） |
| image_pipeline | 0 | 0（—） | 0（—） | 0（—） | 0（—） | 0（—） |
| RTAB-Map | 0 | 0（—） | 0（—） | 0（—） | 0（—） | 0（—） |
| Navigation2 | 44 | 13（29.5%） | 23（52.3%） | 7（15.9%） | 1（2.3%） | 43（97.7%） |
| rclcpp | 34 | 13（38.2%） | 13（38.2%） | 7（20.6%） | 1（2.9%） | 25（73.5%） |
| rosbag2 | 11 | 4（36.4%） | 6（54.5%） | 1（9.1%） | 0（0%） | 9（81.8%） |
| **十一项目合计** | **148** | **42（28.4%）** | **56（37.8%）** | **48（32.4%）** | **2（1.4%）** | **133（89.9%）** |

四个互斥主类之和严格等于各项目 Race 总数。Callback 参与列不参与求和。

## 3. Worker thread 判定实例

- **Navigation2**：`Costmap2DROS` 明确拥有 map-update thread；[PR #3308](https://github.com/ros-navigation/navigation2/pull/3308) 直接列出 `stop_updates_`、`initialized_` 和 `stopped_` 在 map update loop 与控制路径之间的 race。`SimpleActionServer` 还通过 `std::async(std::launch::async)` 启动 action work，因此 action worker 与 callback/costmap worker 的冲突也满足定义。
- **Autoware Universe**：[PR #5532](https://github.com/autowarefoundation/autoware_universe/pull/5532) 明确删除 detached diagnostics thread，并将 `state_ptr_` 限制为只由 sensor callback 访问；这是该项目 31 个严格 data race 中唯一以项目自建 worker 为主类的实例。
- **MoveIt 2**：`PlanningSceneMonitor` 的 scene publishing thread、CollisionMonitor thread 和 Servo loop thread 满足定义；[ServoNode 源码](https://github.com/moveit/moveit2/blob/main/moveit_ros/moveit_servo/src/servo_node.cpp) 明确以 `std::thread(&ServoNode::servoLoop, this)` 创建并在析构时 join。
- **ros2_control**：`ros2_control_node.cpp` 显式创建 `cm_thread` 运行 read-update-write loop；异步 controller 还在 dedicated thread 上运行。实时循环与 diagnostics、robot_description 或 controller-list handoff 并发的四个位置归入 worker_thread。
- **SLAM Toolbox**：[issue #691](https://github.com/SteveMacenski/slam_toolbox/issues/691) 给出 `threads_.push_back(std::make_unique<boost::thread>(...))` 和退出时 UAF 栈；该问题因 teardown 优先归 lifecycle。scan queue、visualization 和 clearQueue 中不以 teardown 为核心的三项则归 worker_thread。
- **rmf_ros2**：代码显式保存 `rxcpp::schedulers::worker`，并以 `make_event_loop().create_worker()` 或 `worker.schedule(...)` 运行/串行化任务；[PR #129](https://github.com/open-rmf/rmf_ros2/pull/129)、[PR #228](https://github.com/open-rmf/rmf_ros2/pull/228) 和 [PR #273](https://github.com/open-rmf/rmf_ros2/pull/273) 的修复均可见这一执行模型。vendored RxCpp `rx-newthread.hpp` 的生产者/消费者冲突也计入项目拥有的线程型调度器。

## 4. Lifecycle 判定边界

`lifecycle` 不限于 ROS 2 Managed/LifecycleNode API，也包括普通 C++ 对象生命周期和异步任务所有权。以下情况优先归 lifecycle：

- callback 或 worker 尚在访问时执行 destructor、cleanup、deactivate、plugin unload 或 interface release；
- 一个线程替换/reset 共享 pointee，另一线程仍持有或解引用旧对象；
- stop/cancel/unsubscribe 与异步工作捕获对象的寿命缺少边界；
- runtime deserialization/remove 导致 dataset-backed 对象被回收，而 scan/callback 仍在访问。

仅仅“代码位于 lifecycle node 中”不够；必须有生命周期动作直接参与冲突。例如 Navigation2 的 costmap update loop 与 lifecycle activate/deactivate 之间的三个原子 flag race 归 lifecycle，而普通 costmap update worker 与规划读取之间的 race 归 worker_thread。

## 5. Callback 判定边界

互斥主类中的 callback 表示没有更高优先级来源，且 ROS subscription、timer、service、action、parameter、Rx event 或等价回调执行是形成并发的核心。Autoware 的“background timer”仍属于 timer callback，不因运行在线程池中而变成项目自建 worker。

独立 `callback_race` 标签采用更宽的参与口径：只要 callback 是直接冲突参与者、与 worker/lifecycle 冲突，或是必要的异步触发者，就标为 callback 参与。因此十一个项目有 133/148（89.9%）个严格 data race 涉及 callback，但只有 48/148（32.4%）个在互斥优先级下以 callback 为主类。

## 6. 可审计数据位置

- 分类覆盖文件：`research/data/execution_source_classification.json`；
- 十一个纳入项目每个严格实例的 ID、类型、原始 issue/PR、首次报告日期、标题、组件、文件路径、函数及访问双方：工作簿各 `Detail <project>` 明细表；修复状态、安全证书、完整证据等底层字段保存在 `research/data/*.json`；
- 公式驱动汇总：工作簿 `Source Class Summary`；
- 原技术根因：工作簿 `root_cause_category`（R1–R7）；
- callback 重叠专项：工作簿 `callback_race`、`callback_relation` 和 `Callback Analysis`。

每个纳入项目的严格 data race 必须且只能得到一个 `source_class`。验证脚本会检查全部 148 个严格实例是否完整覆盖、是否重复，以及四类总数是否等于项目严格 Data Race 总数；广义和证据不足候选不进入严格统计。
