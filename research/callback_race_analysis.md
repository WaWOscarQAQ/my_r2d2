# ROS 2 Callback Race 专项分析

## 1. 总体结果

148 个严格 data race 中，133 个被标记为 `callback_race=yes`，占 89.9%。`strict=no/uncertain` 的 callback 竞态只保留在审计候选池，不进入本节数量或占比。

callback 是执行/触发维度，与 R1–R7 主根因重叠。一个 callback-lifecycle race 可以同时是 R5 和严格 data race；各 callback 子类之间互斥，但 callback 与根因比例不能相加。

| Callback 关系 | 严格 Data Race 数量 | 占全部 Callback 严格 Data Race |
|---|---:|---:|
| callback 与 callback 直接冲突（direct） | 51 | 38.3% |
| callback 与后台/驱动线程（callback_thread） | 58 | 43.6% |
| callback 与 lifecycle/shutdown/destructor（callback_lifecycle） | 21 | 15.8% |
| callback 触发异步工作（callback_indirect） | 3 | 2.3% |
| **合计** | **133** | **100%** |

## 2. 项目差异

- ros2_control、Autoware Universe、MoveIt 2 和 SLAM Toolbox 的严格 data race 均为 100% callback 参与，分别为 7/7、31/31、4/4 和 8/8。
- Navigation2 为 43/44（97.7%）。原先 20 位置的动态参数/lifecycle 讨论族中，19 个证据不足条目已从严格占比分母排除。
- rmf_ros2 为 6/9（66.7%）；纯 RxCpp executor 和 Asio websocket 内部线程不自动算作 ROS callback。
- rclcpp 为 25/34（73.5%）。
- rosbag2 为 9/11（81.8%）。
- robot_localization、image_pipeline 和 RTAB-Map 当前严格 data race 总数为 0，因此 callback 比例记为不适用，而不是 0%。

## 3. 典型失效链

### 3.1 Callback 与普通工作线程

最常见链路是 subscription/service/parameter callback 修改对象，而规划、控制、地图更新、录制或播放线程同时读取。典型修复包括：

- 对同一共享容器建立统一 mutex；
- 在 reader scope 保持 `shared_ptr` 快照；
- 将普通 flag 改为 atomic；
- 把外部操作投递到对象拥有的串行 worker。

### 3.2 Callback 与 lifecycle/shutdown

21 个 callback 严格 data race 涉及 cleanup、deactivate、shutdown、destructor 或 unload。常见问题不是“忘记 reset callback handle”这么简单，而是：

1. callback 仍注册在 node/middleware 中；
2. callback 已进入执行，reset handle 不等待它退出；
3. worker/executor 线程尚未 join；
4. callback 捕获裸 `this` 或 raw pointer；
5. 派生类成员先析构，而基类到更晚才清除 middleware listener。

Navigation2 [#4496](https://github.com/ros-navigation/navigation2/issues/4496)、rclcpp [#2024](https://github.com/ros2/rclcpp/pull/2024) 和 SLAM Toolbox [#691](https://github.com/SteveMacenski/slam_toolbox/issues/691) 分别体现了 callback 注册、middleware listener 和后台线程停机边界。

### 3.3 Callback 顺序和复合原子性

开发者所称的 callback race 不一定是严格内存 data race。robot_localization 的 timer 与 odometry/GPS callback 等顺序竞态仍有工程价值，但被排除在本节主统计之外。严格主统计中的复合原子性实例必须同时能够指出冲突内存位置、至少一个非原子写入或生命周期动作，以及缺失的 happens-before 关系。

## 4. ROS 2 设计含义

- Mutually Exclusive callback group 只约束同组 executor callback，不能保护 callback 与普通线程、不同 executor 或 lifecycle 操作。
- Single-Threaded Executor 也不能消除 callback 与后台线程或 middleware listener 的 race。
- `atomic<bool>` 只能保护单个变量，不能自动保证跨多个资源的 lifecycle 或 start/stop 事务。
- `shared_ptr` 成员本身需要安全发布；reader 应持有局部快照，避免 callback 替换成员后 pointee 被回收。
- teardown 的正确顺序通常是：阻止新 callback → 从注册表移除 → 停止/interrupt worker → join → 最后销毁 callback 使用的资源。

## 5. 后续可量化方向

建议下一轮在当前数据集上增加：callback group 类型、executor 类型、是否 reentrant、callback 是否 hidden、生产/测试代码、检测工具、修复机制等完整度指标。历史 issue 未记录这些字段时应保持空值，不从当前代码配置反推。
