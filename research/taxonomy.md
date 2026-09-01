# ROS Data Race 分类手册（初稿）

## 1. 分类结构

目前不存在一套能够同时覆盖 C++ 严格 data race、广义 concurrency bug、ROS callback 执行环境和具体实现根因的单一标准。本研究组合使用：

1. C++ 内存模型判定严格 data race；
2. Lu 等人的 atomicity/order violation 描述并发错误模式；
3. MITRE CWE 描述同步实现根因；
4. ROS 2 官方 executor 与 callback group 文档判断 callback 并发可行性。

每个 race 只有一个主根因 R1–R7。其他模式、位置和 callback 标签允许重叠。

## 2. 主根因分类

### R1：缺失同步（Missing Synchronization）

共享状态存在冲突访问，但实现没有建立锁、原子操作、线程封闭或其他 happens-before。

典型情况：

- callback 与工作线程无锁读写同一成员；
- vector、map、队列或缓存无保护并发访问；
- 普通布尔标志被跨线程读写。

若已有同步机制但有路径遗漏，归 R2。依据：[CWE-820](https://cwe.mitre.org/data/definitions/820.html)、[CWE-567](https://cwe.mitre.org/data/definitions/567.html)。

### R2：错误或不完整同步（Incorrect/Incomplete Synchronization）

代码存在同步意图，但同步机制没有正确覆盖全部冲突访问。

典型情况：

- 不同路径使用不同 mutex；
- 一方加锁，另一方没有加锁；
- 锁范围提前结束；
- callback group 的互斥范围被错误假设；
- 只保护指针替换，没有保护被指对象。

完全没有同步设计时归 R1。依据：[CWE-821](https://cwe.mitre.org/data/definitions/821.html)、[CWE-667](https://cwe.mitre.org/data/definitions/667.html)、[CWE-413](https://cwe.mitre.org/data/definitions/413.html)。

### R3：复合原子性或 TOCTOU 违反（Compound Atomicity/TOCTOU）

多个访问或步骤在业务语义上应整体原子执行，但另一个执行流可以在中间插入。

典型情况：

- `if (ptr) ptr->use()` 期间另一线程 reset；
- 检查容器状态后再访问元素；
- 单独线程安全的 getter/setter 无法保护跨字段 invariant；
- read-modify-write 被拆成多步。

依据：[Learning from Mistakes](https://www.microsoft.com/en-us/research/publication/learning-from-mistakes-a-comprehensive-study-on-real-world-concurrency-bug-characteristics/)、[CWE-367](https://cwe.mitre.org/data/definitions/367.html)、[CWE-362](https://cwe.mitre.org/data/definitions/362.html)。

### R4：执行顺序或状态转换违反（Ordering/State Transition）

正确性依赖一个线程事件、callback 或状态转换先于另一个发生，但实现没有建立顺序保证。

典型情况：

- 工作线程在初始化完成前访问对象；
- configure/activate 与数据 callback 顺序没有保证；
- stop 尚未完成就重新 start；
- completion callback 早于调用者设置状态。

若核心是对象已经被析构或释放，归 R5。依据：[Learning from Mistakes](https://www.microsoft.com/en-us/research/publication/learning-from-mistakes-a-comprehensive-study-on-real-world-concurrency-bug-characteristics/)。

### R5：生命周期、所有权或回收竞态（Lifetime/Ownership/Reclamation）

一个执行流正在使用对象、ROS entity、buffer、plugin 或句柄时，另一个执行流 reset、replace、destroy、unload 或 free。

典型情况：

- callback 与 node/controller 析构并发；
- timer callback 与 timer reset 并发；
- plugin unload 时工作线程尚未退出；
- callback 捕获的 `this` 已进入析构。

对象生命周期开始或结束本身可以成为 C++ 冲突动作。依据：[C++ `[intro.races]`](https://eel.is/c++draft/intro.races)、[CWE-416](https://cwe.mitre.org/data/definitions/416.html)。

### R6：原子、发布或可见性协议错误（Atomic/Publication/Visibility Protocol）

代码使用 atomic、flag、fence 或 lock-free 协议，但所选操作或 memory order 没有建立业务数据所需的可见性和顺序。

典型情况：

- relaxed flag 被错误用于发布非原子对象；
- 缺少 release/acquire 配对；
- 多个独立 atomic 无法维护跨字段 invariant；
- double-checked initialization。

共享普通变量完全没有 atomic 或锁时归 R1。依据：[C++ `[atomics.order]`](https://eel.is/c++draft/atomics.order)、[CWE-609](https://cwe.mitre.org/data/definitions/609.html)。

### R7：其他或证据不足（Other/Insufficient Evidence）

仅在 R1–R6 均无法可靠映射时使用，并强制填写：

- 缺失证据；
- 暂定类别；
- 无法完成分类的原因。

R7 不是存放未复核候选的默认类别。未获开发者确认的问题仍进入 `Unconfirmed Candidates`。

## 3. 并发错误模式

`bug_patterns` 允许多选：

- `strict_data_race`；
- `atomicity_violation`；
- `order_violation`；
- `toctou`；
- `use_after_free`；
- `lost_update`；
- `other`。

模式不代替主根因。例如，一个 R5 生命周期问题可以同时是 strict data race、order violation 和 use-after-free。

## 4. Callback Race 分类

ROS 2 executor 使用一个或多个操作系统线程调用 subscription、timer、service、action 等 callback。Multi-Threaded Executor 可以并行执行工作；Reentrant callback group 允许同组 callback 乃至同一 callback 的多个实例并行；不同 callback group 之间也可能并行。

权威依据：[ROS 2 Executors](https://docs.ros.org/en/rolling/Concepts/Intermediate/About-Executors.html)、[Using Callback Groups](https://docs.ros.org/en/jazzy/How-To-Guides/Using-callback-groups.html)。

### 4.1 Callback 参与关系

| 编码 | 名称 | 判定 |
|---|---|---|
| CB-D | callback direct | 两个或多个 callback 直接并发访问共享对象 |
| CB-T | callback-thread | callback 与后台、驱动或线程池任务并发 |
| CB-L | callback-lifecycle | callback 与 configure、activate、deactivate、cleanup、shutdown、析构或 unload 并发 |
| CB-I | callback indirect | callback 直接启动异步工作，由该窗口形成 race |
| CB-U | uncertain | 确认 callback 参与，但调度与因果证据不足 |

### 4.2 Callback 配对标签

- `CB-CB-SAME`：同一 callback 重入；
- `CB-CB-DIFF`：不同 callback 并发；
- `CB-WORKER`：callback 与后台或驱动线程；
- `CB-LIFECYCLE`：数据 callback 与 lifecycle 操作；
- `CB-SHUTDOWN`：callback 与 shutdown、析构或 unload；
- `CB-FUTURE`：callback 与 future/action completion callback；
- `CB-PARAM`：参数 callback 与其他执行流；
- `CB-UNKNOWN`：callback 已确认参与，但执行模型证据不足。

### 4.3 重要边界

- Single-Threaded Executor 只串行化其管理的 callback，不能排除 callback 与普通线程之间的 race。
- Mutually Exclusive group 只约束同组 executor callback，不能保护不同组或普通线程。
- 仅因 issue 中出现“callback”一词，不足以标记 callback race。
- callback 必须是冲突参与者，或直接产生必要的异步竞态窗口。

## 5. 位置标签

位置标签允许多选：

- `L1_MEMBER_STATE`：普通成员变量、状态、配置或标志；
- `L2_CONTAINER_BUFFER`：容器、队列、缓存、消息或 buffer；
- `L3_LIFETIME_OBJECT`：指针、所有权或 ROS entity 生命周期；
- `L4_ROS_MIDDLEWARE`：publisher、subscription、service、action、executor、callback group 或 middleware 句柄；
- `L5_PLUGIN_HARDWARE`：controller、hardware interface、plugin 或第三方库状态；
- `L6_GLOBAL_STATIC`：全局、静态或 singleton；
- `L7_MULTI_OR_UNKNOWN`：跨多个位置或证据不足。

## 6. 促成因素标签

`contributing_factors` 允许多选：

- `executor_policy_mismatch`；
- `callback_group_mismatch`；
- `unsafe_reentrancy`；
- `third_party_not_thread_safe`；
- `hidden_callback`；
- `incomplete_shutdown`；
- `unsafe_publication`；
- `multiple_variable_invariant`。

促成因素不得替代主根因。

## 7. 试标注与冻结规则

分类初稿必须先在至少两个不同规模和架构的项目上试标注。若出现以下情况，应修订定义而不是临时增加模糊类别：

- 同一证据被不同研究者稳定分到不同主类；
- R7 占比异常偏高；
- callback 因果关系无法按现有字段表达；
- 主根因无法唯一确定；
- 同一类别同时混入根因、位置和修复手段。

完成试点复核后记录分类版本号。之后发生的类别变动必须对全部既有实例重新映射。

