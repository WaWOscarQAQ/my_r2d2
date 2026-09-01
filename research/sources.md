# 调研来源索引

## 方法与分类

| 来源 | 类型 | 用途 |
|---|---|---|
| [C++ Working Draft — Data races](https://eel.is/c++draft/intro.races) | C++ 权威条文 | strict data race、冲突动作、happens-before、生命周期冲突 |
| [C++ Working Draft — Atomic order](https://eel.is/c++draft/atomics.order) | C++ 权威条文 | 原子操作、release/acquire 和 memory order |
| [Learning from Mistakes](https://www.microsoft.com/en-us/research/publication/learning-from-mistakes-a-comprehensive-study-on-real-world-concurrency-bug-characteristics/) | 原始学术论文页面 | atomicity violation、order violation、真实并发 bug 特征 |
| [Learning from Mistakes PDF](https://people.cs.uchicago.edu/~shanlu/preprint/asplos122-lu.pdf) | 论文 PDF | 分类定义和案例 |
| [ROS 2 Executors](https://docs.ros.org/en/rolling/Concepts/Intermediate/About-Executors.html) | ROS 2 官方文档 | executor 线程与 callback 调度模型 |
| [Using Callback Groups](https://docs.ros.org/en/jazzy/How-To-Guides/Using-callback-groups.html) | ROS 2 官方文档 | Mutually Exclusive、Reentrant 和 callback 并行关系 |
| [CWE-362 Race Condition](https://cwe.mitre.org/data/definitions/362.html) | MITRE CWE | race condition 总体映射 |
| [CWE-367 TOCTOU](https://cwe.mitre.org/data/definitions/367.html) | MITRE CWE | 检查—执行竞态 |
| [CWE-413 Improper Resource Locking](https://cwe.mitre.org/data/definitions/413.html) | MITRE CWE | 资源锁错误 |
| [CWE-416 Use After Free](https://cwe.mitre.org/data/definitions/416.html) | MITRE CWE | 生命周期竞态影响映射 |
| [CWE-567 Unsynchronized Access](https://cwe.mitre.org/data/definitions/567.html) | MITRE CWE | 未同步共享数据访问 |
| [CWE-609 Double-Checked Locking](https://cwe.mitre.org/data/definitions/609.html) | MITRE CWE | 发布和原子协议错误 |
| [CWE-667 Improper Locking](https://cwe.mitre.org/data/definitions/667.html) | MITRE CWE | 锁使用错误 |
| [CWE-820 Missing Synchronization](https://cwe.mitre.org/data/definitions/820.html) | MITRE CWE | 缺失同步 |
| [CWE-821 Incorrect Synchronization](https://cwe.mitre.org/data/definitions/821.html) | MITRE CWE | 错误或不完整同步 |

## 项目证据

项目 issue、discussion、PR、commit、release note、安全通告和 Discourse 链接将在逐项目核验后追加到本节，并同步写入研究工作簿的 `Sources` 工作表。

