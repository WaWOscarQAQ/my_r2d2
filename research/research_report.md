# ROS 2 开源项目 Data Race 调研报告

查询截止日期：2026-08-28。

## 1. 结论摘要

本研究完整审计了 8 个指定项目，并从扩展候选池中正式纳入 Navigation2、rclcpp 和 rosbag2。按照“一个独立共享对象或冲突位置计一个 race”的口径，共识别 231 个开发者确认的 race 候选，其中 **148 个满足严格 C/C++ data race 定义并进入主统计**：

- callback 相关的严格 data race：133 个，占严格 data race 的 89.9%；
- 广义 race 但不满足严格定义：45 个，仅保留在审计候选池；
- 严格性证据不足：38 个，仅保留在审计候选池；
- 未确认候选：28 个，单独保留，不进入主统计；
- 未发现任何主统计实例与公开 CVE 或 GHSA 直接关联。

在严格 data race 中，R1 缺失同步最多，为 91 个（61.5%）；其次是 R2 不完整或错误同步，为 29 个（19.6%）；R5 生命周期/所有权/回收竞态为 25 个（16.9%）。这三类合计占 98.0%。

## 2. 项目汇总

R1–R7 是互斥主根因，项目内数量之和等于严格 data race 总数。callback 是独立重叠标签，因此不参与 R 类求和。以下数量和比例只统计 `strict_data_race=yes`，分母均为该项目的严格 data race 总数；总数为 0 时比例写作“—”。

| 项目 | 严格 Data Race 总数 | Callback 参与 | R1 | R2 | R3 | R4 | R5 | R6 | R7 |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| ros2_control | 7 | 7（100%） | 3（42.9%） | 1（14.3%） | 0 | 0 | 3（42.9%） | 0 | 0 |
| Autoware Universe | 31 | 31（100%） | 28（90.3%） | 2（6.5%） | 0 | 0 | 1（3.2%） | 0 | 0 |
| MoveIt 2 | 4 | 4（100%） | 4（100%） | 0 | 0 | 0 | 0 | 0 | 0 |
| SLAM Toolbox | 8 | 8（100%） | 1（12.5%） | 4（50.0%） | 0 | 0 | 3（37.5%） | 0 | 0 |
| robot_localization | 0 | 0（—） | 0（—） | 0（—） | 0（—） | 0（—） | 0（—） | 0（—） | 0（—） |
| rmf_ros2 | 9 | 6（66.7%） | 4（44.4%） | 4（44.4%） | 0 | 0 | 1（11.1%） | 0 | 0 |
| image_pipeline | 0 | 0（—） | 0（—） | 0（—） | 0（—） | 0（—） | 0（—） | 0（—） | 0（—） |
| RTAB-Map / rtabmap_ros | 0 | 0（—） | 0（—） | 0（—） | 0（—） | 0（—） | 0（—） | 0（—） | 0（—） |
| Navigation2（扩展） | 44 | 43（97.7%） | 21（47.7%） | 11（25.0%） | 2（4.5%） | 0 | 10（22.7%） | 0 | 0 |
| rclcpp（扩展） | 34 | 25（73.5%） | 19（55.9%） | 7（20.6%） | 0 | 1（2.9%） | 7（20.6%） | 0 | 0 |
| rosbag2（扩展） | 11 | 9（81.8%） | 11（100%） | 0 | 0 | 0 | 0 | 0 | 0 |
| **合计** | **148** | **133（89.9%）** | **91（61.5%）** | **29（19.6%）** | **2（1.4%）** | **1（0.7%）** | **25（16.9%）** | **0** | **0** |

### 2.1 十一项目的 callback / worker thread / lifecycle / others 分类

按进一步确认的研究问题，对十一个纳入项目的 148 个严格 data race 增加 `source_class`。为使主类可加总，采用 `lifecycle → 项目自建 worker_thread → callback → others` 的互斥优先级；同时保留可重叠的 `callback_race` 标签，避免 callback 与 worker/lifecycle 同时参与的信息被隐藏。

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

Worker thread 只在项目代码显式创建或拥有 OS 线程/线程型调度器时成立，例如 `std::thread`、`pthread_create`、`boost::thread`、`std::async(std::launch::async)` 或项目持有的专用 event-loop/worker。ROS Executor、DDS 内部线程、普通调用线程不自动计入；ROS timer 仍按 callback 处理。完整边界见 [source_classification.md](source_classification.md)，八个项目逐 race 表格见 [eight_project_race_tables.md](eight_project_race_tables.md)。

## 3. 分类依据

严格 data race 依据 C++ 工作草案 `[intro.races]`：两个可能并发的动作访问同一内存位置或重叠对象生命周期，至少一个是写入/生命周期动作，且不存在 happens-before。严格 data race 导致未定义行为。原子内存顺序另依据 `[atomics.order]`。

广义并发缺陷模式参考 Lu 等人的真实并发缺陷研究，将 atomicity violation 和 order violation 与严格 data race 分离。工程根因再映射到 CWE-820、CWE-567、CWE-821、CWE-667、CWE-367、CWE-362、CWE-416 和 CWE-609。

最终互斥主根因是：

- R1：完全缺失同步；
- R2：已有同步意图，但锁、callback group 或临界区覆盖不完整；
- R3：复合原子性或 TOCTOU；
- R4：执行顺序或状态转换违反；
- R5：生命周期、所有权或回收竞态；
- R6：原子发布、可见性或协议错误；
- R7：其他或证据不足。

ROS 2 callback 并发可行性依据官方 executor 和 callback-group 文档判断。callback 标签只表示 callback 是直接冲突参与者、生命周期参与者或必要异步触发者，不把“代码附近出现 callback”当作充分条件。

## 4. 证据与纳入规则

主统计要求：首次公开报告日期不早于 2020-01-01；能够映射到 ROS 2 分支或执行路径；存在具体共享对象、资源或顺序约束；并得到维护者、仓库成员、主要贡献者或获维护者接受的修复作者确认。

根据确认后的需求，主统计不仅包含已合并修复，也包含开发者已经确认但没有 PR、PR 尚未合并或 PR 关闭未合并的实例。修复状态为：

| 修复状态 | 数量 |
|---|---:|
| PR 已合并 | 133 |
| commit 修复但未找到 PR | 2 |
| PR 尚未合并 | 8 |
| PR 关闭未合并 | 1 |
| 未发现有效修复 | 3 |
| 状态不明 | 1 |

多个 issue/PR 指向相同共享对象、冲突位置、参与者和同步缺陷时合并；backport 不重复计数。一个讨论修复多个独立对象或入口时拆分。

## 5. 计数敏感性

对象级计数比 issue 数更符合本研究问题，但会受到补丁粒度影响，因此保留以下敏感性说明：

1. Navigation2 的 [issue #4496](https://github.com/ros-navigation/navigation2/issues/4496) / [PR #4521](https://github.com/ros-navigation/navigation2/pull/4521) 涉及 20 个位置，但当前只有 AMCL 条目满足严格判定，其余 19 个为 `uncertain`，不进入严格主统计。因此 Navigation2 的严格总数为 44，不使用原先的 67 个广义候选作为分母。
2. Autoware Universe 的 PR #6718 在严格主统计中按 7 个 GoalPlanner 逻辑状态对象计数；若进一步按字段级拆分，严格总数可能由 31 增至 35。
3. rmf_ros2 的 PR #69 和 #129 明确称存在 numerous/many data races，但只有 9 个对象/位置具有足够证据满足严格定义；其他条目留在审计候选池。
4. rclcpp 的严格主统计为 34 个；测试代码仍按仓库内独立严格 data race 纳入，并在明细中保留受影响组件以支持生产代码敏感性分析。
5. rosbag2 的严格主统计为 11 个；开发者确认但不满足严格定义或证据不足的条目不进入其占比分母。

## 6. 安全通告核验

每个主统计实例均分别检查 CVE、GHSA 和项目安全页，未找到公开关联。Navigation2 的 GHSA-mgj5-g2p6-gc5x / CVE-2026-26011 是 AMCL 越界写，不是并发 race，故进入安全排除说明而不与本数据集关联。这里的表述是“本次公开来源检索未找到”，不是对所有未公开漏洞的绝对不存在证明。

## 7. 主要限制

- GitHub Search 结果和 Star 数是 2026-08-28 的快照，之后可能变化。
- commit-only 修复、被删除的源分支和未索引讨论仍可能造成漏检。
- `strict=no/uncertain` 主要用于开发者已确认广义 race，但不满足严格定义，或没有公开 TSan 地址、完整读写对或足够生命周期证据的情况；这些条目不进入主统计分母。
- callback group 和 executor 类型在部分历史讨论中没有记录，不能从当前默认配置反推旧版本运行方式。
- 对测试代码的纳入遵循“仓库内开发者确认的独立 race”口径；报告同时给出 rclcpp 的生产子集，便于后续选择是否排除测试代码。

## 8. 核心来源

- [C++ Working Draft — intro.races](https://eel.is/c++draft/intro.races)
- [C++ Working Draft — atomics.order](https://eel.is/c++draft/atomics.order)
- [Learning from Mistakes: A Comprehensive Study on Real World Concurrency Bug Characteristics](https://www.microsoft.com/en-us/research/publication/learning-from-mistakes-a-comprehensive-study-on-real-world-concurrency-bug-characteristics/)
- [ROS 2 Executors](https://docs.ros.org/en/rolling/Concepts/Intermediate/About-Executors.html)
- [ROS 2 Callback Groups](https://docs.ros.org/en/jazzy/How-To-Guides/Using-callback-groups.html)
- [CWE-362 Race Condition](https://cwe.mitre.org/data/definitions/362.html)
- [CWE-416 Use After Free](https://cwe.mitre.org/data/definitions/416.html)

十一个纳入项目的工作簿明细表精简保留 ID、四类类型、issue/PR 链接、首次报告日期、标题、组件、文件路径、函数及冲突访问双方；PR 状态、CVE/GHSA/CWE、callback 关系及完整证据保存在 `research/data/*.json`。未确认候选和排除项继续保留在工作簿审计表中。
