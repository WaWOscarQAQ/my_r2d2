# ROS Data Race 调研方法

## 1. 研究对象

本研究首先覆盖需求文件中的八个初始项目，随后综合 GitHub Star 数量、2020 年以来 race 候选数量、初步确认数量、race 报告密度、ROS 2 分支可追踪性和证据可得性筛选扩展项目。

时间边界为首次公开报告日期不早于 `2020-01-01`。仅研究 ROS 2 默认分支、发行版分支、维护分支、历史分支和 backport 分支；纯 ROS 1 问题不进入统计。

## 2. 资料范围

“Issue”按广义公开讨论理解，包括：

- GitHub Issues；
- GitHub Discussions；
- Pull Request 描述和审查讨论；
- commit message 和 commit 讨论；
- release note 和 changelog；
- ROS Discourse；
- CVE、GHSA 和项目安全通告；
- 其他具有稳定 URL 的公开开发者讨论。

关键词命中只生成候选项，不能单独证明 race 存在。搜索式、搜索日期和来源 URL 均需保存。

## 3. 统计总体

### 3.1 主统计

主统计单位是获得开发者确认的“独立 race 实例”。一个实例必须具有：

1. 可识别的并发参与者或并发执行路径；
2. 可识别的共享状态、对象、资源或顺序约束；
3. 开发者明确确认，或被维护者接受的修复能够直接证明 race；
4. 可追溯的公开证据链接。

### 3.2 未确认候选统计

普通用户报告、工具输出或研究者判断为疑似 race，但尚未得到开发者认可或缺少充分代码证据的问题，进入 `Unconfirmed Candidates`，单独计数，不进入主统计分母。

### 3.3 排除项

以下内容进入 `Excluded Items`：

- 首次报告早于时间范围；
- 只影响 ROS 1；
- 纯 deadlock、性能问题或线程饥饿；
- 经核验不是 race；
- 完全重复且已映射到主记录；
- 与目标仓库或 ROS 2 代码无关。

## 4. 开发者确认等级

| 等级 | 定义 | 是否进入主统计 |
|---|---|---:|
| A | 维护者或仓库成员明确确认 race | 是 |
| B | 修复 PR/commit 获维护者接受，且代码差异直接证明并发缺陷 | 是 |
| C | 主要贡献者或修复作者明确说明，并有较强代码证据 | 是 |
| U | 普通用户报告或工具输出，尚无开发者认可 | 否 |
| X | 证据表明不是 race | 否，进入排除项 |

主数据集中的 `confirmation_evidence` 必须说明确认发生在哪一条讨论、PR 或 commit 中，并区分开发者原话、代码证据和研究者推断。

## 5. 独立 Race 计数规则

统计按独立 race 数量，而非 issue 数量。

- 一个讨论包含不同共享对象、不同冲突代码位置或不同并发路径时，可拆分成多个 race。
- 多个讨论指向同一共享对象、冲突代码位置、并发参与者和同步缺陷时，合并为一个 race。
- backport PR 不产生新的 race，只记录修复分支。
- 同一根因在不同组件或独立代码位置形成不同缺陷时，可以分别计数。
- 边界不清时标记为“待确认”，在复核前不增加主统计数量。

实例边界依次比较：共享对象、冲突位置、并发参与者和修复机制。

## 6. Strict Data Race 判定

根据 C++ `[intro.races]`，严格 data race 需要：

1. 两个动作可能并发；
2. 两个动作访问同一内存位置或重叠对象生命周期，且至少一个为写入或生命周期变更；
3. 至少一个动作不是原子操作；
4. 两个动作之间不存在 happens-before。

严格 data race 导致未定义行为。每个实例使用：

```text
strict_data_race = yes | no | uncertain
```

判定为 `yes` 时，应填写：

- `memory_location`；
- 访问双方和读写类型；
- `strict_reason`；
- `happens_before_evidence`；
- 检测工具或代码证据。

开发者只写“race condition”而没有足够内存访问证据时，不自动判为严格 data race。

## 7. 根因、模式、位置和 Callback

研究采用多轴模型：

- `primary_root_cause`：每个主统计 race 只能有一个 R1–R7 主根因，用于可加总的项目汇总；
- `secondary_root_causes`：允许多选，用于保存次要原因；
- `bug_patterns`：strict data race、atomicity violation、order violation、TOCTOU、use-after-free、lost update 等，可重叠；
- `location_category`：成员状态、容器/缓冲区、生命周期对象、ROS middleware entity、插件/硬件接口、全局/静态等，可重叠；
- `callback_race`：独立重叠标签，不与主根因互斥。

R1–R7 的数量之和必须等于该项目的主统计 race 总数。callback、次要根因、错误模式和位置标签的比例之和允许超过 100%。

## 8. Callback Race 判定

只有在 callback 是直接冲突参与者，或 callback 的调度/重入直接产生必要竞态窗口时，才标记：

```text
callback_race = yes
```

判定步骤：

1. 至少一个参与者是否是由 ROS 2 executor 调度的 callback？
2. callback 是否访问冲突共享对象，或直接启动形成冲突的异步工作？
3. 是否存在可行并发来源，例如 Reentrant group、不同 callback group、Multi-Threaded Executor、多 executor、工作线程或 lifecycle/shutdown？
4. 是否缺少同步、顺序或生命周期保证？
5. 开发者讨论、PR 或代码差异是否支持该因果链？

如果讨论只提到 callback，但实际冲突仅发生于两个内部工作线程，不计为 callback race。

callback 关系分为：

- `direct`：callback 与 callback；
- `callback_thread`：callback 与后台或驱动线程；
- `callback_lifecycle`：callback 与生命周期、shutdown、析构或 unload；
- `callback_indirect`：callback 启动异步工作并间接形成 race；
- `uncertain`：确认 callback 参与但证据不足。

## 9. 修复状态

统一枚举：

- `unfixed`；
- `pr_pending`；
- `pr_closed_unmerged`；
- `pr_merged`；
- `commit_no_pr`；
- `unknown`。

对同一 race 的多个修复和 backport，应在关联链接与分支字段中完整记录，但不重复计数。

## 10. 比例计算

项目主统计分母：

```text
confirmed_races = COUNT(race_id WHERE project=P AND confirmed_main=yes)
```

严格 data race 比例：

```text
strict_pct = strict_races / confirmed_races
```

callback race 比例：

```text
callback_pct = callback_races / confirmed_races
```

根因比例：

```text
Ri_pct = Ri_count / confirmed_races
```

项目没有确认 race 时，比例记为 `N/A`，而不是 `0%`。未确认候选不进入主统计分母。

## 11. 质量控制

每个主统计 race 至少检查：

- 时间与 ROS 2 分支范围；
- 开发者确认等级；
- 主来源和修复链接；
- 独立实例边界；
- 冲突访问或顺序约束；
- strict data race 判定；
- 唯一主根因；
- callback 因果证据；
- 修复状态；
- CVE、GHSA 和 CWE 是否分别记录；
- 事实、代码证据和研究者推断是否分离。

所有项目汇总数必须能够从 `research/data/*.json` 的严格实例重新计算；八个初始项目还应与工作簿各项目明细表一致。

## 12. 核心权威来源

- C++ data race 与 happens-before：[C++ Working Draft — `[intro.races]`](https://eel.is/c++draft/intro.races)
- C++ atomic ordering：[C++ Working Draft — `[atomics.order]`](https://eel.is/c++draft/atomics.order)
- 真实并发缺陷模式：[Learning from Mistakes](https://www.microsoft.com/en-us/research/publication/learning-from-mistakes-a-comprehensive-study-on-real-world-concurrency-bug-characteristics/)
- ROS 2 executor：[ROS 2 Executors](https://docs.ros.org/en/rolling/Concepts/Intermediate/About-Executors.html)
- ROS 2 callback group：[Using Callback Groups](https://docs.ros.org/en/jazzy/How-To-Guides/Using-callback-groups.html)
- 缺失同步：[CWE-820](https://cwe.mitre.org/data/definitions/820.html)
- 未同步共享访问：[CWE-567](https://cwe.mitre.org/data/definitions/567.html)
- 错误同步：[CWE-821](https://cwe.mitre.org/data/definitions/821.html)
- 不当加锁：[CWE-667](https://cwe.mitre.org/data/definitions/667.html)
- TOCTOU：[CWE-367](https://cwe.mitre.org/data/definitions/367.html)
- Race Condition：[CWE-362](https://cwe.mitre.org/data/definitions/362.html)
- Use After Free：[CWE-416](https://cwe.mitre.org/data/definitions/416.html)
