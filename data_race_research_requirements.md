# ROS 开源项目 Data Race 调研需求

## 一、研究目标

对指定 ROS 开源项目中自 2020 年 1 月 1 日以来公开报告、并得到开发者确认的 data race 及相关并发竞态问题开展系统调研。

研究需要：

1. 找出符合条件的 race 实例及相关讨论、代码位置和修复记录。
2. 分析每个 race 的触发方式、技术根因和发生位置。
3. 建立有文献或权威标准支持的分类体系。
4. 将 callback 导致或参与触发的 race 作为独立重点类别统计。
5. 形成项目级汇总表、race 明细表和排除项审计表。
6. 确保所有统计结果均可追溯到公开讨论、PR、commit 或其他证据。

## 二、研究项目

首先对这8个项目进行研究：

1. [ros2_control](https://github.com/ros-controls/ros2_control)
2. [Autoware Universe](https://github.com/autowarefoundation/autoware_universe)
3. [MoveIt 2](https://github.com/moveit/moveit2)
4. [SLAM Toolbox](https://github.com/SteveMacenski/slam_toolbox)
5. [robot_localization](https://github.com/cra-ros-pkg/robot_localization)
6. [rmf_ros2](https://github.com/open-rmf/rmf_ros2)
7. [image_pipeline](https://github.com/ros-perception/image_pipeline)
8. [RTAB-Map](https://github.com/introlab/rtabmap)

同时也希望你综合star数量和race报告数量，可以自行探索一些新的项目。每个项目应记录调研时的 GitHub Star 数量及查询日期，作为项目影响力背景信息。

## 三、时间和分支范围

### 3.1 时间范围

纳入首次公开报告时间在 **2020-01-01 及以后**的问题。

首次公开报告时间可以是：

- GitHub Issue 创建时间；
- GitHub Discussion 创建时间；
- PR 中首次提出问题的时间；
- commit 讨论中首次确认问题的时间；
- ROS Discourse、安全通告或其他公开讨论的发布时间。

同时记录问题的确认时间、关闭时间和修复合并时间。

### 3.2 分支范围

覆盖仓库中涉及的所有 ROS 分支，包括：

- 不同 ROS 发行版对应的分支；
- 仍可追踪的历史维护分支；
- ROS 2 分支；
- 默认分支和非默认分支；
- backport 或长期维护分支。

Issue 通常属于整个仓库而不是单一分支，因此需要根据标签、讨论、PR 的目标分支、commit 和代码历史判断受影响分支。无法确定时标记为“分支未知”，不得擅自推断。

## 四、资料来源

“Issue”作广义理解，不限于 GitHub Issues。允许使用以下公开、可追溯资料：

- GitHub Issues；
- GitHub Discussions；
- Pull Request 描述及审查讨论；
- commit message 和 commit 讨论；
- ROS Discourse；
- 项目安全公告；
- CVE 或 GHSA 页面；
- release note 和 changelog；
- 开发者在其他公开渠道中的可验证讨论。

每个纳入项必须保留稳定链接和相应证据。

搜索时不能只依赖 `data race` 关键词，还应覆盖：

- `race`
- `race condition`
- `thread safety` / `thread-safe`
- `concurrent access`
- `ThreadSanitizer` / `TSan`
- `mutex`
- `atomic`
- `callback`
- `executor`
- `deadlock` 附近可能提到的 race
- `use-after-free` 或析构竞态
- `lifecycle`
- `reentrant`
- `callback group`

但关键词命中本身不代表符合纳入条件。

## 五、开发者确认标准

以下情况均可视为“得到开发者确认”：

1. 开发者在讨论中明确承认存在 data race 或相关并发竞态。
2. 开发者确认问题，但没有提交修复 PR。
3. 开发者确认存在修复 PR，但 PR 尚未合并。
4. 修复 PR 被关闭或最终没有合并，但讨论能够确认 race 存在。
5. 修复 PR 已合并。
6. commit、release note 或安全公告明确说明修复了 race。
7. 开发者认可 ThreadSanitizer 等工具报告，并将其对应到具体代码问题。

确认者可以包括：

- 项目维护者；
- 仓库成员或 collaborator；
- 主要贡献者；
- 提交相关修复并获得维护者认可的 PR 作者。

需要记录确认者身份及其与项目的关系。仅由普通用户推测、且没有开发者认可或代码证据的问题，需要统计但是不进入主统计。

修复状态统一记录为：

- 未发现修复；
- 已提出 PR、尚未合并；
- PR 已关闭或未合并；
- PR 已合并；
- 已通过 commit 修复但未找到 PR；
- 修复状态不明。

## 六、Race 的定义和边界

调研同时关注两种情况：

1. **严格 data race**：按照 C/C++ 内存模型，并发访问同一内存位置，至少一个访问为写操作，并且不存在充分同步。
2. **广义并发竞态**：开发者称为 race condition，或者表现为检查—执行竞态、回调顺序竞态、对象生命周期竞态、丢失更新等，但不一定满足严格 data race 定义。

每个 race 必须设置字段：

- `strict_data_race = yes`
- `strict_data_race = no`
- `strict_data_race = uncertain`

汇总的主统计只使用严格 data race。全部开发者确认 race 数量可作为候选池审计字段保留，但不得作为 callback、根因或并发来源占比的分母。

单纯的 deadlock、性能问题或线程饥饿不纳入 race 统计；如果同一问题同时包含 deadlock 和 data race，则只统计其中可独立确认的 race。

## 七、计数单位

统计单位为“**独立 race 实例**”，而不是 discussion 或 issue 数量。

计数规则如下：

- 一个讨论中存在两个不同共享对象、代码位置或并发访问路径的独立 race，计为两个。
- 多个 issue、PR 或讨论指向相同代码位置和相同根因时，合并为一个 race。
- 同一根因在不同文件、组件或分支形成独立缺陷时，可以分别计数。
- backport PR 不重复计算为新的 race，但应记录其修复分支。
- 重复 issue 不重复计数，应记录主 issue 和重复讨论链接。
- 无法判断是否独立时，应标记为“待确认”，不得直接增加总数。

每个独立实例分配唯一的 `race_id`。

## 八、分类体系研究

正式分类前，应首先检索和比较相关学术论文、C/C++ 内存模型、CWE、并发缺陷分类研究及权威技术资料。

分类体系应满足：

- 有明确的定义和纳入标准；
- 能映射到 ROS 项目中的实际问题；
- 分类规则可重复执行；
- 对边界案例提供处理规则；
- 明确说明哪些类别互斥、哪些类别允许重叠；
- 保存每个分类所依据的文献或标准。

分类至少包括两个分析维度。

### 8.1 技术根因和代码位置

根据文献研究结果确定最终分类，候选类别包括：

- 未受保护的共享状态或成员变量；
- 共享容器、缓存或缓冲区并发访问；
- 锁缺失或锁范围不足；
- 使用了错误的互斥量或同步对象；
- 原子性或内存可见性错误；
- 对象生命周期、析构或悬空访问竞态；
- 资源所有权或消息所有权不明确；
- 启动、停止、重配置或 lifecycle 状态竞态；
- 第三方库线程安全假设错误；
- 其他；
- 无法分类。

最终类别可以根据实际证据合并或细分。

### 8.2 Callback Data Race

callback race 必须单独分析和统计，不要求与其他根因类别互斥。

一个 race 可以同时被标记为：

- callback race；
- 未保护共享变量；
- 生命周期竞态；
- 共享容器访问。

因此，各类别的数量或比例之和允许超过 100%，必须在表格说明中明确这一点。

callback 相关字段至少包括：

- 是否属于 callback race；
- callback 类型；
- callback 所属组件；
- callback group 类型；
- executor 类型；
- 并发执行的另一方；
- 共享对象；
- 是否允许 reentrant；
- 触发条件；
- callback 与根因之间的因果关系。

callback 类型包括但不限于：

- subscription callback；
- timer callback；
- service callback；
- action callback；
- parameter callback；
- lifecycle callback；
- event callback；
- 多个 callback group 之间的并发；
- callback 与后台线程之间的并发；
- callback 与初始化、关闭或析构之间的并发。

## 九、成果表格

### 9.1 项目汇总表

每个项目一行，至少包含：

| 字段 | 内容 |
|---|---|
| 项目名称 | 仓库或项目名称 |
| 仓库链接 | GitHub 地址 |
| Star 数及查询日期 | 项目影响力快照 |
| 开发者确认 race 候选总数 | 严格、广义和证据不足条目的审计合计，不作为主统计分母 |
| 严格 data race 总数 | 满足严格定义并进入主统计的实例 |
| Callback 严格 data race 数量及占比 | callback 导致或参与的严格实例；分母为严格 data race 总数 |
| 根因类别1数量及占比 | 按最终分类统计 |
| 根因类别2数量及占比 | 按最终分类统计 |
| 其他类别数量及占比 | 依次展开 |
| 未修复数量 | 尚未发现有效修复 |
| PR 未合并数量 | 存在未合并修复 |
| PR 已合并数量 | 已完成修复 |
| CVE/GHSA 数量 | 有安全通告的实例 |
| 备注 | 统计边界和特殊情况 |

所有主表占比的分母统一为该项目的严格 data race 总数。`strict_data_race=no/uncertain` 仅用于审计和边界分析，不进入 callback、根因、并发来源或修复状态占比。项目严格 data race 总数为 0 时，占比写作 `N/A`。

### 9.2 Race 明细表

每行对应一个独立 race，至少保留：

- `race_id`
- 项目名称；
- 仓库链接；
- 主 issue 或主讨论链接；
- 其他关联讨论链接；
- 是否为重复报告；
- 首次报告日期；
- 确认日期；
- 关闭日期；
- issue 或讨论状态；
- 开发者确认依据；
- 确认者及其项目身份；
- 严格 data race 判断；
- 受影响组件；
- 文件、类、函数和变量；
- 受影响 ROS 版本及分支；
- 两个或多个并发参与者；
- 读写访问关系；
- 技术根因分类；
- 是否为 callback race；
- callback 详细类型；
- 触发条件；
- 用户可见影响；
- 检测方法；
- 修复状态；
- PR 链接；
- commit 链接；
- 修复机制；
- CVE 编号；
- GHSA 编号；
- CWE 分类；
- 证据摘录或证据摘要；
- 研究者判断；
- 判断置信度；
- 备注。

CVE、GHSA 和 CWE 应分别记录：CVE/GHSA 属于漏洞或安全通告标识，CWE 属于缺陷类型分类，不能混为同一字段。

### 9.3 排除项审计表

记录搜索到但没有纳入主统计的候选项，包括：

- 链接；
- 项目；
- 命中关键词；
- 问题摘要；
- 排除原因；
- 是否缺少开发者确认；
- 是否仅为 deadlock；
- 是否为重复问题；
- 是否早于时间范围；
- 是否无法证明存在 race；
- 后续是否需要复核。

## 十、描述和证据规范

每个 race 的详细描述应采用统一结构：

1. **位置**：涉及的组件、文件、类、函数和共享对象。
2. **并发参与者**：哪些线程、callback 或生命周期操作同时运行。
3. **冲突访问**：分别执行了什么读写操作。
4. **同步缺陷**：缺少或错误使用了什么同步机制。
5. **触发条件**：在什么执行顺序、负载或配置下发生。
6. **影响**：崩溃、数据损坏、未定义行为、错误输出或其他后果。
7. **开发者确认**：谁在何处以什么方式确认。
8. **修复方式**：增加锁、原子变量、复制数据、调整生命周期或其他方法。
9. **证据强度**：直接确认、代码修复证明或间接推断。

描述应区分：

- 开发者明确陈述的事实；
- PR 或代码差异能够证明的事实；
- 根据证据作出的研究者推断。

不得将研究者推断写成开发者已经确认的事实。

## 十一、最终交付物

最终应交付：

1. 分类标准和研究方法说明；
2. 学术及权威标准来源清单；
3. 八个项目的汇总统计表；
4. 完整 race 明细表；
5. callback race 专项分析；
6. 排除项审计表；
7. 各项目典型案例分析；
8. 数据限制、搜索盲区和置信度说明；
9. 可筛选、可复算统计结果的电子表格；
10. 一份适合进一步讨论和修改的研究报告。
