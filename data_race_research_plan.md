# ROS 开源项目 Data Race 调研执行计划

## 1. 计划目标

本计划用于执行《ROS 开源项目 Data Race 调研需求》中定义的研究任务。

研究首先覆盖需求中指定的八个 ROS 项目，然后结合项目 GitHub Star 数量和 2020 年以来公开报告的 race 数量，筛选并研究一批新的 ROS 2 项目。最终形成可追溯、可复核、可重新计算的 race 数据集、项目统计表和 callback data race 专项分析。

## 2. 研究范围

### 2.1 初始项目

第一阶段研究以下八个项目：

1. [ros2_control](https://github.com/ros-controls/ros2_control)
2. [Autoware Universe](https://github.com/autowarefoundation/autoware_universe)
3. [MoveIt 2](https://github.com/moveit/moveit2)
4. [SLAM Toolbox](https://github.com/SteveMacenski/slam_toolbox)
5. [robot_localization](https://github.com/cra-ros-pkg/robot_localization)
6. [rmf_ros2](https://github.com/open-rmf/rmf_ros2)
7. [image_pipeline](https://github.com/ros-perception/image_pipeline)
8. [RTAB-Map](https://github.com/introlab/rtabmap)

### 2.2 扩展项目

完成初始项目的第一轮检索后，再主动寻找其他 ROS 2 项目。扩展项目必须同时考虑：

- GitHub Star 数量；
- 2020 年以来公开出现的 race/data race/thread-safety 报告数量；
- 项目是否仍具有可追踪的 ROS 2 分支；
- issue、PR、commit 和讨论是否公开且能够形成证据链；
- 项目规模和用户影响范围；
- 与 callback、executor 或多线程组件的相关程度。

扩展项目不只按 Star 绝对值排序。还应计算或估算 race 报告密度，避免大型项目因 issue 总量较大而天然占优。

扩展项目候选表至少记录：

- 项目名称和仓库链接；
- GitHub Star 数量和查询日期；
- 2020 年以来的 issue/discussion 总量；
- race 关键词候选数量；
- 初步确认的 race 数量；
- race 报告密度；
- 是否纳入正式研究；
- 纳入或排除理由。

### 2.3 时间与分支

- 首次公开报告时间：`2020-01-01` 及以后；
- 分支范围：所有能够追踪的 ROS 2 默认分支、发行版分支、维护分支、历史分支和 backport 分支；
- 不研究纯 ROS 1 分支；
- 同一 race 的 backport 修复不得重复计数；
- 无法确认受影响分支时，记录为“分支未知”。

## 3. 研究产物与文件结构

计划生成以下主要成果：

1. `methodology.md`：研究方法、纳入标准、排除标准和证据等级；
2. `taxonomy.md`：学术与权威标准支持的 race 分类体系；
3. `project_candidates.xlsx`：初始项目和扩展项目筛选数据；
4. `race_dataset.xlsx`：race 明细、候选项、排除项和项目统计；
5. `callback_race_analysis.md`：callback data race 专项分析；
6. `case_studies.md`：各项目典型 race 案例；
7. `research_report.md`：最终研究报告；
8. `sources.md`：论文、标准、issue、PR、commit 和安全通告来源索引。

电子表格至少设置以下工作表：

- `project_summary`：项目级汇总；
- `race_instances`：独立 race 实例；
- `callback_races`：callback race 视图；
- `unconfirmed_candidates`：未获开发者确认但仍需统计的候选项；
- `excluded_items`：其他排除项；
- `project_candidates`：扩展项目筛选；
- `taxonomy_mapping`：分类定义和映射；
- `sources`：证据与来源索引。

## 4. 执行阶段

### 阶段一：冻结研究口径和数据结构

#### 工作内容

1. 将需求中的纳入、排除、确认和计数规则转化为可执行判定表。
2. 建立 `race_id` 编码规则，例如 `项目缩写-年份-序号`。
3. 定义主统计、非主统计和排除项之间的边界。
4. 建立统一的数据字段、枚举值和空值规范。
5. 明确所有比例的分母及多标签类别可能超过 100% 的说明。

#### 关键规则

- 主统计：获得开发者确认、能够定位到具体并发问题的独立 race 实例；
- 非主统计：普通用户提出或工具报告的疑似 race，但尚未获得开发者认可或缺少充分代码证据；
- 排除项：时间不符、纯 deadlock、纯性能问题、重复报告或能够证明不是 race 的问题；
- 严格 data race、广义并发竞态和不确定项分别标记；
- callback race 作为独立多标签类别，不要求与技术根因类别互斥。

#### 阶段产物

- 数据字典；
- 纳入与排除决策树；
- 证据等级规则；
- 空白研究工作簿。

### 阶段二：研究学术标准并建立分类体系

#### 工作内容

1. 检索 data race 和 concurrency bug taxonomy 相关论文。
2. 核对 C/C++ 内存模型对 data race 的定义。
3. 检索 CWE 中与并发访问、共享资源、锁和生命周期相关的分类。
4. 比较学术分类与 ROS 实际缺陷之间的映射关系。
5. 建立技术根因、代码位置、触发环境和影响类型等分类维度。
6. 单独制定 callback race 的判断规则和子类型。

#### 分类设计原则

- 技术根因分类应尽量稳定、可重复；
- callback 表示执行或触发环境，可以与技术根因重叠；
- 每个分类必须包含定义、正例、反例和边界案例；
- 每个分类必须保存文献或权威来源；
- 在完成一批真实案例试标注后，才能冻结最终类别。

#### 阶段产物

- 分类标准初稿；
- 文献与权威来源清单；
- CWE 映射表；
- callback race 判定规范。

### 阶段三：初始八项目的候选问题检索

#### 工作内容

对每个仓库分别搜索以下来源：

- GitHub Issues；
- GitHub Discussions；
- Pull Request 及审查讨论；
- commit message 和 commit 讨论；
- release note 和 changelog；
- ROS Discourse；
- CVE、GHSA 和项目安全公告；
- 其他能够稳定引用的公开开发者讨论。

为每个项目建立关键词检索矩阵，至少覆盖：

- `data race`、`race`、`race condition`；
- `thread safety`、`thread-safe`、`concurrent access`；
- `ThreadSanitizer`、`TSan`；
- `mutex`、`atomic`、`lock`；
- `callback`、`executor`、`callback group`；
- `reentrant`、`lifecycle`；
- `use-after-free`、析构和 shutdown 竞态；
- 与 race 同时出现的 `deadlock` 讨论。

每个关键词应结合日期条件进行检索，并保存查询式、查询日期和结果数量。关键词命中只作为候选，不直接纳入统计。

#### 阶段产物

- 每个项目的候选问题清单；
- 搜索日志；
- 初步来源索引；
- 漏检风险记录。

### 阶段四：扩展项目发现与筛选

#### 工作内容

1. 从 ROS 2 生态、相关组织和高 Star ROS 仓库中建立扩展候选池。
2. 对每个候选项目执行轻量关键词检索。
3. 记录 Star 数量、issue 规模和 race 候选数量。
4. 估算 race 报告密度。
5. 根据项目影响力、race 数量和证据可得性确定正式扩展项目。
6. 对纳入的扩展项目执行与初始八项目相同的完整研究流程。

#### 筛选结果要求

- 不预设固定扩展项目数量；
- 所有纳入和排除决定均保留理由；
- 不以单个模糊关键词命中作为纳入依据；
- 如果项目 Star 很高但没有足够 race 证据，应保留在候选表中但不必纳入主研究；
- 如果项目 race 报告较多但 Star 较低，应作为边界候选单独说明。

#### 阶段产物

- 扩展项目候选表；
- 正式扩展项目清单；
- 项目选择方法说明。

### 阶段五：证据核验与开发者身份确认

#### 工作内容

对每个候选问题逐项核验：

1. 判断是否存在具体共享对象或并发执行路径。
2. 判断开发者是否明确确认 race。
3. 记录确认者身份及其与项目的关系。
4. 检查关联 issue、discussion、PR、commit、release note 和安全通告。
5. 判断修复状态和合并状态。
6. 确认涉及的 ROS 2 分支和 backport。
7. 区分开发者事实陈述、代码差异证据和研究者推断。

#### 开发者确认等级

- `A`：维护者或仓库成员明确确认；
- `B`：修复 PR 或 commit 获维护者接受，代码差异直接证明 race；
- `C`：主要贡献者或修复作者明确说明，并有较强代码证据；
- `U`：普通用户报告或工具输出，尚无开发者认可；
- `X`：证据表明不属于 race。

`A`、`B`、`C` 可以进入主统计；`U` 进入未确认候选统计；`X` 进入排除项。

#### 阶段产物

- 已核验候选记录；
- 开发者确认记录；
- 修复状态记录；
- 未确认候选表；
- 排除项审计表。

### 阶段六：独立 Race 实例拆分、合并和编号

#### 工作内容

1. 将一个讨论中的多个独立 race 拆分为不同实例。
2. 将多个讨论指向的同一 race 合并。
3. 合并重复 issue 和 backport PR。
4. 根据共享对象、代码位置、并发参与者和根因判定实例边界。
5. 为每个独立实例分配唯一 `race_id`。
6. 为边界不清的问题设置“待确认”，并保存判断理由。

#### 判定依据

优先比较以下四项：

1. 共享对象是否相同；
2. 冲突代码位置是否相同；
3. 并发参与者是否相同；
4. 修复机制是否针对同一同步缺陷。

#### 阶段产物

- 去重后的 race 实例表；
- issue-to-race 映射；
- 重复项和 backport 映射；
- 实例边界争议清单。

### 阶段七：技术分类与 Callback 专项标注

#### 工作内容

为每个独立 race 完成以下标注：

- 严格 data race：`yes`、`no` 或 `uncertain`；
- 技术根因类别；
- 代码位置类别；
- 并发参与者；
- 冲突读写关系；
- 触发条件；
- 用户可见影响；
- 修复机制；
- CVE、GHSA 和 CWE；
- 置信度和研究者判断。

对 callback race 额外标注：

- callback 类型；
- callback 所属组件；
- executor 类型；
- callback group 类型；
- 是否 reentrant；
- 并发执行的另一方；
- 共享对象；
- callback 与 race 之间的因果关系。

#### Callback Race 判定层级

- `direct`：两个或多个 callback 直接并发访问共享对象；
- `callback_thread`：callback 与后台线程并发；
- `callback_lifecycle`：callback 与启动、关闭、重配置或析构并发；
- `callback_indirect`：callback 触发异步工作，间接形成 race；
- `not_callback`：没有 callback 参与；
- `uncertain`：证据不足以确定。

#### 阶段产物

- 完整分类后的 race 数据集；
- callback race 数据子集；
- 分类争议和低置信度清单。

### 阶段八：统计计算和项目比较

#### 工作内容

1. 计算每个项目的严格 data race 总数；开发者确认的广义或证据不足条目仅作审计计数。
2. 计算 callback 严格 data race 数量及占严格 data race 的比例。
3. 计算每个技术根因类别在严格 data race 中的数量及占比。
5. 计算未修复、PR 未合并、PR 已合并等修复状态数量。
6. 计算 CVE/GHSA 关联数量。
7. 单独统计未获开发者确认的 race 候选数量。
8. 比较初始项目和扩展项目的 race 数量、密度与 callback 比例。

#### 统计规则

- 主统计只纳入 `strict_data_race=yes`，分母统一为项目的严格 data race 总数；
- `strict_data_race=no/uncertain` 仅保留在审计数据中，不进入 callback、根因、并发来源或修复状态占比；
- callback 和技术根因允许重叠，因此类别比例总和可以超过 100%；
- 未确认候选不进入主统计分母；
- 项目没有严格 data race 时，占比显示为 `N/A`，不得显示为 `0%`；
- 所有项目 Star 数量必须附查询日期。

#### 阶段产物

- 项目汇总表；
- 类别分布表；
- callback 专项统计；
- 修复状态统计；
- 初始与扩展项目比较表。

### 阶段九：质量复核

#### 复核内容

1. 每个主统计实例是否具有开发者确认或等价证据。
2. 每个链接是否可访问并直接支持相应结论。
3. race 数量是否按独立实例计数，而不是按 issue 数计数。
4. 重复 issue 和 backport 是否被重复统计。
5. ROS 1 分支是否被错误纳入。
6. 时间范围是否符合要求。
7. callback 标注是否具有明确证据。
8. 严格 data race 与广义竞态是否被正确区分。
9. 统计公式、分母和空值处理是否一致。
10. 研究者推断是否与开发者陈述清楚分离。

#### 复核方式

- 对所有 callback race 进行一次专项复核；
- 对严格 data race 的判断进行一次技术复核；
- 对低置信度实例进行二次证据检索；
- 随机抽取非 callback race 检查分类一致性；
- 使用电子表格公式重新计算所有数量和比例；
- 对项目汇总数与 race 明细数进行交叉核对。

#### 阶段产物

- 质量检查清单；
- 修订记录；
- 未解决争议清单；
- 最终冻结数据集。

### 阶段十：撰写最终报告

#### 报告结构

1. 研究背景和目标；
2. 项目选择方法；
3. 数据源和搜索方法；
4. 纳入、排除和计数标准；
5. 学术分类依据；
6. 项目级统计结果；
7. 技术根因和位置分类；
8. callback data race 专项分析；
9. 修复状态和安全影响；
10. 典型案例；
11. 扩展项目比较；
12. 数据限制、搜索盲区和置信度；
13. 结论；
14. 来源和附录。

#### 阶段产物

- 最终研究报告；
- 完整电子表格；
- 来源索引；
- 可供后续修改的 Markdown 原稿。

## 5. 研究角色建议

如果采用多人或多代理并行研究，建议按职责而不是只按关键词拆分：

- **分类与文献研究者**：负责学术标准、C/C++ 内存模型和 CWE 映射；
- **项目证据研究者**：负责逐项目检索 issue、discussion、PR 和 commit；
- **Callback 专项研究者**：负责 executor、callback group 和 callback 并发路径复核；
- **代码与修复审查者**：负责核对代码位置、冲突访问和修复机制；
- **数据与质量审查者**：负责去重、统计公式、证据等级和一致性检查。

不同研究者必须使用相同的数据字典、分类手册和证据等级。最终数据应由统一复核流程合并，避免项目之间口径漂移。

## 6. 建议执行顺序

1. 冻结数据字段、判定规则和证据等级；
2. 完成学术标准初步研究；
3. 对两个初始项目进行试点检索和试标注；
4. 根据试点结果调整分类和工作簿；
5. 完成其余六个初始项目；
6. 建立并筛选扩展项目候选池；
7. 完成正式扩展项目研究；
8. 统一去重、编号和分类；
9. 完成 callback 专项复核；
10. 计算统计结果并执行质量检查；
11. 撰写最终报告并整理附件。

试点项目建议选择一个 callback/executor 使用较多的大型项目和一个规模较小的项目，以检验分类体系是否同时适用于不同规模和架构的代码库。具体试点项目在查看各仓库的初步候选数量后确定。

## 7. 完成标准

研究满足以下条件后视为完成：

- 初始八项目均完成规定关键词和资料源检索；
- 扩展项目筛选过程、纳入结果和排除理由完整可查；
- 所有主统计 race 均有公开证据和开发者确认等级；
- 未获开发者确认的候选 race 已单独统计；
- 所有 race 均按独立实例完成拆分、合并和编号；
- 所有 ROS 2 相关分支均纳入检查范围，纯 ROS 1 分支未进入统计；
- callback race 已独立标注、统计和复核；
- 严格 data race 与广义并发竞态已明确区分；
- 项目表中的数量能够从 race 明细表重新计算；
- CVE、GHSA 和 CWE 已分别记录；
- 研究者推断与开发者确认事实明确分离；
- 报告中说明了搜索盲区、证据不足和分类不确定性；
- Markdown 报告和电子表格均可继续编辑和复核。
