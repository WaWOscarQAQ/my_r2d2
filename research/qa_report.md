# 数据集与工作簿质量检查

检查日期：2026-08-28。

## 自动校验结果

- 开发者确认 race 候选池：231；
- 严格 data race 主统计：148；
- 排除在主占比之外的 `strict=no`：45，`strict=uncertain`：38；
- 未确认候选：28；
- 排除项：41；
- 来源索引：101；
- execution-source 分类审计记录：231；其中进入严格来源分类的为 148（八个初始项目 59，Navigation2 44，rclcpp 34，rosbag2 11）；
- `race_id` 重复：0；
- 早于 2020-01-01 的主统计项：0；
- 缺失必填字段：0；
- 非法枚举值：0；
- 项目内严格 R1–R7 求和不等于严格 data race 总数：0；
- callback=yes 但缺少 callback relation：0；
- 十一个纳入项目已确认 race 缺失 `source_class`：0；
- 同一 race 重复 `source_class`：0；
- 十一项目严格 data race 的 lifecycle、worker_thread、callback、others 主类求和错误：0；
- 工作簿公式错误（`#REF!`、`#DIV/0!`、`#VALUE!`、`#NAME?`、`#N/A`）：0。

## 工作簿检查

- 工作表标签已中文化：项目汇总、并发来源汇总、十一张“项目名 + 明细”工作表、回调分析、未确认候选、排除项、项目候选、分类映射、来源索引、数据字典；`Race Instances` 已按精简要求删除；
- 项目汇总仅保留项目、仓库、Star 约数、查询日期和严格 Data Race 总数；并发来源汇总仅保留基于严格 race 的各类数量；
- 汇总表、十一张项目明细表、source class 表、callback 表和各审计表均完成 PNG 渲染检查；
- 并发来源汇总已删除全部占比列；零严格 data race 项目的各类别数量正确显示为 0；
- 所有工作表均已删除深蓝色主标题下方的浅蓝色解释条；
- 十一张项目详情表已统一为 13 列：ID、类型、Issue / 讨论链接、修复 PR 链接、首次报告日期、标题、受影响组件、文件路径、函数、访问方1及其操作、访问方2及其操作；类型仅使用 `Lifecycle Race`、`Worker-Thread Race`、`Callback Race`、`Other`；
- 可见表头已统一为中文；四个并发来源和 R1–R7 在展示层使用完整英文名称，内部枚举仍用于自动校验；
- workbook SHA-256：`8e68c4bb8ae1372f2284aff4ee0f6a07a3eb13a208681564bb44f8a079f5aa95`。

## 人工边界复核

- backport、forward-port 和 duplicate discussion 不重复计数；
- 开发者确认但尚未合并的 PR 只有在满足严格 data race 定义时进入主统计；
- 未获开发者确认的 TSan/ASan 报告保留在候选表；
- pure deadlock、性能/锁争用、ABI/编译问题和假设性预防同步进入排除表；
- Navigation2 #4496/#4521、Autoware #6718、rmf_ros2 #69/#129、rclcpp test-only 和 rosbag2 open-WIP 的计数敏感性已在汇总备注和报告中显式记录。
- Worker thread 仅在项目代码显式拥有线程或线程型调度器时成立；ROS Executor、DDS 内部线程和普通外部调用线程已从该类排除。
- 互斥主类采用 lifecycle → worker_thread → callback → others；可重叠 callback 参与口径单独保留，二者的占比都以严格 data race 总数为分母。
