# ROS 2 扩展项目初筛

查询日期：2026-08-28。

## 1. 初筛方法

扩展项目初筛同时考虑：

- GitHub Star 数量；
- 2020-01-01 以来 issue 与 PR 总量；
- 同期正文中命中 `race` 的 issue/PR 数量；
- `"data race"` 和 `"race condition"` 精确短语命中数量；
- ROS 2 关联强度；
- callback/executor/后台线程使用强度；
- 公开证据是否足以进行开发者确认核验。

`race` 命中数量只是候选筛选指标，不等于已确认 race 数量。搜索结果可能包括测试 race、修复说明、讨论评论、重复报告和非缺陷语境。

初步 race 报告密度采用：

```text
race_keyword_density = race_keyword_results / issues_and_prs_since_2020
```

该指标只用于候选排序，不能与最终确认 race 比例混用。

## 2. 初始八项目 Star 快照

| 项目 | Stars | 默认分支 |
|---|---:|---|
| ros2_control | 988 | master |
| Autoware Universe | 1,742 | main |
| MoveIt 2 | 1,981 | main |
| SLAM Toolbox | 2,610 | ros2 |
| robot_localization | 1,949 | rolling-devel |
| rmf_ros2 | 116 | main |
| image_pipeline | 960 | rolling |
| RTAB-Map | 3,973 | master |

数据来源为对应仓库的 GitHub REST API 元数据，查询日期均为 2026-08-28。

## 3. 第一批扩展候选

| 项目 | Stars | 2020年以来 issue/PR | `race` 命中 | 初步密度 | `data race` | `race condition` | 初筛决定 |
|---|---:|---:|---:|---:|---:|---:|---|
| [Navigation2](https://github.com/ros-navigation/navigation2) | 4,638 | 4,935 | 79 | 1.60% | 20 | 40 | 正式纳入 |
| [rclcpp](https://github.com/ros2/rclcpp) | 796 | 2,270 | 133 | 5.86% | 31 | 72 | 正式纳入 |
| [rosbag2](https://github.com/ros2/rosbag2) | 435 | 2,231 | 98 | 4.39% | 9 | 54 | 正式纳入 |
| [rviz](https://github.com/ros2/rviz) | 480 | 未完成 | 2 | N/A | 未完成 | 未完成 | 初筛排除 |
| [BehaviorTree.CPP](https://github.com/BehaviorTree/BehaviorTree.CPP) | 4,174 | 未完成 | 未完成 | N/A | 未完成 | 未完成 | 边界候选 |

## 4. 初步判断

Navigation2、rclcpp 和 rosbag2 同时具有明确 ROS 2 关联和较多 race 相关公开讨论，已正式纳入并执行与初始项目相同的完整证据核验。

- Navigation2 的 Star 数显著高，callback、executor、behavior tree 与生命周期组件丰富。
- rclcpp 的 Star 数低于 Navigation2，但它是 ROS 2 核心 C++ client library，race 讨论密度最高。
- rosbag2 的存储、录制、回放、缓存和线程池结构使其具有较高并发缺陷研究价值。
- rviz 初筛命中较少，暂不优先投入完整研究。
- BehaviorTree.CPP Star 数高且被 ROS 2 项目广泛使用，但它不是 ROS 仓库本身；是否纳入需要明确“ROS 项目”的外延，因此保留为边界候选。

## 5. 限制

- GitHub Search 计数会随仓库更新而变化。
- `race` 会命中与 data race 无关的自然语言。
- 精确短语查询之间可能重叠，不能相加作为候选总数。
- 初筛没有完成所有分支的 commit-only 检索。
- 项目只有在逐条确认开发者证据后，才会被正式加入项目汇总表。
