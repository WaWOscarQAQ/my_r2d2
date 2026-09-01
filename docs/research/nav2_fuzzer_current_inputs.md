# Nav2 Callback Fuzzer 当前输入清单

记录时间：2026-08-31

来源：

- `/home/hwk/FUZZ/call_back_new/config/nav2_callback_event_relations.yaml`
- `/home/hwk/FUZZ/call_back_new/seed_pool/content/event_manifest_content.yaml`
- `/home/hwk/FUZZ/call_back_new/seed_pool_event_validation/content/event_manifest_content.yaml`

当前 `seed_pool` 和 `seed_pool_event_validation` 里的 event 输入一致：总共 41 个输入。
其中 34 个是 topic/service/action manifest 输入，原先计划的 7 个 parameter
里，当前默认只保留 5 个真正适合进入稳定 baseline 的 safe parameter profile。
`local/global_costmap.footprint_padding` 两个参数已经在 2026-09-01 的全量 run
里触发 benchmark timeout / stack unhealthy，并且与已复现的 costmap 动态参数锁路径
重合，因此降级为 bug reproduction / 风险证据，不再进入默认 coverage fuzz 调度。

当前合并路线的 full-stack dry run 实际 fuzz 输入面是 69 个 binding：

- 34 个 manifest 输入；
- 5 个 safe parameter profile 输入；
- 30 个 runtime extension 输入。

这些来源只用于报告阶段的 coverage attribution，不进入调度器；调度器只看
callback-trace oracle 的 `new_state/crash`。

2026-09-01 起，sancov 默认采用 exit-only capture：每轮仍会记录 round/trace，
但不再默认向 Nav2 进程发送 `SIGUSR2` 强制 `__sanitizer_cov_dump()`。原因是
TSan + SanitizerCoverage 下，异步 dump 会和正在执行的
`__sanitizer_cov_trace_pc_guard` 路径并发，已经观察到 planner/controller 进程
在覆盖率 flush 阶段崩溃。需要逐轮强制 dump 时可以显式设置
`R2D2_SANCOV_FLUSH=signal`，但默认 coverage fuzz 不使用这个调试模式。

## 按输入类型统计

| 类型 | 数量 | 对应 fuzzer |
|---|---:|---|
| topic/message | 27 | `topic_fuzzer` |
| service | 26 | `service_fuzzer` |
| action | 11 | `action_fuzzer` |
| parameter | 5 | `parameter_fuzzer` |

## 按输入来源统计

| 来源 | 数量 | 说明 |
|---|---:|---|
| manifest | 34 | 对齐现有 event manifest 的 topic/service/action 输入 |
| safe-parameter-profile | 5 | 已知动态且低风险参数；每轮后 restore |
| runtime-extension | 30 | full-stack ROS graph 中额外可真实发送的 topic/service |

## 按目标包统计（manifest + safe parameter profile 39 个）

下表不展开 26 个 runtime-extension binding；它们在运行日志和 summary 中按
`runtime-extension` 来源归因。

| 包 | 数量 | 当前输入 |
|---|---:|---|
| `nav2_amcl` | 7 | `/scan`, `/map`, `/initialpose`, `/reinitialize_global_localization`, `/request_nomotion_update`, `/set_initial_pose`, parameter `/amcl save_pose_rate` |
| `nav2_bt_navigator` | 3 | `/goal_pose`, `/navigate_to_pose`, `/navigate_through_poses` |
| `nav2_planner` | 4 | `/compute_path_to_pose`, `/compute_path_through_poses`, `/is_path_valid`, parameter `/planner_server expected_planner_frequency` |
| `nav2_controller` | 5 | `/follow_path`, `/speed_limit`, parameter `/controller_server controller_frequency, min_x_velocity_threshold, failure_tolerance` |
| `nav2_smoother` | 1 | `/smooth_path` |
| `nav2_behaviors` | 6 | `/spin`, `/backup`, `/drive_on_heading`, `/wait`, `/assisted_teleop`, `/preempt_teleop` |
| `nav2_costmap_2d` | 12 + 4 runtime-extension | local/global clear costmap services, local/global get cost services, local/global footprint topics；另加入 `/keepout_costmap_filter_info`, `/speed_costmap_filter_info`, `/keepout_filter_mask`, `/speed_filter_mask` 四个真实 filter topic；`footprint_padding` 参数仅用于独立 bug reproduction，不进入默认 safe profile |
| `nav2_map_server` | 1 | `/map_server/load_map` |
| `nav2_lifecycle_manager` | 0 | 当前没有主动输入，只作为 lifecycle/readiness 相关目标 |

## 重要说明

`config/fuzzer_config_event_layer_check.yaml` 里的 `targets` 不是完整输入列表，它只是 readiness 检查用的子集。

合并路线下，真正参与 dry run / benchmark / fuzz 的输入面来自当前 live binding：
manifest、safe parameter profile 和 runtime extension 会一起参与；coverage attribution
只在报告阶段按上述来源拆分。
