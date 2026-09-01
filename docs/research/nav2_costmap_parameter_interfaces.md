# ROS 2 Parameter 接口面与 nav2_costmap_2d 落地边界

日期：2026-08-23

## 1. ROS 2 官方 Parameter 接口面

这次不是再把 parameter 伪装成 topic/service，而是按 ROS 2 自己公开的参数机制来接。

本机 Jazzy 安装树里，参数面对应的是 `rcl_interfaces` 提供的一组 service + message：

| 类别 | 文件 | 作用 |
|---|---|---|
| 参数枚举 | `/opt/ros/jazzy/share/rcl_interfaces/srv/ListParameters.srv` | 列出节点参数名 |
| 参数描述 | `/opt/ros/jazzy/share/rcl_interfaces/srv/DescribeParameters.srv` | 取 descriptor / type / read_only / range |
| 参数读取 | `/opt/ros/jazzy/share/rcl_interfaces/srv/GetParameters.srv` | 取当前值 |
| 参数写入 | `/opt/ros/jazzy/share/rcl_interfaces/srv/SetParameters.srv` | 逐项写入 |
| 原子写入 | `/opt/ros/jazzy/share/rcl_interfaces/srv/SetParametersAtomically.srv` | 整批全成或全败 |

值类型不是普通 ROS message，而是 `ParameterValue` 这套 union：

- `/opt/ros/jazzy/share/rcl_interfaces/msg/ParameterValue.msg`
- 支持 `bool / int64 / float64 / string / byte[] / bool[] / int64[] / float64[] / string[]`

约束信息在 `ParameterDescriptor`：

- `/opt/ros/jazzy/share/rcl_interfaces/msg/ParameterDescriptor.msg`
- 关键字段：`type`、`read_only`、`dynamic_typing`、`floating_point_range`、`integer_range`

ROS 2 CLI `ros2 param set` 最终也是走这条参数服务链路，不是 topic publish：

- `/opt/ros/jazzy/lib/python3.12/site-packages/ros2param/verb/set.py`
- `/opt/ros/jazzy/lib/python3.12/site-packages/ros2param/api/__init__.py`

这里的直接结论是：如果仓库要补 `EndpointBinding::Parameter`，它的 payload 语义必须是
`(node_name, parameter_name, parameter_type, parameter_value)`，不能继续套普通 message 模型。

## 2. nav2_costmap_2d 真实可吃到的动态参数点

当前 live 目标仍是单个 `/costmap` 节点，所以只调查它自己和当前启用插件的参数回调。

### 2.1 根节点 `Costmap2DROS`

注册点：

- `/home/ocsar/ROS/my_r2d2/nav2_ws/src/navigation2/nav2_costmap_2d/src/costmap_2d_ros.cpp:337`

动态回调：

- `/home/ocsar/ROS/my_r2d2/nav2_ws/src/navigation2/nav2_costmap_2d/src/costmap_2d_ros.cpp:736`

回调里实际处理的参数：

- `robot_radius`
- `footprint_padding`
- `transform_tolerance`
- `publish_frequency`
- `resolution`
- `origin_x`
- `origin_y`
- `width`
- `height`
- `footprint`
- `robot_base_frame`

这些参数的声明默认值在：

- `/home/ocsar/ROS/my_r2d2/nav2_ws/src/navigation2/nav2_costmap_2d/src/costmap_2d_ros.cpp:115`

### 2.2 当前配置启用的插件

当前 live 配置文件是：

- `/home/ocsar/ROS/my_r2d2/nav2_ws/costmap_params.yaml`

它启用了：

- `static_layer`
- `obstacle_layer`
- `inflation_layer`

#### `ObstacleLayer`

注册点：

- `/home/ocsar/ROS/my_r2d2/nav2_ws/src/navigation2/nav2_costmap_2d/plugins/obstacle_layer.cpp:108`

动态回调：

- `/home/ocsar/ROS/my_r2d2/nav2_ws/src/navigation2/nav2_costmap_2d/plugins/obstacle_layer.cpp:305`

实际处理：

- `obstacle_layer.enabled`
- `obstacle_layer.footprint_clearing_enabled`
- `obstacle_layer.min_obstacle_height`
- `obstacle_layer.max_obstacle_height`
- `obstacle_layer.combination_method`

声明默认值：

- `/home/ocsar/ROS/my_r2d2/nav2_ws/src/navigation2/nav2_costmap_2d/plugins/obstacle_layer.cpp:79`

#### `StaticLayer`

注册点：

- `/home/ocsar/ROS/my_r2d2/nav2_ws/src/navigation2/nav2_costmap_2d/plugins/static_layer.cpp:185`

动态回调：

- `/home/ocsar/ROS/my_r2d2/nav2_ws/src/navigation2/nav2_costmap_2d/plugins/static_layer.cpp:517`

真正可动态更新：

- `static_layer.enabled`
- `static_layer.footprint_clearing_enabled`
- `static_layer.transform_tolerance`

明确拒绝动态更新：

- `static_layer.map_subscribe_transient_local`
- `static_layer.map_topic`
- `static_layer.subscribe_to_updates`

声明默认值：

- `/home/ocsar/ROS/my_r2d2/nav2_ws/src/navigation2/nav2_costmap_2d/plugins/static_layer.cpp:130`

#### `InflationLayer`

注册点：

- `/home/ocsar/ROS/my_r2d2/nav2_ws/src/navigation2/nav2_costmap_2d/plugins/inflation_layer.cpp:109`

动态回调：

- `/home/ocsar/ROS/my_r2d2/nav2_ws/src/navigation2/nav2_costmap_2d/plugins/inflation_layer.cpp:436`

实际处理：

- `inflation_layer.enabled`
- `inflation_layer.inflation_radius`
- `inflation_layer.cost_scaling_factor`
- `inflation_layer.inflate_unknown`
- `inflation_layer.inflate_around_unknown`

声明默认值：

- `/home/ocsar/ROS/my_r2d2/nav2_ws/src/navigation2/nav2_costmap_2d/plugins/inflation_layer.cpp:82`

### 2.3 当前不纳入本轮 sender 的点

- `PluginContainerLayer` 虽然有动态参数回调，但当前 `costmap_params.yaml` 没启用它。
- `StaticLayer` 里被显式拒绝动态修改的参数不能当成有效 fuzz 输入面。
- 当前 live 目标没有 action server，所以这轮仍然只补 parameter，不补 action。
- 2026-09-01 full-stack run 里，`/local_costmap/local_costmap` 和
  `/global_costmap/global_costmap` 的 `footprint_padding` 参数在 benchmark 阶段
  触发 parameter set timeout / stack unhealthy；它走
  `Costmap2DROS::updateParametersCallback()` 的 `_dynamic_parameter_mutex` 路径，
  与已复现的 costmap 动态参数锁顺序问题重合。因此它现在只保留为独立
  bug reproduction 输入，不进入默认 safe parameter profile。

## 3. 这次实现采用的边界

这次落地只解决 `/costmap` 真实闭环里的 parameter 路径，不夸大成“整个 ROS graph 的通用参数发现器”。

具体边界：

1. 参数集合来自两部分交集：
   - `nav2_costmap_2d` 源码里真实存在的动态参数回调
   - live `/costmap` 当前通过 `ros2 param list` 真正声明出来的参数名
2. 恢复值优先取 live `/costmap` 的 `ros2 param dump` 当前值；拿不到时才回退到
   `nav2_ws/costmap_params.yaml` 或源码默认值。
3. 参数 payload 统一建成单字段接口：顶层只有一个 `value` 字段，字段类型对应 ROS parameter value 类型。
4. sender 通过 `ros2 param set` 走真实参数服务链路，不走 mock，不走自定义 side channel。

这意味着本轮实现是“当前 `/costmap` 目标可真实执行的参数输入面”，不是“对所有 ROS 节点自动枚举所有 parameter”的最终版。

补充口径：

- 当前合并路线中，safe parameter profile 与 topic/service/action 一起进入同一个
  live binding 集合；benchmark 与 fuzz phase 使用同一输入面建立/比较
  callback-trace reference。
- parameter 轮会在采样后恢复默认值，并丢弃恢复期 trace；如果 restore 失败，
  harness 会把本轮判为 stack unhealthy，重启真实 stack，再继续下一轮。

## 4. 为什么 sender 必须同时负责 restore

parameter 和 topic/service 最大的不同，是它会改系统全局状态。

如果一轮只写不恢复，那么下一轮 benchmark / fuzz 比较的就不是“同一个参考系统状态”，而是“被前一轮污染过的系统状态”。这会直接破坏主循环里：

1. 发送 payload
2. 真实系统执行
3. 收集当前轮 trace
4. 与 benchmark reference + mutable global state 比较

中的第 4 步语义。

所以这次 `Ros2ParameterSender` 落地时，机制上必须包含：

1. 发送变异值
2. 等待真实回调执行并收集本轮 trace
3. 把参数恢复到启动值或源码默认值
4. 清掉恢复动作本身产生的 trace，不让它串到下一轮

这里第 4 步仍是 reproduction choice，因为论文没有公开“参数输入如何做轮次清理”的事件格式。

## 5. 本轮实现目标

代码侧要补齐的不是单个类型名，而是一条完整链路：

1. `EndpointBinding::Parameter`
2. `Ros2ParameterSender`
3. `/costmap` 参数 binding 构建
4. 启动值 / 默认值恢复
5. 恢复期 trace 清理

如果这 5 个点里缺任何一个，就不能把“parameter sender 已完成”写成真实结论。
