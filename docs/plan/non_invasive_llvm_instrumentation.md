# 非侵入式 LLVM 双层插桩

日期：2026-08-27

## 边界

当前方案不向 `rclcpp`、`rcl`、`ros2_tracing` 或九个目标 Nav2 包加入
R2D2 源码、头文件依赖或宏调用。`overlay_ws/src` 中三个 ROS 仓库保持各自
`HEAD`；实际构建输入由 `scripts/prepare_clean_ros_overlay.sh` 使用
`git archive HEAD` 生成，因此工作区里的临时修改也不会进入插桩构建。

这里的“非侵入式”指目标项目源码零改动。目标二进制仍会在编译期被 LLVM
改写，这是编译器插桩本身的定义，不等同于动态二进制探针。

## 两层数据来源

LLVM pass 以 ROS 2 已有标准 tracepoint 调用作为稳定的 IR 锚点：

1. **RCL 层**：读取 node、subscription 和 service 初始化事件，建立
   `rcl handle -> name/namespace` 映射。
2. **RCLCPP 层**：读取 subscription/service/timer callback 注册关系，建立
   `callback object -> rcl handle -> name` 映射；同时读取 executor execute、
   callback start 和 callback end 事件。

只有两层映射完整的 callback 才写入运行事件，避免把未参与本次重编译的依赖
库 callback 错配成目标 callback。独立的 `libr2d2_llvm_runtime.so` 负责映射并
复用 `tracer/src/tracers.cpp`，最终写入 registration/runtime 两个 ring buffer。

## 构建流程

```text
ROS/Navigation2 原始源码
        |
        | clang-18 -fpass-plugin=R2D2Instrumentation.so
        v
插入 __r2d2_llvm_trace 的目标文件
        |
        | 链接 libr2d2_llvm_runtime.so
        v
共享内存 registration/runtime ring buffers
```

执行：

```bash
R2D2_PROFILE=coverage scripts/build_nav2_ws.sh --coverage --clean
R2D2_PROFILE=coverage scripts/build_overlay_ws.sh --clean
```

第二条命令会重编译 RCL/RCLCPP runtime 包和这九个目标包：

- `nav2_amcl`
- `nav2_behaviors`
- `nav2_bt_navigator`
- `nav2_controller`
- `nav2_costmap_2d`
- `nav2_lifecycle_manager`
- `nav2_map_server`
- `nav2_planner`
- `nav2_smoother`

## 验证

LLVM IR 最小测试（包含普通 `call` 和异常路径上的 `invoke`）：

```bash
scripts/build_llvm_instrumentation.sh
```

确认 ROS 源码没有 R2D2 hook：

```bash
git -C overlay_ws/src/rclcpp status --short
git -C overlay_ws/src/rcl status --short
git -C overlay_ws/src/ros2_tracing status --short
rg 'tracetools_r2d2|__r2d2_llvm_trace' \
  overlay_ws/src/{rclcpp,rcl,ros2_tracing}
```

前三条应无输出；最后一条应找不到匹配。构建后的 `librcl.so` 和
`librclcpp.so` 应依赖 `libr2d2_llvm_runtime.so`，并包含对
`__r2d2_llvm_trace` 的动态引用。

2026-08-27 的全栈命令行 smoke test 通过：ready barrier 收到 5763 条注册
记录并解析出 158 个完整 callback；单轮产生 8 条 callback graph edge、4 个
callback，进程退出码为 0。

## 当前限制

Jazzy 现有 `rcl_take` 标准 tracepoint 只提供 message 指针，没有论文扩展所需的
buffer size 和 publish timestamp。因此当前非侵入式路径已经覆盖 callback
注册及调用图，但 message throughput 仍为 0。后续若补齐这一项，应增加
ABI-aware LLVM 插桩点；不能重新向 ROS 源码加入 hook。
