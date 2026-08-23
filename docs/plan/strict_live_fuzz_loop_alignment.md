# 严格 Live Fuzz Loop 对齐记录

日期：2026-08-23

目标顺序固定为：

1. 从 seed/new-state pool 生成或变异 payload
2. 通过真实 ROS topic/service/action/parameter 接口发送
3. 真实系统执行
4. 从 `rclcpp/rcl` 层连续收集 registration/runtime trace
5. 聚合当前轮重复 callback/message
6. 与 benchmark reference + mutable global state 比较
7. 判定 crash/new state
8. 保留触发 payload
9. 进入下一轮

## 1. 文档前置核查

### 1.1 论文边界

- 论文 §4.2.2 在 dry run / interface extraction 处明确写的是 `topics and services`。
- 因此，不能把 action/parameter 写成论文原设。

### 1.2 仓库扩展边界

- [docs/research/ros2_interface_matrix.md](/home/ocsar/ROS/my_r2d2/docs/research/ros2_interface_matrix.md:1) 记录了 ROS 2 官方接口矩阵：`topic / service / action`。
- [docs/research/nav2_costmap_parameter_interfaces.md](/home/ocsar/ROS/my_r2d2/docs/research/nav2_costmap_parameter_interfaces.md:1) 记录了 `/costmap` 参数面与恢复机制。
- 当前 live `nav2_costmap_2d` 真实发送面已经覆盖 `topic + service + parameter`；`action` 仍未实现。
- 所以下文第 2 步只能如实写成：`当前真实落地是 topic/service/parameter；action 仍未实现，不能宣称完成。`

### 1.3 合同文档同步

- [docs/plan/r2d2_reproduction_contract.md](/home/ocsar/ROS/my_r2d2/docs/plan/r2d2_reproduction_contract.md:14) 已同步到：
  - 输入面不再写成仅 `/scan`
  - state oracle 不再写成“冻结 benchmark 后只比较当前 trace”
  - action/parameter 不再伪装成论文原设

## 2. 按顺序落实

### 2.0 benchmark 前的真实启动屏障

代码位置：

- `nav2_ws/launch_stack.sh`
- `examples/nav2_costmap_e2e.rs`
- `src/runtime/ros2_sender.rs`

当前 live 路径在进入 fuzz loop 前，先强制执行下面的 ready barrier：

1. 拉起真实 nav2 costmap 栈，但 `launch_stack.sh` 不再固定 `sleep 3` 后盲目 `configure/activate`
2. 打开 shm reader，开始接收 `rclcpp/rcl` 层 registration/runtime trace
3. 在真实 ROS graph 里等待 `/costmap` 节点出现
4. 轮询 `/costmap` lifecycle state；若仍是 `unconfigured`，执行 `configure`
5. 等待节点进入 `inactive`；若尚未 `active`，执行 `activate`
6. 等待节点进入 `active`
7. 等待六个 costmap services、`/costmap` 参数服务和 registration trace 收敛
8. ready barrier 通过后，再执行 dry run 接口提取、`/map` bootstrap 和 benchmark

核查：

- 这一步是对步骤 2 到步骤 4 的前置保证，不属于论文公开的 fuzz 算法主体，但属于 live 真实闭环必须满足的系统前置条件。
- 这样修改后，benchmark 不会再在“节点存在但接口尚未 ready”的窗口内提前发请求。

### 2.1 生成或变异 payload

代码位置：

- [src/payload_generator.rs](/home/ocsar/ROS/my_r2d2/src/payload_generator.rs:180)
- [examples/nav2_costmap_e2e.rs](/home/ocsar/ROS/my_r2d2/examples/nav2_costmap_e2e.rs:1097)

对应代码：

```rust
pub fn retain_if_interesting(&mut self, payload: Payload, oracle: &impl StateOracle) -> bool {
    if oracle.is_new_state() || oracle.crashed() {
        self.pool.push(payload);
        true
    } else {
        false
    }
}

pub fn next_payload(&mut self) -> Result<Payload, Error> {
    let round_seed = self.base_seed.wrapping_add(self.round);
    self.rng = StdRng::seed_from_u64(round_seed);
    self.round += 1;

    let payload = if self.pool.is_empty() {
        let index = self.rng.gen_range(0..self.interfaces.len());
        let interface = &self.interfaces[index];
        let ty = top_level_type(interface);
        let value = generate_value(&ty, &mut self.rng, &self.config);
        Payload::new(interface.name.clone(), interface.kind, value, round_seed)
    } else {
        let picked = self.pool.pick_for_mutation(&mut self.rng).ok_or_else(|| {
            Error::Unsupported("payload pool became empty mid-round".to_string())
        })?;
        let interface = self.interface(&picked.interface_id).ok_or_else(|| {
            Error::Unsupported(format!(
                "pool payload references unknown interface {:?}",
                picked.interface_id
            ))
        })?;
        let ty = top_level_type(interface);
        let value = Mutator::new(self.config.clone()).mutate(&picked.value, &ty, &mut self.rng);
        Payload::new(picked.interface_id.clone(), picked.kind, value, round_seed)
    };

    let interface = self
        .interface(&payload.interface_id)
        .expect("payload interface came from the extracted list");
    let mut payload = payload;
    payload.serialized =
        SimpleSerializer.serialize(&payload.value, &top_level_type(interface))?;
    Ok(payload)
}
```

```rust
let mut generator = PayloadGenerator::new(interfaces, GeneratorConfig::default(), config.seed);
for seed in seed_preload {
    generator.pool_mut().push(seed);
}

for round in 1..=config.rounds {
    let payload = match generator.next_payload() {
        Ok(payload) => payload,
        Err(error) => {
            eprintln!("round {round}: generation failed: {error}");
            continue;
        }
    };
```

核查：

- 已落实。
- 语义符合论文：`pool empty -> random interface + spec-based generation`，`pool non-empty -> mutate interesting payload`。

### 2.1a 启动顺序实机验证

验证命令：

```bash
ROS_DOMAIN_ID=191 cargo run --example nav2_costmap_e2e -- --benchmark-seconds 5 --rounds 0
```

实测结果：

- 日志先出现 `startup: ready barrier passed`
- 随后 dry run 成功抽取 34 个真实接口
- 再之后才出现 `startup: bootstrapped /map` 与 `benchmark: sampling for 5s`
- benchmark 内已实际执行：
  - service 请求 `nav2_msgs/srv/GetCost`
  - parameter 写入 `/costmap` 动态参数
- 本轮结束摘要：`crashes=0 new_states=0 invalid_traces=0 empty_rounds=0`
- 本次 run 未再出现先前的 `Node not found`

### 2.2 通过真实 ROS 接口发送

代码位置：

- [examples/nav2_costmap_e2e.rs](/home/ocsar/ROS/my_r2d2/examples/nav2_costmap_e2e.rs:403)
- [src/runtime/ros2_sender.rs](/home/ocsar/ROS/my_r2d2/src/runtime/ros2_sender.rs:66)

对应代码：

```rust
match &binding.endpoint {
    EndpointBinding::LaserScan { .. } => {
        let sender = Ros2LaserScanSender::new(
            ros_setup,
            install_setup,
            payload_file,
            domain_id,
            LaserScanSchedule {
                rate_hz,
                duration_sec,
                burst_count: burst,
                burst_gap_ms: burst_gap,
                max_publishes,
                stamp_mode,
            },
        );
        if let Err(error) = sender.send(payload) {
            eprintln!("round {display_round}: {error}");
        }
        let drained = drain_runtime_with_live_registry(&round_label, reader, registry)?;
        let attempt_trace = profile_trace(registry, &drained);
        // ...
    }
    EndpointBinding::Topic {
        topic_name,
        message_type,
        options,
    } => {
        let sender = Ros2TopicSender::new(
            ros_setup,
            install_setup,
            domain_id,
            topic_name.clone(),
            message_type.clone(),
            binding.interface.clone(),
            options.clone(),
        );
        if let Err(error) = sender.send(payload) {
            eprintln!("round {display_round}: {error}");
        }
        let drained = drain_runtime_with_live_registry(&round_label, reader, registry)?;
        let attempt_trace = profile_trace(registry, &drained);
        // ...
    }
    EndpointBinding::Service {
        service_name,
        service_type,
    } => {
        let sender = Ros2ServiceSender::new(
            ros_setup,
            install_setup,
            domain_id,
            service_name.clone(),
            service_type.clone(),
            binding.interface.clone(),
        );
        if let Err(error) = sender.send(payload) {
            eprintln!("round {display_round}: {error}");
        }
        let drained = drain_runtime_with_live_registry(&round_label, reader, registry)?;
        let attempt_trace = profile_trace(registry, &drained);
        // ...
    }
}
```

```rust
impl Sender for Ros2TopicSender {
    fn send(&self, payload: &Payload) -> Result<(), Error> {
        let normalized = normalize_value(&self.interface.name, payload);
        let values = render_cli_payload(&normalized, &self.interface.fields)?;
        let mut args = vec![
            "ros2".to_string(),
            "topic".to_string(),
            "pub".to_string(),
            "--once".to_string(),
            "--keep-alive".to_string(),
            self.options.keep_alive_sec.to_string(),
        ];
        // ...
        run_ros2_cli(
            &self.ros_setup,
            &self.install_setup,
            &self.domain_id,
            &args,
            &format!("topic {}", self.topic_name),
        )
    }
}

impl Sender for Ros2ServiceSender {
    fn send(&self, payload: &Payload) -> Result<(), Error> {
        let normalized = normalize_value(&self.interface.name, payload);
        let values = render_cli_payload(&normalized, &self.interface.fields)?;
        let args = vec![
            "ros2".to_string(),
            "service".to_string(),
            "call".to_string(),
            self.service_name.clone(),
            self.service_type.clone(),
            values,
        ];
        run_ros2_cli(
            &self.ros_setup,
            &self.install_setup,
            &self.domain_id,
            &args,
            &format!("service {}", self.service_name),
        )
    }
}
```

核查：

- 已落实：`LaserScan`、通用 `topic`、通用 `service`、`/costmap` 参数写入与恢复。
- 未落实：`action`。
- 因此这里的真实结论只能写成：`当前 live costmap 路径严格使用真实 ROS topic/service/parameter 发送；action 仍未完成。`

### 2.3 真实系统执行

代码位置：

- [examples/nav2_costmap_e2e.rs](/home/ocsar/ROS/my_r2d2/examples/nav2_costmap_e2e.rs:922)
- [examples/nav2_costmap_e2e.rs](/home/ocsar/ROS/my_r2d2/examples/nav2_costmap_e2e.rs:553)

对应代码：

```rust
let mut stack_command = Command::new("setsid");
stack_command
    .arg("bash")
    .arg(&stack_script)
    .env("R2D2_SHM_PATH", &shm_path)
    .env_remove("LD_PRELOAD")
    .env_remove("ASAN_OPTIONS")
    .env_remove("COLCON_CURRENT_PREFIX");
let mut stack = stack_command.spawn().expect("spawn costmap stack");
let stack_pid = stack.id();
println!("stack leader pid = {stack_pid}");
```

```rust
Ok(RoundExecution {
    trace,
    crashed: !stack_alive(stack),
    sched_name,
    interface_label,
    endpoint_label,
})
```

核查：

- 已落实。
- payload 不是打到 mock，而是打到真实 `nav2_costmap_2d` 进程组。

### 2.4 连续收集 registration/runtime trace

代码位置：

- [examples/nav2_costmap_e2e.rs](/home/ocsar/ROS/my_r2d2/examples/nav2_costmap_e2e.rs:342)

对应代码：

```rust
fn ingest_registration_updates(
    round_label: &str,
    reader: &mut TraceReader,
    registry: &mut CallbackRegistry,
) -> Result<usize, String> {
    let drain = reader
        .drain_registration()
        .map_err(|error| format!("{round_label}: registration drain failed: {error}"))?;
    let count = drain.events.len();
    registry.ingest(&drain);
    Ok(count)
}

fn drain_runtime_with_live_registry(
    round_label: &str,
    reader: &mut TraceReader,
    registry: &mut CallbackRegistry,
) -> Result<RuntimeDrain, String> {
    let _ = ingest_registration_updates(round_label, reader, registry)?;
    let runtime = reader
        .drain_runtime()
        .map_err(|error| format!("{round_label}: runtime drain failed: {error}"))?;
    let _ = ingest_registration_updates(round_label, reader, registry)?;
    Ok(runtime)
}
```

```rust
let drained = drain_runtime_with_live_registry(&round_label, reader, registry)?;
let attempt_trace = profile_trace(registry, &drained);
```

```rust
let _ = ingest_registration_updates(&round_label, reader, registry)?;
let trace = profile_trace(
    registry,
    &RuntimeDrain {
        events: round_events,
        missed: round_missed,
    },
);
```

核查：

- 已落实。
- 现在 registration 不再只在启动时 ingest 一次，而是每轮、每次 runtime drain 前后都连续摄入。

### 2.5 聚合当前轮重复 callback/message

代码位置：

- [src/runtime/state_oracle.rs](/home/ocsar/ROS/my_r2d2/src/runtime/state_oracle.rs:121)

对应代码：

```rust
fn aggregate_trace(trace: &CallbackTrace) -> TraceAggregate {
    let mut aggregate = TraceAggregate::default();
    let mut previous = None;
    for latency in &trace.call_trace {
        if let Some(prev) = previous {
            aggregate.graph_edges.insert((prev, latency.callback_id));
        }
        previous = Some(latency.callback_id);

        let entry = aggregate
            .callback_latency
            .entry(latency.callback_id)
            .or_default();
        entry.total_execution_latency = entry
            .total_execution_latency
            .saturating_add(latency.execution_latency);
        entry.execution_samples = entry.execution_samples.saturating_add(1);
        if let Some(scheduling_latency) = latency.scheduling_latency {
            entry.total_scheduling_latency = entry
                .total_scheduling_latency
                .saturating_add(scheduling_latency);
            entry.scheduling_samples = entry.scheduling_samples.saturating_add(1);
        }
    }

    let mut throughput_sum: BTreeMap<u64, f64> = BTreeMap::new();
    let mut throughput_count: BTreeMap<u64, u64> = BTreeMap::new();
    for message in &trace.msg_trace {
        *throughput_sum.entry(message.callback_id).or_default() += message.throughput;
        *throughput_count.entry(message.callback_id).or_default() += 1;
    }
    for (callback_id, samples) in throughput_count {
        let sum = throughput_sum.remove(&callback_id).unwrap_or(0.0);
        aggregate.message_throughput.insert(
            callback_id,
            MessageAggregate {
                mean_throughput: sum / samples as f64,
                samples,
            },
        );
    }

    aggregate
}
```

核查：

- 已落实。
- `callback` 侧现在按当前轮重复出现做聚合。
- `message` 侧现在按当前轮重复出现求均值。
- `scheduling_latency` 已一起进入聚合，不再只看 `execution_latency`。

### 2.6 与 benchmark reference + mutable global state 比较

代码位置：

- [src/runtime/state_oracle.rs](/home/ocsar/ROS/my_r2d2/src/runtime/state_oracle.rs:166)
- [src/runtime/state_oracle.rs](/home/ocsar/ROS/my_r2d2/src/runtime/state_oracle.rs:278)
- [src/runtime/state_oracle.rs](/home/ocsar/ROS/my_r2d2/src/runtime/state_oracle.rs:362)

对应代码：

```rust
pub struct BenchmarkBuilder {
    graph_edges: BTreeSet<(u64, u64)>,
    latency_sum: BTreeMap<u64, u128>,
    latency_count: BTreeMap<u64, u64>,
    scheduling_sum: BTreeMap<u64, u128>,
    scheduling_count: BTreeMap<u64, u64>,
    throughput_sum: BTreeMap<u64, f64>,
    throughput_count: BTreeMap<u64, u64>,
    analyzed_traces: u64,
    empty_traces: u64,
    invalid_traces: u64,
}

impl BenchmarkBuilder {
    pub fn observe(&mut self, trace: &CallbackTrace) -> TraceDisposition {
        let empty = trace.call_trace.is_empty() && trace.msg_trace.is_empty();
        if empty {
            self.empty_traces += 1;
            return TraceDisposition::Empty;
        }
        if !trace.valid_for_state_analysis() {
            self.invalid_traces += 1;
            return TraceDisposition::Invalid;
        }

        let aggregate = aggregate_trace(trace);
        self.observe_aggregate(&aggregate);
        self.analyzed_traces += 1;
        TraceDisposition::Analyzed
    }
}
```

```rust
struct GlobalState {
    graph_edges: BTreeSet<(u64, u64)>,
    callback_latency: BTreeMap<u64, CallbackBenchmark>,
    message_throughput: BTreeMap<u64, MessageBenchmark>,
}

impl GlobalState {
    fn from_benchmark(model: &BenchmarkModel) -> Self {
        Self {
            graph_edges: model.graph_edges.clone(),
            callback_latency: model.callback_latency.clone(),
            message_throughput: model.message_throughput.clone(),
        }
    }

    fn update_from_aggregate(&mut self, aggregate: &TraceAggregate) {
        self.graph_edges.extend(aggregate.graph_edges.iter().copied());
        // callback / message running mean update
    }
}
```

```rust
pub struct BenchmarkStateOracle {
    benchmark: BenchmarkModel,
    global_state: GlobalState,
    thresholds: DeviationThresholds,
    last_verdict: OracleVerdict,
}

impl BenchmarkStateOracle {
    pub fn new(model: BenchmarkModel, thresholds: DeviationThresholds) -> Self {
        Self {
            global_state: GlobalState::from_benchmark(&model),
            benchmark: model,
            thresholds,
            last_verdict: OracleVerdict {
                new_state: false,
                crashed: false,
                trace: TraceDisposition::Empty,
            },
        }
    }

    fn analyze(&mut self, trace: &CallbackTrace) -> bool {
        let aggregate = aggregate_trace(trace);
        let new_state = self.detect_new_state(&aggregate);
        if new_state {
            self.global_state.update_from_aggregate(&aggregate);
        }
        new_state
    }

    fn detect_new_state(&self, aggregate: &TraceAggregate) -> bool {
        let new_edge = aggregate
            .graph_edges
            .iter()
            .any(|edge| !self.global_state.graph_edges.contains(edge));
        let new_callback = aggregate
            .callback_latency
            .keys()
            .any(|callback_id| !self.global_state.callback_latency.contains_key(callback_id));
        let new_message = aggregate.message_throughput.keys().any(|callback_id| {
            !self
                .global_state
                .message_throughput
                .contains_key(callback_id)
        });
        let latency_deviation = aggregate
            .callback_latency
            .iter()
            .any(|(&callback_id, current)| self.callback_latency_deviation(callback_id, current));
        let throughput_deviation = aggregate
            .message_throughput
            .iter()
            .any(|(&callback_id, current)| self.message_throughput_deviation(callback_id, current));

        new_edge || new_callback || new_message || latency_deviation || throughput_deviation
    }
}
```

核查：

- 已落实。
- `benchmark` 现在是 immutable reference。
- `global_state` 现在是 mutable online state。
- 当前轮先 aggregate，再比较，再按 `new_state` 更新 mutable state。

### 2.7 判定 crash/new state

代码位置：

- [examples/nav2_costmap_e2e.rs](/home/ocsar/ROS/my_r2d2/examples/nav2_costmap_e2e.rs:1854)
- [src/runtime/state_oracle.rs](/home/ocsar/ROS/my_r2d2/src/runtime/state_oracle.rs:16)

对应代码：

```rust
let trace = execution.trace;
let crashed = execution.crashed;
let verdict = oracle.evaluate(&trace, crashed);
let empty = verdict.trace == TraceDisposition::Empty;
if empty && !crashed {
    empty_rounds += 1;
} else if verdict.trace == TraceDisposition::Invalid {
    invalid += 1;
}
if verdict.paper_supported_new_state {
    paper_supported_new_states += 1;
}
if verdict.jazzy_reproduction_new_state {
    jazzy_reproduction_new_states += 1;
}
let new_state = verdict.new_state;
if crashed {
    crashes += 1;
}
if new_state {
    active_new_states += 1;
}
```

核查：

- 已落实。
- `crash` 与 `new_state` 已分离判定，之后再组合成日志决策。
- `state_oracle` 现在同时给出三层结果：
  - `paper_supported_new_state`
  - `jazzy_reproduction_new_state`
  - `new_state`（当前 `oracle_mode` 的 active verdict）
- 因此 `latency_factor` / `throughput_floor` 不会再以默认日志口径冒充 paper verdict。

### 2.8 保留触发 payload

代码位置：

- [examples/nav2_costmap_e2e.rs](/home/ocsar/ROS/my_r2d2/examples/nav2_costmap_e2e.rs:1190)
- [src/payload_generator.rs](/home/ocsar/ROS/my_r2d2/src/payload_generator.rs:180)

对应代码：

```rust
generator.retain_if_interesting(payload.clone(), &oracle);
```

```rust
pub fn retain_if_interesting(&mut self, payload: Payload, oracle: &impl StateOracle) -> bool {
    if oracle.is_new_state() || oracle.crashed() {
        self.pool.push(payload);
        true
    } else {
        false
    }
}
```

核查：

- 已落实。
- 当前逻辑与论文一致：`new state` 或 `crash` 的 payload 才入池。

### 2.9 进入下一轮

代码位置：

- [examples/nav2_costmap_e2e.rs](/home/ocsar/ROS/my_r2d2/examples/nav2_costmap_e2e.rs:1133)

对应代码：

```rust
for round in 1..=config.rounds {
    if !stack_alive(&mut stack) {
        println!("round {round:02}: costmap stack is dead, stopping");
        crashes += 1;
        break;
    }
    // generation -> send -> collect -> aggregate -> evaluate -> retain
}
```

核查：

- 已落实。
- 当前真实链路已经形成完整 live loop，不再依赖 mock 骨架来决定主流程。

## 3. 文档逻辑自检

### 3.1 现在文档内部是否自洽

- 是。
- `research` 文档负责记录 ROS 2 官方接口矩阵。
- `reproduction_contract` 负责区分 paper setting / reproduction choice / unresolved gap。
- 本文档只负责一件事：把严格 live fuzz loop 顺序映射到真实代码。

### 3.2 现在文档是否存在夸大

- 不存在以下夸大：
  - 没有把 `action` 写成已完成
  - 没有把 `RoundBoundary` 写成论文公开机制
  - 没有把阈值公式写成论文原始公式
  - 没有把 hand-maintained `costmap` binding 写成“通用 ROS graph 动态发现”

## 4. 真实性自检

### 4.1 已真实完成

- `state_oracle` 改成了 `aggregate current trace -> compare against benchmark reference + mutable global state -> update mutable state`
- `scheduling_latency` 已进入 benchmark 与 new-state 判定
- live 路径 registration/runtime 已改成持续摄入，不再只靠启动期 registry
- `cargo test --lib` 通过
- `cargo build --example nav2_costmap_e2e` 通过

### 4.2 仍未完成

- `action` sender 仍未实现
- dry run 仍是 hand-maintained `costmap` binding，不是完整 ROS graph 自动发现
- “significant deviation” 仍是 reproduction threshold，不是论文公开公式

### 4.3 因此当前最准确的真实表述

- 已经完成的是：`严格 live fuzz loop 顺序` 在当前 `nav2_costmap_2d` 的 topic/service 真实链路上落地。
- 还不能声称的是：`完整 ROS topic/service/action/parameter 全接口闭环` 已全部落地。
