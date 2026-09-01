//! Callback Trace Profile — 论文 §4.1.2 的实现。
//!
//! 论文 Figure 5 规定 Callback ID 由 callback name 与 callback type
//! 哈希得到，RCLCPP/RCL handler 只负责关联。本模块因此只为两层注册信息
//! 均完整的 callback 发布 ID，并把 trace 数据质量传给后续 §4.2.1 状态判定。
//!
//! ## Reproduction choices（论文未披露，此处自行确定）
//! * Hash 算法为 FNV-1a 64 位，name 与 type 之间加入分隔字节。
//! * namespace 按论文 §4.1.1 作为注册属性采集并存入 `CallbackInfo`，但
//!   按 Figure 5 不参与 ID；同一 (name, type) 跨 namespace 出现时计为
//!   `callback_id_collisions` 并使 trace 失去状态反馈资格。
//! * ROS 2/Nav2 会在生命周期切换和动态 BT action 执行中重用 handler
//!   地址。handler 只是运行期关联，不属于论文 Figure 5 的 callback ID；
//!   因此冲突采用 latest-registration-wins，保留诊断计数但不永久作废后续
//!   trace。
//! * scheduling latency 在 invoke 缺失时为 unknown；execution latency 仍按
//!   论文的 start/end 计算。invoke 缺失单独计为 `missing_invokes`，其本身
//!   不取消 trace 的状态反馈资格；drain 边界残留的未配对 runtime 事件也只
//!   作为诊断保留。真正的记录丢失由 `lossy`（drain 的 missed 计数）拦截。
//! * throughput 单位为 bytes/ns；`sub <= pub` 时不生成度量并记录异常。
//! * `rcl_take` 可能覆盖 ROS/DDS 内部或隐藏订阅。未知的 take 只作为诊断
//!   噪声保留；已知 take 仍会生成 `msg_trace`，且不会因为未知 take 把整轮
//!   benchmark 丢弃。

use serde::Serialize;
use std::collections::{HashMap, HashSet};

use crate::trace_buffer::{
    CallbackType, RegistrationDrain, RegistrationEvent, RegistrationSource, RuntimeDrain,
    RuntimeEventType,
};

/// 一个回调的静态档案（论文 Figure 5 的 `CallbackInfo`）。
#[derive(Debug, Clone, PartialEq, Eq, Serialize)]
pub struct CallbackInfo {
    /// `Hash(callback name, callback type)`。
    pub id: u64,
    pub name: String,
    pub callback_type: CallbackType,
    /// 论文 §4.1.1 列为注册属性的 namespace；按 Figure 5 不参与 ID。
    pub namespace: String,
    pub rclcpp_handler: u64,
    pub rcl_handler: u64,
}

/// 一次回调执行的延迟（论文 Figure 5 的 `Callback Latency`）。
#[derive(Debug, Clone, PartialEq, Eq, Serialize)]
pub struct CallbackLatency {
    pub callback_id: u64,
    /// `end - start`。
    pub execution_latency: u64,
    /// `start - invoke`；invoke 记录缺失时为 unknown。
    pub scheduling_latency: Option<u64>,
}

/// 一条消息的吞吐（论文 Figure 5 的 `Message Latency`）。
#[derive(Debug, Clone, PartialEq, Serialize)]
pub struct MessageLatency {
    pub callback_id: u64,
    /// `buffer size / (sub - pub)`，当前 reproduction choice 为 bytes/ns。
    pub throughput: f64,
}

/// 不能静默混入论文 §4.2.1 状态判定的数据质量计数。
#[derive(Debug, Clone, PartialEq, Eq, Default, Serialize)]
pub struct TraceDiagnostics {
    pub registration_records_missed: u64,
    pub runtime_records_missed: u64,
    pub incomplete_registrations: u64,
    pub registration_conflicts: u64,
    pub truncated_callback_names: u64,
    pub truncated_callback_namespaces: u64,
    /// 同一 `(name, type)` 对应多个不同 namespace 的次数。Figure 5 的 ID
    /// 公式只哈希 name 与 type，此时不同回调会得到相同 ID；检测到即视为
    /// 数据歧义，trace 失去状态反馈资格（不改变论文公式本身的保守防护）。
    pub callback_id_collisions: u64,
    pub invalid_timestamp_order: u64,
    /// start 没有对应 invoke 的次数。缺失通常意味着官方 trace
    /// 相关性不完整或局部记录缺口，因此单独计数、不直接取消状态分析资格。
    pub missing_invokes: u64,
    pub unmatched_runtime_events: u64,
    /// 所有未知 handler 事件的总数，包括 callback/executor 事件和 rcl_take。
    pub unknown_handlers: u64,
    /// 未能映射到完整 CallbackInfo 的 rcl_take 数量。该类事件常来自内部
    /// ROS/DDS 订阅，不能生成 message metric，但不污染已知 callback 状态。
    pub unknown_rcl_take_handlers: u64,
    pub executor_execute_events: u64,
    pub known_executor_execute_handlers: u64,
    pub callback_start_events: u64,
    pub known_callback_start_handlers: u64,
    pub callback_end_events: u64,
    pub known_callback_end_handlers: u64,
    pub rcl_take_events: u64,
    pub known_rcl_take_handlers: u64,
    pub invalid_message_durations: u64,
}

/// 一次测试的完整 trace profile（论文 Figure 5 的 `Callback Trace`）。
#[derive(Debug, Clone, PartialEq, Default, Serialize)]
pub struct CallbackTrace {
    pub call_trace: Vec<CallbackLatency>,
    pub msg_trace: Vec<MessageLatency>,
    /// registration/runtime ring 至少有一条记录在读取前被覆盖。
    pub lossy: bool,
    pub diagnostics: TraceDiagnostics,
}

impl CallbackTrace {
    /// 只有本轮实际使用到的 handler 完整、且 runtime 无歧义的 trace 才能进入
    /// 论文 §4.2.1 的全局状态更新。未参与本轮的残缺注册仅作为诊断信息保留。
    pub fn valid_for_state_analysis(&self) -> bool {
        !self.lossy
            && self.diagnostics.truncated_callback_names == 0
            && self.diagnostics.truncated_callback_namespaces == 0
            && self.diagnostics.callback_id_collisions == 0
            && self.diagnostics.invalid_timestamp_order == 0
            && self.diagnostics.unknown_handlers == self.diagnostics.unknown_rcl_take_handlers
            && self.diagnostics.invalid_message_durations == 0
    }
}

/// 跨 shared-memory drain 保存两层注册信息。
#[derive(Debug, Default)]
pub struct CallbackRegistry {
    /// rcl_handler -> (callback name, callback namespace)，来自 RCL 层记录。
    names: HashMap<u64, (String, String)>,
    rclcpp: HashMap<u64, (u64, CallbackType)>,
    seen_handlers: HashSet<u64>,
    registration_records_missed: u64,
    registration_conflicts: u64,
    truncated_callback_names: u64,
    truncated_callback_namespaces: u64,
}

impl CallbackRegistry {
    pub fn new() -> Self {
        Self::default()
    }

    /// 累积一次实时 drain。handler 地址重用采用 latest-registration-wins；
    /// 冲突仍保留为诊断，但不永久污染后续反馈。
    pub fn ingest(&mut self, drain: &RegistrationDrain) {
        self.registration_records_missed = self
            .registration_records_missed
            .saturating_add(drain.missed);
        self.ingest_events(&drain.events);
    }

    pub fn callback_infos(&self) -> Vec<CallbackInfo> {
        let mut keys: Vec<u64> = self
            .names
            .keys()
            .filter(|handler| self.rclcpp.contains_key(handler))
            .copied()
            .collect();
        keys.sort_unstable();

        keys.into_iter()
            .map(|rcl_handler| {
                let (name, namespace) = self
                    .names
                    .get(&rcl_handler)
                    .expect("complete registry entry has a name")
                    .clone();
                let (rclcpp_handler, callback_type) = self
                    .rclcpp
                    .get(&rcl_handler)
                    .copied()
                    .expect("complete registry entry has RCLCPP data");
                CallbackInfo {
                    id: callback_id(&name, callback_type),
                    name,
                    callback_type,
                    namespace,
                    rclcpp_handler,
                    rcl_handler,
                }
            })
            .collect()
    }

    pub fn registration_records_missed(&self) -> u64 {
        self.registration_records_missed
    }

    pub fn registration_conflicts(&self) -> u64 {
        self.registration_conflicts
    }

    pub fn truncated_callback_names(&self) -> u64 {
        self.truncated_callback_names
    }

    pub fn truncated_callback_namespaces(&self) -> u64 {
        self.truncated_callback_namespaces
    }

    pub fn incomplete_registrations(&self) -> u64 {
        let complete = self
            .seen_handlers
            .iter()
            .filter(|handler| self.names.contains_key(handler) && self.rclcpp.contains_key(handler))
            .count() as u64;
        self.seen_handlers.len() as u64 - complete
    }

    fn ingest_events(&mut self, events: &[RegistrationEvent]) {
        for event in events {
            self.seen_handlers.insert(event.rcl_handler);
            match event.source {
                RegistrationSource::Rcl => {
                    if event.callback_name_truncated {
                        self.truncated_callback_names =
                            self.truncated_callback_names.saturating_add(1);
                        continue;
                    }
                    if event.callback_namespace_truncated {
                        self.truncated_callback_namespaces =
                            self.truncated_callback_namespaces.saturating_add(1);
                        continue;
                    }
                    let incoming = (
                        event.callback_name.clone(),
                        event.callback_namespace.clone(),
                    );
                    match self.names.get(&event.rcl_handler) {
                        Some(existing) if existing != &incoming => {
                            self.registration_conflicts =
                                self.registration_conflicts.saturating_add(1);
                            self.names.insert(event.rcl_handler, incoming);
                        }
                        Some(_) => {}
                        None => {
                            self.names.insert(event.rcl_handler, incoming);
                        }
                    }
                }
                RegistrationSource::Rclcpp => {
                    let incoming = (event.rclcpp_handler, event.callback_type);
                    match self.rclcpp.get(&event.rcl_handler) {
                        Some(existing) if *existing != incoming => {
                            self.registration_conflicts =
                                self.registration_conflicts.saturating_add(1);
                            self.rclcpp.insert(event.rcl_handler, incoming);
                        }
                        Some(_) => {}
                        None => {
                            self.rclcpp.insert(event.rcl_handler, incoming);
                        }
                    }
                }
            }
        }
    }
}

/// 一次性注册事件的便捷入口；实时读取应使用 `CallbackRegistry::ingest`。
pub fn build_callback_infos(events: &[RegistrationEvent]) -> Vec<CallbackInfo> {
    let mut registry = CallbackRegistry::new();
    registry.ingest_events(events);
    registry.callback_infos()
}

/// 从一次带丢失计数的 runtime drain 重建 callback trace。
pub fn profile_trace(registry: &CallbackRegistry, runtime: &RuntimeDrain) -> CallbackTrace {
    let infos = registry.callback_infos();
    let rclcpp_to_id: HashMap<u64, u64> = infos
        .iter()
        .map(|info| (info.rclcpp_handler, info.id))
        .collect();
    let rcl_to_id: HashMap<u64, u64> = infos
        .iter()
        .map(|info| (info.rcl_handler, info.id))
        .collect();

    let mut pending_invoke: HashMap<u64, Vec<u64>> = HashMap::new();
    let mut pending_start: HashMap<u64, Vec<(u64, Option<u64>, u64)>> = HashMap::new();

    // Figure 5 的 ID 只哈希 (name, type)；同一 (name, type) 出现在多个
    // namespace 下时不同回调会得到相同 ID，属于数据歧义，显式计数并拦截。
    let mut id_namespaces: HashMap<(&str, CallbackType), &str> = HashMap::new();
    let mut callback_id_collisions: u64 = 0;
    for info in &infos {
        let key = (info.name.as_str(), info.callback_type);
        match id_namespaces.get(&key) {
            Some(existing) if *existing != info.namespace => {
                callback_id_collisions += 1;
            }
            Some(_) => {}
            None => {
                id_namespaces.insert(key, info.namespace.as_str());
            }
        }
    }

    let mut trace = CallbackTrace {
        lossy: registry.registration_records_missed() > 0 || runtime.missed > 0,
        diagnostics: TraceDiagnostics {
            registration_records_missed: registry.registration_records_missed(),
            runtime_records_missed: runtime.missed,
            incomplete_registrations: registry.incomplete_registrations(),
            registration_conflicts: registry.registration_conflicts(),
            truncated_callback_names: registry.truncated_callback_names(),
            truncated_callback_namespaces: registry.truncated_callback_namespaces(),
            callback_id_collisions,
            ..TraceDiagnostics::default()
        },
        ..CallbackTrace::default()
    };

    for event in &runtime.events {
        match event.event_type {
            RuntimeEventType::ExecutorExecute => {
                trace.diagnostics.executor_execute_events += 1;
                if rclcpp_to_id.contains_key(&event.rclcpp_handler) {
                    trace.diagnostics.known_executor_execute_handlers += 1;
                    pending_invoke
                        .entry(event.rclcpp_handler)
                        .or_default()
                        .push(event.timestamp);
                } else {
                    trace.diagnostics.unknown_handlers += 1;
                }
            }
            RuntimeEventType::CallbackStart => {
                trace.diagnostics.callback_start_events += 1;
                let Some(&id) = rclcpp_to_id.get(&event.rclcpp_handler) else {
                    trace.diagnostics.unknown_handlers += 1;
                    continue;
                };
                trace.diagnostics.known_callback_start_handlers += 1;
                let invoke = pending_invoke
                    .get_mut(&event.rclcpp_handler)
                    .and_then(|stack| stack.pop());
                let scheduling_latency = match invoke {
                    Some(invoke) => match event.timestamp.checked_sub(invoke) {
                        Some(duration) => Some(duration),
                        None => {
                            trace.diagnostics.invalid_timestamp_order += 1;
                            None
                        }
                    },
                    None => {
                        trace.diagnostics.missing_invokes += 1;
                        None
                    }
                };
                pending_start
                    .entry(event.rclcpp_handler)
                    .or_default()
                    .push((event.timestamp, scheduling_latency, id));
            }
            RuntimeEventType::CallbackEnd => {
                trace.diagnostics.callback_end_events += 1;
                if rclcpp_to_id.contains_key(&event.rclcpp_handler) {
                    trace.diagnostics.known_callback_end_handlers += 1;
                }
                let start = pending_start
                    .get_mut(&event.rclcpp_handler)
                    .and_then(|stack| stack.pop());
                match start {
                    Some((start, scheduling_latency, id)) => {
                        if let Some(execution_latency) = event.timestamp.checked_sub(start) {
                            trace.call_trace.push(CallbackLatency {
                                callback_id: id,
                                execution_latency,
                                scheduling_latency,
                            });
                        } else {
                            trace.diagnostics.invalid_timestamp_order += 1;
                        }
                    }
                    None => trace.diagnostics.unmatched_runtime_events += 1,
                }
            }
            RuntimeEventType::RclTake => {
                trace.diagnostics.rcl_take_events += 1;
                let Some(&id) = rcl_to_id.get(&event.rcl_handler) else {
                    trace.diagnostics.unknown_handlers += 1;
                    trace.diagnostics.unknown_rcl_take_handlers += 1;
                    continue;
                };
                trace.diagnostics.known_rcl_take_handlers += 1;
                if let Some(duration) = event.sub_timestamp.checked_sub(event.pub_timestamp) {
                    if duration > 0 {
                        trace.msg_trace.push(MessageLatency {
                            callback_id: id,
                            throughput: event.buffer_size as f64 / duration as f64,
                        });
                    } else {
                        trace.diagnostics.invalid_message_durations += 1;
                    }
                } else {
                    trace.diagnostics.invalid_message_durations += 1;
                }
            }
            RuntimeEventType::RoundBoundary => {}
        }
    }

    trace.diagnostics.unmatched_runtime_events += pending_invoke
        .values()
        .map(|stack| stack.len() as u64)
        .sum::<u64>();
    trace.diagnostics.unmatched_runtime_events += pending_start
        .values()
        .map(|stack| stack.len() as u64)
        .sum::<u64>();

    trace
}

fn callback_id(name: &str, callback_type: CallbackType) -> u64 {
    let mut hasher = Fnv1a64::new();
    hasher.write(name.as_bytes());
    hasher.write(&[0x1f]);
    hasher.write(&type_discriminant(callback_type).to_le_bytes());
    hasher.finish()
}

fn type_discriminant(kind: CallbackType) -> u32 {
    match kind {
        CallbackType::Subscription => 0,
        CallbackType::Timer => 1,
        CallbackType::Service => 2,
    }
}

struct Fnv1a64(u64);

impl Fnv1a64 {
    const OFFSET_BASIS: u64 = 0xcbf2_9ce4_8422_2325;
    const PRIME: u64 = 0x0000_0100_0000_01b3;

    fn new() -> Self {
        Self(Self::OFFSET_BASIS)
    }

    fn write(&mut self, bytes: &[u8]) {
        for &byte in bytes {
            self.0 ^= byte as u64;
            self.0 = self.0.wrapping_mul(Self::PRIME);
        }
    }

    fn finish(self) -> u64 {
        self.0
    }
}
