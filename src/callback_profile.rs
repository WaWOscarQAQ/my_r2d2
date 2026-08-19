//! Callback Trace Profile — 论文 §4.1.2 的实现。
//!
//! 论文 Figure 5 定义了三层数据结构，本模块把它们从 `trace_buffer` 的
//! 原始事件组装出来：
//!
//! * `CallbackInfo`   — `Callback ID = Hash(callback name, callback type)`，
//!   把 RCL 层（有 name）与 RCLCPP 层（有 type）的注册记录按 `rcl_handler`
//!   关联成一个条目。
//! * `CallbackLatency` — `Execution Latency = end - start`、
//!   `Scheduling Latency = start - invoke`。
//! * `MessageLatency`  — `Throughput = buffer size / (sub - pub)`。
//!
//! `CallbackTrace` 是两者的集合，供 Feedback Controller（论文 §4.2.1）
//! 做新状态判定。
//!
//! ## Reproduction choices（论文未披露，此处自行确定）
//! * Hash 算法：FNV-1a 64 位，先散列 name 字节，再散列 type 判别值，
//!   两者之间插入分隔字节避免 `("ab", Timer)` 与 `("a", b...)` 碰撞。
//! * 缺少 `namespace`：`RegistrationEvent` 目前未携带论文 §4.1.1 提到的
//!   namespace，故 hash 只用 name + type。补齐需同步改 C++ 布局。
//! * `throughput` 分母为 0（`sub <= pub`）时跳过该条，不产生记录。

use std::collections::HashMap;

use crate::trace_buffer::{
    CallbackType, RegistrationEvent, RegistrationSource, RuntimeEvent, RuntimeEventType,
};

/// 一个回调的静态档案（论文 Figure 5 的 `CallbackInfo`）。
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct CallbackInfo {
    /// `Hash(callback name, callback type)`。
    pub id: u64,
    /// 回调名，来自 RCL registration tracer。
    pub name: String,
    /// 回调类型，来自 RCLCPP registration tracer；仅有 RCL 记录时为 `None`。
    pub callback_type: Option<CallbackType>,
    pub rclcpp_handler: u64,
    pub rcl_handler: u64,
}

/// 一次回调执行的延迟（论文 Figure 5 的 `Callback Latency`）。
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct CallbackLatency {
    pub callback_id: u64,
    /// `end - start`。
    pub execution_latency: u64,
    /// `start - invoke`。
    pub scheduling_latency: u64,
}

/// 一条消息的吞吐（论文 Figure 5 的 `Message Latency`）。
#[derive(Debug, Clone, PartialEq)]
pub struct MessageLatency {
    pub callback_id: u64,
    /// `buffer size / (sub - pub)`。
    pub throughput: f64,
}

/// 一次测试的完整 trace profile（论文 Figure 5 的 `Callback Trace`）。
#[derive(Debug, Clone, PartialEq, Default)]
pub struct CallbackTrace {
    pub call_trace: Vec<CallbackLatency>,
    pub msg_trace: Vec<MessageLatency>,
}

/// 稳定的回调 ID：`Hash(name, type)`。
fn callback_id(name: &str, callback_type: Option<CallbackType>) -> u64 {
    let mut hasher = Fnv1a64::new();
    hasher.write(name.as_bytes());
    hasher.write(&[0x1f]); // 分隔符，避免 name/type 边界碰撞
    match callback_type {
        Some(kind) => hasher.write(&type_discriminant(kind).to_le_bytes()),
        None => hasher.write(&[0xff, 0xff, 0xff, 0xff]),
    }
    hasher.finish()
}

fn type_discriminant(kind: CallbackType) -> u32 {
    match kind {
        CallbackType::Subscription => 0,
        CallbackType::Timer => 1,
        CallbackType::Service => 2,
    }
}

/// 把注册记录按 `rcl_handler` 关联成 `CallbackInfo` 列表。
///
/// RCL 记录提供 name，RCLCPP 记录提供 type；同一回调的两条记录共享
/// `rcl_handler`。结果按 `rcl_handler` 升序，保证确定性。
pub fn build_callback_infos(events: &[RegistrationEvent]) -> Vec<CallbackInfo> {
    let mut names: HashMap<u64, String> = HashMap::new();
    let mut rclcpp: HashMap<u64, (u64, CallbackType)> = HashMap::new();

    for event in events {
        match event.source {
            RegistrationSource::Rcl => {
                names.insert(event.rcl_handler, event.callback_name.clone());
            }
            RegistrationSource::Rclcpp => {
                rclcpp.insert(event.rcl_handler, (event.rclcpp_handler, event.callback_type));
            }
        }
    }

    let mut keys: Vec<u64> = names.keys().chain(rclcpp.keys()).copied().collect();
    keys.sort_unstable();
    keys.dedup();

    keys.into_iter()
        .map(|rcl_handler| {
            let name = names.get(&rcl_handler).cloned().unwrap_or_default();
            let (rclcpp_handler, callback_type) = rclcpp
                .get(&rcl_handler)
                .copied()
                .map(|(h, t)| (h, Some(t)))
                .unwrap_or((0, None));
            let id = callback_id(&name, callback_type);
            CallbackInfo {
                id,
                name,
                callback_type,
                rclcpp_handler,
                rcl_handler,
            }
        })
        .collect()
}

/// 从运行时事件重建延迟 profile。
///
/// 每个 `rclcpp_handler` 用栈配对 `invoke -> start -> end`，因此同一回调
/// 多次执行或重入都不会串号。`RclTake` 事件按 `rcl_handler` 关联回调，
/// 计算消息吞吐。
pub fn profile_trace(infos: &[CallbackInfo], runtime: &[RuntimeEvent]) -> CallbackTrace {
    let rclcpp_to_id: HashMap<u64, u64> = infos
        .iter()
        .filter(|info| info.rclcpp_handler != 0)
        .map(|info| (info.rclcpp_handler, info.id))
        .collect();
    let rcl_to_id: HashMap<u64, u64> = infos
        .iter()
        .filter(|info| info.rcl_handler != 0)
        .map(|info| (info.rcl_handler, info.id))
        .collect();

    // handler -> 未配对的 invoke 时间戳（栈）
    let mut pending_invoke: HashMap<u64, Vec<u64>> = HashMap::new();
    // handler -> (start 时间戳, scheduling latency, callback id)（栈）
    let mut pending_start: HashMap<u64, Vec<(u64, u64, u64)>> = HashMap::new();

    let mut trace = CallbackTrace::default();

    for event in runtime {
        match event.event_type {
            RuntimeEventType::ExecutorExecute => {
                pending_invoke
                    .entry(event.rclcpp_handler)
                    .or_default()
                    .push(event.timestamp);
            }
            RuntimeEventType::CallbackStart => {
                let invoke = pending_invoke
                    .get_mut(&event.rclcpp_handler)
                    .and_then(|stack| stack.pop());
                if let (Some(invoke), Some(&id)) =
                    (invoke, rclcpp_to_id.get(&event.rclcpp_handler))
                {
                    let scheduling_latency = event.timestamp.saturating_sub(invoke);
                    pending_start
                        .entry(event.rclcpp_handler)
                        .or_default()
                        .push((event.timestamp, scheduling_latency, id));
                }
            }
            RuntimeEventType::CallbackEnd => {
                let start = pending_start
                    .get_mut(&event.rclcpp_handler)
                    .and_then(|stack| stack.pop());
                if let Some((start, scheduling_latency, id)) = start {
                    trace.call_trace.push(CallbackLatency {
                        callback_id: id,
                        execution_latency: event.timestamp.saturating_sub(start),
                        scheduling_latency,
                    });
                }
            }
            RuntimeEventType::RclTake => {
                // 分母为 0 时吞吐无定义，跳过（reproduction choice）。
                if event.sub_timestamp > event.pub_timestamp {
                    if let Some(&id) = rcl_to_id.get(&event.rcl_handler) {
                        let duration = event.sub_timestamp - event.pub_timestamp;
                        trace.msg_trace.push(MessageLatency {
                            callback_id: id,
                            throughput: event.buffer_size as f64 / duration as f64,
                        });
                    }
                }
            }
        }
    }

    trace
}

/// FNV-1a 64 位，跨平台稳定（`DefaultHasher` 的种子不保证稳定）。
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
