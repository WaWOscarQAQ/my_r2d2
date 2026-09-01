//! Runtime-ring event definitions, parsing, and reader operations.

use super::{TraceError, TraceReader, read_u32, read_u64};

pub(super) const RECORD_SIZE: u64 = 56;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum RuntimeEventType {
    ExecutorExecute,
    CallbackStart,
    CallbackEnd,
    RclTake,
    RoundBoundary,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct RuntimeEvent {
    pub event_type: RuntimeEventType,
    pub rclcpp_handler: u64,
    pub timestamp: u64,
    pub rcl_handler: u64,
    pub buffer_size: u64,
    pub pub_timestamp: u64,
    pub sub_timestamp: u64,
    pub round_id: u32,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct RuntimeDrain {
    pub events: Vec<RuntimeEvent>,
    pub missed: u64,
}

impl TraceReader {
    pub fn drain_runtime(&mut self) -> Result<RuntimeDrain, TraceError> {
        let (events, missed) = self.runtime.drain(&self.file, RECORD_SIZE, parse)?;
        Ok(RuntimeDrain { events, missed })
    }

    pub fn runtime_overflow(&self) -> Result<u64, TraceError> {
        self.runtime.overflow_count(&self.file)
    }
}

fn parse(bytes: &[u8]) -> Result<RuntimeEvent, TraceError> {
    let event_type = match read_u32(bytes, 0) {
        0 => RuntimeEventType::ExecutorExecute,
        1 => RuntimeEventType::CallbackStart,
        2 => RuntimeEventType::CallbackEnd,
        3 => RuntimeEventType::RclTake,
        4 => RuntimeEventType::RoundBoundary,
        other => {
            return Err(TraceError::Malformed(format!(
                "bad runtime event type {other}"
            )));
        }
    };

    Ok(RuntimeEvent {
        event_type,
        rclcpp_handler: read_u64(bytes, 8),
        timestamp: read_u64(bytes, 16),
        rcl_handler: read_u64(bytes, 24),
        buffer_size: read_u64(bytes, 32),
        pub_timestamp: read_u64(bytes, 40),
        sub_timestamp: read_u64(bytes, 48),
        round_id: read_u32(bytes, 4),
    })
}
