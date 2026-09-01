//! Registration-ring event definitions, parsing, and reader operations.

use serde::Serialize;

use super::{TraceError, TraceReader, read_u32, read_u64};

pub(super) const RECORD_SIZE: u64 = 232;
const CALLBACK_NAME_CAPACITY: usize = 128;
const CALLBACK_NAMESPACE_CAPACITY: usize = 64;
const FLAG_NAME_TRUNCATED: u32 = 1;
const FLAG_NAMESPACE_TRUNCATED: u32 = 2;
const KNOWN_FLAGS: u32 = FLAG_NAME_TRUNCATED | FLAG_NAMESPACE_TRUNCATED;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum RegistrationSource {
    Rclcpp,
    Rcl,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize)]
pub enum CallbackType {
    Subscription,
    Timer,
    Service,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct RegistrationEvent {
    pub source: RegistrationSource,
    pub callback_type: CallbackType,
    pub rclcpp_handler: u64,
    pub rcl_handler: u64,
    pub callback_name: String,
    pub callback_name_truncated: bool,
    pub callback_namespace: String,
    pub callback_namespace_truncated: bool,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct RegistrationDrain {
    pub events: Vec<RegistrationEvent>,
    pub missed: u64,
}

impl TraceReader {
    pub fn drain_registration(&mut self) -> Result<RegistrationDrain, TraceError> {
        let (events, missed) = self.registration.drain(&self.file, RECORD_SIZE, parse)?;
        Ok(RegistrationDrain { events, missed })
    }

    pub fn registration_overflow(&self) -> Result<u64, TraceError> {
        self.registration.overflow_count(&self.file)
    }
}

fn parse(bytes: &[u8]) -> Result<RegistrationEvent, TraceError> {
    let source = match read_u32(bytes, 0) {
        0 => RegistrationSource::Rclcpp,
        1 => RegistrationSource::Rcl,
        other => {
            return Err(TraceError::Malformed(format!(
                "bad registration source {other}"
            )));
        }
    };
    let callback_type = match read_u32(bytes, 4) {
        0 => CallbackType::Subscription,
        1 => CallbackType::Timer,
        2 => CallbackType::Service,
        other => return Err(TraceError::Malformed(format!("bad callback type {other}"))),
    };
    let name_len = read_u32(bytes, 24) as usize;
    let flags = read_u32(bytes, 28);
    if flags & !KNOWN_FLAGS != 0 {
        return Err(TraceError::Malformed(format!(
            "unsupported registration flags 0x{flags:08x}"
        )));
    }
    if name_len > CALLBACK_NAME_CAPACITY {
        return Err(TraceError::Malformed(format!(
            "callback name length {name_len} exceeds capacity"
        )));
    }
    let namespace_len = read_u32(bytes, 160) as usize;
    if namespace_len > CALLBACK_NAMESPACE_CAPACITY {
        return Err(TraceError::Malformed(format!(
            "callback namespace length {namespace_len} exceeds capacity"
        )));
    }
    let callback_name = std::str::from_utf8(&bytes[32..32 + name_len])
        .map_err(|_| TraceError::Malformed("callback name is not valid UTF-8".to_string()))?
        .to_string();
    let callback_namespace = std::str::from_utf8(&bytes[164..164 + namespace_len])
        .map_err(|_| TraceError::Malformed("callback namespace is not valid UTF-8".to_string()))?
        .to_string();

    Ok(RegistrationEvent {
        source,
        callback_type,
        rclcpp_handler: read_u64(bytes, 8),
        rcl_handler: read_u64(bytes, 16),
        callback_name,
        callback_name_truncated: flags & FLAG_NAME_TRUNCATED != 0,
        callback_namespace,
        callback_namespace_truncated: flags & FLAG_NAMESPACE_TRUNCATED != 0,
    })
}
