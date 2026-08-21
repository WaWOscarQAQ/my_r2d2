//! Shared-memory reader for the C++ tracer buffers.
//!
//! Paper-backed part:
//! R2D2 reads the tracer buffers during testing, not after the process
//! ends; this module is the reading end that feeds the callback trace
//! profile.
//!
//! The C++ tracers publish each record before advancing `write_index`, so
//! this reader never locks: it copies the counters, then reads only the
//! records at or below the observed index. The byte layout is defined in
//! `tracer/include/tracer/trace_records.h`; the constants below mirror it,
//! including the fixed mutex slot the reader skips.

use std::fmt;
use std::fs::File;
use std::io;
use std::path::Path;

const MAGIC: u32 = 0x5252_3244;
const VERSION: u32 = 1;

const SHARED_HEADER_SIZE: u64 = 48;
const RING_HEADER_SIZE: u64 = 64;
const RING_MUTEX_SLOT: u64 = 40;
const RING_DATA_SIZE: u64 = 24;
const REGISTRATION_RECORD_SIZE: u64 = 160;
const RUNTIME_RECORD_SIZE: u64 = 56;
const CALLBACK_NAME_CAPACITY: usize = 128;
const REGISTRATION_FLAG_NAME_TRUNCATED: u32 = 1;

/// Which tracer produced a registration record.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum RegistrationSource {
    Rclcpp,
    Rcl,
}

/// The callback types recorded by the RCLCPP registration tracer.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CallbackType {
    Subscription,
    Timer,
    Service,
}

/// The runtime event kinds written by the runtime tracers.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum RuntimeEventType {
    ExecutorExecute,
    CallbackStart,
    CallbackEnd,
    RclTake,
}

/// One entry of the callback registration buffer.
///
/// `callback_type` is only meaningful when `source` is `Rclcpp`; the RCL
/// registration tracer does not record a type and leaves the field zero.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct RegistrationEvent {
    pub source: RegistrationSource,
    pub callback_type: CallbackType,
    pub rclcpp_handler: u64,
    pub rcl_handler: u64,
    pub callback_name: String,
    /// The tracer could not store the complete callback name. A truncated
    /// name must never be used to derive the paper-defined callback ID.
    pub callback_name_truncated: bool,
}

/// One entry of the runtime execution buffer.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct RuntimeEvent {
    pub event_type: RuntimeEventType,
    pub rclcpp_handler: u64,
    pub timestamp: u64,
    pub rcl_handler: u64,
    pub buffer_size: u64,
    pub pub_timestamp: u64,
    pub sub_timestamp: u64,
}

/// Events drained in one read plus the number of records that were
/// overwritten before this reader first observed them.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct RegistrationDrain {
    pub events: Vec<RegistrationEvent>,
    pub missed: u64,
}

/// Events drained in one read plus the number of records that were
/// overwritten before this reader first observed them.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct RuntimeDrain {
    pub events: Vec<RuntimeEvent>,
    pub missed: u64,
}

/// Errors produced while reading the tracer shared memory.
#[derive(Debug)]
pub enum TraceError {
    Io(io::Error),
    Malformed(String),
}

impl fmt::Display for TraceError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            TraceError::Io(error) => write!(f, "io error: {error}"),
            TraceError::Malformed(detail) => write!(f, "malformed trace buffer: {detail}"),
        }
    }
}

impl std::error::Error for TraceError {}

impl From<io::Error> for TraceError {
    fn from(error: io::Error) -> Self {
        TraceError::Io(error)
    }
}

/// The non-mutex tail of the C++ `RingHeader`, read from a fixed offset.
struct RingData {
    write_index: u64,
    overflow_count: u64,
    capacity: u64,
}

/// A reader over the tracer shared memory. Opening maps nothing; every
/// read copies the requested slice from the file backing the shared
/// memory object, which is `/dev/shm/<name>` on Linux.
pub struct TraceReader {
    file: File,
    registration_capacity: u64,
    registration_records_offset: u64,
    runtime_capacity: u64,
    runtime_records_offset: u64,
    registration_cursor: u64,
    runtime_cursor: u64,
}

impl TraceReader {
    pub fn open(path: impl AsRef<Path>) -> Result<Self, TraceError> {
        let file = File::open(path)?;
        let header = read_at(&file, 0, SHARED_HEADER_SIZE)?;

        let magic = read_u32(&header, 0);
        if magic != MAGIC {
            return Err(TraceError::Malformed(format!("bad magic 0x{magic:08x}")));
        }
        let version = read_u32(&header, 4);
        if version != VERSION {
            return Err(TraceError::Malformed(format!(
                "unsupported version {version}"
            )));
        }

        let registration_capacity = read_u64(&header, 16);
        let registration_records_offset = read_u64(&header, 24);
        let runtime_capacity = read_u64(&header, 32);
        let runtime_records_offset = read_u64(&header, 40);
        if registration_capacity == 0
            || runtime_capacity == 0
            || registration_records_offset < SHARED_HEADER_SIZE + RING_HEADER_SIZE
            || runtime_records_offset
                < registration_records_offset
                    + registration_capacity * REGISTRATION_RECORD_SIZE
                    + RING_HEADER_SIZE
        {
            return Err(TraceError::Malformed(
                "inconsistent header offsets".to_string(),
            ));
        }

        Ok(Self {
            file,
            registration_capacity,
            registration_records_offset,
            runtime_capacity,
            runtime_records_offset,
            registration_cursor: 0,
            runtime_cursor: 0,
        })
    }

    /// Reads the registration records written since the previous drain.
    pub fn drain_registration(&mut self) -> Result<RegistrationDrain, TraceError> {
        let (events, missed) = self.drain(
            self.registration_records_offset,
            self.registration_capacity,
            self.registration_cursor,
            REGISTRATION_RECORD_SIZE,
            parse_registration,
        )?;
        self.registration_cursor += events.len() as u64 + missed;
        Ok(RegistrationDrain { events, missed })
    }

    /// Reads the runtime records written since the previous drain.
    pub fn drain_runtime(&mut self) -> Result<RuntimeDrain, TraceError> {
        let (events, missed) = self.drain(
            self.runtime_records_offset,
            self.runtime_capacity,
            self.runtime_cursor,
            RUNTIME_RECORD_SIZE,
            parse_runtime,
        )?;
        self.runtime_cursor += events.len() as u64 + missed;
        Ok(RuntimeDrain { events, missed })
    }

    /// Total records overwritten in the registration ring so far.
    pub fn registration_overflow(&self) -> Result<u64, TraceError> {
        let ring = self.registration_records_offset - RING_HEADER_SIZE;
        Ok(self.ring_data(ring)?.overflow_count)
    }

    /// Total records overwritten in the runtime ring so far.
    pub fn runtime_overflow(&self) -> Result<u64, TraceError> {
        let ring = self.runtime_records_offset - RING_HEADER_SIZE;
        Ok(self.ring_data(ring)?.overflow_count)
    }

    fn drain<T>(
        &self,
        records_offset: u64,
        capacity: u64,
        cursor: u64,
        record_size: u64,
        parse: fn(&[u8]) -> Result<T, TraceError>,
    ) -> Result<(Vec<T>, u64), TraceError> {
        let data = self.ring_data(records_offset - RING_HEADER_SIZE)?;
        if data.capacity != capacity {
            return Err(TraceError::Malformed("ring capacity mismatch".to_string()));
        }
        let start = cursor.max(data.write_index.saturating_sub(capacity));
        let missed = start - cursor;
        let mut events = Vec::with_capacity((data.write_index - start) as usize);
        for index in start..data.write_index {
            let slot = index % capacity;
            let bytes = read_at(&self.file, records_offset + slot * record_size, record_size)?;
            events.push(parse(&bytes)?);
        }
        Ok((events, missed))
    }

    fn ring_data(&self, ring_offset: u64) -> Result<RingData, TraceError> {
        let bytes = read_at(&self.file, ring_offset + RING_MUTEX_SLOT, RING_DATA_SIZE)?;
        Ok(RingData {
            write_index: read_u64(&bytes, 0),
            overflow_count: read_u64(&bytes, 8),
            capacity: read_u64(&bytes, 16),
        })
    }
}

fn read_at(file: &File, offset: u64, len: u64) -> Result<Vec<u8>, TraceError> {
    use std::os::unix::fs::FileExt;
    let mut buffer = vec![0u8; len as usize];
    file.read_exact_at(&mut buffer, offset)?;
    Ok(buffer)
}

fn read_u32(bytes: &[u8], offset: usize) -> u32 {
    u32::from_le_bytes(bytes[offset..offset + 4].try_into().expect("four bytes"))
}

fn read_u64(bytes: &[u8], offset: usize) -> u64 {
    u64::from_le_bytes(bytes[offset..offset + 8].try_into().expect("eight bytes"))
}

fn parse_registration(bytes: &[u8]) -> Result<RegistrationEvent, TraceError> {
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
    if flags & !REGISTRATION_FLAG_NAME_TRUNCATED != 0 {
        return Err(TraceError::Malformed(format!(
            "unsupported registration flags 0x{flags:08x}"
        )));
    }
    if name_len > CALLBACK_NAME_CAPACITY {
        return Err(TraceError::Malformed(format!(
            "callback name length {name_len} exceeds capacity"
        )));
    }
    let callback_name = std::str::from_utf8(&bytes[32..32 + name_len])
        .map_err(|_| TraceError::Malformed("callback name is not valid UTF-8".to_string()))?
        .to_string();
    Ok(RegistrationEvent {
        source,
        callback_type,
        rclcpp_handler: read_u64(bytes, 8),
        rcl_handler: read_u64(bytes, 16),
        callback_name,
        callback_name_truncated: flags & REGISTRATION_FLAG_NAME_TRUNCATED != 0,
    })
}

fn parse_runtime(bytes: &[u8]) -> Result<RuntimeEvent, TraceError> {
    let event_type = match read_u32(bytes, 0) {
        0 => RuntimeEventType::ExecutorExecute,
        1 => RuntimeEventType::CallbackStart,
        2 => RuntimeEventType::CallbackEnd,
        3 => RuntimeEventType::RclTake,
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
    })
}
