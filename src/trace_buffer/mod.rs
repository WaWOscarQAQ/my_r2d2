//! Shared-memory reader for the paper-aligned C++ tracer buffers.
//!
//! R2D2 reads tracer buffers during testing, not after process exit. This
//! module is the Rust reading end over the `/dev/shm` object created by the
//! runtime tracer injected at the `rclcpp/rcl` layer.

use std::fmt;
use std::fs;
use std::fs::File;
use std::io;
use std::path::{Path, PathBuf};

mod registration;
mod runtime;

pub use registration::{CallbackType, RegistrationDrain, RegistrationEvent, RegistrationSource};
pub use runtime::{RuntimeDrain, RuntimeEvent, RuntimeEventType};

const SHARED_HEADER_SIZE: u64 = 48;
const RING_HEADER_SIZE: u64 = 64;
const RING_MUTEX_SLOT: u64 = 40;
const RING_DATA_SIZE: u64 = 24;

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

pub struct TraceSession {
    trace_path: PathBuf,
}

impl TraceSession {
    pub fn create(trace_path: impl AsRef<Path>) -> Result<Self, TraceError> {
        let trace_path = trace_path.as_ref().to_path_buf();
        if let Some(parent) = trace_path.parent() {
            fs::create_dir_all(parent)?;
        }
        cleanup_trace_artifacts(&trace_path)?;
        Ok(Self { trace_path })
    }

    pub fn trace_dir(&self) -> &Path {
        &self.trace_path
    }

    pub fn start(&mut self) -> Result<(), TraceError> {
        Ok(())
    }

    pub fn stop(&mut self) -> Result<(), TraceError> {
        Ok(())
    }

    pub fn destroy(&mut self) -> Result<(), TraceError> {
        cleanup_trace_artifacts(&self.trace_path)
    }
}

impl Drop for TraceSession {
    fn drop(&mut self) {
        let _ = self.destroy();
    }
}

struct RingData {
    write_index: u64,
    overflow_count: u64,
    capacity: u64,
}

struct RingReader {
    capacity: u64,
    records_offset: u64,
    cursor: u64,
}

impl RingReader {
    fn new(capacity: u64, records_offset: u64) -> Self {
        Self {
            capacity,
            records_offset,
            cursor: 0,
        }
    }

    fn drain<T>(
        &mut self,
        file: &File,
        record_size: u64,
        parse: fn(&[u8]) -> Result<T, TraceError>,
    ) -> Result<(Vec<T>, u64), TraceError> {
        let data = self.ring_data(file)?;
        if data.capacity != self.capacity {
            return Err(TraceError::Malformed("ring capacity mismatch".to_string()));
        }

        let start = self
            .cursor
            .max(data.write_index.saturating_sub(self.capacity));
        let missed = start - self.cursor;
        let mut events = Vec::with_capacity((data.write_index - start) as usize);
        for index in start..data.write_index {
            let slot = index % self.capacity;
            let bytes = read_at(file, self.records_offset + slot * record_size, record_size)?;
            events.push(parse(&bytes)?);
        }
        self.cursor = data.write_index;
        Ok((events, missed))
    }

    fn overflow_count(&self, file: &File) -> Result<u64, TraceError> {
        Ok(self.ring_data(file)?.overflow_count)
    }

    fn ring_data(&self, file: &File) -> Result<RingData, TraceError> {
        let ring_offset = self.records_offset - RING_HEADER_SIZE;
        let bytes = read_at(file, ring_offset + RING_MUTEX_SLOT, RING_DATA_SIZE)?;
        Ok(RingData {
            write_index: read_u64(&bytes, 0),
            overflow_count: read_u64(&bytes, 8),
            capacity: read_u64(&bytes, 16),
        })
    }
}

pub struct TraceReader {
    file: File,
    registration: RingReader,
    runtime: RingReader,
}

impl TraceReader {
    pub fn open(path: impl AsRef<Path>) -> Result<Self, TraceError> {
        let file = File::open(path)?;
        let header = read_at(&file, 0, SHARED_HEADER_SIZE)?;

        let reg_capacity = read_u64(&header, 16);
        let reg_records_offset = read_u64(&header, 24);
        let rt_capacity = read_u64(&header, 32);
        let rt_records_offset = read_u64(&header, 40);
        if reg_capacity == 0
            || rt_capacity == 0
            || reg_records_offset < SHARED_HEADER_SIZE + RING_HEADER_SIZE
            || rt_records_offset
                < reg_records_offset + reg_capacity * registration::RECORD_SIZE + RING_HEADER_SIZE
        {
            return Err(TraceError::Malformed(
                "inconsistent header offsets".to_string(),
            ));
        }

        Ok(Self {
            file,
            registration: RingReader::new(reg_capacity, reg_records_offset),
            runtime: RingReader::new(rt_capacity, rt_records_offset),
        })
    }
}

fn cleanup_trace_artifacts(trace_path: &Path) -> Result<(), TraceError> {
    cleanup_path(trace_path)?;
    cleanup_path(&sidecar_path(trace_path, ".pid"))?;
    cleanup_path(&sidecar_path(trace_path, ".profraw"))?;
    Ok(())
}

fn cleanup_path(path: &Path) -> Result<(), TraceError> {
    if !path.exists() {
        return Ok(());
    }
    if path.is_dir() {
        fs::remove_dir_all(path)?;
    } else {
        fs::remove_file(path)?;
    }
    Ok(())
}

fn sidecar_path(path: &Path, suffix: &str) -> PathBuf {
    PathBuf::from(format!("{}{}", path.display(), suffix))
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
