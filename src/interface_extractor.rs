//! ROS interface discovery boundary.
//!
//! Paper-backed part:
//! R2D2 extracts ROS interfaces and uses their data structure to build inputs.
//!
//! Placeholder part:
//! This first version defines the extraction boundary but does not inspect a
//! ROS 2 graph yet.

use std::fmt;

/// The kind of ROS interface that can carry a fuzzing payload.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Kind {
    Topic,
    Service,
    Action,
}

/// One field in a message or service shape.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Field {
    pub name: String,
    pub ty: String,
}

impl Field {
    pub fn new(name: impl Into<String>, ty: impl Into<String>) -> Self {
        Self {
            name: name.into(),
            ty: ty.into(),
        }
    }
}

/// A discovered ROS interface and its top-level fields.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Interface {
    pub name: String,
    pub kind: Kind,
    pub fields: Vec<Field>,
}

impl Interface {
    pub fn new(name: impl Into<String>, kind: Kind, fields: Vec<Field>) -> Self {
        Self {
            name: name.into(),
            kind,
            fields,
        }
    }
}

/// Error returned when interface discovery fails.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Error {
    msg: String,
}

impl Error {
    pub fn new(msg: impl Into<String>) -> Self {
        Self { msg: msg.into() }
    }
}

impl fmt::Display for Error {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(&self.msg)
    }
}

impl std::error::Error for Error {}

/// Abstracts interface discovery from ROS 2.
pub trait Extractor {
    fn extract(&self) -> Result<Vec<Interface>, Error>;
}
