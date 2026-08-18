//! ROS interface discovery boundary.
//!
//! Paper-backed part:
//! R2D2 extracts ROS interfaces and uses their data structure to build
//! inputs. Mutation is conducted recursively based on data files from the
//! interface specification, so each field carries a recursive type tree.
//!
//! Placeholder part:
//! This version defines the extraction boundary but does not inspect a
//! ROS 2 graph yet, and does not parse `.msg` or `.srv` sources.

use std::fmt;

/// The kind of ROS interface that can carry a fuzzing payload.
///
/// Only topics and services are in scope; actions are intentionally
/// excluded per the reproduction contract.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum Kind {
    Topic,
    Service,
}

/// ROS primitive value kinds.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, PartialOrd, Ord)]
pub enum Primitive {
    Bool,
    I8,
    U8,
    I16,
    U16,
    I32,
    U32,
    I64,
    U64,
    F32,
    F64,
    /// Variable-length UTF-8 string.
    String,
    /// Variable-length byte sequence (`uint8[]`).
    Bytes,
}

/// One field in a message or service shape.
#[derive(Debug, Clone, PartialEq)]
pub struct Field {
    pub name: String,
    pub ty: TypeNode,
}

impl Field {
    pub fn new(name: impl Into<String>, ty: impl Into<TypeNode>) -> Self {
        Self {
            name: name.into(),
            ty: ty.into(),
        }
    }
}

/// A node in the recursive type tree describing an interface payload.
#[derive(Debug, Clone, PartialEq)]
pub enum TypeNode {
    /// A primitive leaf value.
    Primitive(Primitive),
    /// A nested message: an ordered sequence of named fields.
    Nested(Vec<Field>),
    /// An array over an element type. `Some(len)` fixes the length;
    /// `None` is a variable-length sequence.
    Array(Box<TypeNode>, Option<usize>),
}

impl TypeNode {
    pub fn primitive(primitive: Primitive) -> Self {
        Self::Primitive(primitive)
    }

    pub fn nested(fields: Vec<Field>) -> Self {
        Self::Nested(fields)
    }

    pub fn array(element: TypeNode) -> Self {
        Self::Array(Box::new(element), None)
    }

    pub fn fixed_array(element: TypeNode, len: usize) -> Self {
        Self::Array(Box::new(element), Some(len))
    }
}

/// Maps a ROS primitive type name to a leaf type node.
///
/// Panics on unknown names: nested and array types have no string form in
/// this placeholder and must be built with the `TypeNode` constructors.
impl From<&str> for TypeNode {
    fn from(name: &str) -> Self {
        let primitive = match name {
            "bool" => Primitive::Bool,
            "int8" => Primitive::I8,
            "uint8" => Primitive::U8,
            "int16" => Primitive::I16,
            "uint16" => Primitive::U16,
            "int32" => Primitive::I32,
            "uint32" => Primitive::U32,
            "int64" => Primitive::I64,
            "uint64" => Primitive::U64,
            "float32" => Primitive::F32,
            "float64" => Primitive::F64,
            "string" => Primitive::String,
            other => panic!(
                "unknown primitive type {other:?}; nested and array types must use TypeNode constructors"
            ),
        };
        TypeNode::Primitive(primitive)
    }
}

/// A data file associated with an interface.
///
/// The parsed type tree lives in `Field::ty`; this structure records the
/// file name and raw source for provenance and the reproduction report.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct DataFile {
    /// File path or message type name, e.g. `geometry_msgs/msg/Twist.msg`.
    pub name: String,
    /// Raw `.msg` or `.srv` source text, unparsed in this placeholder.
    pub source: String,
}

impl DataFile {
    pub fn new(name: impl Into<String>, source: impl Into<String>) -> Self {
        Self {
            name: name.into(),
            source: source.into(),
        }
    }
}

/// A discovered ROS interface and its top-level fields.
#[derive(Debug, Clone, PartialEq)]
pub struct Interface {
    pub name: String,
    pub kind: Kind,
    pub fields: Vec<Field>,
    pub data_files: Vec<DataFile>,
}

impl Interface {
    pub fn new(name: impl Into<String>, kind: Kind, fields: Vec<Field>) -> Self {
        Self {
            name: name.into(),
            kind,
            fields,
            data_files: Vec::new(),
        }
    }

    pub fn with_data_files(mut self, data_files: Vec<DataFile>) -> Self {
        self.data_files = data_files;
        self
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
