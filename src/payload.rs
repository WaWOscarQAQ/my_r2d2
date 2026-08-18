//! Payload value model and serialization boundary.
//!
//! Paper-backed part:
//! Prepared payloads are sent to the ROS system for execution and are kept
//! in the pool when they trigger a crash or a new state.
//!
//! Reproduction choices (paper gaps):
//! The paper does not disclose the payload serialization implementation,
//! so the concrete wire format behind `Serializer` is a reproduction
//! choice (`SimpleSerializer`).

use crate::interface_extractor::{Kind, Primitive, TypeNode};
use std::fmt;

/// A primitive value inside a payload.
#[derive(Debug, Clone, PartialEq)]
pub enum Value {
    Bool(bool),
    I8(i8),
    U8(u8),
    I16(i16),
    U16(u16),
    I32(i32),
    U32(u32),
    I64(i64),
    U64(u64),
    F32(f32),
    F64(f64),
    String(String),
    Bytes(Vec<u8>),
}

/// A structured value tree mirroring `TypeNode` node for node.
#[derive(Debug, Clone, PartialEq)]
pub enum ValueTree {
    Leaf(Value),
    Nested(Vec<ValueTree>),
    Array(Vec<ValueTree>),
}

/// A fuzzing payload bound to one interface.
#[derive(Debug, Clone, PartialEq)]
pub struct Payload {
    pub interface_id: String,
    pub kind: Kind,
    pub value: ValueTree,
    /// Seed of the RNG state used in the round that produced this payload.
    pub rng_seed: u64,
    /// Serialized wire bytes, filled by a `Serializer` before sending.
    pub serialized: Vec<u8>,
}

impl Payload {
    pub fn new(interface_id: impl Into<String>, kind: Kind, value: ValueTree, rng_seed: u64) -> Self {
        Self {
            interface_id: interface_id.into(),
            kind,
            value,
            rng_seed,
            serialized: Vec::new(),
        }
    }
}

/// Serialization boundary between the value tree and the wire format.
pub trait Serializer {
    fn serialize(&self, value: &ValueTree, ty: &TypeNode) -> Result<Vec<u8>, Error>;
    fn deserialize(&self, bytes: &[u8], ty: &TypeNode) -> Result<ValueTree, Error>;
}

/// Errors produced by payload construction and serialization.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum Error {
    /// The value tree does not match the type tree.
    TypeMismatch { expected: String, found: String },
    /// Bytes do not decode against the given type tree.
    Malformed(String),
    /// The requested operation is outside the reproduction scope.
    Unsupported(String),
}

impl fmt::Display for Error {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Error::TypeMismatch { expected, found } => {
                write!(f, "type mismatch: expected {expected}, found {found}")
            }
            Error::Malformed(detail) => write!(f, "malformed payload bytes: {detail}"),
            Error::Unsupported(detail) => write!(f, "unsupported: {detail}"),
        }
    }
}

impl std::error::Error for Error {}

/// Deterministic custom wire format used until a paper-conformant
/// serialization is available.
///
/// Layout: bool is one byte; integers are little-endian fixed width;
/// `f32`/`f64` are little-endian bit patterns; strings and byte sequences
/// are a `u32` little-endian length followed by the payload; arrays are a
/// `u32` little-endian element count followed by the elements; nested
/// messages are their fields in order without framing, because the shape
/// is recovered from the type tree during deserialization.
#[derive(Debug, Clone, Copy, Default)]
pub struct SimpleSerializer;

impl Serializer for SimpleSerializer {
    fn serialize(&self, value: &ValueTree, ty: &TypeNode) -> Result<Vec<u8>, Error> {
        let mut out = Vec::new();
        encode(value, ty, &mut out)?;
        Ok(out)
    }

    fn deserialize(&self, bytes: &[u8], ty: &TypeNode) -> Result<ValueTree, Error> {
        let mut cursor = bytes;
        let value = decode(&mut cursor, ty)?;
        if !cursor.is_empty() {
            return Err(Error::Malformed(format!("{} trailing bytes", cursor.len())));
        }
        Ok(value)
    }
}

fn encode(value: &ValueTree, ty: &TypeNode, out: &mut Vec<u8>) -> Result<(), Error> {
    match (value, ty) {
        (ValueTree::Leaf(Value::Bool(v)), TypeNode::Primitive(Primitive::Bool)) => {
            out.push(u8::from(*v));
        }
        (ValueTree::Leaf(Value::I8(v)), TypeNode::Primitive(Primitive::I8)) => {
            out.extend_from_slice(&v.to_le_bytes());
        }
        (ValueTree::Leaf(Value::U8(v)), TypeNode::Primitive(Primitive::U8)) => {
            out.extend_from_slice(&v.to_le_bytes());
        }
        (ValueTree::Leaf(Value::I16(v)), TypeNode::Primitive(Primitive::I16)) => {
            out.extend_from_slice(&v.to_le_bytes());
        }
        (ValueTree::Leaf(Value::U16(v)), TypeNode::Primitive(Primitive::U16)) => {
            out.extend_from_slice(&v.to_le_bytes());
        }
        (ValueTree::Leaf(Value::I32(v)), TypeNode::Primitive(Primitive::I32)) => {
            out.extend_from_slice(&v.to_le_bytes());
        }
        (ValueTree::Leaf(Value::U32(v)), TypeNode::Primitive(Primitive::U32)) => {
            out.extend_from_slice(&v.to_le_bytes());
        }
        (ValueTree::Leaf(Value::I64(v)), TypeNode::Primitive(Primitive::I64)) => {
            out.extend_from_slice(&v.to_le_bytes());
        }
        (ValueTree::Leaf(Value::U64(v)), TypeNode::Primitive(Primitive::U64)) => {
            out.extend_from_slice(&v.to_le_bytes());
        }
        (ValueTree::Leaf(Value::F32(v)), TypeNode::Primitive(Primitive::F32)) => {
            out.extend_from_slice(&v.to_bits().to_le_bytes());
        }
        (ValueTree::Leaf(Value::F64(v)), TypeNode::Primitive(Primitive::F64)) => {
            out.extend_from_slice(&v.to_bits().to_le_bytes());
        }
        (ValueTree::Leaf(Value::String(s)), TypeNode::Primitive(Primitive::String)) => {
            push_len(s.len(), out)?;
            out.extend_from_slice(s.as_bytes());
        }
        (ValueTree::Leaf(Value::Bytes(b)), TypeNode::Primitive(Primitive::Bytes)) => {
            push_len(b.len(), out)?;
            out.extend_from_slice(b);
        }
        (ValueTree::Nested(values), TypeNode::Nested(fields)) => {
            if values.len() != fields.len() {
                return Err(Error::TypeMismatch {
                    expected: format!("nested message with {} fields", fields.len()),
                    found: format!("nested value with {} fields", values.len()),
                });
            }
            for (v, field) in values.iter().zip(fields) {
                encode(v, &field.ty, out)?;
            }
        }
        (ValueTree::Array(values), TypeNode::Array(element, fixed_len)) => {
            if let Some(expected) = fixed_len
                && values.len() != *expected
            {
                return Err(Error::TypeMismatch {
                    expected: format!("fixed array of {expected} elements"),
                    found: format!("array of {} elements", values.len()),
                });
            }
            push_len(values.len(), out)?;
            for v in values {
                encode(v, element, out)?;
            }
        }
        (other, ty) => {
            return Err(Error::TypeMismatch {
                expected: format!("{ty:?}"),
                found: format!("{other:?}"),
            });
        }
    }
    Ok(())
}

fn push_len(len: usize, out: &mut Vec<u8>) -> Result<(), Error> {
    let len = u32::try_from(len).map_err(|_| Error::Malformed("length exceeds u32".to_string()))?;
    out.extend_from_slice(&len.to_le_bytes());
    Ok(())
}

fn decode(cursor: &mut &[u8], ty: &TypeNode) -> Result<ValueTree, Error> {
    match ty {
        TypeNode::Primitive(Primitive::Bool) => {
            let byte = take(cursor, 1)?[0];
            Ok(ValueTree::Leaf(Value::Bool(byte != 0)))
        }
        TypeNode::Primitive(Primitive::String) => {
            let len = read_len(cursor)?;
            let bytes = take(cursor, len)?.to_vec();
            let text = String::from_utf8(bytes)
                .map_err(|_| Error::Malformed("string is not valid UTF-8".to_string()))?;
            Ok(ValueTree::Leaf(Value::String(text)))
        }
        TypeNode::Primitive(Primitive::Bytes) => {
            let len = read_len(cursor)?;
            let bytes = take(cursor, len)?.to_vec();
            Ok(ValueTree::Leaf(Value::Bytes(bytes)))
        }
        TypeNode::Primitive(primitive) => {
            let width = numeric_width(*primitive);
            let bytes = take(cursor, width)?;
            decode_numeric(*primitive, bytes)
        }
        TypeNode::Nested(fields) => {
            let mut values = Vec::with_capacity(fields.len());
            for field in fields {
                values.push(decode(cursor, &field.ty)?);
            }
            Ok(ValueTree::Nested(values))
        }
        TypeNode::Array(element, fixed_len) => {
            let count = read_len(cursor)?;
            if let Some(expected) = fixed_len
                && count != *expected
            {
                return Err(Error::Malformed(format!(
                    "fixed array length mismatch: expected {expected}, found {count}"
                )));
            }
            let mut values = Vec::with_capacity(count);
            for _ in 0..count {
                values.push(decode(cursor, element)?);
            }
            Ok(ValueTree::Array(values))
        }
    }
}

fn read_len(cursor: &mut &[u8]) -> Result<usize, Error> {
    let bytes: [u8; 4] = take(cursor, 4)?.try_into().expect("slice of length 4");
    Ok(u32::from_le_bytes(bytes) as usize)
}

fn take<'a>(cursor: &mut &'a [u8], n: usize) -> Result<&'a [u8], Error> {
    if cursor.len() < n {
        return Err(Error::Malformed(format!(
            "expected {n} bytes, {} remaining",
            cursor.len()
        )));
    }
    let (head, rest) = cursor.split_at(n);
    *cursor = rest;
    Ok(head)
}

fn numeric_width(primitive: Primitive) -> usize {
    match primitive {
        Primitive::I8 | Primitive::U8 => 1,
        Primitive::I16 | Primitive::U16 => 2,
        Primitive::I32 | Primitive::U32 | Primitive::F32 => 4,
        Primitive::I64 | Primitive::U64 | Primitive::F64 => 8,
        Primitive::Bool | Primitive::String | Primitive::Bytes => unreachable!(),
    }
}

fn decode_numeric(primitive: Primitive, bytes: &[u8]) -> Result<ValueTree, Error> {
    let array = |bytes: &[u8]| -> [u8; 8] {
        let mut padded = [0u8; 8];
        padded[..bytes.len()].copy_from_slice(bytes);
        padded
    };
    let value = match primitive {
        Primitive::I8 => Value::I8(i8::from_le_bytes([bytes[0]])),
        Primitive::U8 => Value::U8(bytes[0]),
        Primitive::I16 => Value::I16(i16::from_le_bytes([bytes[0], bytes[1]])),
        Primitive::U16 => Value::U16(u16::from_le_bytes([bytes[0], bytes[1]])),
        Primitive::I32 => Value::I32(i32::from_le_bytes(array(bytes)[..4].try_into().unwrap())),
        Primitive::U32 => Value::U32(u32::from_le_bytes(array(bytes)[..4].try_into().unwrap())),
        Primitive::I64 => Value::I64(i64::from_le_bytes(array(bytes))),
        Primitive::U64 => Value::U64(u64::from_le_bytes(array(bytes))),
        Primitive::F32 => Value::F32(f32::from_bits(u32::from_le_bytes(
            array(bytes)[..4].try_into().unwrap(),
        ))),
        Primitive::F64 => Value::F64(f64::from_bits(u64::from_le_bytes(array(bytes)))),
        Primitive::Bool | Primitive::String | Primitive::Bytes => unreachable!(),
    };
    Ok(ValueTree::Leaf(value))
}
