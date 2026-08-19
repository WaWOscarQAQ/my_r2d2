//! Recursive payload mutation and type-conformant value generation.
//!
//! Paper-backed part:
//! When the pool is non-empty, R2D2 selects a payload that previously
//! triggered a new state and mutates it recursively based on data files
//! from the interface specification.
//!
//! Reproduction choices (paper gaps):
//! The operator set, operator weights, mutation energy, recursion depth,
//! per-type value ranges, and the array length distribution are not
//! disclosed by the paper and are parameterized here.

use crate::interface_extractor::{Primitive, TypeNode};
use crate::payload::{Value, ValueTree};
use crate::payload_generator::GeneratorConfig;
use rand::Rng;

/// The mutation operators implemented by this reproduction.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum OpKind {
    /// Invert a bool, negate a float, or flip bits of an integer.
    Flip,
    /// Replace a numeric leaf with a boundary value of its range.
    Boundary,
    /// Regenerate the whole subtree type-conformantly.
    Resample,
    /// Grow or shrink a string, byte sequence, or variable-length array.
    Resize,
    /// Replace one byte of a string or byte sequence.
    ByteEdit,
}

/// Per-operator weights used when picking among applicable operators.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct OperatorWeights {
    pub flip: u32,
    pub boundary: u32,
    pub resample: u32,
    pub resize: u32,
    pub byte_edit: u32,
}

impl Default for OperatorWeights {
    fn default() -> Self {
        Self {
            flip: 1,
            boundary: 1,
            resample: 4,
            resize: 2,
            byte_edit: 1,
        }
    }
}

impl OperatorWeights {
    fn get(self, op: OpKind) -> u32 {
        match op {
            OpKind::Flip => self.flip,
            OpKind::Boundary => self.boundary,
            OpKind::Resample => self.resample,
            OpKind::Resize => self.resize,
            OpKind::ByteEdit => self.byte_edit,
        }
    }
}

/// Operators allowed per node kind.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct OperatorsPerType {
    /// Numeric and bool leaves.
    pub primitive: Vec<OpKind>,
    /// String and byte sequence leaves.
    pub string: Vec<OpKind>,
    /// Array nodes.
    pub array: Vec<OpKind>,
    /// Nested message nodes.
    pub nested: Vec<OpKind>,
}

impl Default for OperatorsPerType {
    fn default() -> Self {
        Self {
            primitive: vec![OpKind::Flip, OpKind::Boundary, OpKind::Resample],
            string: vec![OpKind::Resize, OpKind::ByteEdit, OpKind::Resample],
            array: vec![OpKind::Resize, OpKind::Resample],
            nested: vec![OpKind::Resample],
        }
    }
}

/// Generates a value tree conforming to `ty` node for node.
pub fn generate_value(ty: &TypeNode, rng: &mut impl Rng, config: &GeneratorConfig) -> ValueTree {
    match ty {
        TypeNode::Primitive(primitive) => {
            ValueTree::Leaf(generate_primitive(*primitive, rng, config))
        }
        TypeNode::Nested(fields) => ValueTree::Nested(
            fields
                .iter()
                .map(|field| generate_value(&field.ty, rng, config))
                .collect(),
        ),
        TypeNode::Array(element, fixed_len) => {
            let len = match fixed_len {
                Some(len) => *len,
                None => rng.gen_range(config.array_len_range.clone()),
            };
            ValueTree::Array(
                (0..len)
                    .map(|_| generate_value(element, rng, config))
                    .collect(),
            )
        }
    }
}

fn generate_primitive(primitive: Primitive, rng: &mut impl Rng, config: &GeneratorConfig) -> Value {
    let range = config.per_type_value_ranges.get(primitive);
    match primitive {
        Primitive::Bool => Value::Bool(rng.gen_bool(0.5)),
        Primitive::I8 | Primitive::I16 | Primitive::I32 | Primitive::I64 => {
            let value = rng.gen_range((range.min as i64)..=(range.max as i64));
            match primitive {
                Primitive::I8 => Value::I8(value as i8),
                Primitive::I16 => Value::I16(value as i16),
                Primitive::I32 => Value::I32(value as i32),
                _ => Value::I64(value),
            }
        }
        Primitive::U8 | Primitive::U16 | Primitive::U32 | Primitive::U64 => {
            let min = range.min.max(0.0) as u64;
            let max = range.max.max(0.0) as u64;
            let value = rng.gen_range(min..=max);
            match primitive {
                Primitive::U8 => Value::U8(value as u8),
                Primitive::U16 => Value::U16(value as u16),
                Primitive::U32 => Value::U32(value as u32),
                _ => Value::U64(value),
            }
        }
        Primitive::F32 => Value::F32(rng.gen_range(range.min..=range.max) as f32),
        Primitive::F64 => Value::F64(rng.gen_range(range.min..=range.max)),
        Primitive::String => {
            let len = len_range(primitive, config, rng);
            Value::String(random_printable(rng, len))
        }
        Primitive::Bytes => {
            let len = len_range(primitive, config, rng);
            Value::Bytes(random_bytes(rng, len))
        }
    }
}

fn len_range(primitive: Primitive, config: &GeneratorConfig, rng: &mut impl Rng) -> usize {
    let range = config.per_type_value_ranges.get(primitive);
    rng.gen_range((range.min as usize)..=(range.max as usize))
}

fn random_printable(rng: &mut impl Rng, len: usize) -> String {
    (0..len)
        .map(|_| rng.gen_range(0x20u8..=0x7e) as char)
        .collect()
}

fn random_bytes(rng: &mut impl Rng, len: usize) -> Vec<u8> {
    (0..len).map(|_| rng.r#gen()).collect()
}

/// Applies operator hits at randomly chosen nodes of the value tree.
pub struct Mutator {
    config: GeneratorConfig,
}

impl Mutator {
    pub fn new(config: GeneratorConfig) -> Self {
        Self { config }
    }

    /// Applies up to `config.mutation_energy` operator hits. Each hit
    /// picks a random node within `config.max_recursion_depth` and applies
    /// one weighted, applicable operator there.
    pub fn mutate(&self, value: &ValueTree, ty: &TypeNode, rng: &mut impl Rng) -> ValueTree {
        let mut out = value.clone();
        let mut energy = self.config.mutation_energy;
        while energy > 0 {
            let paths = collect_paths(&out, ty, self.config.max_recursion_depth);
            if paths.is_empty() {
                break;
            }
            let path = paths[rng.gen_range(0..paths.len())].clone();
            let Some(node_ty) = type_at(ty, &path) else {
                break;
            };
            let Some(node_value) = value_at(&out, &path) else {
                break;
            };
            let Some(op) = pick_operator(node_value, node_ty, &self.config, rng) else {
                break;
            };
            out = apply_at(out, ty, &path, op, &self.config, rng);
            energy -= 1;
        }
        out
    }
}

/// Collects the paths of all nodes reachable within `max_depth` from the
/// root, including the root itself.
fn collect_paths(value: &ValueTree, ty: &TypeNode, max_depth: u32) -> Vec<Vec<usize>> {
    let mut out = Vec::new();
    walk_paths(value, ty, 0, max_depth, &mut Vec::new(), &mut out);
    out
}

fn walk_paths(
    value: &ValueTree,
    ty: &TypeNode,
    depth: u32,
    max_depth: u32,
    path: &mut Vec<usize>,
    out: &mut Vec<Vec<usize>>,
) {
    out.push(path.clone());
    if depth >= max_depth {
        return;
    }
    match (value, ty) {
        (ValueTree::Nested(values), TypeNode::Nested(fields)) if values.len() == fields.len() => {
            for (index, (child, field)) in values.iter().zip(fields).enumerate() {
                path.push(index);
                walk_paths(child, &field.ty, depth + 1, max_depth, path, out);
                path.pop();
            }
        }
        (ValueTree::Array(values), TypeNode::Array(element, _)) => {
            for (index, child) in values.iter().enumerate() {
                path.push(index);
                walk_paths(child, element, depth + 1, max_depth, path, out);
                path.pop();
            }
        }
        _ => {}
    }
}

fn type_at<'a>(ty: &'a TypeNode, path: &[usize]) -> Option<&'a TypeNode> {
    let mut current = ty;
    for &index in path {
        current = match current {
            TypeNode::Nested(fields) => &fields.get(index)?.ty,
            TypeNode::Array(element, _) => element.as_ref(),
            TypeNode::Primitive(_) => return None,
        };
    }
    Some(current)
}

fn value_at<'a>(value: &'a ValueTree, path: &[usize]) -> Option<&'a ValueTree> {
    let mut current = value;
    for &index in path {
        current = match current {
            ValueTree::Nested(values) => values.get(index)?,
            ValueTree::Array(values) => values.get(index)?,
            ValueTree::Leaf(_) => return None,
        };
    }
    Some(current)
}

fn pick_operator(
    value: &ValueTree,
    ty: &TypeNode,
    config: &GeneratorConfig,
    rng: &mut impl Rng,
) -> Option<OpKind> {
    let allowed = match (value, ty) {
        (ValueTree::Leaf(Value::String(_) | Value::Bytes(_)), TypeNode::Primitive(_)) => {
            &config.operators_per_type.string
        }
        (ValueTree::Leaf(_), TypeNode::Primitive(_)) => &config.operators_per_type.primitive,
        (ValueTree::Array(_), TypeNode::Array(_, _)) => &config.operators_per_type.array,
        (ValueTree::Nested(_), TypeNode::Nested(_)) => &config.operators_per_type.nested,
        _ => return None,
    };
    let candidates: Vec<OpKind> = allowed
        .iter()
        .copied()
        .filter(|op| applicable(*op, value, ty))
        .collect();
    let total: u32 = candidates
        .iter()
        .map(|op| config.operator_weights.get(*op))
        .sum();
    if total == 0 {
        return None;
    }
    let mut draw = rng.gen_range(0..total);
    for op in candidates {
        let weight = config.operator_weights.get(op);
        if draw < weight {
            return Some(op);
        }
        draw -= weight;
    }
    None
}

fn applicable(op: OpKind, value: &ValueTree, ty: &TypeNode) -> bool {
    match op {
        OpKind::Flip => matches!(
            value,
            ValueTree::Leaf(
                Value::Bool(_)
                    | Value::I8(_)
                    | Value::U8(_)
                    | Value::I16(_)
                    | Value::U16(_)
                    | Value::I32(_)
                    | Value::U32(_)
                    | Value::I64(_)
                    | Value::U64(_)
                    | Value::F32(_)
                    | Value::F64(_)
            )
        ),
        OpKind::Boundary => matches!(
            value,
            ValueTree::Leaf(
                Value::I8(_)
                    | Value::U8(_)
                    | Value::I16(_)
                    | Value::U16(_)
                    | Value::I32(_)
                    | Value::U32(_)
                    | Value::I64(_)
                    | Value::U64(_)
                    | Value::F32(_)
                    | Value::F64(_)
            )
        ),
        OpKind::Resample => true,
        OpKind::Resize => matches!(
            (value, ty),
            (ValueTree::Leaf(Value::String(_) | Value::Bytes(_)), _)
                | (ValueTree::Array(_), TypeNode::Array(_, None))
        ),
        OpKind::ByteEdit => {
            matches!(value, ValueTree::Leaf(Value::String(_) | Value::Bytes(_)))
        }
    }
}

fn apply_at(
    value: ValueTree,
    ty: &TypeNode,
    path: &[usize],
    op: OpKind,
    config: &GeneratorConfig,
    rng: &mut impl Rng,
) -> ValueTree {
    if path.is_empty() {
        return apply_op(value, ty, op, config, rng);
    }
    match (value, ty) {
        (ValueTree::Nested(mut values), TypeNode::Nested(fields)) => {
            if let (Some(child), Some(field)) = (values.get(path[0]).cloned(), fields.get(path[0]))
            {
                values[path[0]] = apply_at(child, &field.ty, &path[1..], op, config, rng);
            }
            ValueTree::Nested(values)
        }
        (ValueTree::Array(mut values), TypeNode::Array(element, _)) => {
            if let Some(child) = values.get(path[0]).cloned() {
                values[path[0]] = apply_at(child, element, &path[1..], op, config, rng);
            }
            ValueTree::Array(values)
        }
        (other, _) => other,
    }
}

fn apply_op(
    value: ValueTree,
    ty: &TypeNode,
    op: OpKind,
    config: &GeneratorConfig,
    rng: &mut impl Rng,
) -> ValueTree {
    match op {
        OpKind::Resample => generate_value(ty, rng, config),
        OpKind::Flip => flip(value, rng),
        OpKind::Boundary => boundary(value, ty, config, rng),
        OpKind::Resize => resize(value, ty, config, rng),
        OpKind::ByteEdit => byte_edit(value, rng),
    }
}

fn flip(value: ValueTree, rng: &mut impl Rng) -> ValueTree {
    let flipped = match value {
        ValueTree::Leaf(Value::Bool(v)) => Value::Bool(!v),
        ValueTree::Leaf(Value::I8(v)) => Value::I8(v ^ (1 << rng.gen_range(0..8))),
        ValueTree::Leaf(Value::U8(v)) => Value::U8(v ^ (1 << rng.gen_range(0..8))),
        ValueTree::Leaf(Value::I16(v)) => Value::I16(v ^ (1 << rng.gen_range(0..16))),
        ValueTree::Leaf(Value::U16(v)) => Value::U16(v ^ (1 << rng.gen_range(0..16))),
        ValueTree::Leaf(Value::I32(v)) => Value::I32(v ^ (1 << rng.gen_range(0..32))),
        ValueTree::Leaf(Value::U32(v)) => Value::U32(v ^ (1 << rng.gen_range(0..32))),
        ValueTree::Leaf(Value::I64(v)) => Value::I64(v ^ (1 << rng.gen_range(0..64))),
        ValueTree::Leaf(Value::U64(v)) => Value::U64(v ^ (1 << rng.gen_range(0..64))),
        ValueTree::Leaf(Value::F32(v)) => Value::F32(-v),
        ValueTree::Leaf(Value::F64(v)) => Value::F64(-v),
        other => return other,
    };
    ValueTree::Leaf(flipped)
}

fn boundary(
    value: ValueTree,
    ty: &TypeNode,
    config: &GeneratorConfig,
    rng: &mut impl Rng,
) -> ValueTree {
    let TypeNode::Primitive(primitive) = ty else {
        return value;
    };
    let current = match &value {
        ValueTree::Leaf(Value::I8(v)) => Some(*v as f64),
        ValueTree::Leaf(Value::U8(v)) => Some(*v as f64),
        ValueTree::Leaf(Value::I16(v)) => Some(*v as f64),
        ValueTree::Leaf(Value::U16(v)) => Some(*v as f64),
        ValueTree::Leaf(Value::I32(v)) => Some(*v as f64),
        ValueTree::Leaf(Value::U32(v)) => Some(*v as f64),
        ValueTree::Leaf(Value::I64(v)) => Some(*v as f64),
        ValueTree::Leaf(Value::U64(v)) => Some(*v as f64),
        ValueTree::Leaf(Value::F32(v)) => Some(*v as f64),
        ValueTree::Leaf(Value::F64(v)) => Some(*v),
        _ => None,
    };
    let Some(current) = current else {
        return value;
    };
    let range = config.per_type_value_ranges.get(*primitive);
    let candidates: Vec<f64> = [range.min, 0.0, range.max]
        .into_iter()
        .filter(|candidate| (*candidate - current).abs() > f64::EPSILON)
        .collect();
    if candidates.is_empty() {
        return value;
    }
    let chosen = candidates[rng.gen_range(0..candidates.len())];
    let bounded = match primitive {
        Primitive::I8 => Value::I8(chosen as i64 as i8),
        Primitive::U8 => Value::U8(chosen.max(0.0) as u64 as u8),
        Primitive::I16 => Value::I16(chosen as i64 as i16),
        Primitive::U16 => Value::U16(chosen.max(0.0) as u64 as u16),
        Primitive::I32 => Value::I32(chosen as i64 as i32),
        Primitive::U32 => Value::U32(chosen.max(0.0) as u64 as u32),
        Primitive::I64 => Value::I64(chosen as i64),
        Primitive::U64 => Value::U64(chosen.max(0.0) as u64),
        Primitive::F32 => Value::F32(chosen as f32),
        Primitive::F64 => Value::F64(chosen),
        _ => return value,
    };
    ValueTree::Leaf(bounded)
}

fn resize(
    value: ValueTree,
    ty: &TypeNode,
    config: &GeneratorConfig,
    rng: &mut impl Rng,
) -> ValueTree {
    match (value, ty) {
        (ValueTree::Leaf(Value::String(mut text)), TypeNode::Primitive(_)) => {
            let len = len_range(Primitive::String, config, rng);
            if text.len() > len {
                text.truncate(len);
            } else {
                text.extend(
                    std::iter::repeat_with(|| rng.gen_range(0x20u8..=0x7e) as char)
                        .take(len - text.len()),
                );
            }
            ValueTree::Leaf(Value::String(text))
        }
        (ValueTree::Leaf(Value::Bytes(mut bytes)), TypeNode::Primitive(_)) => {
            let len = len_range(Primitive::Bytes, config, rng);
            if bytes.len() > len {
                bytes.truncate(len);
            } else {
                bytes.extend(std::iter::repeat_with(|| rng.r#gen::<u8>()).take(len - bytes.len()));
            }
            ValueTree::Leaf(Value::Bytes(bytes))
        }
        (ValueTree::Array(mut items), TypeNode::Array(element, None)) => {
            let len = rng.gen_range(config.array_len_range.clone());
            if items.len() > len {
                items.truncate(len);
            } else {
                items.extend(
                    std::iter::repeat_with(|| generate_value(element, rng, config))
                        .take(len - items.len()),
                );
            }
            ValueTree::Array(items)
        }
        (other, _) => other,
    }
}

fn byte_edit(value: ValueTree, rng: &mut impl Rng) -> ValueTree {
    match value {
        ValueTree::Leaf(Value::String(text)) => {
            let mut bytes = text.into_bytes();
            if bytes.is_empty() {
                return ValueTree::Leaf(Value::String(String::new()));
            }
            let index = rng.gen_range(0..bytes.len());
            bytes[index] = rng.gen_range(0x20u8..=0x7e);
            let text = String::from_utf8(bytes).expect("printable ASCII stays valid UTF-8");
            ValueTree::Leaf(Value::String(text))
        }
        ValueTree::Leaf(Value::Bytes(mut bytes)) => {
            if bytes.is_empty() {
                return ValueTree::Leaf(Value::Bytes(bytes));
            }
            let index = rng.gen_range(0..bytes.len());
            bytes[index] = rng.r#gen::<u8>();
            ValueTree::Leaf(Value::Bytes(bytes))
        }
        other => other,
    }
}
