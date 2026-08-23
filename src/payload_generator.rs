//! Payload generation loop boundary.
//!
//! Paper-backed part:
//! When the pool is empty R2D2 selects an interface randomly from the
//! extracted specifications and generates a payload conforming to its
//! specification; otherwise it selects a pool payload that previously
//! triggered a new state and mutates it recursively based on the
//! interface data files.
//!
//! Reproduction choices (paper gaps):
//! Interface and pool-item selection distributions are not disclosed by
//! the paper; this reproduction makes the active selection policy
//! explicit and defaults both to uniform.

use crate::interface_extractor::{Interface, Primitive, TypeNode};
use crate::mutation::{Mutator, OperatorWeights, OperatorsPerType, generate_value};
use crate::payload::{Error, Payload, Serializer, SimpleSerializer};
use crate::payload_pool::{PayloadPool, SelectionPolicy};
use rand::{Rng, SeedableRng, rngs::StdRng};
use std::collections::BTreeMap;

/// A numeric range used for value generation and boundary mutation.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ValueRange {
    pub min: f64,
    pub max: f64,
}

impl ValueRange {
    pub fn new(min: f64, max: f64) -> Self {
        Self { min, max }
    }
}

/// Per-primitive value ranges. `String`/`Bytes` ranges are interpreted as
/// byte length ranges; the `Bool` range is unused because bools are drawn
/// uniformly.
#[derive(Debug, Clone, PartialEq)]
pub struct ValueRanges {
    ranges: BTreeMap<Primitive, ValueRange>,
}

impl ValueRanges {
    pub fn insert(&mut self, primitive: Primitive, range: ValueRange) {
        self.ranges.insert(primitive, range);
    }

    pub fn get(&self, primitive: Primitive) -> ValueRange {
        self.ranges
            .get(&primitive)
            .copied()
            .unwrap_or_else(|| default_range(primitive))
    }
}

fn default_range(primitive: Primitive) -> ValueRange {
    match primitive {
        Primitive::Bool => ValueRange::new(0.0, 1.0),
        Primitive::I8
        | Primitive::I16
        | Primitive::I32
        | Primitive::I64
        | Primitive::F32
        | Primitive::F64 => ValueRange::new(-1000.0, 1000.0),
        Primitive::U8 | Primitive::U16 | Primitive::U32 | Primitive::U64 => {
            ValueRange::new(0.0, 1000.0)
        }
        Primitive::String | Primitive::Bytes => ValueRange::new(0.0, 64.0),
    }
}

impl Default for ValueRanges {
    fn default() -> Self {
        let primitives = [
            Primitive::Bool,
            Primitive::I8,
            Primitive::U8,
            Primitive::I16,
            Primitive::U16,
            Primitive::I32,
            Primitive::U32,
            Primitive::I64,
            Primitive::U64,
            Primitive::F32,
            Primitive::F64,
            Primitive::String,
            Primitive::Bytes,
        ];
        let mut ranges = Self {
            ranges: BTreeMap::new(),
        };
        for primitive in primitives {
            ranges.insert(primitive, default_range(primitive));
        }
        ranges
    }
}

/// All parameterized generation and mutation knobs.
///
/// The interface and pool-item selection distributions are not disclosed
/// by the paper. This reproduction therefore exposes explicit selection
/// policies instead of pretending to implement a paper probability model.
#[derive(Debug, Clone, PartialEq)]
pub struct GeneratorConfig {
    /// Number of operator hits applied per mutation call.
    pub mutation_energy: u32,
    /// Maximum descent depth for one operator hit.
    pub max_recursion_depth: u32,
    /// Length distribution of variable-length arrays.
    pub array_len_range: std::ops::RangeInclusive<usize>,
    pub interface_selection: SelectionPolicy,
    pub pool_selection: SelectionPolicy,
    pub per_type_value_ranges: ValueRanges,
    pub operator_weights: OperatorWeights,
    pub operators_per_type: OperatorsPerType,
}

impl Default for GeneratorConfig {
    fn default() -> Self {
        Self {
            mutation_energy: 8,
            max_recursion_depth: 8,
            array_len_range: 0..=8,
            interface_selection: SelectionPolicy::Uniform,
            pool_selection: SelectionPolicy::Uniform,
            per_type_value_ranges: ValueRanges::default(),
            operator_weights: OperatorWeights::default(),
            operators_per_type: OperatorsPerType::default(),
        }
    }
}

/// Sends a payload to the system under test.
pub trait Sender {
    fn send(&self, payload: &Payload) -> Result<(), Error>;
}

/// Queries the execution outcome. The real oracle compares the callback
/// trace against the global state in the feedback controller phase; this
/// boundary keeps the generator free of that logic.
pub trait StateOracle {
    fn is_new_state(&self) -> bool;
    fn crashed(&self) -> bool;
}

/// The per-round decision maker described by the paper.
pub struct PayloadGenerator {
    interfaces: Vec<Interface>,
    pool: PayloadPool,
    config: GeneratorConfig,
    base_seed: u64,
    round: u64,
    rng: StdRng,
}

impl PayloadGenerator {
    pub fn new(interfaces: Vec<Interface>, config: GeneratorConfig, seed: u64) -> Self {
        Self {
            interfaces,
            pool: PayloadPool::new(),
            config,
            base_seed: seed,
            round: 0,
            rng: StdRng::seed_from_u64(seed),
        }
    }

    pub fn pool(&self) -> &PayloadPool {
        &self.pool
    }

    pub fn pool_mut(&mut self) -> &mut PayloadPool {
        &mut self.pool
    }

    /// Preserves only payloads judged interesting by the state oracle.
    pub fn retain_if_interesting(&mut self, payload: Payload, oracle: &impl StateOracle) -> bool {
        if oracle.is_new_state() || oracle.crashed() {
            self.pool.push(payload);
            true
        } else {
            false
        }
    }

    /// Produces the next payload and advances the round counter.
    ///
    /// Each round reseeds the RNG from a seed derived from `base_seed`
    /// and the round index, which makes a round replayable when the pool
    /// state at that round is reproduced.
    pub fn next_payload(&mut self) -> Result<Payload, Error> {
        let round_seed = self.base_seed.wrapping_add(self.round);
        self.rng = StdRng::seed_from_u64(round_seed);
        self.round += 1;

        let payload = if self.pool.is_empty() {
            if self.interfaces.is_empty() {
                return Err(Error::Unsupported(
                    "no interfaces extracted; run a dry run first".to_string(),
                ));
            }
            let index = self.select_interface_index()?;
            let interface = &self.interfaces[index];
            let ty = top_level_type(interface);
            let value = generate_value(&ty, &mut self.rng, &self.config);
            Payload::new(interface.name.clone(), interface.kind, value, round_seed)
        } else {
            let picked = self
                .pool
                .pick_for_mutation(&mut self.rng, self.config.pool_selection)
                .ok_or_else(|| {
                    Error::Unsupported("payload pool became empty mid-round".to_string())
                })?;
            let interface = self.interface(&picked.interface_id).ok_or_else(|| {
                Error::Unsupported(format!(
                    "pool payload references unknown interface {:?}",
                    picked.interface_id
                ))
            })?;
            let ty = top_level_type(interface);
            let value = Mutator::new(self.config.clone()).mutate(&picked.value, &ty, &mut self.rng);
            Payload::new(picked.interface_id.clone(), picked.kind, value, round_seed)
        };

        let interface = self
            .interface(&payload.interface_id)
            .expect("payload interface came from the extracted list");
        let mut payload = payload;
        payload.serialized =
            SimpleSerializer.serialize(&payload.value, &top_level_type(interface))?;
        Ok(payload)
    }

    fn interface(&self, id: &str) -> Option<&Interface> {
        self.interfaces
            .iter()
            .find(|interface| interface.name == id)
    }

    fn select_interface_index(&mut self) -> Result<usize, Error> {
        match self.config.interface_selection {
            SelectionPolicy::Uniform => Ok(self.rng.gen_range(0..self.interfaces.len())),
        }
    }
}

/// The payload shape of an interface: its top-level fields as one nested
/// message. Service responses are outside the reproduction scope.
fn top_level_type(interface: &Interface) -> TypeNode {
    TypeNode::Nested(interface.fields.clone())
}
