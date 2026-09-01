//! Interesting payload pool.
//!
//! Paper-backed part:
//! Only payloads that trigger a crash or a new system state are preserved
//! in the pool for future mutation.
//!
//! Reproduction choices (paper gaps):
//! The paper does not disclose pool-item selection probabilities, so
//! `pick_for_mutation` selects uniformly.

use crate::payload::Payload;
use rand::Rng;

#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum SelectionPolicy {
    #[default]
    Uniform,
}

impl SelectionPolicy {
    pub fn as_str(self) -> &'static str {
        match self {
            Self::Uniform => "uniform",
        }
    }
}

#[derive(Debug, Default)]
pub struct PayloadPool {
    items: Vec<Payload>,
}

impl PayloadPool {
    pub fn new() -> Self {
        Self { items: Vec::new() }
    }

    /// Keeps a payload for future mutation. Callers must only push
    /// payloads judged as crash or new-state by the state oracle.
    pub fn push(&mut self, payload: Payload) {
        self.items.push(payload);
    }

    pub fn is_empty(&self) -> bool {
        self.items.is_empty()
    }

    pub fn len(&self) -> usize {
        self.items.len()
    }

    pub fn iter(&self) -> std::slice::Iter<'_, Payload> {
        self.items.iter()
    }

    /// Selects a payload for mutation. The active selection policy is a
    /// reproduction choice; the paper does not disclose the distribution.
    pub fn pick_for_mutation(
        &self,
        rng: &mut impl Rng,
        policy: SelectionPolicy,
    ) -> Option<Payload> {
        if self.items.is_empty() {
            return None;
        }
        match policy {
            SelectionPolicy::Uniform => {
                Some(self.items[rng.gen_range(0..self.items.len())].clone())
            }
        }
    }
}
