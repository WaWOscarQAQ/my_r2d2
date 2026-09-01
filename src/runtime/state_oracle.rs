use crate::callback_profile::CallbackTrace;
use crate::payload_generator::StateOracle;
use serde::{Deserialize, Serialize};
use std::collections::{BTreeMap, BTreeSet};
use std::fs;
use std::path::Path;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum TraceDisposition {
    Analyzed,
    Empty,
    Invalid,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Default, Serialize, Deserialize)]
pub struct StateEvidence {
    pub new_edge: bool,
    pub new_callback: bool,
    pub new_message: bool,
    pub latency_deviation: bool,
    pub throughput_deviation: bool,
}

impl StateEvidence {
    pub fn structural_novelty(self) -> bool {
        self.new_edge || self.new_callback || self.new_message
    }

    pub fn callback_trace_new_state(self) -> bool {
        self.structural_novelty() || self.latency_deviation || self.throughput_deviation
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct OracleVerdict {
    pub new_state: bool,
    pub crashed: bool,
    pub trace: TraceDisposition,
    pub evidence: StateEvidence,
}

#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct DeviationThresholds {
    pub latency_factor: f64,
    pub throughput_floor: f64,
}

impl DeviationThresholds {
    pub fn new(latency_factor: f64, throughput_floor: f64) -> Self {
        Self {
            latency_factor,
            throughput_floor,
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct CallbackBenchmark {
    pub mean_latency: f64,
    pub samples: u64,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub mean_scheduling_latency: Option<f64>,
    #[serde(default)]
    pub scheduling_samples: u64,
}

#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct MessageBenchmark {
    pub mean_throughput: f64,
    pub samples: u64,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct BenchmarkModel {
    pub graph_edges: BTreeSet<(u64, u64)>,
    pub callback_latency: BTreeMap<u64, CallbackBenchmark>,
    pub message_throughput: BTreeMap<u64, MessageBenchmark>,
    pub analyzed_traces: u64,
}

impl BenchmarkModel {
    pub fn save_json(&self, path: &Path) -> Result<(), String> {
        if let Some(parent) = path.parent()
            && !parent.as_os_str().is_empty()
        {
            fs::create_dir_all(parent)
                .map_err(|error| format!("create {}: {error}", parent.display()))?;
        }
        let bytes = serde_json::to_vec_pretty(self)
            .map_err(|error| format!("serialize benchmark model: {error}"))?;
        fs::write(path, bytes).map_err(|error| format!("write {}: {error}", path.display()))
    }

    pub fn load_json(path: &Path) -> Result<Self, String> {
        let bytes = fs::read(path).map_err(|error| format!("read {}: {error}", path.display()))?;
        serde_json::from_slice(&bytes)
            .map_err(|error| format!("parse benchmark model {}: {error}", path.display()))
    }

    pub fn edge_count(&self) -> usize {
        self.graph_edges.len()
    }

    pub fn distinct_callbacks(&self) -> usize {
        self.callback_latency
            .keys()
            .chain(self.message_throughput.keys())
            .copied()
            .collect::<BTreeSet<_>>()
            .len()
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
struct CallbackAggregate {
    total_execution_latency: u64,
    execution_samples: u64,
    total_scheduling_latency: u64,
    scheduling_samples: u64,
}

impl CallbackAggregate {
    fn total_scheduling_latency(&self) -> Option<u64> {
        (self.scheduling_samples > 0).then_some(self.total_scheduling_latency)
    }
}

#[derive(Debug, Clone, Copy, PartialEq)]
struct MessageAggregate {
    mean_throughput: f64,
    samples: u64,
}

#[derive(Debug, Clone, PartialEq, Default)]
struct TraceAggregate {
    graph_edges: BTreeSet<(u64, u64)>,
    callback_latency: BTreeMap<u64, CallbackAggregate>,
    message_throughput: BTreeMap<u64, MessageAggregate>,
}

fn aggregate_trace(trace: &CallbackTrace) -> TraceAggregate {
    let mut aggregate = TraceAggregate::default();
    let mut previous = None;
    for latency in &trace.call_trace {
        if let Some(prev) = previous {
            aggregate.graph_edges.insert((prev, latency.callback_id));
        }
        previous = Some(latency.callback_id);

        let entry = aggregate
            .callback_latency
            .entry(latency.callback_id)
            .or_default();
        entry.total_execution_latency = entry
            .total_execution_latency
            .saturating_add(latency.execution_latency);
        entry.execution_samples = entry.execution_samples.saturating_add(1);
        if let Some(scheduling_latency) = latency.scheduling_latency {
            entry.total_scheduling_latency = entry
                .total_scheduling_latency
                .saturating_add(scheduling_latency);
            entry.scheduling_samples = entry.scheduling_samples.saturating_add(1);
        }
    }

    let mut throughput_sum: BTreeMap<u64, f64> = BTreeMap::new();
    let mut throughput_count: BTreeMap<u64, u64> = BTreeMap::new();
    for message in &trace.msg_trace {
        *throughput_sum.entry(message.callback_id).or_default() += message.throughput;
        *throughput_count.entry(message.callback_id).or_default() += 1;
    }
    for (callback_id, samples) in throughput_count {
        let sum = throughput_sum.remove(&callback_id).unwrap_or(0.0);
        aggregate.message_throughput.insert(
            callback_id,
            MessageAggregate {
                mean_throughput: sum / samples as f64,
                samples,
            },
        );
    }

    aggregate
}

/// Benchmark builder matching the paper's split workflow: first sample the
/// target system for a dedicated benchmark period, then freeze the benchmark
/// reference. Each analyzed trace is first aggregated per callback/message
/// before it contributes one sample to the benchmark model.
#[derive(Debug, Default, Clone, PartialEq)]
pub struct BenchmarkBuilder {
    graph_edges: BTreeSet<(u64, u64)>,
    latency_sum: BTreeMap<u64, u128>,
    latency_count: BTreeMap<u64, u64>,
    scheduling_sum: BTreeMap<u64, u128>,
    scheduling_count: BTreeMap<u64, u64>,
    throughput_sum: BTreeMap<u64, f64>,
    throughput_count: BTreeMap<u64, u64>,
    analyzed_traces: u64,
    empty_traces: u64,
    invalid_traces: u64,
}

impl BenchmarkBuilder {
    pub fn observe(&mut self, trace: &CallbackTrace) -> TraceDisposition {
        let empty = trace.call_trace.is_empty() && trace.msg_trace.is_empty();
        if empty {
            self.empty_traces += 1;
            return TraceDisposition::Empty;
        }
        if !trace.valid_for_state_analysis() {
            self.invalid_traces += 1;
            return TraceDisposition::Invalid;
        }

        let aggregate = aggregate_trace(trace);
        self.observe_aggregate(&aggregate);
        self.analyzed_traces += 1;
        TraceDisposition::Analyzed
    }

    pub fn record_invalid_round(&mut self) {
        self.invalid_traces += 1;
    }

    pub fn analyzed_traces(&self) -> u64 {
        self.analyzed_traces
    }

    pub fn empty_traces(&self) -> u64 {
        self.empty_traces
    }

    pub fn invalid_traces(&self) -> u64 {
        self.invalid_traces
    }

    pub fn build(&self) -> BenchmarkModel {
        let callback_latency = self
            .latency_count
            .iter()
            .map(|(&callback_id, &samples)| {
                let sum = *self.latency_sum.get(&callback_id).unwrap_or(&0) as f64;
                let scheduling_samples = *self.scheduling_count.get(&callback_id).unwrap_or(&0);
                let mean_scheduling_latency = (scheduling_samples > 0).then(|| {
                    *self.scheduling_sum.get(&callback_id).unwrap_or(&0) as f64
                        / scheduling_samples as f64
                });
                (
                    callback_id,
                    CallbackBenchmark {
                        mean_latency: sum / samples as f64,
                        samples,
                        mean_scheduling_latency,
                        scheduling_samples,
                    },
                )
            })
            .collect();
        let message_throughput = self
            .throughput_count
            .iter()
            .map(|(&callback_id, &samples)| {
                let sum = *self.throughput_sum.get(&callback_id).unwrap_or(&0.0);
                (
                    callback_id,
                    MessageBenchmark {
                        mean_throughput: sum / samples as f64,
                        samples,
                    },
                )
            })
            .collect();
        BenchmarkModel {
            graph_edges: self.graph_edges.clone(),
            callback_latency,
            message_throughput,
            analyzed_traces: self.analyzed_traces,
        }
    }

    fn observe_aggregate(&mut self, aggregate: &TraceAggregate) {
        self.graph_edges
            .extend(aggregate.graph_edges.iter().copied());
        for (&callback_id, latency) in &aggregate.callback_latency {
            *self.latency_sum.entry(callback_id).or_default() +=
                latency.total_execution_latency as u128;
            *self.latency_count.entry(callback_id).or_default() += 1;
            if let Some(total_scheduling_latency) = latency.total_scheduling_latency() {
                *self.scheduling_sum.entry(callback_id).or_default() +=
                    total_scheduling_latency as u128;
                *self.scheduling_count.entry(callback_id).or_default() += 1;
            }
        }
        for (&callback_id, message) in &aggregate.message_throughput {
            *self.throughput_sum.entry(callback_id).or_default() += message.mean_throughput;
            *self.throughput_count.entry(callback_id).or_default() += 1;
        }
    }
}

#[derive(Debug, Clone, PartialEq)]
struct GlobalState {
    graph_edges: BTreeSet<(u64, u64)>,
    callback_latency: BTreeMap<u64, CallbackBenchmark>,
    message_throughput: BTreeMap<u64, MessageBenchmark>,
}

impl GlobalState {
    fn from_benchmark(model: &BenchmarkModel) -> Self {
        Self {
            graph_edges: model.graph_edges.clone(),
            callback_latency: model.callback_latency.clone(),
            message_throughput: model.message_throughput.clone(),
        }
    }

    fn edge_count(&self) -> usize {
        self.graph_edges.len()
    }

    fn distinct_callbacks(&self) -> usize {
        self.callback_latency
            .keys()
            .chain(self.message_throughput.keys())
            .copied()
            .collect::<BTreeSet<_>>()
            .len()
    }

    fn update_from_aggregate(&mut self, aggregate: &TraceAggregate) {
        self.graph_edges
            .extend(aggregate.graph_edges.iter().copied());

        for (&callback_id, current) in &aggregate.callback_latency {
            let entry = self
                .callback_latency
                .entry(callback_id)
                .or_insert(CallbackBenchmark {
                    mean_latency: 0.0,
                    samples: 0,
                    mean_scheduling_latency: None,
                    scheduling_samples: 0,
                });
            update_mean(
                &mut entry.mean_latency,
                &mut entry.samples,
                current.total_execution_latency as f64,
            );
            if let Some(total_scheduling_latency) = current.total_scheduling_latency() {
                let mut mean = entry.mean_scheduling_latency.unwrap_or(0.0);
                let mut samples = entry.scheduling_samples;
                update_mean(&mut mean, &mut samples, total_scheduling_latency as f64);
                entry.mean_scheduling_latency = Some(mean);
                entry.scheduling_samples = samples;
            }
        }

        for (&callback_id, current) in &aggregate.message_throughput {
            let entry = self
                .message_throughput
                .entry(callback_id)
                .or_insert(MessageBenchmark {
                    mean_throughput: 0.0,
                    samples: 0,
                });
            update_mean(
                &mut entry.mean_throughput,
                &mut entry.samples,
                current.mean_throughput,
            );
        }
    }
}

fn update_mean(mean: &mut f64, samples: &mut u64, sample: f64) {
    let next_samples = samples.saturating_add(1);
    *mean = if *samples == 0 {
        sample
    } else {
        (*mean * *samples as f64 + sample) / next_samples as f64
    };
    *samples = next_samples;
}

/// Fuzzing-phase oracle: compare the current trace aggregate against an
/// immutable benchmark reference, while the discovered graph/latency state is
/// updated online as new states are found.
pub struct BenchmarkStateOracle {
    benchmark: BenchmarkModel,
    global_state: GlobalState,
    thresholds: DeviationThresholds,
    last_verdict: OracleVerdict,
}

impl BenchmarkStateOracle {
    pub fn new(model: BenchmarkModel, thresholds: DeviationThresholds) -> Self {
        Self {
            global_state: GlobalState::from_benchmark(&model),
            benchmark: model,
            thresholds,
            last_verdict: OracleVerdict {
                new_state: false,
                crashed: false,
                trace: TraceDisposition::Empty,
                evidence: StateEvidence::default(),
            },
        }
    }

    pub fn evaluate(&mut self, trace: &CallbackTrace, crashed: bool) -> OracleVerdict {
        let empty = trace.call_trace.is_empty() && trace.msg_trace.is_empty();
        let disposition = if empty {
            TraceDisposition::Empty
        } else if trace.valid_for_state_analysis() {
            TraceDisposition::Analyzed
        } else {
            TraceDisposition::Invalid
        };

        let evidence = if disposition == TraceDisposition::Analyzed {
            self.analyze(trace)
        } else {
            StateEvidence::default()
        };
        let new_state = evidence.callback_trace_new_state();
        self.last_verdict = OracleVerdict {
            new_state,
            crashed,
            trace: disposition,
            evidence,
        };
        self.last_verdict
    }

    pub fn benchmark(&self) -> &BenchmarkModel {
        &self.benchmark
    }

    pub fn edge_count(&self) -> usize {
        self.global_state.edge_count()
    }

    pub fn distinct_callbacks(&self) -> usize {
        self.global_state.distinct_callbacks()
    }

    fn analyze(&mut self, trace: &CallbackTrace) -> StateEvidence {
        let aggregate = aggregate_trace(trace);
        let evidence = self.detect_evidence(&aggregate);
        if evidence.callback_trace_new_state() {
            self.global_state.update_from_aggregate(&aggregate);
        }
        evidence
    }

    fn detect_evidence(&self, aggregate: &TraceAggregate) -> StateEvidence {
        let new_edge = aggregate
            .graph_edges
            .iter()
            .any(|edge| !self.global_state.graph_edges.contains(edge));
        let new_callback = aggregate
            .callback_latency
            .keys()
            .any(|callback_id| !self.global_state.callback_latency.contains_key(callback_id));
        let new_message = aggregate.message_throughput.keys().any(|callback_id| {
            !self
                .global_state
                .message_throughput
                .contains_key(callback_id)
        });
        let latency_deviation = aggregate
            .callback_latency
            .iter()
            .any(|(&callback_id, current)| self.callback_latency_deviation(callback_id, current));
        let throughput_deviation = aggregate
            .message_throughput
            .iter()
            .any(|(&callback_id, current)| self.message_throughput_deviation(callback_id, current));

        StateEvidence {
            new_edge,
            new_callback,
            new_message,
            latency_deviation,
            throughput_deviation,
        }
    }

    fn callback_latency_deviation(&self, callback_id: u64, current: &CallbackAggregate) -> bool {
        let Some(benchmark) = self.benchmark.callback_latency.get(&callback_id) else {
            return false;
        };
        if benchmark.mean_latency > 0.0
            && current.total_execution_latency as f64
                > benchmark.mean_latency * self.thresholds.latency_factor
        {
            return true;
        }
        if let (Some(mean_scheduling_latency), Some(total_scheduling_latency)) = (
            benchmark.mean_scheduling_latency,
            current.total_scheduling_latency(),
        ) && mean_scheduling_latency > 0.0
            && total_scheduling_latency as f64
                > mean_scheduling_latency * self.thresholds.latency_factor
        {
            return true;
        }
        false
    }

    fn message_throughput_deviation(&self, callback_id: u64, current: &MessageAggregate) -> bool {
        if let Some(benchmark) = self.benchmark.message_throughput.get(&callback_id)
            && current.mean_throughput
                < benchmark.mean_throughput * self.thresholds.throughput_floor
        {
            return true;
        }
        false
    }
}

impl StateOracle for BenchmarkStateOracle {
    fn is_new_state(&self) -> bool {
        self.last_verdict.new_state
    }

    fn crashed(&self) -> bool {
        self.last_verdict.crashed
    }
}

#[cfg(test)]
mod tests {
    use super::{
        BenchmarkBuilder, BenchmarkModel, BenchmarkStateOracle, CallbackBenchmark,
        DeviationThresholds, MessageBenchmark, TraceDisposition,
    };
    use crate::callback_profile::{CallbackLatency, CallbackTrace, MessageLatency};
    use crate::payload_generator::StateOracle;
    use std::collections::{BTreeMap, BTreeSet};

    fn trace(callbacks: &[(u64, u64, Option<u64>)], messages: &[(u64, f64)]) -> CallbackTrace {
        CallbackTrace {
            call_trace: callbacks
                .iter()
                .map(
                    |(callback_id, execution_latency, scheduling_latency)| CallbackLatency {
                        callback_id: *callback_id,
                        execution_latency: *execution_latency,
                        scheduling_latency: *scheduling_latency,
                    },
                )
                .collect(),
            msg_trace: messages
                .iter()
                .map(|(callback_id, throughput)| MessageLatency {
                    callback_id: *callback_id,
                    throughput: *throughput,
                })
                .collect(),
            ..CallbackTrace::default()
        }
    }

    fn uniform_trace(ids: &[u64], latency: u64, throughput: f64) -> CallbackTrace {
        let callbacks = ids
            .iter()
            .map(|id| (*id, latency, None))
            .collect::<Vec<_>>();
        let messages = ids
            .last()
            .map(|id| vec![(*id, throughput)])
            .unwrap_or_default();
        trace(&callbacks, &messages)
    }

    #[test]
    fn builder_collects_aggregated_benchmark_model_from_analyzed_traces() {
        let mut builder = BenchmarkBuilder::default();
        assert_eq!(
            builder.observe(&trace(
                &[(1, 10, Some(3)), (2, 5, Some(2)), (1, 7, Some(4))],
                &[(1, 8.0), (1, 4.0), (2, 10.0)],
            )),
            TraceDisposition::Analyzed
        );
        assert_eq!(
            builder.observe(&trace(&[(2, 15, Some(6))], &[(2, 6.0)])),
            TraceDisposition::Analyzed
        );
        let model = builder.build();

        assert_eq!(builder.analyzed_traces(), 2);
        assert_eq!(builder.empty_traces(), 0);
        assert_eq!(builder.invalid_traces(), 0);
        assert!(model.graph_edges.contains(&(1, 2)));
        assert!(model.graph_edges.contains(&(2, 1)));

        let callback1 = model.callback_latency.get(&1).unwrap();
        assert_eq!(callback1.mean_latency, 17.0);
        assert_eq!(callback1.mean_scheduling_latency, Some(7.0));

        let callback2 = model.callback_latency.get(&2).unwrap();
        assert_eq!(callback2.mean_latency, 10.0);
        assert_eq!(callback2.mean_scheduling_latency, Some(4.0));

        assert_eq!(
            model.message_throughput.get(&1).unwrap().mean_throughput,
            6.0
        );
        assert_eq!(
            model.message_throughput.get(&2).unwrap().mean_throughput,
            8.0
        );
    }

    #[test]
    fn oracle_reports_new_edges_and_threshold_deviations() {
        let mut graph_edges = BTreeSet::new();
        graph_edges.insert((1, 2));
        let mut callback_latency = BTreeMap::new();
        callback_latency.insert(
            1,
            CallbackBenchmark {
                mean_latency: 10.0,
                samples: 5,
                mean_scheduling_latency: None,
                scheduling_samples: 0,
            },
        );
        callback_latency.insert(
            2,
            CallbackBenchmark {
                mean_latency: 10.0,
                samples: 5,
                mean_scheduling_latency: None,
                scheduling_samples: 0,
            },
        );
        let mut message_throughput = BTreeMap::new();
        message_throughput.insert(
            2,
            MessageBenchmark {
                mean_throughput: 10.0,
                samples: 5,
            },
        );
        let model = BenchmarkModel {
            graph_edges,
            callback_latency,
            message_throughput,
            analyzed_traces: 5,
        };
        let thresholds = DeviationThresholds::new(2.0, 0.5);
        let mut oracle = BenchmarkStateOracle::new(model.clone(), thresholds);

        let verdict = oracle.evaluate(&uniform_trace(&[1, 2], 25, 4.0), false);
        assert!(verdict.new_state);
        assert!(verdict.evidence.latency_deviation);
        assert!(verdict.evidence.throughput_deviation);
        assert!(!verdict.evidence.new_edge);
        assert_eq!(verdict.trace, TraceDisposition::Analyzed);
        assert!(oracle.is_new_state());
        assert!(!oracle.crashed());

        let mut oracle = BenchmarkStateOracle::new(model, thresholds);
        let verdict = oracle.evaluate(&uniform_trace(&[2, 3], 10, 10.0), false);
        assert!(verdict.new_state);
        assert!(verdict.evidence.new_edge);
        assert_eq!(verdict.trace, TraceDisposition::Analyzed);
    }

    #[test]
    fn oracle_updates_mutable_global_state_after_new_callback() {
        let mut callback_latency = BTreeMap::new();
        callback_latency.insert(
            1,
            CallbackBenchmark {
                mean_latency: 10.0,
                samples: 5,
                mean_scheduling_latency: None,
                scheduling_samples: 0,
            },
        );
        let model = BenchmarkModel {
            graph_edges: BTreeSet::new(),
            callback_latency,
            message_throughput: BTreeMap::new(),
            analyzed_traces: 5,
        };
        let mut oracle = BenchmarkStateOracle::new(model, DeviationThresholds::new(2.0, 0.5));
        let discovered = uniform_trace(&[1, 3], 10, 10.0);

        let first = oracle.evaluate(&discovered, false);
        assert!(first.new_state);
        assert_eq!(oracle.edge_count(), 1);
        assert_eq!(oracle.distinct_callbacks(), 2);

        let second = oracle.evaluate(&discovered, false);
        assert!(!second.new_state);
        assert_eq!(oracle.edge_count(), 1);
        assert_eq!(oracle.distinct_callbacks(), 2);
    }

    #[test]
    fn scheduling_latency_deviation_counts_as_new_state() {
        let mut callback_latency = BTreeMap::new();
        callback_latency.insert(
            1,
            CallbackBenchmark {
                mean_latency: 10.0,
                samples: 5,
                mean_scheduling_latency: Some(5.0),
                scheduling_samples: 5,
            },
        );
        let model = BenchmarkModel {
            graph_edges: BTreeSet::new(),
            callback_latency,
            message_throughput: BTreeMap::new(),
            analyzed_traces: 5,
        };
        let mut oracle = BenchmarkStateOracle::new(model, DeviationThresholds::new(2.0, 0.5));

        let verdict = oracle.evaluate(&trace(&[(1, 10, Some(15))], &[]), false);
        assert!(verdict.new_state);
        assert!(verdict.evidence.latency_deviation);
        assert_eq!(verdict.trace, TraceDisposition::Analyzed);
    }

    #[test]
    fn invalid_traces_do_not_trigger_new_state() {
        let mut builder = BenchmarkBuilder::default();
        builder.observe(&uniform_trace(&[1, 2], 10, 10.0));
        let model = builder.build();
        let mut oracle = BenchmarkStateOracle::new(model, DeviationThresholds::new(2.0, 0.5));
        let mut invalid = uniform_trace(&[1, 2], 10, 10.0);
        invalid.lossy = true;

        let verdict = oracle.evaluate(&invalid, true);
        assert!(!verdict.new_state);
        assert_eq!(verdict.trace, TraceDisposition::Invalid);
        assert!(oracle.crashed());
    }

    #[test]
    fn threshold_only_candidates_count_in_unified_callback_trace_oracle() {
        let mut callback_latency = BTreeMap::new();
        callback_latency.insert(
            1,
            CallbackBenchmark {
                mean_latency: 10.0,
                samples: 5,
                mean_scheduling_latency: None,
                scheduling_samples: 0,
            },
        );
        let model = BenchmarkModel {
            graph_edges: BTreeSet::new(),
            callback_latency,
            message_throughput: BTreeMap::new(),
            analyzed_traces: 5,
        };
        let mut oracle = BenchmarkStateOracle::new(model, DeviationThresholds::new(2.0, 0.5));

        let verdict = oracle.evaluate(&trace(&[(1, 25, None)], &[]), false);
        assert!(verdict.new_state);
        assert!(verdict.evidence.latency_deviation);
        assert_eq!(verdict.trace, TraceDisposition::Analyzed);
    }

    #[test]
    fn unified_callback_trace_oracle_accepts_new_edges() {
        let mut graph_edges = BTreeSet::new();
        graph_edges.insert((1, 2));
        let model = BenchmarkModel {
            graph_edges,
            callback_latency: BTreeMap::new(),
            message_throughput: BTreeMap::new(),
            analyzed_traces: 5,
        };
        let mut oracle = BenchmarkStateOracle::new(model, DeviationThresholds::new(2.0, 0.5));

        let verdict = oracle.evaluate(&uniform_trace(&[2, 3], 10, 10.0), false);
        assert!(verdict.new_state);
        assert!(verdict.evidence.new_edge);
    }
}
