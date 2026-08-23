//! End-to-end tests for the payload generation loop over in-memory
//! stand-ins; no ROS 2 environment is required.

use my_r2d2::interface_extractor::{DataFile, Field, Interface, Kind, Primitive, TypeNode};
use my_r2d2::payload::{Error, Payload, Serializer, SimpleSerializer, Value, ValueTree};
use my_r2d2::payload_generator::{GeneratorConfig, PayloadGenerator, Sender, StateOracle};
use my_r2d2::payload_pool::{PayloadPool, SelectionPolicy};
use rand::{SeedableRng, rngs::StdRng};

/// Records every payload handed to the transport.
#[derive(Default)]
struct MockSender {
    sent: std::cell::RefCell<Vec<Payload>>,
}

impl Sender for MockSender {
    fn send(&self, payload: &Payload) -> Result<(), Error> {
        self.sent.borrow_mut().push(payload.clone());
        Ok(())
    }
}

/// Preset verdicts standing in for the feedback controller.
struct MockOracle {
    new_state: bool,
    crashed: bool,
}

impl StateOracle for MockOracle {
    fn is_new_state(&self) -> bool {
        self.new_state
    }

    fn crashed(&self) -> bool {
        self.crashed
    }
}

/// A `geometry_msgs/Twist`-like topic with two nested Vector3 messages.
fn twist_interface() -> Interface {
    let vector3 = |prefix: &str| {
        TypeNode::nested(vec![
            Field::new(format!("{prefix}_x"), Primitive::F64),
            Field::new(format!("{prefix}_y"), Primitive::F64),
            Field::new(format!("{prefix}_z"), Primitive::F64),
        ])
    };
    Interface::new(
        "/cmd_vel",
        Kind::Topic,
        vec![
            Field::new("linear", vector3("linear")),
            Field::new("angular", vector3("angular")),
        ],
    )
    .with_data_files(vec![DataFile::new(
        "geometry_msgs/msg/Twist.msg",
        "linear: Vector3\nangular: Vector3",
    )])
}

fn top_level(interface: &Interface) -> TypeNode {
    TypeNode::nested(interface.fields.clone())
}

#[test]
fn empty_pool_generates_type_conformant_payload() {
    let interface = twist_interface();
    let mut generator =
        PayloadGenerator::new(vec![interface.clone()], GeneratorConfig::default(), 7);
    let payload = generator.next_payload().unwrap();

    assert_eq!(payload.interface_id, "/cmd_vel");
    assert_eq!(payload.kind, Kind::Topic);
    match &payload.value {
        ValueTree::Nested(outer) => {
            assert_eq!(outer.len(), 2);
            for message in outer {
                match message {
                    ValueTree::Nested(inner) => {
                        assert_eq!(inner.len(), 3);
                        for field in inner {
                            assert!(matches!(field, ValueTree::Leaf(Value::F64(_))));
                        }
                    }
                    other => panic!("expected nested message, got {other:?}"),
                }
            }
        }
        other => panic!("expected nested root, got {other:?}"),
    }

    assert!(!payload.serialized.is_empty());
    let restored = SimpleSerializer
        .deserialize(&payload.serialized, &top_level(&interface))
        .unwrap();
    assert_eq!(restored, payload.value);
}

#[test]
fn same_seed_reproduces_identical_payload() {
    let interface = twist_interface();
    let config = GeneratorConfig::default();
    let mut first = PayloadGenerator::new(vec![interface.clone()], config.clone(), 42);
    let mut second = PayloadGenerator::new(vec![interface], config, 42);

    let a = first.next_payload().unwrap();
    let b = second.next_payload().unwrap();
    assert_eq!(a.serialized, b.serialized);
    assert_eq!(a.value, b.value);
}

#[test]
fn pool_payload_is_mutated_in_place_of_generation() {
    let interface = twist_interface();
    let mut generator = PayloadGenerator::new(vec![interface], GeneratorConfig::default(), 5);
    let original = generator.next_payload().unwrap();
    generator.pool_mut().push(original.clone());

    let mutated = generator.next_payload().unwrap();
    assert_eq!(mutated.interface_id, original.interface_id);
    assert_eq!(mutated.kind, original.kind);
    assert_ne!(mutated.value, original.value);
}

#[test]
fn only_new_state_or_crash_payloads_enter_pool() {
    let interface = twist_interface();
    let mut generator = PayloadGenerator::new(vec![interface], GeneratorConfig::default(), 9);
    let sender = MockSender::default();

    let oracle = MockOracle {
        new_state: false,
        crashed: false,
    };
    let payload = generator.next_payload().unwrap();
    sender.send(&payload).unwrap();
    generator.retain_if_interesting(payload, &oracle);
    assert!(generator.pool().is_empty());

    let oracle = MockOracle {
        new_state: true,
        crashed: false,
    };
    let payload = generator.next_payload().unwrap();
    sender.send(&payload).unwrap();
    generator.retain_if_interesting(payload, &oracle);
    assert_eq!(generator.pool().len(), 1);

    let oracle = MockOracle {
        new_state: false,
        crashed: true,
    };
    let payload = generator.next_payload().unwrap();
    sender.send(&payload).unwrap();
    generator.retain_if_interesting(payload, &oracle);
    assert_eq!(generator.pool().len(), 2);
    assert_eq!(sender.sent.borrow().len(), 3);
}

#[test]
fn arrays_are_generated_within_configured_ranges() {
    let interface = Interface::new(
        "/scan",
        Kind::Topic,
        vec![
            Field::new(
                "ranges",
                TypeNode::array(TypeNode::Primitive(Primitive::F64)),
            ),
            Field::new(
                "covariance",
                TypeNode::fixed_array(TypeNode::Primitive(Primitive::F32), 4),
            ),
        ],
    );
    let config = GeneratorConfig {
        array_len_range: 2..=5,
        ..GeneratorConfig::default()
    };
    let mut generator = PayloadGenerator::new(vec![interface.clone()], config, 11);
    let payload = generator.next_payload().unwrap();

    match &payload.value {
        ValueTree::Nested(fields) => {
            match &fields[0] {
                ValueTree::Array(ranges) => {
                    assert!((2..=5).contains(&ranges.len()));
                    assert!(
                        ranges
                            .iter()
                            .all(|r| matches!(r, ValueTree::Leaf(Value::F64(_))))
                    );
                }
                other => panic!("expected variable array, got {other:?}"),
            }
            match &fields[1] {
                ValueTree::Array(covariance) => assert_eq!(covariance.len(), 4),
                other => panic!("expected fixed array, got {other:?}"),
            }
        }
        other => panic!("expected nested root, got {other:?}"),
    }

    let restored = SimpleSerializer
        .deserialize(&payload.serialized, &top_level(&interface))
        .unwrap();
    assert_eq!(restored, payload.value);
}

#[test]
fn string_and_bytes_round_trip() {
    let interface = Interface::new(
        "/chatter",
        Kind::Topic,
        vec![
            Field::new("frame_id", Primitive::String),
            Field::new("blob", Primitive::Bytes),
        ],
    );
    let mut generator =
        PayloadGenerator::new(vec![interface.clone()], GeneratorConfig::default(), 13);
    let payload = generator.next_payload().unwrap();

    let restored = SimpleSerializer
        .deserialize(&payload.serialized, &top_level(&interface))
        .unwrap();
    assert_eq!(restored, payload.value);
}

#[test]
fn pool_pick_returns_a_stored_payload_without_removing_it() {
    let mut pool = PayloadPool::new();
    assert!(pool.is_empty());
    pool.push(Payload::new(
        "/a",
        Kind::Topic,
        ValueTree::Leaf(Value::Bool(true)),
        1,
    ));
    pool.push(Payload::new(
        "/b",
        Kind::Topic,
        ValueTree::Leaf(Value::Bool(false)),
        2,
    ));
    assert_eq!(pool.len(), 2);

    let mut rng = StdRng::seed_from_u64(3);
    let picked = pool
        .pick_for_mutation(&mut rng, SelectionPolicy::Uniform)
        .unwrap();
    assert!(picked.interface_id == "/a" || picked.interface_id == "/b");
    assert_eq!(pool.len(), 2);
}

#[test]
fn empty_interface_list_is_rejected() {
    let mut generator = PayloadGenerator::new(vec![], GeneratorConfig::default(), 1);
    let error = generator.next_payload().unwrap_err();
    assert!(matches!(error, Error::Unsupported(_)));
}
