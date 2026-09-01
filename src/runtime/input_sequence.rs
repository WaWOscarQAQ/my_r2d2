use serde_yaml::{Mapping, Value as YamlValue};
use std::fs;
use std::path::{Path, PathBuf};

const INPUT_SEQUENCE_VERSION: u64 = 1;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum InputStepPhase {
    Startup,
    RoundPrefix,
    Both,
}

impl InputStepPhase {
    pub fn parse(value: &str) -> Result<Self, String> {
        match normalize_token(value).as_str() {
            "startup" | "once" | "warmup" | "warm_up" => Ok(Self::Startup),
            "round" | "roundprefix" | "round_prefix" | "perround" | "per_round" => {
                Ok(Self::RoundPrefix)
            }
            "both" | "all" => Ok(Self::Both),
            other => Err(format!("unknown input step phase {other:?}")),
        }
    }

    pub fn runs_in(self, phase: InputSequencePhase) -> bool {
        matches!(
            (self, phase),
            (Self::Both, _)
                | (Self::Startup, InputSequencePhase::Startup)
                | (Self::RoundPrefix, InputSequencePhase::RoundPrefix)
        )
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum InputSequencePhase {
    Startup,
    RoundPrefix,
}

impl InputSequencePhase {
    pub fn as_str(self) -> &'static str {
        match self {
            Self::Startup => "startup",
            Self::RoundPrefix => "round-prefix",
        }
    }
}

#[derive(Debug, Clone)]
pub struct InputSequence {
    pub path: PathBuf,
    pub name: String,
    pub steps: Vec<InputStep>,
}

impl InputSequence {
    pub fn load(path: impl AsRef<Path>) -> Result<Self, String> {
        let path = path.as_ref();
        let text = fs::read_to_string(path)
            .map_err(|error| format!("read {}: {error}", path.display()))?;
        Self::parse(path, &text)
    }

    pub fn parse(path: impl AsRef<Path>, text: &str) -> Result<Self, String> {
        let path = path.as_ref().to_path_buf();
        let yaml: YamlValue = serde_yaml::from_str(text)
            .map_err(|error| format!("parse {}: {error}", path.display()))?;
        let root = yaml_mapping(&yaml, "input sequence root")?;
        let version = optional_u64(root, "version")?.unwrap_or(INPUT_SEQUENCE_VERSION);
        if version != INPUT_SEQUENCE_VERSION {
            return Err(format!(
                "unsupported input sequence version {version} in {}",
                path.display()
            ));
        }
        let name = optional_string(root, "name")?.unwrap_or_else(|| {
            path.file_stem()
                .and_then(|value| value.to_str())
                .unwrap_or("input_sequence")
                .to_string()
        });
        let delayer = parse_delayer(root)?;
        let default_delay_after_ms = root
            .get(yaml_key("defaults"))
            .and_then(|value| value.as_mapping())
            .map(|defaults| optional_u64(defaults, "delay_after_ms"))
            .transpose()?
            .flatten()
            .unwrap_or(0);
        let steps_yaml = root
            .get(yaml_key("steps"))
            .or_else(|| root.get(yaml_key("user_input")))
            .ok_or_else(|| {
                format!(
                    "{} must contain either steps: or user_input:",
                    path.display()
                )
            })?;
        let steps = steps_yaml
            .as_sequence()
            .ok_or_else(|| format!("steps/user_input in {} must be a YAML list", path.display()))?
            .iter()
            .enumerate()
            .map(|(index, value)| {
                InputStep::parse(
                    index,
                    value,
                    delayer.get(index).copied(),
                    default_delay_after_ms,
                )
            })
            .collect::<Result<Vec<_>, _>>()?;
        if steps.is_empty() {
            return Err(format!("{} contains no input steps", path.display()));
        }
        Ok(Self { path, name, steps })
    }

    pub fn steps_for_phase(&self, phase: InputSequencePhase) -> impl Iterator<Item = &InputStep> {
        self.steps
            .iter()
            .filter(move |step| step.phase.runs_in(phase))
    }
}

#[derive(Debug, Clone)]
pub struct InputStep {
    pub label: Option<String>,
    pub phase: InputStepPhase,
    pub name: String,
    pub ros_type: String,
    pub interface_id: Option<String>,
    pub payload: YamlValue,
    pub delay_after_ms: u64,
    pub allow_failure: bool,
}

impl InputStep {
    fn parse(
        index: usize,
        value: &YamlValue,
        delayer_ms: Option<u64>,
        default_delay_after_ms: u64,
    ) -> Result<Self, String> {
        let step = yaml_mapping(value, &format!("input step {index}"))?;
        let type_map = step.get(yaml_key("type")).and_then(YamlValue::as_mapping);

        let kind_text = optional_string(step, "kind")?
            .or(optional_nested_string(type_map, "A-S-T")?)
            .ok_or_else(|| format!("input step {index} needs kind or type.A-S-T"))?;
        parse_readiness_kind(&kind_text).map_err(|error| format!("input step {index}: {error}"))?;
        let phase = optional_string(step, "phase")?
            .map(|value| InputStepPhase::parse(&value))
            .transpose()
            .map_err(|error| format!("input step {index}: {error}"))?
            .unwrap_or(InputStepPhase::Startup);
        let label = optional_string(step, "label")?;

        let name = first_string(
            step,
            &["endpoint", "name", "topic", "service", "action", "node"],
        )?
        .or(optional_nested_string(type_map, "name")?)
        .ok_or_else(|| format!("input step {index} needs endpoint/name"))?;

        let ros_type = first_string(
            step,
            &[
                "ros_type",
                "message_type",
                "service_type",
                "action_type",
                "parameter",
                "param",
                "transition",
                "operation",
            ],
        )?
        .or(optional_nested_string(type_map, "type")?)
        .or_else(|| {
            if step.get(yaml_key("kind")).is_some() {
                step.get(yaml_key("type"))
                    .and_then(YamlValue::as_str)
                    .map(ToString::to_string)
            } else {
                None
            }
        })
        .ok_or_else(|| format!("input step {index} needs ros_type/type"))?;

        let interface_id =
            optional_string(step, "interface")?.or_else(|| infer_interface_id(&ros_type));

        let payload = step
            .get(yaml_key("payload"))
            .or_else(|| step.get(yaml_key("data")))
            .cloned()
            .unwrap_or(YamlValue::Null);
        let allow_failure = optional_bool(step, "allow_failure")?.unwrap_or(false);
        let delay_after_ms = optional_u64(step, "delay_after_ms")?
            .or(optional_u64(step, "delay_ms")?)
            .or(delayer_ms)
            .unwrap_or(default_delay_after_ms);

        Ok(Self {
            label,
            phase,
            name,
            ros_type,
            interface_id,
            payload,
            delay_after_ms,
            allow_failure,
        })
    }

    pub fn label_or_name(&self) -> &str {
        self.label.as_deref().unwrap_or(&self.name)
    }
}

fn yaml_mapping<'a>(value: &'a YamlValue, label: &str) -> Result<&'a Mapping, String> {
    value
        .as_mapping()
        .ok_or_else(|| format!("{label} must be a YAML mapping"))
}

fn yaml_key(key: &str) -> YamlValue {
    YamlValue::String(key.to_string())
}

fn optional_string(map: &Mapping, key: &str) -> Result<Option<String>, String> {
    match map.get(yaml_key(key)) {
        Some(YamlValue::String(value)) => Ok(Some(value.clone())),
        Some(YamlValue::Number(value)) => Ok(Some(value.to_string())),
        Some(YamlValue::Bool(value)) => Ok(Some(value.to_string())),
        Some(YamlValue::Null) | None => Ok(None),
        Some(_) => Err(format!("{key} must be a scalar string")),
    }
}

fn optional_nested_string(map: Option<&Mapping>, key: &str) -> Result<Option<String>, String> {
    map.map(|map| optional_string(map, key)).unwrap_or(Ok(None))
}

fn first_string(map: &Mapping, keys: &[&str]) -> Result<Option<String>, String> {
    for key in keys {
        if let Some(value) = optional_string(map, key)? {
            return Ok(Some(value));
        }
    }
    Ok(None)
}

fn optional_u64(map: &Mapping, key: &str) -> Result<Option<u64>, String> {
    match map.get(yaml_key(key)) {
        Some(YamlValue::Number(value)) => value
            .as_u64()
            .ok_or_else(|| format!("{key} must be a non-negative integer"))
            .map(Some),
        Some(YamlValue::String(value)) if value.trim().is_empty() => Ok(None),
        Some(YamlValue::String(value)) => value
            .parse::<u64>()
            .map(Some)
            .map_err(|_| format!("{key} must be a non-negative integer")),
        Some(YamlValue::Null) | None => Ok(None),
        Some(_) => Err(format!("{key} must be a non-negative integer")),
    }
}

fn optional_bool(map: &Mapping, key: &str) -> Result<Option<bool>, String> {
    match map.get(yaml_key(key)) {
        Some(YamlValue::Bool(value)) => Ok(Some(*value)),
        Some(YamlValue::String(value)) => match normalize_token(value).as_str() {
            "true" | "yes" | "1" => Ok(Some(true)),
            "false" | "no" | "0" => Ok(Some(false)),
            _ => Err(format!("{key} must be a boolean")),
        },
        Some(YamlValue::Null) | None => Ok(None),
        Some(_) => Err(format!("{key} must be a boolean")),
    }
}

fn parse_delayer(root: &Mapping) -> Result<Vec<u64>, String> {
    let Some(value) = root.get(yaml_key("delayer")) else {
        return Ok(Vec::new());
    };
    let sequence = value
        .as_sequence()
        .ok_or_else(|| "delayer must be a YAML list".to_string())?;
    sequence
        .iter()
        .enumerate()
        .map(|(index, value)| match value {
            YamlValue::Number(number) => number
                .as_u64()
                .ok_or_else(|| format!("delayer[{index}] must be a non-negative integer")),
            YamlValue::String(text) => text
                .parse::<u64>()
                .map_err(|_| format!("delayer[{index}] must be a non-negative integer")),
            _ => Err(format!("delayer[{index}] must be a non-negative integer")),
        })
        .collect()
}

fn infer_interface_id(ros_type: &str) -> Option<String> {
    let last = if ros_type.contains('/') {
        ros_type.rsplit('/').next()
    } else {
        ros_type.rsplit("::").next()
    };
    last.filter(|value| !value.trim().is_empty())
        .map(ToString::to_string)
}

fn parse_readiness_kind(value: &str) -> Result<(), String> {
    match normalize_token(value).as_str() {
        "topic" => Ok(()),
        other => Err(format!(
            "readiness input step kind must be topic, got {other:?}"
        )),
    }
}

fn normalize_token(value: &str) -> String {
    value
        .trim()
        .to_ascii_lowercase()
        .chars()
        .filter(|ch| !matches!(ch, '-' | '_' | ' '))
        .collect()
}

#[cfg(test)]
mod tests {
    use super::{InputSequence, InputSequencePhase, InputStepPhase};

    #[test]
    fn parses_rocf_style_user_input() {
        let sequence = InputSequence::parse(
            "/tmp/rocf_seed.yaml",
            r#"
version: 1
name: rocf_style
delayer: [100, 200]
user_input:
  - data:
      header:
        frame_id: map
      pose:
        pose:
          position: {x: 0.6, y: -1.2, z: 0.0}
    type:
      A-S-T: topic
      name: /initialpose
      type: geometry_msgs/msg/PoseWithCovarianceStamped
  - phase: round
    data:
      header:
        frame_id: laser
      angle_min: -1.57
      angle_max: 1.57
      angle_increment: 0.01
      range_min: 0.05
      range_max: 10.0
      ranges: [1.0, 1.0, 1.0]
    type:
      A-S-T: topic
      name: /scan
      type: sensor_msgs/msg/LaserScan
"#,
        )
        .unwrap();

        assert_eq!(sequence.name, "rocf_style");
        assert_eq!(sequence.steps.len(), 2);
        assert_eq!(sequence.steps[0].name, "/initialpose");
        assert_eq!(
            sequence.steps[0].ros_type,
            "geometry_msgs/msg/PoseWithCovarianceStamped"
        );
        assert_eq!(
            sequence.steps[0].interface_id.as_deref(),
            Some("PoseWithCovarianceStamped")
        );
        assert_eq!(sequence.steps[0].delay_after_ms, 100);
        assert_eq!(sequence.steps[1].phase, InputStepPhase::RoundPrefix);
        assert_eq!(sequence.steps[1].delay_after_ms, 200);
        assert_eq!(
            sequence
                .steps_for_phase(InputSequencePhase::RoundPrefix)
                .count(),
            1
        );
    }

    #[test]
    fn parses_clear_r2d2_style_steps() {
        let sequence = InputSequence::parse(
            "/tmp/r2d2_sequence.yaml",
            r#"
version: 1
steps:
  - label: set_pose
    phase: both
    kind: topic
    endpoint: /initialpose
    ros_type: geometry_msgs/msg/PoseWithCovarianceStamped
    interface: PoseWithCovarianceStamped
    delay_after_ms: 50
    payload:
      header: {frame_id: map}
"#,
        )
        .unwrap();

        assert_eq!(sequence.name, "r2d2_sequence");
        assert_eq!(sequence.steps[0].label_or_name(), "set_pose");
        assert_eq!(sequence.steps[0].phase, InputStepPhase::Both);
        assert_eq!(
            sequence
                .steps_for_phase(InputSequencePhase::Startup)
                .count(),
            1
        );
        assert_eq!(
            sequence
                .steps_for_phase(InputSequencePhase::RoundPrefix)
                .count(),
            1
        );
    }

    #[test]
    fn rejects_route_scripting_steps() {
        let error = InputSequence::parse(
            "/tmp/route_scripting.yaml",
            r#"
version: 1
steps:
  - phase: round
    kind: action
    endpoint: /navigate_to_pose
    ros_type: nav2_msgs/action/NavigateToPose
    payload: {}
"#,
        )
        .unwrap_err();

        assert!(error.contains("readiness input step kind must be topic"));
    }

    #[test]
    fn loads_default_full_stack_bootstrap_sequence() {
        let repo_root = std::path::Path::new(env!("CARGO_MANIFEST_DIR"));
        let full =
            InputSequence::load(repo_root.join("config/nav2_sequences/full_stack_bootstrap.yaml"))
                .unwrap();

        assert_eq!(full.name, "full_stack_bootstrap");
        assert_eq!(full.steps.len(), 3);
        assert!(full.steps.iter().all(|step| step.allow_failure));
        assert_eq!(full.steps_for_phase(InputSequencePhase::Startup).count(), 2);
        assert_eq!(
            full.steps_for_phase(InputSequencePhase::RoundPrefix)
                .count(),
            3
        );
    }
}
