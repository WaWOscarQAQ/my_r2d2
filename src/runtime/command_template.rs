use serde::Deserialize;
use std::collections::BTreeMap;
use std::fs;
use std::path::{Path, PathBuf};
use std::process::Command;

#[derive(Debug, Clone, Default)]
pub struct CommandParameters(BTreeMap<String, Vec<String>>);

impl CommandParameters {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn insert(&mut self, name: impl Into<String>, value: impl ToString) -> &mut Self {
        self.0.insert(name.into(), vec![value.to_string()]);
        self
    }

    pub fn insert_path(&mut self, name: impl Into<String>, value: &Path) -> &mut Self {
        self.insert(name, value.display())
    }

    pub fn insert_many<I, S>(&mut self, name: impl Into<String>, values: I) -> &mut Self
    where
        I: IntoIterator<Item = S>,
        S: ToString,
    {
        self.0.insert(
            name.into(),
            values.into_iter().map(|value| value.to_string()).collect(),
        );
        self
    }
}

#[derive(Debug, Clone, Deserialize)]
pub struct CommandTemplate {
    executable: String,
    #[serde(default)]
    args: Vec<String>,
}

impl CommandTemplate {
    pub fn validate(&self, name: &str) -> Result<(), String> {
        if self.executable.trim().is_empty() {
            Err(format!("command {name} has an empty executable"))
        } else {
            Ok(())
        }
    }

    pub fn build(&self, name: &str, values: &CommandParameters) -> Result<Command, String> {
        self.validate(name)?;
        let mut command = Command::new(&self.executable);
        for token in &self.args {
            command.args(expand(name, token, &values.0)?);
        }
        Ok(command)
    }
}

#[derive(Debug, Deserialize)]
pub struct RuntimeCommands {
    version: u32,
    commands: BTreeMap<String, CommandTemplate>,
}

impl RuntimeCommands {
    pub fn load_default() -> Result<Self, String> {
        let path = std::env::var_os("R2D2_RUNTIME_COMMANDS_CONFIG")
            .filter(|path| !path.is_empty())
            .map(PathBuf::from)
            .unwrap_or_else(|| {
                Path::new(env!("CARGO_MANIFEST_DIR")).join("config/runtime_commands.yaml")
            });
        let text = fs::read_to_string(&path)
            .map_err(|error| format!("read {}: {error}", path.display()))?;
        let config: Self = serde_yaml::from_str(&text)
            .map_err(|error| format!("parse {}: {error}", path.display()))?;
        if config.version != 1 {
            return Err(format!(
                "unsupported command file version {}",
                config.version
            ));
        }
        Ok(config)
    }

    pub fn build(&self, name: &str, values: &CommandParameters) -> Result<Command, String> {
        self.commands
            .get(name)
            .ok_or_else(|| format!("runtime command {name:?} is not defined"))?
            .build(name, values)
    }
}

pub fn runtime_command(name: &str, values: &CommandParameters) -> Result<Command, String> {
    RuntimeCommands::load_default()?.build(name, values)
}

fn expand(
    command: &str,
    token: &str,
    values: &BTreeMap<String, Vec<String>>,
) -> Result<Vec<String>, String> {
    if let Some(name) = token
        .strip_prefix("{{")
        .and_then(|value| value.strip_suffix("...}}"))
    {
        return values
            .get(name)
            .cloned()
            .ok_or_else(|| format!("command {command} needs parameter {name}"));
    }

    let mut result = token.to_string();
    while let Some(start) = result.find("{{") {
        let end = result[start + 2..]
            .find("}}")
            .map(|offset| start + 2 + offset)
            .ok_or_else(|| format!("command {command} has an invalid placeholder"))?;
        let name = result[start + 2..end].to_string();
        let value = values
            .get(&name)
            .filter(|items| items.len() == 1)
            .ok_or_else(|| format!("command {command} needs one value for {name}"))?;
        result.replace_range(start..end + 2, &value[0]);
    }
    Ok(vec![result])
}

#[cfg(test)]
mod tests {
    use super::{CommandParameters, RuntimeCommands};

    #[test]
    fn loads_file_and_keeps_values_as_separate_arguments() {
        let commands = RuntimeCommands::load_default().unwrap();
        let mut values = CommandParameters::new();
        values.insert("stack_script", "/tmp/nav ws/launch.sh");
        let command = commands.build("nav2_stack", &values).unwrap();
        assert_eq!(command.get_program(), "setsid");
        assert_eq!(
            command
                .get_args()
                .map(|arg| arg.to_string_lossy().into_owned())
                .collect::<Vec<_>>(),
            ["bash", "/tmp/nav ws/launch.sh"]
        );
    }
}
