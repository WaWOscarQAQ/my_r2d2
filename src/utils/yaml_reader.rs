use serde::Deserialize;
use serde_yaml::Value;
use std::collections::HashMap;
use std::fs;
use std::path::{Path, PathBuf};

#[derive(Debug, Default, Deserialize)]
struct EnvYamlFile {
    #[serde(default)]
    env: HashMap<String, Value>,
}

#[derive(Debug, Clone)]
pub struct YamlEnv {
    config_path: PathBuf,
    values: HashMap<String, String>,
}

impl YamlEnv {
    pub fn load(repo_root: &Path) -> Result<Self, String> {
        let config_path = default_config_path(repo_root);

        let content = fs::read_to_string(&config_path)
            .map_err(|e| format!("failed to read {}: {e}", config_path.display()))?;

        let parsed: EnvYamlFile = serde_yaml::from_str(&content)
            .map_err(|e| format!("failed to parse {}: {e}", config_path.display()))?;

        let base_dir = config_path
            .parent()
            .map(Path::to_path_buf)
            .unwrap_or_else(|| repo_root.to_path_buf());

        let values = parsed
            .env
            .into_iter()
            .filter_map(|(key, value)| {
                yaml_scalar_to_string(value)
                    .map(|value| (key.clone(), resolve_yaml_path(&key, value, &base_dir)))
            })
            .collect();

        Ok(Self {
            config_path,
            values,
        })
    }

    pub fn require_string(&self, key: &str) -> Result<String, String> {
        match self.values.get(key) {
            Some(value) if !value.trim().is_empty() => Ok(value.clone()),
            _ => Err(format!("{key} is empty in {}", self.config_path.display())),
        }
    }

    pub fn require_path(&self, key: &str) -> Result<PathBuf, String> {
        self.require_string(key).map(PathBuf::from)
    }

    pub fn config_path(&self) -> &Path {
        &self.config_path
    }
}

pub fn default_config_path(repo_root: &Path) -> PathBuf {
    repo_root.join("config/r2d2_env.yaml")
}

fn yaml_scalar_to_string(value: Value) -> Option<String> {
    match value {
        Value::Null => None,
        Value::Bool(v) => Some(v.to_string()),
        Value::Number(v) => Some(v.to_string()),
        Value::String(v) => Some(v),
        _ => None,
    }
}

fn is_path_key(key: &str) -> bool {
    matches!(
        key,
        "R2D2_NAV2_WS"
            | "R2D2_ROS_SETUP"
            | "R2D2_PYTHON_EXECUTABLE"
            | "R2D2_COSTMAP_PARAMS"
            | "R2D2_SHM_PATH"
            | "R2D2_FUZZ_SOURCE"
    )
}

fn resolve_yaml_path(key: &str, value: String, base_dir: &Path) -> String {
    if !is_path_key(key) || value.trim().is_empty() {
        return value;
    }
    let path = PathBuf::from(&value);
    if path.is_absolute() {
        value
    } else {
        base_dir.join(path).display().to_string()
    }
}

#[cfg(test)]
mod tests {
    use super::YamlEnv;
    use std::fs;
    use std::path::PathBuf;

    fn test_repo(name: &str) -> PathBuf {
        std::env::temp_dir().join(format!("my_r2d2_yaml_{name}_{}", std::process::id()))
    }

    #[test]
    fn reads_values_and_resolves_relative_paths() {
        let repo = test_repo("values");
        let config_dir = repo.join("config");
        fs::create_dir_all(&config_dir).unwrap();
        fs::write(
            config_dir.join("r2d2_env.yaml"),
            "env:\n  R2D2_NAV2_WS: ../nav2_ws\n  ROS_DOMAIN_ID: 190\n",
        )
        .unwrap();

        let env = YamlEnv::load(&repo).unwrap();
        assert_eq!(
            env.require_path("R2D2_NAV2_WS").unwrap(),
            config_dir.join("../nav2_ws")
        );
        assert_eq!(env.require_string("ROS_DOMAIN_ID").unwrap(), "190");

        fs::remove_dir_all(repo).unwrap();
    }

    #[test]
    fn reports_blank_or_missing_values_as_empty() {
        let repo = test_repo("empty");
        let config_dir = repo.join("config");
        fs::create_dir_all(&config_dir).unwrap();
        fs::write(
            config_dir.join("r2d2_env.yaml"),
            "env:\n  R2D2_SHM_PATH: '   '\n",
        )
        .unwrap();

        let env = YamlEnv::load(&repo).unwrap();
        assert!(
            env.require_string("R2D2_SHM_PATH")
                .unwrap_err()
                .contains("R2D2_SHM_PATH is empty")
        );
        assert!(
            env.require_string("ROS_DOMAIN_ID")
                .unwrap_err()
                .contains("ROS_DOMAIN_ID is empty")
        );

        fs::remove_dir_all(repo).unwrap();
    }
}
