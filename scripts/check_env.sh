#!/usr/bin/env bash
# scripts/check_env.sh — 只读取并打印 YAML 中的运行配置。
#
# 所需配置全部从 config/r2d2_env.yaml 读取。
# 若某个键为空，Rust helper 会直接报 “<KEY> is empty in ...”。
set -euo pipefail

SCRIPT_DIR=$(cd "$(dirname "$0")" && pwd)
REPO_ROOT=$(cd "$SCRIPT_DIR/.." && pwd)

yaml_env() {
  local key="$1"
  local helper="$REPO_ROOT/target/debug/my_r2d2"
  if [ -x "$helper" ]; then
    "$helper" yaml-get "$key"
  elif command -v cargo >/dev/null 2>&1; then
    cargo run --quiet --manifest-path "$REPO_ROOT/Cargo.toml" -- yaml-get "$key"
  else
    echo "check_env: cargo is required to read YAML config" >&2
    exit 1
  fi
}

for key in \
  R2D2_NAV2_WS \
  R2D2_ROS_SETUP \
  R2D2_PYTHON_EXECUTABLE \
  R2D2_COSTMAP_PARAMS \
  R2D2_SHM_PATH \
  R2D2_FUZZ_SOURCE \
  ROS_DOMAIN_ID
do
  printf '%s=%s\n' "$key" "$(yaml_env "$key")"
done
