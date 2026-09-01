#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR=$(cd "$(dirname "$0")" && pwd)
REPO_ROOT=$(cd "$SCRIPT_DIR/.." && pwd)
SOURCE_WS="$REPO_ROOT/overlay_ws/src"
GENERATED_WS="$REPO_ROOT/llvm_overlay_ws"
GENERATED_SRC="$GENERATED_WS/src"
MARKER="$GENERATED_WS/.r2d2_generated_clean_overlay"

case "$GENERATED_WS" in
  "$REPO_ROOT"/llvm_overlay_ws) ;;
  *) echo "refusing unsafe generated workspace path: $GENERATED_WS" >&2; exit 1 ;;
esac

if [ -e "$GENERATED_WS" ] && [ ! -f "$MARKER" ]; then
  echo "$GENERATED_WS exists but is not marked as generated; refusing to overwrite it" >&2
  exit 1
fi

if [ -f "$MARKER" ]; then
  rm -rf "$GENERATED_SRC"
fi
mkdir -p "$GENERATED_SRC"
touch "$MARKER"

for repository in rclcpp rcl ros2_tracing; do
  source_repo="$SOURCE_WS/$repository"
  destination="$GENERATED_SRC/$repository"
  test -d "$source_repo/.git"
  mkdir -p "$destination"
  git -C "$source_repo" archive HEAD | tar -x -C "$destination"
done

# git archive preserves source mtimes from the committed tree.  If the LLVM
# pass/runtime changes but upstream ROS sources do not, CMake can otherwise
# keep old objects that were compiled without the newest instrumentation hooks.
find "$GENERATED_SRC" -type f \
  \( -name '*.c' -o -name '*.cc' -o -name '*.cpp' -o -name '*.h' -o -name '*.hpp' \) \
  -exec touch {} +

if command -v rg >/dev/null 2>&1; then
  hook_matches=$(rg -n "tracetools_r2d2|__r2d2_llvm_trace" "$GENERATED_SRC" \
    --glob '*.{c,cc,cpp,h,hpp}' || true)
else
  hook_matches=$(grep -R -n -E \
    --include='*.c' --include='*.cc' --include='*.cpp' --include='*.h' --include='*.hpp' \
    "tracetools_r2d2|__r2d2_llvm_trace" "$GENERATED_SRC" || true)
fi

if [ -n "$hook_matches" ]; then
  echo "generated overlay unexpectedly contains R2D2 source hooks" >&2
  exit 1
fi

echo "Prepared clean upstream ROS sources in $GENERATED_SRC"
