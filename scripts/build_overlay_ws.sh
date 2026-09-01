#!/usr/bin/env bash
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
    echo "build_overlay_ws: cargo is required to read YAML config" >&2
    exit 1
  fi
}

ROS_SETUP="$(yaml_env R2D2_ROS_SETUP)"
NAV2_WS="$(yaml_env R2D2_NAV2_WS)"
NAV2_SOURCE_ROOT="$(yaml_env R2D2_NAV2_SOURCE_ROOT)"
NAV2_INSTALL_SETUP="$(yaml_env R2D2_NAV2_INSTALL_SETUP)"
NAV2_BUILD_BASE="$(yaml_env R2D2_NAV2_BUILD_BASE)"
NAV2_INSTALL_BASE="$(yaml_env R2D2_NAV2_INSTALL_BASE)"
NAV2_LOG_BASE="$(yaml_env R2D2_NAV2_LOG_BASE)"
OVERLAY_BUILD_BASE="$(yaml_env R2D2_OVERLAY_BUILD_BASE)"
OVERLAY_INSTALL_BASE="$(yaml_env R2D2_OVERLAY_INSTALL_BASE)"
OVERLAY_LOG_BASE="$(yaml_env R2D2_OVERLAY_LOG_BASE)"
OVERLAY_INSTALL_SETUP="$(yaml_env R2D2_OVERLAY_INSTALL_SETUP)"
AMENT_PYTHON="$(yaml_env R2D2_PYTHON_EXECUTABLE)"
OVERLAY_WS="$REPO_ROOT/llvm_overlay_ws"
LLVM_BUILD="$REPO_ROOT/llvm_instrumentation/build"
LLVM_PASS="$LLVM_BUILD/R2D2Instrumentation.so"
LLVM_RUNTIME="$LLVM_BUILD/libr2d2_llvm_runtime.so"
GCC_INSTALL_DIR="$(dirname "$(g++ -print-file-name=libstdc++.so)")"
CLANG_GCC_FLAG="--gcc-install-dir=$GCC_INSTALL_DIR"
PACKAGES="tracetools rcl rcl_lifecycle rclcpp rclcpp_lifecycle"
TARGET_PACKAGES="nav2_amcl nav2_behaviors nav2_behavior_tree nav2_bt_navigator nav2_collision_monitor nav2_controller nav2_costmap_2d nav2_lifecycle_manager nav2_map_server nav2_mppi_controller nav2_navfn_planner nav2_planner nav2_route nav2_smoother nav2_velocity_smoother nav2_waypoint_follower opennav_docking opennav_following"
SUPPORT_PACKAGES="r2d2_tracer r2d2_scan_bridge"
LAST_MODE_FILE="$NAV2_BUILD_BASE/.r2d2_last_mode"
MODE=plain
CLEAN=0

if [ "${1:-}" = "--clean" ]; then
  CLEAN=1
elif [ $# -gt 0 ]; then
  echo "build_overlay_ws: only --clean is supported" >&2
  exit 1
fi

if [ "$CLEAN" = 1 ]; then
  rm -rf "$OVERLAY_BUILD_BASE" "$OVERLAY_INSTALL_BASE" "$OVERLAY_LOG_BASE"
  for package in $TARGET_PACKAGES $SUPPORT_PACKAGES; do
    rm -rf "$NAV2_BUILD_BASE/$package" "$NAV2_INSTALL_BASE/$package"
  done
fi

if [ -f "$LAST_MODE_FILE" ]; then
  MODE=$(cat "$LAST_MODE_FILE")
fi

CMAKE_ARGS=(-DBUILD_TESTING=OFF -DCMAKE_BUILD_TYPE=RelWithDebInfo "-DPython3_EXECUTABLE=$AMENT_PYTHON")
"$REPO_ROOT/scripts/build_llvm_instrumentation.sh"
"$REPO_ROOT/scripts/prepare_clean_ros_overlay.sh"
LLVM_COMPILE_FLAGS="-fpass-plugin=$LLVM_PASS"
LLVM_LINK_FLAGS="-Wl,--no-as-needed,$LLVM_RUNTIME,--as-needed -Wl,-rpath,$LLVM_BUILD"
case "$MODE" in
  plain)
    CMAKE_ARGS+=(-DCMAKE_C_COMPILER=/usr/bin/clang-18 -DCMAKE_CXX_COMPILER=/usr/bin/clang++-18
      "-DCMAKE_C_FLAGS=-O1 -g $LLVM_COMPILE_FLAGS"
      "-DCMAKE_CXX_FLAGS=-O1 -g -Wno-error=inconsistent-missing-override $CLANG_GCC_FLAG $LLVM_COMPILE_FLAGS"
      "-DCMAKE_EXE_LINKER_FLAGS=$LLVM_LINK_FLAGS"
      "-DCMAKE_SHARED_LINKER_FLAGS=$LLVM_LINK_FLAGS"
      "-DCMAKE_MODULE_LINKER_FLAGS=$LLVM_LINK_FLAGS") ;;
  coverage)
    CMAKE_ARGS+=(-DCMAKE_C_COMPILER=/usr/bin/clang-18 -DCMAKE_CXX_COMPILER=/usr/bin/clang++-18
      "-DCMAKE_C_FLAGS=--coverage -DCOVERAGE_RUN=1 -w -Wno-error $LLVM_COMPILE_FLAGS"
      "-DCMAKE_CXX_FLAGS=--coverage -DCOVERAGE_RUN=1 -w -Wno-error $CLANG_GCC_FLAG $LLVM_COMPILE_FLAGS"
      "-DCMAKE_EXE_LINKER_FLAGS=--coverage $LLVM_LINK_FLAGS"
      "-DCMAKE_SHARED_LINKER_FLAGS=--coverage $LLVM_LINK_FLAGS"
      "-DCMAKE_MODULE_LINKER_FLAGS=--coverage $LLVM_LINK_FLAGS") ;;
  tsan)
    CMAKE_ARGS+=(-DCMAKE_C_COMPILER=/usr/bin/clang-18 -DCMAKE_CXX_COMPILER=/usr/bin/clang++-18
      "-DCMAKE_C_FLAGS=-O1 -g -w -Wno-error -fno-omit-frame-pointer -fsanitize=thread -fno-sanitize-link-runtime $LLVM_COMPILE_FLAGS"
      "-DCMAKE_CXX_FLAGS=-O1 -g -w -Wno-error -fno-omit-frame-pointer -fsanitize=thread -fno-sanitize-link-runtime $CLANG_GCC_FLAG $LLVM_COMPILE_FLAGS"
      "-DCMAKE_EXE_LINKER_FLAGS=-Wl,--no-as-needed,-ltsan,--as-needed $LLVM_LINK_FLAGS"
      "-DCMAKE_SHARED_LINKER_FLAGS=-Wl,--no-as-needed,-ltsan,--as-needed $LLVM_LINK_FLAGS"
      "-DCMAKE_MODULE_LINKER_FLAGS=-Wl,--no-as-needed,-ltsan,--as-needed $LLVM_LINK_FLAGS") ;;
  sancov)
    CMAKE_ARGS+=(-DCMAKE_C_COMPILER=/usr/bin/clang-18 -DCMAKE_CXX_COMPILER=/usr/bin/clang++-18
      "-DCMAKE_C_FLAGS=-O1 -g -w -Wno-error -fno-omit-frame-pointer -fsanitize=thread -fsanitize-coverage=trace-pc-guard,pc-table -DSANITIZER_COVERAGE_RUN=1 $LLVM_COMPILE_FLAGS"
      "-DCMAKE_CXX_FLAGS=-O1 -g -w -Wno-error -fno-omit-frame-pointer -fsanitize=thread -fsanitize-coverage=trace-pc-guard,pc-table -DSANITIZER_COVERAGE_RUN=1 $CLANG_GCC_FLAG $LLVM_COMPILE_FLAGS"
      "-DCMAKE_EXE_LINKER_FLAGS=-fsanitize=thread $LLVM_LINK_FLAGS"
      "-DCMAKE_SHARED_LINKER_FLAGS=-fsanitize=thread $LLVM_LINK_FLAGS"
      "-DCMAKE_MODULE_LINKER_FLAGS=-fsanitize=thread $LLVM_LINK_FLAGS") ;;
  *) echo "build_overlay_ws: unsupported nav2 mode '$MODE'" >&2; exit 1 ;;
esac

unset COLCON_CURRENT_PREFIX
set +u
source "$ROS_SETUP"
source "$NAV2_INSTALL_SETUP"
set -u

cd "$OVERLAY_WS"
colcon --log-base "$OVERLAY_LOG_BASE" build --symlink-install --parallel-workers 12 \
  --build-base "$OVERLAY_BUILD_BASE" \
  --install-base "$OVERLAY_INSTALL_BASE" \
  --packages-select $PACKAGES \
  --cmake-clean-cache \
  --cmake-args "${CMAKE_ARGS[@]}"

set +u
source "$OVERLAY_INSTALL_SETUP"
set -u

cd "$NAV2_WS"
colcon --log-base "$NAV2_LOG_BASE" build --symlink-install --parallel-workers 12 \
  --base-paths "$NAV2_SOURCE_ROOT" \
  --build-base "$NAV2_BUILD_BASE" \
  --install-base "$NAV2_INSTALL_BASE" \
  --packages-select $TARGET_PACKAGES \
  --cmake-clean-cache \
  --cmake-args "${CMAKE_ARGS[@]}"

# Keep the project's ROS helper packages in the same local Nav2 prefix used by
# the file-driven senders.  They must use the active mode's link flags when the
# local rclcpp underlay is sanitizer-instrumented.
colcon --log-base "$NAV2_LOG_BASE" build --symlink-install --parallel-workers 2 \
  --base-paths "$NAV2_WS/src/r2d2_tracer" "$NAV2_WS/src/r2d2_scan_bridge" \
  --build-base "$NAV2_BUILD_BASE" \
  --install-base "$NAV2_INSTALL_BASE" \
  --packages-select $SUPPORT_PACKAGES \
  --cmake-clean-cache \
  --cmake-args "${CMAKE_ARGS[@]}"
