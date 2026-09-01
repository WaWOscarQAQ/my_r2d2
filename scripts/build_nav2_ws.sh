#!/usr/bin/env bash
# scripts/build_nav2_ws.sh — nav2_ws 一键构建入口
#
# 把散落在文档里的三种 colcon 构建命令收口成一个入口：
#   plain    普通 RelWithDebInfo 构建（Quickstart 默认，无 sanitizer/覆盖）
#   coverage 纯 coverage 构建（1a，gcc + --coverage + atomic，论文覆盖口径近似）
#   tsan      TSAN-only 构建（并发检测战役用；不混 gcov 覆盖率）
#   sancov    TSAN + LLVM SanitizerCoverage 构建（同进程 race + edge 覆盖）
#
# 用法：
#   scripts/build_nav2_ws.sh [--coverage|--tsan|--sancov] [--clean] [--help]
#
# 配置来源：
#   所需配置全部从 config/r2d2_env.yaml 读取。
#   若某个键为空，Rust helper 会直接报 “<KEY> is empty in ...”。
#
# 说明：
# - 模式切换必须 --clean（不同编译标志混用会产生无法链接的产物，文档记载
#   切换需干净重建）；脚本会记录上次构建模式，检测到切换且未带 --clean 时提示。
# - 构建 coverage 模式后若跑覆盖战役，需先清历史计数（见
#   docs/plan/nav2_lcov_full_run.md 第二节：find build -name "*.gcda" -delete）。
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
    echo "build_nav2_ws: cargo is required to read YAML config" >&2
    exit 1
  fi
}
NAV2_WS="$(yaml_env R2D2_NAV2_WS)"
NAV2_SOURCE_ROOT="$(yaml_env R2D2_NAV2_SOURCE_ROOT)"
NAV2_BUILD_BASE="$(yaml_env R2D2_NAV2_BUILD_BASE)"
NAV2_INSTALL_BASE="$(yaml_env R2D2_NAV2_INSTALL_BASE)"
NAV2_LOG_BASE="$(yaml_env R2D2_NAV2_LOG_BASE)"
PROFILE="$(yaml_env R2D2_PROFILE)"
ROS_SETUP="$(yaml_env R2D2_ROS_SETUP)"
AMENT_PYTHON="$(yaml_env R2D2_PYTHON_EXECUTABLE)"
GCC_INSTALL_DIR="$(dirname "$(g++ -print-file-name=libstdc++.so)")"
CLANG_GCC_FLAG="--gcc-install-dir=$GCC_INSTALL_DIR"
NAV2_PACKAGES="nav2_msgs nav2_common nav2_ros_common nav2_util nav2_voxel_grid nav2_core nav2_costmap_2d costmap_queue nav_2d_msgs nav_2d_utils dwb_msgs dwb_core dwb_critics dwb_plugins nav2_dwb_controller nav2_amcl nav2_behaviors nav2_behavior_tree nav2_bt_navigator nav2_collision_monitor nav2_controller nav2_constrained_smoother nav2_graceful_controller nav2_lifecycle_manager nav2_loopback_sim nav2_map_server nav2_mppi_controller nav2_navfn_planner nav2_planner nav2_regulated_pure_pursuit_controller nav2_rotation_shim_controller nav2_route nav2_rviz_plugins nav2_simple_commander nav2_smac_planner nav2_smoother nav2_system_tests nav2_theta_star_planner nav2_velocity_smoother nav2_waypoint_follower opennav_docking_core opennav_docking_bt opennav_docking opennav_following nav2_bringup navigation2"
SUPPORT_PACKAGES="r2d2_tracer r2d2_scan_bridge"
PACKAGES="$SUPPORT_PACKAGES $NAV2_PACKAGES"
MODE=plain
CLEAN=0

usage() {
  sed -n '2,17p' "$0" | sed 's/^# \{0,1\}//'
}

die() {
  echo "build_nav2_ws: $*" >&2
  exit 1
}

while [ $# -gt 0 ]; do
  case "$1" in
    --coverage) MODE=coverage ;;
    --tsan) MODE=tsan ;;
    --clean) CLEAN=1 ;;
    --sancov) MODE=sancov ;;
    --help | -h) usage; exit 0 ;;
    *) die "unknown argument: $1（可用 --coverage/--tsan/--sancov/--clean/--help）" ;;
  esac
  shift
done

if { [ "$PROFILE" = "coverage" ] && [ "$MODE" != "coverage" ] && [ "$MODE" != "plain" ]; } ||
  { [ "$PROFILE" = "tsan" ] && [ "$MODE" != "tsan" ] && [ "$MODE" != "plain" ]; } ||
  { [ "$PROFILE" = "sancov" ] && [ "$MODE" != "sancov" ] && [ "$MODE" != "plain" ]; }; then
  die "R2D2_PROFILE=$PROFILE conflicts with --$MODE; use matching profile and mode"
fi

# 上次构建模式（用于提示模式切换需 --clean）
LAST_MODE_FILE="$NAV2_BUILD_BASE/.r2d2_last_mode"
if [ "$CLEAN" = 0 ] && [ -f "$LAST_MODE_FILE" ]; then
  LAST_MODE=$(cat "$LAST_MODE_FILE")
  if [ "$LAST_MODE" != "$MODE" ]; then
    echo "build_nav2_ws: 上次构建模式为 $LAST_MODE，本次为 $MODE；不同标志混用会失败，请加 --clean 干净重建（约 2-3 分钟）"
    exit 1
  fi
fi

if [ "$CLEAN" = 1 ] && [ -d "$NAV2_BUILD_BASE" ]; then
  echo "build_nav2_ws: --clean：移除旧构建与安装产物（$PACKAGES）"
  for p in $PACKAGES; do
    rm -rf "$NAV2_BUILD_BASE/$p" "$NAV2_INSTALL_BASE/$p"
  done
  rm -f "$LAST_MODE_FILE"
fi

# ---- 各模式的 CMake 参数 ---------------------------------------------------
# shellcheck disable=SC2086
CMAKE_ARGS=(-DBUILD_TESTING=OFF -DCMAKE_BUILD_TYPE=RelWithDebInfo "-DPython3_EXECUTABLE=$AMENT_PYTHON")
case "$MODE" in
  plain) ;;
  coverage)
    CMAKE_ARGS+=(
      -DCMAKE_C_COMPILER=/usr/bin/cc
      -DCMAKE_CXX_COMPILER=/usr/bin/c++
      "-DCMAKE_C_FLAGS=--coverage -DCOVERAGE_RUN=1 -fprofile-update=atomic -w -Wno-error"
      "-DCMAKE_CXX_FLAGS=--coverage -DCOVERAGE_RUN=1 -fprofile-update=atomic -w -Wno-error"
      "-DCMAKE_EXE_LINKER_FLAGS=--coverage"
      "-DCMAKE_SHARED_LINKER_FLAGS=--coverage"
    ) ;;
  tsan)
    CMAKE_ARGS+=(
      -DCMAKE_C_COMPILER=/usr/bin/cc
      -DCMAKE_CXX_COMPILER=/usr/bin/c++
      "-DCMAKE_C_FLAGS=-O1 -g -w -Wno-error -fno-omit-frame-pointer -fsanitize=thread"
      "-DCMAKE_CXX_FLAGS=-O1 -g -w -Wno-error -fno-omit-frame-pointer -fsanitize=thread"
      "-DCMAKE_EXE_LINKER_FLAGS=-fsanitize=thread"
      "-DCMAKE_SHARED_LINKER_FLAGS=-fsanitize=thread"
    ) ;;
  sancov)
    CMAKE_ARGS+=(
      -DCMAKE_C_COMPILER=/usr/bin/clang-18
      -DCMAKE_CXX_COMPILER=/usr/bin/clang++-18
      "-DCMAKE_C_FLAGS=-O1 -g -w -Wno-error -fno-omit-frame-pointer -fsanitize=thread -fsanitize-coverage=trace-pc-guard,pc-table -DSANITIZER_COVERAGE_RUN=1"
      "-DCMAKE_CXX_FLAGS=-O1 -g -w -Wno-error -fno-omit-frame-pointer -fsanitize=thread -fsanitize-coverage=trace-pc-guard,pc-table -DSANITIZER_COVERAGE_RUN=1 $CLANG_GCC_FLAG"
      "-DCMAKE_EXE_LINKER_FLAGS=-fsanitize=thread"
      "-DCMAKE_SHARED_LINKER_FLAGS=-fsanitize=thread"
    ) ;;
esac

# ---- 构建 -----------------------------------------------------------------
echo "build_nav2_ws: profile=$PROFILE mode=$MODE ws=$NAV2_WS build=$NAV2_BUILD_BASE install=$NAV2_INSTALL_BASE python=$AMENT_PYTHON"
# 用户 shell 可能残留 COLCON_CURRENT_PREFIX（zsh 场景踩坑记录），先清掉。
unset COLCON_CURRENT_PREFIX
# setup.bash 引用未绑定变量，set -u 下 source 会报错（launch scripts 同款处理）。
set +u
source "$ROS_SETUP"
set -u
cd "$NAV2_WS"
colcon --log-base "$NAV2_LOG_BASE" build --symlink-install --parallel-workers 12 \
  --base-paths "$NAV2_SOURCE_ROOT" \
  --build-base "$NAV2_BUILD_BASE" \
  --install-base "$NAV2_INSTALL_BASE" \
  --packages-select $NAV2_PACKAGES \
  --cmake-clean-cache \
  --cmake-args "${CMAKE_ARGS[@]}"

colcon --log-base "$NAV2_LOG_BASE" build --symlink-install --parallel-workers 2 \
  --base-paths "$NAV2_WS/src/r2d2_tracer" "$NAV2_WS/src/r2d2_scan_bridge" \
  --build-base "$NAV2_BUILD_BASE" \
  --install-base "$NAV2_INSTALL_BASE" \
  --packages-select $SUPPORT_PACKAGES \
  --cmake-clean-cache \
  --cmake-args -DBUILD_TESTING=OFF -DCMAKE_BUILD_TYPE=RelWithDebInfo \
    "-DPython3_EXECUTABLE=$AMENT_PYTHON"

mkdir -p "$NAV2_BUILD_BASE"
echo "$MODE" >"$LAST_MODE_FILE"
echo "build_nav2_ws: 构建完成（mode=$MODE）。"
if [ "$MODE" = "coverage" ]; then
  echo "build_nav2_ws: 跑覆盖战役前记得清历史计数：find build -name '*.gcda' -delete（见 nav2_lcov_full_run.md 第二节）"
fi
