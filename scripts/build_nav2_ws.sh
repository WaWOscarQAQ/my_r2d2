#!/usr/bin/env bash
# scripts/build_nav2_ws.sh — nav2_ws 一键构建入口
#
# 把散落在文档里的三种 colcon 构建命令收口成一个入口：
#   plain    普通 RelWithDebInfo 构建（Quickstart 默认，无 sanitizer/覆盖）
#   coverage 纯 coverage 构建（1a，gcc + --coverage + atomic，论文覆盖口径近似）
#   tsan      TSAN + coverage 构建（1b，并发检测战役用）
#
# 用法：
#   scripts/build_nav2_ws.sh [--coverage|--tsan] [--clean] [--help]
#
# 环境变量（同 nav2_costmap_e2e，见 README 第 5 节）：
#   R2D2_NAV2_WS    工作区路径（默认：本仓库的 nav2_ws/）
#   R2D2_ROS_SETUP  ROS 2 setup 脚本（默认 /opt/ros/jazzy/setup.bash）
#
# 说明：
# - 模式切换必须 --clean（不同编译标志混用会产生无法链接的产物，文档记载
#   切换需干净重建）；脚本会记录上次构建模式，检测到切换且未带 --clean 时提示。
# - 构建覆盖/TSAN 模式后若跑覆盖战役，需先清历史计数（见
#   docs/plan/nav2_lcov_full_run.md 第二节：find build -name "*.gcda" -delete）。
set -euo pipefail

SCRIPT_DIR=$(cd "$(dirname "$0")" && pwd)
NAV2_WS="${R2D2_NAV2_WS:-$SCRIPT_DIR/../nav2_ws}"
ROS_SETUP="${R2D2_ROS_SETUP:-/opt/ros/jazzy/setup.bash}"
PACKAGES="r2d2_tracer r2d2_scan_bridge nav2_msgs nav2_common nav2_util nav2_voxel_grid nav2_costmap_2d"
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
    --help | -h) usage; exit 0 ;;
    *) die "unknown argument: $1（可用 --coverage/--tsan/--clean/--help）" ;;
  esac
  shift
done

# ---- 前置检查 -------------------------------------------------------------
[ -f "$ROS_SETUP" ] || die "ROS setup 不存在：$ROS_SETUP（可设置 R2D2_ROS_SETUP 覆盖）"
[ -d "$NAV2_WS" ] || die "工作区不存在：$NAV2_WS（可设置 R2D2_NAV2_WS 覆盖）"
[ -d "$NAV2_WS/src/r2d2_tracer" ] || die "缺少 $NAV2_WS/src/r2d2_tracer；工作区搭建步骤见 docs/plan/nav2_jazzy_instrumentation_plan.md 第 2 节"
[ -d "$NAV2_WS/src/r2d2_scan_bridge" ] || die "缺少 $NAV2_WS/src/r2d2_scan_bridge；工作区搭建步骤见 docs/plan/nav2_jazzy_instrumentation_plan.md 第 2 节"
[ -d "$NAV2_WS/src/navigation2/nav2_costmap_2d" ] || die "缺少 $NAV2_WS/src/navigation2/nav2_costmap_2d（插桩后的 navigation2 副本）"
[ -f "$NAV2_WS/costmap_params.yaml" ] || die "缺少 $NAV2_WS/costmap_params.yaml"
command -v colcon >/dev/null 2>&1 || die "colcon 不在 PATH（ubuntu 安装 python3-colcon-common-extensions）"

# 上次构建模式（用于提示模式切换需 --clean）
LAST_MODE_FILE="$NAV2_WS/build/.r2d2_last_mode"
if [ "$CLEAN" = 0 ] && [ -f "$LAST_MODE_FILE" ]; then
  LAST_MODE=$(cat "$LAST_MODE_FILE")
  if [ "$LAST_MODE" != "$MODE" ]; then
    echo "build_nav2_ws: 上次构建模式为 $LAST_MODE，本次为 $MODE；不同标志混用会失败，请加 --clean 干净重建（约 2-3 分钟）"
    exit 1
  fi
fi

if [ "$CLEAN" = 1 ] && [ -d "$NAV2_WS/build" ]; then
  echo "build_nav2_ws: --clean：移除旧构建与安装产物（$PACKAGES）"
  for p in $PACKAGES; do
    rm -rf "$NAV2_WS/build/$p" "$NAV2_WS/install/$p"
  done
  rm -f "$LAST_MODE_FILE"
fi

# ---- 各模式的 CMake 参数（与文档 1a/1b 逐字一致）--------------------------
# shellcheck disable=SC2086
CMAKE_ARGS=(-DBUILD_TESTING=OFF -DCMAKE_BUILD_TYPE=RelWithDebInfo)
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
      "-DCMAKE_C_FLAGS=-O1 -g -w -Wno-error -fno-omit-frame-pointer -fsanitize=thread --coverage -fprofile-update=atomic -DCOVERAGE_RUN=1"
      "-DCMAKE_CXX_FLAGS=-O1 -g -w -Wno-error -fno-omit-frame-pointer -fsanitize=thread --coverage -fprofile-update=atomic -DCOVERAGE_RUN=1"
      "-DCMAKE_EXE_LINKER_FLAGS=-fsanitize=thread --coverage"
      "-DCMAKE_SHARED_LINKER_FLAGS=-fsanitize=thread --coverage"
    ) ;;
esac

# ---- 构建 -----------------------------------------------------------------
echo "build_nav2_ws: mode=$MODE ws=$NAV2_WS"
# 用户 shell 可能残留 COLCON_CURRENT_PREFIX（zsh 场景踩坑记录），先清掉。
unset COLCON_CURRENT_PREFIX
# setup.bash 引用未绑定变量，set -u 下 source 会报错（与 launch_stack.sh 同款处理）。
set +u
source "$ROS_SETUP"
set -u
cd "$NAV2_WS"
colcon build --symlink-install --parallel-workers 12 \
  --packages-select $PACKAGES \
  --cmake-clean-cache \
  --cmake-args "${CMAKE_ARGS[@]}"

mkdir -p "$NAV2_WS/build"
echo "$MODE" >"$LAST_MODE_FILE"
echo "build_nav2_ws: 构建完成（mode=$MODE）。"
if [ "$MODE" != "plain" ]; then
  echo "build_nav2_ws: 跑覆盖战役前记得清历史计数：find build -name '*.gcda' -delete（见 nav2_lcov_full_run.md 第二节）"
fi
