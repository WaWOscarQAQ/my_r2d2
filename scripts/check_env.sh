#!/usr/bin/env bash
# scripts/check_env.sh — 运行 nav2_costmap_e2e 前的环境预检
#
# 在真正运行 example 前把常见缺依赖 / 路径问题提前暴露。只读检查，唯一副作用
# 是 /dev/shm 写一个临时文件并立即删除（验证可写）。
#
# 用法：scripts/check_env.sh
#
# 环境变量（同 nav2_costmap_e2e，见 README 第 5 节）：
#   R2D2_NAV2_WS    工作区路径（默认：本仓库的 nav2_ws/）
#   R2D2_ROS_SETUP  ROS 2 setup 脚本（默认 /opt/ros/jazzy/setup.bash）
#
# 退出码：0 = 必需项全部通过（WARN 不影响）；1 = 有必需项失败。
set -u

SCRIPT_DIR=$(cd "$(dirname "$0")" && pwd)
NAV2_WS="${R2D2_NAV2_WS:-$SCRIPT_DIR/../nav2_ws}"
ROS_SETUP="${R2D2_ROS_SETUP:-/opt/ros/jazzy/setup.bash}"

FAILED=0

ok()   { echo "  [OK]   $*"; }
fail() { echo "  [FAIL] $*"; FAILED=1; }
warn() { echo "  [WARN] $*"; }

echo "check_env: 检查运行 nav2_costmap_e2e 的环境"

echo "== 系统 =="
if [ "$(uname -s)" = "Linux" ]; then
  ok "Linux（本闭环仅支持 Linux）"
else
  fail "非 Linux 系统：$(uname -s)（tracer 依赖 /dev/shm 与 POSIX shm）"
fi

echo "== ROS 2 =="
if [ -f "$ROS_SETUP" ]; then
  ok "ROS setup：$ROS_SETUP"
  ROS_BIN=$(dirname "$ROS_SETUP")/bin
  if command -v ros2 >/dev/null 2>&1; then
    ok "ros2 在 PATH（$(command -v ros2)）"
  elif [ -x "$ROS_BIN/ros2" ]; then
    ok "ros2 位于 $ROS_BIN/ros2（不在 PATH；example 与构建脚本内部会 source setup，无需手动 export）"
  else
    fail "找不到 ros2（PATH 与 $ROS_BIN 都没有）"
  fi
else
  fail "ROS setup 不存在：$ROS_SETUP（可设置 R2D2_ROS_SETUP 覆盖）"
fi

echo "== 工具链 =="
command -v colcon >/dev/null 2>&1 && ok "colcon（$(command -v colcon)）" || fail "colcon 不在 PATH（ubuntu 安装 python3-colcon-common-extensions）"
command -v setarch >/dev/null 2>&1 && ok "setarch（$(command -v setarch)）" || fail "setarch 不在 PATH（util-linux；TSAN 构建下关闭 ASLR 必需）"
if command -v cargo >/dev/null 2>&1; then
  RUSTC_VER=$(rustc --version 2>/dev/null | awk '{print $2}')
  if [ -n "$RUSTC_VER" ] && [ "$(printf '1.85\n%s' "$RUSTC_VER" | sort -V | head -1)" = "1.85" ]; then
    ok "Rust 工具链 $RUSTC_VER（edition 2024 需要 1.85+）"
  else
    fail "rustc 版本 $RUSTC_VER 过低（edition 2024 需要 1.85+）"
  fi
else
  fail "cargo 不在 PATH（安装 Rust 1.85+：https://rustup.rs）"
fi
command -v lcov >/dev/null 2>&1 && ok "lcov（$(command -v lcov)）" || warn "lcov 不在 PATH；只有加 --lcov-dir 抓分支覆盖时才需要"

echo "== /dev/shm =="
if mountpoint -q /dev/shm 2>/dev/null || df -h /dev/shm >/dev/null 2>&1; then
  SHM_PROBE="/dev/shm/.r2d2_check_env_$$"
  if : >"$SHM_PROBE" 2>/dev/null; then
    rm -f "$SHM_PROBE"
    ok "/dev/shm 已挂载且可写"
  else
    fail "/dev/shm 不可写（tracer 环形缓冲与 shm 对象都放在这里）"
  fi
else
  fail "/dev/shm 不可用（tracer 依赖 POSIX shared memory）"
fi

echo "== nav2_ws =="
if [ -d "$NAV2_WS" ]; then
  ok "工作区存在：$NAV2_WS"
  if [ -f "$NAV2_WS/install/setup.bash" ]; then
    ok "已构建：$NAV2_WS/install/setup.bash"
  else
    fail "工作区未构建（缺少 install/setup.bash）；先跑 scripts/build_nav2_ws.sh"
  fi
  [ -f "$NAV2_WS/costmap_params.yaml" ] && ok "costmap_params.yaml 存在" || fail "缺少 $NAV2_WS/costmap_params.yaml（工作区搭建见 docs/plan/nav2_jazzy_instrumentation_plan.md 第 2 节）"
  [ -f "$NAV2_WS/launch_stack.sh" ] && ok "launch_stack.sh 存在" || fail "缺少 $NAV2_WS/launch_stack.sh"
  [ -d "$NAV2_WS/src/navigation2/nav2_costmap_2d" ] && ok "插桩 navigation2 副本存在" || fail "缺少 $NAV2_WS/src/navigation2/nav2_costmap_2d"
else
  fail "工作区不存在：$NAV2_WS（可设置 R2D2_NAV2_WS 覆盖；搭建见 docs/plan/nav2_jazzy_instrumentation_plan.md 第 2 节）"
fi

echo
if [ "$FAILED" = 0 ]; then
  echo "check_env: 全部必需项通过，可以运行 cargo run --example nav2_costmap_e2e"
  exit 0
fi
echo "check_env: 存在未通过的必需项，修复后重试（FAIL 行附排查方向）"
exit 1
