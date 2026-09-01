#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR=$(cd "$(dirname "$0")" && pwd)
REPO_ROOT=$(cd "$SCRIPT_DIR/.." && pwd)
SOURCE_DIR="$REPO_ROOT/llvm_instrumentation"
BUILD_DIR="$SOURCE_DIR/build"
GCC_INSTALL_DIR="$(dirname "$(g++ -print-file-name=libstdc++.so)")"

cmake -S "$SOURCE_DIR" -B "$BUILD_DIR" \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_CXX_COMPILER=/usr/bin/clang++-18 \
  "-DCMAKE_CXX_FLAGS=--gcc-install-dir=$GCC_INSTALL_DIR" \
  -DLLVM_DIR=/usr/lib/llvm-18/lib/cmake/llvm
cmake --build "$BUILD_DIR" --parallel

PASS="$BUILD_DIR/R2D2Instrumentation.so"
RUNTIME="$BUILD_DIR/libr2d2_llvm_runtime.so"
test -f "$PASS"
test -f "$RUNTIME"

OUT="$BUILD_DIR/tracepoint_fixture.instrumented.ll"
opt-18 -load-pass-plugin "$PASS" -passes=r2d2-instrument -S \
  "$SOURCE_DIR/tests/tracepoint_fixture.ll" -o "$OUT"
COUNT=$(grep -c '__r2d2_llvm_trace' "$OUT")
if [ "$COUNT" -lt 8 ]; then
  echo "build_llvm_instrumentation: expected hook declaration plus seven calls, got $COUNT" >&2
  exit 1
fi

echo "LLVM pass: $PASS"
echo "LLVM runtime: $RUNTIME"
echo "IR smoke: $OUT ($COUNT hook references)"
