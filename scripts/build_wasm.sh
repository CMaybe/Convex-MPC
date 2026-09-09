#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
OUTPUT_DIR="${ROOT_DIR}/web/public/wasm"
BUILD_DIR="${ROOT_DIR}/build-wasm"
QPOASES_DIR="${ROOT_DIR}/build/_deps/qpoases-src"

if ! command -v em++ >/dev/null 2>&1; then
    printf 'error: em++ was not found. Rebuild the Dev Container to install Emscripten.\n' >&2
    exit 1
fi

if [[ ! -d "${QPOASES_DIR}" ]]; then
    printf 'error: qpOASES sources are missing. Configure the native CMake build first.\n' >&2
    exit 1
fi

mkdir -p "${OUTPUT_DIR}" "${BUILD_DIR}"

mapfile -t QPOASES_SOURCES < <(find "${QPOASES_DIR}/src" -maxdepth 1 -name '*.cpp' -print | sort)

em++ \
    -std=c++17 \
    -O3 \
    -DNDEBUG \
    -I"${ROOT_DIR}/ConvexMPC/include" \
    -I"${QPOASES_DIR}/include" \
    -I/usr/include/eigen3 \
    "${ROOT_DIR}/wasm/bindings.cpp" \
    "${ROOT_DIR}/ConvexMPC/src/convex_mpc.cpp" \
    "${ROOT_DIR}/ConvexMPC/src/robot_model.cpp" \
    "${ROOT_DIR}/ConvexMPC/src/robot_state.cpp" \
    "${ROOT_DIR}/ConvexMPC/src/robot_controller.cpp" \
    "${QPOASES_SOURCES[@]}" \
    --bind \
    -s MODULARIZE=1 \
    -s EXPORT_ES6=1 \
    -s ENVIRONMENT=web \
    -s ALLOW_MEMORY_GROWTH=1 \
    -s STACK_SIZE=5242880 \
    -s DISABLE_EXCEPTION_CATCHING=0 \
    -o "${OUTPUT_DIR}/convex_mpc.js"

printf 'WASM module written to %s\n' "${OUTPUT_DIR}/convex_mpc.js"
