#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
BUILD_DIR="${ROOT_DIR}/build-wasm/full-simulation"

if ! command -v emcmake >/dev/null 2>&1; then
	printf 'error: emcmake was not found. Rebuild the Dev Container to install Emscripten.\n' >&2
	exit 1
fi

emcmake cmake -S "${ROOT_DIR}/wasm" -B "${BUILD_DIR}" -DCMAKE_BUILD_TYPE=Release
cmake --build "${BUILD_DIR}" --target convex_mpc_sim_wasm --parallel
"${ROOT_DIR}/scripts/stage_mujoco_web.sh"
