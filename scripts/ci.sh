#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "${ROOT_DIR}"

cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build --parallel
./build/simulation/mpc_locomotion --headless --steps 10000 --vx 0.2

npm --prefix web ci
npm --prefix web run build:wasm
npm --prefix web run test:wasm
npm --prefix web run build:full-wasm
npm --prefix web run build
