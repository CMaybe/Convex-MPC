#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
SDK_DIR="${ROOT_DIR}/build/_deps/mujoco_sdk-src"
PUBLIC_DIR="${ROOT_DIR}/web/public"

if [[ ! -f "${SDK_DIR}/wasm/mujoco.js" ]]; then
    printf 'error: MuJoCo SDK is missing. Configure the native CMake build first.\n' >&2
    exit 1
fi

mkdir -p "${PUBLIC_DIR}/mujoco" "${PUBLIC_DIR}/robots"
cp "${SDK_DIR}/wasm/mujoco.js" "${SDK_DIR}/wasm/mujoco.wasm" "${PUBLIC_DIR}/mujoco/"
rm -rf "${PUBLIC_DIR}/robots/anymal_c"
cp -R "${ROOT_DIR}/simulation/rsc/anymal_c" "${PUBLIC_DIR}/robots/anymal_c"
{
    printf '[\n'
    find "${PUBLIC_DIR}/robots/anymal_c" -type f ! -name scene.xml -printf '%P\n' | sort | while IFS= read -r path; do
        printf '  "%s",\n' "${path}"
    done
    printf '  "scene.xml"\n]\n'
} > "${PUBLIC_DIR}/robots/anymal_c/assets.json"
printf 'MuJoCo WASM and ANYmal C assets staged in %s\n' "${PUBLIC_DIR}"
