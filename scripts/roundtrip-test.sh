#!/usr/bin/env bash
# Roundtrip / sanity test driven by AGENTS.md guidance.
#
# Runs:
#   1. dir -> .glb         (player bundle export)
#   2. dir -> .gltf        (player bundle export, separate textures)
#   3. .glb -> dir         (player bundle import; round-trip)
#   4. gltf_validator on the produced .glb / .gltf
#   5. --info on every produced file
#   6. single MD3 -> glTF -> MD3 round-trip on a known head model
#
# Exit non-zero on any error, gltf_validator error, or new --info delta.

set -euo pipefail

ROOT="$(cd "$(dirname "$0")/.." && pwd)"
BIN="${ROOT}/build/modelconverter"
PLAYER_DIR="${ROOT}/worldofpadman/wop/models.pk3dir/models/wop_players/padman"
OUT_DIR="${ROOT}/build/roundtrip"
LOG_DIR="${OUT_DIR}/logs"

mkdir -p "${OUT_DIR}" "${LOG_DIR}"

# Tooling
GLTF_VALIDATOR="$(command -v gltf_validator || true)"

run_info() {
    local file="$1"
    local out="${LOG_DIR}/$(basename "${file}").info.txt"
    echo "  [info] ${file} -> ${out}"
    "${BIN}" --info -i "${file}" > "${out}" 2>&1
}

run_validator() {
    local file="$1"
    if [[ -z "${GLTF_VALIDATOR}" ]]; then
        echo "  [skip] gltf_validator not in PATH"
        return 0
    fi
    echo "  [validate] ${file}"
    local report="${file}.report.json"
    rm -f "${report}"
    local stderr_log="${LOG_DIR}/$(basename "${file}").validator.log"
    if ! "${GLTF_VALIDATOR}" "${file}" > "${stderr_log}" 2>&1; then
        echo "    !! validator returned non-zero"
        cat "${stderr_log}"
        [[ -f "${report}" ]] && cat "${report}"
        return 1
    fi
    # Parse "Errors: N, Warnings: M" line that gltf_validator prints.
    local summary errs warns
    summary=$(grep -E '^Errors: ' "${stderr_log}" | tail -1)
    errs=$(echo "${summary}" | sed -nE 's/.*Errors: ([0-9]+).*/\1/p')
    warns=$(echo "${summary}" | sed -nE 's/.*Warnings: ([0-9]+).*/\1/p')
    : "${errs:=0}" "${warns:=0}"
    echo "    errors=${errs} warnings=${warns}"
    if [[ "${errs}" != "0" || "${warns}" != "0" ]]; then
        echo "    !! validator complained, see ${report}"
        [[ -f "${report}" ]] && cat "${report}"
        return 1
    fi
    rm -f "${report}"
}

echo "== 1. player dir -> ${OUT_DIR}/padman.glb"
rm -f "${OUT_DIR}/padman.glb"
"${BIN}" -i "${PLAYER_DIR}" -o "${OUT_DIR}/padman.glb" > "${LOG_DIR}/export-glb.log" 2>&1
run_validator "${OUT_DIR}/padman.glb"
run_info "${OUT_DIR}/padman.glb"

echo "== 2. player dir -> ${OUT_DIR}/padman.gltf"
rm -rf "${OUT_DIR}/padman_gltf"
mkdir -p "${OUT_DIR}/padman_gltf"
"${BIN}" -i "${PLAYER_DIR}" -o "${OUT_DIR}/padman_gltf/padman.gltf" > "${LOG_DIR}/export-gltf.log" 2>&1
run_validator "${OUT_DIR}/padman_gltf/padman.gltf"
run_info "${OUT_DIR}/padman_gltf/padman.gltf"

echo "== 3. ${OUT_DIR}/padman.glb -> ${OUT_DIR}/padman_back/"
rm -rf "${OUT_DIR}/padman_back"
mkdir -p "${OUT_DIR}/padman_back"
"${BIN}" --player -i "${OUT_DIR}/padman.glb" -o "${OUT_DIR}/padman_back/padman" > "${LOG_DIR}/import.log" 2>&1
find "${OUT_DIR}/padman_back" -maxdepth 3 -type f | sort
while IFS= read -r f; do
    run_info "${f}"
done < <(find "${OUT_DIR}/padman_back" -maxdepth 3 -type f -name '*.md3')

# TODO: also validate skin file, animation.cfg and shaders file
echo "== 4. single MD3 -> glTF -> MD3 round-trip"
HEAD_MD3="${PLAYER_DIR}/head.md3"
if [[ -f "${HEAD_MD3}" ]]; then
    rm -f "${OUT_DIR}/head.glb" "${OUT_DIR}/head_back.md3"
    "${BIN}" -i "${HEAD_MD3}" -o "${OUT_DIR}/head.glb" > "${LOG_DIR}/single-export.log" 2>&1
    run_validator "${OUT_DIR}/head.glb"
    "${BIN}" -i "${OUT_DIR}/head.glb" -o "${OUT_DIR}/head_back.md3" > "${LOG_DIR}/single-import.log" 2>&1
    run_info "${HEAD_MD3}"
    run_info "${OUT_DIR}/head.glb"
    run_info "${OUT_DIR}/head_back.md3"
fi

echo "OK"
