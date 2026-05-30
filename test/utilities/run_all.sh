#!/usr/bin/env bash

# This is a tool. Per SDR's internal standards, these are safe to
# vibe code provided you do so safely!

set -uo pipefail

# ── Resolve paths relative to this script's location ─────────────────────────
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$(cd "$SCRIPT_DIR/../.." && pwd)"
TARGET_DIR="$ROOT_DIR/test/app/rev2"

# ── Tracking ──────────────────────────────────────────────────────────────────
declare -A RESULTS
PASS_COUNT=0
FAIL_COUNT=0

# ── Process one directory ─────────────────────────────────────────────────────
process_dir() {
    local dir="$1"
    local name
    name="$(basename "$dir")"
    echo "── Running: $name ──────────────────────────────────────"

    # Step 1: make test
    if ! make -C "$dir" test; then
        echo "  [FAIL] make test failed in $name"
        RESULTS["$name"]="FAIL (make test failed)"
        (( FAIL_COUNT++ ))   # ← was missing
        return
    fi

    # Step 2: check results.txt
    local results_file="$dir/results.txt"
    if [[ ! -f "$results_file" ]]; then
        echo "  [FAIL] results.txt not found in $name"
        RESULTS["$name"]="FAIL (results.txt missing)"
        (( FAIL_COUNT++ ))   # ← was missing
        return
    fi

    if tail -n 7 "$results_file" | grep -q "Result: PASS"; then
        echo "  [PASS] $name"
        RESULTS["$name"]="PASS"
        (( PASS_COUNT++ ))
    else
        echo "  [FAIL] 'Result: PASS' not found in last 7 lines of results.txt"
        RESULTS["$name"]="FAIL (Result: PASS not found)"
        (( FAIL_COUNT++ ))
    fi
}

# ── Main ──────────────────────────────────────────────────────────────────────
if [[ ! -d "$TARGET_DIR" ]]; then
    echo "ERROR: target directory not found: $TARGET_DIR"
    exit 1
fi

# Immediate subdirectories of rev2 only (maxdepth 1)
while IFS= read -r -d '' dir; do
    process_dir "$dir"
done < <(find "$TARGET_DIR" -mindepth 1 -maxdepth 1 -type d -print0 | sort -z)

# ── Summary ───────────────────────────────────────────────────────────────────
echo ""
echo "════════════════════════════════════════"
echo " Results"
echo "════════════════════════════════════════"
for name in $(printf '%s\n' "${!RESULTS[@]}" | sort); do
    printf "  %-30s %s\n" "$name" "${RESULTS[$name]}"
done
echo "────────────────────────────────────────"
echo "  PASSED: $PASS_COUNT"
echo "  FAILED: $FAIL_COUNT"
echo "════════════════════════════════════════"

# Exit non-zero if any failures
[[ $FAIL_COUNT -eq 0 ]]