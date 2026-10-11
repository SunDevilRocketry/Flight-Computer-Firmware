#!/usr/bin/env bash
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

python3 "$repo_root/test/framework/tools/trace_report.py" \
    --search-root "$repo_root" \
    --req-format '^\s*(RQ\.FC-SYS\.[0-9]{5})(?:\s+\([^)]*\))?\s+-\s+(.+?)\s*$' \
    --req-format '^\s*(RQ\.FC-SW\.[0-9]{5})(?:\s+\([^)]*\))?\s+-\s+(.+?)\s*$' \
    --spec-files '^reqs/SysRD\.md$' \
    --spec-files '^reqs/SDD\.md$' \
    --results-files '^test/app/(?:[^/]+/)*(results\.txt|[^/]*results\.md)$'

while IFS= read -r tag; do
    printf '%s\n' \
        "trace_fc.sh: warning: $tag has no (Derived) marking or (Trace: RQ.FC-SYS.#####) reference." \
        >&2
done < <(
    awk '
        match($0, /RQ\.FC-SW\.[0-9][0-9][0-9][0-9][0-9]/) &&
        $0 ~ /[[:space:]]+-[[:space:]]+/ &&
        $0 !~ /\(Derived\)/ &&
        $0 !~ /\(Trace:[[:space:]]*RQ\.FC-SYS\.[0-9][0-9][0-9][0-9][0-9][[:space:]]*\)/ {
            print substr($0, RSTART, RLENGTH)
        }
    ' "$repo_root/reqs/SDD.md"
)