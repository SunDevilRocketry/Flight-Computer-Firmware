#!/usr/bin/env bash
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

python3 "$repo_root/test/framework/tools/trace_report.py" \
    --search-root "$repo_root" \
    --req-format '^\s*(RQ\.FC-SYS\.[0-9]{5})(?:\s+\([^)]*\))?\s+-\s+(.+?)\s*$' \
    --req-format '^\s*(RQ\.FC-SW\.[0-9]{5})(?:\s+\([^)]*\))?\s+-\s+(.+?)\s*$' \
    --spec-files '^reqs/SysRD\.md$' \
    --spec-files '^reqs/SRD\.md$' \
    --results-files '^test/app/(?:[^/]+/)*(results\.txt|[^/]*results\.md)$'