"""
Build a regex from a .doorstop.yml file's digits, prefix, and sep.
Usage: python req_tracer.py <path_to_doorstop.yml> [other args...]
"""

import re
import sys
from pathlib import Path

# Directories skipped when scanning for implementation files (speeds up run).
EXCLUDE_IMPL_DIRS = frozenset({"test", "reqs", ".git", "auto", "__pycache__", "node_modules", ".venv"})
# Files never searched for requirement IDs (e.g. our own report).
EXCLUDE_FILES = frozenset({"req_traceability_report.html"})


def load_doorstop_config(path: str | Path) -> dict:
    """Load and parse a .doorstop.yml file. Returns the settings dict."""
    try:
        import yaml
    except ImportError:
        raise ImportError("PyYAML is required. Install with: pip install pyyaml")

    path = Path(path)
    if not path.exists():
        raise FileNotFoundError(f"No such file: {path}")

    with open(path, encoding="utf-8") as f:
        data = yaml.safe_load(f)

    if not data or "settings" not in data:
        raise ValueError(f"Invalid .doorstop.yml: missing 'settings' in {path}")

    return data["settings"]


def build_id_regex(path: str | Path) -> re.Pattern[str]:
    """
    Read a .doorstop.yml file and build a regex that matches requirement IDs
    of the form: prefix + sep + exactly `digits` numeric digits.

    Example: digits=7, prefix=APPA, sep='-Req-' matches "APPA-Req-0000001".

    Returns a compiled regex pattern.
    """
    settings = load_doorstop_config(path)
    digits = settings.get("digits")
    prefix = settings.get("prefix")
    sep = settings.get("sep")

    if digits is None:
        raise ValueError("settings.digits is required in .doorstop.yml")
    if prefix is None:
        raise ValueError("settings.prefix is required in .doorstop.yml")
    if sep is None:
        raise ValueError("settings.sep is required in .doorstop.yml")

    digits = int(digits)
    prefix = str(prefix)
    sep = str(sep)

    pattern = re.escape(prefix) + re.escape(sep) + r"\d{" + str(digits) + r"}"
    return re.compile(pattern)


def find_normative_files(doorstop_path: str | Path) -> list[Path]:
    """
    Look through all YAML files in the directory containing the .doorstop.yml
    and return paths for those that have normative: true.

    Skips .doorstop.yml itself. Uses the same YAML loader as load_doorstop_config.
    """
    try:
        import yaml
    except ImportError:
        raise ImportError("PyYAML is required. Install with: pip install pyyaml")

    doorstop_path = Path(doorstop_path).resolve()
    if not doorstop_path.exists():
        raise FileNotFoundError(f"No such file: {doorstop_path}")

    req_dir = doorstop_path.parent
    normative_paths: list[Path] = []

    for p in req_dir.iterdir():
        if not p.is_file() or p.suffix.lower() not in (".yml", ".yaml") or p.name == ".doorstop.yml":
            continue
        try:
            with open(p, encoding="utf-8") as f:
                data = yaml.safe_load(f)
            if isinstance(data, dict) and data.get("normative") is True:
                normative_paths.append(p)
        except Exception:
            continue

    return sorted(normative_paths)


def _grep_directory(
    root: Path,
    search_text: str,
    *,
    exclude_dirs: set[str] | None = None,
    only_under_dirs: set[str] | None = None,
) -> bool:
    """
    Search for `search_text` in files under `root`.
    - If exclude_dirs is set, do not descend into any directory with that name.
    - If only_under_dirs is set, only consider paths under root/<name> for each name.
    Returns True if at least one file contains search_text (as substring).
    """
    exclude_dirs = exclude_dirs or set()
    found = False

    def _scan(p: Path) -> None:
        nonlocal found
        if found:
            return
        if p.is_file():
            try:
                raw = p.read_bytes()
                if b"\x00" in raw:
                    return
                text = raw.decode("utf-8", errors="replace")
                if search_text in text:
                    found = True
            except OSError:
                pass
            return
        if p.is_dir():
            if p.name in exclude_dirs:
                return
            for c in p.iterdir():
                _scan(c)

    if only_under_dirs:
        for name in only_under_dirs:
            d = root / name
            if d.is_dir():
                for item in d.rglob("*"):
                    if found:
                        return True
                    if item.is_file():
                        try:
                            raw = item.read_bytes()
                            if b"\x00" in raw:
                                continue
                            if search_text in raw.decode("utf-8", errors="replace"):
                                found = True
                        except OSError:
                            pass
        return found
    _scan(root)
    return found


def _grep_directory_first_match(
    root: Path,
    search_text: str,
    *,
    exclude_dirs: set[str] | None = None,
    only_under_dirs: set[str] | None = None,
) -> Path | None:
    """
    Same as _grep_directory but returns the path of the first file containing
    search_text, or None if not found.
    """
    exclude_dirs = exclude_dirs or set()
    first_match: Path | None = None

    def _scan(p: Path) -> None:
        nonlocal first_match
        if first_match is not None:
            return
        if p.is_file():
            try:
                raw = p.read_bytes()
                if b"\x00" in raw:
                    return
                text = raw.decode("utf-8", errors="replace")
                if search_text in text:
                    first_match = p
            except OSError:
                pass
            return
        if p.is_dir():
            if p.name in exclude_dirs:
                return
            for c in sorted(p.iterdir()):
                _scan(c)
                if first_match is not None:
                    return

    if only_under_dirs:
        for name in sorted(only_under_dirs):
            d = root / name
            if d.is_dir():
                for item in sorted(d.rglob("*")):
                    if first_match is not None:
                        return first_match
                    if item.is_file():
                        try:
                            raw = item.read_bytes()
                            if b"\x00" in raw:
                                continue
                            if search_text in raw.decode("utf-8", errors="replace"):
                                return item
                        except OSError:
                            pass
        return first_match
    _scan(root)
    return first_match


def _grep_directory_all_matches(
    root: Path,
    search_text: str,
    *,
    exclude_dirs: set[str] | None = None,
    only_under_dirs: set[str] | None = None,
) -> list[Path]:
    """
    Same as _grep_directory but returns a sorted list of all file paths that
    contain search_text (no duplicates).
    """
    exclude_dirs = exclude_dirs or set()
    matches: list[Path] = []
    seen: set[Path] = set()

    def _add(p: Path) -> None:
        r = p.resolve()
        if r not in seen:
            seen.add(r)
            matches.append(p)

    def _scan(p: Path) -> None:
        if p.is_file():
            if p.name in EXCLUDE_FILES:
                return
            try:
                raw = p.read_bytes()
                if b"\x00" in raw:
                    return
                text = raw.decode("utf-8", errors="replace")
                if search_text in text:
                    _add(p)
            except OSError:
                pass
            return
        if p.is_dir():
            if p.name in exclude_dirs:
                return
            for c in sorted(p.iterdir()):
                _scan(c)

    if only_under_dirs:
        for name in sorted(only_under_dirs):
            d = root / name
            if d.is_dir():
                for item in sorted(d.rglob("*")):
                    if item.is_file() and item.name not in EXCLUDE_FILES:
                        try:
                            raw = item.read_bytes()
                            if b"\x00" in raw:
                                continue
                            if search_text in raw.decode("utf-8", errors="replace"):
                                _add(item)
                        except OSError:
                            pass
        return sorted(matches, key=lambda x: x.resolve())
    _scan(root)
    return sorted(matches, key=lambda x: x.resolve())


def _grep_directory_all_matches_with_lines(
    root: Path,
    search_text: str,
    *,
    exclude_dirs: set[str] | None = None,
    only_under_dirs: set[str] | None = None,
) -> list[tuple[Path, int]]:
    """
    Like _grep_directory_all_matches but returns (path, line_no) for each
    occurrence (line numbers 1-based). Sorted by path then line number.
    """
    exclude_dirs = exclude_dirs or set()
    results: list[tuple[Path, int]] = []
    seen: set[tuple[Path, int]] = set()

    def _add(p: Path, line_no: int) -> None:
        r = (p.resolve(), line_no)
        if r not in seen:
            seen.add(r)
            results.append((p, line_no))

    def _scan_file(p: Path) -> None:
        if p.name in EXCLUDE_FILES:
            return
        try:
            raw = p.read_bytes()
            if b"\x00" in raw:
                return
            text = raw.decode("utf-8", errors="replace")
            for i, line in enumerate(text.splitlines(), start=1):
                if search_text in line:
                    _add(p, i)
        except OSError:
            pass

    def _scan(p: Path) -> None:
        if p.is_file():
            _scan_file(p)
            return
        if p.is_dir():
            if p.name in exclude_dirs:
                return
            for c in sorted(p.iterdir()):
                _scan(c)

    if only_under_dirs:
        for name in sorted(only_under_dirs):
            d = root / name
            if d.is_dir():
                for item in sorted(d.rglob("*")):
                    if item.is_file():
                        _scan_file(item)
        return sorted(results, key=lambda x: (x[0].resolve(), x[1]))
    _scan(root)
    return sorted(results, key=lambda x: (x[0].resolve(), x[1]))


def _build_regex_index(
    root: Path,
    pattern: re.Pattern[str],
    *,
    exclude_dirs: set[str] | frozenset[str] | None = None,
    only_under_dirs: set[str] | frozenset[str] | None = None,
) -> dict[str, list[tuple[Path, int]]]:
    """
    Scan the tree once and find every match of the requirement-ID regex.
    Returns req_id -> [(path, line_no), ...] (1-based line numbers, sorted).
    """
    exclude_dirs = exclude_dirs or set()
    index: dict[str, list[tuple[Path, int]]] = {}
    # Dedupe (path, line_no) per req_id
    seen_per_id: dict[str, set[tuple[Path, int]]] = {}

    def _add(req_id: str, p: Path, line_no: int) -> None:
        r = (p.resolve(), line_no)
        if req_id not in seen_per_id:
            seen_per_id[req_id] = set()
        if r not in seen_per_id[req_id]:
            seen_per_id[req_id].add(r)
            index.setdefault(req_id, []).append((p, line_no))

    def _scan_file(p: Path) -> None:
        if p.name in EXCLUDE_FILES:
            return
        try:
            raw = p.read_bytes()
            if b"\x00" in raw:
                return
            text = raw.decode("utf-8", errors="replace")
            for i, line in enumerate(text.splitlines(), start=1):
                for m in pattern.finditer(line):
                    req_id = m.group()
                    _add(req_id, p, i)
        except OSError:
            pass

    def _scan(p: Path) -> None:
        if p.is_file():
            _scan_file(p)
            return
        if p.is_dir():
            if p.name in exclude_dirs:
                return
            for c in sorted(p.iterdir()):
                _scan(c)

    if only_under_dirs:
        for name in sorted(only_under_dirs):
            d = root / name
            if d.is_dir():
                for item in sorted(d.rglob("*")):
                    if item.is_file():
                        _scan_file(item)
    else:
        _scan(root)

    for req_id in index:
        index[req_id] = sorted(index[req_id], key=lambda x: (x[0].resolve(), x[1]))
    return index


def _default_progress_callback(_req_id: str, _index: int, _total: int) -> None:
    pass


def compute_traceability(
    doorstop_path: str | Path,
    cwd: str | Path | None = None,
    progress_callback: None = None,
) -> list[dict]:
    """
    For each normative requirement in the .doorstop.yml directory:
    - Assert its filename (stem) matches the doorstop ID regex.
    - Find requirement file (normative .yml), implementation file (cwd excl. test/ reqs/), test file (test/).

    progress_callback(req_id, index, total) is called at the start of each requirement (0-based index).

    Also includes requirement IDs found in code/tests that are not in the normative
    list (req_file is None for those). All such rows are untraceable.

    Returns a list of dicts, each with keys:
      req_id, req_file (Path | None), impl_locations, test_locations.
    """
    doorstop_path = Path(doorstop_path).resolve()
    cwd = Path(cwd or ".").resolve()
    pattern = build_id_regex(doorstop_path)
    normative_paths = find_normative_files(doorstop_path)
    rows: list[dict] = []
    cb = progress_callback or _default_progress_callback
    total = len(normative_paths)

    # Single-pass index: scan impl and test trees once for all regex matches.
    impl_index = _build_regex_index(cwd, pattern, exclude_dirs=EXCLUDE_IMPL_DIRS)
    test_index = (
        _build_regex_index(cwd, pattern, only_under_dirs={"test"})
        if (cwd / "test").is_dir()
        else {}
    )

    normative_ids = set()
    for i, p in enumerate(normative_paths):
        stem = p.stem
        assert pattern.fullmatch(stem), (
            f"Normative file name must match requirement ID regex: {p.name!r} does not match {pattern.pattern!r}"
        )
        req_id = stem
        normative_ids.add(req_id)
        cb(req_id, i, total)
        impl_locations = impl_index.get(req_id, [])
        test_locations = test_index.get(req_id, [])
        rows.append({
            "req_id": req_id,
            "req_file": p,
            "impl_locations": impl_locations,
            "test_locations": test_locations,
        })

    # Requirements tagged in code/tests but not in the normative list are also untraceable.
    all_referenced = set(impl_index.keys()) | set(test_index.keys())
    referenced_but_not_normative = sorted(all_referenced - normative_ids)
    for req_id in referenced_but_not_normative:
        rows.append({
            "req_id": req_id,
            "req_file": None,
            "impl_locations": impl_index.get(req_id, []),
            "test_locations": test_index.get(req_id, []),
        })
    return rows


def write_traceability_html(
    rows: list[dict],
    output_path: str | Path,
    cwd: str | Path | None = None,
) -> None:
    """
    Write an HTML report with summary, untraceable section (collapsible, near top),
    and traceable section. Table columns: ID, Requirement file, Implementation file, Test file.
    Cells are green if present, red if missing (ID column has no color).
    """
    cwd = Path(cwd or ".").resolve()
    output_path = Path(output_path)

    def rel(p: Path) -> str:
        try:
            return p.resolve().relative_to(cwd).as_posix()
        except ValueError:
            return p.as_posix()

    # Traceable = normative requirement with both impl and test; rest (incl. referenced-but-not-normative) untraceable.
    traceable = [
        r for r in rows
        if r["req_file"] is not None and r["impl_locations"] and r["test_locations"]
    ]
    untraceable = [r for r in rows if r not in traceable]
    total = len(rows)
    n_traceable = len(traceable)
    n_untraceable = len(untraceable)
    n_normative = sum(1 for r in rows if r["req_file"] is not None)
    pct = (100.0 * n_traceable / n_normative) if n_normative else 0.0

    def html_escape(s: str) -> str:
        return (
            s.replace("&", "&amp;")
            .replace("<", "&lt;")
            .replace(">", "&gt;")
            .replace('"', "&quot;")
        )

    def cell(locations: list[tuple[Path, int]], present: bool) -> str:
        if not present or not locations:
            return '<td class="missing">(missing)</td>'
        title = html_escape("\n".join(f"{p}:{ln}" for p, ln in locations))
        items = "".join(f"<li>{html_escape(rel(p))}:{ln}</li>" for p, ln in locations)
        return f'<td class="present" title="{title}"><ul class="file-list">{items}</ul></td>'

    def table_body(rows_list: list[dict]) -> str:
        if not rows_list:
            return '<tr><td colspan="4">(none)</td></tr>'
        lines = []
        for r in rows_list:
            req_file = r["req_file"]
            impl_locations = r["impl_locations"]
            test_locations = r["test_locations"]
            impl_ok = bool(impl_locations)
            test_ok = bool(test_locations)
            if req_file is not None:
                req_cell = f'<td class="present" title="{req_file}">{html_escape(rel(req_file))}</td>'
            else:
                req_cell = '<td class="missing">(no normative requirement)</td>'
            lines.append(
                "<tr>"
                f'<td>{html_escape(r["req_id"])}</td>'
                + req_cell
                + cell(impl_locations, impl_ok)
                + cell(test_locations, test_ok)
                + "</tr>"
            )
        return "\n".join(lines)

    html = f"""<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1.0">
<title>Requirement Traceability Report</title>
<style>
  body {{ font-family: system-ui, sans-serif; margin: 1rem 2rem; background: #1a1a1a; color: #e0e0e0; }}
  h1 {{ font-size: 1.5rem; margin-bottom: 0.5rem; }}
  h2 {{ font-size: 1.15rem; margin-top: 1.25rem; margin-bottom: 0.5rem; }}
  .summary {{ margin-bottom: 1.5rem; padding: 0.75rem 1rem; background: #2a2a2a; border-radius: 6px; }}
  .summary p {{ margin: 0.25rem 0; }}
  table {{ border-collapse: collapse; width: 100%; margin-bottom: 0.5rem; }}
  th, td {{ border: 1px solid #444; padding: 0.4rem 0.6rem; text-align: left; }}
  th {{ background: #333; }}
  td.present {{ background: #1a3d1a; color: #a0e0a0; }}
  td.missing {{ background: #3d1a1a; color: #e0a0a0; }}
  details {{ margin-bottom: 1rem; }}
  details summary {{ cursor: pointer; font-weight: 600; padding: 0.3rem 0; }}
  .trace-bar {{ height: 1.25rem; background: #3d1a1a; border-radius: 4px; overflow: hidden; margin: 0.5rem 0; }}
  .trace-bar-fill {{ height: 100%; background: #1a3d1a; border-radius: 4px; transition: width 0.2s; }}
  .file-list {{ margin: 0; padding-left: 1.2rem; }}
  .file-list li {{ margin: 0.15rem 0; }}
</style>
</head>
<body>
<h1>Requirement Traceability Report</h1>
<div class="summary">
  <p><strong>Summary</strong></p>
  <p>Traceable requirements: <strong>{n_traceable}</strong></p>
  <p>Untraceable requirements: <strong>{n_untraceable}</strong></p>
  <p>Normative requirements: <strong>{n_normative}</strong></p>
  <p>Referenced in code/tests but not normative: <strong>{total - n_normative}</strong></p>
  <p>Percentage traceable (of normative): <strong>{pct:.1f}%</strong></p>
  <div class="trace-bar" title="{pct:.1f}%">
    <div class="trace-bar-fill" style="width: {pct:.1f}%"></div>
  </div>
</div>

<h2>Untraceable requirements</h2>
<details open>
<summary>Untraceable ({n_untraceable}) — click to collapse/expand</summary>
<table>
<thead><tr><th>ID</th><th>Requirement file</th><th>Implementation file</th><th>Test file</th></tr></thead>
<tbody>
{table_body(untraceable)}
</tbody>
</table>
</details>

<h2>Traceable requirements</h2>
<details>
<summary>Traceable ({n_traceable}) — click to collapse/expand</summary>
<table>
<thead><tr><th>ID</th><th>Requirement file</th><th>Implementation file</th><th>Test file</th></tr></thead>
<tbody>
{table_body(traceable)}
</tbody>
</table>
</details>
</body>
</html>
"""
    output_path.write_text(html, encoding="utf-8")


def main(*args: str) -> re.Pattern[str] | None:
    """
    Entry point with variadic arguments.
    First argument must be the path to a .doorstop.yml file.
    Returns the compiled regex, or None if no path given.
    """
    if not args:
        return None
    doorstop_path = args[0]
    return build_id_regex(doorstop_path)


if __name__ == "__main__":
    argv = sys.argv[1:]
    if not argv:
        print("Usage: python req_tracer.py <path_to_.doorstop.yml> [output.html]", file=sys.stderr)
        sys.exit(1)

    try:
        doorstop_path = argv[0]
        cwd = Path.cwd()
        pattern = main(*argv)
        if pattern is not None:
            print("Regex:", pattern.pattern)
        normative = find_normative_files(doorstop_path)
        if normative:
            print("Normative files:")
            for p in normative:
                print(f"  {p}")
        else:
            print("Normative files: (none)")

        bar_width = 24
        last_bar_len = [0]  # use list so callback can mutate

        def progress_cb(req_id: str, index: int, total: int) -> None:
            n = total or 1
            filled = int(bar_width * (index + 1) / n)
            bar = "=" * filled + "-" * (bar_width - filled)
            msg = f"  [{bar}]  {index + 1}/{total}  {req_id}"
            pad = max(0, last_bar_len[0] - len(msg))
            sys.stderr.write(f"\r{msg}{' ' * pad}\r")
            sys.stderr.flush()
            last_bar_len[0] = len(msg)

        rows = compute_traceability(doorstop_path, cwd=cwd, progress_callback=progress_cb)
        sys.stderr.write("\r" + " " * (last_bar_len[0] or 60) + "\r")
        sys.stderr.flush()
        traceable = [
            r for r in rows
            if r["req_file"] is not None and r["impl_locations"] and r["test_locations"]
        ]
        untraceable = [r for r in rows if r not in traceable]

        print()
        print("Traceable requirements (normative with impl + test):")
        for r in traceable:
            print(f"  {r['req_id']}")
        if not traceable:
            print("  (none)")
        print()
        print("Untraceable requirements:")
        for r in untraceable:
            if r["req_file"] is None:
                print(f"  {r['req_id']}  referenced in code/tests but not in normative list")
            else:
                missing = []
                if not r["impl_locations"]:
                    missing.append("cwd (excluding test/ and reqs/)")
                if not r["test_locations"]:
                    missing.append("test/")
                print(f"  {r['req_id']}  missing: {', '.join(missing)}")
        if not untraceable:
            print("  (none)")

        html_path = argv[1] if len(argv) > 1 else cwd / "req_traceability_report.html"
        write_traceability_html(rows, html_path, cwd=cwd)
        print()
        print(f"HTML report written to: {html_path}")
        if untraceable:
            sys.exit(1)
    except Exception as e:
        print(f"Error: {e}", file=sys.stderr)
        sys.exit(1)
