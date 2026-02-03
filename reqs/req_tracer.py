"""
Build a regex from a .doorstop.yml file's digits, prefix, and sep.
Usage: python req_tracer.py <path_to_doorstop.yml> [other args...]
"""

import re
import sys
from pathlib import Path


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


def compute_traceability(
    doorstop_path: str | Path,
    cwd: str | Path | None = None,
) -> list[dict]:
    """
    For each normative requirement in the .doorstop.yml directory:
    - Assert its filename (stem) matches the doorstop ID regex.
    - Find requirement file (normative .yml), implementation file (cwd excl. test/ reqs/), test file (test/).

    Returns a list of dicts, each with keys:
      req_id, req_file (Path), impl_file (Path | None), test_file (Path | None).
    """
    doorstop_path = Path(doorstop_path).resolve()
    cwd = Path(cwd or ".").resolve()
    pattern = build_id_regex(doorstop_path)
    normative_paths = find_normative_files(doorstop_path)
    rows: list[dict] = []

    for p in normative_paths:
        stem = p.stem
        assert pattern.fullmatch(stem), (
            f"Normative file name must match requirement ID regex: {p.name!r} does not match {pattern.pattern!r}"
        )
        req_id = stem
        impl_file = _grep_directory_first_match(cwd, req_id, exclude_dirs={"test", "reqs"})
        test_file = (
            _grep_directory_first_match(cwd, req_id, only_under_dirs={"test"})
            if (cwd / "test").is_dir()
            else None
        )
        rows.append({
            "req_id": req_id,
            "req_file": p,
            "impl_file": impl_file,
            "test_file": test_file,
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

    traceable = [r for r in rows if r["impl_file"] is not None and r["test_file"] is not None]
    untraceable = [r for r in rows if r not in traceable]
    total = len(rows)
    n_traceable = len(traceable)
    n_untraceable = len(untraceable)
    pct = (100.0 * n_traceable / total) if total else 0.0

    def cell(path: Path | None, present: bool) -> str:
        if present and path is not None:
            return f'<td class="present" title="{path}">{html_escape(rel(path))}</td>'
        return '<td class="missing">(missing)</td>'

    def html_escape(s: str) -> str:
        return (
            s.replace("&", "&amp;")
            .replace("<", "&lt;")
            .replace(">", "&gt;")
            .replace('"', "&quot;")
        )

    def table_body(rows_list: list[dict]) -> str:
        if not rows_list:
            return '<tr><td colspan="4">(none)</td></tr>'
        lines = []
        for r in rows_list:
            req_file = r["req_file"]
            impl_file = r["impl_file"]
            test_file = r["test_file"]
            impl_ok = impl_file is not None
            test_ok = test_file is not None
            lines.append(
                "<tr>"
                f'<td>{html_escape(r["req_id"])}</td>'
                f'<td class="present" title="{req_file}">{html_escape(rel(req_file))}</td>'
                + cell(impl_file, impl_ok)
                + cell(test_file, test_ok)
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
</style>
</head>
<body>
<h1>Requirement Traceability Report</h1>
<div class="summary">
  <p><strong>Summary</strong></p>
  <p>Traceable requirements: <strong>{n_traceable}</strong></p>
  <p>Untraceable requirements: <strong>{n_untraceable}</strong></p>
  <p>Total normative requirements: <strong>{total}</strong></p>
  <p>Percentage traceable: <strong>{pct:.1f}%</strong></p>
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

        rows = compute_traceability(doorstop_path, cwd=cwd)
        traceable = [r for r in rows if r["impl_file"] is not None and r["test_file"] is not None]
        untraceable = [r for r in rows if r not in traceable]

        print()
        print("Traceable requirements (found in cwd, test/, and reqs):")
        for r in traceable:
            print(f"  {r['req_id']}")
        if not traceable:
            print("  (none)")
        print()
        print("Untraceable requirements (missing one or more of the three):")
        for r in untraceable:
            missing = []
            if r["impl_file"] is None:
                missing.append("cwd (excluding test/ and reqs/)")
            if r["test_file"] is None:
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
