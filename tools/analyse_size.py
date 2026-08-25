#!/usr/bin/env python3

"""Compare the release binary between two commits and explain the size delta.

Usage: analyze_size.py <to_commit> [--from <from_commit>] [--clone-dir <dir>]

If --from is omitted it defaults to the parent of <to_commit>. Builds each
commit in a clone under ./agents/analyse_size/ (or --clone-dir) — the
from-commit in "from", the to-commit in "to" — then reports:
  - overall binary sizes and the delta between them
  - per-translation-unit object-file (.o) size deltas
  - per-section (text/rodata/plt/eh_frame/...) size deltas

This is the data used to attribute binary growth to a commit. See
docs/size_notes.md for worked examples.

Output is a plain table, one line per item, sorted by absolute delta.
"""

import argparse
import re
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
DEFAULT_CLONE_DIR = ROOT / "agents" / "analyse_size"
THIRD_PARTY = ROOT / "third-party"


def sh(*cmd: str, cwd: Path | None = None) -> str:
    proc = subprocess.run(cmd, cwd=cwd, capture_output=True, text=True)
    if proc.returncode != 0:
        raise RuntimeError(
            f"command failed ({' '.join(cmd)}):\n{proc.stdout}{proc.stderr}"
        )
    return proc.stdout


def build(name: str, commit: str) -> Path:
    clone = CLONE_DIR / name
    if not clone.exists():
        clone.parent.mkdir(parents=True, exist_ok=True)
        sh("git", "-C", str(ROOT), "clone", str(ROOT), str(clone))
    sh("git", "-C", str(clone), "checkout", "-q", "--detach", commit)
    # third-party isn't tracked by git; symlink it so ./dev can build.
    link = clone / "third-party"
    if not link.exists() and not link.is_symlink():
        link.symlink_to(THIRD_PARTY, target_is_directory=True)
    # Reusing a build dir across different commits is unsafe: ninja's mtime
    # checks won't reliably rebuild when switching revisions, so we force a
    # clean build whenever the clone's last-built commit changes.
    build_dir = clone / "build" / "release"
    marker = build_dir / ".built_commit"
    last = marker.read_text().strip() if marker.exists() else ""
    if last != commit:
        sh("./dev", "-v", "release", "clean", cwd=clone)
        sh("./dev", "-v", "release", "build", "game", cwd=clone)
        marker.write_text(commit)
    else:
        sh("./dev", "-v", "release", "build", "game", cwd=clone)
    return build_dir / "game"


def object_sizes(build_dir: Path) -> dict[str, int]:
    sizes: dict[str, int] = {}
    for obj in (build_dir / "obj").glob("*.o"):
        sizes[obj.name] = obj.stat().st_size
    return sizes


def section_sizes(binary: Path) -> dict[str, int]:
    out = sh("readelf", "-SW", str(binary))
    sections: dict[str, int] = {}
    for line in out.splitlines():
        m = re.match(
            r"\s*\[\s*\d+\]\s+(\S+)\s+(PROGBITS|NOBITS)\s+\S+\s+\S+\s+(\S+)\s+", line
        )
        if m:
            sections[m.group(1)] = int(m.group(3), 16)
    return sections


def report(title: str, new: dict[str, int], old: dict[str, int]) -> None:
    names = sorted(
        set(new) | set(old),
        key=lambda n: abs(new.get(n, 0) - old.get(n, 0)),
        reverse=True,
    )
    print(f"== {title} ==")
    for name in names:
        d = new.get(name, 0) - old.get(name, 0)
        print(f"{d:+8d}  {name:20} new={new.get(name, 0):7} old={old.get(name, 0):7}")
    print()


def main() -> int:
    global CLONE_DIR
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("to_commit", help="commit under investigation")
    parser.add_argument(
        "--from",
        dest="from_commit",
        help="baseline commit (older); defaults to the parent of <to_commit>",
    )
    parser.add_argument(
        "--clone-dir",
        default=str(DEFAULT_CLONE_DIR),
        help=f"directory for the from/to clones (default: {DEFAULT_CLONE_DIR})",
    )
    args = parser.parse_args()
    CLONE_DIR = Path(args.clone_dir)

    if args.from_commit is None:
        args.from_commit = sh(
            "git", "-C", str(ROOT), "rev-parse", f"{args.to_commit}~1"
        ).strip()

    new_bin = build("to", args.to_commit)
    old_bin = build("from", args.from_commit)

    new_size = new_bin.stat().st_size
    old_size = old_bin.stat().st_size
    print(f"game binary: {old_size} -> {new_size}  (delta {new_size - old_size:+d})")
    print(f"  from build: {old_bin.parent}")
    print(f"  to   build: {new_bin.parent}")
    print()

    report(
        "object files (.o)", object_sizes(new_bin.parent), object_sizes(old_bin.parent)
    )
    report("binary sections", section_sizes(new_bin), section_sizes(old_bin))
    return 0


if __name__ == "__main__":
    sys.exit(main())
