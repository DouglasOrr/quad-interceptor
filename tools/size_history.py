#!/usr/bin/env python3

"""Scan commits, build build/release/game, and report its size as jsonlines.

Usage: size_history.py [--to <commit>] > out.jsonl

Iterates every commit from START (inclusive) up to --to (default HEAD),
in chronological order. For each commit it checks the commit out (detached),
runs `./dev -v release build game`, and records the resulting binary size.

Output (jsonlines) fields:
  date:   ISO date of the commit
  commit: full commit hash
  size:   size of build/release/game in bytes
  error:  "build" (the dev build failed) or "file_missing" (no game binary)
"""

import argparse
import json
import subprocess
import sys
from pathlib import Path

FROM = "e950a8fdeabe666c837ff2158f4b6b01d1d8a236"
ROOT = Path(__file__).resolve().parent.parent
GAME = ROOT / "build" / "release" / "game"


def sh(*cmd: str) -> tuple[int, str]:
    proc = subprocess.run(cmd, cwd=ROOT, capture_output=True, text=True)
    return proc.returncode, proc.stdout + proc.stderr


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--from", dest="from_", default=FROM)
    parser.add_argument("--to", default="HEAD")
    args = parser.parse_args()

    commits = sh("git", "rev-list", "--reverse", f"{args.from_}..{args.to}")[1].split()
    if not commits:
        print("no commits in range", file=sys.stderr)
        return 1

    for commit in commits:
        if sh("git", "checkout", "-q", commit)[0]:
            print(f"checkout failed for {commit}", file=sys.stderr)
            continue
        iso_date = sh("git", "show", "-s", "--format=%cs", commit)[1].strip()
        record: dict = {"date": iso_date, "commit": commit}
        # Always clean-build: the shared build dir persists across checkouts, so
        # a stale (ninja mtime) or orphaned (.o) build would give wrong sizes.
        code, _ = sh("./dev", "-v", "release", "clean")
        if code != 0:
            record["error"] = "clean"
        else:
            code, _ = sh("./dev", "-v", "release", "build", "game")
            if code != 0:
                record["error"] = "build"
            elif GAME.exists():
                record["size"] = GAME.stat().st_size
            else:
                record["error"] = "file_missing"
        print(json.dumps(record), flush=True)

    sh("git", "checkout", "-q", args.to)
    return 0


if __name__ == "__main__":
    sys.exit(main())
