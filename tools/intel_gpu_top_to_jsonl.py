#!/usr/bin/env python3
"""Convert `intel_gpu_top -J` capture to jsonlines for the game client.

Reads the concatenated (and possibly truncated) JSON stream that
`intel_gpu_top -J -o <file>` produces and writes one object per line:

    {"dt": 0.216, "freq_MHz": 225.0, "busy": 0.284}

  dt       -- period duration in seconds
  freq_MHz -- actual GPU frequency
  power_W  -- GPU power in watts
  busy     -- game client's Render/3D busy, as a fraction 0..1

The "busy" key is omitted when the game client is absent. Usage:

    python3 tools/gpu_top_to_jsonl.py gpu.json > gpu.jsonl
"""

import argparse
import json
import sys


def iter_records(stream: str):
    decoder = json.JSONDecoder()
    text = stream.lstrip()
    if text.startswith("["):
        text = text[1:].lstrip()
    while text:
        if text[0] == ",":
            text = text[1:].lstrip()
            continue
        if text[0] == "]":
            return
        try:
            obj, end = decoder.raw_decode(text)
        except json.JSONDecodeError:
            return  # truncated final record; stop cleanly
        yield obj
        text = text[end:].lstrip()


def game_render_busy(record):
    for client in record.get("clients", {}).values():
        if client.get("name") != "game":
            continue
        render = (client.get("engine-classes") or {}).get("Render/3D")
        busy = (render or {}).get("busy")
        return float(busy) / 100.0 if busy is not None else None
    return None


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("input", nargs="?", default="-", help="capture file, or '-' for stdin")
    args = parser.parse_args()

    stream = sys.stdin.read() if args.input == "-" else open(args.input).read()
    for record in iter_records(stream):
        sample = {
            "dt": record["period"]["duration"] / 1000.0,
            "freq_MHz": record["frequency"]["actual"],
            "power_W": record["power"]["GPU"],
        }
        busy = game_render_busy(record)
        if busy is not None:
            sample["busy"] = busy
        print(json.dumps(sample))


if __name__ == "__main__":
    main()
