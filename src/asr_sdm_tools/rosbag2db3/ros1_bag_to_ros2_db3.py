#!/usr/bin/env python3
"""
Convert a ROS1 .bag into a ROS2 rosbag2 (SQLite3 .db3) bag directory.

This script is a thin wrapper around the `rosbags` project's converter:
  - `python -m rosbags.convert ...`

Why this approach:
  - Pure Python (no ROS1/ROS2 install required)
  - Handles ROS1 serialization -> ROS2 CDR serialization correctly
  - Produces a standard ROS2 bag directory containing `metadata.yaml` and `*.db3`
"""

from __future__ import annotations

import argparse
import shutil
import subprocess
import sys
from pathlib import Path


def _default_dst_from_src(src: Path) -> Path:
    # ROS2 bags are directories. Keep it obvious and non-destructive.
    # e.g. foo.bag -> foo_ros2
    stem = src.name
    if stem.endswith(".bag"):
        stem = stem[: -len(".bag")]
    return src.with_name(f"{stem}_ros2")


def _parse_args() -> argparse.Namespace:
    ws_default_src = Path(__file__).resolve().parents[1] / "MH_01_easy.bag"

    p = argparse.ArgumentParser(
        description="Convert a ROS1 .bag into a ROS2 rosbag2 SQLite3 (.db3) bag directory (via rosbags)."
    )
    p.add_argument(
        "--src",
        type=Path,
        default=ws_default_src,
        help="Path to input ROS1 .bag file.",
    )
    p.add_argument(
        "--dst",
        type=Path,
        default=None,
        help="Output ROS2 bag directory. If omitted, uses <src>_ros2 next to the source.",
    )
    p.add_argument(
        "--overwrite",
        action="store_true",
        help="If set, delete --dst if it already exists.",
    )
    p.add_argument(
        "--dst-typestore",
        default=None,
        help=(
            "Optional rosbags typestore for the destination (e.g. ros2_humble, ros2_iron, ros2_jazzy). "
            "If omitted, rosbags will copy message definitions from the source."
        ),
    )
    p.add_argument(
        "--include-topic",
        nargs="*",
        default=None,
        help="Only include these topics (exact match).",
    )
    p.add_argument(
        "--exclude-topic",
        nargs="*",
        default=None,
        help="Exclude these topics (exact match). Exclusions take precedence.",
    )
    return p.parse_args()


def main() -> int:
    args = _parse_args()

    src: Path = args.src.expanduser().resolve()
    if not src.exists():
        print(f"ERROR: --src does not exist: {src}", file=sys.stderr)
        return 2
    if src.suffix != ".bag":
        print(f"WARNING: --src does not end with .bag: {src}", file=sys.stderr)

    dst: Path = (args.dst if args.dst is not None else _default_dst_from_src(src)).expanduser().resolve()
    if dst.exists():
        if not args.overwrite:
            print(f"ERROR: --dst already exists (use --overwrite): {dst}", file=sys.stderr)
            return 2
        if dst.is_dir():
            shutil.rmtree(dst)
        else:
            dst.unlink()

    # Prefer the python module entrypoint. It is present whenever `rosbags` is installed.
    cmd = [
        sys.executable,
        "-m",
        "rosbags.convert",
        "--src",
        str(src),
        "--dst",
        str(dst),
        "--dst-storage",
        "sqlite3",
    ]

    if args.dst_typestore:
        cmd += ["--dst-typestore", args.dst_typestore]
    if args.include_topic:
        cmd += ["--include-topic", *args.include_topic]
    if args.exclude_topic:
        cmd += ["--exclude-topic", *args.exclude_topic]

    print("Running:")
    print("  " + " ".join(cmd))
    try:
        subprocess.run(cmd, check=True)
    except FileNotFoundError:
        print("ERROR: Python executable not found when launching subprocess.", file=sys.stderr)
        return 3
    except subprocess.CalledProcessError as e:
        print(f"ERROR: conversion failed with exit code {e.returncode}", file=sys.stderr)
        return e.returncode

    db3_files = sorted(dst.glob("*.db3"))
    print(f"\nDone. ROS2 bag directory:\n  {dst}")
    if db3_files:
        print("DB3 file(s):")
        for p in db3_files:
            print(f"  {p}")
    else:
        print("NOTE: No *.db3 files found in destination; check conversion output above.", file=sys.stderr)

    return 0


if __name__ == "__main__":
    raise SystemExit(main())


