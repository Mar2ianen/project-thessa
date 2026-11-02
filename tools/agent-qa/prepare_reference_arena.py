#!/usr/bin/env python3
"""Standardize and prepare blind visual arena reviews across reference scenes B01, B02, and B03.

Assembles paired PNG frames from native Thessa captures and external Cycles runs:
  - B01: PBR sphere grid roughness/metallic test (640x480)
  - B02: Masked vegetation fern cutout test (640x480)
  - B03 Procedural: Coastal survey photogrammetric ground test (1280x720)
  - B03 Golden: Coastal survey golden sunset photo-reference test (1280x720)

Usage:
  # Populate frames and write manifest:
  python3 tools/agent-qa/prepare_reference_arena.py

  # Directly prepare blind review package:
  python3 tools/agent-qa/prepare_reference_arena.py --blind /tmp/arena/review --private /tmp/arena/private --seed 42
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path
import shutil
import sys
from PIL import Image

sys.dont_write_bytecode = True
QA_DIR = Path(__file__).resolve().parent
ROOT = QA_DIR.parents[1]
sys.path.insert(0, str(QA_DIR))
import arena


def discover_default_dirs() -> dict[str, Path | None]:
    """Find recent standard capture / cycles run directories if present."""
    tmp = Path("/tmp")

    def find_dir(patterns: list[str]) -> Path | None:
        for p in patterns:
            matches = sorted(tmp.glob(p), key=lambda d: d.stat().st_mtime, reverse=True)
            for m in matches:
                if m.is_dir():
                    return m
        return None

    return {
        "b01": find_dir(["b01-cycles*", "*b01*cycles*"]),
        "b02": find_dir(["b02-cycles*", "*b02*cycles*"]),
        "b03_procedural": find_dir(["cycles_b03_procedural*", "*b03*procedural*"]),
        "b03_golden": find_dir(["cycles_golden_v*", "*b03*golden*"]),
    }


def populate_frames(
    frames_dir: Path,
    b01_dir: Path,
    b02_dir: Path,
    b03_proc_dir: Path,
    b03_gold_dir: Path,
) -> None:
    frames_dir.mkdir(parents=True, exist_ok=True)

    # 1. B01
    b01_thessa = b01_dir / "thessa.png"
    b01_cycles = b01_dir / "display.png"
    if not b01_thessa.exists() or not b01_cycles.exists():
        raise FileNotFoundError(f"B01 frames missing in {b01_dir}")
    shutil.copyfile(b01_thessa, frames_dir / "b01_thessa.png")
    shutil.copyfile(b01_cycles, frames_dir / "b01_cycles.png")

    # 2. B02
    b02_thessa = b02_dir / "thessa.png"
    b02_cycles = b02_dir / "display.png"
    if not b02_thessa.exists() or not b02_cycles.exists():
        raise FileNotFoundError(f"B02 frames missing in {b02_dir}")
    shutil.copyfile(b02_thessa, frames_dir / "b02_thessa.png")
    shutil.copyfile(b02_cycles, frames_dir / "b02_cycles.png")

    # 3. B03 Procedural
    b03_thessa = b03_proc_dir / "thessa_display.png"
    if not b03_thessa.exists():
        b03_thessa = ROOT / "data/reference-scenes/b03-canonical-coast/display.png"
    b03_cycles = b03_proc_dir / "cycles_display.png"
    if not b03_thessa.exists() or not b03_cycles.exists():
        raise FileNotFoundError(f"B03 procedural frames missing in {b03_proc_dir}")
    shutil.copyfile(b03_thessa, frames_dir / "b03_thessa.png")
    shutil.copyfile(b03_cycles, frames_dir / "b03_cycles.png")

    # 4. B03 Golden
    gold_ref = b03_gold_dir / "golden_reference.jpg"
    if not gold_ref.exists():
        gold_ref = ROOT / "data/reference-scenes/b03-canonical-coast/golden_reference.jpg"
    gold_cycles = b03_gold_dir / "cycles_display.png"
    if not gold_ref.exists() or not gold_cycles.exists():
        raise FileNotFoundError(f"B03 golden frames missing in {b03_gold_dir}")

    # Convert JPG reference to matched 1280x720 PNG
    ref_img = Image.open(gold_ref).resize((1280, 720), Image.Resampling.LANCZOS)
    ref_img.save(frames_dir / "b03_golden_ref.png")
    shutil.copyfile(gold_cycles, frames_dir / "b03_golden_cycles.png")

    # Verify PNG dimensions
    for name, w, h in [
        ("b01_thessa.png", 640, 480),
        ("b01_cycles.png", 640, 480),
        ("b02_thessa.png", 640, 480),
        ("b02_cycles.png", 640, 480),
        ("b03_thessa.png", 1280, 720),
        ("b03_cycles.png", 1280, 720),
        ("b03_golden_ref.png", 1280, 720),
        ("b03_golden_cycles.png", 1280, 720),
    ]:
        path = frames_dir / name
        size = arena.png_size(path)
        if size != (w, h):
            raise ValueError(f"Frame {name} extent mismatch: {size} != ({w}, {h})")


def write_manifest(manifest_path: Path, frames_rel_path: str = "frames") -> None:
    content = f"""# Canonical Reference Scenes Arena Manifest
# Paired blind comparisons between Thessa Native (Vulkan) and Blender Cycles 5.2.2 LTS.
# Prepared via tools/agent-qa/prepare_reference_arena.py

[[scene]]
id = "b01-material-lab"
first = "{frames_rel_path}/b01_thessa.png"
first_label = "Thessa Native (Vulkan)"
second = "{frames_rel_path}/b01_cycles.png"
second_label = "Blender Cycles 5.2.2 LTS"

[[scene]]
id = "b02-vegetation-cutout"
first = "{frames_rel_path}/b02_thessa.png"
first_label = "Thessa Native (Vulkan)"
second = "{frames_rel_path}/b02_cycles.png"
second_label = "Blender Cycles 5.2.2 LTS"

[[scene]]
id = "b03-canonical-coast"
first = "{frames_rel_path}/b03_thessa.png"
first_label = "Thessa Native (Vulkan)"
second = "{frames_rel_path}/b03_cycles.png"
second_label = "Blender Cycles 5.2.2 LTS"

"""
    manifest_path.parent.mkdir(parents=True, exist_ok=True)
    manifest_path.write_text(content, encoding="utf-8")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--b01", type=Path, help="Directory containing B01 capture & cycles results")
    parser.add_argument("--b02", type=Path, help="Directory containing B02 capture & cycles results")
    parser.add_argument("--b03-procedural", type=Path, help="Directory containing B03 procedural cycles results")
    parser.add_argument("--b03-golden", type=Path, help="Directory containing B03 golden cycles results")
    parser.add_argument(
        "--frames-dir",
        type=Path,
        default=ROOT / "data/reference-scenes/frames",
        help="Directory to populate paired review PNG frames",
    )
    parser.add_argument(
        "--manifest",
        type=Path,
        default=ROOT / "data/reference-scenes/arena-scenes.toml",
        help="Path to emit the arena manifest TOML",
    )
    parser.add_argument("--blind", type=Path, help="Prepare blind review package in this directory")
    parser.add_argument("--private", type=Path, help="Store private answer key in this directory")
    parser.add_argument("--seed", type=int, default=None, help="Random seed for deterministic A/B blinding")

    args = parser.parse_args()
    defaults = discover_default_dirs()

    b01 = args.b01 or defaults["b01"]
    b02 = args.b02 or defaults["b02"]
    b03_proc = args.b03_procedural or defaults["b03_procedural"]
    b03_gold = args.b03_golden or defaults["b03_golden"]

    missing = []
    if not b01 or not b01.is_dir():
        missing.append("b01")
    if not b02 or not b02.is_dir():
        missing.append("b02")
    if not b03_proc or not b03_proc.is_dir():
        missing.append("b03_procedural")
    if not b03_gold or not b03_gold.is_dir():
        missing.append("b03_golden")

    if missing:
        print(f"Error: could not resolve directories for: {', '.join(missing)}", file=sys.stderr)
        print("Please provide paths explicitly with --b01, --b02, --b03-procedural, --b03-golden.", file=sys.stderr)
        return 1

    print(f"B01 source:            {b01}")
    print(f"B02 source:            {b02}")
    print(f"B03 procedural source: {b03_proc}")
    print(f"B03 golden source:     {b03_gold}")

    frames_dir = args.frames_dir.resolve()
    manifest_path = args.manifest.resolve()
    populate_frames(frames_dir, b01, b02, b03_proc, b03_gold)
    print(f"Populated 8 frames in: {frames_dir}")

    # Compute relative path from manifest directory to frames directory
    try:
        rel_frames = str(frames_dir.relative_to(manifest_path.parent))
    except ValueError:
        rel_frames = str(frames_dir)

    write_manifest(manifest_path, rel_frames)
    print(f"Wrote manifest:        {manifest_path}")

    if args.blind:
        private = args.private or (args.blind.parent / "private-key")
        arena.prepare(manifest_path, args.blind, private, args.seed)
        print(f"Prepared blind review: {args.blind}")
        print(f"Private answer key in: {private}")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
