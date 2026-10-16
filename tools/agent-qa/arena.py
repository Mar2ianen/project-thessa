#!/usr/bin/env python3
"""Prepare and score blind A/B comparisons of actual Thessa/reference PNG frames."""
from __future__ import annotations

import argparse
import json
import random
import struct
import zlib
import tomllib
from collections import defaultdict
from pathlib import Path

PNG_MAGIC = b"\x89PNG\r\n\x1a\n"
RUBRIC = """# Blind visual review

Judge images **without trying to identify their source**. Assess the matched
scene, not the reputation of the application. Do not access the answer key.

Record one vote per case and reviewer with `arena.py vote`.

Required fields for each case (in reviewer notes):
- **Preference:** A, B or tie.
- **Lighting/atmosphere:** which looks better, and why?
- **Geometry/LOD:** cracks, pop-in, silhouette quality, distant detail.
- **Artifacts:** temporal noise, clipping, aliasing, mismatched shadows.
- **Physical plausibility:** evaluated separately from aesthetic preference.
- **Confidence:** low / medium / high; note mismatched framing or quality.

If camera, FOV, lighting, exposure, HUD, resolution or scene content is not
comparable, mark the case invalid instead of inferring renderer quality.
The paired comparison is subjective evidence, not a physics or performance test.
"""


def png_size(path: Path) -> tuple[int, int]:
    with path.open("rb") as stream:
        head = stream.read(24)
    if len(head) != 24 or head[:8] != PNG_MAGIC or head[12:16] != b"IHDR":
        raise ValueError(f"Not a PNG with an IHDR header: {path}")
    width, height = struct.unpack(">II", head[16:24])
    if width == 0 or height == 0:
        raise ValueError(f"Invalid PNG dimensions: {path}")
    return width, height


def sanitized_png(source: Path, destination: Path) -> None:
    # WHY: PNG text/EXIF metadata can disclose an allegedly blind frame's origin.
    # Preserve render-affecting chunks but discard descriptive and unknown ancillary chunks.
    keep_ancillary = {b"cHRM", b"gAMA", b"sRGB", b"iCCP", b"sBIT", b"pHYs", b"bKGD", b"tRNS"}
    raw = source.read_bytes()
    if not raw.startswith(PNG_MAGIC):
        raise ValueError(f"Invalid PNG signature: {source}")
    pos = len(PNG_MAGIC)
    output = bytearray(PNG_MAGIC)
    saw_end = False
    while pos + 12 <= len(raw):
        size = struct.unpack_from(">I", raw, pos)[0]
        if size > len(raw) - pos - 12:
            raise ValueError(f"Truncated PNG chunk: {source}")
        kind = raw[pos + 4:pos + 8]
        chunk = raw[pos + 8:pos + 8 + size]
        checksum = struct.unpack_from(">I", raw, pos + 8 + size)[0]
        if zlib.crc32(kind + chunk) != checksum:
            raise ValueError(f"Bad PNG checksum: {source}")
        # Critical chunks must survive; unknown critical chunks make decoding unsafe.
        if kind in {b"IHDR", b"PLTE", b"IDAT", b"IEND"} or kind in keep_ancillary:
            output.extend(raw[pos:pos + 12 + size])
        elif not (kind[0] & 0x20):
            raise ValueError(f"Unsupported critical PNG chunk: {kind!r}")
        pos += 12 + size
        if kind == b"IEND":
            saw_end = True
            break
    if not saw_end or pos != len(raw):
        raise ValueError(f"PNG missing IEND or has trailing data: {source}")
    destination.write_bytes(output)


def contained(path: Path, other: Path) -> bool:
    return path == other or path in other.parents


def prepare(manifest: Path, blind: Path, private: Path, seed: int | None) -> None:
    blind, private = blind.resolve(), private.resolve()
    if contained(blind, private) or contained(private, blind):
        raise ValueError("Blind output and private key directories must not overlap")
    if blind.exists() or private.exists():
        raise FileExistsError("Output directories must not exist (avoid overwriting votes or keys)")
    with manifest.open("rb") as stream:
        data = tomllib.load(stream)
    scenes = data.get("scene")
    if not isinstance(scenes, list) or not scenes:
        raise ValueError("Manifest must contain at least one [[scene]]")
    rng = random.Random(seed) if seed is not None else random.SystemRandom()
    cases = []
    keys = []
    names = set()
    sources: list[tuple[Path, Path]] = []
    for item in scenes:
        case_id = item["id"]
        if not isinstance(case_id, str) or not case_id or not all(
            c.isascii() and (c.isalnum() or c in "_-") for c in case_id
        ) or case_id in names:
            raise ValueError(f"Invalid or duplicate scene id: {case_id!r}")
        names.add(case_id)
        first = (manifest.parent / item["first"]).resolve()
        second = (manifest.parent / item["second"]).resolve()
        if first == second or png_size(first) != png_size(second):
            raise ValueError(f"Scene {case_id} has identical files or mismatched PNG sizes")
        labels = (item["first_label"], item["second_label"])
        if not all(isinstance(label, str) and label.strip() for label in labels):
            raise ValueError("Every frame requires a non-empty provenance label")
        swap = bool(rng.getrandbits(1))
        a_source, b_source = (second, first) if swap else (first, second)
        keys.append({"id": case_id, "A": labels[1] if swap else labels[0],
                     "B": labels[0] if swap else labels[1]})
        cases.append({"id": case_id, "A": f"{case_id}/A.png",
                      "B": f"{case_id}/B.png", "size": list(png_size(first))})
        sources.append((a_source, b_source))
    blind.mkdir(parents=True)
    private.mkdir(parents=True)
    for case, (a_source, b_source) in zip(cases, sources):
        folder = blind / case["id"]
        folder.mkdir()
        sanitized_png(a_source, folder / "A.png")
        sanitized_png(b_source, folder / "B.png")
    (blind / "cases.json").write_text(json.dumps(cases, indent=2) + "\n")
    (blind / "REVIEW.md").write_text(RUBRIC)
    (private / "answer-key.json").write_text(json.dumps(keys, indent=2) + "\n")
    print(f"Prepared {len(cases)} blind cases in {blind}; private key in {private}")


def vote(blind: Path, case_id: str, judge: str, choice: str, notes: str) -> None:
    if not judge.strip():
        raise ValueError("Reviewer id cannot be empty")
    cases = json.loads((blind / "cases.json").read_text())
    if case_id not in {case["id"] for case in cases}:
        raise ValueError(f"Unknown case: {case_id}")
    path = blind / "votes.jsonl"
    if path.exists():
        for line in path.read_text().splitlines():
            past = json.loads(line)
            if past["id"] == case_id and past["judge"] == judge:
                raise ValueError("One vote per reviewer per case; use a new reviewer id for a rerun")
    with path.open("a", encoding="utf-8") as stream:
        stream.write(json.dumps({"id": case_id, "judge": judge, "choice": choice,
                                 "notes": notes}) + "\n")


def score(blind: Path, key: Path) -> None:
    cases = json.loads((blind / "cases.json").read_text())
    answers = json.loads(key.read_text())
    if {x["id"] for x in answers} != {x["id"] for x in cases}:
        raise ValueError("Answer key does not match the blind run")
    mapping = {x["id"]: x for x in answers}
    tally: dict[str, float] = defaultdict(float)
    counts: dict[str, int] = defaultdict(int)
    vote_path = blind / "votes.jsonl"
    if not vote_path.exists():
        raise ValueError("No votes recorded")
    seen: set[tuple[str, str]] = set()
    for line in vote_path.read_text().splitlines():
        item = json.loads(line)
        name, judge, choice = item["id"], item["judge"], item["choice"]
        if name not in mapping or choice not in ("A", "B", "tie", "invalid"):
            raise ValueError("Unknown case or vote choice")
        pair = (name, judge)
        if pair in seen:
            raise ValueError("Duplicate reviewer vote")
        seen.add(pair)
        if choice == "invalid":
            continue
        labels = mapping[name]
        counts[name] += 1
        if choice == "tie":
            tally[labels["A"]] += 0.5
            tally[labels["B"]] += 0.5
        else:
            tally[labels[choice]] += 1
    print(json.dumps({"valid_votes_by_case": counts, "preference_points": tally,
                      "note": "Pairwise preference only, not renderer correctness or FPS"}, indent=2))


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest="command", required=True)
    prep = sub.add_parser("prepare")
    prep.add_argument("manifest", type=Path)
    prep.add_argument("--blind", type=Path, required=True)
    prep.add_argument("--private", type=Path, required=True)
    prep.add_argument("--seed", type=int, help="For repeatable tests, not for blinded judges")
    voting = sub.add_parser("vote")
    voting.add_argument("--blind", type=Path, required=True)
    voting.add_argument("--id", required=True)
    voting.add_argument("--judge", required=True)
    voting.add_argument("--choice", choices=["A", "B", "tie", "invalid"], required=True)
    voting.add_argument("--notes", default="")
    scoring = sub.add_parser("score")
    scoring.add_argument("--blind", type=Path, required=True)
    scoring.add_argument("--key", type=Path, required=True)
    args = parser.parse_args()
    try:
        if args.command == "prepare":
            prepare(args.manifest, args.blind, args.private, args.seed)
        elif args.command == "vote":
            vote(args.blind, args.id, args.judge, args.choice, args.notes)
        else:
            score(args.blind, args.key)
    except (OSError, KeyError, ValueError, tomllib.TOMLDecodeError) as exc:
        parser.error(str(exc))


if __name__ == "__main__":
    main()
