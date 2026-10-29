#!/usr/bin/env python3
"""Build and capture the native app with explicit source/input/output provenance."""
from __future__ import annotations
import argparse
import hashlib
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
import tomllib

sys.dont_write_bytecode = True
ROOT = Path(__file__).resolve().parents[2]
RESERVED = {"--headless", "--perf-capture", "--linear-hdr", "--frames", "--width", "--height", "--frame-sequence", "--help"}


def sha256(path: Path) -> str:
    with path.open("rb") as source:
        return hashlib.file_digest(source, "sha256").hexdigest()


def source_state(root: Path) -> dict:
    def git(*args):
        return subprocess.check_output(["git", *args], cwd=root)
    diff = git("diff", "--binary", "HEAD")
    untracked = git("ls-files", "--others", "--exclude-standard", "-z").decode().split("\0")
    return {"commit": git("rev-parse", "HEAD").decode().strip(),
            "dirty": bool(diff or any(untracked)),
            "tracked_diff_sha256": hashlib.sha256(diff).hexdigest(),
            "untracked_sha256": {p: sha256(root / p) for p in untracked if p}}


def load_manifest(path: Path, root: Path = ROOT) -> tuple[dict, list[dict]]:
    data = tomllib.loads(path.read_text())
    if data.get("schema_version") != 1 or not isinstance(data.get("id"), str) or not data["id"]:
        raise ValueError("manifest requires schema_version=1 and a nonempty id")
    for key, maximum in [("width", 4096), ("height", 4096), ("frames", 1800)]:
        if type(data.get(key)) is not int or not 1 <= data[key] <= maximum:
            raise ValueError(f"{key} must be an integer in 1..{maximum}")
    args = data.get("native_args", [])
    if not isinstance(args, list) or any(not isinstance(a, str) for a in args):
        raise ValueError("native_args must be a list of strings")
    if any(a.split("=", 1)[0] in RESERVED for a in args):
        raise ValueError("capture tool owns output paths, resolution and frame count")
    verified = []
    for kind in ("inputs", "assets"):
        for item in data.get(kind, []):
            source = (root / item["path"]).resolve()
            expected = item.get("sha256", "")
            if len(expected) != 64 or any(c not in "0123456789abcdef" for c in expected):
                raise ValueError(f"{source}: exact lowercase SHA-256 is required")
            if sha256(source) != expected:
                raise ValueError(f"{source}: checksum mismatch")
            if kind == "assets" and any(not item.get(k) or item[k] == "REQUIRED" for k in (
                    "id", "source", "license", "upstream_revision_or_version", "source_sha256",
                    "converted_sha256", "conversion_tool_version", "asset_format", "unit_scale",
                    "coordinate_convention", "texture_color_spaces", "geometry_lod", "license_notice")):
                raise ValueError(f"{source}: incomplete asset provenance")
            if kind == "assets":
                for field in ("source_sha256", "converted_sha256"):
                    digest = item[field]
                    if len(digest) != 64 or any(c not in "0123456789abcdef" for c in digest):
                        raise ValueError(f"{source}: invalid {field}")
                if item["converted_sha256"] != expected:
                    raise ValueError(f"{source}: converted checksum differs from the consumed file")
                item = {**item, "license_notice_sha256": sha256((root / item["license_notice"]).resolve())}
            verified.append({"kind": kind, **item, "resolved_path": str(source)})
    return data, verified


def validate_capture(data: dict, manifest: dict, label: str) -> None:
    meta = data["metadata"]
    if meta["build"]["commit"] != label:
        raise ValueError("compiled source label does not match the capture")
    if (meta["display"]["width"], meta["display"]["height"]) != (manifest["width"], manifest["height"]):
        raise ValueError("captured resolution differs from the manifest")
    if len(data["frames"]) != manifest["frames"]:
        raise ValueError("captured frame count differs from the manifest")
    values = meta["graphics"]["resolved"]
    for name, count in [("capture_world_from_view", 16), ("capture_clip_from_view", 16), ("capture_camera_origin_m", 3)]:
        numbers = json.loads(values[name])
        if len(numbers) != count or any(not isinstance(n, (int, float)) or not (-float("inf") < n < float("inf")) for n in numbers):
            raise ValueError(f"invalid camera metadata: {name}")
    for name in ("capture_exposure_gain", "capture_simulation_time_s", "capture_ambient_lux",
                 "capture_light_directions", "capture_light_colors_lux", "capture_projection_convention",
                 "capture_linear_hdr", "capture_warmup_frames", "requested_toml", "measurement_boundary"):
        if name not in values:
            raise ValueError(f"missing capture metadata: {name}")
    if values["capture_warmup_frames"] != "30":
        raise ValueError("unexpected native warmup contract")


def run(manifest_path: Path, output: Path, profile: str, allow_dirty: bool) -> None:
    manifest_path = manifest_path.resolve()
    output = output.resolve()
    if output == ROOT or ROOT in output.parents:
        raise ValueError("capture output must be outside the source checkout")
    manifest_hash = sha256(manifest_path)
    manifest, inputs = load_manifest(manifest_path)
    source = source_state(ROOT)
    if source["dirty"] and not allow_dirty:
        raise ValueError("source checkout is dirty; commit first or explicitly use --allow-dirty")
    label = source["commit"] + ("-dirty" if source["dirty"] else "")
    output.mkdir(parents=True, exist_ok=False)
    shutil.copyfile(manifest_path, output / "scene.toml")
    build = ["cargo", "build", "--offline", "-p", "thessa-terrain-native", "--message-format=json"]
    if profile == "release":
        build.append("--release")
    env = os.environ.copy()
    env["THESSA_BUILD_COMMIT"] = label
    with (output / "build.jsonl").open("w") as stdout, (output / "build.log").open("w") as stderr:
        subprocess.run(build, cwd=ROOT, env=env, stdout=stdout, stderr=stderr, check=True)
    artifacts = [json.loads(line) for line in (output / "build.jsonl").read_text().splitlines()]
    binaries = [a["executable"] for a in artifacts if a.get("executable") and a.get("target", {}).get("name") == "thessa-terrain-native"]
    if len(binaries) != 1:
        raise ValueError("expected exactly one native app executable")
    binary = Path(binaries[0])
    binary_hash = sha256(binary)
    command = [str(binary), *manifest.get("native_args", []), "--headless", str(output / "display.png"),
               "--perf-capture", str(output / "perf.json"), "--linear-hdr", str(output / "linear.rgba16f"),
               "--width", str(manifest["width"]), "--height", str(manifest["height"]),
               "--frames", str(manifest["frames"])]
    (output / "commands.json").write_text(json.dumps({"build": build, "capture": command}, indent=2) + "\n")
    with (output / "native.log").open("w") as log:
        subprocess.run(command, cwd=ROOT, env=env, stdout=log, stderr=subprocess.STDOUT, check=True)
    capture = json.loads((output / "perf.json").read_text())
    validate_capture(capture, manifest, label)
    from arena import png_size, sanitized_png
    # Reuse the arena CRC/chunk validation and prepare an origin-free frame.
    sanitized_png(output / "display.png", output / "review.png")
    if png_size(output / "review.png") != (manifest["width"], manifest["height"]):
        raise ValueError("PNG dimensions differ from the manifest")
    hdr = output / "linear.rgba16f"
    available = capture["metadata"]["graphics"]["resolved"]["capture_linear_hdr"].startswith("available:")
    if available != hdr.exists() or (available and hdr.stat().st_size != manifest["width"] * manifest["height"] * 8):
        raise ValueError("linear HDR artifact does not match capability metadata")
    if source_state(ROOT) != source or sha256(binary) != binary_hash or sha256(manifest_path) != manifest_hash:
        raise ValueError("source, binary or manifest changed during capture")
    for item in inputs:
        if sha256(Path(item["resolved_path"])) != item["sha256"]:
            raise ValueError("input changed during capture")
    artifacts = {p.name: sha256(p) for p in output.iterdir() if p.is_file()}
    record = {"schema_version": 1, "scene_id": manifest["id"], "scene_manifest_sha256": manifest_hash,
              "source": source, "binary_sha256": binary_hash, "profile": profile,
              "inputs": inputs, "metadata": capture["metadata"], "artifacts_sha256": artifacts,
              "linear_hdr": {"available": available, "format": "RGBA16Float", "pre_exposed": True,
                             "tonemapped": False, "row_padding_bytes": 0},
              "acceptance": {"physical": "not evaluated", "visual": "not evaluated",
                             "performance": "raw serial diagnostic samples only", "temporal": "not evaluated"}}
    (output / "capture.json").write_text(json.dumps(record, indent=2) + "\n")
    print(output / "capture.json")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("manifest", type=Path)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--profile", choices=("dev", "release"), default="release")
    parser.add_argument("--allow-dirty", action="store_true", help="record an explicit dirty source fingerprint")
    args = parser.parse_args()
    try:
        run(args.manifest, args.output, args.profile, args.allow_dirty)
    except (OSError, ValueError, KeyError, TypeError, subprocess.CalledProcessError) as exc:
        print(f"Capture failed: {exc}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
