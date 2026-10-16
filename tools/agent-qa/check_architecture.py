#!/usr/bin/env python3
"""Enforce the authoritative simulation and licensing dependency boundaries."""
from __future__ import annotations

import json
import subprocess
import sys
from pathlib import Path

# These crates are authoritative, backend-neutral foundations, not GPU adapters.
FOUNDATIONS = frozenset({
    "thessa-sim-core", "thessa-aero-core", "thessa-celestial",
    "thessa-trajectory", "thessa-propulsion", "thessa-vehicle-core",
    "thessa-rcbt-core", "thessa-graphics", "thessa-plume-core",
    "thessa-microstore-core", "thessa-perf", "thessa-audio-core",
})
FORBIDDEN_BACKENDS = frozenset({
    "bevy", "bevy_ecs", "bevy_render", "wgpu", "ash", "vulkano",
    "gpui", "winit", "eframe", "egui", "tokio", "lightyear",
})


def violations(metadata: dict) -> list[str]:
    packages = {item["id"]: item for item in metadata["packages"]}
    workspace = [packages[pid] for pid in metadata["workspace_members"]]
    by_name = {item["name"]: item for item in workspace}
    errors = []
    for item in workspace:
        name = item["name"]
        license_id = item.get("license") or ""
        for dep in item.get("dependencies", []):
            target = dep["name"]
            if name in FOUNDATIONS and target.replace("-", "_") in FORBIDDEN_BACKENDS:
                errors.append(f"{name}: forbidden backend dependency {target}")
            local = by_name.get(target)
            if (local and license_id == "MIT" and
                "GPL" in (local.get("license") or "")):
                errors.append(f"{name}: MIT crate depends on GPL crate {target}")
    return sorted(set(errors))


def main() -> int:
    root = Path(__file__).resolve().parents[2]
    try:
        raw = subprocess.check_output(
            ["cargo", "metadata", "--no-deps", "--format-version", "1"],
            cwd=root, stderr=subprocess.PIPE,
        )
        errors = violations(json.loads(raw))
    except (OSError, subprocess.CalledProcessError, ValueError) as exc:
        print(f"Cannot inspect workspace: {exc}", file=sys.stderr)
        return 2
    for error in errors:
        print("Architecture violation: " + error, file=sys.stderr)
    if errors:
        return 1
    print(f"Architecture: OK ({len(FOUNDATIONS)} guarded foundational crates)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
