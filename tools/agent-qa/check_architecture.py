#!/usr/bin/env python3
"""Enforce engine boundaries on Cargo's resolved dependency graph."""
from __future__ import annotations

import argparse
from collections import deque
import json
import subprocess
import sys
from pathlib import Path

FOUNDATIONS = frozenset({
    "thessa-sim-core", "thessa-aero-core", "thessa-celestial",
    "thessa-trajectory", "thessa-propulsion", "thessa-vehicle-core",
    "thessa-rcbt-core", "thessa-graphics", "thessa-plume-core",
    "thessa-microstore-core", "thessa-perf", "thessa-audio-core",
    "thessa-render-core", "thessa-render-assets", "thessa-fastrt",
    "thessa-simd", "thessa-collision", "thessa-aero-surfaces", "thessa-fuselage",
})
FORBIDDEN_BACKENDS = frozenset({
    "bevy", "wgpu", "ash", "vulkano", "gpui", "winit", "eframe", "egui",
    "tokio", "lightyear",
})
GENERIC_RENDERERS = frozenset({
    "thessa-render-core", "thessa-render-assets", "thessa-render-wgpu",
    "thessa-graphics", "thessa-fastrt", "thessa-fastrt-wgpu",
    "thessa-rcbt-core", "thessa-rcbt-wgpu", "thessa-bevy-rcbt",
    "thessa-microstore-core", "thessa-plume-core", "thessa-wgpu-exp",
})
GENERIC_PHYSICS = frozenset({
    "thessa-aero-core", "thessa-propulsion", "thessa-vehicle-core",
    "thessa-collision", "thessa-aero-surfaces", "thessa-fuselage", "thessa-simd",
})
CELESTIAL_AGGREGATES = frozenset({"thessa-sim-core", "thessa-celestial", "thessa-trajectory"})
# Exact existing runtime edges, not exemptions for every path from these roots.
# R2 removes collision's edge and its allowance together. Other removals stay staged.
STAGED_EDGES = frozenset({
    ("thessa-collision", "thessa-sim-core"),
    ("thessa-aero-surfaces", "thessa-sim-core"),
    ("thessa-fuselage", "thessa-sim-core"),
})


def _backend(name: str) -> bool:
    name = name.replace("-", "_")
    return name in FORBIDDEN_BACKENDS or name.startswith(("bevy_", "wgpu_", "tokio_", "lightyear_"))


def _graphs(metadata: dict) -> tuple[dict, dict, dict]:
    packages = {p["id"]: p for p in metadata["packages"]}
    runtime = {pid: set() for pid in packages}
    own_tests = {pid: set() for pid in packages}
    resolved = metadata.get("resolve")
    if resolved is not None:
        for node in resolved["nodes"]:
            for dep in node["deps"]:
                own_tests[node["id"]].add(dep["pkg"])
                kinds = dep.get("dep_kinds", [])
                if not kinds or any(k.get("kind") != "dev" for k in kinds):
                    runtime[node["id"]].add(dep["pkg"])
    else:
        # Declaration-only fixtures retain the original public violations API.
        # The CLI always requires a complete resolved graph for transitive checks.
        by_name = {p["name"]: pid for pid, p in packages.items()}
        for pid, package in list(packages.items()):
            for dep in package.get("dependencies", []):
                target = by_name.get(dep["name"], "external:" + dep["name"])
                packages.setdefault(target, {"id": target, "name": dep["name"]})
                runtime.setdefault(target, set())
                own_tests.setdefault(target, set())
                own_tests[pid].add(target)
                if dep.get("kind") != "dev":
                    runtime[pid].add(target)
    return packages, runtime, own_tests


def _path(packages: dict, graph: dict, root: str, predicate, first=None, skip=frozenset()):
    queue = deque([(root, [root])])
    seen = {root}
    while queue:
        current, path = queue.popleft()
        edges = first if current == root and first is not None else graph[current]
        for target in sorted(edges, key=lambda p: (packages[p]["name"], p)):
            pair = (packages[current]["name"], packages[target]["name"])
            if pair in skip or target in seen:
                continue
            next_path = path + [target]
            if predicate(target):
                return next_path
            seen.add(target)
            queue.append((target, next_path))
    return None


def inspect(metadata: dict) -> dict:
    packages, runtime, own_tests = _graphs(metadata)
    members = set(metadata["workspace_members"])
    local = {packages[pid]["name"]: packages[pid] for pid in members}
    errors, staged = [], []
    names = lambda path: " -> ".join(packages[p]["name"] for p in path)
    applications = {p for p in members if "/apps/" in packages[p].get("manifest_path", "")}
    # Names also permit relocatable synthetic regression graphs.
    applications |= {p for p in members if packages[p]["name"] in {
        "thessa-client", "thessa-server", "thessa-terrain-native",
    }}
    for pid in sorted(members, key=lambda p: packages[p]["name"]):
        item = packages[pid]
        name = item["name"]
        # Preserve the original direct checks, including declared optional/dev edges.
        for dep in item.get("dependencies", []):
            target = dep["name"]
            if name in FOUNDATIONS and _backend(target):
                errors.append(f"{name}: forbidden backend dependency {target}")
            if (item.get("license") == "MIT" and target in local
                    and "GPL" in (local[target].get("license") or "")):
                errors.append(f"{name}: MIT crate depends on GPL crate {target}")
        if item.get("license") == "MIT":
            path = _path(packages, runtime, pid,
                         lambda p: p in members and "GPL" in (packages[p].get("license") or ""),
                         first=own_tests[pid])
            if path and len(path) > 2:
                errors.append(f"{name}: MIT-to-GPL dependency path: {names(path)}")
        if name in FOUNDATIONS:
            path = _path(packages, runtime, pid, lambda p: _backend(packages[p]["name"]),
                         first=own_tests[pid])
            if path and len(path) > 2:
                errors.append(f"{name}: backend dependency path: {names(path)}")
        if pid not in applications:
            path = _path(packages, runtime, pid, lambda p: p in applications)
            if path:
                errors.append(f"{name}: reverse application dependency: {names(path)}")
        if name in GENERIC_RENDERERS:
            path = _path(packages, runtime, pid,
                         lambda p: packages[p]["name"] in {"thessa-worldgen-rocky", "thessa-terrain-stream"})
            if path:
                errors.append(f"{name}: game terrain is mandatory for rendering: {names(path)}")
        if name in GENERIC_PHYSICS:
            predicate = lambda p: packages[p]["name"] in CELESTIAL_AGGREGATES
            path = _path(packages, runtime, pid, predicate, skip=STAGED_EDGES)
            if path:
                errors.append(f"{name}: generic physics requires simulation/celestial: {names(path)}")
            legacy = _path(packages, runtime, pid, predicate)
            if legacy and not path:
                staged.append(f"{name}: staged facade removal: {names(legacy)}")
    return {
        "errors": sorted(set(errors)), "staged": sorted(set(staged)),
        "workspace_runtime_edges": sorted(
            [packages[p]["name"], packages[d]["name"]]
            for p in members for d in runtime[p] if d in members
        ),
        "packages": sorted([
            {"name": packages[p]["name"], "license": packages[p].get("license"),
             "manifest_path": packages[p].get("manifest_path")}
            for p in members
        ], key=lambda p: p["name"]),
    }


def violations(metadata: dict) -> list[str]:
    return inspect(metadata)["errors"]


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--metadata", type=Path, help="inspect a previously saved full Cargo graph")
    parser.add_argument("--report", type=Path, help="write machine-readable boundaries and workspace runtime edges")
    args = parser.parse_args()
    root = Path(__file__).resolve().parents[2]
    try:
        raw = args.metadata.read_bytes() if args.metadata else subprocess.check_output(
            ["cargo", "metadata", "--format-version", "1", "--all-features"],
            cwd=root, stderr=subprocess.PIPE,
        )
        metadata = json.loads(raw)
        if metadata.get("resolve") is None:
            raise ValueError("full Cargo resolve graph is required (do not use --no-deps)")
        result = inspect(metadata)
        if args.report:
            args.report.parent.mkdir(parents=True, exist_ok=True)
            args.report.write_text(json.dumps(result, indent=2) + "\n")
    except (OSError, subprocess.CalledProcessError, ValueError, KeyError, TypeError) as exc:
        print(f"Cannot inspect workspace: {exc}", file=sys.stderr)
        return 2
    for error in result["errors"]:
        print("Architecture violation: " + error, file=sys.stderr)
    for pending in result["staged"]:
        print("Architecture staged: " + pending)
    if result["errors"]:
        return 1
    print(f"Architecture: OK ({len(result['packages'])} workspace packages, "
          f"{len(FOUNDATIONS)} guarded foundations; runtime/build closure, own dev edges only)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
