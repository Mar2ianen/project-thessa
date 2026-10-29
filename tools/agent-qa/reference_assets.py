#!/usr/bin/env python3
"""Fetch pinned B02 source files and reproduce declared derived assets outside Git."""
from __future__ import annotations

import argparse
import hashlib
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
import urllib.request

ROOT = Path(__file__).resolve().parents[2]
DEFAULT_MANIFEST = ROOT / "data/reference-scenes/b02-assets.json"
DEFAULT_CACHE = Path(os.environ.get("XDG_CACHE_HOME", Path.home() / ".cache")) / "thessa/reference-assets"
USER_AGENT = "ProjectThessaReferenceScenes/1.0 (+https://github.com/Mar2ianen/project-thessa)"


def sha256(path: Path) -> str:
    with path.open("rb") as source:
        return hashlib.file_digest(source, "sha256").hexdigest()


def cache_path(cache: Path, relative: str) -> Path:
    path = (cache / relative).resolve()
    if not path.is_relative_to(cache.resolve()) or path == cache.resolve():
        raise ValueError("asset path escapes the cache")
    return path


def fetch_source(cache: Path, item: dict) -> Path:
    path = cache_path(cache, item["path"])
    if path.exists():
        if path.stat().st_size != item["bytes"] or sha256(path) != item["sha256"]:
            raise ValueError(f"existing source checksum mismatch: {path}")
        return path
    if not item["url"].startswith("https://dl.polyhaven.org/") or not 0 < item["bytes"] <= 16 << 20:
        raise ValueError("unexpected source host or file budget")
    path.parent.mkdir(parents=True, exist_ok=True)
    request = urllib.request.Request(item["url"], headers={"User-Agent": USER_AGENT})
    with urllib.request.urlopen(request, timeout=60) as response:
        data = response.read(item["bytes"] + 1)
    if len(data) != item["bytes"] or hashlib.sha256(data).hexdigest() != item["sha256"]:
        raise ValueError(f"download checksum mismatch: {item['url']}")
    with path.open("xb") as destination:
        destination.write(data)
    return path


def prepare(manifest_path: Path, cache: Path) -> None:
    cache = cache.resolve()
    if cache == ROOT or ROOT in cache.parents:
        raise ValueError("reference assets must be cached outside the source checkout")
    manifest = json.loads(manifest_path.read_text())
    if manifest.get("schema_version") != 1:
        raise ValueError("unknown asset manifest schema")
    converter = ROOT / manifest["converter"]["path"]
    if sha256(converter) != manifest["converter"]["sha256"]:
        raise ValueError("converter changed; audit/re-pin its version before ingestion")
    for asset in manifest["assets"]:
        if sha256(ROOT / asset["license_notice"]) != manifest["license_text"]["sha256"]:
            raise ValueError("license notice differs from pinned text")
        target = cache_path(cache, asset["path"])
        if target.exists():
            if sha256(target) != asset["converted_sha256"]:
                raise ValueError(f"existing converted asset checksum mismatch: {target}")
            print(f"verified {asset['id']}: {target}")
            continue
        sources = {item["role"]: fetch_source(cache, item) for item in asset["source_files"]}
        source = sources["blend"]
        target.parent.mkdir(parents=True, exist_ok=True)
        with tempfile.TemporaryDirectory(prefix="convert-", dir=target.parent) as temporary:
            output = Path(temporary)
            env = os.environ.copy()
            env["THESSA_ASSET_EXPORT"] = str(output)
            command = ["blender", "--background", "--factory-startup", "--disable-autoexec",
                       "--python-exit-code", "1", "--threads", "2", str(source), "--python", str(converter)]
            log_path = target.parent / "conversion.log"
            with log_path.open("w") as log:
                subprocess.run(command, env=env, stdout=log, stderr=subprocess.STDOUT, check=True)
            generated = output / target.name
            report = json.loads((output / "conversion.json").read_text())
            if (sha256(generated) != asset["converted_sha256"]
                    or report["triangles"] != asset["triangles"]
                    or report["converter_sha256"] != manifest["converter"]["sha256"]
                    or report["source_blend_sha256"] != asset["source_sha256"]
                    or report["blender_version"] != manifest["converter"]["blender_version"]
                    or report["blender_build_hash"] != manifest["converter"]["blender_build_hash"]):
                raise ValueError(f"conversion differs from pinned artifact: {asset['id']}; see {log_path}")
            shutil.copyfile(output / "conversion.json", target.parent / "conversion.json")
            generated.replace(target)
            (target.parent / "command.json").write_text(json.dumps(command, indent=2) + "\n")
        print(f"converted {asset['id']}: {target}")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--manifest", type=Path, default=DEFAULT_MANIFEST)
    parser.add_argument("--cache", type=Path, default=DEFAULT_CACHE)
    args = parser.parse_args()
    try:
        prepare(args.manifest, args.cache)
    except (OSError, ValueError, KeyError, TypeError, subprocess.CalledProcessError) as exc:
        print(f"Asset ingestion failed: {exc}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
