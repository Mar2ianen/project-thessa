# 130 — Sparse surface sculpt checkpoints

Status: implemented deterministic snapshot codec, world-bound file checkpoints
and exact loaded-layer raster regression. Hosts can save and restore an authored
layer before stream construction. Automatic game saves, contact-generated
tracks, regional backing storage and live dirty-page refresh remain open.

## Reference and ownership

The primary implementation reference is OpenEngine's actual
[`SphereSculptSerialization.h`](https://github.com/Game-Crafters-Guild/OpenEngine/blob/bdf5c2710fe37c1c5ddbac7787477daa87524af3/Engine/Modules/CBTTerrain/Include/CBTTerrain/SphereSculptSerialization.h)
and corresponding `Source/SphereSculptSerialization.cpp` at
`bdf5c2710fe37c1c5ddbac7787477daa87524af3` (MIT). Its sparse, bounded, device-free
snapshot separates saved geometry from the live allocation budget. This is an
independent Rust format and implementation; OpenEngine's TSCP files are not
interchangeable with these files. No upstream code or CBT kernel is copied.

`rcbt-core::surface_sculpt` owns bytes and validation without filesystem or
world/game dependencies. `worldgen-rocky::surface_sculpt_io` owns the file and
its caller-supplied world/source key. Existing canonical field attachment and
height-page rendering remain the consumers; no new render pass, graphics
setting, residency cache or topology system is introduced.

## Byte contract

`SurfaceSculpt::to_bytes()` emits deterministic little-endian TSCS version 1:

| Offset | Field |
| ---: | --- |
| 0 | `TSCS` magic, four bytes |
| 4 | Version u16 = 1; cells per page u16 = 32 |
| 8 | Planet radius f64, then requested texel spacing f64 |
| 24 | Authored revision u64 |
| 32 | Derived level u8 plus seven zero reserved bytes |
| 40 | Page count u64 |
| 48 | Sorted records: face u8, level u8, zero u16, x/y u32, then 1,089 f64 metre offsets |
| End | IEEE CRC-32 u32 over the complete preceding header and records |

Each page record is 8,724 bytes; an empty snapshot is 52 bytes.
`from_bytes(bytes, max_pages)` takes its allocation budget from the caller,
never from disk. It checks the geometry-derived level, page count/grid bounds,
checked exact length and CRC before allocating page samples. Record validation
rejects invalid or duplicate/unsorted addresses, nonzero reserved bytes and
nonfinite samples. Unsupported versions/schema, truncation and trailing data
are rejected. Retained values, addresses, geometry and revision round-trip
exactly; subsequent edits remain supported under the new caller budget.

`SurfaceSculpt::snapshot_size_limit(max_pages)` provides checked byte bounds to
file readers, avoiding duplicate assumptions about format size. CRC has the
independent standard `123456789 -> 0xcbf43926` oracle. It detects corruption;
it is not an authentication mechanism or semantic proof of an authored shape.
The codec retains allocated zero pages rather than changing page identity or
claiming to implement upstream's zero-layer elision/adaptive page levels.

During review, finite opposite-sign endpoint values exposed overflow in the
existing difference-based bilinear interpolation. Sampling now uses bounded
convex combinations, keeping finite loaded endpoints finite without inventing
a physical height cutoff. A regression exercises extreme finite values after
snapshot decoding. Canonical final-height limits still belong to PlanetField.

## World-bound files and publication

`surface_sculpt_io::save(path, world_key, &layer)` writes a TSA1 envelope:
magic, u32 key length, UTF-8 key bytes, then the TSCS snapshot. The key contains
1–4,096 bytes. The host must supply a stable planet/source identifier; radius
alone cannot distinguish two planets. The wrapper compares the exact expected
key before decoding pages. It does not derive or authenticate that identity.

Saving creates an exclusive temporary file beside the destination, writes the
complete envelope, syncs file contents, closes it and atomically renames it to
the checkpoint. Failure before publication leaves the previous checkpoint
intact. The function removes its own temporary file on an ordinary failure;
leftovers from process crashes are not automatically scavenged. Atomic rename
uses the platform filesystem semantics; power-loss durability of directory
metadata and multi-writer conflict resolution are not promised. Concurrent
successful saves follow last-publication-wins behavior.

`load(path, expected_world_key, max_pages)` validates the fixed envelope and
bounded key, then limits its read to the maximum codec size plus one byte.
Oversize, mismatched-world, malformed or corrupt inputs return an error. It
constructs a new layer and cannot mutate an existing live source on failure.

Host usage, before sharing the canonical field with jobs:

```rust
let layer = thessa_worldgen_rocky::surface_sculpt_io::load(
    checkpoint_path, "stable-body-id/base-source-version", 128,
)?;
let field = field.with_surface_sculpt(layer)?;
// Construct the existing TerrainStream from this field and normal policy.
```

These are explicit host APIs. They do not add automatic startup loading,
periodic checkpointing, a native CLI flag or a live stream replacement path.
Loading does not invalidate already-published pages: the loaded field must
enter a newly constructed stream until changed-region refresh is implemented.

## Executed evidence

- Thirteen focused sculpt tests pass, with the manual CPU workload ignored.
  Codec tests cover exact/empty round-trip, continued editing, external page
  budgets, malicious counts, schema/key failures, duplicate pages, nonfinite
  values, truncation, trailing bytes and checksum corruption. The all-twelve-
  edges/eight-corners planetary acceptance case now includes snapshot decoding.
- Five filesystem cases pass: exact save/load and replacement, wrong world/
  insufficient budget/corruption rejection, failed-publication cleanup/data
  preservation, re-entry into actual canonical contact/prefix queries, and bounded
  rejection of malformed envelope lengths/oversized files.
- The actual Radeon 880M/RADV 65×65 normal diagnostic now renders baseline,
  authored, decoded and restored baseline. Authored/decoded are byte-identical;
  restored baseline is byte-identical. The depression still changes 950/4,225
  pixels. This is numerical/raster persistence acceptance, not a soil beauty
  comparison or a frame-time benchmark.
- Strict core/worldgen library/test and renderer-test Clippy, native application
  compilation, formatting and the architecture guard pass. No full-scene visual
  improvement, complete renderer parity or new speedup is claimed.

Commands:

```sh
cargo test -p thessa-rcbt-core surface_sculpt --lib
cargo test -p thessa-worldgen-rocky surface_sculpt_io --lib
cargo test -p thessa-render-wgpu --test surface_sculpt -- --ignored --nocapture --test-threads=1
cargo clippy -p thessa-rcbt-core -p thessa-worldgen-rocky --lib --tests -- -D warnings
cargo clippy -p thessa-render-wgpu --test surface_sculpt -- -D warnings
cargo check -p thessa-terrain-native
python3 tools/agent-qa/check_architecture.py
```

The GPU executable runs in the existing render-node-only bubblewrap envelope.
Source hashes, actual logs and PNG hashes are recorded in
[the evidence artifact](artifacts/surface-sculpt-checkpoints-2026-11-03.json).
Next required terrain integration is changed-region refresh with stale-job
rejection; contact/soil physics, tread shapes and regional disk-backed eviction
remain separate tasks. The existing B03 ground texture is unchanged.
