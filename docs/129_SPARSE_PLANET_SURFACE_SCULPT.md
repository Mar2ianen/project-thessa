# 129 — Sparse spherical surface sculpt: first OpenEngine adaptation

Status: implemented CPU height-layer core, canonical field snapshot integration
and actual GPU height-page consumption. Live wheel/foot contact production,
stream invalidation and reference-quality soil remain open. Doc 130 adds disk
checkpoint APIs; automatic saving and regional backing storage remain open.

## Primary source and existing owners

Inspected OpenEngine revision `bdf5c2710fe37c1c5ddbac7787477daa87524af3`:

- [`SphereSculptLayer.h`](https://github.com/Game-Crafters-Guild/OpenEngine/blob/bdf5c2710fe37c1c5ddbac7787477daa87524af3/Engine/Modules/CBTTerrain/Include/CBTTerrain/SphereSculptLayer.h): sparse additive editable height, absent pages equal zero, world-direction
  brush evaluation, bounded storage and revision tracking.
- [`SphereSculptPaging.h`](https://github.com/Game-Crafters-Guild/OpenEngine/blob/bdf5c2710fe37c1c5ddbac7787477daa87524af3/Engine/Modules/CBTTerrain/Include/CBTTerrain/SphereSculptPaging.h): radius-scaled physical texel demand and rejection/escalation of
  brushes that cannot be represented by the grid.

The repository is MIT. This is an independent Rust adaptation of these
mechanisms, not a wholesale C++/GLSL transplant. No upstream CBT kernels are
copied, including the separately recorded upstream `large_cbt` licensing
boundary (doc 127). Large CBT remains the topology reference; Thessa retains
its existing persistent conforming bisectors, source pages and raster path.

CBT controls the mesh representation. A wheel rut must instead have a stable
surface-data owner, surviving a change in bisector allocation, material LOD or
resident page. Increasing triangles alone cannot retain a rut.

## Implemented contract

`thessa_rcbt_core::surface_sculpt::SurfaceSculpt` is backend-neutral. It adds no
render handles, game dependencies, allocation proportional to planetary area,
new shaders, settings or render passes.

- `new(radius_m, target_texel_m, max_pages)` derives a fixed virtual cube-page
  level from physical spacing. Level is at most 31 for CPU addresses.
- Each allocated page stores 33×33 f64 metre offsets: 32 cells plus shared
  endpoint samples, 8,712 bytes of sample payload. Map/allocator metadata and
  temporary transaction storage are additional.
- `apply_dab(direction, radius_m, delta_m)` adds a circular smoothstep stamp in
  true angular distance, measured with atan2 of cross length and dot product.
  Radius is metres, amplitude is authored metres, not an inferred soil response.
- A conservative spherical-cap bound enumerates affected face rectangles.
  Candidate work and retained pages are bounded by `max_pages`. A conservative
  bound may reject a stamp before exhausting actual nonzero storage.
- Capacity or invalid/nonfinite input refuses the whole operation. Existing
  samples and revision remain unchanged. Zero-strength operations allocate no
  page or revision. Positive/negative circular stamps are supported; rectangular
  shoes, tread kernels, undo and page reclamation are not implemented.
- The brush diameter must cover four requested texels and radius must not exceed
  1% of the body radius. Too-small brushes return an error rather than vanish.
  This first version rejects unsupported demand; it does not implement upstream's
  adaptive per-page escalation or silent coarsening.
- `sample(direction)` bilinearly samples the retained page, with absent pages
  exactly zero. Inputs normalize finite nonzero direction vectors. Face/UV
  conventions match existing TileKey/render consumers; f64 mapping avoids
  quantizing centimetre edits through the existing topology-address grid.

`PlanetField::with_surface_sculpt` attaches an owned frozen layer before the
field is shared with workers. Canonical contact heights, shared-prefix surface
samples and height-page bakes all compose the same layer after the geological
parent, before final height limits. Existing material base normals consume
those surface samples too. The geological-parent diagnostic excludes edits.
Empty layers retain the original arithmetic path. Radius mismatch and a second
nonempty attachment are rejected. Authoring does not rebuild climate/hydrology.

This is immutable snapshot integration. Replacing a populated live stream is
not implemented: a new stream must consume the new field. There is no API that
mutates a shared field while old jobs publish obsolete pages. Erosion-source
signatures reject authored layers, preventing accidental double baking of edits
into geological erosion identity. [Doc 130](130_SURFACE_SCULPT_CHECKPOINTS.md)
adds an exact snapshot codec and world-bound file APIs. This first layer still
has no unlimited planetary traffic storage or live region refresh.

The layer changes source data, not an optional lighting effect. Existing
render quality still determines which geometry/material samples can resolve
it. It does not force nearby pages or guarantee that a sub-grid footprint is
visible in every quality tier. The current renderer's below-datum clamp remains.

## Verification

- Six sculpt correctness cases cover finite/degenerate inputs, unsupported
  brush footprints, peak/range, untouched zero, radius scaling, transaction
  refusal, zero work and all twelve cube edges/eight corners. A 20 cm radius,
  4 cm deep stamp at 2.5 cm requested spacing stays within 1 mm peak error and
  0.1 mm two-sided seam difference at these planetary fixtures.
- Canonical integration verifies contact, shared-prefix height/surface queries,
  empty-field bit identity, radius refusal, erosion identity refusal and actual
  page decoding within the existing 1 mm encoding budget.
- All 82 core unit tests and 296 worldgen tests passed (one unrelated worldgen
  test remains ignored). Strict core/worldgen Clippy, native application check,
  renderer test Clippy and the architecture guard passed.
- `render-wgpu/tests/surface_sculpt.rs` renders baseline/edited/restored pages
  through the actual portable terrain pipeline on Radeon 880M/RADV. A 60 cm
  diameter, 4 cm deep circular depression changes 950 of 4,225 diagnostic pixels;
  restoring the base is byte-identical. Source storage is 16 pages / 139,392
  sample bytes. This synthetic normal diagnostic is not a soil beauty/reference
  acceptance test, and does not exercise a moving camera or live page refresh.
- The manual unoptimized test-profile CPU workload applies 128 authored circular
  dabs over 10.16 m: 30 retained pages / 261,360 sample bytes; approximately
  55.3 ms total write work and 30.0 ms for 100,000 samples in that run. It excludes
  page reconstruction, uploads and rendering. No scene FPS or speedup is claimed.

Commands:

```sh
cargo test -p thessa-rcbt-core --lib
cargo test -p thessa-worldgen-rocky --lib
cargo test -p thessa-rcbt-core bounded_planet_track_workload --lib -- --ignored --nocapture
cargo test -p thessa-render-wgpu --test surface_sculpt -- --ignored --nocapture --test-threads=1
cargo clippy -p thessa-rcbt-core -p thessa-worldgen-rocky --lib -- -D warnings
cargo clippy -p thessa-render-wgpu --test surface_sculpt -- -D warnings
cargo check -p thessa-terrain-native
python3 tools/agent-qa/check_architecture.py
```

GPU execution used the existing render-node-only bubblewrap envelope without a
display connection. Optional test PNGs use the existing
`THESSA_TEST_ARTIFACT_DIR` convention and do not select an algorithm. Evidence is
in [the source/check record](artifacts/sparse-surface-sculpt-2026-11-03.json).
The default B03 field has no authored stamps and its beauty is unchanged by
this milestone; a fresh full B03 visual improvement is not asserted.

## Next integration slices

1. Snapshot save/load is implemented in doc 130. Add a journal and bounded
   regional backing storage, so a long drive does not fill one authored-page pool.
2. Changed-region invalidation of height/material jobs and publication, with
   stale result rejection and conservative refinement/height bounds. Reuse
   existing stream/cache/source generations; avoid a second topology system.
3. Wheel/foot contact patches from authoritative contact state. Derive plastic
   depth/compaction from a calibrated soil/contact law and actual load/slip;
   circular authored dabs do not constitute that physics. Add oriented tread/
   sole shapes and compaction material/vegetation response through shared data.
4. Near-field representation demand for centimetre features, with coarse filtered
   fallback and moving-camera/page-boundary verification.
5. Fine moss/soil material integration as a separate source/shading task. This
   mechanism does not repair repetitive or muddy existing substrate textures.
