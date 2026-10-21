# 74 — Datum-surface cloud shadow receivers

**Scope:** WORLD-02, follow-up to the surface foundation at `20c3098`.
This repairs primary terrain illumination, not terrain generation, water
simulation, surface detail or vegetation. Visual acceptance remains separate.

## Reproduction and cause

WORLD00's fixed land target is 23.565679258039204° N,
165.13881581863882° E. The overview uses 800 m radial eye clearance and
1800 m survey distance; close frames use 50 m/2 m clearance and 80 m distance.
All captures freeze simulation epoch and step at zero and explicitly mute audio.
See [doc 71](71_NATIVE_SURFACE_ACCEPTANCE.md) for camera semantics.

At `20c3098`, dark curved stippling covers the water. Disabling the atlas
(`shadows.enabled`), specular reflections or material normals independently did
not remove it. Those controls did **not** disable cloud receiver shadows:
`clouds.cast_shadows` is a separate resolved setting. Changing only that setting
removes the dominant pattern while retaining clouds and the atlas.

The primary terrain hook multiplied atlas visibility by
`cloud_sun_visibility`. That shared helper reconstructs a planet-centric f32
origin from the camera and relative fragment position, then rejects any ray
starting inside the datum sphere. Terrain samples are clamped to the datum
before meshing, but triangle interiors are chords inside the sphere; f32
planetary reconstruction also rounds across its boundary. An outward day-side
ray then alternated between lit and classified as inside solid planet. Empty
cloud weather cannot repair that rejection: it occurs before density integration.

There is no second coplanar water-shell draw in this path. Increasing terrain
noise, changing albedo, disabling shadows or adding an arbitrary metre-scale
receiver bias would not fix the faulty boundary contract.

## Correction and ownership

Only the primary terrain hook uses `cloud_terrain_sun_visibility`.
For reconstructed origin `o`, unit sun direction `d` and datum radius `R`, it
uses the terrain support convention:

```text
h = max(length(o) - R, 0)
b = dot(o, d)
planet blocks iff b < 0 and b² > h(2R + h)
```

This is the forward ray/sphere discriminant for a non-interior receiver. It
avoids cancellation in near roots, retains elevated views of sunlight below
the local horizontal, rejects inward/night-side rays and leaves a tangent
with zero interior path unblocked. The cloud column still uses the original
origin, weather, deck geometry and integration budget. No geometry is moved.

Reproduce the primary overview without changing graphics defaults:

```sh
mkdir -p target/test-tmp/coastal-cloud-shadow
TMPDIR=/home/chechulin/Projects/project-thessa/target/tmp \
cargo run --release -p thessa-terrain-native -- \
  --graphics graphics.toml \
  --survey-lat 23.565679258039204 --survey-lon 165.13881581863882 \
  --survey-distance 1800 --survey-eye-agl 800 --sim-time 0 --sim-step 0 \
  --width 1280 --height 720 --prestream-seconds 60 --frames 32 \
  --mute --no-hud --headless target/test-tmp/coastal-cloud-shadow/overview.png \
  --perf-capture target/test-tmp/coastal-cloud-shadow/overview.json
```

The generic `cloud_sun_visibility`, primary/secondary volume transport and mesh
queries remain strict, including real interior rejection. Reflected terrain
hits still use the generic path; this change does not fix every secondary
receiver. Regular and conforming primary terrain share the corrected hook,
including spectral-ocean direct lighting. Ocean troughs can physically cross
below datum: this convention treats the datum as surface support rather than
solid water, and does not implement wave-horizon self-occlusion.

No setting, quality tier, binding or allocation is added. Existing cloud/receiver
disabled branches still return before geometric work. Enabled shadow integration
uses the same sample budget and shared settings ownership.

## Numerical boundary and remaining gaps

The clamp restores a lower bound, not exact surface altitude. It does not fix
f32 planetary reconstruction, positive altitude error, tessellation sag or
near-limb cloud quadrature. For scale, an edge midpoint's chord sag is about
`L²/(8R)` (3.9 m for a 10 km edge at R = 3200 km); this is not a whole-triangle
error certificate. A hypothetical 0.25–1 m positive altitude error shifts the
datum horizon by roughly 0.023–0.045°. These are sensitivity examples, not
measured global bounds or physical terrain accuracy.

Cloud-field calibration, exact displaced-ocean self-occlusion, secondary terrain
receivers, full terrain convergence and temporal/Metal/WebGPU acceptance remain
open. Flat optical vegetation cover, angular/coarse shoreline transitions and
missing plant geometry are independent visible limitations.

## Independent preference boundary

The previously unreviewed surface-material pairs from doc 71 received an
independent blind review (`ses_ea2be7e81ffe7EOfLFwq7Xb501`) before this fix's
pairs were prepared: the old sandy overview was preferred, with ties at 50 m
and 2 m. The reviewer found pale angular patches and no convincing additional
surface detail. This does not invalidate the tested cover/substrate causality
correction, but it rules out claiming that recoloring alone established better
natural appearance. Physical material correctness and aesthetic preference are
different gates; the old sand paint is not restored to win a preference vote.

## Executed evidence

The actual production-renderer regression failed at `20c3098`: transparent
weather darkened **3,732 of 4,225** covered daylight pixels, with maximum
128/255 linear RGB error against a 1/255 tolerance. With the corrected helper,
the same fixture has **zero changed pixels and zero maximum error**. The
nonnegative sloped night fixture remains genuinely illuminated when receiver
shadows are disabled and becomes black with planet occultation enabled; a
Lambert back face cannot make that test pass vacuously.

The shared-WGSL compute contract covers four orientations, negative chord/error
proxies, daylight/night, elevated limb-clear and blocked rays, exact axis datum
tangents with ±0.001 slope perturbations, disabled flags, generic underground
rejection, nonzero weather attenuation against an independent Simpson column,
and a safe above-datum camera rebase. Visibility tolerance is `1e-5` and column
transmission/rebase tolerance is `0.002`. New planetary-scale raster fixtures
cover regular patches, not separate conforming or displaced-ocean executions.

Six new paired silent frames retain all cloud, cloud-shadow, atlas, material
normal and reflection settings enabled. Final camera metadata is exactly equal
for every pair; all material misses are zero and visible/wanted/cache counts
match. A fixed 350×120 overview-water ROI changes from **6,902** pixels with
RGB sum below 180 to **zero**. This is a diagnostic for this artifact, not a
universal water-quality metric. The first corrected overview still had four
height jobs in flight; a 60-second-prestream repeat drains them with the same
camera/counts and no return of the stippling. That repeat differs at 233 pixels
from the reviewed frame, so it is not claimed byte-identical or independently
blind-reviewed. Resident-ID snapshots were not recorded.

The independent blinded reviewer (`ses_ea2b6cb0effecvMUluj0ZiRp0V`) preferred the
corrected frame in all three pairs, chiefly for removal of the black pattern;
confidence was high/high/medium. They still rejected any inference of convincing
fine ground detail, correct shadow placement, water physics or temporal/LOD
acceptance. This is one reviewer and one shuffle, not a statistical aesthetic
ranking. A separate read-only source review found no blocking implementation
or test assertion defect; GPU execution belongs to the integration owner.

Measured whole-client GPU medians in the first paired capture were
74.17/74.27/70.90 ms before and 87.99/82.47/77.95 ms after (overview/50 m/2 m).
The corrected drained overview repeat measured 76.46 ms. These single-process
observations have varying load and no counterbalanced repeats. They establish
neither a stable regression magnitude nor a speedup: previously falsely
rejected pixels now actually integrate the cloud column, so unchanged sample
budgets do **not** imply unchanged cost. No CPU tail, allocation, memory-high-water
or complete performance acceptance is claimed.

The first fixed debug test attempt reused a stale baseline shader from a shared
Cargo target after the isolated worktree run. A direct shader probe exposed the
missing new helper; cleaning only the renderer package cache and rebuilding
produced the passing regression. That stale run is retained as invalid evidence,
not hidden as a product failure or counted as a pass. The first workspace
Clippy attempt also caught a new test-only constant-chunk lint; the test now
uses `as_chunks`. Final gate results are recorded in the stage artifact.

All **12** final local command gates passed on **752 unchanged inputs**:
workspace formatting/all-target-all-feature Clippy/architecture, the three
targeted cloud hardware tests, 2,003 workspace lib/bin tests, 98 integration
tests, 79 ignored renderer hardware tests, 43 ignored native hardware/CLI tests,
the current debug server build, WORLD00 numeric regression and native release
build. The required legacy ignored command matched zero tests; actual hardware
coverage is in the renderer/native targets. The rebuilt release binary exactly
matches the captured corrected binary. This is local evidence, not remote CI.

Exact ownership, baseline, source/binary hashes, commands, logs and limitations:
[`artifacts/coastal-cloud-receivers-2026-10-21.json`](artifacts/coastal-cloud-receivers-2026-10-21.json).
