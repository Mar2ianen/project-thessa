# 111 — FastRT visibility policy and primary shadow receivers

Status: implemented opt-in receiver, not coastal visual/performance acceptance;
2026-11-02. The current priority is the user's full-density sunset forest,
wet ground/puddles and structured sky at affordable cost. Low-density B03 is
only a bounded technical diagnostic, not the appearance target.

## One scene and one query owner

The existing `RayScene` remains the sole mesh/terrain acceleration owner.
Primary ground and spectral water can now consume that same TLAS in their
ordinary opaque pass. No second tracer, BLAS/TLAS, geometry copy, depth prepass,
simulation clock or force path is introduced. The existing frame order already
emits the CBT surface and builds the scene before primary shading.

The new primary receiver has explicit compatible layouts and rebinds the same
sky/cloud/ocean publications. Missing, empty, unavailable or differently rebased
scenes retain the existing raster/cloud result. Negotiated limits gate the
optional pipeline. The source adds a 16-byte bias/distance uniform only when
the receiver is requested, charged to the existing RT scene budget. A shared
binding references the existing TLAS. `PrimaryShadowStats` reports allocation
and actual last-pass use; native captures print `primary_rt_shadows`.

## Default-off quality/cost boundary

```toml
[renderer]
ray_tracing = "local"
[shadows]
enabled = true
[raytracing]
primary_shadows = true
```

`primary_shadows` defaults to false and is registered in the shared requested,
resolved and metadata pipeline. Both RT and shadows must be enabled. Existing
shadow quality/sample policy remains the raster fallback. Scene admission and
normal-distance/range exclusions remain authoritative renderer inputs; quality
does not change the physical geometry. The option needs renderer recreation.
When disabled it creates no primary pipeline, binding or parameter buffer.
Bevy retains its existing consumer; its resolved RT toggle mirrors the new flag,
but this receiver is currently native-only. TAA remains default-off.

This correction is deliberately **not enabled by default** after measurement:
querying the scene for every lit primary sample is too expensive to call an
affordable FastRT solution. It remains an oracle/fallback foundation while
cheaper representations and confidence gates are developed.

## Shared sparse-query gate

`thessa-fastrt::visibility` owns a backend-neutral f64-SI decision and its
portable scalar WGSL counterpart. An already occluded estimate, back-facing
surface or empty configured ray interval needs no geometry query. A lit/partial
estimate still needs correction; a lit shadow-map texel does not prove that a
thin occluder was represented. Invalid CPU inputs are rejected. GPU consumers
retain a fail-closed query gate for invalid/non-finite intervals.

Mesh, primary terrain and water use the same traversal and gate. The direct
and specular ground terms reuse one per-star visibility; the spectral water
hook receives that same array instead of querying again. Raster-only defaults
and unsupported-device fallback preserve the existing path. Analytic legacy
water keeps its established receiver-normal convention.

This is a small executed sample-level policy, not an SSR implementation,
tile-ray allocator, temporal reuse system or complete FastRT implementation.
[Doc 39 §15](39_BEVY_EXIT_AND_ENGINE_MIGRATION.md#15-fastrt) still defines the
larger cheap-estimate → confidence → sparse-query architecture.

## Evidence

The portable record is [the receiver artifact](artifacts/primary-rt-shadows-2026-11-02.json).
Raw source/binary identities, logs, commands, image/HDR and exported GLB are in
this session's `outputs/primary-rt-shadows-2026-11-02`.

- Actual RADV 880M primary ground and spectral-water controls deliberately omit
  a small blocker from raster shadow casters. A red stellar light is blocked
  without changing a blue stellar light. Removing/restoring/moving the blocker
  restores exact pixels. An empty scene cannot retain a stale blocker. RT miss
  equals the raster-only control byte-for-byte. Water is verified by its actual
  water/motion-invalid alpha, not merely by allocating an ocean field.
- Ground centre RGB: clear `[145,22,123]`, red source blocked `[18,22,123]`.
  Water: clear `[85,12,59]`, blocked `[7,12,59]`. Ambient/other-star light stays.
- Twelve GPU scalar-policy controls on RADV and llvmpipe match the independent
  f64 policy, including non-finite/invalid input and interval boundaries.
- Existing 28 scene RT, 6 surface-lighting and 42 regular renderer-library
  controls pass on RADV in the recorded pre-opt-in build; final gated checks
  are listed in the artifact. Core geometry/shader tests and strict Clippy
  cover the shared hook signature and explicit ABI.
- Actual conforming CBT B03 with 4× MSAA, sky/clouds/ocean and local scene RT ran
  at 640×360, with 8,077 TLAS instances and 384,104 source triangles. Primary
  receiver allocation/use were both true. This capture predates the new
  default-off flag; replay it with `primary_shadows=true`. It is the same
  correction path, not evidence of default-on production behavior.

The three trace-fenced measured frames have a 272.243 ms median, **not game
FPS**. In the last three frames, opaque/MSAA submit-to-fence intervals are
about 73–75 ms, surface 20–22 ms and cloud/sky overlays 25–29 ms. These include
scheduling/fence overhead and are not isolated GPU timestamps. Shadow encoding
also costs CPU time. This identifies expensive stages; it is not a speedup.

The old/new B03 exports differ in streamed CBT triangles (6,258 vs 6,134), even
though the plants/camera/lights agree. Therefore the old Cycles result cannot
be relabeled as a matched new reference or used for an equal-work A/B claim.
The sparse diagnostic still does not match the full reference composition.

## Next: affordable reference-scene quality

The user's latest image is an appearance target with rich vegetation, golden
low-angle light, blue/gold clouds, contact structure, wetness and puddle
reflections. It is not an identical optical/geometry numeric oracle.

1. Restore the full botanical composition in the review scene. Keep projected
   source/error and geometry fixed in performance comparisons.
2. Split/cache static mesh shadow depth while CBT terrain updates, preserving
   exact depth composition and explicit memory/admission budgets. First profile
   opaque shading and cloud overlays too; shadow triangles are not the only cost.
3. Direct native surface light currently uses top-of-atmosphere stellar RGB.
   Reuse the existing atmosphere transmittance model to supply local sunlight;
   select a reproducible physical sunset epoch instead of grading a white sun.
4. SSR/contact depth traces and sparse RT correction need actual current-frame
   receiver/radiance data, confidence, range and edge/disocclusion rules.
   Do not label stale/ref-photo pixels, environment-only specular or geometry
   classification as implemented SSR.

Primary optimization references:

- [GPUOpen SSSR](https://gpuopen.com/manuals/fidelityfx_sdk/techniques/stochastic-screen-space-reflections/): current-frame color/depth/normals/material data, hierarchical traversal, tile classification and reconstruction; use its separation of responsibilities, not unverified performance numbers.
- [McGuire/Mara](https://jcgt.org/published/0003/04/04/): perspective-correct screen ray traversal.
- [GPUOpen Hybrid Shadows](https://gpuopen.com/fidelityfx-hybrid-shadows/): shadow maps plus selective ray correction.
- [Nubis](https://www.guerrilla-games.com/read/nubis-authoring-real-time-volumetric-cloudscapes-with-the-decima-engine): cloud shape, atmospheric integration and reconstruction cost.
- [Hillaire 2020](https://sebh.github.io/publications/egsr2020.pdf): reusable atmospheric transport/lookups across ground and space.

These are algorithmic references. No external SDK/source was imported, no
cross-engine speed ratio or blind preference is claimed, and no visual parity,
water force/fluid physics or calibrated plume emission is completed here.
