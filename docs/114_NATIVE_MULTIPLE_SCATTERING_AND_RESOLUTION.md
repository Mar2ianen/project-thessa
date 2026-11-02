# 114 — Native higher-order sky source and resolution/reference checks

Status: implemented optional isotropic multiple-scattering closure; coastal
visual/cost parity remains incomplete. 2026-11-02. This supersedes the native
single-scattering-only limitation in docs 112/113. It does not close cloud or
terrain bounce, ground-to-orbit acceptance, the horizon defect, or the user's
full sunset-forest target.

## Existing owners and the missing source

The native adapter now consumes the existing resolved
`atmosphere.multiple_scattering` flag. Backend-neutral `thessa-atmosphere` builds
an optical source table; `EnvironmentField` caches it and evaluates the same
sky-direction grid used by background, hemispherical diffuse/SH, mesh/terrain/
ocean reflections and ray misses. Existing cloud consumers receive the changed
environment boundary. There is no new renderer, simulation state, light, GPU
binding, image or pass. No target pixels or fitted warm/cool tint enter this
production calculation.

The independently implemented closure follows the isotropic higher-order
approximation described by [Hillaire's sky-atmosphere work](https://sebh.github.io/publications/)
and its [author implementation](https://github.com/sebh/UnrealEngineSkyAtmosphere):

`Psi(h, mu_s) = average(L_single_isotropic) / (1 - average(F_scatter))`.

`L_single_isotropic` is the angularly averaged first-order response to unit
stellar irradiance with isotropic phase. `F_scatter` integrates the response to
a unit isotropic incident field. Both reuse the existing atmosphere clipping,
f64 SI coefficients, density profiles and cached solar extinction. Each stellar
source contributes its own RGB irradiance and visibility. Direct first-order
sky scattering retains Rayleigh/Henyey–Greenstein phase. The higher-order source
is locally isotropic, with black ground; cloud/terrain bounce and anisotropic
higher orders are not simulated. The source's altitude coordinate is quadratic
and the stellar zenith cosine is linear; bilinear lookup uses f32 source knots
inside f64 integration. Invalid domains, excessive work, nonfinite/unrepresentable
sources and nonconvergent feedback are rejected rather than brightness-clamped.

A shared integration defect also needed correction: source previously used the
left-end transmission times the full step length. A thick cell could then
scatter more than its incident energy and make feedback exceed one. Source now
uses the exact constant-cell integral `-expm1(-sigma * ds) / sigma`, with `ds` in
vacuum. **This also changes the MS-Off first-order result.** An old-binary/new-
binary image difference therefore cannot be attributed entirely to the flag.
Extinction remains the same integrated optical depth; source/feedback use the
same cell transport. Vacuum, half-absorption and segment composition tests pin
this correction independently of the closure.

## Policy, cost and lifecycle

| Resolved atmosphere quality | Source knots | Angular directions | View steps | CPU source bytes |
| --- | ---: | ---: | ---: | ---: |
| Low | 16×8 | 16 | 16 | 1,536 |
| Medium | 32×16 | 32 | 24 | 6,144 |
| High | 64×32 | 64 | 32 | 24,576 |

The table rebuilds only when optical state or its integration budget changes;
ordinary camera, stellar pose, exposure and emission changes reuse it. Off,
atmosphere disabled or no optical atmosphere releases the source and does no
higher-order work. A failed build warns once and retains single scattering;
a changed budget permits a retry. The existing sky-grid cadence remains in
force. Startup is synchronous: the actual debug High coastal run spent
1,896.383 ms building this table, once. Its initial whole environment evaluation
was 2,175.86 ms; these are CPU startup diagnostics, not steady GPU cost or a
release benchmark. Telemetry reports activation, bytes, build count/time and
maximum feedback. The captured atmosphere's maximum feedback was 0.652996.

## Executed correctness and native controls

[The artifact](artifacts/native-multiple-scattering-resolution-2026-11-02.json)
records hashes, exact commands, raw images/HDR, profiles and logs. All
67 atmosphere tests pass; 10 native environment CPU tests pass (three
GPU environment controls remain ignored in that ordinary invocation). Strict
Clippy for atmosphere/render-wgpu/native and the existing Bevy client compile
check also pass. Four new controls include vacuum/domain/work guards,
finite-cell conservation/composition, multi-source additivity, eclipsed-source
scaling, rotation invariance and unchanged transmission. The native cache test
covers enable/off, unchanged input, moved camera/light and emission-only reuse, changed density and
atmosphere disable.

The actual High table's maximum absolute source error was
**0.0009262782 per unit stellar irradiance**, below the pinned 0.0015 bound,
against independent 16×32 tensor solid-angle quadrature with 128 view steps at
six altitude/sun-angle coordinates. A 32×16/64-direction/64-step table gave
0.0010420163. This measures quadrature/lookup error of the **same declared
isotropic closure**, not model error against full multiple-scattering Monte
Carlo, all planetary atmospheres, or low-tier accuracy.

An ignored native GPU test passed on RADV 880M and isolated software Vulkan
(without `/dev/dri`); both exclude display access: 257×257 terrain plus the retained X15 mesh, frozen
camera/time/source lighting, no streaming tick or topology dispatch between
Off/On/Off. Terrain vertices, mesh epoch and primary instance/triangle/draw
counts remain identical; ambient and more than 100 pixels change; Off restores
the original image byte for byte and releases the source. This establishes
native publication and disable behavior on these two Vulkan implementations, not coastal visual acceptance. The
first test attempt incorrectly compared first-publication upload byte counts;
that assertion was corrected to compare geometry/work while excluding uploads.

## Full-population FHD observations

Actual 1920×1080 native frames retain 17,992 mesh candidates, 19,652,948 candidate
triangles and 2,768,212 primary mesh triangles in 22 API draws. The profile uses
four-sample MSAA, RT Off, EV11, the canonical camera and physical 12° descending
sun, with the declared diagnostic 100 m / 1024 / one-cascade shadow tier. The
original 12 km/four-cascade High shadow tier is not being benchmarked.

With the source enabled, ambient RGB changes from approximately
`[3867,3429,1866]` to `[5003,5642,5557]` lux and shadows/sky lose some orange
monotony. The complete scene still looks far below the target: pale cloud/sky
structure, repetitive sparse middle masses, an elevated artificial horizon
band and coarse/ambiguous wet ground remain visible. The horizon changes shape
between runs; **it is still a defect**, not accepted as repaired by doc 112's
sentinel correction. Current-frame height/depth/residency diagnosis is required
before accepting another fix.

These separate captures have different terrain residency/topology (11,232
versus 6,772 active terrain triangles), and the successful retry has one measured
frame versus three for Off. They are qualitative full-population images, not
an equal-work speedup or numeric image parity pair. The first enabled full run
was interrupted: outer exec returned 143 after the last recorded opaque-stage
submission intent at warmup frame 11. No completed image/timing is claimed from
it; no kernel GPU reset was observed. A rejected `--warmup` attempt failed
argument validation before GPU setup. The successful retry preserves the
standard 30 warmup frames.

## 720p, FHD and QHD

The earlier source-bound `c7e7880` capture series tests resolution with the same
full mesh population, profile, camera, epoch and exposure, before this source
was implemented. It retains exact commands, frozen runtime source/binary and
profile hashes in `outputs/fhd-qhd-2026-11-02`.

| Output | Trace-fenced wall median, ms | Active terrain triangles | Resident height pages | MSAA target bytes |
| --- | ---: | ---: | ---: | ---: |
| 1280×720 | 570.144 | 6,348 | 748 | 44,236,800 |
| 1920×1080 | 861.648 | 10,626 | 726 | 99,532,800 |
| 2560×1440 | 1206.419 | 14,314 | 706 | 176,947,200 |

These are serial-stage diagnostic wall times, **not game FPS, GPU timestamps,
or equal-work resolution scaling**. Resolution changes CBT refinement and
asynchronous residency. QHD improves thin leaf readability but cannot repair
lighting, habitat composition, false horizon or ground semantics. FHD is a
useful review size; no default/output resolution setting was changed.

## Cycles and generated artwork: use both in their actual roles

The user's generated images are explicitly accepted as visual art direction.
They guide warm sun/cool sky separation, cloud structure, layered botanical
masses and readable wet ground. They are not geometry, radiometry or physical
truth. The located `cycles_b03_golden_v33` image is a useful visual target in
this role: its golden mode projects target sky/ground pixels and adds foliage.
Its own target-image metrics are circular and are not renderer parity evidence.
A preserved copy and hash are recorded beside the new native frames.

The independent old `cycles_b03_procedural` 64-sample result is also preserved
and inspected: it has the older roughly 27.9k-object botanical population and
an approximate procedural floor. It is useful for assessing silhouettes,
grounding and the old adapter's material behavior. It does not share current
CC0 geometry, all stellar sources, solar extinction, sky/cloud transport or
native soil/water semantics. The adapter includes approximate lighting gains.
No old RMSE is relabeled as a comparison against the current native frame.
A fresh physically matched export/Cycles adapter remains required for numeric
coastal parity; artwork remains useful throughout that work.
