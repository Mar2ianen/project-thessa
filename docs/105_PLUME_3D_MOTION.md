# 105 — Spatial plume motion, rotating nozzle frames and density support

Status: native Medium/High now sweeps the shared participating medium through a
three-dimensional centerline driven by read-only flight kinematics and local
air. Gaussian support and stretched-column density are corrected. A bounded
receive-queue race found during validation is fixed. Actual engine/nozzle
hardware publication, accepted pose history, calibrated optics and KSA visual
parity remain open.

## Inputs belong to the moving source

`Scene::update_plume` reuses `local_air_kinematics`, the same rotating-atmosphere
frame conversion used by flight and instruments. It extracts atmosphere minus
COM velocity, vehicle angular velocity and atmosphere angular velocity in
inertial-aligned render axes. `Cache::frame` evaluates relative air at the
physical nozzle offset, including its point velocity:

```text
u_nozzle = Q_nozzle^-1 * (u_COM + (Omega_air - Omega_vehicle) cross r_nozzle)
```

Position therefore affects pressure/density and the rotating-air sample;
attitude sets the nozzle basis and flow components. A constant translational
velocity alone cannot bend a vacuum jet in its co-moving frame. An angular
velocity along the exhaust axis leaves the mean centerline unchanged while
nozzle roll rotates asymmetric shell structure.

The starter adapter still has its provisional 0.55 m / 480 kg/s / 3550 m/s
methalox deck and physical X-15 offset. This patch does **not** replace those
with live compiled nozzle hardware. Only the motion/environment inputs are now
read from the existing flight state. Assets are not rescaled into physical
nozzle metres a second time, and all astronomical subtraction happens in f64
before GPU narrowing. Existing 1e12 m origin checks and shared Medium/High
centerline stations pass. No force, mass inventory, actuator or clock is written.

## A reduced three-dimensional motion field

The backend-neutral `plume-core::motion::centerline` uses nozzle-local +Z and
emitted axial coordinate `s`. Its constant-current-rate retarded attitude is
`R(-omega*s/Ve) * Z`; displacement is `s` times that axis. For a lateral air
component `w`, projected-width momentum capture of a circular column gives:

```text
age = s / Ve
column mass per metre = mdot / Ve
k = 2 * rho_air * r_exit * Ve / mdot
q = k * abs(w) * age
lateral displacement = w * age * (1 - ln(1+q)/q)
```

The small-q series avoids cancellation. `mdot` is the source's effective flow,
so an allocated operating point must still be supplied with source throttle
one. Zero density removes lateral air coupling exactly. An independent 100,000
step midpoint integration checks the quadratic lateral-velocity equation;
retarded rotation, reflection symmetry, engine-off and invalid inputs also
have CPU tests. This is an independent geometric/momentum closure, not copied
KSA code. The [NASA jet-interaction study](https://ntrs.nasa.gov/api/citations/20110013214/downloads/20110013214.pdf)
is a future crossflow calibration reference, not a claim that this formula
implements that experiment's CFD or all its flow phenomena.

The closure assumes constant nozzle-point translation, current angular velocity,
constant exit speed, a frozen local atmosphere and exit-width capture. It does
not replay accepted nozzle history, include axial counterflow/folding physics,
acceleration/gravity differences, startup/shutdown gas persistence, chemistry,
condensation or impingement. Large rate changes and self-intersecting streams
need a richer model; prettier shader noise cannot supply those inputs.

Native Medium samples nine centers, High seventeen; maximum GPU publication
is thirty-three. No-deformation inputs skip centerline allocation. When sampled
displacement from the old straight field is below 0.1 mm, the original straight
consumer is retained. These are representation choices, not physics writes.
Motion continues when cosmetic `flicker` is disabled. Low remains the existing
cheap impostor; the legacy Bevy volume remains straight.

## Integrate a volume, including camera-inside and disconnected crossings

`VolumeDraw` carries camera-relative XYZ plus emitted `s` at each center and a
unit nozzle-radial reference. A 528-byte enabled-only storage buffer holds the
maximum centerline; draw uniforms are 96 bytes. Profile buffers remain separate
and are reused while only pose/air changes. The producer preview uploads its
48-station optical profile once across all thirty-two frames.

The native curved fragment locates the nearest finite centerline segment and
uses its interpolated emitted coordinate plus spatial radial distance to sample
the same axial optical field. Internal joins are rounded; inlet/outlet planes
are flat. Ray/capsule intersections give conservative support intervals, sorted
and merged before transport. Disconnected crossings are visited front to back
and overlapping support is integrated once. Opaque scene depth clips the ray
before integration. AABB boxes only determine conservative screen coverage.

An initial box-interval version exposed a real camera-basis defect: rotating
the entire scene and camera changed the quadrature endpoints and gave 0.02124
maximum RGBA error on an asymmetric-shell test. Capsule intervals remove that
world-axis dependence. Nozzle-radial axes use shortest-arc transport along the
curve, replacing the previous world-up pole switch. Common scene/camera rotation
now fits 0.002 absolute RGBA: observed maxima 0.00083923 on RADV and 0.00115967 on
llvmpipe. Rolling the nozzle changes the asymmetric shell, as required.

Nearest-segment evaluation is a **single connected field** representation. It
does not add separately overlapping fluids or solve self-intersection mixing;
that remains F07. No multi-nozzle/RT or cloud-composition claim follows from
supporting a spatial path.

## Correct density rather than hiding its silhouette

Two field issues became visible in larger curved views:

1. The Gaussian radial coordinate was clamped at three radii. Beyond that point,
   emission/extinction stopped decreasing, leaving a nonzero pedestal throughout
   the much wider tail bound. Near-camera/vacuum views could expose angular
   support/segment boundaries. CPU, native, legacy volume and the reference now
   evaluate the actual Gaussian decay. At six radii its weight is below 3e-12;
   there is no constant outer-density floor. The finite published support bound
   is retained.
2. Emitted distance and curved arc length differ. Each segment scales extinction
   and emission by `delta_s / length(delta_center)`, retaining column content
   per emitted metre rather than brightening stretched segments. This is a
   per-segment line-density correction, not a proof of global mass conservation
   for intersecting/strongly curved tube volumes. The source-function ratio
   remains unchanged.

The reference metadata explicitly records `radial_weight_mode="gaussian-v2"`
and `curve_metric="emitted-length-v1"`. Old captures retain their declared or
default clamped/unscaled modes; they are not silently interpreted as the new
field. Shock mode remains `axisymmetric-j0-v1` from doc 104. Shared exposure,
source operating points, station optics and TAA defaults are not retuned.

## Actual reference and runtime checks

Final controls use stock Blender/Cycles 5.2.2 LTS `d13f752e3b9c`, CPU/two threads,
2048 samples, seed zero, black environment, no denoise or scattering. Stock
material math describes the closest-segment field, stretch factor and frozen
48-station coefficients; Cycles' integrator is unchanged. Saved `.blend` files,
EXR/HDR, matrices, publication hashes and source states are retained externally.

| Curved compiled-engine case | Native 14 / Cycles 2048, linear RGB RMSE |
| --- | ---: |
| Sea/full with crossflow and angular motion | 0.00046907 |
| Thin/full with crossflow and angular motion | 0.00169218 |
| Matched/inside curved field | 0.00062103 |

All nine straight compiled-engine controls are also rerendered under the new
Gaussian contract; RMSE ranges from 0.00017659 to 0.00166901. The four Gaussian
analytic controls remain byte-identical to the previous shipping controls.
Before correcting the density floor, all eighteen straight 14/16-step outputs
also matched doc 104 exactly; that earlier equality is not claimed for the
subsequent changed field.

An independent f64 world-space reference integrates 8192 uniform cells per ray
and computes segment geometry from the original publication. Eight views include
inside-camera and separated folded crossings, each with/without an opaque
plane: 784 probe rays, maximum absolute RGBA error 0.00584054 against native 14
steps, inside the declared 0.012 envelope. RADV and llvmpipe pass the same probes
and nozzle-frame test, plus the six existing transport/exposure/depth/lifecycle
GPU controls. The two lib GPU oracles also verify the 96-byte uniform layout's
station access and the independent Bessel mode. These are fixture-specific
bounds, not arbitrary-flow or temporal-error guarantees.

The actual native `Cache` and `PlumeRenderer` additionally render thirty-two
640×360 High HDR frames with prescribed translation, angular motion, atmospheric
flow and camera orbit. Every frame retains source/camera/centerline metadata;
linear output and the shared Reinhard/sRGB preview are separate. This is a
kinematic diagnostic, not a measured firing, full flight replay or blind visual
acceptance. No graphics session/window is opened by these headless checks.

## Cost, quality and next work

The effect uses the existing requested/resolved policy. Off produces no frame
or volume work. Medium uses eight steps per merged ray interval and eight
segments; High fourteen steps and sixteen segments. A pathological ray can
therefore use at most 64/224 cells, rather than a promised eight/fourteen over
arbitrary disconnected volumes. `PlumeStats` and native diagnostics expose
centerline segment count/upload bytes separately from profile/uniform traffic.

Two repeated release runs per variant on ten 1280×720 views use 20 warmup and
80 measured frames, with compilers stopped. Final straight p50 spans 0.125–0.702
ms; curved p50 0.504–5.819 ms. Profiles are identical across variants, repeated
HDR is exact within each variant, but geometry/fields differ: this is the cost
of the new representation, not an equal-output speedup or whole-client FPS.
Near-camera High is still expensive. An earlier isolated hoisting change
removed repeated normalization/profile lookup and reduced its old curved-fixture
cost by 4–22% with identical HDR in all forty ABBA outputs. Final support/density
changes are measured separately and are not added to that optimization claim.

Next priorities are actual accepted nozzle/mount/gimbal publication (F03),
accepted motion history and calibrated flow/optics, field-derived hull/ground
light (F05), mixed-medium composition (F07), and accelerated segment/support
queries under the same optical field and error (F08). The full living backlog
is [doc 98](98_GRAPHICS_PARITY_ROADMAP.md).

## Keep sole state publications while the host drains

Broader native tests first exposed eight writes blocked by the read-only test
sandbox. Mounting their declared scratch target outside the checkout resolved
the harness restriction and exposed a separate real PRIMARY receive race:
`try_lock` could discard the only first/stopped snapshot while the host drained
its queue. A targeted contention regression fails before the fix and passes
after it.

The existing two-arrival latest-wins queue now uses the same short receive-thread
lock for PRIMARY and on-change catalogues. Host draining moves at most two items
and releases the queue before presentation work; no lock is held during pipe IO.
Receive timestamps, independent generations, interpolation, wire version and
physics are unchanged. This is relevant to dependable remote pose input, not
an alternative flight scheduler. All 47 flight-net tests and all 15 native server
unit cases pass, including sole authoritative-stop delivery/relaunch; perf and
producer tests pass with the corrected scratch mount. Other native unit cases
from the broad run passed; the evidence retains the initial failures and their
successful targeted recovery.

39 plume-core, 40 render-wgpu lib and 41 Python tests, strict all-target Clippy
for the four affected crates, portable Naga validation, native compilation and
the 44-package/20-foundation architecture gate pass. Full KSA runtime visual,
physical optical calibration and long real-flight temporal acceptance remain
open. Evidence: [JSON record](artifacts/plume-3d-motion-2026-10-31.json).
