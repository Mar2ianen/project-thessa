# 110 — Native reference scenes and accepted multi-nozzle integration

Status: partial integration, 2026-11-02. Gemini's `feat/cycles-reference-parity`
workspace at `3e9e30c` was reviewed against `5715435`, preserving the newer
material, transport and source contracts. This does not establish visual parity.

## Reference-scene integration

Native `--reference-scene b02 --reference-asset /absolute/cache/fern.glb` now
uses the neutral `VegetationLab` data and the existing mesh/display/capture
owners. The original pinned evaluated Fern geometry is retained; B01 and B05
remain available. `--scene b03` selects the canonical coastal survey defaults
and the surface-showcase profile unless an explicit graphics file is supplied.
Explicit survey coordinates still override defaults. Window and headless hosts
use the same scene path.

`--export-canonical-coast /absolute/new-directory` exports the actual GPU-active
conforming terrain, retained plants/ground meshes, camera and stellar inputs.
It requires headless capture and a new destination. Exported camera near/far,
exposure and extent come from that capture; they are not hard-coded reference
values. Readback validates counts, finite positions and indices. Files use
exclusive creation. Generated game geometry keeps GPL-3.0-or-later terms;
third-party material/asset notices remain applicable. Terrain UVs/materials
remain approximations, not a lossless export of the native procedural shader.

The Cycles B03 adapter consumes the exported extent and pre-exposure. Its
procedural mode is an independent **material approximation** using the same
geometry/camera, not the same native BSDF/sky/ground implementation. The optional
`golden_photo` mode projects target pixels into emission on the ground/sky.
That is an art study, not an independent renderer reference. It is explicitly
labeled and excluded from the default blind manifest. Gemini's imported
93/94/95 documents and 2026-11-01 dataset are dated branch records; current
implementation and acceptance are described here. The branch's roughness-only
GGX compensation fit was not accepted into the shared production kernel.
Native botanical enrichment equivalent to its Cycles art study remains open.

## Accepted source, wire and frame ownership

Wire version **11** adds optional `RocketPlumeSample` to `Snapshot`. Both peers
must upgrade; version 10 is rejected. Samples carry vehicle identity, accepted
interval endpoint/duration, operating-point pressure, endpoint atmosphere/O2,
relative air motion, air rotation and definition-local nozzle observations.
They reuse authority's accepted resource allocation and existing atmosphere
queries. Reading a snapshot does not advance physics or solve propulsion again.
Validation bounds the list at 1,024 nozzles, rejects duplicate IDs, wrong owners,
non-finite inputs, invalid unit axes and inconsistent outlet radii. Old baked
vehicle definitions without physical nozzle geometry remain loadable and do
not invent circular outlets. Fleet samples carry/validate their own owner;
secondary fleet plume rendering is still pending.

The stdio reader retains a bounded two-arrival **complete primary frame** inbox:
a snapshot plus its exact resolved geometry revision. A later descriptor cannot
steal an already received pose. A snapshot arriving before its descriptor has
one bounded pending slot and retains its original receive timestamp when the
geometry arrives. The existing interpolation policy is reused; accepted flow
and its interval stamp are not interpolated or throttled again. Missing geometry
preserves the last complete visual frame under the existing stale timeout.
This adds neither a second simulation clock nor a prediction/fallback solver.

The native consumer builds actual primary-vehicle nozzle sources from that
sample and the matching presentation pose. Positions, exhaust axes, relative
air velocity and angular rates are transformed into the same planet-body scene
axes as the retained hull. A rotating planet and a large inertial origin are
covered by the source-placement test. Postburn COM translation changes source
placement without rebuilding an otherwise identical optical profile. Missing,
inactive or starved sources clear their consumer. Only the known scalar starter
retains the explicitly provisional X-15 deck.

Server and native `--vehicle-definition /absolute/baked.json` use the existing
compiled-vehicle path; the native option requires `--server-flight` and passes
the path directly as a child argument. `data/vehicles/reference_twin_rocket.toml`
is an authored two-engine fixture for the existing vehicle baker. It is not a
flight-ready calibrated engine model.

## Joint medium transport

One active source retains the existing volume path. Multiple primary nozzles
use one joint optical pass with the same station interpolation, spatial shocks,
radial field and curved support. The pass merges source integration boundaries,
sums emission and extinction at each cell, then applies Beer–Lambert transport.
Independent alpha-sorted jets would give incorrect overlap colors/attenuation.
This follows the participating-media transport described by
[PBRT](https://www.pbr-book.org/4ed/Volume_Scattering/Transmittance).

The backend has an explicit 64-field capacity and errors before allocation on
excess input. It does not silently drop sources. Per-field bounds reject
invisible sources/rays. Finite-pixel samples average after transport; opaque
depth clips the field. Existing Low/Medium/High budgets and settings own the
representation; no new physical coefficients or graphics toggle are added.
Draw/Frame remain 112/160 bytes. The joint consumer is lazy; disabled/absent
plumes use the existing release path. Telemetry adds admitted `source_count`.

## Executed evidence and limits

Raw records are under the local session's
`outputs/render-integration-2026-11-02`; the portable digest is
[the integration record](artifacts/native-scene-nozzle-integration-2026-11-02.json).
Each capture records commands, binary/source identities, PNG/HDR and trace logs.

- Native B02: actual RADV 880M, 640×480, original 6,232-triangle Fern.
- B03 bounded diagnostic: 640×360, Low plant density, understory disabled,
  one raster cascade (three stellar layers), 500 m shadow domain. It exported
  8,013 plant/ground instances. Local RT was active with 8,077 instances and
  384,104 triangles. These are explicit workload reductions, not High parity.
- Actual independent Cycles CPU B03: the same bounded export/camera/extent/gain,
  procedural materials, 8 spp. Pre-exposed RGB MAE 0.1088501, RMSE 0.1394591,
  relative L2 0.3151161. This noisy control exposes a substantial material/light
  gap. It is not a full-quality or performance comparison.
- Full High B03 shadow submission was rejected **before stage submission** at
  207,471,832 mesh triangles against a 20M diagnostic admission limit. Even
  one cascade/100 m gives 61,857,162 triangles at 1 texel error; 4 texels gives
  47,726,996. Those configurations failed admission; no speedup is claimed.
- Live accepted two-engine native/server run: retained hull, matching geometry
  revision, two admitted physical nozzle fields in one pass. Each engine
  received 31.1253 kg/s at requested throttle 0.6. The final display still has
  weak visible exhaust: the allocated exit temperature is about 1,437 K and
  the provisional graybody optical model lacks calibrated chemiluminescence.
  Source delivery is verified; attractive/realistic flame appearance is not.
  The 106.8 ms trace-fenced median is a diagnostic, **not game FPS**.

CPU/wire/renderer, strict Clippy, architecture, Python and GPU results are listed
in the artifact after final checks. The joint GPU oracle covers homogeneous
coincident fields, partial overlap, off-axis Gaussian columns, collinear swept
support, order, opaque clipping and excess-capacity rejection. These controls
do not certify arbitrary curved/heterogeneous overlap or temporal stability.

## Next work

F03 remains partial for fleet rendering, active gimbal geometry and transient
interval history. F04 needs calibrated thermal/chemical emission rather than
raising temperature/exposure to conceal the weak real-source flame. F05 field
illumination, F06 history, F07 cloud/plume shared transport and F08 near-camera
cost remain open. Clouds, spectral ocean and local mesh RT already exist;
primary terrain/ocean scene-object RT receivers are still missing. No water
force, fluid solver, cloud physics or universal RT integration is added here.

For P04, profile the existing shadow/RT owners before expanding the feature set.
[Unreal's VSM documentation](https://dev.epicgames.com/documentation/unreal-engine/virtual-shadow-maps-in-unreal-engine)
explains receiver-driven detail, invalidation and the cost of foliage; it does
not mean Thessa already has a virtual shadow cache. Preserve physical stellar
inputs and declared quality/error in any future performance comparison.
