# 98 — Graphics parity roadmap: engine plumes and procedural CBT planets

Status: living implementation backlog, audited 2026-11-01. The active goal is
beautiful, causally plausible rocket exhaust and procedural planets from
close-up soil to orbit, with measured frame cost. Checked boxes require linked
evidence; listing a feature does not implement it or establish visual parity.

## Reference audit and implementation boundary

Use other engines to expose missing behavior and make reproducible controls,
not to replace Thessa's renderer or simulation. The first actual adapters are
Cycles B01/B05 and [Unreal B05](97_UNREAL_B05_COMPARISON.md). Godot is downloaded,
not yet rendered. NASA/photos and artistic references can guide appearance;
unmatched optics, exposure, fuel, time or atmosphere make them qualitative
references, not pixel A/B controls.

| Primary reference | Relevant behavior | Thessa implication |
| --- | --- | --- |
| [Large CBT paper](https://arxiv.org/html/2407.02215v1), [upstream demo](https://github.com/AnisB/large_cbt) | CBT manages allocation for persistent neighbor-linked bisectors; conforming refinement/decimation, GPU indexation and indirect rendering form the whole system | Compare topology, allocation/exhaustion and complete update/draw cost at equivalent error; rank-tree microbenchmarks cannot establish planetary parity |
| [KSA October build audit](104_KSA_AUDIT_AND_RADIAL_SHOCK.md), [BRUTAL source](https://github.com/ahwootech/BRUTAL) | Per-nozzle gas dynamics, authored shock templates, merged plumes, transient state and October interval-based cloud composition; BRUTAL itself contains no renderer | Finish actual nozzle publication and medium overlap before adding artistic layers; use independent public mathematics and retain audit/version boundaries |
| [NASA nozzle design](https://www1.grc.nasa.gov/beginners-guide-to-aeronautics/nozzle-design/) | Nozzle geometry controls flow, exit velocity/pressure/temperature | Consume compiled/runtime nozzle state; a fixed artistic source is not a substitute |
| [Unreal heterogeneous volumes](https://dev.epicgames.com/documentation/en-us/unreal-engine/heterogeneous-volumes-in-unreal-engine) | Participating-media transport with separate integration, shadow and cache accuracy/cost controls | Reuse the existing plume medium; validate integrals and lighting, rather than authoring an unrelated cone effect |
| [Unreal sky atmosphere](https://dev.epicgames.com/documentation/en-us/unreal-engine/sky-atmosphere-component-in-unreal-engine) | Planet-scale density profiles, transmittance/multiple scattering and aerial perspective | Validate existing ground-to-orbit transport and terminator continuity with fixed physical inputs |
| [Unreal volumetric clouds](https://dev.epicgames.com/documentation/en-us/unreal-engine/volumetric-cloud-component-in-unreal-engine) | Distinct view/shadow tracing quality, conservative empty-space rejection and cached sky lighting | Audit existing clouds before adding effects; measure per-pass cost and invalidation, retain conservative support |
| [Unreal virtual shadows](https://dev.epicgames.com/documentation/en-us/unreal-engine/virtual-shadow-maps-in-unreal-engine) | Visibility-dependent detail and cache invalidation govern shadow cost | Profile current shadow geometry/publications; importing VSM wholesale is not a prerequisite |
| [Godot standard materials](https://docs.godotengine.org/en/4.7/tutorials/3d/standard_material_3d.html) | Explicit alpha modes and practical sorting/coverage limits | Keep cutout, coverage blending and physical transmission distinct; test foliage and intersecting surfaces |
| [Filament material model](https://google.github.io/filament/Filament.html) | Energy-aware microfacet response, filtering and material conventions | Resolve remaining B01 rough-metal/normal/filtering differences against declared BRDFs, not arbitrary color grading |

Upstream Large CBT main inspected at `7351e6fc603b9b2c2ab4da399b13a9ab0f327398`
(2025-10-26 commit). GitHub reports no repository license. Its DX12 demo is an
algorithmic reference, not a runtime dependency or Linux benchmark executed
here. Existing quarantined CPU FFI pins `091068c6a8`; it is a different, older
oracle scope. Keep production Rust/WGSL independently implemented from the
paper and project contracts; do not copy unlicensed renderer code. Published
console-level paper timings are not measurements of Thessa on 880M.

## Already implemented: preserve and extend

- [x] Shared requested/resolved quality policy, disabled/fallback behavior,
  source-bound HDR/camera/perf capture and reference-scene export (docs 88–90).
- [x] PBR direct GGX repair, MASK coverage, BLEND/emission, opaque ray-hit
  emission, Cycles diagnostics and actual Unreal HDR B05 (docs 92–97).
- [x] Order-preserving transparent instance batching with exact HDR and fixed-work
  Radeon ABBA, [doc 96](96_TRANSPARENT_INSTANCE_RUNS.md). Coastal speedup unmeasured.
- [x] Neutral analytic plume profile/optics, legacy Low impostor, native Low/Medium/High
  volume integration and opaque-depth clipping, [doc 38](38_ENGINE_PLUME_RENDERING.md).
  Current color ramps and client engine deck are provisional, not calibrated.
- [x] Native plume shared pre-exposure correction and a five-candidate lookup
  investigation, [doc 99](99_PLUME_EXPOSURE_AND_LOOKUP_AUDIT.md). All candidates
  regress a near-nozzle case; the existing linear lookup is retained.
- [x] Finite-cell emission transport, independent conservation/homogeneous
  GPU tests and four actual native/Cycles volume controls with a Gaussian
  mathematical oracle, [doc 100](100_PLUME_SEGMENT_TRANSPORT.md). This closes
  an integrator defect, not physical engine calibration or the full F02 gallery.
- [x] First circular radial optical shock mode, independent CPU/GPU Bessel
  oracles, nine stock Cycles controls and release fidelity-cost ABBA;
  [doc 104](104_KSA_AUDIT_AND_RADIAL_SHOCK.md). October KSA inspected statically;
  nonlinear shock/combustion and visual parity are still open.
- [x] Native 3D kinematic centerline, body-frame turbulence, corrected Gaussian
  and stretched-column density; eight-view independent GPU probes, three curved
  and nine updated straight Cycles controls, actual native motion preview and
  measured cost, [doc 105](105_PLUME_3D_MOTION.md). Full history/flow, hardware
  nozzle sources and aesthetic acceptance remain open.
- [x] Independent spatial compression cells, quadratic mean stations,
  validated exhaust gamma, geometric emission dilution, blue/violet methalox
  reference preset, a photometric brightness reference and stable-core/moving-skirt
  sampling; expanded dense
  oracles and stock Cycles diagnostics, [doc 106](106_SPATIAL_SHOCK_FIELD.md).
  Nonlinear Mach disk, radiometric emission and visual acceptance remain open.
- [x] Persistent GPU bisector pool, neighbor conformity, parallel bounded
  split/merge, canonical height/material sampling, GPU indexation and indirect
  raster/shadow/ocean/RT publication (docs 61–63). WORLD13 native configurations
  select it. The older missing-feature list in doc 22 is historical; do not
  rebuild this as classic tile LOD or mistake residency for topology.

## First implementation sequence

P0 items run first. P1 supplies the next fidelity gain after its prerequisite.
Each row is a bounded change in existing owners, not a new generic framework.

| ID / priority | Current gap and owner | Next work and acceptance |
| --- | --- | --- |
| F01 / measured, closed | `render-wgpu` plume lookup scans up to 64 stations per sample | Five exact-output alternatives rejected after release ABBA on ten views; retain linear lookup. [Doc 99](99_PLUME_EXPOSURE_AND_LOOKUP_AUDIT.md) records p50/p95/p99 and hashes. Reopen only with a changed workload or a new measured reason |
| F02 / partial, P0 | Pre-exposure and finite-cell emission fixed; four constant-axial controls (doc 100) and nine compiled-methalox pressure/throttle/view controls (doc 102) have actual Cycles comparisons; the reference inside-camera ray start is corrected; the radial mode and 3D motion have independent/stock Cycles controls (docs 104–105), including eight discrete views and a prescribed native motion preview; spatial compression and near-nozzle dense controls now exist (doc 106); finite-pixel filtering and explicit same-filter Cycles controls resolve dominant thin-core aliasing (doc 107) | Extend the frozen gallery to continuous camera/time trajectories, more engine families and general heterogeneous-field convergence. Include continuous axial/side/inside motion, depth occlusion and astronomical origin changes. Existing chord tests do not certify arbitrary animated views |
| F09 / native implemented | Native Low consumes the same optical volume at 6 base steps / 1 spatial ray, with the same source/profile and up to 9 motion points. Positive daylight emission, EV13/5 scaling, opaque depth, unchanged authority and lower cost than Medium are verified in doc 108 | Legacy Low remains a separate texture path. Low point sampling still aliases unresolved compression sheets; general angular/temporal/optical calibration is not complete (F02/F04). Preserve the measured accuracy/cost boundary when refining it |
| F03 / P0 | `terrain-native/src/plume.rs` uses fixed radius, flow, exhaust state and mount; body/rotating-air kinematics now drive 3D motion (doc 105), while environment assumes oxygen | Publish accepted per-nozzle operating points/mounts from the existing resource allocation into presentation; accepted liquid/solid/chamber observations are retained without another solve (doc 101), now with authored circular outlets, gamma, allocated plume state and COM rebasing ([doc 109](109_ACCEPTED_NOZZLE_OUTLETS.md)). Bind owner/frame revisions and publish/consume them, rather than recomputing from a requested throttle. Legacy/annular missing geometry stays explicit. Use composition/flow queries. Pin SI/frame transforms and inactive/starved engines; no renderer writes to forces, mass flow or clocks |
| P01 / partial, P0 | Frozen production ground→orbit→ground, closedness, canonical arithmetic and overload recovery now pass; land-only scratch optimization has exact-output release ABBA ([doc 103](103_CBT_FLIGHT_AND_SCRATCH.md)) | Extend to dense equal-error geometry and production multi-level residency, astronomical/body-frame shifts and close soil. Profile shared update/raster/shadow/ocean/RT separately under the same source/error. The fixed six-root-page control is not full Large CBT parity |
| F04 / P1 | Optical templates are authored RGB ramps; absolute spectral/radiometric calibration absent | Separate regular cells from Mach reflection/Mach-disk topology using consistent nozzle/stagnation inputs, not an arbitrary exit-to-ambient pressure threshold. Calibrate declared exhaust families against physical input ranges and primary observations; preserve pressure-dependent expansion and throttling. Separate model uncertainty from integration error. Do not equate pretty bloom with validated chemistry |
| F05 / P1 | Native plume has no field-derived local illumination; only legacy nozzle light exists | Declare radiometric emission units and integrate the actual radial/spatial field; the historical `radiant_power` proxy is not that integral (doc 106). Build its CDF/PDF, sample emissive volume points, use existing FastRT geometry visibility plus volumetric transmittance, and compare diffuse/metal receivers with occlusion. Keep a reference sample-count tier and Off cost; no unrelated point-light substitute |
| P02 / P1 | WORLD13 uses sampled edge/chord error rather than a universal geometric bound | Compare against dense canonical geometry along close-up/horizon/orbit trajectories; detect missed extrema and popping. Improve the error estimator only where measured; hold source/residency and pixel error constant during topology performance comparison |
| P03 / P1 | Close soil/material detail and large-scale variation still need visual acceptance | Separate geometric microrelief from normal/roughness and geological macrovariation; filter each by footprint. Fixed centimetre→kilometre views, grazing light and movement must retain substrate character without tiling, swimming or view-dependent physical height |
| P04 / P1 | Shadow triangle submissions and full-capacity RT publication remain expensive | Profile dirty surface/height/material/ocean epochs; reduce redundant work in existing consumers at equal geometry/error. Preserve shared origins and same-frame visibility; use docs 84–87 rejected experiments as constraints, not claimed optimizations |
| P05 / P1 | Atmosphere/cloud/ocean paths exist, but ground↔orbit and terminator comparisons are incomplete | Match planet radius, gas/density, sun angle/intensity and camera to reference controls; validate LUT limits, horizon seams, shadows/reflections and precision. Profile with effects isolated and together |
| S01 / P1 | MASK/BLEND shadow construction fails; nonopaque RT falls back | Add coverage-aware shadows/ray visibility with the same alpha/UV/material contract. Compare a leaf to actual cut geometry; prevent rectangular shadows, depth leaks and solid-card reflections |
| S02 / P1 | B01 rough-metal model and B02 normal/filtering differences remain | Explicitly match tangent basis, texture color space, normal strength, energy treatment and footprint filtering. Render original full Fern/Moss tiers with declared material derivation; source asset cuts are not an equal-work speedup |
| S03 / P2 | Intersecting alpha layers, glass/transmission and emissive textures unsupported | Add only the required representation behind existing capability/quality resolution, with same-scene engine controls for overlap, absorption, light sources and occlusion. No silent opaque substitution |
| F06 / partial, P1 | Native has reduced lateral momentum capture and current-rate retarded 3D attitude (doc 105); actual nozzle history, axial counterflow, chemistry, condensation and impingement remain absent | Extend accepted nozzle history and causal atmosphere/composition/wall inputs with calibrated flow references. Pin rate changes, ignition/shutdown persistence and uncertainty separately from rendering error; no shader noise substitute |
| F07 / P1 | Native plume and clouds do not share a depth-interval/mixed-medium composition contract | Follow the public deep-compositing reference identified by the October KSA audit (doc 104). Reuse existing cloud/plume consumers, publish conservative intervals, and compare identical/disjoint/overlapping emission-plus-absorption to direct integration. Handle zero opacity, dense media, opaque clipping and disabled cost; alpha sorting is insufficient |
| F08 / P1 | Swept-volume nearest-segment queries and near-camera High remain expensive (doc 105) | Reuse existing bounded region/query owners for hierarchical support/segment rejection. Hold the entire optical field, pose trajectory and representation error fixed; use release ABBA plus independent geometry/transport probes. Do not gain speed by dropping motion or density |
| S04 / P2 | Moving-camera temporal acceptance open; TAA default-off | Use disocclusion/thin-foliage/plume trajectories, frozen exposure and independent reviewers. Do not enable history to hide topology, coverage or transport errors |

F01 yielded diagnostics rather than an accepted optimization. F02/F03 and P01
are the next substantive plume/planet acceptance tasks. Native B02 and Godot
remain useful controls, but do not displace the user's plume/planet priorities.
Reinhard/PBR-neutral tone mapping, automatic exposure and bloom already exist;
adding a second unverified implementation is not a backlog item.

## A/B rules and completion criteria

1. Freeze geometry/field, camera, time, atmosphere/light, exposure, residency,
   settings and source/binary hashes. Separate content changes from optimizations.
2. Numerical tests check analytic/special cases, HDR finiteness, integration
   error and frame/units. External images declare import/BSDF/unit differences;
   avoid fitted exposure or material changes to lower error.
3. Performance uses retained release binaries, warmup, ABBA, per-frame samples
   and GPU intervals. Record CPU submit/wait, GPU pass/full-frame and memory
   separately. No concurrent shader build, clock/driver changes or FPS derived
   from summed isolated passes. No universal 880M frame-rate promise.
4. Temporal tests use continuous trajectories and recovery/invalidation cases;
   static snapshots do not establish stability. TAA stays default-off.
5. Blind preference packs hide origin labels, keep keys separately, allow ties
   and invalid pairs. Independent votes, physical correctness and performance
   are separate acceptance results. No self-awarded aesthetic parity.

The goal remains active until its scoped accepted milestones and remaining
limits are explicit. Every implementation updates this list, feature docs,
README/status and evidence in the same commit, preserving user/Gemini edits.


Doc 107 verification adds 12 frozen straight/curved BOX-filter comparisons,
independent dense pixel quadrature and a 32-frame native motion preview.
This is spatial transport evidence, not blind preference or real-engine optics
acceptance. The initial unconstrained smooth-skirt supersampling regressed the
far curved vacuum control; the retained feature budgets and measured costs
are documented there. Do not reuse historical point-sampling pass times as
current whole-frame performance.


Doc 108 completes native F09 without a fitted display gain: all native tiers
share the optical field and differ in representation budget. Six steps pass
180 normalized chord controls; the four-step candidate was rejected. Low
remains less accurate than Medium/High in axial and subpixel controls, and the
legacy client still has independent Low optics. This does not close F02/F04.
