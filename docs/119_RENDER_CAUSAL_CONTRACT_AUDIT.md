# 119 — Render causal contracts: review of the post-diagnostic Gemini changes

Status: audited `bf8635a`; synthetic primary-sun indirect lighting removed,
specular-disable contract restored, unsupported fade tuning reverted. Full
forest visual/performance parity is not achieved. This follows the user's
request to address fundamental problems rather than accumulate appearance
iterations. Generated artwork remains valid visual direction.

## Frozen review scope

The previous checkout changed branch while work continued: doc 115's diagnostic
milestone was committed as `6064cc0`, followed by `977c3f8`, `7961c7d`,
`87fe9b3`, `4681672`, `ca9c0ad`, `a26e045` and `bf8635a`. The original showcase's
eight edits were independently committed as `a2527af`. They are no longer
uncommitted changes requiring the older eight-file integration procedure.
This review uses an isolated checkout pinned to `bf8635a`; it does not reset
another workspace or merge into main. Source/binary/command identities and
controls are in [the artifact](artifacts/render-causal-contracts-2026-11-02.json).

## Backend routing is part of the contract

The separate `fix/cloud-coastal-visual-overhaul` worktree at `7e588be` changes
`assets/shaders/cloud_volume.wesl`, the Bevy client and Bevy GPU tests. Native
B03 instead consumes `apps/terrain-native/src/sky_layers.rs` and
`crates/render-wgpu/src/clouds.wgsl` through `clouds::SHADER`. A cloud improvement
in that branch does not automatically affect native B03. This audit verifies
that routing distinction; it does not claim to validate every cloud algorithm
in the other branch. Numerical/visual/cost evidence must identify its actual
consumer before it can establish integration or parity.

## Verified regressions and corrections

### Stellar slot order selected indirect illumination

Primary terrain and meshes synthesized warm indirect light from
`sunlight[0] * albedo * (1 - visibility[0])`, with coefficients reaching
0.30/0.28. They also dimmed the entire sky hemisphere by 48%/45% according to
that same source's direct shadow. No neighboring reflector, visibility path or
sky solid-angle integral supplied these terms. A receiver's own albedo was
used as its presumed reflecting neighborhood. The terms were absent from the
shared atmospheric field and secondary material transport.

A frozen native mesh/atlas fixture exposes the error without any art-reference
metric: keep geometry, camera, environment, material and one physical star
unchanged, moving only that star between the three array slots. Before the fix,
a shadowed pixel changes from **[150,58,83]** at slot 0 to **[141,63,89]** at
slot 1, and the full images differ. The same world cannot depend on this
bookkeeping choice.

The synthetic source and canopy factors are removed in both primary consumers.
They again use the existing environment irradiance plus existing material AO;
real indirect visibility/bounce remains unimplemented here. After correction,
all three slot placements give **[141,63,89]**, with byte-identical full fixture
images on software Vulkan and RADV 880M. This restores a causal invariant; it
does not implement GI, canopy sky visibility or visual-reference acceptance.

### Disabled reflections still evaluated direct stellar specular

`bf8635a` moved stellar GGX and roughness filtering outside the mesh reflection
flag, while terrain and the graphics metadata still define that flag as
**stellar and environment** reflection. Medium/High meshes could therefore
retain hidden specular after disabling it. The existing documented flag once
again gates both source and environment specular and the associated filtering.
No new setting or renderer mode is introduced.

An independent GPU control keeps material albedo/metallicity, source and geometry
fixed, disables reflections, and compares roughness 0.2 versus 1.0. Their direct
images must be identical for both Medium and High, and the receiver must remain
visible. The control passes on both Vulkan implementations. This does not
calibrate leaf optical constants or establish performance improvement.

### The alleged five-metre fade was a global-scale threshold

The shared shader computes `globe_texel_m = PI * radius / globe_texture_height`.
Native currently binds a **1×1 fallback texture**, not an authored 512-high
global albedo map. At the 3.2 Mm body radius, the scale is approximately
**10,053,096 m**. Changing the transition from `[1,2] * globe_texel_m` to
`[2,8] * globe_texel_m` cannot explain a five-metre ground-detail cutoff.
Even the 512-high case asserted in doc 116 would be about 19.6 km, still far
outside the claimed scale. Doc 118 contradicts doc 116's own rejection of this
hypothesis.

The unsupported multiplier expansion is reverted consistently in albedo,
normal and relief-weight paths. This removes unjustified tuning; it is not
presented as a close-ground visual fix or a measured speedup. The useful
macro-gradient preservation/fallback changes are retained, with partial
coverage and source-normal consistency still requiring independent controls.

## Claims that remain unproved

- **Water semantics:** the narrower 0.18–0.22 roughness / 0–0.25 m predicate
  still derives water from unrelated properties. It rejects the selected
  roughness-0.45 soil case but does not add water presence. Smooth dry land
  at height 0.1 m and roughness 0.1 still gets about 64.8% water weight. Real
  water with roughness 0.3 gets none. Existing basin/ocean ownership must
  supply presence/depth separately from roughness.
- **Diagnostic decomposition:** `direct`/`ambient` return source irradiance
  divided by PI and multiplied by exposure; they omit material albedo and
  later energy/transmission factors. `specular` returns a pre-mix land
  reflection term. These are useful source views, **not** exact additive
  final HDR components, particularly for water, emission and haze.
- **Visual/cost parity:** regional RGB means, changed-pixel counts and MAE
  against generated artwork cannot prove transport correctness, physical
  calibration or a speedup. Camera, data readiness, topology, population,
  exposure and presentation changed across the reported iterations.
- **Authored scene changes:** foliage colors/transmission, pebble roughness,
  puddle selection and weather/exposure profiles are retained as art inputs,
  not accepted physical calibration or proof that core defects are resolved.
  No claim is made that this audit independently validates every such change.

## The foundational work sequence

1. **Reproducible frame state.** Freeze world recipe, camera/time, source
   identities, height/material residency, emitted CBT topology and exposure.
   Record publication/resource identities. A measured comparison must replay
   that state in one process; longer prestream is not a moving-camera fix.
   Production GPU timestamps must remain separate from serial-fence diagnostics.
2. **One declared surface contract.** Extend existing height/material owners
   with independent land/water presence and, when needed, optical water depth.
   Geometry, material normals and filtering must reference the same declared
   field/footprint. Bound source-page error separately from CBT chord error;
   splitting triangles cannot recover missing height data.
3. **Auditable lighting transport.** Preserve shared solar/sky physics, then
   estimate indirect visibility and bounce from actual geometry/fields using
   existing owners. Cheap approximations need explicit inputs, error/cost
   controls and disabled fallbacks. A sun-shadow scalar is not sky visibility.
4. **Reusable current-frame receivers and real component outputs.** Depth,
   normals, material tags and decomposed radiance must describe the same frozen
   visible surfaces. These can support selective shadow correction, screen
   traces/caches and an ambient-only visibility term without darkening direct
   light or emission. Their storage/passes must be enabled only when needed.
5. **Quality and cost acceptance.** Compare stable frames to independent
   physically matched engine controls and to artwork in its visual role;
   measure actual production stages and reject candidates without demonstrated
   benefit. Avoid a wholesale deferred rewrite without profiling evidence.

The first three corrective edits above are implemented. This sequence is a
work plan, **not a claim that frozen replay, water semantics, GI or new receiver
buffers are already implemented**. No further color/roughness/atmosphere tuning
is substituted for those contracts in this milestone.

Current continuation: [doc 120](120_FROZEN_NATIVE_DIAGNOSTICS.md) implements
step 1 as an in-process published-frame replay, with a complete FHD sequence.
Independent process restore remains open. [Doc 121](121_LIQUID_WATER_SURFACE_CONTRACT.md)
closes the independent water-presence slice of step 2; source accuracy and
steps 3–5 remain open.

## Task-by-task acceptance ledger

| Task | Current result | Closure criterion / existing owner |
|---|---|---|
| Frozen published native frame | Closed in doc 120 | Same-process view replay, unchanged depth/vertices/topology/epochs, beauty A/B/A identity; native QA over existing owners |
| Independent liquid-water presence | Closed in doc 121 | Explicit source coverage; rough land/rough water/missing-plane GPU controls; raster, waves, rays and Bevy residency agree |
| Height source versus CBT chord error | Open | Quantify each independently at fixed residency/camera, control transient and stationary error; existing worldgen/height stream and conforming LOD |
| Geometry-based sky visibility and bounce | Open | Actual geometry/field inputs, source-slot invariance, no direct/emission attenuation by ambient visibility, reference error and disabled/cheap tiers; existing environment/FastRT owners |
| Current-frame receiver depth/normal/material tags | Open | Same visible surfaces/coverage as beauty, selective allocation and portable fallback; existing terrain/mesh/MSAA ownership |
| Exact additive radiance components | Open | Sum matches frozen linear HDR within format rounding, including direct/indirect/specular/emission/air; shader terms are final contributions rather than irradiance source views |
| Physically matched external-engine controls | Open | Same camera/time/assets/materials/exposure and readiness, independent transport oracle; artwork remains a separate visual target |
| Production quality/cost acceptance | Open | Real production GPU stages, repeatable same-workload A/B, unchanged populations and acceptable image/temporal error; fenced diagnostic wall time cannot substitute |

Each remaining row is a separate implementation/acceptance task. Completing the
first two rows is not closure of step 2 as a whole, GI, a receiver pipeline or
reference parity. No completion mark is based solely on adding a flag or a
promising single image.

## Executed validation

Eight bounded surface-lighting tests pass on isolated software Vulkan and
RADV 880M, including source-slot permutation, disabled-specular behavior,
per-source shadows, shared caches, large origins, material maps, normals/AO
and resident terrain casters. The unrelated 1024-patch fixed-cost benchmark is
explicitly excluded. The retained skirt-diagnostic fixture was also corrected
to view a real skirt from above the substrate with a compatible up axis; its
old background-pixel failure and corrected software/RADV pass are retained.
No production geometry was changed for that fixture. No full-forest timing or
aesthetic win is claimed.
CPU, strict lint, architecture and client checks are recorded in the artifact.
The regression test's pre-fix failure is retained rather than relabeled as a
passing baseline.
