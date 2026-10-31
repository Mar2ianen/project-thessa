# 104 — KSA October audit and radial optical shock structure

Status: a bounded radial-field correction is implemented and checked against
stock Cycles and independent mathematical oracles. The user-supplied October
KSA build has been inspected statically. This does not establish KSA visual
parity, calibrated rocket combustion, or completion of live nozzle extraction.

## What the reference actually contains

The downloaded archive is `ksa_linux_v2026.10.25.5707.tar.gz`. REA 6.5.0 verifies
its `KSA.dll` assembly version as **2026.10.25.5707**, not the previously
installed **2026.9.10.5438**. Their DLL SHA-256 values are respectively
`0578047715c733b4157161c4bb0e9e7bee5ec769909c91e24fa1dd0d81c8e8c1` and
`7aadb15ef67094b276b290074075b565a221ebbe18dfef49151cd19f832a265e`.
The new MVID is `bdb346bd-bdf8-48b4-b231-124a5ff90f11`.

Selected managed inspection covers `RocketNozzle`, `ExhaustGasDynamics`,
`ExhaustLobes` and `VolumetricExhaustRenderer`: 105 methods, 176 fields,
1,901 call edges and 2,156 field accesses, with no unmatched selected type.
It supplies CIL bodies, metadata and call relationships, not recovered original
C#. An earlier unfiltered inspection hit REA's 64 MB representation budget;
that failure says nothing about missing engine functionality. No configuration
repair or target execution was necessary.

| Observed reference behavior | Evidence boundary | Thessa implication |
| --- | --- | --- |
| Per-nozzle state feeds plume construction; gas dynamics supplies expansion, extent and core calculations | Selected CIL call graph, including `RocketNozzle.ComputePlumeData`, `ExhaustGasDynamics.ComputePlumeRadius` and renderer `AddInstanceCore` | Finish F03 using accepted operating observations and actual mounts; do not evaluate a requested throttle again |
| Physical density, isentropic turn and compression helpers coexist with authored shock/shape LUTs | Plaintext `PhysicalDensity.glsl`, `ShockCells.glsl`, instance parameters and `ExhaustAssets.xml`; these four files are byte-identical between the two builds | Preserve causal pressure/Mach/geometry inputs; do not mistake a LUT-based approximation for a CFD solver |
| Grouped/merged plumes, lobe shape, bending/folding and transient state are distinct mechanisms | Selected managed methods and shipped field/instance shaders; present in the older build too | One cached plume is insufficient for general multi-engine vehicles; do not draw only the first nozzle or hide overlapping volumes behind alpha sorting |
| October adds depth intervals and cloud/exhaust mixing | New `ComposeWithClouds.glsl`, changes to `VolumetricExhaust.frag` / `ExhaustOit.comp`, and new `SetCloudRenderer` method | Track volume support along each ray before implementing overlap; average depth and one alpha-over layer cannot represent interpenetration |
| Separate downsample, depth-aware upscale, denoise, OIT and refraction passes | Shipped shader files and selected renderer methods; several descriptor sets changed in October | Measure accuracy/cost of existing consumers before importing more passes; feature names are not performance evidence |

October's fragment shader records nearest/farthest samples carrying weight.
Its cloud composition explicitly reduces heterogeneous volumes to homogeneous
intervals and estimates cloud thickness; comments acknowledge approximation
and filtering limits. This is useful reference architecture, not proof of exact
transport for arbitrary overlapping fields. The cited public background is
Tom Duff's *Deep Compositing Using Lie Algebras* (2017),
[paper](https://graphics.pixar.com/library/DeepCompositing/paper.pdf).
The old Pixar URL currently redirects in a direct open; the searchable paper
text was available. No claim here depends on a successful new PDF download.

[BRUTAL](https://github.com/ahwootech/BRUTAL/tree/7a3b72d84368872b885094e34fe8cdf5f7989460)
was inspected at `7a3b72d84368872b885094e34fe8cdf5f7989460`. Its declared scope is
low-level C# infrastructure; it does **not** include KSA's renderer. Its custom
MIT-style license also has credits, logo and NOTICE obligations. No BRUTAL
implementation, proprietary KSA shader, optical LUT, texture or DLL is copied
into the repository. Full managed evidence and the extracted audit inputs stay
in the external output directory. The historical September audit in doc 44
remains a dated snapshot.

## Change the field where the defect occurs

Previously the interpolated axial `shock` multiplied the entire Gaussian
cross-section, including its broad mixing skirt. Compression/expansion thus
brightened or darkened the whole slice together, producing transverse bands.
The new neutral CPU medium and native/retained volume shaders use a circular
radial mode instead:

```text
u = abs(r / R(z))
phi(u) = J0(j01*u), u < 1; otherwise 0
j01 = 2.404825557695773
m(z,r) = max(1 + (max(axial_shock(z), 0.15) - 1)*phi(u), 0.15)
```

The regular axisymmetric solution of the radial Helmholtz equation is `J0`;
a pressure node at the core boundary selects its first zero. This follows the
public vortex-sheet mode construction discussed in Mancinelli et al.,
[*A complex-valued resonance model for axisymmetric screech tones in supersonic
jets*, equations 2.10–2.13](https://arxiv.org/pdf/2012.01342#page=8).
The [Tam/Jackson/Seiner NTRS record](https://ntrs.nasa.gov/citations/19850067203)
provides related shock-cell background; its record has no downloadable full
paper. This patch implements neither paper's full flow/acoustic model.

Here the mode shapes an **existing optical perturbation**. Axial wavelength,
strength/decay, envelope, emission hues and source model are retained. The
center keeps the old multiplier, the perturbation tends to zero at `r=R`,
and the mixing skirt is unmodulated. Emission and extinction receive the same
factor, preserving the constant source-function transport identity. This is a
weak-mode approximation, not a nonlinear Mach disk, compression-temperature
solution or calibrated spectrum; upgrade it when strong-shock/flow references
and source data are available. It cannot validate the existing amplitude model.

`J0` uses its degree-six polynomial in `q=j01²*u²/4`, seven terms including
the constant. The next alternating-series term bounds absolute truncation
below `5.3e-7` on the core. There is no texture/LUT upload or new dependency.
Native `mach_diamonds=false` skips the mode; the existing enabled/resolved
quality policy, Off behavior, Low impostor and 8/14-step budgets are retained.
The legacy volume gets the same radial factor but still uses fitted axial
uniforms; no new claim of full native/legacy field parity is made. TAA remains
default-off, and propulsion forces, mass flow and clocks are untouched.

## Actual checks and cost

The CPU oracle compares 10,001 radii to an independently summed 32-term f64
Bessel series. A compute probe calls the production WGSL helper at 2,268 queries
on RADV and llvmpipe, including compression, rarefaction, symmetry and boundary
ties. Maximum absolute multiplier error is `1.564897441e-6`, within the declared
`(1+abs(axial-1))*1e-6` envelope.

All nine compiled-methalox controls from doc 102 are rerendered at the same
source stations, pressure/throttle, camera, exposure and 14/16 native steps.
The five matched cases have **byte-identical** native HDR. The four pressure-
mismatch cases change optical fields deliberately. Cycles reads the explicit
`axisymmetric-j0-v1` scene tag and represents it using stock material math;
older captures without a tag retain `uniform-v1` behavior.

| Case | Native 14 vs stock Cycles 2048, linear RGB RMSE |
| --- | ---: |
| Sea/full | 0.00031172 |
| Sea/half | 0.00020251 |
| Matched/full | 0.00028862 |
| Matched/half | 0.00017666 |
| Thin/full | 0.00166952 |
| Thin/half | 0.00066527 |
| Matched/axial | 0.00045641 |
| Matched/inside | 0.00040836 |
| Matched/oblique | 0.00038993 |

Blender/Cycles is 5.2.2 LTS `d13f752e3b9c`, CPU/two threads, seed zero,
2048 samples, black environment, no denoise/adaptive sampling, coefficient
scattering zero. This measures transport of the **declared field**, not whether
its chemistry or appearance matches a real engine. Lower error against the
changed field is not improved accuracy against an unchanged physical reference.
All four constant-axial Gaussian native controls remain byte-identical.
Software/hardware RGBA fits `0.002*abs(RADV)+2e-6`; maximum difference is
`0.0001220703125`.

Release ABBA uses the existing ten 1280×720 plume-only views, uniform/radial/
radial/uniform, 20 warmup plus 80 measured frames per process and case. No
compilers run during timing. All station bytes are unchanged, repeated HDR
is exact within each variant, and the measured function body is identical to
the retained baseline fixture. Average-of-run p50 spans 0.118–0.676 ms before
and 0.126–0.689 ms after. Per-case changes range from −6.78% to +6.60%; the
largest increase is 0.01379 ms. These are fidelity-cost diagnostics with
process variability, **not** an equal-output optimization or whole-client FPS.
No KSA runtime frame-cost or matched camera comparison has been run.

36 plume-core and 40 render-wgpu lib tests, 40 Python tests, two plume GPU lib
oracles plus six GPU integration tests on each backend, strict all-target
Clippy, native-client compilation and the 44-package/20-foundation architecture
gate pass. Naga validates the full retained material body with only imported
Bevy types stubbed; a live Bevy material render is not covered by that check.

Evidence: [JSON record](artifacts/ksa-radial-plume-2026-10-31.json).
Raw commands, exact source patch, binary hashes, CIL evidence, old/new shader
inventory, HDR/EXR and saved `.blend` files remain under the external
`outputs/ksa-audit-2026-10-31` directory. The annotated galleries are technical
comparisons, not blind preference evidence.

The next substantive work remains F03 actual per-nozzle geometry/publication,
F04 optical/strong-shock validation, F05 field-derived light, and F07
multi-medium composition from the [living roadmap](98_GRAPHICS_PARITY_ROADMAP.md).
