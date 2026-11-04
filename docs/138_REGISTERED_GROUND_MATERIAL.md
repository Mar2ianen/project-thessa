# 138 — Preserve registered ground material through composition

Status: source/composition slice verified, 2026-11-04; baseline `e5cae71`.
Broad foreground smear/striations and full-scene reference acceptance remain open.

## Reference research and applicability review

[Filament's full PBR sample](https://github.com/google/filament/blob/e079be78521a8da7542ebedfb86d279deaa98132/samples/sample_full_pbr.cpp#L260)
consumes the source base-color and roughness maps independently (lines 260/278).
[Unreal's PBR material documentation](https://dev.epicgames.com/documentation/en-us/unreal-engine/physically-based-materials-in-unreal-engine)
distinguishes photographed albedo from lighting and exposes separate material
inputs. Neither reference forbids authored material grading. The relevant question
here is whether Thessa preserves its existing registered scan or invents another
surface without matching its other source channels. No engine code is copied.

Two existing overrides were isolated:

1. `forest_substrate` classified minerals from source RGB luminance and attenuated
   dark source pixels to 24–28% of their linear reflectance. An offline float64
   visualization estimates 47.6% of the 2K source at full attenuation; the exact
   runtime regression independently reproduces an encoded red value 94 instead
   of source 115. The source crop shows artificial near-black interstitial areas.
2. `sample()` already mixes the soil and pinned litter scans across albedo,
   roughness, centered height and normal residuals. Later `forest_floor()` applied
   two more litter noise masks, a flat brown `[0.048,0.038,0.024]` endpoint with
   source contrast clamped to `[0.5,1.5]`, and a flat roughness 0.75 endpoint.
   This second optical coat hid the registered scan while keeping its height.

The first ablation alone produced little native improvement. The fixed page's
average encoded RGB barely changed after all later layers, motivating isolation
of the second coat rather than accepting the source crop as a scene-quality win.

## Implementation

Decode the soil scan's native RGB reflectance in linear light, as already done
for litter/moss. Remove the private transfer switch and classifier. Update the
exact soil coarse mean from `[0.106332146,0.075658605,0.04803002]` to
`[0.1558859,0.11072619,0.066067934]`; keep roughness and height means unchanged.
Linear-light mip generation remains. This restores source fidelity, not a new
reflectance calibration or extra bandwidth.

Remove the second flat litter color/roughness coat and its two noise evaluations.
The real scanned litter mixture remains. Existing wetness, mineral modifiers,
moss pigment/coverage, ecological eligibility, later rock/salt/snow/frost ownership
and source-height/normal composition remain. Both native and Bevy consume the
shared producer. There is no new setting, source asset, pass, binding, texture,
page dimension, cache budget, terrain subdivision or temporal state. Existing
quality/disabled paths retain ownership. Large CBT remains the topology reference.

## Tests and controlled evidence

- The exact source regression fails before removing the grade and passes after:
  every soil RGB sample round-trips to its original encoded source byte, while
  roughness and R16 height remain exact. A separate 2x2 sRGB64/192 fixture pins
  its independent linear mean 0.2891923, retaining the linear filtering contract.
- On the intermediate raw-source/still-coated implementation, the new dry-floor
  fixture reports 20/32 roughness and 180/384 contrast failures. Removing only
  the flat coat makes all cases pass; height is unchanged. Contrast tolerance
  remains `1e-7`, with nonfinite values explicitly failing.
- One legacy moss fixture selected maximum green coverage while requiring
  underlying substrate contrast. It now requires actual partial moss coverage
  `[0.25,0.5]` and retains the existing >3% per-channel contrast threshold,
  maximum-green and spatial-variation assertions.
- The coastal fixture's roughness `<0.85` expectation encoded the removed 0.75
  coat. It now checks the actual no-floor/floor source endpoint range and a
  nonzero floor contribution. Later material ownership still reduces the same
  source weight; it is not replaced with another arbitrary roughness ceiling.
  Independent review approved these fixture-contract changes.
- All 307 worldgen tests pass (1 ignored); strict all-target/all-feature Clippy,
  native build, formatting and the architecture gate pass. Shader code is unchanged.

Reproducible CPU/build checks:

```sh
cargo test -p thessa-worldgen-rocky --lib
cargo clippy -p thessa-worldgen-rocky --all-targets --all-features -- -D warnings
cargo build -p thessa-terrain-native
cargo fmt --all -- --check
python3 tools/agent-qa/check_architecture.py
```

Native captures use the render-node-only bubblewrap commands preserved in each
capture's `command.json`, with the same FHD settings/camera/time request as doc 137.

The exact L25 page sequence uses face 1, x=12325312, y=24347772 throughout:
baseline, raw-source-only ablation, and final composition. Optical normals,
canonical base normals and water planes are byte-identical. Final page mean
roughness is 0.882688 versus 0.822618 before, with the registered spatial source
variation retained. The source-only ablation keeps roughness byte-identical;
the final composition intentionally removes the old coat's roughness override.

Root inspected every emitted source crop, page/control image and both FHD
captures' 21-view diagnostic sets, plus native beauties and the final albedo.
Independent Luna review found no blocking regression: retain the bounded
source-registration/readability improvement. Native albedo is modestly less
uniformly brown-painted; the full beauty still has conspicuous broad smear and
striations. These are not an accepted blur fix or reference parity.

Separate timed native publications are not exact whole-frame A/B; fixed page
controls are exact. Each native diagnostic sequence retains its own frozen
geometry, depth and beauty round trip. No blind preference claim is made.
TAA remains off. [The evidence record](artifacts/registered-ground-material-2026-11-04.json)
pins the baseline, source/binary/log/image identities and both reviewed ablations.

## Cost and next unresolved isolation

No persistent CPU/GPU memory, GPU samples, draw calls or passes are added.
The decoder's classifier and two source-side litter noise evaluations are removed.
Release frame time was not measured; serial-fenced debug captures explicitly
have `performance_equivalent=false`, so no speedup is claimed. Source cache
costs from docs 135/137 remain.

The shared 0.5–8 m procedural grain is present in geometry and optical sources,
but full canonical-base reconstruction can intentionally replace the geometric
normal. Code overlap alone therefore does not prove doubled final slope. A
future single-octave height-only/normal-only/combined fixture with full and partial
base coverage must decide this before changing relief amplitudes or masking bands.
Source scale calibration, close-view aliasing/motion, atmosphere, plant contact,
shoreline integration and full-scene quality remain open.
