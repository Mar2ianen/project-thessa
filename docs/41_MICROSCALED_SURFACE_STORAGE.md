# 41 — Microscaled surface storage

Status: **design / prototype target**.

Implementation status (branch `feat/microstorage-phase-a`):

- Phase A done in `thessa-microstore-core`: scalar 4x4 codec (Raw8,
  Residual8/4), deterministic wire format, error metrics, 7 seeded
  fixtures, PGM comparison dumps.
- Phase B done in `thessa-microstore-core::wgsl` (sample-time WGSL
  decoder) + `thessa-rcbt-wgpu::microstore` (upload + compute decode,
  GPU==CPU parity on all fixtures); the RGBA material path is untouched.
  Honest scope: this is a GPU decode *parity prototype* (whole-page
  dispatch + sync readback), not a measured material-shader sample —
  bilinear/aniso/mips, which the RGBA path gets from hardware, are still
  open, as is the R6 tail-byte upload slack now guaranteed by the
  backend.
- Phase C done in `thessa-microstore-core`: Residual6/Residual2,
  cheapest-first adaptive ladder (2/4/6-bit, Raw8 fallback), per-channel
  color pages with linear-light error, header/payload overhead
  accounting, 128/256 extents in benches.
- Measured so far (128x128): R4 0.689 B/tex (err <= 8), R6 0.938 B/tex
  (err <= 2.6), R2 0.438 B/tex (err <= 43, headers 43% of bytes);
  adaptive@2.0 holds the budget by construction; GPU decode ~0.1-0.3 ms
  per page on a Radeon 780M; 4 adaptive channels ~= 0.94-1.19x one RGBA
  page upload — the density win lands with higher texel counts (Phase D).
  Accounting is split three ways (wire vs gpu-upload vs gpu-resident);
  the A/B baseline is the full 87,380 B mip chain (4 adaptive noise
  channels upload 77,984 B = 0.89x of full-mip RGBA but 1.19x of the
  65,536 B base level alone), with the no-mips-yet caveat stated in the
  bench output.
- Phase D done in `thessa-microstore-core::residency`: key-addressed
  stable slots, dirty-block upload ranges, LRU eviction by encoded byte
  cost, telemetry (resident bytes/texels, hit rate, texels/MiB).
  Measured: one-texel update uploads 15 B vs 3861 B full page (257x);
  cyclic scan over 4x cache converges to exactly 0.25 hit rate.
  Per-block patches are only emitted while the block layout is unchanged;
  codec-rung or extent changes fall back to a full page + table reupload
  (variable block sizes shift every later offset). Eviction cost is
  backend-reportable per page instead of assumed wire-equal.
- Fixture 8 done: `worldgen-rocky --example dump_microstore_fixtures`
  vendors real Thessa bytes into `microstore-core/tests/assets`
  (65x65 height ocean/coast/mountain, 128x128 coast albedo+roughness);
  material color tests and the color bench row run on the real albedo.
- Phase E done in `thessa-microstore-core::height`: f32 micro-codec
  (page min/max header, per-block offset/scale, adaptive 8/16-bit with
  Raw32 fallback) plus the three demanded verifiers (absolute error,
  conservative bound slack, normal angle) and a shared-edge crack metric
  for independently encoded neighbor pages.
  Measured on real grids (raw f32 = 4.000 B/tex): R16 2.811 B/tex with
  err <= 0.11 m and physical-space normals <= 0.05 deg (texel spacing in
  metres, anisotropic); shared-edge crack on the coast split is 0.262 m —
  recorded as a crack, not a pass: independent lossy pages provably
  diverge on shared edges, lossless pages share them bit-exactly, and
  `border_is_lossless` gates geometry suitability. Verdict: safe as an
  experimental lossy residency cache; crack-free geometry and canonical
  baked-format adoption stay blocked on a boundary strategy (lossless
  border strip or global-lattice references) plus material/lighting
  review of the normal-angle sensitivity.
- Follow-up hardening (same branch): pluggable selection metric
  (`ColorBudget::Levels` matches scalar adaptive byte-for-byte,
  `ColorBudget::Linear` selects rungs in linear light and measurably
  spends more bytes on bright ramps); `Relocated` flushes (changed
  blocks + fresh table instead of full pages on rung changes, with the
  backend applying the table first); CPU mip chains as plain page
  vectors (8-level 128x128 chain = 1.32x the base page, geometric
  series made explicit; GPU mip sampling stays open).
- Second follow-up: slab allocator (first-fit, realloc grow/move/shrink,
  coalescing, deterministic slide-down compaction, `check_invariants`)
  with a 1500-step cache+allocator integration workload asserting
  cross-structure invariants after every op; GPU scattered-layout decode
  (allocator offsets, fragmented buffer) bit-exact on hardware; GPU mip
  chain bit-exact level by level; packed sample-time bilinear filtering
  at 2.7-3.0 M samples/s with zero observed drift vs the CPU mirror
  (tolerance: one code level).
- Sample-time completion: CPU footprint LOD selection
  (`lod_level`, `aniso_ratio`, `sample_aniso` with 1/2/4/8 taps) plus
  `lod_select` and `sample_aniso_packed` WGSL kernels. Measured on
  hardware: LOD exact 256/256 footprints (446 ns/select); aniso taps
  2.4/2.3/1.9/1.7 M samples/s with 96-100% of samples within one code
  level of the CPU mirror and bounded tails (worst 22) from texel-
  boundary floor() flips under f32/FMA divergence — pinned
  statistically, with bit-exactness on uniform fields. Combined
  LOD+mip-fetch (per-sample level buffers) stays engine-integration
  work.
- Integrated LOD+mip path (`LodMipSampler`): GPU decode, GPU mip chain,
  GPU level selection, grouped plain sampling per level, reassembled in
  sample order — with a CPU `sample_lod` mirror. Measured end-to-end on
  hardware across all 7 fixtures: 128/128 samples within one code level,
  worst drift 0 (1.18 ms / 128 samples incl. per-stage readbacks; a
  production backend would chain device buffers with no roundtrips).

This document defines a reusable microscaled storage layer for render-side and
streamed surface data. The immediate target is terrain material pages. Height
pages are a secondary target after the codec and error metrics are proven on
visual data.

The idea is inspired by microscaling formats such as NVFP4, but **NVFP4 is only
a reference for the representation principle**. Thessa does not require NVIDIA
hardware, FP4 arithmetic, Tensor Cores, or the NVFP4 bit layout.

The useful principle is:

> A globally wide numeric range often has a much smaller local range. Store the
> local reference/scale once, then encode the remaining values cheaply.

This is the numeric analogue of several existing Thessa choices:

- camera-relative rendering removes a large global position before using local
  f32;
- height pages store a base height plus quantized residuals;
- RCBT dirty-path updates preserve global tree state through small local deltas
  instead of rebuilding it from scratch;
- hierarchical terrain/page addressing makes locality explicit before rendering.

The target is therefore not "FP4 textures". The target is a general
**hierarchical residual representation** that lets storage precision follow
spatial locality.

---

## 1. Primary goal

Increase useful surface texel density without scaling memory, upload bandwidth,
and cache pressure linearly with the uncompressed texture size.

Current local material pages are deliberately simple and portable. Their cost is
easy to reason about, but every texel pays for the full channel range even when a
small neighborhood occupies only a tiny part of that range.

A microscaled block instead stores something conceptually like:

~~~text
block reference / base
block scale or range
small residuals
~~~

For a local material field:

~~~text
surface page
  |
  +-- microblock
  |     base
  |     scale
  |     packed residuals
  |
  +-- microblock
  |     ...
  |
  `-- ...
~~~

The optimization target is not minimum bytes in isolation. It is the best
quality/bandwidth/decode tradeoff at the actual terrain sampling workload.

---

## 2. First target: material pages, not height geometry

Material data is the safest first consumer.

Reasons:

1. Visual channels tolerate bounded quantization error better than authoritative
   geometry.
2. Artifacts are immediately visible in captures and can be scored numerically.
3. Material pages already have strong spatial coherence.
4. Decode cost is naturally paid close to sampling.
5. A reusable codec can later be tested on height residuals and other scalar
   fields.

The first prototype should therefore operate on terrain-local material data:

- albedo or a transformed color representation;
- roughness;
- metallic/mineral/semantic weights where applicable;
- optional slope/normal-like fields if a compact residual representation beats
  recomputation.

Do **not** begin by replacing authoritative terrain queries or physics data.

---

## 3. Microscaling is a pattern, not one format

No single fixed format is declared by this document.

A generic scalar block can be thought of as:

~~~text
decoded = offset + quantized * scale
~~~

but individual channels may benefit from different transforms.

Candidate residual widths:

~~~text
2 bit
4 bit
5/6 bit packed
8 bit
raw fallback
~~~

Candidate block sizes:

~~~text
4x4   = 16 texels
8x4   = 32 texels
8x8   = 64 texels
~~~

Small blocks improve local fit but spend more metadata. Large blocks amortize
metadata but increase quantization error around edges and mixed materials.

The initial benchmark matrix should test at least 4x4 and 8x8 blocks.

---

## 4. Adaptive block precision

A fixed 4-bit codec is not a requirement.

A stronger design is for each microblock to select the cheapest codec that
satisfies an explicit error budget:

~~~text
almost flat block
    -> 2-4 bit residual

ordinary coherent block
    -> 4-6 bit residual

high contrast block
    -> 8 bit residual

pathological / discontinuous block
    -> raw or higher-precision fallback
~~~

A tiny per-block codec tag is acceptable if it saves enough payload and does not
make shader control flow expensive.

This lets smooth rock, ice, regolith, ocean, and low-frequency color fields stay
very cheap while boundaries retain detail.

Codec choice must be deterministic for a given source page and configuration.

---

## 5. Channel-specific encoding

Do not assume that interleaved RGBA residuals are optimal.

### 5.1 Albedo

A useful baseline is:

~~~text
local base color
+ per-channel scale/range
+ packed residuals
~~~

Alternative color spaces or decorrelated transforms are allowed only if they
produce a measured quality/bandwidth win and remain cheap to decode.

The error metric should not be raw byte equality. At minimum measure decoded
linear-light error and rendered image error.

### 5.2 Roughness and scalar material fields

These are especially strong candidates because a local patch often covers a
small range.

A block may need only:

~~~text
min/range + UNORM residual
~~~

or:

~~~text
center + signed residual
~~~

### 5.3 Normal-like data

Do not blindly microscale XYZ normals.

Prefer one of:

- recompute the geometric normal from terrain where possible;
- store a local slope/gradient residual;
- encode a perturbation relative to a block or geometric reference normal.

The representation should preserve normalization cheaply at decode time.

---

## 6. Precision follows locality

The desired numeric hierarchy is:

~~~text
global simulation / ephemeris
    high precision

body / camera reference frame
    f64 -> local f32 where safe

terrain page
    page-local base/range

material microblock
    block-local base/scale

sample
    small packed residual
~~~

Each level should store only the information not already supplied by its parent
reference frame.

This is the same general optimization rule across coordinates, topology, and
surface data:

> Do not repeatedly store or recompute the global component when a local delta is
> sufficient.

---

## 7. Relationship to current HeightPage

The existing height representation already has one level of quantization:

~~~text
height = base_height + i16_residual * residual_scale
~~~

That is effectively page-scale quantization.

A future height codec may add another level:

~~~text
page base
    +
microblock scale / optional local offset
    +
small residual
~~~

However, height data has stricter requirements than material data:

- cracks across shared page boundaries are unacceptable;
- decoded error affects actual geometry;
- normal estimation amplifies local height error;
- conservative min/max and slope bounds must remain valid;
- contact/server semantics must not accidentally depend on a renderer-only
  lossy representation.

Therefore height microscaling is a second-stage experiment.

If adopted for canonical baked height data, the encoder must preserve declared
absolute error bounds and conservative metadata. If adopted only for GPU
storage, the canonical page remains the authority and the compressed form is a
cache representation.

---

## 8. RCBT interaction

Microscaling complements RCBT but is not part of the tree semantics.

RCBT answers:

~~~text
which spatial regions need representation?
~~~

Microscaled storage answers:

~~~text
how densely can the resident data for that region be represented?
~~~

The two layers must remain separable.

A renderer should be free to change material codec without changing observable
RCBT topology. Likewise, RCBT refinement must not depend on accidental details
of one compression format.

The combined path is:

~~~text
canonical surface/material field
          |
          v
     page generation
          |
          v
  microscaled encoding
          |
          v
   residency / streaming
          |
          v
 RCBT-selected consumers
          |
          v
 decode near sampling
~~~

---

## 9. Streaming and residency goals

The primary performance win is expected from reduced data movement and larger
effective resident coverage, not from arithmetic speed.

Measure:

- CPU encoded page bytes;
- upload bytes/frame;
- GPU resident bytes;
- page-cache hit rate;
- page eviction rate;
- visible texels represented per MiB;
- decode cost per sampled texel;
- end-to-end terrain GPU time;
- cold-page generation/encoding latency.

The successful result is allowed to spend a few cheap integer/float operations
to save substantially more memory traffic.

This is particularly attractive on UMA systems where CPU/GPU share memory
bandwidth and where avoiding unnecessary resident bytes helps both sides.

---

## 10. Decode placement

Possible decode strategies:

### Sample-time decode

Read packed residuals and metadata directly in the material shader.

Advantages:

- no expanded cache;
- minimum residency;
- naturally sparse sampling.

Costs:

- repeated decode for repeated samples;
- packing layout must be shader-friendly.

### Page decode into GPU cache

Decode a compressed page once into a conventional sampled representation.

Advantages:

- simple downstream shaders;
- amortized decode.

Costs:

- expanded GPU residency returns;
- explicit cache management;
- decode work on page arrival.

### Hybrid

Keep very compact scalar/semantic fields packed while expanding only channels
that benefit strongly from filtered texture hardware.

No strategy is selected yet. Benchmark actual hardware.

---

## 11. Texture filtering constraints

Compression must not silently destroy filtering quality.

Important cases:

- bilinear samples crossing a microblock boundary;
- mip generation;
- anisotropic sampling at grazing angles;
- transitions between local material pages and the globe fallback;
- neighboring pages encoded independently.

The decoder must produce consistent sample values at shared boundaries.

If block metadata makes cross-block interpolation expensive, options include:

- decode the four contributing texels independently;
- duplicate a narrow border;
- share/canonicalize boundary samples;
- predecode only boundary strips;
- fall back to a conventional representation for channels dominated by texture
  filtering hardware.

The right answer is workload-dependent.

---

## 12. Mip strategy

Do not generate mips by decoding a compressed base page every frame.

Candidate approaches:

1. generate canonical mips before encoding, then encode each level separately;
2. encode only the near/local levels and use the existing global map farther out;
3. store a small conventional low-frequency mip tail plus microscaled fine
   levels.

A separate codec per mip level is acceptable. Coarser mips often have even
smaller local ranges and should compress well.

---

## 13. Error budgets

Every codec must have a declared quality metric.

For scalar channels:

~~~text
max absolute error
RMS error
percentile error
~~~

For color:

~~~text
linear-light channel error
rendered-frame comparison
temporal stability under camera motion
~~~

For future height use:

~~~text
max height error
edge equality / crack test
normal angular error
conservative bound validity
~~~

A codec that is smaller but causes temporal shimmer is a regression.

---

## 14. Proposed backend-neutral API shape

Exact names are not fixed, but the abstraction should describe a codec rather
than a graphics API.

Example:

~~~rust
pub enum MicroCodec {
    Raw8,
    Residual8,
    Residual6,
    Residual4,
    Residual2,
}

pub struct MicroBlockHeader {
    pub codec: MicroCodec,
    pub offset: [f32; 4],
    pub scale: [f32; 4],
}

pub struct EncodedPage {
    pub extent: [u32; 2],
    pub block_extent: [u32; 2],
    pub headers: Vec<MicroBlockHeader>,
    pub payload: Vec<u32>,
}
~~~

This is illustrative, not an ABI.

The production format should likely use much smaller metadata than four f32
offsets plus four f32 scales. Header compression is part of the benchmark, not
something to guess in the public contract.

The CPU reference encoder/decoder must exist before backend-specific fast paths.

---

## 15. GPU layout rules

The packed representation should optimize for actual GPU access:

- aligned word loads;
- avoid byte-addressing patterns that turn one texel into many uncoalesced
  fetches;
- decode several neighboring samples from the same loaded word when possible;
- metadata should be cache-friendly and preferably SoA if that reduces redundant
  fetches;
- avoid vendor checks; select paths from capabilities and benchmarks;
- portable integer shift/mask/multiply is the baseline;
- subgroup/cooperative features are optional fast paths, never semantic
  requirements.

NVFP4/Tensor Core support is explicitly **not** a dependency.

---

## 16. Candidate benchmark fixtures

The benchmark set should include deliberately different surface statistics:

1. nearly uniform regolith;
2. smooth gradient;
3. noisy rock;
4. sharp biome/material boundary;
5. checker/high-frequency adversarial pattern;
6. coast/ocean boundary;
7. volcanic/high-contrast terrain;
8. real current Thessa captures/pages.

For each fixture compare:

~~~text
raw baseline
fixed 8-bit local residual
fixed 4-bit local residual
adaptive 4/8
adaptive 2/4/6/8/raw
~~~

Block sizes:

~~~text
4x4
8x4
8x8
~~~

Do not choose the final codec from synthetic compression ratio alone.

---

## 17. Acceptance criteria for the material-page prototype

A prototype is worth keeping when it demonstrates all of the following:

- deterministic encode/decode;
- bounded visual error with no obvious block seams;
- no temporal shimmer introduced by block selection;
- materially lower resident/upload bytes than the current page representation;
- either higher texel density at approximately the same memory budget or lower
  bandwidth at the same visual density;
- decode overhead smaller than the bandwidth/cache win on target hardware;
- graceful fallback for incompressible blocks;
- no new dependency of simulation or canonical terrain semantics on renderer
  state.

A higher-resolution page that merely shifts the bottleneck from memory to a
worse shader is not a success.

---

## 18. Suggested implementation order

### Phase A — CPU codec experiment

- extract current generated material pages;
- implement 4x4 block reference encoder/decoder;
- start with scalar u8 -> local offset/range + 4/8-bit residual;
- add deterministic round-trip/error tests;
- dump decoded comparison images.

### Phase B — GPU sample-time decoder

- upload headers + packed payload;
- decode one or two scalar channels in shader;
- compare GPU time and upload/residency telemetry;
- keep the current material path as an A/B fallback.

### Phase C — color and adaptive bit width

- add color representation;
- add per-block codec selection;
- measure block metadata overhead;
- test 128/256 and other page extents under the same memory budget.

### Phase D — integration with residency

- stable page slots;
- dirty uploads only;
- eviction based on encoded byte cost, not only page count;
- telemetry in bytes and effective texel coverage.

### Phase E — evaluate height microscaling

Only after material storage has a proven codec and GPU decoder:

- encode copies of current HeightPage data;
- enforce existing absolute error budget;
- verify boundaries, normals, min/max/slope bounds;
- decide whether the codec belongs only in GPU residency or in the canonical
  baked format.

---

## 19. Non-goals

This work does not:

- replace f64 authoritative simulation state;
- make lossy renderer data authoritative;
- require FP4 hardware;
- require machine-learning inference;
- require cooperative matrix instructions;
- couple RCBT topology to one texture codec;
- replace the canonical PlanetField/surface contracts;
- justify compression when raw storage benchmarks faster at the same budget.

---

## 20. General engine rule

The broader architectural lesson is useful beyond terrain:

~~~text
global value
    -> local reference frame
        -> block reference/range
            -> small residual
~~~

Potential future users include:

- terrain material fields;
- height residual caches;
- volumetric/atmospheric intermediate fields;
- particle and exhaust fields;
- BVH bounds relative to a parent node;
- render-only positions relative to a chunk/frame;
- presentation/network deltas where the authoritative state remains
  higher-precision.

The representation should be introduced only where locality is real and the
error/bandwidth tradeoff is measured.

---

## 21. Summary

Microscaling is valuable to Thessa because the project already exposes locality
at multiple architectural levels. The next step is to expose that locality in
the numeric representation of resident surface data.

The first practical target is:

~~~text
higher-density terrain material pages
        +
small local blocks
        +
shared local references/scales
        +
cheap packed residuals
        +
adaptive precision under an error budget
~~~

The desired result is not a branded low-precision format. It is a reusable rule:

> **precision follows locality; storage pays only for the residual information
> that remains locally significant.**
