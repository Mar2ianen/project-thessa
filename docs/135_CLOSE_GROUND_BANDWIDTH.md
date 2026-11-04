# 135 — Close-ground source/page bandwidth and integer L25 addresses

Status: bounded bandwidth/addressing improvement verified, 2026-11-04;
baseline `529f51e`. Foreground soil still looks smeared/striated and reference
acceptance remains open. This does not implement a new terrain topology.

## Research and applicability review

[Poly Haven Forest Ground 04](https://polyhaven.com/a/forest_ground_04) publishes
1K and 2K source tiers. The existing 3.15 m authored repeat gives a 3.076 mm source
pitch at 1K. Doc 134 measures foreground pixels below that pitch and confirms that
missing material ancestors are not the dominant blur explanation in its L24 frame.
Its independent periodic GPU fixture also retains the expected bilinear response
at the two nominal magnification footprints. A palette/exposure/sharpen change
would not add missing source bandwidth.

Pinned OpenEngine `bdf5c2710fe37c1c5ddbac7787477daa87524af3`,
`Engine/Modules/CBTTerrain/Shaders/CBT/cbt_surface_pages.glsl:94–109,130–139`,
selects material pages from physical footprint. Thessa reuses its existing
source/page producer, stream, resident directories and split chart. Unreal's
[detail-texturing documentation](https://dev.epicgames.com/documentation/unreal-engine/adding-detail-textures-to-unreal-engine-materials)
describes another way to add close bandwidth; this iteration does not introduce
that extra texture layer or copy Unreal code. Large CBT remains the topology
reference; optical page levels are separate from geometry LOD.

## Implementation

Verified native 2K diffuse/roughness/16-bit height/OpenGL-normal maps replace the
1K soil and moss/litter scans. Physical repeats (soil 3.15 m, moss 3 m), height
ranges, stochastic phases/rotations, material transfer, lighting and exposure
are preserved. Importers pin source checksums and the existing FG01/GN01 formats.
The 512-square litter scan is unchanged. Source mip chains retain linear-light
averaging; normal residuals still subtract matched source-height gradients.

Source means are recomputed after the existing nonlinear authored substrate
transfer, retaining the strict mean regressions rather than relaxing them. Soil
mean linear RGB changes from [0.0970324,0.0685435,0.0432257] to
[0.1063321,0.0756586,0.0480300], and roughness from 0.7991561 to 0.7873502.
Moss mean RGB changes slightly as well. Thus the source upgrade is not a pure
frequency-only color identity experiment; no exposure or additional grading gain
is used to compensate these source statistics.

High's optical page ceiling becomes L25 (1.526 mm nominal pitch at R=3.2 Mm),
matching the 2K soil pitch of 1.538 mm and moss pitch of 1.465 mm. Low stays L20,
Medium L21, ground-detail-disabled L17, and rendered height geometry stays L22.
The existing resolver and both stream admission clamps agree. Page/layer/cache
budgets, page size/format and geometry representation are unchanged.

### Precision failures reproduced before acceptance

Simply lifting the ceiling fails the physical-GPU L25 subtexel test: face 0,
step -4 returns red 168 instead of 147 (unchanged two-code bound). Large page
indices exceed f32's consecutive integer range: 29053700 + 3 rounds to 29053704.
`MaterialCoordinates` now keeps the page index/carry in i32 and the small local
fraction in f32. Negative offsets and boundary clamping retain their semantics.
No GPU buffer/material ABI is changed; this is a local shader value.

The follow-up cube-edge fixture also reproduces missing coverage with an actual
resident adjacent-face page: face 0 / negative U gives green 132 instead of full
coverage. A half-cell disappears during float reprojection of a positive edge.
Neighbor presence now uses integer cube orientation/ownership and a bounded
one-cell discrete mapping, following the existing `conforming_address.wgsl`
face convention. Same-face indices remain integer as well. Parent fallback and
missing-neighbor feathering remain; no neighbor is declared resident by default.

## Cost and limitations

There is no new GPU pass, image, binding, page size or fixed GPU cache allocation.
Extra optical demand still competes for the same budget: the inspected L25 frame
has 5.3446% of covered bottom-third pixels more than half a page level behind its
request, compared with 0.1846% in the earlier L24 frame. These are separate timed
stream publications, not an exact residency/performance A/B.

The two color/roughness/height source mip chains add 160 MiB of CPU cache; matched
normal-residual chains add 64 MiB, totaling **224 MiB** after initialization.
Temporary decode/residual work and larger embedded source bytes are additional.
Disabled ground detail still performs no source decoding. This is a real memory
cost; unchanged GPU budgets do not make it free. Release frame-time acceptance is
open. No performance win, physical scan-scale calibration or full renderer parity
is claimed.

## Verification and actual visual decision

The source-only 2K/L24 control and the 2K/L25 candidate each complete 21 frozen
FHD views at 0.3 m. The author inspected both complete sheets plus native-resolution
beauty, albedo and normal images. Source-only replacement does not repair the
appearance. L25 retains somewhat finer foreground variation, while the broad
smeared/striated mud remains prominent. An independent Luna review reaches the
same narrow conclusion: retain a bandwidth slice, not a soil/scene acceptance.

The L25 subtexel oracle covers all six cube faces and positive/negative chart
carries; the cube-edge residency oracle compares resident/absent neighbors with
an independent f64 projection on all 24 face-edge combinations. Older material,
normal-coverage, upload/fallback and periodic-signal regressions remain intact.
All 10 material GPU regressions and 32 ray-scene regressions pass. The edge
fixture also passes at L17/L24/L25 (72 face-edge cases with resident/absent
controls). CPU checks pass 305 worldgen cases (1 ignored), 34 stream, 55 graphics
and 153 native cases (51 ignored); strict all-target/all-feature Clippy and
architecture pass. The final wide FHD capture includes the final cross-face
helper and completes all 21 views with unchanged per-run geometry, depth and
beauty round trip; all its images were inspected. No new scene regression was
observed, and no scene-quality acceptance is inferred.

[The evidence record](artifacts/close-ground-bandwidth-2026-11-04.json) pins
source/image/log identities, scope and the rejected failing intermediates. Direct
GPU diagonal-corner rendering, moving-camera preference, full source compression
policy and reference-quality ground appearance remain open.

Packed soil/moss sources grow from 17,790,320 to 71,853,064 bytes
(+54,062,744 bytes, 51.56 MiB); manifests pin each payload.
