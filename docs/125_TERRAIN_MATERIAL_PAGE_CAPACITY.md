# 125 — Terrain material-page capacity by quality and device limit

Status: implemented in the shared graphics resolver and native wgpu host.
High quality can retain 512 material pages on a device that exposes that
texture-array limit. Medium remains at 256, Low uses 128 while ground detail
is enabled, and the disabled path retains the established 256-layer capacity.
Unknown/legacy hosts also retain 256. A device limit caps the resolved value;
the host requests only that resolved limit and reports the device value back to
the resolver after creation.

The purpose is to retain more of the already-generated source material pages
for the camera-visible area. It does not change terrain topology, height-page
generation, the world field, or material-page source quality. High allocates a
larger material array only when the optional ground-detail path is requested.
The existing client host does not report this capability and therefore keeps
the legacy capacity. The effect is not a substitute for fixing page priority or
temporal gaps in residency.

## Executed evidence

The resolver test covers Low/Medium/High, ground detail enabled/disabled,
unknown capacity and reported device limits of 128, 256, 512 and 2048 layers.
It also verifies old serialized settings deserialize to 256. The native B03
FHD capture resolved High to 512 layers and retained 357 GPU pages while the
CPU had prepared 366 pages. The preceding 256-layer capture retained 256 GPU
pages for the same view. This is a capacity/residency observation, not a claim
that every missing page or close-ground material artifact is solved.

The two captures use the same fixed camera and scene profile. Increased
residency changes material selection at some ground footprints, so their
beauty images are not an equal-input lighting/BRDF A/B. Depth, terrain
topology, and terrain vertex positions are the relevant invariants for checking
that the capacity change did not move the planet.

## Reference-informed implementation boundary

The quality tier follows the project-wide Low/Medium/High contract. Existing
terrain instances, material pages, and rendering passes remain the owners; no
parallel cache, global renderer switch, or extra frame pass is introduced.
The page array stays bounded by the adapter capability, with an explicit
legacy fallback when the host cannot report it. The increased High allocation
and the unresolved 9 CPU-only pages are retained as limitations, not hidden
behind a visual-quality claim.

Capture directories are under
`outputs/ground-detail-repair-2026-11-03/{normals-fhd-20s,capacity-512-fhd-20s}`
in the local workspace. Resolver tests and raw native diagnostics accompany
this implementation; no blind visual vote has been made for page capacity.
