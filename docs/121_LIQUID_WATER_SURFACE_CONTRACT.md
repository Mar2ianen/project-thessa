# 121 — Independent liquid-water surface coverage

Status: water presence separated from material roughness and emitted height.
Native regular/conforming raster, spectral deformation, reflected ray hits and
Bevy raw/compact material residency consume the same declared field. Source
height error, water absorption/refraction and overall visual parity remain open.

## Source and representation

The existing worldgen material producer samples canonical bathymetry before
raster height is clamped to the sea datum. Negative source height declares sea;
the existing explicit ice cover removes the frozen area from liquid coverage.
Foam/rough water stays liquid. Neither roughness, albedo nor geometry height is
a presence classifier. Existing closed-basin puddles remain explicit mesh
surfaces from their existing solver; no second hydrology or inferred puddle
system was added. No new optical depth is invented: depth-dependent absorption
and refraction are still future transport work.

`CbtMaterialPage` carries an optional independent R8_UNORM liquid-area chain.
Absent planes declare land. The same tile ID, immutable generation, border,
resident slot/ancestry, cube chart, gradients, page/mip blend and sampler govern
colour and liquid coverage. Liquid coverage does not fade with the colour-only
globe transition. Mips preserve exact integer source sums before quantizing,
so area error is at most 0.5/255 and does not accumulate down the hierarchy.
Lossy colour compression cannot change this semantic plane.

Each raw GPU material slot now adds 21,845 bytes for the complete 128²-to-1²
scalar chain (5,592,320 bytes for 256 slots; 11,184,640 for 512 Bevy slots).
Changed slots always upload/clear that chain, including pages with no plane,
so recycled layers cannot leak water. Native `MaterialUpload.bytes` includes
these transfers. Compact telemetry distinguishes colour-codec wire bytes from
exact scalar-plane bytes. The existing quality controls still bound material
residency/filtering and ocean optical/geometry work. No geometry prepass, extra
stream, separate directory or per-frame readback is added to production.

Water owns the spectral wave normal and dielectric response. Its material
roughness combines with unresolved spectral slope variance instead of deciding
whether water exists. Ray-source resource identity and capability accounting
include the borrowed scalar texture. Ordinary non-native shader consumers
without a bound semantic plane resolve to land, with no roughness fallback.

## Acceptance and deliberately separate work

- CPU area/shape/clone tests validate exact source-area mips and missing planes.
- The world producer is checked against independently sampled unclamped source
  height and its existing ice coverage at land/ocean sites.
- Regular/conforming GPU diagnostics check 0/64/255/missing coverage over
  roughness 10/115/240 and height -1/0/2 m, including reuse after water.
- Spectral geometry is invariant to a water roughness change and deterministic
  under pause/rewind; declared land remains fixed. The compact Bevy replay
  preserves every coverage mip while colour passes through its lossy codec.
- The wider material suite revealed an inherited invalid expectation that
  optical normals disappear when a canonical base-normal plane is absent.
  The failure reproduces at a3ad3dc before this patch. The fixture now checks
  the documented geometry-plus-optical fallback; its complete-height-plane
  Lambert/reference tests remain strict. No production normal change was made
  to accommodate that test.

Executed: 76 core + 33 terrain-stream CPU tests, the canonical producer control
and compact Bevy replay pass. Fourteen bounded software Vulkan tests and 49
Radeon 880M/RADV tests pass across diagnostics, material normals/residency,
ocean geometry/FFT, primary hybrid shadows, reflected ray transport and motion.
Strict native/render/Bevy Clippy, client compile and the architecture guard pass.
The complete High B03 1920×1080/4× MSAA sequence has 17 views with identical
depth, unchanged published geometry and byte-identical beauty round trip.
Candidate/submitted mesh triangles are 19,660,054/2,775,318. Its asynchronous
height/topology publication differs from the earlier capture, so the two
processes are not presented as a frozen before/after quality or timing pair.
No Bevy window/client GPU launch was performed.

Commands, identities, positive/negative controls and executed outcomes are
recorded in [the artifact](artifacts/liquid-water-contract-2026-11-03.json).
This closes the independent water-presence slice of doc 119 step 2. It does not
close source-versus-chord error, subtexel coastline integration, finite water
optics, sky visibility, bounce, full receiver buffers or affordable parity.
