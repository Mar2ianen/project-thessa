# 122 — Native mesh interpolation on covered MSAA samples

Status: native mesh coverage interpolation corrected. This is a raster input
contract fix, not an art adjustment, a new effect or proof of reference parity.

## Reproduced cause

The native mesh vertex output used the default perspective/centre sampling for
normals, UVs, camera-relative position and baked ambient visibility. A thin
triangle can cover MSAA subsamples while missing the pixel centre. Evaluating
attributes at that centre extrapolates outside the triangle: a visibility field
whose three source values are [1,1,0] can become negative. Texture coordinates
and the shaded/reflected position can similarly leave the covered surface.

A bounded production-renderer fixture places one subpixel triangle in a 65²
4× MSAA target. Its material has unit diffuse albedo, no direct light or
reflection, and uniform PI skylight. Before correction the actual HDR minimum
is **-0.33325195**, with **93 negative RGB channels**, despite valid source
visibility. The old full B03 receiver audit also exposes negative values before
screen-space visibility is applied; that control is retained as failed input
evidence rather than attributed to the new visibility pass.

The four floating-point mesh varyings now use
`@interpolate(perspective, centroid)`. This evaluates attributes within the
covered primitive while retaining perspective interpolation and pixel-frequency
shading. There is no radiance clamp, altered source AO, additional image/pass,
geometry change or sample-frequency shading requirement. Single-sample rendering
still evaluates the one covered sample. The source and sampling semantics follow
[WGSL interpolation](https://www.w3.org/TR/WGSL/#interpolation) and
[Vulkan shader interpolation](https://docs.vulkan.org/spec/latest/chapters/shaders.html).

## Validation scope

The same fixture now has minimum 0 and no negative channel, while retaining
positive subpixel coverage (maximum 0.14733887 on isolated software Vulkan and
0.14794922 on Radeon 880M/RADV). Source payloads and geometry were not changed.
The receiver, material/light, hybrid-shadow and motion regressions were rerun
on RADV; the simultaneous visibility-control fixture verifies HDR identity when
its correction is disabled for both single-sample and 4× MSAA rendering.
Raw red/green commands, binaries and outputs are recorded in
[the artifact](artifacts/msaa-centroid-2026-11-03.json).

The fix covers the native `MeshRenderer`, including opaque, MASK and BLEND
varyings. Bevy uses its own mesh shaders. General moving-camera foliage
stability, shadow softness, sky/soil quality and reference acceptance remain
separate work; centroid interpolation cannot recover unresolved geometry.
