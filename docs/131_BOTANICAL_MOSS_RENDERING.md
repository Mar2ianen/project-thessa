# 131 — Botanical moss in the native ground renderer

Status: integrated pinned CC0 shoot geometry and PBR cutouts through the existing
native renderer. This improves the represented plant organs; realistic dense
moss, close soil and whole-scene reference acceptance remain open.

The reference is [Poly Haven Moss 01](https://polyhaven.com/a/moss_01), by Rob
Tuytel, CC0. Its source glTF contains a small organ gallery; the much denser
Geometry Nodes preview is not imported. Three six/eight-triangle shoot forms
replace the previous untextured curved strips. Source dimensions, folded
geometry and UVs are retained with a uniform growth scale, rather than
stretching every form to the same height. Roots still sample canonical optical
soil independently. A tighter colony footprint groups the same 24 shoots;
it adds no population or density demand outside those existing sites.

OpenEngine `TerrainGrass/Shaders/TerrainGrass/terrain_grass_surface.glsl` at
`bdf5c2710fe37c1c5ddbac7787477daa87524af3` was inspected for its shared textured
blade albedo/alpha/normal path and alpha-to-coverage contract. Thessa reuses its
existing glTF/PBR cutout, MSAA coverage, shadow and ray-query consumers; no
OpenEngine shader is copied or new render pass introduced.

`import_moss_shoots.py` packs the verified original buffers/maps into a
1,050,036-byte GLB. Diffuse JPEG and the separate source alpha are combined
losslessly into RGBA PNG. Standard normal/ORM maps are retained. The original
BLEND material becomes explicit MASK at 0.5; unsupported specular/IOR extensions
are removed in favor of the existing native dielectric prior. This conversion
is documented, not a claim of exact Blender material parity. Source URLs,
MD5/SHA-256 and packed SHA-256 accompany the asset. No runtime download occurs.

All three tint materials share one base/normal/ORM set, using the existing
foliage material/image remapping helper. High/Medium limits remain 4,250/1,200
moss colonies and 21,200/1,600 grass tufts. Low/Off publish no moss images or
geometry. Three additional 1024² source images cost 12,582,912 decoded bytes;
textures are separate from the unchanged 48 MB mesh admission limit.

## Actual checks and review

- Pinned-source tests verify three lightweight forms, real alpha holes and the
  standard PBR maps. Existing grass-tip/root, moss area/volume, population,
  image-sharing and full-canopy source-admission checks pass. Shared fern
  material remapping also retains its existing attachment/alpha acceptance.
- The actual native mesh renderer renders a macro specimen and an opaque
  carrier control. The mask must retain visible moss and remove carrier pixels.
  This neutral, one-sample QA view isolates shape/material integration; it is
  not a coastal lighting reference or temporal-quality test.
- Fixed B03 runs at 1920×1080, High, 4× MSAA, RT Off, fixed time/camera and
  20-second readiness. All 20 frozen diagnostic views preserve published
  geometry, depth and beauty round-trip within each run. The full-scene debug
  build and serial stage fences have `performance_equivalent=false`: timings
  are not production FPS or a speedup measurement.
- The first wide-colony candidate reads as disconnected little shoots in its
  macro view. The follow-up groups roots more closely at identical shoot/site
  counts. Root authored this iteration and inspected it without blind labels; no
  independent aesthetic acceptance is asserted. Independent runs have different
  page readiness/publications and are not exact whole-frame pixel A/B controls.

Source mesh cost rises from v4/v5's 25,918,356 bytes to 33,531,332 bytes;
source images rise from 16,343,040 to 28,925,952 bytes. The 36 mesh draw groups
and instance population remain. This is a quality/cost tradeoff, not a free
performance improvement. Close soil remains muddy/repetitive and the global
scene remains far below the supplied visual target.

Capture commands, actual counters, macro PNGs and hashes are preserved in
[the evidence record](artifacts/botanical-moss-rendering-2026-11-03.json).

## Deferred trace lifetime policy

Per the user's direction, trace filling is a future simple visual lifetime
policy: planet-level erosion speed, a Bezier progression and a specified end
time. Atmosphere-bearing worlds may enable it; airless worlds default to
retaining traces. Wind/rain sediment simulation is not required. Engine/landing-
support interactions and live surface refresh are postponed while rendering
quality is the current priority. This pass adds no decay model or planet knob.

The current macro removes 5,452 pixels compared with the opaque carrier
control while retaining visible moss. The FHD candidate submits 3,680,080
primary mesh triangles and 51,040,872 shadow mesh triangles over three passes,
versus 3,408,188 / 50,225,196 in the preceding source. Counts express added
work; neither debug frame duration nor these counts prove production speed.

Doc [132](132_FOREST_QUALITY_ITERATIONS.md) records subsequent plant-community,
cattail and low-grass changes. The counters above describe this moss iteration's
capture, not the later scene membership or source group count.
