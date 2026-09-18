# X-15 asset provenance

## `assets/models/north_american_x-15_plane.glb`

- Purpose: imported visual mesh for the pilot preview instead of a procedural
  placeholder.
- Source: Sketchfab, **North American X-15 Plane**, by cmoreau:
  https://sketchfab.com/3d-models/north-american-x-15-plane-bf491206ba844282949734b48b938c53
- License: CC BY 4.0. Required attribution is next to the binary asset in
  `north_american_x-15_plane.LICENSE.txt`.
- Import cleanup removed viewer camera/light/Cube objects. Model geometry and
  textures were preserved.

### Imported axes

Inside the GLB, the nose points along `-X`, the top/canopy/spine along `+Y`,
and span along `Z`. The root nodes contain scale only. Blender converts glTF
Y-up to its Z-up convention; Blender world axes must not be copied directly
into Bevy as source-file axes.

`x15_asset_to_craft_rotation()` maps `-X → +Y`, `+Y → -Z`, and `+Z → +X` in
the parent’s local coordinates. `render_orientation()` then applies the
physical orientation and Z-up to Y-up conversion. The physical craft axes are
therefore `+X` forward, `+Y` right/span, and `+Z` up. The navball uses the same
physical orientation.

Regression tests cover the complete chain, including the GLB child rotation,
for initial, pitch, roll, and combined attitudes.

## `assets/textures/x15-inconel-albedo-v1.png`

- Purpose: prototype visual albedo/metal texture for the X-15 flight-test
  asset; it is not runtime physics data or a claim about exact Inconel.
- Source: generated in the Codex ImageGen session for Project Thessa. The
  original is recorded in the repository’s asset history.
- No external material was copied.
- Format: PNG, `1254×1254`, 8-bit RGB.
- SHA-256: `d3bfed7618acb656e9686ec7c1d18f29d3f556cafdf825fd639ed038f1481779`.
- License: generated project asset, permitted for use and modification in
  Project Thessa; no third-party license is claimed.

The texture is not used by the imported mesh because the GLB contains its own
materials and texture images. This record is retained for future procedural
surface variants.
