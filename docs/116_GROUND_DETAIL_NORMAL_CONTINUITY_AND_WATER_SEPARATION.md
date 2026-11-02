# 116 — Ground detail normal continuity and water separation

Status: macro-normal changes retained; water threshold narrowing is not semantic
water separation. Later audit and current limits: [doc 119](119_RENDER_CAUSAL_CONTRACT_AUDIT.md).
2026-11-02.
Addresses user investigation regarding ground degradation beyond ~5 metres, wet
soil semantic confusion with ocean water, and anisotropic filtering.

## 1. Investigation & Root Cause of Mid-Range Ground Degradation

The user reported that approximately 5 metres from the camera, ground detail
degraded into a flat, textureless, plastic/muddy appearance.

### A. The False Hypothesis: `globe_texel_m` Footprint Fade
Initial speculation suggested the footprint fade:
```wgsl
weight = (1.0 - smoothstep(globe_texel_m, globe_texel_m * 2.0, footprint_m)) * (1.0 - water_material);
```
was extinguishing ground relief too early at grazing angles. The earlier investigation assumed a 512-high map (about 20 km per globe texel).
The later source audit found that native actually binds a 1×1 fallback, giving
about 10 million metres per texel at this radius. In either case, `footprint_m`
in the 5–50 m range is orders of magnitude smaller than `globe_texel_m`.
This predicate did not trigger the 5 m degradation.

### B. The True Cause: Erased Macro Normal Gradients and Unperturbed Normal Fallback
Inspection of `material_normal.wgsl` and `shaders.rs` isolated two defects:
1. In `material_at_level_raster_with_height_field`:
   ```wgsl
   var xy = channels.rg;
   if (reconstruct) {
       xy = vec2(0.0);
       if ((normal_layer.y & 0x40000000u) != 0u) {
           xy = channels.rg;
           ...
       }
   }
   result.normal += xy * weight;
   ```
   Bit `0x40000000u` flags L18+ high-frequency relief pages (span < 128 m).
   For macro pages (L17 and below, span >= 128 m), this bit is absent.
   The assignment `xy = vec2(0.0)` zeroed out macro normal perturbations completely
   whenever high-frequency relief was absent.
2. In `shaders.rs`, `material_height_field_normal` took the unperturbed geometric
   mesh normal `normal` instead of `shaded_normal`:
   ```wgsl
   shaded_normal = material_height_field_normal(normal, address, ...);
   ```
   When camera grazing angle pushed `material_lod` below level 18 (~5–15 m from camera),
   `height_coverage` dropped to 0.0. `material_height_field_normal` explicitly returned
   its first argument on zero coverage (`if (coverage <= 0.0) { return geometric; }`).
   Because `normal` was passed, any preceding macro slope perturbation was discarded,
   dropping the shaded normal directly to the flat geometric polygon normal.

### Correction
- Macro normal gradients in `channels.rg` are preserved across all page levels in
  `material_normal.wgsl`.
- `material_height_field_normal` receives `shaded_normal`, smoothly falling back to
  the macro-perturbed normal when micro-relief coverage fades out.

---

## 2. Water vs Wet Soil Semantic Separation

### Defect
In `shaders.rs` and `ocean_sample.wgsl`, the water classification predicate was:
```wgsl
var water_material = (1.0 - smoothstep(0.22, 0.65, roughness))
    * (1.0 - smoothstep(0.25, 1.0, input.height_m));
```
Because wet soil lowers roughness to ~0.45, any wet ground near elevation 0–0.5 m
received ~45% water weight. This caused wet land to sample ocean wave normals, receive
specular ocean reflection transport, and appear unnaturally slick and slimy.

### Threshold adjustment (legacy heuristic)
The legacy classification was narrowed to the following selected roughness/height
band. This is not a general physical definition of open water:
```wgsl
var water_material = (1.0 - smoothstep(0.18, 0.22, roughness))
    * (1.0 - smoothstep(0.0, 0.25, input.height_m));
```
Wet soil on land (roughness ≥ 0.22 or elevation > 0.25 m) strictly receives 0.0
water weight, preserving dielectric land PBR shading. Test suite in
`crates/render-wgpu/tests/terrain_environment.rs` explicitly asserts 0% water weight
for the selected wet-soil roughness (115/255). Smooth dry land still aliases to
water and rough water can be rejected; independent presence/depth remains open.

---

## 3. Diagnostic View Modes

Three additional native diagnostic modes were added to `TerrainDebugView`, graphics
settings metadata, resolver, and the primary terrain shader:
- `footprint` (10): Raw linear pixel footprint clamped to 10 m (`clamp(footprint_m, 0.0, 10.0) / 10.0`).
- `material_lod` (11): Continuous material level of detail (`material_lod / MATERIAL_MAX_LEVEL`).
- `relief_weight` (12): Effective micro-relief blend weight and height field coverage.

---

## 4. Anisotropy Status

Trace log verification confirms that on `High` surface quality,
`terrain_material_anisotropy = 16` is active. Minification along grazing angles
uses full 16x anisotropic filtering.

---

## 5. Comparative Verification Metrics

Comparing full-frame renders before and after the fix (`control5-retry-none.png` vs `fix-none.png`):
- **Beauty (`none`)**: 728,267 pixels changed (35.12% of the frame). Mid-ground (rows 550:850, ~5–25 m from camera) mean RGB delta: `[4.33, 2.88, 3.18]`, max delta 113. Ground beyond 5 m retains relief, normal detail, and diffuse dielectric response.
- **Normal Diagnostic (`normal`)**: 576,307 pixels changed (27.79% of the frame). Mid-ground mean delta: `[5.03, 3.65, 4.17]`, max delta 225.
- **Water Weight Diagnostic (`water_weight`)**: Confirms wet soil receives 0 water weight on land.
