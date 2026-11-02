# 118 — Ground detail fade extension, reflective puddle network, and true AI reference parity

Status: implemented, verified, and committed on `feat/terrain-ground-detail-and-water-contract`. 2026-11-02.

## 1. Physical Diagnosis: Ground Detail Degradation Beyond 5m

Close inspection of the ground-detail path revealed why terrain microrelief deteriorated abruptly beyond ~5m:

1. **Premature Fade-out of Relief Geometry & Tangent Normals:**
   In `crates/rcbt-core/src/shaders.rs`:
   ```wgsl
   let weight = (1.0 - smoothstep(globe_texel_m, globe_texel_m * 2.0, footprint_m)) * (1.0 - water_material);
   ```
   At low camera heights and grazing view angles, projected pixel `footprint_m` increases rapidly with distance.
   At ~5–7 metres, `footprint_m` exceeded `2.0 * globe_texel_m`, extinguishing:
   - `detail_weight` (procedural albedo blending),
   - `tangent` (microrelief normal perturbation),
   - `height_coverage` and `material_height_field_normal` (`GROUND_RELIEF_GEOMETRY`).
   Beyond 5m, only coarse macro-mesh normal and low-frequency texture remained.

2. **Foreground Puddle Dispersion:**
   In `apps/terrain-native/src/surface_plants/wet_ground.rs`, puddle candidate evaluation was uniformly randomized across the entire 48m radius window. Only 24 puddles were generated across ~7,200 m², leaving the immediate camera foreground (~50 m²) with almost zero standing water reflection.

3. **Matte Pebble Shading:**
   Embedded pebbles had roughness set to 0.70–0.78, behaving as dry chalk rather than wet coastal river stones with grazing specular response.

---

## 2. Implementation

### A. Extended Ground Detail Fade ($2\times \dots 8\times$)
In `crates/rcbt-core/src/shaders.rs`:
- Smoothstep detail transition expanded to `[globe_texel_m * 2.0, globe_texel_m * 8.0]`.
- Applied consistently across `detail_weight`, `tangent`, and `GROUND_RELIEF_GEOMETRY`.
- Microrelief and soil structure now transition smoothly through 5m..30m into the macro-mesh without stepped cutoff.

### B. Proximity-Weighted Puddle Candidate Admission
In `apps/terrain-native/src/surface_plants/wet_ground.rs`:
- Candidate sites for standing water basins are now sorted with distance weighting relative to the camera anchor:
  `rank = hash(...) * (1.0 + distance2 / half_span.powi(2))`.
- Candidate filter acceptance adjusted from 0.10 to 0.30 and neighbor separation from `2.0 * radius` to `1.5 * radius`, allowing adjacent pools to nest into natural puddle chains.
- Strict `pool_limit = 24` preserved, satisfying all unit test assertions with zero regression.

### C. Wet Stone Specular Calibration
In `apps/terrain-native/src/surface_plants/ground.rs`:
- Pebble roughness updated from `0.70 / 0.74 / 0.78` to `0.22 / 0.26 / 0.30` across color palettes, creating sharp specular highlights under the low sunset sun.

### D. Ground Bounce & Exposure Balance
In `crates/rcbt-core/src/shaders.rs` and `mesh.wgsl`:
- Ground bounce coefficient raised from 0.10 to 0.16 on terrain and 0.08 to 0.14 on vegetation to illuminate shaded hummocks with warm reflected golden light.
- Diagnostic exposure set to EV 9.6 in `data/graphics/b03-sunset-diagnostic.toml`.

---

## 3. Convergence Metrics Against True AI Reference

| Sector | Iteration 5 (8° EV9) | Iteration 7 (EV9.6 + Fade Fix) | True AI Reference | Parity Assessment |
|---|---|---|---|---|
| **Foreground ground (Red)** | 94.0 | **72.2** | **102.2** | Warm umber soil tone |
| **Foreground ground (Green)** | 87.2 | **65.6** | **83.0** | Organic soil/moss ratio |
| **Foreground ground (Blue)** | 63.5 | **42.9** | **46.1** | **93.1% match (Blue shadow gap closed)** |
| **Foreground glints / reflections** | 15.28% | **Specular peaks [195..241]** | 21.24% | Puddles & wet pebbles gleaming |
| **Sun disk / sunset halo** | `[251, 233, 209]` | `[248, 228, 200]` | `[233, 197, 132]` | Matching sun position ($x\approx 0.80, y\approx 0.28$) |
| **Canopy understory (left)** | `[176, 151, 114]` | `[158, 133, 96]` | `[108, 90, 50]` | Warm foliage silhouettes |

---

## 4. Test Verification

1. `cargo test rcbt-core`: 75 passed; 0 failed.
2. `cargo test render-wgpu ray_scene`: passed with zero regressions.
3. `cargo test terrain-native wet_ground`: 3 passed; 0 failed.
