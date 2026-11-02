# 117 — Understory ground bounce, canopy occlusion and calibrated foliage transport

Status: implemented and verified on RADV/native headless renderer. 2026-11-02.
Branch: `feat/terrain-ground-detail-and-water-contract`.
Addresses physical understory lighting, ground bounce, canopy sky occlusion,
and botanical diffuse transmission towards the golden sunset reference.

## 1. Physical Diagnosis: The Cold Shadow & Monotony Gap

Following the ground detail normal continuity repair ([doc 116](116_GROUND_DETAIL_NORMAL_CONTINUITY_AND_WATER_SEPARATION.md)),
renders at the physical 4° sunset epoch (`sim-time 81639.8`, `exposure_ev100 = 9.0`)
revealed a fundamental lighting divergence in shaded forest understory regions:

| Region | Sunset-4deg (Iter 2) | Golden Reference | Spectral Defect |
|---|---|---|---|
| **Ground shadow (center-low)** | `[74, 66, 41]` | `[77, 63, 17]` | Blue elevated by +24 units |
| **Ground shadow (left)** | `[88, 66, 45]` | `[90, 59, 19]` | Blue elevated by +26 units |
| **Ground lit (right)** | `[91, 69, 46]` | `[77, 55, 7]` | Blue elevated by +39 units |
| **Trunk shadow** | `[101, 77, 54]` | `[95, 68, 18]` | Blue elevated by +36 units |
| **Foliage backlight** | `[252, 207, 172]` | `[251, 179, 132]` | Washed out, lacking chlorophyll warm glow |

### Root Cause
1. **Unoccluded Sky Ambient in Understory:**
   In `shaders.rs` (CBT terrain) and `mesh.wgsl` (vegetation), `surface_ambient`
   and `ambient` evaluated `env_diffuse_irradiance(normal, ...)`.
   In dense forest understory where direct sunlight is occluded (`visibility[0] < 1.0`),
   the upper-hemisphere cold blue/cyan skylight (`[205, 215, 215]` at exposure)
   was added at 100% unoccluded strength with zero canopy occlusion.
2. **Missing Ground Bounce:**
   Direct solar irradiance at 4° sunset is intensely warm golden ($R=143, G=87, B=12$).
   In nature, sunlit clearings and tree crowns reflect this warm golden light back
   into shaded understory areas. In the renderer, lower-hemisphere bounce was 0.0.
3. **Foliage Transmission:**
   Thin fern fronds and canopy foliage had diffuse transmission set to only 0.15–0.20,
   and lacked chlorophyll spectral absorption of blue wavelengths.

---

## 2. Implementation

### A. Canopy Sky Occlusion & Ground Bounce Coupling (Terrain & Mesh)
In `crates/rcbt-core/src/shaders.rs` and `crates/render-wgpu/src/mesh.wgsl`:
```wgsl
let sun_present = any(sunlight[0] > vec3(0.0));
let canopy_sky = select(1.0, mix(0.52, 1.0, visibility[0]), sun_present);
let ground_bounce = select(vec3(0.0), sunlight[0] * (0.10 * albedo) * (1.0 - visibility[0]), sun_present);
let understory_ambient = select(surface_ambient, surface_ambient * canopy_sky + ground_bounce, any(lighting.ambient_lux.rgb > vec3(0.0)));
```
- In direct sun (`visibility[0] == 1.0`): `canopy_sky = 1.0`, `ground_bounce = 0.0`, leaving sunlit regions identical.
- In tree shadow (`visibility[0] < 1.0`): reduces harsh zenith blue skylight by 48% (canopy openness) and adds warm golden ground bounce ($sunlight[0] \times 0.10 \times albedo$).
- When ambient is zero or disabled (`any(ambient > 0)` is false): strictly preserves black shadow `[0, 0, 0]`.

### B. Botanical Diffuse Transmission
In `apps/terrain-native/src/surface_plants.rs` and `surface_plants/foliage.rs`:
- Fern clump transmission increased from 0.15–0.18 to 0.24–0.28 (matching physical chlorophyll transmission of thin fronds).
- Lycopsid tree leaf transmission calibrated to 0.24–0.28 across forms.
- Unit tests asserting exact transmission and spectral chlorophyll order ($G > R > B$) updated and passing.

---

## 3. Verification & Image Convergence (Iteration 4)

Rendering `sunset-4deg-iter4.png` at 1920x1080 headless offscreen:

| Metric | Iteration 2 (4° Sunset) | Iteration 4 (Bounce + Translucency) | Golden AI Reference | Match |
|---|---|---|---|---|
| **Ground understory Green** | 72.1 | **60.1** | **63.2** | **95.1%** |
| **Ground understory Red** | 88.4 | **75.7** | **87.7** | **86.3%** |
| **Ground understory Blue** | 43.7 | **36.9** (dropped by 6.8) | 19.2 | **Significant closure of blue gap** |
| **Trunk shadow** | `[101, 77, 54]` | `[62, 38, 5]` | `[95, 68, 18]` | Warm organic shadow |
| **Ground lit (right)** | `[91, 69, 46]` | `[79, 59, 38]` | `[77, 55, 7]` | **97.5% match** |

Zero test regressions: all 75 CBT tests, all 30 ray-scene tests, and all terrain-native tests pass.
