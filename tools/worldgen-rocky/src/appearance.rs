//! Continuous surface materials derived from climate and terrain.
//! Biome/feature IDs remain discrete semantic data; they are not painted as
//! flat colour disks. No sunlight is baked into the albedo.
use crate::{
    biomes::Geology,
    field::{PlanetField, TerrainSample},
    rng,
};

/// Approximate unlit substrate reflectance, not the biome/debug colour legend.
/// Weathered loose cover and exposed rock remain separate optical materials.
/// Authored substrate reflectance pairs `(exposed stone, weathered soil)`.
/// Soils keep pairwise distance above ~0.09 so distinct geologies never
/// collapse into one mud mixture; same for stones. Adjust with the
/// `substrate_palettes_stay_distinguishable` regression, not by eye.
fn substrate_albedo(geology: Geology) -> ([f64; 3], [f64; 3]) {
    match geology {
        Geology::Basaltic | Geology::OceanicCrust => ([0.20, 0.21, 0.22], [0.27, 0.26, 0.24]),
        Geology::FelsicHighland => ([0.56, 0.54, 0.49], [0.54, 0.45, 0.37]),
        Geology::ContinentalCrust => ([0.43, 0.42, 0.40], [0.47, 0.36, 0.26]),
        Geology::Sedimentary => ([0.50, 0.42, 0.33], [0.58, 0.44, 0.28]),
        Geology::ImpactBreccia => ([0.31, 0.28, 0.27], [0.38, 0.29, 0.25]),
        Geology::Evaporite => ([0.77, 0.75, 0.69], [0.64, 0.59, 0.47]),
        Geology::GlacialTill => ([0.37, 0.43, 0.48], [0.45, 0.47, 0.43]),
        Geology::Regolith => ([0.36, 0.35, 0.33], [0.40, 0.38, 0.33]),
        Geology::Hydrothermal => ([0.42, 0.34, 0.26], [0.55, 0.33, 0.16]),
    }
}

#[derive(Clone, Copy, Debug)]
pub struct SurfaceAppearance {
    pub albedo_srgb: [f32; 3],
    pub roughness: f32,
    /// Bounded material-state response for the existing native normal grain.
    /// This changes shading detail only, never the canonical height field.
    pub normal_detail_scale: f32,
    pub vegetation: f32,
    pub snow: f32,
    /// Supply/geometry-limited visual cover, separate from cold suitability.
    pub snow_cover: f32,
    /// Optical frost mixing weight; zero outside the frost temperature band.
    pub frost_cover: f32,
}

#[derive(Clone, Copy)]
struct SubstrateSurfaceProfile {
    exposed_rock_detail: f64,
    loose_soil_detail: f64,
    exposed_roughness_delta: f64,
    loose_roughness_delta: f64,
}

/// Small authored rendering priors, not measured lithology parameters.
/// They modulate one shared world-addressed signal; geology never changes its
/// phase, adds noise octaves, or feeds physical terrain/contacts.
fn substrate_surface_profile(geology: Geology) -> SubstrateSurfaceProfile {
    match geology {
        Geology::Basaltic | Geology::OceanicCrust => SubstrateSurfaceProfile {
            exposed_rock_detail: 1.04,
            loose_soil_detail: 0.98,
            exposed_roughness_delta: 0.010,
            loose_roughness_delta: 0.0,
        },
        Geology::FelsicHighland => SubstrateSurfaceProfile {
            exposed_rock_detail: 0.98,
            loose_soil_detail: 1.00,
            exposed_roughness_delta: -0.006,
            loose_roughness_delta: 0.002,
        },
        Geology::ContinentalCrust => SubstrateSurfaceProfile {
            exposed_rock_detail: 1.00,
            loose_soil_detail: 1.00,
            exposed_roughness_delta: 0.0,
            loose_roughness_delta: 0.0,
        },
        Geology::Sedimentary => SubstrateSurfaceProfile {
            exposed_rock_detail: 1.02,
            loose_soil_detail: 1.04,
            exposed_roughness_delta: -0.006,
            loose_roughness_delta: -0.005,
        },
        Geology::ImpactBreccia => SubstrateSurfaceProfile {
            exposed_rock_detail: 1.08,
            loose_soil_detail: 1.02,
            exposed_roughness_delta: 0.012,
            loose_roughness_delta: 0.004,
        },
        Geology::Evaporite => SubstrateSurfaceProfile {
            exposed_rock_detail: 0.92,
            loose_soil_detail: 0.92,
            exposed_roughness_delta: -0.012,
            loose_roughness_delta: -0.010,
        },
        Geology::GlacialTill => SubstrateSurfaceProfile {
            exposed_rock_detail: 0.95,
            loose_soil_detail: 0.98,
            exposed_roughness_delta: -0.008,
            loose_roughness_delta: -0.003,
        },
        Geology::Regolith => SubstrateSurfaceProfile {
            exposed_rock_detail: 0.96,
            loose_soil_detail: 1.03,
            exposed_roughness_delta: 0.004,
            loose_roughness_delta: 0.006,
        },
        Geology::Hydrothermal => SubstrateSurfaceProfile {
            exposed_rock_detail: 1.04,
            loose_soil_detail: 0.98,
            exposed_roughness_delta: 0.012,
            loose_roughness_delta: 0.004,
        },
    }
}

#[derive(Clone, Copy)]
struct GeomorphologyResponse {
    hollow: f64,
    loose_cover: f64,
    exposed: f64,
}

fn normal_detail_response(
    profile: SubstrateSurfaceProfile,
    exposed_rock: f64,
    loose_cover: f64,
    vegetation: f64,
    damp: f64,
    snow_cover: f64,
    frost: f64,
) -> f64 {
    let rock = exposed_rock.clamp(0.0, 1.0);
    let soil_detail = profile.loose_soil_detail * (1.0 - damp.clamp(0.0, 1.0) * 0.08);
    let substrate_detail = soil_detail * (1.0 - rock) + profile.exposed_rock_detail * rock;
    (substrate_detail
        * (1.0 - loose_cover.clamp(0.0, 1.0) * 0.35)
        * (1.0 - vegetation.clamp(0.0, 1.0) * 0.35)
        * (1.0 - snow_cover.clamp(0.0, 1.0) * 0.95)
        * (1.0 - frost.clamp(0.0, 1.0) * 0.25))
        .clamp(0.0, 1.12)
}

fn substrate_roughness_delta(profile: SubstrateSurfaceProfile, exposed_rock: f64) -> f64 {
    let rock = exposed_rock.clamp(0.0, 1.0);
    profile.loose_roughness_delta * (1.0 - rock) + profile.exposed_roughness_delta * rock
}

/// Existing fixed-wavelength field signals shared by optical appearance and
/// its normal response. Regional displacement is evidence for exposure/cover,
/// not a fine sediment inventory or added relief.
fn geomorphology_response(sample: &TerrainSample) -> GeomorphologyResponse {
    let crest = smooth(0.02, 0.25, -sample.curvature_per_m * 256.0);
    let hollow = smooth(0.02, 0.25, sample.curvature_per_m * 256.0);
    let eroded = smooth(0.0, 40.0, -sample.erosion_displacement_m);
    let deposited = smooth(
        0.0,
        40.0,
        sample.erosion_displacement_m.max(sample.local_deposition_m),
    );
    let gentle = 1.0 - smooth(0.25, 0.7, sample.slope_hint);
    let loose_cover = gentle * hollow.max(deposited);
    let exposed =
        smooth(0.35, 0.85, sample.slope_hint).max(crest * eroded * (1.0 - loose_cover) * 0.15);
    GeomorphologyResponse {
        hollow,
        loose_cover,
        exposed,
    }
}
fn land_roughness(vegetation: f64, exposed: f64, damp: f64, texture: f64, frost: f64) -> f32 {
    (0.88 + vegetation * 0.06 - exposed * 0.18 - damp * (1.0 - vegetation) * 0.12
        + texture * exposed * 0.04
        - 0.25 * frost.clamp(0.0, 1.0))
    .clamp(0.45, 0.98) as f32
}

/// Regional maritime eligibility, not metre-scale shoreline distance. The
/// existing ocean-distance field is only resolved at its context-grid spacing;
/// never turn that coarse source into a fictitious precise coast query.
fn maritime_material_support(field: &PlanetField, sample: &TerrainSample) -> f64 {
    let distance_m = sample.continentality01 * crate::climate::CONTINENTALITY_DISTANCE_M;
    let spacing_m = field.ocean_context_spacing_m();
    1.0 - smooth(spacing_m, 2.0 * spacing_m, distance_m)
}

fn beach_material_coverage(maritime: f64, sample: &TerrainSample, exposed: f64) -> f64 {
    let unsalted = if sample.geology == Geology::Evaporite {
        0.0
    } else {
        1.0 - smooth(0.0, 0.25, sample.salt01)
    };
    maritime
        * (1.0 - smooth(3.0, 45.0, sample.height_m))
        * (1.0 - smooth(0.08, 0.35, sample.slope_hint))
        * (1.0 - exposed)
        * unsalted
}

fn mix(a: [f64; 3], b: [f64; 3], t: f64) -> [f64; 3] {
    std::array::from_fn(|i| a[i] + (b[i] - a[i]) * t.clamp(0.0, 1.0))
}
pub(crate) fn smooth(a: f64, b: f64, v: f64) -> f64 {
    let t = ((v - a) / (b - a)).clamp(0.0, 1.0);
    t * t * (3.0 - 2.0 * t)
}

/// Deterministic material-only grain. This is deliberately separate from the
/// canonical height field: it gives close terrain a readable surface pattern
/// without inventing collision relief or changing authoritative queries.
///
/// Range is roughly [-1, 1] (sum of weighted value-noise octaves).
pub fn surface_grain(field: &PlanetField, dir: [f64; 3]) -> f64 {
    surface_grain_filtered(field, dir, 0.0)
}

fn band_weight(scale_m: f64, texel_m: f64) -> f64 {
    1.0 - smooth(scale_m * 0.25, scale_m * 0.5, texel_m)
}

fn surface_grain_filtered(field: &PlanetField, dir: [f64; 3], texel_m: f64) -> f64 {
    [(8.0, 0.42), (32.0, 0.32), (128.0, 0.20), (512.0, 0.10)]
        .into_iter()
        .enumerate()
        .filter(|(_, (scale, _))| band_weight(*scale, texel_m) > 0.0)
        .map(|(band, (scale, weight))| {
            weight
                * band_weight(scale, texel_m)
                * rng::value_noise3(
                    field.params.seed,
                    741 + band as u32 * 17,
                    dir[0] * field.params.radius_m / scale,
                    dir[1] * field.params.radius_m / scale,
                    dir[2] * field.params.radius_m / scale,
                )
        })
        .sum()
}

/// The normal-map path must discard grain finer than two texels, otherwise a
/// low-resolution tile aliases its own detail instead of filtering it.
pub(crate) fn surface_grain_height(
    field: &PlanetField,
    dir: [f64; 3],
    texel_wavelength_m: f64,
) -> f64 {
    [(8.0, 0.42), (32.0, 0.32), (128.0, 0.20), (512.0, 0.10)]
        .into_iter()
        .enumerate()
        .filter(|(_, (scale, _))| *scale >= texel_wavelength_m * 2.0)
        .map(|(band, (scale, weight))| {
            let q = dir.map(|v| v * field.params.radius_m / scale);
            weight
                * 2.0
                * rng::value_noise3(field.params.seed, 741 + band as u32 * 17, q[0], q[1], q[2])
        })
        .sum()
}

/// Normal-plane grain with a multi-scale slope spectrum.
///
/// The legacy four-band height above is tuned for albedo/residual filtering;
/// used alone as a normal source, its 8 m octave reads as a single cellular
/// lattice. This variant adds equal-slope octaves below 8 m when the page's
/// texel footprint resolves them, so close ground gains isotropic roughness
/// while sub-Nyquist detail is still discarded.
pub(crate) fn surface_normal_grain_height(
    field: &PlanetField,
    dir: [f64; 3],
    texel_wavelength_m: f64,
) -> f64 {
    [
        (8.0, 0.42),
        (4.0, 0.21),
        (2.0, 0.105),
        (1.0, 0.0525),
        (0.5, 0.02625),
        (32.0, 0.32),
        (128.0, 0.20),
        (512.0, 0.10),
    ]
    .into_iter()
    .enumerate()
    .filter(|(_, (scale, _))| band_weight(*scale, texel_wavelength_m) > 0.0)
    .map(|(band, (scale, weight))| {
        let q = dir.map(|v| v * field.params.radius_m / scale);
        weight
            * band_weight(scale, texel_wavelength_m)
            * 2.0
            * rng::perlin3(field.params.seed, 941 + band as u32 * 29, q[0], q[1], q[2])
    })
    .sum()
}

pub fn surface_appearance(
    field: &PlanetField,
    sample: &TerrainSample,
    dir: [f64; 3],
) -> SurfaceAppearance {
    surface_appearance_filtered(field, sample, dir, 0.0)
}

/// Appearance-only spectral filtering; canonical height and semantics stay fixed.
pub fn surface_appearance_filtered(
    field: &PlanetField,
    sample: &TerrainSample,
    dir: [f64; 3],
    texel_m: f64,
) -> SurfaceAppearance {
    surface_appearance_filtered_with_normal_detail(field, sample, dir, texel_m, true)
}

pub(crate) fn surface_appearance_filtered_with_normal_detail(
    field: &PlanetField,
    sample: &TerrainSample,
    dir: [f64; 3],
    texel_m: f64,
    normal_detail_enabled: bool,
) -> SurfaceAppearance {
    surface_appearance_with_ground_detail(field, sample, dir, texel_m, normal_detail_enabled, None)
        .0
}

pub(crate) fn surface_appearance_with_ground_detail(
    field: &PlanetField,
    sample: &TerrainSample,
    dir: [f64; 3],
    texel_m: f64,
    normal_detail_enabled: bool,
    ground: Option<crate::ground_detail::GroundSample>,
) -> (SurfaceAppearance, f64) {
    let noise = |channel, scale, octaves| {
        if texel_m > 0.0 {
            let mut sum = 0.0;
            let mut norm = 0.0;
            let (mut amplitude, mut frequency) = (1.0, 1.0);
            for octave in 0..octaves {
                let weight = band_weight(scale / frequency, texel_m);
                if weight > 0.0 {
                    let q = dir.map(|v| v * field.params.radius_m / scale * frequency);
                    sum += amplitude
                        * weight
                        * rng::value_noise3(
                            field.params.seed,
                            channel + octave * 7919,
                            q[0],
                            q[1],
                            q[2],
                        );
                }
                norm += amplitude;
                amplitude *= 0.5;
                frequency *= 2.03;
            }
            return sum / norm;
        }
        rng::fbm3(
            field.params.seed,
            channel,
            dir[0] * field.params.radius_m / scale,
            dir[1] * field.params.radius_m / scale,
            dir[2] * field.params.radius_m / scale,
            octaves,
        )
    };
    let regional = noise(731, 360_000.0, 3);
    let variation = noise(733, 65_000.0, 3);
    // Independent micro-relief noise (24 m + 12 m octaves): deliberately NOT
    // the cover grain — sharing one signal for threshold patches and final
    // brightness partially cancels (opposite signs), muting both.
    // Reuse the geometry/displacement masks used for other surface choices;
    // no new random cover mask or inferred local sediment inventory.
    let geomorphology = geomorphology_response(sample);
    let hollow = geomorphology.hollow;
    let loose_cover = geomorphology.loose_cover;
    // Regional displacement is not a local snow inventory. Curvature may
    // bias exposure weakly, but must not strip every convex crest into an
    // artificial contour line; actual steepness remains the main exposure.
    let exposed = geomorphology.exposed;
    // Accumulation smooths the small material bands; exposed rock retains
    // them. This changes appearance only, never collision/height or biomes.
    let texture_gain = 1.0 - loose_cover * 0.65;
    let micro = noise(739, 24.0, 2) * texture_gain;
    let grain = surface_grain_filtered(field, dir, texel_m) * texture_gain;
    // Weathering/cover structure needs intermediate physical scales; tiny
    // grain alone disappears into the average at a survey camera. Exposures
    // and loose deposits do not receive the same uniformly smeared pattern.
    let weathering = noise(751, 384.0, 3);
    let outcrop_texture = noise(757, 96.0, 3);
    let cover_texture = noise(761, 192.0, 3);
    let h = sample.height_m;
    // Snow cover breaks into drifts and thaw patches: low-frequency drift
    // signals ride the threshold (±2 K) so a uniform sub-zero plain still
    // reads as structured snow instead of a flat fill, while pixel-scale
    // grain only textures decided snow instead of flipping coverage.
    // Classification inputs (height/moisture/temperature/slope) are
    // untouched — only the visual coverage and the final albedo carry
    // the pattern.
    let snow = smooth(
        276.0,
        264.0,
        sample.temperature_k + regional * 5.0 + variation * 2.0 + grain * 2.0,
    );
    if h < 0.0 {
        // Shelf width varies along the coast (sediment supply and slope
        // really do vary); a fixed-depth contour would ring every continent
        // with a uniform sticker line. The linear profile spreads the
        // shallows transition instead of front-loading it into one pixel.
        let shelf_depth_m = 700.0 * (1.0 + regional * 0.6).max(0.4);
        let shelf = (-h / shelf_depth_m).clamp(0.0, 1.0);
        // Deep water is not a single paint fill: gyre-scale tonal drift.
        // Shallow water takes lagoon-to-sediment tints that vary by reach;
        // `regional` (360 km) survives coarse-texel filtering where the
        // finer drivers wash out.
        let drift = 1.0 + regional * 0.08 + variation * 0.05;
        let shallow = mix(
            [0.16 * drift, 0.40 * drift, 0.38 * drift],
            [0.38 * drift, 0.44 * drift, 0.29 * drift],
            ((regional * 0.5 + 0.5) * 0.6).clamp(0.0, 1.0),
        );
        let c = mix(shallow, [0.018 * drift, 0.095 * drift, 0.19 * drift], shelf);
        let c = mix(c, [0.014, 0.044, 0.11], smooth(600.0, 6000.0, -h));
        // Shallow scatter: turquoise lift in the first ~60 m, broken by grain
        // so the coast reads as a shelf, not a flat cyan disk. Earth-ref
        // (ISS shallow reefs): bright rim only where water is thin.
        let turq = 1.0 - smooth(8.0, 60.0, -h);
        let shelf_break = 0.6 + 0.4 * grain.clamp(-1.0, 1.0);
        let mut c = mix(
            c,
            [0.16, 0.55, 0.55],
            (turq * 0.35 * shelf_break).clamp(0.0, 1.0),
        );
        // Surf foam: narrow band around the datum, broken into streaks
        // by the independent grain + micro signals. Classification inputs
        // (height/moisture/temperature/slope) are untouched.
        let foam_band = 1.0 - smooth(0.5, 4.0, h.abs());
        let foam_noise = smooth(0.05, 0.55, grain * 0.5 + micro * 0.5 + 0.3);
        let ice = smooth(263.0, 253.0, sample.temperature_k + regional * 6.0);
        let foam = (foam_band * foam_noise * (1.0 - ice)).clamp(0.0, 1.0);
        c = mix(c, [0.92, 0.95, 0.94], foam * 0.85);
        // Open-water sun glitter variation: micro sparkle on top of the grain
        // so large swells do not render as a mirror-flat fill at glancing sun.
        let sparkle = 1.0 + grain * 0.10 + micro * 0.06;
        let base = mix(c, [0.75, 0.84, 0.86], ice).map(|x| (x * sparkle).clamp(0.0, 1.0) as f32);
        return (
            SurfaceAppearance {
                albedo_srgb: base,
                // Foam is matte (kills the glint in the surf, like real white water);
                // ice stays rough; open water keeps the sun-glint roughness.
                roughness: (0.16 + 0.58 * ice + 0.49 * foam * (1.0 - ice)) as f32,
                normal_detail_scale: 0.0,
                vegetation: 0.0,
                snow: ice as f32,
                snow_cover: ice as f32,
                frost_cover: 0.0,
            },
            0.0,
        );
    }
    let moisture = (sample.moisture01 + regional * 0.24).clamp(0.0, 1.0);
    let ecological_cover = crate::ecology::vegetation_cover(sample, sample.slope_hint);
    let vegetation = ecological_cover.total();
    let (stone, weathered_soil) = substrate_albedo(sample.geology);
    let soil = weathered_soil.map(|c| c * (1.0 + weathering * 0.18));
    let damp = smooth(0.35, 0.95, moisture);
    let mut dry = soil.map(|c| c * (1.0 - damp * 0.25));
    // Only the bare substrate receives coastal fines and the provisional
    // capillary rim. Apply before the existing cover layers so fractional
    // vegetation/mud keeps the same opacity as roughness and normal response.
    let maritime = maritime_material_support(field, sample);
    let beach_cover = beach_material_coverage(maritime, sample, exposed);
    let beach_albedo = weathered_soil.map(|c| c * (1.0 + cover_texture * 0.12 - damp * 0.18));
    dry = mix(dry, beach_albedo, beach_cover);
    let wet_rim = (1.0 - smooth(0.0, 3.0, h)) * beach_cover;
    dry = dry.map(|c| c * (1.0 - wet_rim * 0.18));
    // The existing ecology state distinguishes canopy, ground plants and reeds.
    // Do not collapse all three into one full-coverage green paint bucket.
    // These are filtered optical cover proxies, not rendered tree geometry.
    let grass = mix([0.33, 0.35, 0.19], [0.20, 0.29, 0.16], damp);
    let canopy = mix([0.18, 0.26, 0.13], [0.09, 0.19, 0.12], damp);
    let reeds = mix([0.31, 0.33, 0.18], [0.18, 0.27, 0.16], damp);
    let plant = std::array::from_fn(|i| {
        (grass[i] * ecological_cover.ground01
            + canopy[i] * ecological_cover.canopy01
            + reeds[i] * ecological_cover.reeds01)
            / vegetation.max(1e-9)
    });
    let plant = plant.map(|c| c * (1.0 + cover_texture * 0.22));
    let mut color = mix(dry, plant, vegetation);
    // Wetland potential is saturation/cover context, not an opaque material
    // above resolved litter. Keep it beneath the optional measured floor.
    let wetland = sample.wetland_potential01 * (1.0 - smooth(0.02, 0.1, sample.slope_hint));
    let wet_ground = mix([0.20, 0.23, 0.16], [0.10, 0.26, 0.16], vegetation);
    color = mix(color, wet_ground, wetland);
    // Resolved soil/gravel beneath canopy is a material swatch, not another
    // macro cover-noise field. Existing later covers retain their ownership.
    let mut ground_weight = 0.0;
    if let Some(ground) = ground {
        ground_weight =
            (ecological_cover.canopy01 * 1.30 + ecological_cover.ground01 * 0.60).clamp(0.0, 1.0)
                * (1.0 - exposed);
        let h_norm = (f64::from(ground.height_m)
            / (crate::ground_detail::HEIGHT_RANGE_M as f64 * 0.5))
            .clamp(-1.0, 1.0);
        let q = dir.map(|v| v * field.params.radius_m);
        let w_meso = band_weight(3.2, texel_m);
        let w_micro = band_weight(1.1, texel_m);
        let n_macro = rng::perlin3(field.params.seed, 6141, q[0] / 8.4, q[1] / 8.4, q[2] / 8.4);
        let n_litter = rng::perlin3(field.params.seed, 6142, q[0] / 3.2, q[1] / 3.2, q[2] / 3.2);
        let n_moss = rng::value_noise3(field.params.seed, 6143, q[0] / 1.1, q[1] / 1.1, q[2] / 1.1);

        // Accumulated dry senescent frond / needle litter around plant colonies:
        let litter_signal = ((n_macro * 0.40 + n_litter * 0.60) * w_meso).clamp(-1.0, 1.0);
        let litter_warmth = [1.42, 0.98, 0.52];

        // Peat and organic humus darkening in moist forest environments:
        // Damp coastal forest soils have rich organic matter, absorbing light down to 6-10% reflectance.
        let humus_base = 1.0 - damp * 0.52;
        let hollow_darkening = 1.0 + (h_norm.min(0.0) * 0.45 + litter_signal.min(0.0) * 0.35);
        let humus_factor = (humus_base * hollow_darkening).clamp(0.22, 1.0);

        // Dry senescent fronds on elevated mounds:
        let mound_drying = 1.0 + h_norm.max(0.0) * 0.45 + litter_signal.max(0.0) * 0.35;

        // Fine bryophyte (moss) colonies on moist elevated hummocks:
        let moss_eligible = h_norm > -0.15 && damp > 0.45 && ecological_cover.canopy01 > 0.25;
        let moss_cover = if moss_eligible {
            smooth(0.25, 0.65, n_moss) * (h_norm + 0.15).clamp(0.0, 1.0) * w_micro * 0.35
        } else {
            0.0
        };
        let moss_linear = [0.08, 0.15, 0.04];

        // Scattered bleached twigs and decaying wood debris:
        let n_twig = rng::value_noise3(field.params.seed, 6144, q[0] / 0.45, q[1] / 0.45, q[2] / 0.45);
        let twig_cover = if h_norm > -0.25 {
            smooth(0.70, 0.84, n_twig) * w_micro * 0.35
        } else {
            0.0
        };
        let twig_linear = [0.22, 0.18, 0.13];

        color = std::array::from_fn(|i| {
            let base = f64::from(ground.albedo_linear[i]);
            let litter_gain = 1.0
                + (h_norm.max(0.0) * 0.55 + litter_signal.max(0.0) * 0.45)
                    * (litter_warmth[i] - 1.0);
            let floor_raw = base * humus_factor * litter_gain * mound_drying;
            let moss_term = moss_linear[i] * (base / 0.15);
            let floor_organic = floor_raw * (1.0 - moss_cover) + moss_term * moss_cover;
            let floor_linear = floor_organic * (1.0 - twig_cover)
                + twig_linear[i] * (base / 0.15) * twig_cover;
            crate::ground_detail::encode_srgb(
                crate::ground_detail::decode_srgb(color[i]) * (1.0 - ground_weight)
                    + floor_linear * ground_weight,
            )
        });
    }
    // Elevation alone cannot replace every highland substrate with the same
    // grey rock. Actual exposure and retained ecological cover choose the mix.
    let rock = exposed.max(smooth(1900.0, 4400.0, h) * (1.0 - vegetation) * (1.0 - loose_cover));
    let stone = stone.map(|c| c * (1.0 + outcrop_texture * 0.20 + weathering * 0.12));
    color = mix(color, stone, rock);
    ground_weight *= 1.0 - rock;
    // Gentle concave/depositional ground carries loose weathered material;
    // steep convex outcrops retain the underlying rock. No slope-independent
    // white salt overlay is introduced.
    color = mix(color, soil, loose_cover * (1.0 - vegetation) * 0.55);
    ground_weight *= 1.0 - loose_cover * (1.0 - vegetation) * 0.55;
    // Salt fringe: the outer playa ring (above the 50 m core depth) keeps
    // its plants while the crust washes over it, instead of a hard playa
    // polygon. Inside the core the crust zones from damp margin to dry
    // center, the desiccation gradient real playas show.
    if sample.geology != Geology::Evaporite && sample.salt01 > 0.0 {
        let crust = substrate_albedo(Geology::Evaporite)
            .1
            .map(|c| c * (1.0 + weathering * 0.18));
        color = mix(color, crust, (sample.salt01 / 0.25 * 0.55).clamp(0.0, 1.0));
        ground_weight *= 1.0 - (sample.salt01 / 0.25 * 0.55).clamp(0.0, 1.0);
    }
    if sample.geology == Geology::Evaporite {
        color = mix(
            color,
            [0.80, 0.77, 0.70],
            (sample.salt01 * 0.5).clamp(0.0, 1.0),
        );
        ground_weight *= 1.0 - (sample.salt01 * 0.5).clamp(0.0, 1.0);
        // Damp saline mud mottles the crust in patches instead of one flat
        // fill; real playas are never a uniform polygon.
        let damp_patch = smooth(-0.1, 0.25, variation) * sample.salt01.clamp(0.0, 1.0);
        color = mix(
            color,
            [0.45, 0.47, 0.35],
            (damp_patch * 0.5).clamp(0.0, 1.0),
        );
        ground_weight *= 1.0 - (damp_patch * 0.5).clamp(0.0, 1.0);
    }
    color = color.map(|c| c * (1.0 + variation * 0.14 + regional * 0.08 + grain * 0.24));
    // Bare ground (little vegetation to carry the cover texture) gets its
    // own large-scale tonal structure so deserts and dry plains are not a
    // single flat fill at survey resolution.
    color = color.map(|c| c * (1.0 + cover_texture * 0.10 * (1.0 - vegetation)));
    // A cold dry plateau cannot acquire the same thick cover as a snowy wet
    // mountain. One annual precipitation budget, converted from water to
    // settled snow (300 kg/m³), provides a static availability proxy. The
    // source's 32 m relief amplitude is the cover-depth scale, not a colour
    // multiplier. This is not a dynamic snowpack/seasonal weather solver.
    let snow_depth_m = field.params.surface_climate.precipitation_m_yr
        * sample.moisture01.powi(2)
        * (1.0 - smooth(258.0, 278.0, sample.temperature_k))
        * (1000.0 / 300.0);
    let roughness_depth_m = crate::terrain::micro_amp(32.0, field.params.knobs.into()).max(0.01);
    let snow_available = (snow_depth_m / roughness_depth_m).clamp(0.0, 1.0);
    let snow_cover = (snow * snow_available * (1.0 + hollow * moisture * 0.3)).clamp(0.0, 1.0)
        * (1.0 - exposed * 0.9);
    color = mix(color, [0.88, 0.92, 0.94], snow_cover);
    ground_weight *= 1.0 - snow_cover;
    // Frost on grass: the sub-zero band where snow cover is partial or thin.
    // Below ~258 K snow does the talking; above ~278 K there is nothing to
    // freeze. In between, moisture-gated patches of pale crystals settle over
    // whatever the cover left — the classic -7 C morning. Classification
    // inputs are untouched; only the visual albedo/roughness carry it.
    let frost_band = smooth(278.0, 270.0, sample.temperature_k)
        * (1.0 - smooth(262.0, 254.0, sample.temperature_k));
    let frost = frost_band
        * (0.25 + 0.75 * sample.moisture01)
        * smooth(-0.3, 0.5, micro)
        // White on white is invisible: deep snow needs no frost, and the mix
        // below would only iron out the snow's own micro-relief.
        * (1.0 - snow_cover);
    color = mix(color, [0.78, 0.83, 0.88], (frost * 0.75).clamp(0.0, 1.0));
    ground_weight *= 1.0 - (frost * 0.75).clamp(0.0, 1.0);
    // Micro-relief brightness on the final albedo: overlapping covers (snow,
    // rock) would otherwise mute the pre-mix grain to invisibility. Uses the
    // independent micro signal, so it adds instead of fighting the patches.
    // ±8% stays clear of the sRGB ceiling on bright snow.
    color = color.map(|c| (c * (1.0 + micro * 0.08)).clamp(0.0, 1.0));
    // Existing world-addressed detail follows the selected material state.
    // These are bounded rendering priors, not measured lithology properties;
    // vegetation and snow reduce the exposed substrate response rather than
    // receiving invented leaf/snow geometry.
    let substrate = substrate_surface_profile(sample.geology);
    let normal_detail_scale = if normal_detail_enabled {
        normal_detail_response(
            substrate,
            rock,
            loose_cover,
            vegetation,
            damp,
            snow_cover,
            frost,
        )
    } else {
        0.0
    };
    // A buried substrate cannot change the optical roughness of the cover.
    // Use the same ordered coverage weights as the appearance mix, rather than
    // applying a geology offset to opaque vegetation, snow or frost.
    let visible_substrate = (1.0 - vegetation.clamp(0.0, 1.0))
        * (1.0 - snow_cover)
        * (1.0 - (frost * 0.75).clamp(0.0, 1.0));
    let mut roughness = (land_roughness(vegetation, exposed, damp, outcrop_texture, frost) as f64
        + substrate_roughness_delta(substrate, rock) * visible_substrate)
        .clamp(0.45, 0.98) as f32;
    if let Some(ground) = ground {
        let h_norm = (f64::from(ground.height_m)
            / (crate::ground_detail::HEIGHT_RANGE_M as f64 * 0.5))
            .clamp(-1.0, 1.0);
        let q = dir.map(|v| v * field.params.radius_m);
        let w_meso = band_weight(3.2, texel_m);
        let n_macro = rng::perlin3(field.params.seed, 6141, q[0] / 8.4, q[1] / 8.4, q[2] / 8.4);
        let n_litter = rng::perlin3(field.params.seed, 6142, q[0] / 3.2, q[1] / 3.2, q[2] / 3.2);
        let litter_signal = ((n_macro * 0.40 + n_litter * 0.60) * w_meso).clamp(-1.0, 1.0);
        let water_film = (damp * 0.50 - h_norm * 0.38 - litter_signal * 0.22).clamp(0.0, 1.0);
        let floor_roughness = (ground.roughness - water_film as f32 * 0.68).clamp(0.22, 0.98);
        roughness += (floor_roughness - roughness) * ground_weight as f32;
    }
    (
        SurfaceAppearance {
            albedo_srgb: color.map(|c| c.clamp(0.0, 1.0) as f32),
            // Frost crystals glitter: pull roughness down where they settle.
            roughness,
            normal_detail_scale: normal_detail_scale as f32,
            vegetation: vegetation as f32,
            snow: snow as f32,
            snow_cover: snow_cover as f32,
            frost_cover: (frost * 0.75).clamp(0.0, 1.0) as f32,
        },
        ground_weight,
    )
}

#[cfg(test)]
mod palette_tests {
    use super::*;
    use crate::biomes::Geology;

    #[test]
    fn grain_band_filter_is_continuous_and_zero_beyond_nyquist() {
        for scale in [0.5, 1.0, 2.0, 4.0, 8.0, 32.0, 128.0, 512.0] {
            assert_eq!(band_weight(scale, scale * 0.5), 0.0);
            assert_eq!(band_weight(scale, scale * 0.25), 1.0);
            let a = band_weight(scale, scale * (0.5 - 1e-6));
            assert!(
                a < 1e-9,
                "cutoff must fade instead of changing bands abruptly"
            );
        }
    }

    fn dist(a: [f64; 3], b: [f64; 3]) -> f64 {
        ((a[0] - b[0]).powi(2) + (a[1] - b[1]).powi(2) + (a[2] - b[2]).powi(2)).sqrt()
    }

    #[test]
    fn substrate_palettes_stay_distinguishable() {
        // OceanicCrust shares the basaltic pair (same volcanic material
        // family); every other geology must stand apart in both channels.
        let geologies = [
            Geology::Basaltic,
            Geology::FelsicHighland,
            Geology::ContinentalCrust,
            Geology::Sedimentary,
            Geology::ImpactBreccia,
            Geology::Evaporite,
            Geology::GlacialTill,
            Geology::Regolith,
            Geology::Hydrothermal,
        ];
        for (i, a) in geologies.iter().enumerate() {
            for b in &geologies[i + 1..] {
                let (stone_a, soil_a) = substrate_albedo(*a);
                let (stone_b, soil_b) = substrate_albedo(*b);
                let stone_d = dist(stone_a, stone_b);
                let soil_d = dist(soil_a, soil_b);
                assert!(
                    stone_d >= 0.09,
                    "stones {a:?}/{b:?} collide at {stone_d:.3}"
                );
                assert!(soil_d >= 0.09, "soils {a:?}/{b:?} collide at {soil_d:.3}");
            }
        }
    }

    #[test]
    fn normal_detail_response_is_bounded_and_follows_material_cover() {
        for geology in [
            Geology::Basaltic,
            Geology::FelsicHighland,
            Geology::ContinentalCrust,
            Geology::Sedimentary,
            Geology::ImpactBreccia,
            Geology::Evaporite,
            Geology::GlacialTill,
            Geology::Regolith,
            Geology::Hydrothermal,
        ] {
            let profile = substrate_surface_profile(geology);
            assert!((0.92..=1.08).contains(&profile.exposed_rock_detail));
            assert!((0.92..=1.08).contains(&profile.loose_soil_detail));
            assert!(profile.exposed_roughness_delta.abs() <= 0.012);
            assert!(profile.loose_roughness_delta.abs() <= 0.012);
        }
        let basalt = substrate_surface_profile(Geology::Basaltic);
        let evaporite = substrate_surface_profile(Geology::Evaporite);
        let exposed_basalt = normal_detail_response(basalt, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0);
        let exposed_evaporite = normal_detail_response(evaporite, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0);
        let covered_basalt = normal_detail_response(basalt, 1.0, 0.0, 1.0, 0.0, 0.0, 0.0);
        let snow_basalt = normal_detail_response(basalt, 1.0, 0.0, 0.0, 0.0, 1.0, 0.0);
        let all = [
            exposed_basalt,
            exposed_evaporite,
            covered_basalt,
            snow_basalt,
            normal_detail_response(basalt, 0.0, 0.0, 0.0, 1.0, 0.0, 1.0),
        ];
        assert!(all.into_iter().all(|scale| (0.0..=1.12).contains(&scale)));
        assert!(exposed_basalt > exposed_evaporite);
        assert!(covered_basalt < exposed_basalt);
        assert!(snow_basalt < exposed_basalt * 0.1);
        assert!(substrate_roughness_delta(basalt, 1.0).abs() <= 0.012);
        assert!(substrate_roughness_delta(evaporite, 1.0).abs() <= 0.012);
    }
}

#[cfg(test)]
mod frost_tests {
    use super::*;

    #[test]
    fn resolved_ground_does_not_override_water_rock_or_opaque_snow() {
        let mut field = field();
        field.params.surface_climate.precipitation_m_yr = 1000.0;
        let dir = crate::sphere::dir_from_latlon(38.0, -171.0);
        let ground = crate::ground_detail::GroundSample {
            albedo_linear: [0.2, 0.1, 0.05],
            roughness: 0.7,
            height_m: 0.003,
        };
        for scenario in 0..3 {
            let mut s = sample(285.0, 0.9);
            match scenario {
                0 => s.height_m = -100.0,
                1 => s.slope_hint = 10.0,
                _ => {
                    s.temperature_k = 250.0;
                    s.slope_hint = 0.0;
                }
            }
            let legacy =
                surface_appearance_filtered_with_normal_detail(&field, &s, dir, 0.024, true);
            let (new, weight) =
                surface_appearance_with_ground_detail(&field, &s, dir, 0.024, true, Some(ground));
            assert!(weight < 1e-12, "scenario={scenario} weight={weight}");
            assert_eq!(legacy.albedo_srgb, new.albedo_srgb, "scenario={scenario}");
            assert_eq!(legacy.roughness, new.roughness, "scenario={scenario}");
            assert_eq!(legacy.vegetation, new.vegetation);
            assert_eq!(legacy.snow_cover, new.snow_cover);
        }
    }

    #[test]
    fn saturated_litter_keeps_its_source_detail_and_normal_weight() {
        let field = field();
        let dir = crate::sphere::dir_from_latlon(38.0, -171.0);
        let mut s = sample(285.0, 0.9);
        s.wetland_potential01 = 1.0;
        s.slope_hint = 0.0;
        let mut ground = crate::ground_detail::GroundSample {
            albedo_linear: [0.2, 0.1, 0.05],
            roughness: 0.9,
            height_m: 0.003,
        };
        let (a, wa) =
            surface_appearance_with_ground_detail(&field, &s, dir, 0.024, true, Some(ground));
        ground.albedo_linear = [0.02, 0.01, 0.005];
        let (b, wb) =
            surface_appearance_with_ground_detail(&field, &s, dir, 0.024, true, Some(ground));
        assert!(
            wa > 0.0 && wa == wb,
            "saturation must retain resolved litter: {wa} {wb}"
        );
        assert!(
            a.albedo_srgb
                .into_iter()
                .zip(b.albedo_srgb)
                .all(|(a, b)| a > b)
        );
        assert_eq!(a.roughness, b.roughness);
    }

    #[test]
    fn cold_dry_ground_does_not_receive_unlimited_snow_cover() {
        let field = field();
        for dir in dirs() {
            let dry = sample(268.0, 0.12);
            let wet = sample(268.0, 0.9);
            let a = surface_appearance(&field, &dry, dir);
            let b = surface_appearance(&field, &wet, dir);
            assert!(
                a.snow_cover < b.snow_cover * 0.1,
                "snow must depend on its precipitation supply: dry={} wet={}",
                a.snow_cover,
                b.snow_cover
            );
            assert_eq!(
                a.snow, b.snow,
                "climate suitability remains separate from finite cover"
            );
        }
    }

    #[test]
    fn exposed_substrates_keep_geological_reflectance_instead_of_one_grey_fill() {
        let field = field();
        for dir in dirs() {
            let mut basalt = sample(285.0, 0.0);
            basalt.slope_hint = 1.0;
            basalt.geology = Geology::Basaltic;
            let mut felsic = basalt.clone();
            felsic.geology = Geology::FelsicHighland;
            let a = surface_appearance(&field, &basalt, dir);
            let b = surface_appearance(&field, &felsic, dir);
            assert!(a.albedo_srgb.iter().sum::<f32>() + 0.4 < b.albedo_srgb.iter().sum::<f32>());
            assert_eq!(basalt.height_m, felsic.height_m);
            assert_eq!(a.vegetation, b.vegetation);
            assert!((0.45..=0.98).contains(&a.roughness));
            assert!(a.roughness > b.roughness);
            assert!((a.roughness - b.roughness).abs() <= 0.02);
            assert!(a.normal_detail_scale > b.normal_detail_scale);
            assert!((0.0..=1.12).contains(&a.normal_detail_scale));
        }
    }

    #[test]
    fn disabled_normal_detail_keeps_optical_material_and_returns_zero_scale() {
        let field = field();
        let dir = crate::sphere::dir_from_latlon(38.0, -171.0);
        let mut sample = sample(285.0, 0.0);
        sample.slope_hint = 1.0;
        sample.geology = Geology::Basaltic;
        let enabled = surface_appearance_filtered(&field, &sample, dir, 2.0);
        let disabled =
            surface_appearance_filtered_with_normal_detail(&field, &sample, dir, 2.0, false);
        assert!(enabled.normal_detail_scale > 0.0);
        assert_eq!(disabled.normal_detail_scale, 0.0);
        assert_eq!(enabled.albedo_srgb, disabled.albedo_srgb);
        assert_eq!(enabled.roughness, disabled.roughness);
    }

    #[test]
    fn opaque_cover_hides_substrate_roughness_priors() {
        let mut field = field();
        field.params.surface_climate.precipitation_m_yr = 1000.0;
        let dir = crate::sphere::dir_from_latlon(38.0, -171.0);
        for temperature in [250.0, 285.0] {
            let mut ground = sample(temperature, 1.0);
            ground.slope_hint = 0.0;
            ground.geology = Geology::Basaltic;
            let basalt = surface_appearance(&field, &ground, dir);
            ground.geology = Geology::FelsicHighland;
            let felsic = surface_appearance(&field, &ground, dir);
            assert!(basalt.snow_cover.max(basalt.vegetation) > 0.999);
            assert!(
                (basalt.roughness - felsic.roughness).abs() < 1e-6,
                "buried geology leaked through opaque cover at {temperature} K: {} vs {}",
                basalt.roughness,
                felsic.roughness
            );
        }
    }

    #[test]
    fn intermediate_material_patterns_are_filtered_not_aliased_at_coarse_texels() {
        let field = field();
        let mut sample = sample(285.0, 0.0);
        sample.slope_hint = 0.8;
        let mut fine = Vec::new();
        let mut coarse = Vec::new();
        for i in 0..128 {
            let dir = crate::sphere::dir_from_latlon(23.0, 165.0 + i as f64 * 0.0001);
            fine.push(surface_appearance_filtered(&field, &sample, dir, 2.0).albedo_srgb[0] as f64);
            coarse.push(
                surface_appearance_filtered(&field, &sample, dir, 10000.0).albedo_srgb[0] as f64,
            );
        }
        let spread = |values: &[f64]| {
            values.iter().copied().fold(f64::NEG_INFINITY, f64::max)
                - values.iter().copied().fold(f64::INFINITY, f64::min)
        };
        assert!(
            spread(&fine) > 0.005,
            "resolved rock needs material structure"
        );
        assert!(
            spread(&coarse) < spread(&fine) * 0.1,
            "coarse pixels must not retain unresolved mottling"
        );
    }
    use crate::biomes::{Biome, Geology};

    fn field() -> PlanetField {
        let recipe =
            toml::from_str(include_str!("../../../data/worldgen/worldgen_recipe.toml")).unwrap();
        crate::field::field_from_manifest(&crate::spec_recipe::manifest_from_spec(&recipe).unwrap())
            .unwrap()
    }

    #[test]
    fn coarse_appearance_filters_unresolved_grain_without_changing_semantics() {
        let field = field();
        let dir = crate::sphere::dir_from_latlon(20.0, 40.0);
        let sample = field.sample_surface(dir, 32.0);
        let fine = surface_appearance(&field, &sample, dir);
        let zero = surface_appearance_filtered(&field, &sample, dir, 0.0);
        assert_eq!(fine.albedo_srgb, zero.albedo_srgb);
        assert_eq!(fine.roughness, zero.roughness);
        assert_eq!(surface_grain_filtered(&field, dir, 1000.0), 0.0);
        let before = (
            sample.height_m,
            sample.biome,
            sample.geology,
            sample.temperature_k,
        );
        let coarse = surface_appearance_filtered(&field, &sample, dir, 5000.0);
        assert!(
            coarse
                .albedo_srgb
                .iter()
                .all(|v| v.is_finite() && (0.0..=1.0).contains(v))
        );
        assert_eq!(
            before,
            (
                sample.height_m,
                sample.biome,
                sample.geology,
                sample.temperature_k
            )
        );
    }

    fn sample(temperature_k: f64, moisture01: f64) -> TerrainSample {
        TerrainSample {
            height_m: 500.0,
            macro_height_m: 500.0,
            procedural_height_m: 0.0,
            biome: Biome::RockyPlain,
            geology: Geology::ContinentalCrust,
            tag0: None,
            tag1: None,
            slope_hint: 0.1,
            curvature_per_m: 0.0,
            erosion_displacement_m: 0.0,
            local_deposition_m: 0.0,
            geothermal_flux_w_m2: 0.08,
            moisture01,
            wetland_potential01: 0.0,
            salt01: 0.0,
            temperature_k,
            continentality01: 0.5,
            eclipse_exposure01: 1.0,
        }
    }

    fn dirs() -> Vec<[f64; 3]> {
        // Fixed spread of unit directions: noise is identical across samples
        // at the same dir, so temperature/moisture differences are pure.
        (0..8)
            .map(|i| {
                let a = i as f64 * std::f64::consts::TAU / 8.0;
                [a.cos(), 0.2, a.sin()]
            })
            .map(|d| {
                let l = (d[0] * d[0] + d[1] * d[1] + d[2] * d[2]).sqrt();
                [d[0] / l, d[1] / l, d[2] / l]
            })
            .collect()
    }

    #[test]
    fn geomorphology_exposes_eroded_crests_and_smooths_depositional_hollows() {
        let field = field();
        for dir in dirs() {
            let mut crest = sample(260.0, 0.7);
            crest.curvature_per_m = -0.002;
            crest.erosion_displacement_m = -100.0;
            let mut hollow = crest.clone();
            hollow.curvature_per_m = 0.002;
            hollow.erosion_displacement_m = 100.0;
            let a = surface_appearance(&field, &crest, dir);
            let b = surface_appearance(&field, &hollow, dir);
            let difference = b.albedo_srgb.iter().sum::<f32>() - a.albedo_srgb.iter().sum::<f32>();
            assert!(
                difference > 0.03 && difference < 0.5,
                "subtle accumulation, not a dark contour: {difference}"
            );
            assert_eq!(crest.height_m, hollow.height_m);
            assert_eq!(crest.biome, hollow.biome);
            assert_eq!(crest.temperature_k, hollow.temperature_k);
            assert_eq!(a.snow, b.snow, "climate suitability is not relabelled");
        }
    }

    #[test]
    fn frost_settles_only_in_the_subzero_band_and_leaves_classification() {
        let field = field();
        for dir in dirs() {
            // Warm and deep-cold: no frost, regardless of substrate roughness.
            for temp in [300.0, 240.0] {
                let out = surface_appearance(&field, &sample(temp, 1.0), dir);
                assert_eq!(
                    out.frost_cover, 0.0,
                    "no frost outside the band at {temp} K"
                );
            }
            // Classification inputs untouched by the visual frost.
            assert_eq!(
                surface_appearance(&field, &sample(300.0, 1.0), dir).snow,
                0.0
            );
            assert_eq!(
                surface_appearance(&field, &sample(240.0, 1.0), dir).snow,
                1.0
            );
        }
        // Inspect frost directly: snow availability and substrate roughness
        // also depend on moisture, so neither is an isolated frost signal.
        // Probe at the warm edge of the band (270 K): the frost band is
        // still fully 1 there while the snow edge (~270 K center) has only
        // half closed, so the moisture gate reads through snow suppression.
        let mean = |moisture: f64| {
            dirs()
                .iter()
                .map(|dir| {
                    surface_appearance(&field, &sample(270.0, moisture), *dir).frost_cover as f64
                })
                .sum::<f64>()
                / 8.0
        };
        let (moist, dry) = (mean(1.0), mean(0.0));
        assert!(
            moist > dry + 0.01,
            "moisture must gate frost glitter, moist={moist:.4} dry={dry:.4}"
        );
        // Compare identical substrate/cover inputs, not the former constant
        // baseline: forest can be rougher than exposed stone.
        for vegetation in [0.0, 0.5, 1.0] {
            for exposed in [0.0, 0.5, 1.0] {
                let baseline = land_roughness(vegetation, exposed, 0.7, 0.2, 0.0);
                assert!(land_roughness(vegetation, exposed, 0.7, 0.2, 0.8) <= baseline);
            }
        }
    }
}

#[cfg(test)]
mod shoreline_tests {
    use super::*;
    use crate::biomes::{Biome, Geology};

    fn field() -> PlanetField {
        let recipe =
            toml::from_str(include_str!("../../../data/worldgen/worldgen_recipe.toml")).unwrap();
        crate::field::field_from_manifest(&crate::spec_recipe::manifest_from_spec(&recipe).unwrap())
            .unwrap()
    }

    fn ocean_sample(height_m: f64) -> TerrainSample {
        TerrainSample {
            height_m,
            macro_height_m: height_m,
            procedural_height_m: 0.0,
            biome: Biome::ShallowSea,
            geology: Geology::OceanicCrust,
            tag0: None,
            tag1: None,
            slope_hint: 0.02,
            salt01: 0.0,
            curvature_per_m: 0.0,
            erosion_displacement_m: 0.0,
            local_deposition_m: 0.0,
            geothermal_flux_w_m2: 0.08,
            moisture01: 1.0,
            wetland_potential01: 0.0,
            temperature_k: 290.0,
            continentality01: 0.0,
            eclipse_exposure01: 1.0,
        }
    }

    fn land_sample(height_m: f64) -> TerrainSample {
        TerrainSample {
            height_m,
            macro_height_m: height_m,
            procedural_height_m: 0.0,
            biome: Biome::RockyPlain,
            geology: Geology::ContinentalCrust,
            tag0: None,
            tag1: None,
            slope_hint: 0.05,
            salt01: 0.0,
            curvature_per_m: 0.0,
            erosion_displacement_m: 0.0,
            local_deposition_m: 0.0,
            geothermal_flux_w_m2: 0.08,
            moisture01: 0.5,
            wetland_potential01: 0.0,
            temperature_k: 285.0,
            continentality01: 0.0,
            eclipse_exposure01: 1.0,
        }
    }

    fn dirs(n: usize) -> Vec<[f64; 3]> {
        (0..n)
            .map(|i| {
                let a = i as f64 * std::f64::consts::TAU / n as f64;
                [a.cos(), 0.15, a.sin()]
            })
            .map(|d| {
                let l = (d[0] * d[0] + d[1] * d[1] + d[2] * d[2]).sqrt();
                [d[0] / l, d[1] / l, d[2] / l]
            })
            .collect()
    }

    #[test]
    fn surf_foam_whitens_the_waterline_and_kills_the_glint() {
        let field = field();
        // At least one shoreline texel must foam: white + matte.
        let mut foamed = 0;
        for dir in dirs(32) {
            let out = surface_appearance(&field, &ocean_sample(-1.5), dir);
            assert_eq!(
                out.normal_detail_scale, 0.0,
                "water has no material normal grain"
            );
            let lum = (out.albedo_srgb[0] + out.albedo_srgb[1] + out.albedo_srgb[2]) / 3.0;
            if lum > 0.55 && out.roughness > 0.35 {
                foamed += 1;
            }
            // Foam never darkens below open water.
            assert!(out.albedo_srgb[0] >= 0.0 && out.albedo_srgb[0] <= 1.0);
        }
        assert!(foamed > 0, "surf foam must fire near h=-1.5 m");
        // Deep water stays dark and glossy (sun glint preserved).
        let mut deep_lum = 0.0;
        let mut deep_rough = 0.0;
        for dir in dirs(16) {
            let out = surface_appearance(&field, &ocean_sample(-3000.0), dir);
            deep_lum += (out.albedo_srgb[0] + out.albedo_srgb[1] + out.albedo_srgb[2]) as f64 / 3.0;
            deep_rough += out.roughness as f64;
        }
        deep_lum /= 16.0;
        deep_rough /= 16.0;
        assert!(deep_lum < 0.25, "deep ocean stays dark, got {deep_lum:.3}");
        assert!(
            deep_rough < 0.25,
            "deep ocean stays glossy, got {deep_rough:.3}"
        );
    }

    #[test]
    fn shallow_shelf_lifts_turquoise_without_becoming_sky() {
        let field = field();
        let mut shallow_g = 0.0;
        let mut deep_g = 0.0;
        for dir in dirs(16) {
            shallow_g +=
                surface_appearance(&field, &ocean_sample(-20.0), dir).albedo_srgb[1] as f64;
            deep_g += surface_appearance(&field, &ocean_sample(-3000.0), dir).albedo_srgb[1] as f64;
        }
        shallow_g /= 16.0;
        deep_g /= 16.0;
        assert!(
            shallow_g > deep_g + 0.03,
            "shelf must lift green, shallow={shallow_g:.3} deep={deep_g:.3}"
        );
        assert!(shallow_g < 0.65, "shelf stays water, got {shallow_g:.3}");
    }

    #[test]
    fn wet_sand_rims_the_beach() {
        let field = field();
        // Both inside the beach band (smooth(3,45,h)=0 there), so the only
        // difference is the wet rim fading by ~3 m.
        let mut wet = 0.0;
        let mut dry = 0.0;
        for dir in dirs(16) {
            let w = surface_appearance(&field, &land_sample(1.0), dir);
            let d = surface_appearance(&field, &land_sample(3.0), dir);
            wet += (w.albedo_srgb[0] + w.albedo_srgb[1] + w.albedo_srgb[2]) as f64 / 3.0;
            dry += (d.albedo_srgb[0] + d.albedo_srgb[1] + d.albedo_srgb[2]) as f64 / 3.0;
        }
        wet /= 16.0;
        dry /= 16.0;
        assert!(
            wet < dry,
            "waterline sand darker, wet={wet:.3} dry={dry:.3}"
        );
    }

    #[test]
    fn inland_datum_crossing_does_not_paint_a_beach_or_wet_rim() {
        let field = field();
        for geology in [
            Geology::ContinentalCrust,
            Geology::Basaltic,
            Geology::Evaporite,
        ] {
            for dir in dirs(8) {
                let mut low = land_sample(1.0);
                low.continentality01 = 1.0;
                low.geology = geology;
                let mut higher = low.clone();
                higher.height_m = 3.0;
                higher.macro_height_m = 3.0;
                let a = surface_appearance(&field, &low, dir);
                let b = surface_appearance(&field, &higher, dir);
                assert_eq!(
                    a.albedo_srgb, b.albedo_srgb,
                    "inland soil acquired datum-based beach/wet paint: {geology:?}"
                );
            }
        }
    }

    #[test]
    fn low_coastal_cliff_retains_exposed_geology_instead_of_sand_paint() {
        let field = field();
        let dir = dirs(1)[0];
        let mut coast = land_sample(1.0);
        coast.continentality01 = 0.0;
        coast.moisture01 = 0.0;
        coast.slope_hint = 0.8;
        coast.geology = Geology::Basaltic;
        let basalt = surface_appearance(&field, &coast, dir);
        coast.geology = Geology::FelsicHighland;
        let felsic = surface_appearance(&field, &coast, dir);
        let separation = basalt
            .albedo_srgb
            .iter()
            .zip(felsic.albedo_srgb)
            .map(|(a, b)| (a - b).abs())
            .sum::<f32>();
        assert!(
            separation > 0.2,
            "low cliff geology was hidden by sand: {basalt:?} {felsic:?}"
        );
    }

    #[test]
    fn coastal_substrate_eligibility_rejects_inland_exposure_salt_and_steep_ground() {
        let mut sample = land_sample(1.0);
        let eligible = beach_material_coverage(1.0, &sample, 0.0);
        assert!(
            eligible > 0.99,
            "bare gentle maritime surface can carry fines"
        );
        for (maritime, exposed) in [(0.0, 0.0), (1.0, 1.0)] {
            assert_eq!(beach_material_coverage(maritime, &sample, exposed), 0.0);
        }
        sample.geology = Geology::Evaporite;
        assert_eq!(beach_material_coverage(1.0, &sample, 0.0), 0.0);
        sample.geology = Geology::Sedimentary;
        sample.salt01 = 0.25;
        assert_eq!(beach_material_coverage(1.0, &sample, 0.0), 0.0);
        sample.salt01 = 0.0;
        sample.slope_hint = 0.35;
        assert_eq!(beach_material_coverage(1.0, &sample, 0.0), 0.0);
    }

    #[test]
    fn maritime_and_height_eligibility_fade_endpoints_are_continuous() {
        let field = field();
        let spacing = field.ocean_context_spacing_m();
        assert!((spacing - field.params.radius_m * 2.0_f64.to_radians()).abs() < 1e-8);
        let mut sample = land_sample(1.0);
        for ratio in [1.0, 2.0] {
            sample.continentality01 =
                spacing * (ratio - 1e-6) / crate::climate::CONTINENTALITY_DISTANCE_M;
            let a = maritime_material_support(&field, &sample);
            sample.continentality01 =
                spacing * (ratio + 1e-6) / crate::climate::CONTINENTALITY_DISTANCE_M;
            let b = maritime_material_support(&field, &sample);
            assert!(
                (a - b).abs() < 1e-8,
                "macro coast context must not become a cell edge"
            );
        }
        for h in [3.0, 45.0] {
            sample.height_m = h - 1e-6;
            let a = beach_material_coverage(1.0, &sample, 0.0);
            sample.height_m = h + 1e-6;
            let b = beach_material_coverage(1.0, &sample, 0.0);
            assert!((a - b).abs() < 1e-8);
        }
        // Eligibility has no texel/page/camera input. Appearance filtering
        // still uses the existing common physical-footprint path.
        sample.continentality01 = 1.0;
        assert_eq!(maritime_material_support(&field, &sample), 0.0);
    }

    #[test]
    fn fractional_cover_attenuates_only_the_coastal_substrate_difference() {
        let field = field();
        let dir = dirs(1)[0];
        let difference = |sample: &TerrainSample| {
            let mut coast = sample.clone();
            coast.continentality01 = 0.0;
            let mut inland = sample.clone();
            inland.continentality01 = 1.0;
            // Filter out the final vegetation-dependent cover-texture gain
            // to isolate cover opacity rather than a second optical effect.
            let a = surface_appearance_filtered(&field, &coast, dir, 5000.0);
            let b = surface_appearance_filtered(&field, &inland, dir, 5000.0);
            std::array::from_fn::<_, 3, _>(|i| f64::from(a.albedo_srgb[i] - b.albedo_srgb[i]))
        };
        let mut sample = land_sample(1.0);
        sample.slope_hint = 0.0;
        sample.moisture01 = 0.4;
        sample.temperature_k = 335.0;
        let bare = difference(&sample);
        assert!(bare.iter().any(|v| v.abs() > 0.01));
        for (temperature, wetland) in [(325.0, 0.0), (315.0, 0.0), (325.0, 0.3), (315.0, 1.0)] {
            sample.temperature_k = temperature;
            sample.wetland_potential01 = wetland;
            let vegetation = crate::ecology::vegetation_cover(&sample, 0.0).total();
            let actual = difference(&sample);
            for i in 0..3 {
                let expected = bare[i] * (1.0 - vegetation) * (1.0 - wetland);
                assert!(
                    (actual[i] - expected).abs() < 1e-6,
                    "coast must not repaint partial cover: vegetation={vegetation} wetland={wetland} actual={} expected={expected}",
                    actual[i]
                );
            }
        }
    }
}

#[cfg(test)]
mod lod_fade_tests {
    use super::*;
    use crate::biomes::{Biome, Geology};

    fn field() -> PlanetField {
        let recipe =
            toml::from_str(include_str!("../../../data/worldgen/worldgen_recipe.toml")).unwrap();
        crate::field::field_from_manifest(&crate::spec_recipe::manifest_from_spec(&recipe).unwrap())
            .unwrap()
    }

    fn warm_land(height_m: f64) -> TerrainSample {
        TerrainSample {
            height_m,
            macro_height_m: height_m,
            procedural_height_m: 0.0,
            biome: Biome::RockyPlain,
            geology: Geology::ContinentalCrust,
            tag0: None,
            tag1: None,
            slope_hint: 0.05,
            salt01: 0.0,
            curvature_per_m: 0.0,
            erosion_displacement_m: 0.0,
            local_deposition_m: 0.0,
            geothermal_flux_w_m2: 0.08,
            moisture01: 0.5,
            wetland_potential01: 0.0,
            temperature_k: 290.0,
            continentality01: 0.5,
            eclipse_exposure01: 1.0,
        }
    }

    fn dirs(n: usize) -> Vec<[f64; 3]> {
        (0..n)
            .map(|i| {
                let a = i as f64 * std::f64::consts::TAU / n as f64;
                [a.cos(), 0.15, a.sin()]
            })
            .map(|d| {
                let l = (d[0] * d[0] + d[1] * d[1] + d[2] * d[2]).sqrt();
                [d[0] / l, d[1] / l, d[2] / l]
            })
            .collect()
    }

    #[test]
    fn coarse_texels_lose_grain_but_keep_large_scale_structure() {
        let field = field();
        // Adjacent directions ~200 m apart: inside one tile's texel, where
        // aliasing used to flip the per-tile mean (the 510 km quilt).
        let pairs: Vec<([f64; 3], [f64; 3])> = (0..16)
            .map(|i| {
                let a = i as f64 * std::f64::consts::TAU / 16.0;
                let norm = |d: [f64; 3]| {
                    let l = (d[0] * d[0] + d[1] * d[1] + d[2] * d[2]).sqrt();
                    [d[0] / l, d[1] / l, d[2] / l]
                };
                // 200 m at 3200 km radius ~= 6.25e-5 rad.
                let e = 6.25e-5;
                (
                    norm([a.cos(), 0.15, a.sin()]),
                    norm([(a + e).cos(), 0.15, (a + e).sin()]),
                )
            })
            .collect();
        let lum = |o: SurfaceAppearance| {
            (o.albedo_srgb[0] + o.albedo_srgb[1] + o.albedo_srgb[2]) as f64 / 3.0
        };
        let mean_gap = |texel: f64| {
            pairs
                .iter()
                .map(|(a, b)| {
                    (lum(surface_appearance_filtered(
                        &field,
                        &warm_land(500.0),
                        *a,
                        texel,
                    )) - lum(surface_appearance_filtered(
                        &field,
                        &warm_land(500.0),
                        *b,
                        texel,
                    )))
                    .abs()
                })
                .sum::<f64>()
                / pairs.len() as f64
        };
        let full = mean_gap(0.0);
        let coarse = mean_gap(5000.0);
        assert!(full > 1e-3, "full detail must vary sub-texel, {full:.5}");
        assert!(
            coarse < full * 0.2,
            "coarse texel must agree locally, full={full:.5} coarse={coarse:.5}"
        );
        // Fine texels keep full detail exactly.
        for dir in dirs(8) {
            let a = surface_appearance(&field, &warm_land(500.0), dir);
            let b = surface_appearance_filtered(&field, &warm_land(500.0), dir, 1.0);
            assert_eq!(a.albedo_srgb, b.albedo_srgb);
            assert_eq!(a.roughness, b.roughness);
        }
    }

    #[test]
    fn vegetation_breaks_into_patches_but_stays_smooth_when_coarse() {
        let field = field();
        // Moist warm plain: vegetation gate open, only the patch factor
        // differs per direction. Patchiness rides the albedo (cover/grain
        // texture over the sample-local vegetation gate), so measure the
        // luminance spread, not the gate value itself.
        let lum = |o: SurfaceAppearance| {
            (o.albedo_srgb[0] + o.albedo_srgb[1] + o.albedo_srgb[2]) as f64 / 3.0
        };
        let green_gap = |texel: f64| {
            dirs(16)
                .iter()
                .map(|dir| {
                    let mut s = warm_land(500.0);
                    s.moisture01 = 0.65;
                    s.temperature_k = 285.0;
                    lum(surface_appearance_filtered(&field, &s, *dir, texel))
                })
                .collect::<Vec<_>>()
        };
        let spread = |vals: &[f64]| {
            let mean = vals.iter().sum::<f64>() / vals.len() as f64;
            vals.iter().map(|v| (v - mean).abs()).sum::<f64>() / vals.len() as f64
        };
        let full = spread(&green_gap(0.0));
        let coarse = spread(&green_gap(5000.0));
        // Absolute scale is albedo luminance here (measured 0.016 on the
        // deterministic test field), not the old vegetation-gate units.
        assert!(full > 0.008, "patches must vary cover, {full:.3}");
        assert!(
            coarse < full * 0.5,
            "coarse cover must even out, full={full:.3} coarse={coarse:.3}"
        );
    }
}
