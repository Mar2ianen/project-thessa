//! World-addressed rock and plant descriptors, without a renderer dependency.
//!
//! A tile is a query window, never a random seed. Version 1 candidates occupy
//! fixed L20 cube cells; near understory and pebbles use disjoint L24 tags.
//! Density only thins these immutable populations.
//! Positions are body-fixed unit directions plus final ground height in metres.
//! See `docs/70_WORLD_SCATTER_IDENTITY.md` for limits and numerical contracts.

use crate::{
    biomes::{Biome, Geology},
    ecology::VegetationCover,
    lod::{TileKey, cbt_node_for_tile},
    rng,
};

/// Part of the version-1 address/placement contract, not a render LOD setting.
pub const CANDIDATE_LEVEL: u8 = 20;
/// Separate identity lattice for near-ground fern/reed clumps. Its CBT depth
/// keeps these IDs disjoint from unchanged L20 rocks/plants. Its even tag
/// also addresses the separate near-ground pebble population.
pub const UNDERSTORY_CANDIDATE_LEVEL: u8 = 24;
/// Authored colony occupancy prior, not simulated moisture or plant growth.
pub const UNDERSTORY_PATCH_WAVELENGTH_M: f64 = 8.0;
const UNDERSTORY_PATCH_MAX: f64 = 1.85;
const UNDERSTORY_MOISTURE_DENSITY_MAX: f64 = 1.35;
const JITTER_BITS: u8 = 24;
const MAX_ROCK_DENSITY_PER_KM2: f64 = 640.0;
pub const PLANT_DENSITY_PER_KM2: f64 = 3000.0;

/// Fixed canonical ground/placement cutoff and geometry scale, in metres.
/// This is a plant placement contract, not a query/render LOD or erosion grid.
pub const PLANT_SURFACE_WAVELENGTH_M: f64 = 32.0;

/// Plant-only callback adapter for the caller's existing canonical field,
/// including its attached global/regional erosion and retained analytic bands.
/// Borrow the field from `TerrainStream::field` / `canonical_world_field`;
/// this function neither loads nor caches another world.
///
/// Ground and placement slope use the fixed 32 m contract (a centred +/-16 m
/// final-height stencil). Ecological cover uses the same fixed 256 m geometry
/// as material pages, not the zero slope from `sample_surface` or a tile LOD.
/// Water retains the coarse basin-cell descriptor limitations of
/// `PlanetField::water_height_m`. No rock-density noise is invented here.
/// `dir` must be a finite body-fixed unit direction; the scatter kernel
/// guarantees this precondition for its surface callback.
pub fn canonical_plant_surface(field: &crate::field::PlanetField, dir: [f64; 3]) -> ScatterSurface {
    let sample = field.sample_surface(dir, PLANT_SURFACE_WAVELENGTH_M);
    let slope = field.surface_geometry(dir, PLANT_SURFACE_WAVELENGTH_M).0;
    let material_slope = field
        .surface_geometry(dir, crate::lod::MATERIAL_SLOPE_WAVELENGTH_M)
        .0;
    ScatterSurface::plants(
        sample.height_m,
        slope,
        Some(field.water_height_m(dir)),
        crate::ecology::vegetation_cover(&sample, material_slope),
    )
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, serde::Serialize, serde::Deserialize)]
pub enum ScatterKind {
    SmallRock,
    Boulder,
    LargeBoulder,
    Outcrop,
    TalusField,
    VolcanicBlock,
    IceBlock,
    FernClump,
    ReedClump,
    LycopsidTree,
    Pebble,
}

/// Collision-free fixed64 address *within a world*. Persist the world identity
/// alongside it. Zero is never generated.
#[derive(
    Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash, serde::Serialize, serde::Deserialize,
)]
#[serde(transparent)]
pub struct ScatterId(pub u64);

impl ScatterId {
    /// Decode/validate the canonical cell, reusing the existing CBT address map.
    pub fn cell(self) -> Option<TileKey> {
        let id = self.0 >> 1;
        let depth = u8::try_from(63_u32.checked_sub(id.leading_zeros())?).ok()?;
        if depth != crate::lod::CBT_FACE_DEPTH + 2 * CANDIDATE_LEVEL
            && depth != crate::lod::CBT_FACE_DEPTH + 2 * UNDERSTORY_CANDIDATE_LEVEL
        {
            return None;
        }
        let node = thessa_rcbt_core::Node::new(id, depth).ok()?;
        crate::lod::tile_for_cbt_node(node)
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ScatterPopulation {
    Rocks = 0,
    Plants = 1,
    Understory = 2,
    GroundPebbles = 3,
}

/// One world-space instance. Yaw is about radial up, relative to the canonical
/// `sphere::enu_basis(direction)` east/north frame; scale is dimensionless.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ScatterItem {
    pub id: ScatterId,
    pub kind: ScatterKind,
    pub direction: [f64; 3],
    pub ground_height_m: f64,
    pub scale: f64,
    pub yaw_rad: f64,
}

/// Local final-surface inputs, evaluated at the candidate, never at tile centre.
/// Density is per datum-sphere square kilometre, not per relief-surface area.
#[derive(Debug, Clone, Copy)]
pub struct ScatterSurface {
    pub ground_height_m: f64,
    /// Rise/run, not normalized slope or radians; sampled at a fixed world scale.
    pub slope_rise_run: f64,
    /// Local water elevation in metres above the same datum; None means dry.
    /// Ground at or below this surface rejects both populations.
    pub water_height_m: Option<f64>,
    /// Rock classification only; ignored for the plant population.
    pub biome: Biome,
    /// Rock classification only; ignored for the plant population.
    pub geology: Geology,
    /// Rock density multiplier input only, constrained to [0, 1].
    pub rocky01: f64,
    /// Plant population only: fractional ecological cover for clumps/crowns,
    /// not individual grass blades.
    pub vegetation: VegetationCover,
}

impl ScatterSurface {
    /// Plant-only input without evaluating unused rock classification fields.
    pub fn plants(
        ground_height_m: f64,
        slope_rise_run: f64,
        water_height_m: Option<f64>,
        vegetation: VegetationCover,
    ) -> Self {
        Self {
            ground_height_m,
            slope_rise_run,
            water_height_m,
            vegetation,
            biome: Biome::RockyPlain,
            geology: Geology::Regolith,
            rocky01: 0.0,
        }
    }
}

/// Quality thins candidates; budgets reject whole queries, never truncate them.
#[derive(Debug, Clone, Copy, serde::Serialize, serde::Deserialize)]
#[serde(default)]
pub struct ScatterPolicy {
    pub density_scale: f64,
    pub max_candidates: u64,
    pub max_items: usize,
    pub min_plant_height_m: f64,
    pub max_plant_slope_rise_run: f64,
    /// Reference centre spacing for a fully eligible fern/reed habitat, metres.
    /// An authored plant-density prior, not a calibrated canopy-coverage law.
    pub understory_spacing_m: f64,
    /// Authored dimensionless size range for near-ground clumps. The existing
    /// 0.5..2.3 range remains for the larger plant survey; understory uses
    /// shorter geometry so it reads as ground cover rather than a wall.
    pub understory_min_scale: f64,
    pub understory_max_scale: f64,
}

impl Default for ScatterPolicy {
    fn default() -> Self {
        Self {
            density_scale: 1.0,
            max_candidates: 1 << 24,
            max_items: 1_024_000,
            min_plant_height_m: 2.0,
            max_plant_slope_rise_run: 0.25,
            understory_spacing_m: 0.95,
            understory_min_scale: 0.25,
            understory_max_scale: 1.10,
        }
    }
}

impl ScatterPolicy {
    pub fn validate(self) -> Result<(), ScatterError> {
        if !self.density_scale.is_finite()
            || self.density_scale < 0.0
            || !self.min_plant_height_m.is_finite()
            || !self.max_plant_slope_rise_run.is_finite()
            || self.max_plant_slope_rise_run < 0.0
            || !self.understory_spacing_m.is_finite()
            || self.understory_spacing_m <= 0.0
            || !self.understory_min_scale.is_finite()
            || !self.understory_max_scale.is_finite()
            || self.understory_min_scale <= 0.0
            || self.understory_max_scale < self.understory_min_scale
        {
            return Err(ScatterError::InvalidRequest("invalid scatter policy"));
        }
        Ok(())
    }
}

#[derive(Debug, Clone, PartialEq)]
pub enum ScatterError {
    InvalidRequest(&'static str),
    InvalidSurface(ScatterId),
    CandidateBudgetExceeded {
        required: u64,
        limit: u64,
    },
    ItemBudgetExceeded {
        limit: usize,
    },
    AllocationFailed {
        additional: usize,
    },
    DensityCapacityExceeded {
        requested_per_km2: f64,
        capacity_per_km2: f64,
    },
}

impl std::fmt::Display for ScatterError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "scatter: {self:?}")
    }
}
impl std::error::Error for ScatterError {}

#[derive(Debug, Clone, PartialEq)]
pub struct ScatterBatch {
    /// Strictly increasing IDs. Union/deduplicate by ID for overlapping windows.
    pub items: Vec<ScatterItem>,
    pub candidates_visited: u64,
    pub surface_samples: u64,
}

/// Enumerate a bounded window of the immutable world population. The callback
/// must be a pure final-surface sampler independent of region and request order.
/// It runs only after conservative density rejection. Errors return no partial
/// batch. Valid TileKey levels are 0..=27, matching the existing CBT adapter.
pub fn scatter_region(
    planet_seed: u64,
    radius_m: f64,
    region: TileKey,
    population: ScatterPopulation,
    policy: ScatterPolicy,
    mut surface_at: impl FnMut([f64; 3]) -> ScatterSurface,
) -> Result<ScatterBatch, ScatterError> {
    policy.validate()?;
    if !radius_m.is_finite() || radius_m <= 0.0 || !(radius_m * radius_m).is_finite() {
        return Err(ScatterError::InvalidRequest(
            "radius must be finite positive metres",
        ));
    }
    if region.face >= 6
        || region.level > 27
        || region.x >= 1_u32 << region.level
        || region.y >= 1_u32 << region.level
    {
        return Err(ScatterError::InvalidRequest("invalid cube-sphere region"));
    }
    let mut batch = ScatterBatch {
        items: Vec::new(),
        candidates_visited: 0,
        surface_samples: 0,
    };
    if policy.density_scale == 0.0 {
        return Ok(batch);
    }
    let candidate_level = if matches!(
        population,
        ScatterPopulation::Understory | ScatterPopulation::GroundPebbles
    ) {
        UNDERSTORY_CANDIDATE_LEVEL
    } else {
        CANDIDATE_LEVEL
    };
    let reference_density = match population {
        ScatterPopulation::Rocks => MAX_ROCK_DENSITY_PER_KM2,
        ScatterPopulation::Plants => PLANT_DENSITY_PER_KM2,
        ScatterPopulation::GroundPebbles => 2_000_000.0,
        ScatterPopulation::Understory => 1e6 / policy.understory_spacing_m.powi(2),
    };
    let habitat_ceiling = if population == ScatterPopulation::Understory {
        UNDERSTORY_MOISTURE_DENSITY_MAX
    } else {
        1.0
    };
    let ceiling = reference_density
        * policy.density_scale
        * habitat_ceiling
        * if matches!(
            population,
            ScatterPopulation::Understory | ScatterPopulation::GroundPebbles
        ) {
            UNDERSTORY_PATCH_MAX
        } else {
            1.0
        };
    // Largest cell Jacobian is at the face centre. Refuse saturation even if
    // local cover later rejects everything: density must never silently clip.
    let max_cell_area_km2 = (2.0 * radius_m / (1_u64 << candidate_level) as f64).powi(2) / 1e6;
    let capacity = 1.0 / max_cell_area_km2;
    if !capacity.is_finite() {
        return Err(ScatterError::InvalidRequest(
            "radius is below numeric scatter range",
        ));
    }
    if !ceiling.is_finite() || ceiling > capacity {
        return Err(ScatterError::DensityCapacityExceeded {
            requested_per_km2: ceiling,
            capacity_per_km2: capacity,
        });
    }
    let bounds = |index: u32| {
        if region.level <= candidate_level {
            let shift = candidate_level - region.level;
            (index << shift, (index + 1) << shift)
        } else {
            let cell = index >> (region.level - candidate_level);
            (cell, cell + 1)
        }
    };
    let (x0, x1) = bounds(region.x);
    let (y0, y1) = bounds(region.y);
    let required = u64::from(x1 - x0) * u64::from(y1 - y0);
    if required > policy.max_candidates {
        return Err(ScatterError::CandidateBudgetExceeded {
            required,
            limit: policy.max_candidates,
        });
    }
    batch.candidates_visited = required;
    for y in y0..y1 {
        for x in x0..x1 {
            let cell = TileKey {
                face: region.face,
                level: candidate_level,
                x,
                y,
            };
            let tag = u64::from(matches!(
                population,
                ScatterPopulation::Plants | ScatterPopulation::Understory
            ));
            let id = ScatterId((cbt_node_for_tile(cell).unwrap().id() << 1) | tag);
            let random = |channel| rng::hash01(planet_seed, channel, id.0 as i64, 0);
            // Midpoints of a 24-bit subcell grid: strictly interior, even when
            // hash01 rounds to 1.0. Dyadic ownership is exact through level 27.
            let jitter = |channel| {
                let q = ((random(channel) * (1_u64 << JITTER_BITS) as f64) as u32)
                    .min((1 << JITTER_BITS) - 1);
                (q as f64 + 0.5) / (1_u64 << JITTER_BITS) as f64
            };
            let (jx, jy) = (jitter(402), jitter(403));
            if region.level > candidate_level {
                let n = (1_u64 << candidate_level) as f64;
                let rn = (1_u64 << region.level) as f64;
                if (((x as f64 + jx) / n * rn) as u32 != region.x)
                    || (((y as f64 + jy) / n * rn) as u32 != region.y)
                {
                    continue;
                }
            }
            let n = (1_u64 << candidate_level) as f64;
            let u = 2.0 * (x as f64 + jx) / n - 1.0;
            let v = 2.0 * (y as f64 + jy) / n - 1.0;
            let area_km2 = max_cell_area_km2 / (1.0 + u * u + v * v).powf(1.5);
            let keep = random(401);
            if keep >= ceiling * area_km2 {
                continue;
            }
            let direction = cell.direction(jx, jy);
            let patch_density = if matches!(
                population,
                ScatterPopulation::Understory | ScatterPopulation::GroundPebbles
            ) {
                let patch_seed = if population == ScatterPopulation::GroundPebbles {
                    planet_seed ^ 0x7065_6262_6c65
                } else {
                    planet_seed
                };
                let density = understory_patch_density(patch_seed, radius_m, direction);
                // Reject colony gaps before the expensive canonical surface query.
                if keep
                    >= reference_density
                        * policy.density_scale
                        * density
                        * habitat_ceiling
                        * area_km2
                {
                    continue;
                }
                density
            } else {
                1.0
            };
            batch.surface_samples += 1;
            let mut surface = surface_at(direction);
            validate_surface(surface, id, population)?;
            if population != ScatterPopulation::Rocks && surface.vegetation.total() > 1.0 {
                let total = surface.vegetation.total();
                surface.vegetation.ground01 /= total;
                surface.vegetation.canopy01 /= total;
                surface.vegetation.reeds01 /= total;
            }
            if surface
                .water_height_m
                .is_some_and(|h| surface.ground_height_m <= h)
            {
                continue;
            }
            let (moisture_factor, reed_bias) = if population == ScatterPopulation::Understory {
                understory_habitat_weights(surface)
            } else {
                (1.0, 1.0)
            };
            let density = match population {
                ScatterPopulation::Rocks => {
                    base_density_per_km2(surface.biome) * (1.0 + 3.0 * surface.rocky01)
                }
                ScatterPopulation::GroundPebbles => {
                    if surface.ground_height_m < 0.0
                        || surface.slope_rise_run > policy.max_plant_slope_rise_run
                    {
                        continue;
                    }
                    reference_density * surface.vegetation.total() * patch_density
                }
                ScatterPopulation::Plants | ScatterPopulation::Understory => {
                    if surface.ground_height_m < policy.min_plant_height_m
                        || surface.slope_rise_run > policy.max_plant_slope_rise_run
                    {
                        continue;
                    }
                    let canopy_cover = surface.vegetation.canopy01.clamp(0.0, 1.0);
                    // Dappled light niche: sciophytes flourish under canopy shelter and in
                    // forest glades, avoiding bare exposed soil and lightless overgrowth.
                    let light_niche = (1.0 + 0.40 * canopy_cover
                        - 0.50 * canopy_cover * canopy_cover)
                        .clamp(0.40, 1.15);
                    reference_density
                        * if population == ScatterPopulation::Understory {
                            (surface.vegetation.ground01 + surface.vegetation.reeds01)
                                * patch_density
                                * light_niche
                                * moisture_factor
                        } else {
                            surface.vegetation.total()
                        }
                }
            } * policy.density_scale;
            if keep >= density * area_km2 {
                continue;
            }
            let kind = match population {
                ScatterPopulation::Rocks => pick_rock_kind(planet_seed, id, surface),
                ScatterPopulation::GroundPebbles => ScatterKind::Pebble,
                ScatterPopulation::Plants => {
                    let choice = random(431) * surface.vegetation.total();
                    if choice < surface.vegetation.reeds01 {
                        ScatterKind::ReedClump
                    } else if choice < surface.vegetation.reeds01 + surface.vegetation.canopy01 {
                        ScatterKind::LycopsidTree
                    } else {
                        ScatterKind::FernClump
                    }
                }
                ScatterPopulation::Understory => {
                    let reed_weight = surface.vegetation.reeds01 * reed_bias;
                    let ground_weight = surface.vegetation.ground01;
                    let choice = random(431) * (ground_weight + reed_weight);
                    if choice < reed_weight {
                        ScatterKind::ReedClump
                    } else {
                        ScatterKind::FernClump
                    }
                }
            };
            if batch.items.len() == policy.max_items {
                return Err(ScatterError::ItemBudgetExceeded {
                    limit: policy.max_items,
                });
            }
            if batch.items.len() == batch.items.capacity() {
                let additional =
                    (policy.max_items - batch.items.len()).min(batch.items.capacity().max(1024));
                try_reserve_items(&mut batch.items, additional)?;
            }
            let scale = if population == ScatterPopulation::GroundPebbles {
                // Unit-diameter visual pebble: many small grains, rare cobbles.
                0.04 + random(404).powi(4) * 0.42
            } else if population == ScatterPopulation::Understory {
                let p = direction.map(|v| v * (radius_m / 24.0));
                let community = rng::value_noise3(planet_seed ^ 0x636f6d6d, 8192, p[0], p[1], p[2]);
                // Authored spatial cohorts bias the size distribution within
                // the caller's interval; a fixed scale must remain fixed.
                let size_fraction = (0.70 * random(404)
                    + 0.30 * crate::appearance::smooth(-0.40, 0.40, community))
                .powf(0.90);
                policy.understory_min_scale
                    + size_fraction * (policy.understory_max_scale - policy.understory_min_scale)
            } else {
                0.5 + random(404) * 1.8
            };
            batch.items.push(ScatterItem {
                id,
                kind,
                direction,
                ground_height_m: surface.ground_height_m,
                scale,
                yaw_rad: random(405) * std::f64::consts::TAU,
            });
        }
    }
    batch.items.sort_unstable_by_key(|item| item.id);
    Ok(batch)
}

/// Authored relative-water niche, not a soil-water or plant growth solver.
fn understory_habitat_weights(surface: ScatterSurface) -> (f64, f64) {
    let Some(water_height) = surface.water_height_m else {
        return (1.0, 1.0);
    };
    let elevation = (surface.ground_height_m - water_height).max(0.0);
    let moist = 1.0 - crate::appearance::smooth(2.5, 8.0, elevation);
    let riparian = 1.0 - crate::appearance::smooth(2.2, 8.0, elevation);
    (
        0.88 + (UNDERSTORY_MOISTURE_DENSITY_MAX - 0.88) * moist,
        0.45 + 1.30 * riparian,
    )
}

/// Shared by every query window and quality tier; no cube-face seed or camera.
/// Bounded pioneer floor and dense colonies create natural clearings and dense glades.
fn understory_patch_density(seed: u64, radius_m: f64, direction: [f64; 3]) -> f64 {
    let p = direction.map(|v| v * (radius_m / UNDERSTORY_PATCH_WAVELENGTH_M));
    let noise = rng::value_noise3(seed, 437, p[0], p[1], p[2]);
    let occupancy = 0.22 + 0.78 * crate::appearance::smooth(-0.16, 0.25, noise);
    UNDERSTORY_PATCH_MAX * occupancy
}

/// Fallible growth for the large descriptor/instance buffers, not a guarantee
/// against allocations performed by callbacks or unrelated library code.
pub(crate) fn try_reserve_items<T>(
    items: &mut Vec<T>,
    additional: usize,
) -> Result<(), ScatterError> {
    items
        .try_reserve_exact(additional)
        .map_err(|_| ScatterError::AllocationFailed { additional })
}

fn validate_surface(
    s: ScatterSurface,
    id: ScatterId,
    population: ScatterPopulation,
) -> Result<(), ScatterError> {
    if !s.ground_height_m.is_finite()
        || !s.slope_rise_run.is_finite()
        || s.slope_rise_run < 0.0
        || s.water_height_m.is_some_and(|h| !h.is_finite())
        || (population == ScatterPopulation::Rocks
            && (!s.rocky01.is_finite() || !(0.0..=1.0).contains(&s.rocky01)))
        || (population != ScatterPopulation::Rocks
            && ([
                s.vegetation.ground01,
                s.vegetation.canopy01,
                s.vegetation.reeds01,
            ]
            .iter()
            .any(|v| !v.is_finite() || *v < 0.0)
                // Stored covers admit 1e-12; bilinear arithmetic adds a few
                // ulps. Only this bounded overshoot is normalized above.
                || s.vegetation.total()
                    > 1.0 + crate::ecology::COVER_SUM_TOLERANCE + 8.0 * f64::EPSILON))
    {
        return Err(ScatterError::InvalidSurface(id));
    }
    Ok(())
}

/// Base density per km² before the local rocky modifier (bounded by 4).
pub fn base_density_per_km2(biome: Biome) -> f64 {
    match biome {
        Biome::SandDesert | Biome::DuneField => 2.0,
        Biome::DeepOcean | Biome::ShallowSea => 0.0,
        Biome::PolarIceCap | Biome::Snowfield | Biome::Glacier => 8.0,
        Biome::SaltFlat | Biome::SedimentaryPlain => 15.0,
        Biome::RockyPlain | Biome::RollingHighlands | Biome::Plateau => 60.0,
        Biome::Badlands | Biome::CanyonProvince | Biome::Escarpment => 140.0,
        Biome::MountainRange | Biome::AlpinePeaks => 120.0,
        Biome::BasaltPlain | Biome::VolcanicField | Biome::ShieldVolcano => 110.0,
        Biome::Caldera | Biome::LavaFlow => 90.0,
        Biome::SimpleCrater | Biome::ComplexCrater | Biome::EjectaField => 160.0,
        Biome::CrateredHighlands | Biome::ImpactBasin => 100.0,
        Biome::CoastBeach | Biome::Archipelago => 25.0,
        Biome::ColdOcean | Biome::CoastalShelf => 0.0,
        Biome::TidalFlat | Biome::Beach | Biome::RockyCoast => 20.0,
        Biome::CoolMaritimePlain | Biome::TemperateGrassland | Biome::Steppe => 30.0,
        Biome::Wetland | Biome::RiverDelta | Biome::GeothermalWetland => 18.0,
        Biome::TemperateForest | Biome::CoolForest => 45.0,
        Biome::ColdDesert | Biome::StonyDesert | Biome::DryBasin => 25.0,
        Biome::RockyPlateau | Biome::AlpineMeadow | Biome::AlpineBarren => 55.0,
        Biome::MountainRidge => 115.0,
        Biome::SeasonalSnow | Biome::PermanentSnow | Biome::IceCap => 8.0,
        Biome::FreshLava | Biome::FumaroleField | Biome::SulfurField => 85.0,
        Biome::CraterFloor | Biome::CraterRim | Biome::EjectaPlain => 150.0,
        Biome::AncientImpactBasin => 95.0,
        Biome::PeriglacialBarren => 70.0,
    }
}

fn pick_rock_kind(seed: u64, id: ScatterId, s: ScatterSurface) -> ScatterKind {
    let random = |channel| rng::hash01(seed, channel, id.0 as i64, 0);
    if matches!(
        s.biome,
        Biome::Glacier | Biome::PolarIceCap | Biome::Snowfield
    ) && random(406) < 0.6
    {
        return ScatterKind::IceBlock;
    }
    if s.geology == Geology::Basaltic && random(407) < 0.3 {
        return ScatterKind::VolcanicBlock;
    }
    if s.slope_rise_run > 0.6 && random(408) < 0.5 {
        return ScatterKind::TalusField;
    }
    match random(409) {
        v if v < 0.55 => ScatterKind::SmallRock,
        v if v < 0.8 => ScatterKind::Boulder,
        v if v < 0.9 => ScatterKind::LargeBoulder,
        _ => ScatterKind::Outcrop,
    }
}

#[cfg(test)]
#[path = "scatter_tests.rs"]
mod tests;

#[cfg(test)]
#[path = "scatter_runtime_tests.rs"]
mod runtime_tests;
