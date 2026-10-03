//! Deterministic surface scatter descriptors (no rendering dependency).
//!
//! Rocks are NOT baked into global maps. A runtime tile regenerates identical
//! scatter from (planet_seed, tile_id, biome/geology/slope context).
//!
//! Density rules: plains sparse, talus/ejecta dense, dunes ~empty,
//! glaciers carry ice blocks.

use crate::{
    biomes::{Biome, Geology},
    rng,
};

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
}

/// One deterministic scatter item in tile-local metres.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ScatterItem {
    pub kind: ScatterKind,
    /// Tile-local position, metres from tile origin.
    pub x_m: f64,
    pub y_m: f64,
    /// Uniform scale multiplier.
    pub scale: f64,
    /// Yaw radians.
    pub yaw_rad: f64,
}

/// Context a tile provides.
#[derive(Debug, Clone, Copy)]
pub struct ScatterContext {
    pub planet_seed: u64,
    pub tile_id: u64,
    pub biome: Biome,
    pub geology: Geology,
    /// 0 flat .. 1 cliff.
    pub slope01: f64,
    /// 1.0 on fresh ejecta / below cliffs, 0.0 elsewhere.
    pub rocky01: f64,
    /// Tile edge length in metres.
    pub tile_size_m: f64,
}

/// Base density per km^2 for a biome before modifiers.
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

/// Deterministic jittered-grid placement (stable Poisson-like distribution).
/// Same inputs => byte-identical items, any tile order.
pub fn scatter_tile(ctx: ScatterContext) -> Vec<ScatterItem> {
    let per_km2 = base_density_per_km2(ctx.biome) * (1.0 + 3.0 * ctx.rocky01);
    scatter_distribution(ctx, per_km2, |seed, gx, gy| pick_kind(seed, ctx, gx, gy))
}

/// Carboniferous-inspired instance descriptors over the existing placement
/// kernel. This does not render plants or bake them into the global map.
pub fn scatter_vegetation(
    ctx: ScatterContext,
    cover: crate::ecology::VegetationCover,
) -> Vec<ScatterItem> {
    let total = cover.total();
    if !total.is_finite()
        || total <= 0.0
        || total > 1.0
        || [cover.ground01, cover.canopy01, cover.reeds01]
            .iter()
            .any(|v| !v.is_finite() || *v < 0.0)
    {
        return Vec::new();
    }
    // Density describes clumps/crowns, not individual grass blades.
    scatter_distribution(ctx, total * 3000.0, |seed, gx, gy| {
        let selection = rng::hash01(seed, 431, gx, gy) * total;
        if selection < cover.reeds01 {
            ScatterKind::ReedClump
        } else if selection < cover.reeds01 + cover.canopy01 {
            ScatterKind::LycopsidTree
        } else {
            ScatterKind::FernClump
        }
    })
}

fn scatter_distribution(
    ctx: ScatterContext,
    per_km2: f64,
    kind_at: impl Fn(u64, i64, i64) -> ScatterKind,
) -> Vec<ScatterItem> {
    let area_km2 = (ctx.tile_size_m / 1000.0).powi(2);
    let mut target = (per_km2 * area_km2).round() as usize;
    target = target.min(4000);
    let seed = ctx.planet_seed ^ ctx.tile_id.wrapping_mul(0x9E37_79B9_7F4A_7C15);
    let grid = (target as f64).sqrt().ceil().max(1.0) as usize;
    let mut items = Vec::new();
    let cell = ctx.tile_size_m / grid as f64;
    for gx in 0..grid {
        for gy in 0..grid {
            if items.len() >= target {
                break;
            }
            let keep = rng::hash01(seed, 401, gx as i64, gy as i64);
            // Keep probability scaled so expected count == target.
            let cells = (grid * grid) as f64;
            if keep > target as f64 / cells {
                continue;
            }
            let jx = rng::hash01(seed, 402, gx as i64, gy as i64);
            let jy = rng::hash01(seed, 403, gx as i64, gy as i64);
            let kind = kind_at(seed, gx as i64, gy as i64);
            items.push(ScatterItem {
                kind,
                x_m: (gx as f64 + jx) * cell,
                y_m: (gy as f64 + jy) * cell,
                scale: 0.5 + rng::hash01(seed, 404, gx as i64, gy as i64) * 1.8,
                yaw_rad: rng::hash01(seed, 405, gx as i64, gy as i64) * std::f64::consts::TAU,
            });
        }
    }
    items
}

fn pick_kind(seed: u64, ctx: ScatterContext, gx: i64, gy: i64) -> ScatterKind {
    // Ice biomes carry ice blocks; volcanic geology carries blocks.
    if matches!(
        ctx.biome,
        Biome::Glacier | Biome::PolarIceCap | Biome::Snowfield
    ) && rng::hash01(seed, 406, gx, gy) < 0.6
    {
        return ScatterKind::IceBlock;
    }
    if matches!(ctx.geology, Geology::Basaltic) && rng::hash01(seed, 407, gx, gy) < 0.3 {
        return ScatterKind::VolcanicBlock;
    }
    if ctx.slope01 > 0.6 && rng::hash01(seed, 408, gx, gy) < 0.5 {
        return ScatterKind::TalusField;
    }
    match rng::hash01(seed, 409, gx, gy) {
        v if v < 0.55 => ScatterKind::SmallRock,
        v if v < 0.8 => ScatterKind::Boulder,
        v if v < 0.9 => ScatterKind::LargeBoulder,
        _ => ScatterKind::Outcrop,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn ctx(biome: Biome) -> ScatterContext {
        ScatterContext {
            planet_seed: 7,
            tile_id: 123,
            biome,
            geology: Geology::Regolith,
            slope01: 0.2,
            rocky01: 0.0,
            tile_size_m: 1000.0,
        }
    }

    #[test]
    fn deterministic_regeneration() {
        let a = scatter_tile(ctx(Biome::RockyPlain));
        let b = scatter_tile(ctx(Biome::RockyPlain));
        assert_eq!(a.len(), b.len());
        assert_eq!(a.first(), b.first());
    }

    #[test]
    fn vegetation_descriptors_reuse_stable_placement_and_cover_types() {
        let context = ctx(Biome::Wetland);
        let cover = crate::ecology::VegetationCover {
            ground01: 0.2,
            canopy01: 0.2,
            reeds01: 0.6,
        };
        let items = scatter_vegetation(context, cover);
        assert_eq!(items, scatter_vegetation(context, cover));
        assert!(items.len() <= 4000 && !items.is_empty());
        for kind in [
            ScatterKind::FernClump,
            ScatterKind::LycopsidTree,
            ScatterKind::ReedClump,
        ] {
            assert!(items.iter().any(|item| item.kind == kind));
        }
        assert!(
            items
                .iter()
                .all(|item| (0.0..context.tile_size_m).contains(&item.x_m)
                    && (0.0..context.tile_size_m).contains(&item.y_m))
        );
        assert!(
            scatter_vegetation(
                context,
                crate::ecology::VegetationCover {
                    ground01: 0.0,
                    canopy01: 0.0,
                    reeds01: 0.0
                }
            )
            .is_empty()
        );
    }

    #[test]
    fn dunes_nearly_empty_ejecta_dense() {
        let dunes = scatter_tile(ctx(Biome::DuneField)).len();
        let ejecta = scatter_tile(ctx(Biome::EjectaField)).len();
        assert!(dunes < ejecta, "{dunes} vs {ejecta}");
        assert!(dunes <= 4);
    }

    #[test]
    fn talus_below_cliffs() {
        let mut c = ctx(Biome::MountainRange);
        c.slope01 = 0.8;
        let items = scatter_tile(c);
        assert!(items.iter().any(|i| i.kind == ScatterKind::TalusField));
    }

    #[test]
    fn different_tiles_differ() {
        let mut c = ctx(Biome::RockyPlain);
        c.tile_id = 999;
        let a = scatter_tile(ctx(Biome::RockyPlain));
        let b = scatter_tile(c);
        assert_ne!(a.first(), b.first());
    }
}
