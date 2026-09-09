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

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ScatterKind {
    SmallRock,
    Boulder,
    LargeBoulder,
    Outcrop,
    TalusField,
    VolcanicBlock,
    IceBlock,
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
        Biome::PeriglacialBarren => 70.0,
    }
}

/// Deterministic jittered-grid placement (stable Poisson-like distribution).
/// Same inputs => byte-identical items, any tile order.
pub fn scatter_tile(ctx: ScatterContext) -> Vec<ScatterItem> {
    let per_km2 = base_density_per_km2(ctx.biome) * (1.0 + 3.0 * ctx.rocky01);
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
            let kind = pick_kind(seed, ctx, gx as i64, gy as i64);
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
