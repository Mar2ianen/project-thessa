//! Biome / geology / feature-tag taxonomy.
//!
//! World semantics are NOT one RGB palette. A site has a primary [`Biome`],
//! a [`Geology`] and optional [`FeatureTag`]s. Rendering palettes are derived
//! from this triple, never the other way around.

use std::{error::Error, fmt};

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum Biome {
    // water / coast
    DeepOcean,
    ShallowSea,
    CoastBeach,
    Archipelago,
    // lowlands
    SedimentaryPlain,
    RockyPlain,
    SandDesert,
    DuneField,
    SaltFlat,
    Badlands,
    // uplands
    RollingHighlands,
    Plateau,
    MountainRange,
    AlpinePeaks,
    Escarpment,
    CanyonProvince,
    // volcanic
    BasaltPlain,
    VolcanicField,
    ShieldVolcano,
    Caldera,
    LavaFlow,
    // impact
    SimpleCrater,
    ComplexCrater,
    CrateredHighlands,
    EjectaField,
    ImpactBasin,
    // cold
    Snowfield,
    Glacier,
    PolarIceCap,
    PeriglacialBarren,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum Geology {
    OceanicCrust,
    ContinentalCrust,
    Sedimentary,
    Basaltic,
    FelsicHighland,
    ImpactBreccia,
    Evaporite,
    GlacialTill,
    Regolith,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum FeatureTag {
    Caldera,
    Glacier,
    CentralPeak,
    MultiRing,
    LavaTube,
    DuneSea,
    SaltPan,
    RiverDelta,
    Fjord,
    RiftValley,
    HotSpot,
    ColdTrap,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct SiteClass {
    pub biome: Biome,
    pub geology: Geology,
    pub tag0: Option<FeatureTag>,
    pub tag1: Option<FeatureTag>,
}

impl SiteClass {
    pub fn new(biome: Biome, geology: Geology) -> Self {
        Self {
            biome,
            geology,
            tag0: None,
            tag1: None,
        }
    }

    pub fn with_tags(mut self, tag0: FeatureTag, tag1: Option<FeatureTag>) -> Self {
        self.tag0 = Some(tag0);
        self.tag1 = tag1;
        self
    }
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct TaxonomyError(pub String);

impl fmt::Display for TaxonomyError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "unknown biome/geology/tag: {}", self.0)
    }
}

impl Error for TaxonomyError {}

pub fn parse_biome(name: &str) -> Result<Biome, TaxonomyError> {
    Ok(match name {
        "deep_ocean" => Biome::DeepOcean,
        "shallow_sea" => Biome::ShallowSea,
        "coast_beach" => Biome::CoastBeach,
        "archipelago" => Biome::Archipelago,
        "sedimentary_plain" => Biome::SedimentaryPlain,
        "rocky_plain" => Biome::RockyPlain,
        "sand_desert" => Biome::SandDesert,
        "dune_field" => Biome::DuneField,
        "salt_flat" => Biome::SaltFlat,
        "badlands" => Biome::Badlands,
        "rolling_highlands" => Biome::RollingHighlands,
        "plateau" => Biome::Plateau,
        "mountain_range" => Biome::MountainRange,
        "alpine_peaks" => Biome::AlpinePeaks,
        "escarpment" => Biome::Escarpment,
        "canyon_province" => Biome::CanyonProvince,
        "basalt_plain" => Biome::BasaltPlain,
        "volcanic_field" => Biome::VolcanicField,
        "shield_volcano" => Biome::ShieldVolcano,
        "caldera" => Biome::Caldera,
        "lava_flow" => Biome::LavaFlow,
        "simple_crater" => Biome::SimpleCrater,
        "complex_crater" => Biome::ComplexCrater,
        "cratered_highlands" => Biome::CrateredHighlands,
        "ejecta_field" => Biome::EjectaField,
        "impact_basin" => Biome::ImpactBasin,
        "snowfield" => Biome::Snowfield,
        "glacier" => Biome::Glacier,
        "polar_ice_cap" => Biome::PolarIceCap,
        "periglacial_barren" => Biome::PeriglacialBarren,
        other => return Err(TaxonomyError(other.to_string())),
    })
}

pub fn parse_geology(name: &str) -> Result<Geology, TaxonomyError> {
    Ok(match name {
        "oceanic_crust" => Geology::OceanicCrust,
        "continental_crust" => Geology::ContinentalCrust,
        "sedimentary" => Geology::Sedimentary,
        "basaltic" => Geology::Basaltic,
        "felsic_highland" => Geology::FelsicHighland,
        "impact_breccia" => Geology::ImpactBreccia,
        "evaporite" => Geology::Evaporite,
        "glacial_till" => Geology::GlacialTill,
        "regolith" => Geology::Regolith,
        other => return Err(TaxonomyError(other.to_string())),
    })
}

pub fn parse_tag(name: &str) -> Result<FeatureTag, TaxonomyError> {
    Ok(match name {
        "caldera" => FeatureTag::Caldera,
        "glacier" => FeatureTag::Glacier,
        "central_peak" => FeatureTag::CentralPeak,
        "multi_ring" => FeatureTag::MultiRing,
        "lava_tube" => FeatureTag::LavaTube,
        "dune_sea" => FeatureTag::DuneSea,
        "salt_pan" => FeatureTag::SaltPan,
        "river_delta" => FeatureTag::RiverDelta,
        "fjord" => FeatureTag::Fjord,
        "rift_valley" => FeatureTag::RiftValley,
        "hot_spot" => FeatureTag::HotSpot,
        "cold_trap" => FeatureTag::ColdTrap,
        other => return Err(TaxonomyError(other.to_string())),
    })
}

/// Flat display palette derived FROM the taxonomy (preview/debug only).
/// Returns sRGB hex triple.
pub fn biome_color(biome: Biome) -> (u8, u8, u8) {
    match biome {
        Biome::DeepOcean => (0, 24, 168),
        Biome::ShallowSea => (30, 111, 255),
        Biome::CoastBeach => (217, 196, 150),
        Biome::Archipelago => (64, 160, 200),
        Biome::SedimentaryPlain => (168, 152, 120),
        Biome::RockyPlain => (138, 123, 107),
        Biome::SandDesert => (217, 179, 128),
        Biome::DuneField => (226, 190, 138),
        Biome::SaltFlat => (240, 238, 230),
        Biome::Badlands => (150, 100, 80),
        Biome::RollingHighlands => (120, 118, 105),
        Biome::Plateau => (140, 120, 100),
        Biome::MountainRange => (107, 107, 107),
        Biome::AlpinePeaks => (220, 228, 235),
        Biome::Escarpment => (110, 80, 65),
        Biome::CanyonProvince => (90, 58, 46),
        Biome::BasaltPlain => (58, 52, 50),
        Biome::VolcanicField => (74, 46, 38),
        Biome::ShieldVolcano => (88, 60, 48),
        Biome::Caldera => (58, 30, 20),
        Biome::LavaFlow => (255, 74, 0),
        Biome::SimpleCrater => (150, 140, 128),
        Biome::ComplexCrater => (160, 148, 132),
        Biome::CrateredHighlands => (130, 122, 110),
        Biome::EjectaField => (168, 160, 148),
        Biome::ImpactBasin => (100, 95, 110),
        Biome::Snowfield => (232, 244, 255),
        Biome::Glacier => (200, 225, 245),
        Biome::PolarIceCap => (232, 244, 255),
        Biome::PeriglacialBarren => (150, 155, 160),
    }
}

/// Roughness prior (0 smooth .. 255 rough) derived from biome, before variation.
pub fn biome_roughness_prior(biome: Biome) -> u8 {
    match biome {
        Biome::DeepOcean | Biome::ShallowSea => 20,
        Biome::PolarIceCap | Biome::Snowfield | Biome::Glacier => 30,
        Biome::SaltFlat | Biome::CoastBeach => 60,
        Biome::SedimentaryPlain | Biome::SandDesert => 90,
        Biome::RockyPlain | Biome::RollingHighlands | Biome::Plateau => 128,
        Biome::Archipelago => 110,
        Biome::DuneField => 100,
        Biome::Badlands | Biome::CanyonProvince | Biome::Escarpment => 180,
        Biome::MountainRange | Biome::AlpinePeaks => 200,
        Biome::BasaltPlain | Biome::VolcanicField | Biome::ShieldVolcano => 170,
        Biome::Caldera | Biome::LavaFlow => 150,
        Biome::SimpleCrater | Biome::ComplexCrater | Biome::EjectaField => 190,
        Biome::CrateredHighlands | Biome::ImpactBasin => 175,
        Biome::PeriglacialBarren => 140,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn all_documented_names_parse() {
        for name in [
            "deep_ocean",
            "shallow_sea",
            "coast_beach",
            "archipelago",
            "sedimentary_plain",
            "rocky_plain",
            "sand_desert",
            "dune_field",
            "salt_flat",
            "badlands",
            "rolling_highlands",
            "plateau",
            "mountain_range",
            "alpine_peaks",
            "escarpment",
            "canyon_province",
            "basalt_plain",
            "volcanic_field",
            "shield_volcano",
            "caldera",
            "lava_flow",
            "simple_crater",
            "complex_crater",
            "cratered_highlands",
            "ejecta_field",
            "impact_basin",
            "snowfield",
            "glacier",
            "polar_ice_cap",
            "periglacial_barren",
        ] {
            assert!(parse_biome(name).is_ok(), "{name}");
        }
    }

    #[test]
    fn unknown_names_rejected() {
        assert!(parse_biome("mordor").is_err());
        assert!(parse_geology("unobtanium").is_err());
        assert!(parse_tag("sauron").is_err());
    }

    #[test]
    fn site_triple_example() {
        let site = SiteClass::new(Biome::MountainRange, Geology::Basaltic)
            .with_tags(FeatureTag::Caldera, Some(FeatureTag::Glacier));
        assert_eq!(site.tag0, Some(FeatureTag::Caldera));
        assert_eq!(site.tag1, Some(FeatureTag::Glacier));
    }
}
