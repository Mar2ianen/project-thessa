//! Ecological coverage derived from terrain, climate and regional hydrology.
//! A coverage proxy/art direction, not plant physiology or vegetation rendering.
use crate::{
    appearance::smooth,
    biomes::{Biome, Geology, SiteClass},
    field::TerrainSample,
};

/// Primary ecological biome, independent of tile IDs and rendering quality.
/// Geology and relief tags survive: an old impact floor can support a forest.
pub fn classify_ecological_site(
    mut site: SiteClass,
    height_m: f64,
    temperature_k: f64,
    moisture01: f64,
    wetland01: f64,
    continentality01: f64,
) -> SiteClass {
    if height_m < 0.0 {
        if temperature_k < 273.15 {
            site.biome = Biome::ColdOcean;
        }
        return site;
    }
    // Active volcanic and explicitly saline substrates keep their identity.
    if matches!(
        site.biome,
        Biome::Caldera | Biome::LavaFlow | Biome::ShieldVolcano
    ) || site.geology == Geology::Evaporite
    {
        return site;
    }
    if temperature_k < 258.0 {
        site.biome = if moisture01 > 0.35 {
            Biome::PermanentSnow
        } else {
            Biome::ColdDesert
        };
    } else if temperature_k < 268.0 {
        site.biome = if moisture01 > 0.55 {
            Biome::SeasonalSnow
        } else {
            Biome::PeriglacialBarren
        };
    } else if wetland01 > 0.5 && height_m < 400.0 {
        site.biome = Biome::Wetland;
        site.geology = Geology::Sedimentary;
    } else {
        site.biome = terrestrial_biome(
            site.biome,
            height_m,
            temperature_k,
            moisture01,
            continentality01,
        );
    }
    site
}

fn terrestrial_biome(relief: Biome, h: f64, t: f64, moisture: f64, continentality: f64) -> Biome {
    // Dry plateaus and high basins must not bypass the ecological decision.
    // Relief remains in the height field and geology, not a forced biome ID.
    if moisture < 0.2 {
        return if t < 278.0 {
            Biome::ColdDesert
        } else {
            Biome::StonyDesert
        };
    }
    if h > 2500.0 {
        return if moisture > 0.45 && t > 272.0 && h < 4000.0 {
            Biome::AlpineMeadow
        } else {
            Biome::AlpineBarren
        };
    }
    if matches!(
        relief,
        Biome::CanyonProvince
            | Biome::Escarpment
            | Biome::MountainRange
            | Biome::Plateau
            | Biome::VolcanicField
            | Biome::Glacier
    ) {
        return relief;
    }
    if moisture < 0.4 {
        return Biome::Steppe;
    }
    if moisture > 0.62 {
        return if t < 280.0 {
            Biome::CoolForest
        } else {
            Biome::TemperateForest
        };
    }
    if h > 1500.0 {
        return Biome::RollingHighlands;
    }
    if t < 280.0 && continentality < 0.15 {
        Biome::CoolMaritimePlain
    } else {
        Biome::TemperateGrassland
    }
}

#[derive(Debug, Clone, Copy, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct VegetationCover {
    pub ground01: f64,
    pub canopy01: f64,
    pub reeds01: f64,
}

impl VegetationCover {
    pub fn total(self) -> f64 {
        self.ground01 + self.canopy01 + self.reeds01
    }
}

/// Local material slope must be supplied at a consistent physical wavelength.
/// Saturated lowlands favour reeds/forest; dry/cold/steep/salty sites suppress
/// cover. No tile IDs, random biome colours or render quality enter this field.
pub fn vegetation_cover(sample: &TerrainSample, slope: f64) -> VegetationCover {
    if sample.height_m < 0.0 {
        return VegetationCover {
            ground01: 0.0,
            canopy01: 0.0,
            reeds01: 0.0,
        };
    }
    let wet = sample.wetland_potential01 * (1.0 - smooth(0.02, 0.1, slope));
    let moisture = sample.moisture01.max(wet);
    let thermal = smooth(255.0, 278.0, sample.temperature_k)
        * (1.0 - smooth(315.0, 335.0, sample.temperature_k));
    let substrate = if sample.geology == Geology::Evaporite {
        0.05
    } else {
        1.0
    };
    let total = smooth(0.15, 0.65, moisture)
        * thermal
        * substrate
        * (1.0 - smooth(0.25, 0.8, slope))
        * (1.0 - smooth(2500.0, 5500.0, sample.height_m));
    let reeds = wet * 0.65;
    let canopy = smooth(0.45, 0.85, moisture)
        * smooth(260.0, 280.0, sample.temperature_k)
        * (1.0 - smooth(1500.0, 3000.0, sample.height_m))
        * (1.0 - reeds)
        * 0.75;
    VegetationCover {
        ground01: total * (1.0 - reeds - canopy),
        canopy01: total * canopy,
        reeds01: total * reeds,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn ecological_biomes_follow_climate_and_keep_inherited_geology() {
        let old_floor = SiteClass::new(Biome::ImpactBasin, Geology::ImpactBreccia);
        for (t, m, wet, expected) in [
            (285.0, 0.8, 0.0, Biome::TemperateForest),
            (275.0, 0.8, 0.0, Biome::CoolForest),
            (285.0, 0.5, 0.0, Biome::TemperateGrassland),
            (285.0, 0.3, 0.0, Biome::Steppe),
            (275.0, 0.1, 0.0, Biome::ColdDesert),
            (285.0, 0.1, 0.0, Biome::StonyDesert),
            (250.0, 0.8, 0.0, Biome::PermanentSnow),
            (275.0, 0.8, 1.0, Biome::Wetland),
        ] {
            let site = classify_ecological_site(old_floor, 100.0, t, m, wet, 0.2);
            assert_eq!(site.biome, expected);
            assert_eq!(
                site.geology,
                if wet > 0.5 {
                    Geology::Sedimentary
                } else {
                    Geology::ImpactBreccia
                }
            );
        }
        let salt = SiteClass::new(Biome::SaltFlat, Geology::Evaporite);
        assert_eq!(
            classify_ecological_site(salt, 100.0, 285.0, 0.8, 1.0, 0.2),
            salt
        );
        let ocean = SiteClass::new(Biome::DeepOcean, Geology::OceanicCrust);
        assert_eq!(
            classify_ecological_site(ocean, -1000.0, 275.0, 1.0, 1.0, 0.0),
            ocean
        );
    }

    #[test]
    fn wet_lowlands_support_cover_but_water_cliffs_cold_and_salt_do_not() {
        let recipe =
            toml::from_str(include_str!("../../../data/worldgen/worldgen_recipe.toml")).unwrap();
        let field = crate::field::field_from_manifest(
            &crate::spec_recipe::manifest_from_spec(&recipe).unwrap(),
        )
        .unwrap();
        let mut sample = field.sample_surface([1.0, 0.0, 0.0], 32.0);
        sample.height_m = 100.0;
        sample.temperature_k = 285.0;
        sample.moisture01 = 0.8;
        sample.wetland_potential01 = 1.0;
        sample.geology = Geology::Sedimentary;
        let wet = vegetation_cover(&sample, 0.0);
        assert!(wet.reeds01 > 0.5 && wet.canopy01 > 0.1 && wet.total() > 0.9);
        sample.wetland_potential01 = 0.0;
        sample.moisture01 = 0.05;
        assert!(vegetation_cover(&sample, 0.0).total() < wet.total());
        sample.moisture01 = 0.8;
        assert_eq!(vegetation_cover(&sample, 1.0).total(), 0.0);
        sample.temperature_k = 240.0;
        assert_eq!(vegetation_cover(&sample, 0.0).total(), 0.0);
        sample.temperature_k = 285.0;
        sample.geology = Geology::Evaporite;
        assert!(vegetation_cover(&sample, 0.0).total() <= 0.05);
        sample.height_m = -1.0;
        assert_eq!(vegetation_cover(&sample, 0.0).total(), 0.0);
    }
}
