//! Derived inhabited-region descriptors, not city growth/traffic simulation.
use crate::{appearance::smooth, field::PlanetField, sphere};

#[derive(Debug, Clone, Copy, serde::Deserialize, serde::Serialize)]
#[serde(default)]
pub struct CivilizationRecipe {
    pub total_population: u64,
    pub region_count: usize,
    pub candidate_samples: usize,
    pub min_separation_m: f64,
    pub max_water_distance_m: f64,
    pub max_slope: f64,
}

impl Default for CivilizationRecipe {
    fn default() -> Self {
        Self {
            total_population: 150_000_000,
            region_count: 48,
            candidate_samples: 8192,
            min_separation_m: 250_000.0,
            max_water_distance_m: 200_000.0,
            max_slope: 0.05,
        }
    }
}

impl CivilizationRecipe {
    pub fn validate(self) -> Result<(), String> {
        if self.total_population == 0
            || self.total_population > 1_000_000_000_000
            || !(1..=256).contains(&self.region_count)
            || !(self.region_count..=131072).contains(&self.candidate_samples)
            || self.total_population < self.region_count as u64
        {
            return Err("civilization needs positive bounded population, 1..256 regions and sufficient candidates".into());
        }
        for value in [
            self.min_separation_m,
            self.max_water_distance_m,
            self.max_slope,
        ] {
            if !value.is_finite() || value <= 0.0 {
                return Err("civilization distances/slope must be finite and positive".into());
            }
        }
        if self.max_slope > 1.0 {
            return Err("civilization slope must be <= 1".into());
        }
        Ok(())
    }
}

#[derive(Debug, Clone, PartialEq, serde::Serialize)]
pub struct InhabitedRegion {
    pub id: u32,
    pub center_dir: [f64; 3],
    /// Includes surrounding settlements, not a literal city-proper headcount.
    pub population: u64,
    pub height_m: f64,
    pub slope: f64,
    pub temperature_k: f64,
    pub water_distance_m: f64,
    pub suitability: f64,
}

/// Stable equal-area candidate survey scored by water/terrain/climate, with
/// separated centers. Does not flatten terrain or create buildings/light masks.
pub fn derive_regions(
    field: &PlanetField,
    recipe: CivilizationRecipe,
) -> Result<Vec<InhabitedRegion>, String> {
    recipe.validate()?;
    let mut candidates = Vec::new();
    for i in 0..recipe.candidate_samples {
        let y = 1.0 - 2.0 * (i as f64 + 0.5) / recipe.candidate_samples as f64;
        let angle = i as f64 * 2.399963229728653;
        let r = (1.0 - y * y).sqrt();
        let dir = [r * angle.cos(), y, r * angle.sin()];
        let mut sample = field.sample_surface(dir, 32.0);
        if sample.height_m < 20.0
            || sample.height_m > 2000.0
            || !(265.0..=305.0).contains(&sample.temperature_k)
            || sample.wetland_potential01 > 0.4
            || sample.geothermal_flux_w_m2 > 2.0
        {
            continue;
        }
        sample.slope_hint = field.slope_hint(dir, 256.0);
        if sample.slope_hint > recipe.max_slope {
            continue;
        }
        let mut water_distance = sample.continentality01 * 2_500_000.0;
        for river in field.rivers() {
            water_distance = water_distance.min(sphere::distance_to_arc_m(
                dir,
                river.start_dir,
                river.end_dir,
                field.params.radius_m,
            ));
        }
        if water_distance > recipe.max_water_distance_m {
            continue;
        }
        let cover = crate::ecology::vegetation_cover(&sample, sample.slope_hint).total();
        let suitability = (1.0 - smooth(0.0, recipe.max_water_distance_m, water_distance))
            * (1.0 - smooth(0.0, recipe.max_slope, sample.slope_hint))
            * (0.5 + 0.5 * cover);
        if suitability <= 0.0 {
            continue;
        }
        candidates.push(InhabitedRegion {
            id: i as u32,
            center_dir: dir,
            population: 0,
            height_m: sample.height_m,
            slope: sample.slope_hint,
            temperature_k: sample.temperature_k,
            water_distance_m: water_distance,
            suitability,
        });
    }
    candidates.sort_by(|a, b| {
        b.suitability
            .total_cmp(&a.suitability)
            .then(a.id.cmp(&b.id))
    });
    let mut regions: Vec<InhabitedRegion> = Vec::new();
    for candidate in candidates {
        if regions.iter().all(|region| {
            sphere::great_circle_m(
                region.center_dir,
                candidate.center_dir,
                field.params.radius_m,
            ) >= recipe.min_separation_m
        }) {
            regions.push(candidate);
            if regions.len() == recipe.region_count {
                break;
            }
        }
    }
    if regions.len() != recipe.region_count {
        return Err(format!(
            "only {} suitable separated regions for requested {}",
            regions.len(),
            recipe.region_count
        ));
    }
    allocate_population(&mut regions, recipe.total_population);
    Ok(regions)
}

fn allocate_population(regions: &mut [InhabitedRegion], total: u64) {
    let sum = regions.iter().map(|r| r.suitability).sum::<f64>();
    let mut remaining = total - regions.len() as u64;
    let distributable = remaining;
    for region in regions.iter_mut() {
        let share =
            ((distributable as f64 * region.suitability / sum).floor() as u64).min(remaining);
        region.population = 1 + share;
        remaining -= share;
    }
    // Deterministic residual assignment, preserving the exact authored total.
    let count = regions.len() as u64;
    for (i, region) in regions.iter_mut().enumerate() {
        region.population += remaining / count + u64::from((i as u64) < remaining % count);
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn canonical_regions_are_deterministic_separated_and_conserve_population() {
        let spec: crate::spec_recipe::SpecRecipe =
            toml::from_str(include_str!("../../../data/worldgen/worldgen_recipe.toml")).unwrap();
        let recipe = spec.civilization.unwrap();
        let manifest = crate::spec_recipe::manifest_from_spec(&spec).unwrap();
        let field = crate::field::field_from_manifest(&manifest).unwrap();
        assert_eq!(
            field.inhabited_regions,
            derive_regions(&field, recipe).unwrap()
        );
        assert_eq!(field.inhabited_regions.len(), recipe.region_count);
        assert_eq!(
            field
                .inhabited_regions
                .iter()
                .map(|r| r.population)
                .sum::<u64>(),
            recipe.total_population
        );
        for (i, region) in field.inhabited_regions.iter().enumerate() {
            let sample = field.sample_surface(region.center_dir, 32.0);
            assert!(sample.height_m >= 20.0 && sample.height_m <= 2000.0);
            assert!(sample.wetland_potential01 <= 0.4);
            assert!(region.population > 0 && region.slope <= recipe.max_slope);
            assert!(region.water_distance_m <= recipe.max_water_distance_m);
            for other in &field.inhabited_regions[..i] {
                assert!(
                    sphere::great_circle_m(
                        region.center_dir,
                        other.center_dir,
                        field.params.radius_m
                    ) >= recipe.min_separation_m
                );
            }
        }
        let mut uninhabited = manifest.clone();
        uninhabited.civilization = None;
        assert!(
            crate::field::field_from_manifest(&uninhabited)
                .unwrap()
                .inhabited_regions
                .is_empty()
        );
    }

    #[test]
    fn invalid_population_and_site_budgets_fail_explicitly() {
        for recipe in [
            CivilizationRecipe {
                total_population: 0,
                ..Default::default()
            },
            CivilizationRecipe {
                region_count: 257,
                ..Default::default()
            },
            CivilizationRecipe {
                min_separation_m: f64::NAN,
                ..Default::default()
            },
        ] {
            assert!(recipe.validate().is_err());
        }
    }
}
