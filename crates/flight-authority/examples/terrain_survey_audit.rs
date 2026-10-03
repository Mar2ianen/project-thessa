//! Inspect the actual shared launch field at the client's survey bookmarks.
use thessa_flight_authority::launch_site::canonical_launch_setup;
use thessa_worldgen_rocky::{features, sphere};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let config: thessa_sim_core::SystemConfig =
        toml::from_str(include_str!("../../../data/system.toml"))?;
    let (field, sites) = canonical_launch_setup(&config.bake()?)?;
    for (name, dir) in ["coast", "highlands", "volcanic"].into_iter().zip(sites) {
        let (lat, lon) = sphere::latlon_from_dir(dir);
        let mut sample = field.sample(dir, 32.0);
        let (slope, curvature) = field.surface_geometry(dir, 256.0);
        sample.slope_hint = slope;
        sample.curvature_per_m = curvature;
        let material = thessa_worldgen_rocky::appearance::surface_appearance(&field, &sample, dir);
        println!(
            "  substrate: {:?}; biome: {:?}; geothermal_flux_w_m2={}",
            sample.geology, sample.biome, sample.geothermal_flux_w_m2
        );
        println!(
            "  cover: snow_suitability={} snow_cover={} local_deposition_m={}",
            material.snow, material.snow_cover, sample.local_deposition_m
        );
        println!(
            "{name}: lat={lat} lon={lon} h={} temperature={} moisture={} macro={} detail={} slope={} erosion={}",
            sample.height_m,
            sample.temperature_k,
            sample.moisture01,
            sample.macro_height_m,
            sample.procedural_height_m,
            field.surface_geometry(dir, 256.0).0,
            sample.erosion_displacement_m
        );
        for feature in &field.features {
            let h = features::eval_feature_height_m(feature, lat, lon, field.params.radius_m);
            if h.abs() > 0.01 {
                println!("  {}: {h} m", feature.id);
            }
        }
    }
    Ok(())
}
