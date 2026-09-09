//! Thessa rocky-planet worldgen dev tool (MIT, dev-only).
//!
//! Authored macrostructure + deterministic geology/biome logic + explicit
//! landmark generators + procedural physical-scale detail. Resolution-agnostic:
//! all scales in metres, gores as normalized fractions.

use thessa_worldgen_rocky::{bake, manifest, preview, terrain};

use std::{env, error::Error, fmt, fs, path::PathBuf};

use manifest::{CANONICAL_LAYERS, Manifest, validate_manifest};

#[derive(Debug)]
struct ToolError(String);

impl fmt::Display for ToolError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{}", self.0)
    }
}

impl Error for ToolError {}

fn fail(message: impl Into<String>) -> Box<dyn Error> {
    Box::new(ToolError(message.into()))
}

fn main() -> Result<(), Box<dyn Error>> {
    let mut args = env::args().skip(1).peekable();
    let command = args.next().ok_or_else(|| {
        fail("usage: thessa-worldgen-rocky <check|sample|list-prompts|preview|bake> ...")
    })?;
    match command.as_str() {
        "check" => cmd_check(args),
        "sample" => cmd_sample(args),
        "list-prompts" => {
            for layer in CANONICAL_LAYERS {
                println!("prompts/{layer}.md");
            }
            Ok(())
        }
        "preview" => cmd_preview(args),
        "bake" => cmd_bake(args),
        "--help" | "-h" | "help" => {
            print_help();
            Ok(())
        }
        unknown => Err(fail(format!("unknown command {unknown}; use --help"))),
    }
}

fn print_help() {
    println!("Usage:");
    println!("  thessa-worldgen-rocky check --manifest <TOML>");
    println!(
        "  thessa-worldgen-rocky sample --manifest <TOML> --lat-deg 12 --lon-deg -40 --detail-scale-m 250"
    );
    println!("  thessa-worldgen-rocky list-prompts");
    println!("  thessa-worldgen-rocky preview --manifest <TOML> --step-deg 2 --out /tmp/pv");
    println!("  thessa-worldgen-rocky bake --manifest <TOML> --step-deg 2 [--out report.json]");
}

fn load_manifest(path: &PathBuf) -> Result<Manifest, Box<dyn Error>> {
    let source = fs::read_to_string(path)?;
    Ok(toml::from_str(&source)?)
}

fn cmd_check(mut args: impl Iterator<Item = String>) -> Result<(), Box<dyn Error>> {
    let mut manifest_path = PathBuf::from("data/worldgen/example_rocky.toml");
    while let Some(arg) = args.next() {
        match arg.as_str() {
            "--manifest" => {
                manifest_path = PathBuf::from(
                    args.next()
                        .ok_or_else(|| fail("--manifest requires a value"))?,
                );
            }
            unknown => return Err(fail(format!("unknown argument {unknown}"))),
        }
    }
    let manifest = load_manifest(&manifest_path)?;
    validate_manifest(&manifest).map_err(fail)?;
    for feature in &manifest.features {
        feature.validate().map_err(fail)?;
    }
    println!(
        "planet: {} (rocky, seed {})",
        manifest.planet.name, manifest.planet.seed
    );
    println!(
        "gores: {} overlap {:.3} caps {}",
        manifest.gores.count, manifest.gores.overlap_fraction, manifest.gores.polar_caps
    );
    println!("layers: {}", manifest.layers.len());
    for layer in &manifest.layers {
        println!(
            "  {name:>10}  {file}  <- {prompt}",
            name = layer.name,
            file = layer.file,
            prompt = layer.prompt
        );
    }
    println!("features: {}", manifest.features.len());
    println!(
        "recipe: mountains {:.2} craters {:.2} volcanism {:.2} erosion {:.2} aridity {:.2}",
        manifest.terrain.mountain_coverage,
        manifest.terrain.crater_density,
        manifest.terrain.volcanism,
        manifest.terrain.erosion,
        manifest.climate.aridity,
    );
    println!("ok: resolution-agnostic, no pixel constants pinned");
    Ok(())
}

fn cmd_sample(mut args: impl Iterator<Item = String>) -> Result<(), Box<dyn Error>> {
    let mut manifest_path = PathBuf::from("data/worldgen/example_rocky.toml");
    let mut lat_deg: f64 = 0.0;
    let mut lon_deg: f64 = 0.0;
    let mut detail_scale_m: f64 = 250.0;
    while let Some(arg) = args.next() {
        match arg.as_str() {
            "--manifest" => {
                manifest_path = PathBuf::from(
                    args.next()
                        .ok_or_else(|| fail("--manifest requires a value"))?,
                );
            }
            "--lat-deg" => {
                lat_deg = args
                    .next()
                    .ok_or_else(|| fail("--lat-deg requires a value"))?
                    .parse()?
            }
            "--lon-deg" => {
                lon_deg = args
                    .next()
                    .ok_or_else(|| fail("--lon-deg requires a value"))?
                    .parse()?
            }
            "--detail-scale-m" => {
                detail_scale_m = args
                    .next()
                    .ok_or_else(|| fail("--detail-scale-m requires a value"))?
                    .parse()?
            }
            unknown => return Err(fail(format!("unknown argument {unknown}"))),
        }
    }
    if !lat_deg.is_finite() || !(-90.0..=90.0).contains(&lat_deg) {
        return Err(fail("--lat-deg must be within -90..90"));
    }
    if !lon_deg.is_finite() || !(-180.0..=180.0).contains(&lon_deg) {
        return Err(fail("--lon-deg must be within -180..180"));
    }
    if !detail_scale_m.is_finite() || detail_scale_m <= 0.0 {
        return Err(fail("--detail-scale-m must be positive"));
    }
    let manifest = load_manifest(&manifest_path)?;
    validate_manifest(&manifest).map_err(fail)?;
    let knobs = bake::knobs_from_manifest(&manifest);
    let macro_h = terrain::eval_macro_m(
        &manifest.features,
        lat_deg,
        lon_deg,
        manifest.planet.datum_radius_m,
    );
    let meso_h = terrain::eval_meso_m(
        manifest.planet.seed,
        knobs,
        lat_deg,
        lon_deg,
        manifest.planet.datum_radius_m,
    );
    let micro_h = terrain::eval_micro_m(
        manifest.planet.seed,
        knobs,
        lat_deg,
        lon_deg,
        manifest.planet.datum_radius_m,
        detail_scale_m,
    );
    let site = bake::classify_site_coarse(&manifest, lat_deg, lon_deg, macro_h + meso_h);
    println!("macro_m: {macro_h:.1} meso_m: {meso_h:.1} micro_m: {micro_h:.1}");
    println!("height_m: {:.1}", macro_h + meso_h + micro_h);
    println!("site: {:?} / {:?}", site.biome, site.geology);
    Ok(())
}

fn cmd_preview(mut args: impl Iterator<Item = String>) -> Result<(), Box<dyn Error>> {
    let mut manifest_path = PathBuf::from("data/worldgen/example_rocky.toml");
    let mut step_deg: f64 = 2.0;
    let mut out = String::from("/tmp/thessa_preview");
    while let Some(arg) = args.next() {
        match arg.as_str() {
            "--manifest" => {
                manifest_path = PathBuf::from(
                    args.next()
                        .ok_or_else(|| fail("--manifest requires a value"))?,
                );
            }
            "--step-deg" => {
                step_deg = args
                    .next()
                    .ok_or_else(|| fail("--step-deg requires a value"))?
                    .parse()?
            }
            "--out" => out = args.next().ok_or_else(|| fail("--out requires a value"))?,
            unknown => return Err(fail(format!("unknown argument {unknown}"))),
        }
    }
    let manifest = load_manifest(&manifest_path)?;
    validate_manifest(&manifest).map_err(fail)?;
    let (h, b) = preview::render_preview(&manifest, step_deg, &out).map_err(fail)?;
    println!("wrote: {h}");
    println!("wrote: {b}");
    Ok(())
}

fn cmd_bake(mut args: impl Iterator<Item = String>) -> Result<(), Box<dyn Error>> {
    let mut manifest_path = PathBuf::from("data/worldgen/example_rocky.toml");
    let mut step_deg: f64 = 2.0;
    let mut out: Option<PathBuf> = None;
    while let Some(arg) = args.next() {
        match arg.as_str() {
            "--manifest" => {
                manifest_path = PathBuf::from(
                    args.next()
                        .ok_or_else(|| fail("--manifest requires a value"))?,
                );
            }
            "--step-deg" => {
                step_deg = args
                    .next()
                    .ok_or_else(|| fail("--step-deg requires a value"))?
                    .parse()?
            }
            "--out" => {
                out = Some(PathBuf::from(
                    args.next().ok_or_else(|| fail("--out requires a value"))?,
                ))
            }
            unknown => return Err(fail(format!("unknown argument {unknown}"))),
        }
    }
    let manifest = load_manifest(&manifest_path)?;
    validate_manifest(&manifest).map_err(fail)?;
    let report = bake::bake_report(&manifest, step_deg).map_err(fail)?;
    println!(
        "cells: {} ocean: {} lakes: {} rivers: {} salt: {} ice: {}",
        report.cells,
        report.ocean_cells,
        report.lake_cells,
        report.river_cells,
        report.salt_cells,
        report.ice_cells
    );
    println!(
        "height: {:.0}..{:.0} m",
        report.min_height_m, report.max_height_m
    );
    println!("mineral hotspots: {}", report.mineral_hotspots);
    let mut biomes: Vec<_> = report.biome_histogram.iter().collect();
    biomes.sort_by_key(|(_, n)| std::cmp::Reverse(**n));
    for (name, count) in biomes.iter().take(8) {
        println!("  {name:>20}: {count}");
    }
    if !report.errors.is_empty() {
        for error in &report.errors {
            println!("ERROR: {error}");
        }
        return Err(fail(format!(
            "bake failed with {} consistency errors",
            report.errors.len()
        )));
    }
    if let Some(path) = out {
        let json = serde_json::to_string_pretty(&report).map_err(|e| fail(e.to_string()))?;
        fs::write(&path, format!("{json}\n"))?;
        println!("wrote: {}", path.display());
    }
    println!("bake ok");
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    fn example() -> Manifest {
        toml::from_str(include_str!("../../../data/worldgen/example_rocky.toml"))
            .expect("example manifest parses")
    }

    #[test]
    fn example_manifest_is_valid_rocky_seven_layers() {
        let manifest = example();
        validate_manifest(&manifest).expect("example valid");
        assert_eq!(manifest.layers.len(), 7);
    }

    #[test]
    fn non_rocky_kind_is_rejected() {
        let mut manifest = example();
        manifest.planet.kind = "gas_giant".into();
        assert!(validate_manifest(&manifest).is_err());
    }

    #[test]
    fn gores_cover_full_longitude_range() {
        use thessa_worldgen_rocky::gores::GoreLayout;
        let layout = GoreLayout {
            count: 8,
            overlap_fraction: 0.06,
        };
        layout.validate().unwrap();
        let mut owners = std::collections::HashSet::new();
        for i in 0..800 {
            owners.insert(layout.owner_for_lon01(i as f64 / 800.0));
        }
        assert_eq!(owners.len(), 8);
    }

    #[test]
    fn detail_is_deterministic_and_scale_aware() {
        let manifest = example();
        let knobs = bake::knobs_from_manifest(&manifest);
        let a = terrain::eval_micro_m(7, knobs, 12.0, -40.0, 3_200_000.0, 250.0);
        let b = terrain::eval_micro_m(7, knobs, 12.0, -40.0, 3_200_000.0, 250.0);
        assert_eq!(a, b);
        let c = terrain::eval_micro_m(8, knobs, 12.0, -40.0, 3_200_000.0, 250.0);
        assert!((a - c).abs() > 1e-9);
    }

    #[test]
    fn gore_for_longitude_helper_still_consistent() {
        use thessa_worldgen_rocky::gores::GoreLayout;
        assert_eq!(
            GoreLayout {
                count: 8,
                overlap_fraction: 0.0
            }
            .owner_for_lon01(0.999),
            7
        );
    }
}
