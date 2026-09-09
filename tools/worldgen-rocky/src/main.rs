//! Thessa rocky-planet worldgen dev tool (MIT, dev-only).
//!
//! Authored macrostructure + deterministic geology/biome logic + explicit
//! landmark generators + procedural physical-scale detail. Resolution-agnostic:
//! all scales in metres, gores as normalized fractions.

use thessa_worldgen_rocky::{
    bake, client_export, field, geothermal, manifest, preview, spec_recipe, system_body,
};

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
        fail("usage: thessa-worldgen-rocky <check|sample|sample-lods|list-prompts|preview|bake|bake-spec|list-landmarks|check-landmarks> ...")
    })?;
    match command.as_str() {
        "check" => cmd_check(args),
        "sample" => cmd_sample(args),
        "sample-lods" => cmd_sample_lods(args),
        "list-landmarks" => cmd_list_landmarks(args),
        "check-landmarks" => cmd_check_landmarks(args),
        "list-prompts" => {
            for layer in CANONICAL_LAYERS {
                println!("prompts/{layer}.md");
            }
            Ok(())
        }
        "preview" => cmd_preview(args),
        "bake" => cmd_bake(args),
        "bake-spec" => cmd_bake_spec(args),
        "export-client-texture" => cmd_export_client_texture(args),
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
        "  thessa-worldgen-rocky check --system <TOML> --body <id> --recipe <TOML> [--override-radius]"
    );
    println!(
        "  thessa-worldgen-rocky sample --manifest <TOML> --lat-deg 12 --lon-deg -40 --min-wavelength-m 250"
    );
    println!(
        "  thessa-worldgen-rocky sample --system <TOML> --body <id> --manifest <TOML> --lat-deg 12 --lon-deg -40"
    );
    println!(
        "  thessa-worldgen-rocky sample-lods --manifest <TOML> --lat-deg 12 --lon-deg -40 --wavelengths 64000,8000,1000,100"
    );
    println!("  thessa-worldgen-rocky list-landmarks --manifest <TOML>");
    println!("  thessa-worldgen-rocky check-landmarks --manifest <TOML>");
    println!(
        "  thessa-worldgen-rocky export-client-texture --manifest <TOML> --out <PNG> [--width 2048 --min-wavelength-m 2000]"
    );
    println!(
        "  thessa-worldgen-rocky export-client-texture --recipe <TOML> --body-file <TOML> --out <PNG>"
    );
    println!("  thessa-worldgen-rocky list-prompts");
    println!("  thessa-worldgen-rocky preview --manifest <TOML> --step-deg 2 --out /tmp/pv");
    println!("  thessa-worldgen-rocky bake --manifest <TOML> --step-deg 2 [--out report.json]");
    println!(
        "  thessa-worldgen-rocky bake-spec --recipe <TOML> --body <TOML> --out-dir /tmp/spec [--map 1920x1080]"
    );
}

fn load_manifest(path: &PathBuf) -> Result<Manifest, Box<dyn Error>> {
    let source = fs::read_to_string(path)?;
    Ok(toml::from_str(&source)?)
}

fn cmd_check(mut args: impl Iterator<Item = String>) -> Result<(), Box<dyn Error>> {
    let mut manifest_path = PathBuf::from("data/worldgen/example_rocky.toml");
    let mut system: Option<PathBuf> = None;
    let mut body: Option<String> = None;
    let mut recipe: Option<PathBuf> = None;
    let mut allow_override = false;
    while let Some(arg) = args.next() {
        match arg.as_str() {
            "--manifest" => {
                manifest_path = PathBuf::from(
                    args.next()
                        .ok_or_else(|| fail("--manifest requires a value"))?,
                );
            }
            "--system" => {
                system = Some(PathBuf::from(
                    args.next()
                        .ok_or_else(|| fail("--system requires a value"))?,
                ));
            }
            "--body" => {
                body = Some(args.next().ok_or_else(|| fail("--body requires a value"))?);
            }
            "--recipe" => {
                recipe = Some(PathBuf::from(
                    args.next()
                        .ok_or_else(|| fail("--recipe requires a value"))?,
                ));
            }
            "--override-radius" => allow_override = true,
            unknown => return Err(fail(format!("unknown argument {unknown}"))),
        }
    }
    // Spec mode: body from the system design + worldgen recipe.
    if system.is_some() || recipe.is_some() {
        let system_path = system.ok_or_else(|| fail("check needs --system with --recipe"))?;
        let body_id = body.ok_or_else(|| fail("check needs --body with --recipe"))?;
        let recipe_path = recipe.ok_or_else(|| fail("check needs --recipe with --system"))?;
        let spec: spec_recipe::SpecRecipe = toml::from_str(&fs::read_to_string(&recipe_path)?)?;
        let sys_source = fs::read_to_string(&system_path)?;
        let resolved = system_body::resolve_body(&sys_source, &body_id).map_err(fail)?;
        system_body::check_radius_agreement(spec.planet.datum_radius_m, &resolved, allow_override)
            .map_err(fail)?;
        let placed = spec_recipe::place_spec_features(&spec).map_err(fail)?;
        println!(
            "body: {} ({:?}, radius {:.0} km, host {})",
            resolved.entry.id,
            resolved.kind,
            resolved.radius_m().map_err(fail)? / 1000.0,
            resolved.entry.host.as_deref().unwrap_or("-"),
        );
        println!("recipe features placed: {}", placed.len());
        println!("ok: body + recipe agree");
        return Ok(());
    }
    let mut manifest = load_manifest(&manifest_path)?;
    if let Some(resolved) = resolve_system_body(&system, &body, &mut manifest, allow_override)? {
        println!(
            "body: {} ({:?}, radius {:.0} km)",
            resolved.entry.id,
            resolved.kind,
            resolved.radius_m().map_err(fail)? / 1000.0,
        );
    }
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
    let mut system: Option<PathBuf> = None;
    let mut body: Option<String> = None;
    let mut allow_override = false;
    let mut lat_deg: f64 = 0.0;
    let mut lon_deg: f64 = 0.0;
    let mut min_wavelength_m: f64 = 250.0;
    while let Some(arg) = args.next() {
        match arg.as_str() {
            "--manifest" => {
                manifest_path = PathBuf::from(
                    args.next()
                        .ok_or_else(|| fail("--manifest requires a value"))?,
                );
            }
            "--system" => {
                system = Some(PathBuf::from(
                    args.next()
                        .ok_or_else(|| fail("--system requires a value"))?,
                ));
            }
            "--body" => {
                body = Some(args.next().ok_or_else(|| fail("--body requires a value"))?);
            }
            "--override-radius" => allow_override = true,
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
            "--detail-scale-m" | "--min-wavelength-m" => {
                min_wavelength_m = args
                    .next()
                    .ok_or_else(|| fail("--min-wavelength-m requires a value"))?
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
    if !min_wavelength_m.is_finite() || min_wavelength_m <= 0.0 {
        return Err(fail("--min-wavelength-m must be positive"));
    }
    let mut manifest = load_manifest(&manifest_path)?;
    if let Some(resolved) = resolve_system_body(&system, &body, &mut manifest, allow_override)? {
        println!(
            "body: {} ({:?}, g from {:.4} M_earth)",
            resolved.entry.id,
            resolved.kind,
            resolved.entry.mass_earth.unwrap_or(f64::NAN),
        );
    }
    validate_manifest(&manifest).map_err(fail)?;
    // SAME global field as bake/preview: point query, no tiles involved.
    let field = field::field_from_manifest(&manifest).map_err(fail)?;
    let dir = thessa_worldgen_rocky::sphere::dir_from_latlon(lat_deg, lon_deg);
    let s = field.sample(dir, min_wavelength_m);
    println!(
        "height_m: {:.1}  (macro {:.1} + procedural {:.1})",
        s.height_m, s.macro_height_m, s.procedural_height_m
    );
    println!(
        "site: {:?} / {:?}  tags {:?} {:?}",
        s.biome, s.geology, s.tag0, s.tag1
    );
    println!(
        "slope {:.4}  geothermal {:.3} W/m2  moisture {:.2}  continentality {:.2}  eclipse {:.2}",
        s.slope_hint,
        s.geothermal_flux_w_m2,
        s.moisture01,
        s.continentality01,
        s.eclipse_exposure01,
    );
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

fn cmd_bake_spec(mut args: impl Iterator<Item = String>) -> Result<(), Box<dyn Error>> {
    let mut recipe_path = PathBuf::from("data/worldgen/worldgen_recipe.toml");
    let mut body_path = PathBuf::from("data/worldgen/thessa_v02.toml");
    let mut system: Option<PathBuf> = None;
    let mut body_id: Option<String> = None;
    let mut allow_override = false;
    let mut out_dir = PathBuf::from("/tmp/thessa_spec");
    let mut map: Option<(usize, usize)> = None;
    while let Some(arg) = args.next() {
        match arg.as_str() {
            "--recipe" => {
                recipe_path = PathBuf::from(
                    args.next()
                        .ok_or_else(|| fail("--recipe requires a value"))?,
                );
            }
            "--body-file" => {
                body_path = PathBuf::from(
                    args.next()
                        .ok_or_else(|| fail("--body-file requires a value"))?,
                );
            }
            "--system" => {
                system = Some(PathBuf::from(
                    args.next()
                        .ok_or_else(|| fail("--system requires a value"))?,
                ));
            }
            "--body" => {
                body_id = Some(args.next().ok_or_else(|| fail("--body requires a value"))?);
            }
            "--override-radius" => allow_override = true,
            "--out-dir" => {
                out_dir = PathBuf::from(
                    args.next()
                        .ok_or_else(|| fail("--out-dir requires a value"))?,
                );
            }
            "--map" => {
                let size = args.next().ok_or_else(|| fail("--map requires WxH"))?;
                let (w, h) = size
                    .split_once('x')
                    .ok_or_else(|| fail("--map must look like 1920x1080"))?;
                map = Some((
                    w.parse().map_err(|_| fail("bad map width"))?,
                    h.parse().map_err(|_| fail("bad map height"))?,
                ));
            }
            unknown => return Err(fail(format!("unknown argument {unknown}"))),
        }
    }
    let recipe: spec_recipe::SpecRecipe = toml::from_str(&fs::read_to_string(&recipe_path)?)?;
    let body: spec_recipe::BodyFile = toml::from_str(&fs::read_to_string(&body_path)?)?;
    spec_recipe::validate_spec(&recipe, &body).map_err(fail)?;
    if let (Some(system_path), Some(id)) = (system, body_id.clone()) {
        // Canonical body from the system design wins over the reference file.
        let sys_source = fs::read_to_string(&system_path)?;
        let resolved = system_body::resolve_body(&sys_source, &id).map_err(fail)?;
        system_body::require_rocky(&resolved).map_err(fail)?;
        system_body::check_radius_agreement(
            recipe.planet.datum_radius_m,
            &resolved,
            allow_override,
        )
        .map_err(fail)?;
        println!(
            "body: {} ({:?}, radius {:.0} km)",
            resolved.entry.id,
            resolved.kind,
            resolved.radius_m().map_err(fail)? / 1000.0,
        );
    } else if body_id.is_some() || allow_override {
        return Err(fail(
            "--system/--body/--override-radius must be combined consistently",
        ));
    }
    let manifest = spec_recipe::manifest_from_spec(&recipe).map_err(fail)?;
    validate_manifest(&manifest).map_err(fail)?;
    std::fs::create_dir_all(&out_dir)?;
    let prefix = out_dir
        .join(&manifest.planet.name)
        .to_string_lossy()
        .into_owned();

    // Report at dev resolution.
    let report = bake::bake_report(&manifest, 2.0).map_err(fail)?;
    println!(
        "cells: {} ocean: {} lakes: {} rivers: {} salt: {} ice: {}",
        report.cells,
        report.ocean_cells,
        report.lake_cells,
        report.river_cells,
        report.salt_cells,
        report.ice_cells
    );
    if !report.errors.is_empty() {
        for error in &report.errors {
            println!("ERROR: {error}");
        }
        return Err(fail("bake-spec consistency errors"));
    }

    // Geothermal provinces prefer volcanic/rift landmarks.
    let hot_spots: Vec<(f64, f64)> = manifest
        .features
        .iter()
        .filter_map(|f| {
            use thessa_worldgen_rocky::features::Feature;
            match &f.feature {
                Feature::VolcanicProvince { .. }
                | Feature::ShieldVolcano { .. }
                | Feature::Canyon { .. } => Some((f.lat_deg, f.lon_deg)),
                _ => None,
            }
        })
        .collect();
    let (hlats, hlons): (Vec<f64>, Vec<f64>) = hot_spots.iter().cloned().unzip();
    let major = recipe.geothermal.major_provinces_min
        + (thessa_worldgen_rocky::rng::hash01(manifest.planet.seed, 600, 0, 0)
            * (recipe.geothermal.major_provinces_max - recipe.geothermal.major_provinces_min + 1)
                as f64) as u32;
    let secondary = recipe.geothermal.secondary_fields_min
        + (thessa_worldgen_rocky::rng::hash01(manifest.planet.seed, 601, 0, 0)
            * (recipe.geothermal.secondary_fields_max - recipe.geothermal.secondary_fields_min + 1)
                as f64) as u32;
    let provinces =
        geothermal::place_provinces(manifest.planet.seed, major, secondary, &hlats, &hlons);
    println!("geothermal: {major} major + {secondary} secondary provinces");

    // 480x270 diagnostic previews (spec acceptance size).
    let grid = bake::evaluate_height_grid_steps(&manifest, 2.0 / 3.0, 0.75).map_err(fail)?;
    let water = bake::classify_water_driven(&grid, &manifest);
    let (h, b) =
        preview::render_preview_steps(&manifest, 2.0 / 3.0, 0.75, &prefix).map_err(fail)?;
    println!("wrote: {h}");
    println!("wrote: {b}");
    let overlays = preview::OverlayInputs {
        provinces,
        eclipse_strength: 0.28,
    };
    for path in
        preview::render_spec_overlays(&manifest, &grid, &water, &overlays, &prefix).map_err(fail)?
    {
        println!("wrote: {path}");
    }

    // Readability gate: landmarks must survive at 480x270.
    let readability = preview::readability_480x270(&manifest).map_err(fail)?;
    println!("readability: {readability:?}");
    if !readability.passes() {
        return Err(fail("readability gate failed at 480x270"));
    }

    // Optional runtime-size global map (macro + large meso only).
    if let Some((w, h)) = map {
        if w == 0 || h == 0 || w > 8192 || h > 8192 {
            return Err(fail("--map dimensions must be within 1..=8192"));
        }
        let map_prefix = format!("{prefix}_map-{w}x{h}");
        let (mh, mb) = preview::render_preview_steps(
            &manifest,
            180.0 / h as f64,
            360.0 / w as f64,
            &map_prefix,
        )
        .map_err(fail)?;
        println!("wrote: {mh}");
        println!("wrote: {mb}");
    }
    println!("bake-spec ok");
    Ok(())
}

/// Resolve `--system/--body` to a rocky body and align the manifest radius.
/// Returns the body for callers that print inherited metadata.
fn resolve_system_body(
    system: &Option<PathBuf>,
    body_id: &Option<String>,
    manifest: &mut Manifest,
    allow_override: bool,
) -> Result<Option<system_body::ResolvedBody>, Box<dyn Error>> {
    let (system_path, body) = match (system, body_id) {
        (Some(path), Some(id)) => (path, id),
        (None, None) => return Ok(None),
        _ => return Err(fail("--system and --body must be given together")),
    };
    let source = fs::read_to_string(system_path)?;
    let resolved = system_body::resolve_body(&source, body).map_err(fail)?;
    let radius_m = system_body::require_rocky(&resolved).map_err(fail)?;
    system_body::check_radius_agreement(manifest.planet.datum_radius_m, &resolved, allow_override)
        .map_err(fail)?;
    if !allow_override {
        // Inherit canonical radius unless an explicit override was stated.
        manifest.planet.datum_radius_m = radius_m;
    }
    Ok(Some(resolved))
}

fn cmd_sample_lods(mut args: impl Iterator<Item = String>) -> Result<(), Box<dyn Error>> {
    let mut manifest_path = PathBuf::from("data/worldgen/example_rocky.toml");
    let mut lat_deg: f64 = 0.0;
    let mut lon_deg: f64 = 0.0;
    let mut wavelengths = String::from("64000,8000,1000,100");
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
            "--wavelengths" => {
                wavelengths = args
                    .next()
                    .ok_or_else(|| fail("--wavelengths requires a value"))?
            }
            unknown => return Err(fail(format!("unknown argument {unknown}"))),
        }
    }
    let manifest = load_manifest(&manifest_path)?;
    validate_manifest(&manifest).map_err(fail)?;
    let field = field::field_from_manifest(&manifest).map_err(fail)?;
    let dir = thessa_worldgen_rocky::sphere::dir_from_latlon(lat_deg, lon_deg);
    for wl in wavelengths.split(',') {
        let min_wl: f64 = wl.trim().parse().map_err(|_| fail("bad wavelength"))?;
        let s = field.sample(dir, min_wl);
        println!(
            "min_wl {min_wl:>8.0} m  height {sheight:>10.1} m",
            sheight = s.height_m
        );
    }
    Ok(())
}

fn cmd_list_landmarks(mut args: impl Iterator<Item = String>) -> Result<(), Box<dyn Error>> {
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
    if manifest.landmark_zones.is_empty() {
        println!("no landmark zones");
        return Ok(());
    }
    for zone in &manifest.landmark_zones {
        println!(
            "{id}  {mode:?}  center ({lat:.3}, {lon:.3})  radius {r:.0} m  blend {b:.0} m",
            id = zone.id,
            mode = zone.mode,
            lat = zone.center_lat_deg,
            lon = zone.center_lon_deg,
            r = zone.radius_m,
            b = zone.blend_width_m,
        );
    }
    Ok(())
}

fn cmd_check_landmarks(mut args: impl Iterator<Item = String>) -> Result<(), Box<dyn Error>> {
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
    let field = field::field_from_manifest(&manifest).map_err(fail)?;
    for zone in &manifest.landmark_zones {
        // ENU round-trip at 5 km offset must return to centimetres.
        let dir = thessa_worldgen_rocky::sphere::dir_from_latlon(
            zone.center_lat_deg + 0.05,
            zone.center_lon_deg + 0.05,
        );
        let enu = zone.to_local(dir, manifest.planet.datum_radius_m);
        let back = zone.to_world(enu, manifest.planet.datum_radius_m);
        let err_m = thessa_worldgen_rocky::sphere::great_circle_m(
            dir,
            back,
            manifest.planet.datum_radius_m,
        );
        // Blend continuity: sample weights across the ring, must be monotone.
        let mut prev = 2.0;
        let mut monotone = true;
        for i in 0..=20 {
            let d = zone.radius_m + zone.blend_width_m * i as f64 / 20.0;
            let ring_dir = zone.to_world([d, 0.0, 0.0], manifest.planet.datum_radius_m);
            let w = zone.blend_weight(ring_dir, manifest.planet.datum_radius_m);
            if w > prev + 1e-9 {
                monotone = false;
            }
            prev = w;
        }
        // Same base terrain with and without the zone differs only by delta.
        let s = field.sample(zone.center_dir(), 100.0);
        println!(
            "{id}: enu_roundtrip {err_m:.4} m  blend_monotone {monotone}  center_height {h:.1} m",
            id = zone.id,
            h = s.height_m,
        );
        if err_m > 0.05 || !monotone {
            return Err(fail(format!("landmark {} failed checks", zone.id)));
        }
    }
    println!("landmarks ok: {}", manifest.landmark_zones.len());
    Ok(())
}

fn cmd_export_client_texture(mut args: impl Iterator<Item = String>) -> Result<(), Box<dyn Error>> {
    let mut manifest_path: Option<PathBuf> = None;
    let mut recipe_path: Option<PathBuf> = None;
    let mut body_path = PathBuf::from("data/worldgen/thessa_v02.toml");
    let mut out: Option<PathBuf> = None;
    let mut width: usize = 2048;
    let mut height: usize = 1024;
    let mut min_wl: f64 = 2000.0;
    while let Some(arg) = args.next() {
        match arg.as_str() {
            "--manifest" => {
                manifest_path = Some(PathBuf::from(
                    args.next()
                        .ok_or_else(|| fail("--manifest requires a value"))?,
                ));
            }
            "--recipe" => {
                recipe_path = Some(PathBuf::from(
                    args.next()
                        .ok_or_else(|| fail("--recipe requires a value"))?,
                ));
            }
            "--body-file" => {
                body_path = PathBuf::from(
                    args.next()
                        .ok_or_else(|| fail("--body-file requires a value"))?,
                );
            }
            "--out" => {
                out = Some(PathBuf::from(
                    args.next().ok_or_else(|| fail("--out requires a value"))?,
                ));
            }
            "--width" => {
                width = args
                    .next()
                    .ok_or_else(|| fail("--width requires a value"))?
                    .parse()?;
            }
            "--height" => {
                height = args
                    .next()
                    .ok_or_else(|| fail("--height requires a value"))?
                    .parse()?;
            }
            "--min-wavelength-m" => {
                min_wl = args
                    .next()
                    .ok_or_else(|| fail("--min-wavelength-m requires a value"))?
                    .parse()?;
            }
            unknown => return Err(fail(format!("unknown argument {unknown}"))),
        }
    }
    let out = out.ok_or_else(|| fail("--out is required"))?;
    let manifest = match (manifest_path, recipe_path) {
        (Some(path), None) => load_manifest(&path)?,
        (None, Some(recipe)) => {
            let spec: spec_recipe::SpecRecipe = toml::from_str(&fs::read_to_string(&recipe)?)?;
            let body: spec_recipe::BodyFile = toml::from_str(&fs::read_to_string(&body_path)?)?;
            spec_recipe::validate_spec(&spec, &body).map_err(fail)?;
            spec_recipe::manifest_from_spec(&spec).map_err(fail)?
        }
        _ => return Err(fail("give exactly one of --manifest or --recipe")),
    };
    validate_manifest(&manifest).map_err(fail)?;
    let field = field::field_from_manifest(&manifest).map_err(fail)?;
    if let Some(parent) = out.parent().filter(|p| !p.as_os_str().is_empty()) {
        std::fs::create_dir_all(parent)?;
    }
    let file = std::fs::File::create(&out)?;
    let writer = std::io::BufWriter::new(file);
    client_export::write_client_texture_png(&field, width, height, min_wl, writer).map_err(fail)?;
    println!("wrote: {} ({width}x{height})", out.display());
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
        use thessa_worldgen_rocky::terrain;
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
