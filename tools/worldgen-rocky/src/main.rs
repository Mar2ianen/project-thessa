//! Thessa rocky-planet worldgen dev tool.
//!
//! Dev-only helper, not runtime: validates a resolution-agnostic manifest for
//! GPT-image-generated base maps (orange-slice gores) plus deterministic
//! procedural micro-detail. Rocky planets only, no clouds.

use std::{env, error::Error, fmt, fs, path::PathBuf};

use serde::{Deserialize, Serialize};

/// Canonical layer set. Exactly these 7 files per planet.
pub const CANONICAL_LAYERS: [&str; 7] = [
    "height",
    "albedo",
    "biomes",
    "roughness",
    "normal",
    "hydrology",
    "minerals",
];

#[derive(Debug, Clone, Deserialize, Serialize)]
struct Manifest {
    planet: Planet,
    gores: Gores,
    layers: Vec<Layer>,
}

#[derive(Debug, Clone, Deserialize, Serialize)]
struct Planet {
    name: String,
    kind: String,
    datum_radius_m: f64,
    height_min_m: f64,
    height_max_m: f64,
    seed: u64,
}

#[derive(Debug, Clone, Deserialize, Serialize)]
struct Gores {
    /// Number of orange-slice segments around the equator. No pixels here,
    /// only normalized fractions; source resolution can be upgraded later.
    count: u32,
    /// Fractional overlap between neighbours, 0..0.25, for seamless stitch.
    overlap_fraction: f64,
    /// Whether polar caps are generated as separate images.
    polar_caps: bool,
}

#[derive(Debug, Clone, Deserialize, Serialize)]
struct Layer {
    name: String,
    file: String,
    prompt: String,
}

#[derive(Debug)]
enum WorldgenError {
    Message(String),
}

impl fmt::Display for WorldgenError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Message(m) => write!(f, "{m}"),
        }
    }
}

impl Error for WorldgenError {}

fn fail(message: impl Into<String>) -> Box<dyn Error> {
    Box::new(WorldgenError::Message(message.into()))
}

fn main() -> Result<(), Box<dyn Error>> {
    let mut args = env::args().skip(1).peekable();
    let command = args.next().ok_or_else(|| {
        fail("usage: thessa-worldgen-rocky <check|list-prompts|sample> [--manifest PATH] ...")
    })?;
    match command.as_str() {
        "check" => {
            let options = CheckOptions::parse(args)?;
            let manifest = load_manifest(&options.manifest)?;
            validate_manifest(&manifest)?;
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
            println!("ok: resolution-agnostic, no pixel constants pinned");
            Ok(())
        }
        "list-prompts" => {
            for layer in CANONICAL_LAYERS {
                println!("prompts/{layer}.md");
            }
            Ok(())
        }
        "sample" => {
            let options = SampleOptions::parse(args)?;
            let manifest = load_manifest(&options.manifest)?;
            validate_manifest(&manifest)?;
            let height_m = sample_height_detail(
                manifest.planet.seed,
                options.lat_deg,
                options.lon_deg,
                options.base_height_m,
                options.detail_scale_m,
            )?;
            println!("height_detail_m: {height_m:.3}");
            Ok(())
        }
        "--help" | "-h" | "help" => {
            print_help();
            Ok(())
        }
        unknown => Err(fail(format!("unknown command {unknown}; use --help"))),
    }
}

fn print_help() {
    println!("Usage:");
    println!("  thessa-worldgen-rocky check --manifest data/worldgen/example_rocky.toml");
    println!("  thessa-worldgen-rocky list-prompts");
    println!(
        "  thessa-worldgen-rocky sample --manifest <TOML> --lat-deg 12 --lon-deg -40 --base-height-m 800 --detail-scale-m 250"
    );
}

struct CheckOptions {
    manifest: PathBuf,
}

impl CheckOptions {
    fn parse(mut args: impl Iterator<Item = String>) -> Result<Self, Box<dyn Error>> {
        let mut manifest = PathBuf::from("data/worldgen/example_rocky.toml");
        while let Some(arg) = args.next() {
            match arg.as_str() {
                "--manifest" => {
                    manifest = PathBuf::from(
                        args.next()
                            .ok_or_else(|| fail("--manifest requires a value"))?,
                    );
                }
                unknown => return Err(fail(format!("unknown argument {unknown}"))),
            }
        }
        Ok(Self { manifest })
    }
}

struct SampleOptions {
    manifest: PathBuf,
    lat_deg: f64,
    lon_deg: f64,
    base_height_m: f64,
    detail_scale_m: f64,
}

impl SampleOptions {
    fn parse(mut args: impl Iterator<Item = String>) -> Result<Self, Box<dyn Error>> {
        let mut manifest = PathBuf::from("data/worldgen/example_rocky.toml");
        let mut lat_deg: f64 = 0.0;
        let mut lon_deg: f64 = 0.0;
        let mut base_height_m: f64 = 0.0;
        let mut detail_scale_m: f64 = 250.0;
        while let Some(arg) = args.next() {
            match arg.as_str() {
                "--manifest" => {
                    manifest = PathBuf::from(
                        args.next()
                            .ok_or_else(|| fail("--manifest requires a value"))?,
                    );
                }
                "--lat-deg" => {
                    lat_deg = args
                        .next()
                        .ok_or_else(|| fail("--lat-deg requires a value"))?
                        .parse()?;
                }
                "--lon-deg" => {
                    lon_deg = args
                        .next()
                        .ok_or_else(|| fail("--lon-deg requires a value"))?
                        .parse()?;
                }
                "--base-height-m" => {
                    base_height_m = args
                        .next()
                        .ok_or_else(|| fail("--base-height-m requires a value"))?
                        .parse()?;
                }
                "--detail-scale-m" => {
                    detail_scale_m = args
                        .next()
                        .ok_or_else(|| fail("--detail-scale-m requires a value"))?
                        .parse()?;
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
        if !base_height_m.is_finite() || !detail_scale_m.is_finite() || detail_scale_m <= 0.0 {
            return Err(fail("heights must be finite and detail scale positive"));
        }
        Ok(Self {
            manifest,
            lat_deg,
            lon_deg,
            base_height_m,
            detail_scale_m,
        })
    }
}

fn load_manifest(path: &PathBuf) -> Result<Manifest, Box<dyn Error>> {
    let source = fs::read_to_string(path)?;
    Ok(toml::from_str(&source)?)
}

fn validate_manifest(manifest: &Manifest) -> Result<(), Box<dyn Error>> {
    if manifest.planet.kind.trim().to_lowercase() != "rocky" {
        return Err(fail(format!(
            "planet kind must be \"rocky\" for this tool, got {:?} (gas/ice giants go elsewhere)",
            manifest.planet.kind
        )));
    }
    if manifest.planet.name.trim().is_empty() {
        return Err(fail("planet name must not be empty"));
    }
    if !manifest.planet.datum_radius_m.is_finite() || manifest.planet.datum_radius_m <= 0.0 {
        return Err(fail("datum_radius_m must be finite and positive"));
    }
    if !manifest.planet.height_min_m.is_finite()
        || !manifest.planet.height_max_m.is_finite()
        || manifest
            .planet
            .height_min_m
            .partial_cmp(&manifest.planet.height_max_m)
            != Some(std::cmp::Ordering::Less)
    {
        return Err(fail("need height_min_m < height_max_m, both finite"));
    }
    if !(4..=16).contains(&manifest.gores.count) {
        return Err(fail("gores.count must be 4..=16 orange slices"));
    }
    if !manifest.gores.overlap_fraction.is_finite()
        || !(0.0..=0.25).contains(&manifest.gores.overlap_fraction)
    {
        return Err(fail("gores.overlap_fraction must be within 0..=0.25"));
    }
    if manifest.layers.len() != CANONICAL_LAYERS.len() {
        return Err(fail(format!(
            "need exactly {} layers, got {}",
            CANONICAL_LAYERS.len(),
            manifest.layers.len()
        )));
    }
    for expected in CANONICAL_LAYERS {
        let found = manifest.layers.iter().any(|l| l.name == expected);
        if !found {
            return Err(fail(format!("missing canonical layer {expected:?}")));
        }
    }
    for layer in &manifest.layers {
        if layer.file.trim().is_empty() || layer.prompt.trim().is_empty() {
            return Err(fail(format!(
                "layer {:?} needs file and prompt",
                layer.name
            )));
        }
    }
    Ok(())
}

/// Which orange-slice gore owns a longitude, in normalized [0,1) space.
/// Resolution-independent: works for any source pixel size.
pub fn gore_for_longitude(lon_deg: f64, gore_count: u32) -> u32 {
    let wrapped = (lon_deg + 180.0).rem_euclid(360.0) / 360.0;
    ((wrapped * f64::from(gore_count)).floor() as u32).min(gore_count - 1)
}

/// Deterministic procedural micro-detail, resolution-agnostic.
///
/// `detail_scale_m` is a physical wavelength, not pixels, so the same manifest
/// stays valid when the GPT base maps are regenerated at higher resolution.
pub fn sample_height_detail(
    seed: u64,
    lat_deg: f64,
    lon_deg: f64,
    base_height_m: f64,
    detail_scale_m: f64,
) -> Result<f64, Box<dyn Error>> {
    if !lat_deg.is_finite()
        || !lon_deg.is_finite()
        || !base_height_m.is_finite()
        || !detail_scale_m.is_finite()
        || detail_scale_m <= 0.0
    {
        return Err(fail("non-finite detail input"));
    }
    // Physical-space coordinates: degrees scaled to pseudo-meters at this
    // detail wavelength. No image resolution involved.
    let x = lon_deg.to_radians().cos() * 1000.0 / detail_scale_m * 1000.0;
    let y = lat_deg.to_radians().sin() * 1000.0 / detail_scale_m * 1000.0;
    let mut amplitude = 0.35 * detail_scale_m.min(500.0);
    let mut frequency = 1.0;
    let mut detail = 0.0;
    for octave in 0..4 {
        detail += amplitude * value_noise(seed, octave, x * frequency, y * frequency);
        amplitude *= 0.5;
        frequency *= 2.03;
    }
    let out = base_height_m + detail;
    if out.is_finite() {
        Ok(out)
    } else {
        Err(fail("non-finite detail"))
    }
}

fn hash01(seed: u64, octave: u32, ix: i64, iy: i64) -> f64 {
    let mut h = seed
        .wrapping_add(u64::from(octave).wrapping_mul(0x9E37_79B9_7F4A_7C15))
        .wrapping_add((ix as u64).wrapping_mul(0xBF58_476D_1CE4_E5B9))
        .wrapping_add((iy as u64).wrapping_mul(0x94D0_49BB_1331_11EB));
    h ^= h >> 30;
    h = h.wrapping_mul(0xBF58_476D_1CE4_E5B9);
    h ^= h >> 27;
    h = h.wrapping_mul(0x94D0_49BB_1331_11EB);
    h ^= h >> 31;
    // Map to [-1, 1].
    (h as f64 / u64::MAX as f64) * 2.0 - 1.0
}

fn value_noise(seed: u64, octave: u32, x: f64, y: f64) -> f64 {
    let ix = x.floor() as i64;
    let iy = y.floor() as i64;
    let fx = x - ix as f64;
    let fy = y - iy as f64;
    let sx = fx * fx * (3.0 - 2.0 * fx);
    let sy = fy * fy * (3.0 - 2.0 * fy);
    let a = hash01(seed, octave, ix, iy);
    let b = hash01(seed, octave, ix + 1, iy);
    let c = hash01(seed, octave, ix, iy + 1);
    let d = hash01(seed, octave, ix + 1, iy + 1);
    a + (b - a) * sx + (c - a) * sy + (a - b - c + d) * sx * sy
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
        for lon in [-180.0, -90.0, 0.0, 90.0, 179.9] {
            assert!(gore_for_longitude(lon, 8) < 8);
        }
        assert_eq!(gore_for_longitude(-180.0, 8), gore_for_longitude(180.0, 8));
    }

    #[test]
    fn detail_is_deterministic_and_scale_aware() {
        let a = sample_height_detail(7, 12.0, -40.0, 800.0, 250.0).expect("sample");
        let b = sample_height_detail(7, 12.0, -40.0, 800.0, 250.0).expect("sample");
        assert!((a - b).abs() == 0.0);
        let c = sample_height_detail(8, 12.0, -40.0, 800.0, 250.0).expect("sample");
        assert!((a - c).abs() > 1e-9);
    }
}
