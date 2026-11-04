//! Emit canonical land pages for fixed-source normal/filtering comparisons.
//! Usage: ground_page_probe NEW_DIRECTORY [FACE LEVEL X Y]...
use std::{error::Error, path::PathBuf, time::Instant};
use thessa_worldgen_rocky::{
    field,
    lod::{self, TileKey},
    offline::FrozenSurface,
    spec_recipe,
};
fn main() -> Result<(), Box<dyn Error>> {
    let mut args = std::env::args().skip(1);
    let out = PathBuf::from(args.next().ok_or("requires a new output directory")?);
    std::fs::create_dir(&out)?;
    let mut keys = Vec::new();
    while let Some(face) = args.next() {
        keys.push(TileKey {
            face: face.parse()?,
            level: args.next().ok_or("level")?.parse()?,
            x: args.next().ok_or("x")?.parse()?,
            y: args.next().ok_or("y")?.parse()?,
        });
    }
    if keys.is_empty() {
        return Err("requires explicit FACE LEVEL X Y tile arguments".into());
    }
    let recipe = toml::from_str(include_str!("../../../data/worldgen/worldgen_recipe.toml"))?;
    let source = field::field_from_manifest(&spec_recipe::manifest_from_spec(&recipe)?)?;
    let frozen = FrozenSurface::read_gzip(
        &include_bytes!("../../../data/worldgen/thessa-erosion-v3.surface.json.gz")[..],
    )?;
    let field = source.with_frozen_erosion(&frozen)?;
    let mut rows = Vec::new();
    for (i, key) in keys.into_iter().enumerate() {
        if key.face >= 6 || key.level > 25 || key.x >= 1 << key.level || key.y >= 1 << key.level {
            return Err("invalid tile".into());
        }
        let start = Instant::now();
        let page = lod::build_gpu_material_page_with_relief(&field, key, true, true, true);
        let ms = start.elapsed().as_secs_f64() * 1000.;
        for (suffix, bytes) in [
            ("rgba", &page.rgba),
            ("normal", &page.normal),
            ("base-normal", &page.base_normal),
            ("water", &page.water_coverage),
        ] {
            std::fs::write(out.join(format!("{i}-{suffix}.bin")), bytes)?;
        }
        let preview: Vec<u8> = page
            .normal
            .as_chunks::<2>()
            .0
            .iter()
            .flat_map(|p| {
                let [x, y] =
                    p.map(thessa_rcbt_core::material_page::decode_material_normal_component);
                [x, y, (1. - x * x - y * y).max(0.).sqrt()]
                    .map(|v| ((v * 0.5 + 0.5) * 255.).round() as u8)
            })
            .collect();
        for (suffix, data, color) in [
            ("albedo", page.rgba, png::ColorType::Rgba),
            ("normal", preview, png::ColorType::Rgb),
        ] {
            let mut encoder = png::Encoder::new(
                std::fs::File::create(out.join(format!("{i}-{suffix}.png")))?,
                128,
                128,
            );
            encoder.set_color(color);
            encoder.set_depth(png::BitDepth::Eight);
            encoder.write_header()?.write_image_data(&data)?;
        }
        rows.push(serde_json::json!({"key":key,"physical_texel_m":key.span_m(field.params.radius_m)/125.,"cpu_build_ms":ms,"first_page_may_include_lazy_scan_decode":i==0}));
    }
    std::fs::write(out.join("pages.json"), serde_json::to_vec_pretty(&rows)?)?;
    println!("{}", serde_json::to_string(&rows)?);
    Ok(())
}
