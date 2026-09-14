//! Legacy tiles and RCBT pages against the shipped game assets.
//! Run with `cargo bench -p thessa-worldgen-rocky --bench assets`.
//!
//! Ground truth is `assets/worlds/thessa-v3/albedo.png` (4096x2048, baked by
//! `client_export` from the same recipe family) plus the canonical field for
//! heights. Pixel mapping inverts `sample_texel` exactly:
//! `lat = 90-(r+.5)*180/h`, `lon_raw = (c+.5)*360/w`, `dir_from_latlon`.
//!
//! What this proves and what it does not:
//! - legacy tile albedo vs shipped pixels anchors the mapping (same code
//!   path should reproduce the bake up to recipe drift);
//! - heights of both paths are checked against the field, not the PNG (the
//!   PNG carries no height channel);
//! - RCBT pages are geometry-only by design, so they have no albedo column;
//!   their row is heights + build cost + payload.

use std::{hint::black_box, time::Instant};

use thessa_worldgen_rocky::{
    field::field_from_manifest,
    lod::{self, TileKey, build_surface_texture, build_tile},
    spec_recipe::{SpecRecipe, manifest_from_spec},
    sphere::dir_from_latlon,
};

const ASSET_W: usize = 4096;
const ASSET_H: usize = 2048;
const ASSET_MIN_WL: f64 = 8000.0;

fn linear(v: f32) -> f32 {
    if v <= 0.04045 {
        v / 12.92
    } else {
        ((v + 0.055) / 1.055).powf(2.4)
    }
}

/// Direction back to shipped-asset pixel. Inverse of `sample_texel`.
fn dir_to_pixel(dir: [f64; 3]) -> (usize, usize) {
    let n = (dir[0] * dir[0] + dir[1] * dir[1] + dir[2] * dir[2]).sqrt();
    let (x, y, z) = (dir[0] / n, dir[1] / n, dir[2] / n);
    let lat = y.clamp(-1.0, 1.0).asin().to_degrees();
    let lon = (-z).atan2(x).to_degrees();
    let lon_raw = if lon < 0.0 { lon + 360.0 } else { lon };
    let r = ((90.0 - lat) * ASSET_H as f64 / 180.0 - 0.5).round() as isize;
    let c = (lon_raw * ASSET_W as f64 / 360.0 - 0.5).round() as isize;
    (
        r.clamp(0, ASSET_H as isize - 1) as usize,
        c.clamp(0, ASSET_W as isize - 1) as usize,
    )
}

fn decode_albedo() -> Vec<u8> {
    let bytes = include_bytes!("../../../assets/worlds/thessa-v3/albedo.png");
    let decoder = png::Decoder::new(std::io::Cursor::new(&bytes[..]));
    let mut reader = decoder.read_info().expect("shipped albedo decodes");
    let mut buf = vec![0; reader.output_buffer_size().expect("png buffer size")];
    let info = reader.next_frame(&mut buf).expect("shipped albedo frame");
    assert_eq!(info.width as usize, ASSET_W, "shipped albedo width");
    assert_eq!(info.height as usize, ASSET_H, "shipped albedo height");
    // Normalize to RGB8 regardless of source color type.
    let rgb = match info.color_type {
        png::ColorType::Rgb => buf[..ASSET_W * ASSET_H * 3].to_vec(),
        png::ColorType::Rgba => {
            let mut out = Vec::with_capacity(ASSET_W * ASSET_H * 3);
            for px in buf.as_chunks::<4>().0.iter().take(ASSET_W * ASSET_H) {
                out.extend_from_slice(&px[..3]);
            }
            out
        }
        other => panic!("unexpected shipped albedo color type: {other:?}"),
    };
    assert_eq!(rgb.len(), ASSET_W * ASSET_H * 3);
    rgb
}

fn main() {
    let recipe: SpecRecipe =
        toml::from_str(include_str!("../../../data/worldgen/worldgen_recipe.toml")).unwrap();
    let field = field_from_manifest(&manifest_from_spec(&recipe).unwrap()).unwrap();
    println!(
        "field radius {:.0} m, seed {}",
        field.params.radius_m, field.params.seed
    );

    let t = Instant::now();
    let albedo = decode_albedo();
    println!(
        "shipped albedo decode: {:.1} ms",
        t.elapsed().as_secs_f64() * 1000.0
    );

    // 1. Sparse-pixel drift check: does the live field still reproduce the
    // shipped bake? Stride 16 keeps this at 32k field samples.
    let mut sum_sq = 0.0_f64;
    let mut count = 0_usize;
    let t = Instant::now();
    for r in (0..ASSET_H).step_by(16) {
        for c in (0..ASSET_W).step_by(16) {
            let lat = 90.0 - (r as f64 + 0.5) * (180.0 / ASSET_H as f64);
            let lon_raw = (c as f64 + 0.5) * (360.0 / ASSET_W as f64);
            let lon = if lon_raw > 180.0 {
                lon_raw - 360.0
            } else {
                lon_raw
            };
            let dir = dir_from_latlon(lat, lon);
            let s = field.sample_surface(dir, ASSET_MIN_WL);
            let mat = thessa_worldgen_rocky::appearance::surface_appearance(&field, &s, dir);
            for (k, v) in mat.albedo_srgb.iter().enumerate() {
                let want = linear(albedo[(r * ASSET_W + c) * 3 + k] as f32 / 255.0);
                let got = linear(*v);
                sum_sq += (got as f64 - want as f64).powi(2);
                count += 1;
            }
        }
    }
    let drift_ms = t.elapsed().as_secs_f64() * 1000.0;
    let rmse = (sum_sq / count as f64).sqrt();
    println!("field-vs-shipped albedo RMSE (linear, stride 16): {rmse:.4} [{drift_ms:.0} ms]");
    black_box(sum_sq);

    // 2. Real tile footprints across faces and levels.
    let mut tiles = Vec::new();
    for face in 0..6u8 {
        for level in [10u8, 12, 14] {
            let n = 1u32 << level;
            tiles.push(TileKey {
                face,
                level,
                x: n / 2,
                y: n / 3,
            });
        }
    }
    println!(
        "| tile | backend | build_ms | payload_bytes | height_maxerr_m | albedo_rmse_vs_shipped |"
    );
    println!("|---|---|---|---|---|---|");
    for key in tiles {
        // Legacy: full mesh + shipped-style texture.
        let t = Instant::now();
        let tile = build_tile(&field, key, 24);
        let tex = build_surface_texture(&field, key, lod::texture_cells_for_level(key.level));
        let legacy_ms = t.elapsed().as_secs_f64() * 1000.0;
        let payload = (tex.albedo.len() + tex.roughness.len() + tex.normal.len()) as u64;

        // Legacy heights vs field at mesh vertices (f32 rounding only).
        let wavelength = (key.span_m(field.params.radius_m) / 24.0 * 2.0).max(32.0);
        let mut height_err = 0.0_f64;
        let stride = 25_usize;
        for (i, pos) in tile.positions.iter().enumerate() {
            if i >= stride * stride {
                break; // vertex grid only, skirts excluded
            }
            let gx = tile.anchor_m[0] + pos[0] as f64;
            let gy = tile.anchor_m[1] + pos[1] as f64;
            let gz = tile.anchor_m[2] + pos[2] as f64;
            let n = (gx * gx + gy * gy + gz * gz).sqrt();
            let dir = [gx / n, gy / n, gz / n];
            let reference = field.height_m(dir, wavelength).max(0.0);
            height_err = height_err.max(((n - field.params.radius_m) - reference).abs());
        }

        // Legacy albedo vs shipped pixels at the same vertex directions.
        let mut sum_sq = 0.0_f64;
        let mut n_chan = 0_usize;
        for (i, pos) in tile.positions.iter().enumerate() {
            if i >= stride * stride {
                break;
            }
            let gx = tile.anchor_m[0] + pos[0] as f64;
            let gy = tile.anchor_m[1] + pos[1] as f64;
            let gz = tile.anchor_m[2] + pos[2] as f64;
            let (r, c) = dir_to_pixel([gx, gy, gz]);
            for k in 0..3 {
                let want = linear(albedo[(r * ASSET_W + c) * 3 + k] as f32 / 255.0) as f64;
                sum_sq += (tile.colors[i][k] as f64 - want).powi(2);
                n_chan += 1;
            }
        }
        let albedo_rmse = (sum_sq / n_chan as f64).sqrt();
        println!(
            "| f{}L{} | legacy | {legacy_ms:.1} | {payload} | {height_err:.3} | {albedo_rmse:.4} |",
            key.face, key.level
        );

        // RCBT page on the identical footprint: heights + cost + payload.
        let t = Instant::now();
        let page = lod::bake_height_page(&field, key, 9, 100.0).expect("page bake");
        let page_ms = t.elapsed().as_secs_f64() * 1000.0;
        let cells = 8.0_f64;
        let page_wl = (key.span_m(field.params.radius_m) / cells).max(32.0);
        let mut page_err = 0.0_f64;
        for y in 0..9 {
            for x in 0..9 {
                let dir = key.direction(x as f64 / cells, y as f64 / cells);
                let reference = field.height_m(dir, page_wl);
                let got = page.decoded_sample(x, y).expect("in-grid") as f64;
                page_err = page_err.max((got - reference).abs());
            }
        }
        println!(
            "| f{}L{} | rcbt-page | {page_ms:.2} | {} | {page_err:.3} | n/a (geometry-only) |",
            key.face,
            key.level,
            page.to_bytes().len()
        );
        black_box((payload, page_err));
    }
}
