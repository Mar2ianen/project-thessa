//! Microstore encoding for game material pages (integration seam).
//!
//! Splits the RGBA mip levels of a [`CbtMaterialPage`] into scalar
//! channels and encodes each with the backend-neutral microstore codec.
//! The Bevy schedule owns *when* encoding happens; the bytes stay
//! renderer-neutral (no `wgpu`/`bevy_render` types cross this boundary).

use bevy::prelude::*;
use thessa_microstore_core::{ColorField, ColorPage, EncodeMode, EncodedPage, ScalarField};

use crate::material_pages::{CbtMaterialPage, MATERIAL_PAGE_SIZE};

/// One encoded material mip level: RGB color plus roughness.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct EncodedMaterialLevel {
    /// RGB planes.
    pub color: ColorPage,
    /// Roughness (alpha) plane.
    pub roughness: EncodedPage,
}

impl EncodedMaterialLevel {
    /// Exact wire size of both parts.
    pub fn encoded_bytes(&self) -> usize {
        self.color.encoded_bytes() + self.roughness.encoded_bytes()
    }
}

/// Encode one RGBA mip level (`width` x `height` texels) with an adaptive
/// per-channel budget in code levels. Returns `None` on extent mismatch.
pub fn encode_material_level(
    rgba: &[u8],
    width: u32,
    height: u32,
    max_abs_error: f64,
) -> Option<EncodedMaterialLevel> {
    if rgba.len() != width as usize * height as usize * 4 {
        return None;
    }
    let mut planes = [Vec::new(), Vec::new(), Vec::new(), Vec::new()];
    for px in rgba.as_chunks::<4>().0 {
        for (plane, v) in planes.iter_mut().zip(px.iter()) {
            plane.push(*v);
        }
    }
    let mut planes = planes.into_iter();
    let rgb = [
        ScalarField::new(width, height, planes.next().expect("r")).expect("extent"),
        ScalarField::new(width, height, planes.next().expect("g")).expect("extent"),
        ScalarField::new(width, height, planes.next().expect("b")).expect("extent"),
    ];
    let a = ScalarField::new(width, height, planes.next().expect("a")).expect("extent");
    let mode = EncodeMode::Adaptive { max_abs_error };
    Some(EncodedMaterialLevel {
        color: ColorPage::encode(&ColorField::new(rgb).expect("rgb"), mode),
        roughness: EncodedPage::encode(&a, mode),
    })
}

/// Inbox of freshly streamed material pages (node id + page).
#[derive(Resource, Default)]
pub struct MicrostoreInbox {
    /// Pending pages.
    pub pages: Vec<(u64, CbtMaterialPage)>,
}

/// Encoded output, one entry per mip level, finest first.
#[derive(Resource, Default)]
pub struct MicrostoreOutbox {
    /// Encoded pages by node id.
    pub pages: Vec<(u64, Vec<EncodedMaterialLevel>)>,
}

/// System: drain the inbox, encode every mip level of every page.
/// Mip extents halve from [`MATERIAL_PAGE_SIZE`] per level.
pub fn encode_inbox_system(
    mut inbox: ResMut<MicrostoreInbox>,
    mut outbox: ResMut<MicrostoreOutbox>,
) {
    for (id, page) in inbox.pages.drain(..) {
        let mut levels = Vec::with_capacity(page.mips.len());
        let (mut w, mut h) = (MATERIAL_PAGE_SIZE, MATERIAL_PAGE_SIZE);
        for mip in page.mips.iter() {
            if let Some(encoded) = encode_material_level(mip, w, h, 2.0) {
                levels.push(encoded);
            }
            w = (w / 2).max(1);
            h = (h / 2).max(1);
        }
        outbox.pages.push((id, levels));
    }
}

#[cfg(test)]
pub(crate) use tests::rock_rgba;

#[cfg(test)]
mod tests {
    use super::*;
    use crate::CbtPlugin;

    /// Deterministic rock-like RGBA: banded strata plus grain, no RNG
    /// dependency. Exercises the real `from_rgba8` constructor including
    /// its linear-light mip averaging.
    pub(crate) fn rock_rgba(size: u32, seed: u64) -> Vec<u8> {
        let mut s = seed | 1;
        let mut next = || {
            s = s.wrapping_add(0x9E37_79B9_7F4A_7C15);
            let mut z = s;
            z = (z ^ (z >> 30)).wrapping_mul(0xBF58_476D_1CE4_E5B9);
            z = (z ^ (z >> 27)).wrapping_mul(0x94D0_49BB_1331_11EB);
            (z ^ (z >> 31)) as f64 / u64::MAX as f64
        };
        let mut out = Vec::with_capacity(size as usize * size as usize * 4);
        for y in 0..size {
            for x in 0..size {
                // Strata bands plus per-texel grain.
                let band = ((y / 16) % 2) as f64;
                let grain = next();
                let r = (90.0 + band * 60.0 + grain * 40.0).clamp(0.0, 255.0) as u8;
                let g = (70.0 + band * 40.0 + grain * 30.0).clamp(0.0, 255.0) as u8;
                let b = (50.0 + grain * 60.0 + (x % 7) as f64 * 3.0).clamp(0.0, 255.0) as u8;
                let a = (110.0 + band * 80.0).clamp(0.0, 255.0) as u8;
                out.extend_from_slice(&[r, g, b, a]);
            }
        }
        out
    }

    #[test]
    fn encode_level_round_trips_channels() {
        let rgba = rock_rgba(16, 7);
        let level = encode_material_level(&rgba, 16, 16, 2.0).expect("encodes");
        let decoded = level.color.decode();
        let rough = level.roughness.decode();
        for (plane, channel) in decoded.iter().chain([&rough]).zip(0..4) {
            for (i, v) in plane.data.iter().enumerate() {
                let want = rgba[i * 4 + channel];
                assert!(
                    v.abs_diff(want) <= 2,
                    "channel {channel} texel {i}: {v} vs {want}"
                );
            }
        }
    }

    #[test]
    fn encode_level_rejects_extent_mismatch() {
        assert!(encode_material_level(&[0u8; 10], 4, 4, 2.0).is_none());
        assert!(encode_material_level(&[0u8; 4 * 4 * 4], 4, 4, 2.0).is_some());
    }

    #[test]
    fn headless_app_encodes_real_material_page_through_schedule() {
        // The real game constructor (linear-light mip averaging included),
        // then a headless Bevy schedule runs the bridge system.
        let rgba = rock_rgba(MATERIAL_PAGE_SIZE, 0x5EED);
        let page = CbtMaterialPage::from_rgba8(rgba).expect("real page builds");
        assert_eq!(page.mips.len(), 8);
        let mut app = App::new();
        app.add_plugins(CbtPlugin::default());
        app.insert_resource(MicrostoreInbox {
            pages: vec![(17, page)],
        });
        app.init_resource::<MicrostoreOutbox>();
        app.add_systems(Update, encode_inbox_system);
        app.update();
        app.update();
        let outbox = app.world().resource::<MicrostoreOutbox>();
        assert_eq!(outbox.pages.len(), 1);
        let (id, levels) = &outbox.pages[0];
        assert_eq!(*id, 17);
        assert_eq!(levels.len(), 8);
        // The inbox drained exactly once (second update is a no-op).
        let inbox = app.world().resource::<MicrostoreInbox>();
        assert!(inbox.pages.is_empty());
        let total: usize = levels.iter().map(EncodedMaterialLevel::encoded_bytes).sum();
        eprintln!("8 game mips microstore-encoded: {total} B vs 87,380 B RGBA");
        assert!(total < 87_380, "must beat the raw mip chain");
    }
}
