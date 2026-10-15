//! Immutable 128² material pages: RGB sRGB albedo, alpha linear roughness.
use std::sync::Arc;

/// Immutable page payloads keyed by canonical CBT cube-tile ID, with source
/// generations for dirty publication. A generation must identify the same
/// payload even across eviction/reinsertion; bump it whenever bytes change.
/// May be shared behind an `Arc` snapshot.
pub type MaterialPages = std::collections::BTreeMap<u64, (u64, CbtMaterialPage)>;

pub const MATERIAL_PAGE_SIZE: u32 = 128;

#[derive(Debug, Clone)]
pub struct CbtMaterialPage {
    mips: Arc<[Vec<u8>]>,
}

impl CbtMaterialPage {
    /// Build the mip chain off the render thread. RGB averages in linear
    /// light; roughness stays linear. Clones share the immutable payload.
    pub fn from_rgba8(rgba: Vec<u8>) -> Option<Self> {
        if rgba.len() != (MATERIAL_PAGE_SIZE * MATERIAL_PAGE_SIZE * 4) as usize {
            return None;
        }
        let linear: [f32; 256] = std::array::from_fn(|i| {
            let s = i as f32 / 255.0;
            if s <= 0.04045 {
                s / 12.92
            } else {
                ((s + 0.055) / 1.055).powf(2.4)
            }
        });
        let mut mips = vec![rgba];
        let mut size = MATERIAL_PAGE_SIZE as usize;
        while size > 1 {
            let source = mips.last().unwrap();
            let next_size = size / 2;
            let mut next = vec![0; next_size * next_size * 4];
            for y in 0..next_size {
                for x in 0..next_size {
                    for c in 0..4 {
                        let mut sum = 0.0;
                        for dy in 0..2 {
                            for dx in 0..2 {
                                let value = source[((y * 2 + dy) * size + x * 2 + dx) * 4 + c];
                                sum += if c < 3 {
                                    linear[value as usize]
                                } else {
                                    value as f32 / 255.0
                                };
                            }
                        }
                        let avg = sum * 0.25;
                        let encoded = if c == 3 {
                            avg
                        } else if avg <= 0.0031308 {
                            12.92 * avg
                        } else {
                            1.055 * avg.powf(1.0 / 2.4) - 0.055
                        };
                        next[(y * next_size + x) * 4 + c] =
                            (encoded * 255.0).round().clamp(0.0, 255.0) as u8;
                    }
                }
            }
            mips.push(next);
            size = next_size;
        }
        Some(Self { mips: mips.into() })
    }

    pub fn byte_len(&self) -> usize {
        self.mips.iter().map(Vec::len).sum()
    }

    /// Finest first, halving from 128² to 1². Missing coarse levels are
    /// allowed for decoded payloads; GPU consumers must clamp their max LOD.
    pub fn mips(&self) -> &[Vec<u8>] {
        &self.mips
    }

    pub fn from_decoded_mips(mips: Vec<Vec<u8>>) -> Option<Self> {
        if mips.is_empty() || mips.len() > 8 {
            return None;
        }
        let mut size = MATERIAL_PAGE_SIZE as usize;
        for mip in &mips {
            if mip.len() != size * size * 4 {
                return None;
            }
            size = (size / 2).max(1);
        }
        Some(Self { mips: mips.into() })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn linear_light_mips_and_shared_payload_survive_decode_roundtrip() {
        let mut rgba = vec![0; 128 * 128 * 4];
        for (i, pixel) in rgba.as_chunks_mut::<4>().0.iter_mut().enumerate() {
            let v = if i % 2 == 0 { 0 } else { 255 };
            *pixel = [v, v, v, 128];
        }
        let page = CbtMaterialPage::from_rgba8(rgba).unwrap();
        assert_eq!(page.mips().len(), 8);
        assert_eq!(&page.mips()[1][..4], &[188, 188, 188, 128]);
        assert_eq!(page.byte_len(), 87_380);
        assert!(std::ptr::eq(page.mips(), page.clone().mips()));
        let decoded = CbtMaterialPage::from_decoded_mips(page.mips().to_vec()).unwrap();
        assert_eq!(page.mips(), decoded.mips());
        assert!(CbtMaterialPage::from_rgba8(vec![0; 4]).is_none());
        assert!(CbtMaterialPage::from_decoded_mips(vec![]).is_none());
        assert!(CbtMaterialPage::from_decoded_mips(vec![vec![0; 4]]).is_none());
    }
}
