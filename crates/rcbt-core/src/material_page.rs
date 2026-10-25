//! Immutable 128² material pages: RGB sRGB albedo, alpha linear roughness.
//! Optional RG8_SNORM planes carry full tangent normals and unresolved relief normals.
use std::sync::Arc;

/// Immutable page payloads keyed by canonical CBT cube-tile ID, with source
/// generations for dirty publication. A generation must identify the same
/// payload even across eviction/reinsertion; bump it whenever bytes change.
/// May be shared behind an `Arc` snapshot.
pub type MaterialPages = std::collections::BTreeMap<u64, (u64, CbtMaterialPage)>;

pub const MATERIAL_PAGE_SIZE: u32 = 128;

/// Encode a finite unit-normal component as an i8 byte for RG8_SNORM.
/// Zero is exact; symmetric rounding has error at most 0.5 / 127.
pub fn encode_material_normal_component(component: f64) -> u8 {
    (component.clamp(-1.0, 1.0) * 127.0).round() as i8 as u8
}

/// Match GPU SNORM decoding, including the non-canonical -128 byte alias.
pub fn decode_material_normal_component(encoded: u8) -> f32 {
    ((encoded as i8) as f32 / 127.0).max(-1.0)
}

#[derive(Debug, Clone)]
pub struct CbtMaterialPage {
    mips: Arc<[Vec<u8>]>,
    normal_mips: Option<Arc<[Vec<u8>]>>,
    relief_residual_mips: Option<Arc<[Vec<u8>]>>,
}

impl CbtMaterialPage {
    /// Build the mip chain off the render thread. RGB averages in linear
    /// light; roughness stays linear. Clones share the immutable payload.
    pub fn from_rgba8(rgba: Vec<u8>) -> Option<Self> {
        Self::from_rgba8_with_normals(rgba, None)
    }

    /// Build an albedo/roughness page plus an optional tangent-space normal
    /// plane. Normal bytes are RG8_SNORM, encoded with
    /// [`encode_material_normal_component`];
    /// the sampler reconstructs positive z. Normal mips average the decoded
    /// height gradients (`normal.xy / normal.z`) before reconstructing a unit
    /// normal. This is linear in bump influence, not in unit-normal XY.
    pub fn from_rgba8_with_normals(rgba: Vec<u8>, normal_rg8: Option<Vec<u8>>) -> Option<Self> {
        let mips = color_mips(rgba)?;
        let normal_mips = match normal_rg8 {
            Some(bytes) => Some(normal_mips_from_rg8(bytes)?),
            None => None,
        };
        Some(Self {
            mips: mips.into(),
            normal_mips: normal_mips.map(Into::into),
            relief_residual_mips: None,
        })
    }

    /// Optional material gradient remaining after the matching 33-square
    /// rendered soil-height page. Full optical normals remain available.
    pub fn with_relief_residuals(mut self, rg8: Vec<u8>) -> Option<Self> {
        self.normal_mips.as_ref()?;
        self.relief_residual_mips = Some(normal_mips_from_rg8(rg8)?.into());
        Some(self)
    }

    pub fn relief_residual_mips(&self) -> Option<&[Vec<u8>]> {
        self.relief_residual_mips.as_deref()
    }

    pub fn byte_len(&self) -> usize {
        self.relief_residual_mips
            .as_ref()
            .map_or(0, |mips| mips.iter().map(Vec::len).sum::<usize>())
            + self.mips.iter().map(Vec::len).sum::<usize>()
            + self
                .normal_mips
                .as_ref()
                .map_or(0, |mips| mips.iter().map(Vec::len).sum())
    }

    /// Finest first, halving from 128² to 1². Missing coarse levels are
    /// allowed for decoded payloads; GPU consumers must clamp their max LOD.
    pub fn mips(&self) -> &[Vec<u8>] {
        &self.mips
    }

    /// Optional RG8_SNORM normal chain with the same halving shape.
    pub fn normal_mips(&self) -> Option<&[Vec<u8>]> {
        self.normal_mips.as_deref()
    }

    pub fn from_decoded_mips(mips: Vec<Vec<u8>>) -> Option<Self> {
        Self::from_decoded_mips_with_normals(mips, None)
    }

    /// Adopt decoded planes. Normal bytes use the same RG8_SNORM contract as
    /// [`Self::from_rgba8_with_normals`], with no unsigned-format auto-detection.
    pub fn from_decoded_mips_with_normals(
        mips: Vec<Vec<u8>>,
        normal_mips: Option<Vec<Vec<u8>>>,
    ) -> Option<Self> {
        if !valid_chain(&mips, 4) {
            return None;
        }
        if let Some(normal) = &normal_mips
            && !valid_chain(normal, 2)
        {
            return None;
        }
        Some(Self {
            mips: mips.into(),
            normal_mips: normal_mips.map(Into::into),
            relief_residual_mips: None,
        })
    }
}

/// Colour/roughness mips: RGB averages in linear light, roughness in linear.
fn color_mips(rgba: Vec<u8>) -> Option<Vec<Vec<u8>>> {
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
    Some(mips)
}

/// Normal mips average decoded surface gradients, then reconstruct a unit
/// normal. A finite z floor matches the shader for extreme/invalid RG pairs.
fn normal_mips_from_rg8(rg8: Vec<u8>) -> Option<Vec<Vec<u8>>> {
    if rg8.len() != (MATERIAL_PAGE_SIZE * MATERIAL_PAGE_SIZE * 2) as usize {
        return None;
    }
    let mut mips = vec![rg8];
    let mut size = MATERIAL_PAGE_SIZE as usize;
    while size > 1 {
        let source = mips.last().unwrap();
        let next_size = size / 2;
        let mut next = vec![0; next_size * next_size * 2];
        for y in 0..next_size {
            for x in 0..next_size {
                let mut sx = 0.0_f32;
                let mut sy = 0.0_f32;
                for dy in 0..2 {
                    for dx in 0..2 {
                        let i = ((y * 2 + dy) * size + x * 2 + dx) * 2;
                        let x = decode_material_normal_component(source[i]);
                        let y = decode_material_normal_component(source[i + 1]);
                        let z = (1.0 - x * x - y * y).max(0.0001).sqrt();
                        sx += x / z;
                        sy += y / z;
                    }
                }
                sx *= 0.25;
                sy *= 0.25;
                let length = (1.0 + sx * sx + sy * sy).sqrt();
                sx /= length;
                sy /= length;
                next[(y * next_size + x) * 2] = encode_material_normal_component(f64::from(sx));
                next[(y * next_size + x) * 2 + 1] = encode_material_normal_component(f64::from(sy));
            }
        }
        mips.push(next);
        size = next_size;
    }
    Some(mips)
}

/// Validate a decoded chain whose base level is 128² with `stride` bytes per
/// texel. Allows omitted coarse levels, matching the existing decoded-page
/// contract, but rejects gaps and oversized payloads.
fn valid_chain(mips: &[Vec<u8>], stride: usize) -> bool {
    if mips.is_empty() || mips.len() > 8 {
        return false;
    }
    let mut size = MATERIAL_PAGE_SIZE as usize;
    for mip in mips {
        if mip.len() != size * size * stride {
            return false;
        }
        size = (size / 2).max(1);
    }
    true
}

#[cfg(test)]
mod tests {
    #[test]
    fn residual_plane_is_validated_shared_and_independently_mipped() {
        let full = super::CbtMaterialPage::from_rgba8_with_normals(
            vec![0; 128 * 128 * 4],
            Some([32, 0].repeat(128 * 128)),
        )
        .unwrap();
        assert!(full.clone().with_relief_residuals(vec![0; 2]).is_none());
        let length = full.byte_len();
        let residual = full
            .with_relief_residuals([0, 16].repeat(128 * 128))
            .unwrap();
        assert_eq!(residual.byte_len(), length + 43690);
        assert!(
            residual.normal_mips().unwrap().iter().all(|m| m
                .as_chunks::<2>()
                .0
                .iter()
                .all(|xy| *xy == [32, 0]))
        );
        assert!(
            residual.relief_residual_mips().unwrap().iter().all(|m| m
                .as_chunks::<2>()
                .0
                .iter()
                .all(|xy| *xy == [0, 16]))
        );
        assert!(
            super::CbtMaterialPage::from_rgba8(vec![0; 128 * 128 * 4])
                .unwrap()
                .with_relief_residuals(vec![0; 128 * 128 * 2])
                .is_none()
        );
    }

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

    #[test]
    fn normal_mips_halve_from_rg8_and_share_clone_payload() {
        let normal = [0, 0].repeat(128 * 128);
        let page = CbtMaterialPage::from_rgba8_with_normals(
            vec![128; 128 * 128 * 4],
            Some(normal.clone()),
        )
        .unwrap();
        let mips = page.normal_mips().expect("normal plane");
        assert_eq!(mips.len(), 8);
        assert_eq!(mips[0].len(), 128 * 128 * 2);
        assert_eq!(mips[1].len(), 64 * 64 * 2);
        assert_eq!(mips[7].len(), 2);
        assert!(
            mips.iter()
                .all(|mip| mip.as_chunks::<2>().0.iter().all(|p| p == &[0, 0]))
        );
        assert_eq!(page.byte_len(), 87_380 + 43_690);
        assert!(std::ptr::eq(
            page.normal_mips().unwrap(),
            page.clone().normal_mips().unwrap()
        ));
        let decoded = CbtMaterialPage::from_decoded_mips_with_normals(
            page.mips().to_vec(),
            Some(mips.to_vec()),
        )
        .unwrap();
        assert_eq!(decoded.normal_mips(), page.normal_mips());
        assert!(
            CbtMaterialPage::from_rgba8_with_normals(vec![0; 4], Some(normal.clone())).is_none()
        );
        assert!(
            CbtMaterialPage::from_rgba8_with_normals(vec![0; 128 * 128 * 4], Some(vec![0; 4]))
                .is_none()
        );
        assert!(
            CbtMaterialPage::from_decoded_mips_with_normals(
                vec![vec![0; 128 * 128 * 4]],
                Some(vec![vec![0; 128 * 128 * 2], vec![0; 4]])
            )
            .is_none()
        );
    }

    #[test]
    fn normal_mips_average_decoded_slopes_before_encoding() {
        // Alternating opposite gradients cancel at the first mip.
        let mut normal = Vec::with_capacity(128 * 128 * 2);
        for i in 0..128 * 128 {
            normal.extend(if i % 2 == 0 { [76, 0] } else { [180, 0] });
        }
        let page =
            CbtMaterialPage::from_rgba8_with_normals(vec![0; 128 * 128 * 4], Some(normal)).unwrap();
        let mips = page.normal_mips().unwrap();
        assert_eq!(&mips[1][..2], &[0, 0]);
        assert_eq!(&mips[7][..2], &[0, 0]);
    }

    #[test]
    fn normal_mips_mix_bump_influence_not_unit_normal_xy() {
        let normal = [102, 0, 0, 0].repeat(128 * 128 / 2);
        let page =
            CbtMaterialPage::from_rgba8_with_normals(vec![0; 128 * 128 * 4], Some(normal)).unwrap();
        // Average slopes (1.35 + 0)/2 -> normal.x ~0.56 -> signed byte 71.
        // Averaging unit-normal XY would instead produce byte 51.
        assert!(page.normal_mips().unwrap()[1][0].abs_diff(71) <= 1);
    }
    #[test]
    fn signed_normal_encoding_is_zero_exact_symmetric_and_bounded() {
        assert_eq!(encode_material_normal_component(0.0), 0);
        assert_eq!(decode_material_normal_component(0), 0.0);
        assert_eq!(decode_material_normal_component(128), -1.0);
        for step in -100_000..=100_000 {
            let value = f64::from(step) / 100_000.0;
            let encoded = encode_material_normal_component(value);
            let decoded = f64::from(decode_material_normal_component(encoded));
            assert!((decoded - value).abs() <= 0.5 / 127.0 + f64::from(f32::EPSILON));
            assert_eq!(
                encode_material_normal_component(-value),
                (encoded as i8).wrapping_neg() as u8
            );
        }
    }
}
