//! CPU mip-chain data for microscaled pages (follow-up: data side only).
//!
//! Box-filter downsampling of decoded scalar fields into power-of-two
//! chains, encoded rung by rung like any other page. This answers the
//! byte-accounting half of the mip question (how much a chain costs and
//! what it measures); GPU mip *sampling* (LOD selection, filtering in the
//! sample-time decoder) stays open work.
//!
//! Chains are plain vectors of [`EncodedPage`]: each level is an
//! independent residency entry, so no wire format changes and no coupling
//! to the mip policy of the current RGBA path.

use crate::codec::{EncodeMode, EncodedPage, ScalarField};

/// One box-downsample step: 2x2 average with edge replication, rounded.
/// Halves each extent (odd extents replicate the edge texel); a 1-wide
/// axis stays 1.
pub fn downsample_box(field: &ScalarField) -> ScalarField {
    let w2 = field.width.div_ceil(2).max(1);
    let h2 = field.height.div_ceil(2).max(1);
    let mut data = Vec::with_capacity(w2 as usize * h2 as usize);
    for y in 0..h2 {
        for x in 0..w2 {
            let mut sum = 0u32;
            for dy in 0..2 {
                for dx in 0..2 {
                    sum += field.sample_edge((x * 2 + dx) as i64, (y * 2 + dy) as i64) as u32;
                }
            }
            data.push(((sum + 2) / 4) as u8);
        }
    }
    ScalarField {
        width: w2,
        height: h2,
        data,
    }
}

/// A decoded mip chain: level 0 is the source field.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct MipChain {
    /// Finest first, down to 1x1.
    pub levels: Vec<ScalarField>,
}

impl MipChain {
    /// Build `levels` downsamples of `field` (level 0 included, so the
    /// chain always holds `levels + 1` entries or fewer once 1x1 hits).
    pub fn build(field: &ScalarField, levels: u32) -> Self {
        let mut chain = vec![field.clone()];
        for _ in 0..levels {
            let last = chain.last().expect("nonempty chain");
            if last.width == 1 && last.height == 1 {
                break;
            }
            chain.push(downsample_box(last));
        }
        Self { levels: chain }
    }

    /// Encode every level with one mode.
    pub fn encode(&self, mode: EncodeMode) -> Vec<EncodedPage> {
        self.levels
            .iter()
            .map(|level| EncodedPage::encode(level, mode))
            .collect()
    }

    /// Total wire bytes of an encoded chain.
    pub fn chain_bytes(pages: &[EncodedPage]) -> usize {
        pages.iter().map(EncodedPage::encoded_bytes).sum()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::fixtures;

    #[test]
    fn chain_halves_to_1x1_and_stops() {
        let field = fixtures::noise(128, 128, 7);
        let chain = MipChain::build(&field, 20);
        let dims: Vec<_> = chain.levels.iter().map(|l| (l.width, l.height)).collect();
        assert_eq!(
            dims,
            vec![
                (128, 128),
                (64, 64),
                (32, 32),
                (16, 16),
                (8, 8),
                (4, 4),
                (2, 2),
                (1, 1)
            ]
        );
        assert_eq!(chain.levels[0], field);
    }

    #[test]
    fn odd_extents_replicate_edges() {
        // 5x5 -> 3x3 -> 2x2 -> 1x1, no panics, exact dims.
        let field = fixtures::noise(5, 5, 3);
        let chain = MipChain::build(&field, 9);
        let dims: Vec<_> = chain.levels.iter().map(|l| (l.width, l.height)).collect();
        assert_eq!(dims, vec![(5, 5), (3, 3), (2, 2), (1, 1)]);
    }

    #[test]
    fn uniform_survives_downsampling_exactly() {
        let field = fixtures::uniform(32, 32, 200);
        let chain = MipChain::build(&field, 3);
        for level in &chain.levels {
            assert!(
                level.data.iter().all(|v| (198..=201).contains(v)),
                "drifted uniform"
            );
        }
    }

    #[test]
    fn checker_averages_to_mid() {
        // 2x2-alternating checker collapses toward mid-grey down the chain.
        let field = fixtures::checker(16, 16, 1);
        let chain = MipChain::build(&field, 2);
        assert_eq!((chain.levels[1].width, chain.levels[1].height), (8, 8));
        // A 2x2 quad holds two blacks and two whites -> 127/128.
        assert!(chain.levels[1].data.iter().all(|v| (127..=128).contains(v)));
    }

    #[test]
    fn chain_bytes_bound_chain_cost() {
        // Lossless chain of a 64x64 page costs under 4/3 of the base plus
        // per-level headers: the geometric series made explicit.
        let field = fixtures::noise(64, 64, 11);
        let chain = MipChain::build(&field, 6);
        let pages = chain.encode(EncodeMode::Residual8);
        let total = MipChain::chain_bytes(&pages);
        let base = pages[0].encoded_bytes();
        assert!(total < base * 4 / 3 + pages.len() * 64, "{total} vs {base}");
        // Every level round-trips losslessly.
        for (level, page) in chain.levels.iter().zip(pages.iter()) {
            assert_eq!(&page.decode(), level);
        }
    }

    #[test]
    fn determinism() {
        let field = fixtures::noise(48, 48, 5);
        assert_eq!(
            MipChain::build(&field, 4).levels,
            MipChain::build(&field, 4).levels
        );
    }
}
