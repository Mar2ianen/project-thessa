//! Immutable material pages; RGB is sRGB albedo and alpha is linear roughness.
use bevy::prelude::Resource;
use std::{collections::BTreeMap, sync::Arc};

pub const MATERIAL_PAGE_SIZE: u32 = 128;

#[derive(Debug, Clone)]
pub struct CbtMaterialPage {
    pub(crate) mips: Arc<[Vec<u8>]>,
}

impl CbtMaterialPage {
    /// Generate the small mip chain on the streaming worker, never the render
    /// thread. RGB averages in linear light; roughness alpha stays linear.
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
}

#[derive(Debug, Clone, Default, Resource)]
pub struct CbtRenderMaterialPages {
    pub(crate) generation: u64,
    pub(crate) priority: Vec<u64>,
    pub(crate) pages: BTreeMap<u64, (u64, CbtMaterialPage)>,
}

impl CbtRenderMaterialPages {
    /// Ordered visible/prefetch sources, independent from topology ordering.
    pub fn set_priority(&mut self, priority: Vec<u64>) {
        if self.priority != priority {
            self.priority = priority;
            self.generation = self.generation.saturating_add(1);
        }
    }

    /// Immutable view of the current priority. Check this through a shared
    /// (`Res`/`&`) borrow *before* taking a mutable borrow: calling any
    /// `&mut self` method via Bevy `ResMut` marks the resource as changed
    /// (via `DerefMut`) even when the bytes end up identical, which forces
    /// `ExtractResourcePlugin` to snapshot `priority + BTreeMap` into the
    /// render world every frame. The `&self` path uses `Deref` only and
    /// leaves the change flag alone.
    pub fn priority(&self) -> &[u64] {
        &self.priority
    }

    pub fn byte_len(&self) -> usize {
        self.pages.values().map(|(_, page)| page.byte_len()).sum()
    }
    pub fn contains_page(&self, node_id: u64) -> bool {
        self.pages.contains_key(&node_id)
    }
    pub fn set_page(&mut self, node_id: u64, page: CbtMaterialPage) {
        self.generation = self.generation.saturating_add(1);
        self.pages.insert(node_id, (self.generation, page));
    }
    pub fn remove_page(&mut self, node_id: u64) {
        if self.pages.remove(&node_id).is_some() {
            self.generation = self.generation.saturating_add(1);
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn mips_average_color_in_linear_light_and_keep_roughness_linear() {
        let mut bytes = vec![0; 128 * 128 * 4];
        for (i, p) in bytes.chunks_exact_mut(4).enumerate() {
            let value = if i % 2 == 0 { 0 } else { 255 };
            p.copy_from_slice(&[value; 4]);
        }
        let page = CbtMaterialPage::from_rgba8(bytes).unwrap();
        assert_eq!(page.mips.len(), 8);
        assert_eq!(&page.mips[1][..4], &[188, 188, 188, 128]);
        assert_eq!(page.mips.last().unwrap().len(), 4);
        assert!(CbtMaterialPage::from_rgba8(vec![0; 4]).is_none());
    }
}
