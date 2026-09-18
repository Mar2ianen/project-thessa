//! Page residency policy (doc §18 Phase D, §9 telemetry).
//!
//! Backend-neutral cache over encoded pages: stable key-addressed slots,
//! dirty-block upload ranges (only changed blocks re-upload), eviction by
//! encoded byte cost rather than page count, and telemetry in bytes and
//! effective texel coverage.
//!
//! The policy never touches GPU handles: a backend maps resident keys to
//! its own slot indices and consumes [`DirtyRange`] bytes. Ordering is by
//! `BTreeMap` key plus an explicit access tick, so a given operation
//! sequence evicts identically on every run.

use std::collections::BTreeMap;

use crate::codec::{EncodeMode, EncodedPage, ScalarField};
use crate::wgsl::block_base_table;

/// One dirty range to re-upload: `bytes` belong at `byte_offset` in the
/// page wire image (`page.to_bytes()`).
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct DirtyRange {
    /// Byte offset in the page wire image.
    pub byte_offset: u32,
    /// Replacement bytes (`3 + payload` for the block).
    pub bytes: Vec<u8>,
}

/// Residency failure modes.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum ResidencyError {
    /// No page under this key.
    NotResident(u64),
}

impl std::fmt::Display for ResidencyError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            ResidencyError::NotResident(key) => write!(f, "no resident page {key}"),
        }
    }
}

impl std::error::Error for ResidencyError {}

struct Slot {
    page: EncodedPage,
    /// Block indices changed since the last [`ResidencyCache::flush`].
    dirty: Vec<u32>,
    /// Page layout as of the last flush (or mark_clean): wire offsets of
    /// per-block patches are only valid while the layout is unchanged.
    /// `None` means the backend has never seen this page.
    flushed: Option<LayoutSig>,
    /// Eviction currency in bytes. Defaults to the wire image; a backend
    /// whose residency costs more (offset tables, decode targets, padding)
    /// reports its own cost via [`ResidencyCache::set_page_cost`].
    /// Updates reset it to the fresh wire image until re-reported.
    cost_bytes: u64,
    /// Monotonic access stamp for LRU (ties broken by key order).
    stamp: u64,
}

/// Block-grid layout signature: block payload lengths (and therefore every
/// wire offset) derive from exactly these fields.
#[derive(Debug, Clone, PartialEq, Eq)]
struct LayoutSig {
    width: u32,
    height: u32,
    blocks_x: u32,
    blocks_y: u32,
    tags: Vec<u8>,
}

fn layout_of(page: &EncodedPage) -> LayoutSig {
    LayoutSig {
        width: page.width,
        height: page.height,
        blocks_x: page.blocks_x,
        blocks_y: page.blocks_y,
        tags: page.blocks.iter().map(|b| b.codec.tag()).collect(),
    }
}

/// What a [`ResidencyCache::flush`] hands to the backend.
///
/// Block sizes are codec-dependent (R2 7 B .. Raw8 19 B), so a codec-rung
/// or extent change shifts every later wire offset *and* the offset table.
/// Per-block patches are only valid while the layout is unchanged;
/// otherwise the whole page plus its table must go up together.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum FlushPayload {
    /// Full page wire image plus the fresh block offset table. Required
    /// after insert and after any layout change.
    Full {
        /// `page.to_bytes()` snapshot.
        page_bytes: Vec<u8>,
        /// Fresh [`block_base_table`](crate::wgsl::block_base_table).
        table: Vec<u32>,
    },
    /// Wire-image patches, valid against the previously uploaded layout.
    Incremental {
        /// Changed-block patches (possibly empty).
        patches: Vec<DirtyRange>,
    },
}

/// Byte-cost residency cache over encoded scalar pages.
pub struct ResidencyCache {
    /// Soft byte budget: inserts evict LRU victims until the cache fits,
    /// but one oversized page is still admitted (see [`CacheTelemetry`]).
    budget_bytes: u64,
    slots: BTreeMap<u64, Slot>,
    clock: u64,
    hits: u64,
    misses: u64,
    evictions: u64,
    evicted_bytes: u64,
    uploaded_bytes: u64,
}

/// Snapshot of cache telemetry (doc §9: bytes and effective coverage).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct CacheTelemetry {
    /// Sum of resident `encoded_bytes`.
    pub resident_bytes: u64,
    /// Sum of resident true-extent texels.
    pub resident_texels: u64,
    /// Resident page count.
    pub pages: usize,
    /// Configured budget.
    pub budget_bytes: u64,
    /// Whether residents exceed the budget (one oversized page admitted).
    pub over_budget: bool,
    /// Successful lookups.
    pub hits: u64,
    /// Failed lookups.
    pub misses: u64,
    /// Evicted pages (lifetime total).
    pub evictions: u64,
    /// Freed bytes (lifetime total).
    pub evicted_bytes: u64,
    /// Wire bytes handed out as full pages plus dirty ranges (lifetime).
    pub uploaded_bytes: u64,
}

impl CacheTelemetry {
    /// Effective texel coverage per MiB resident (doc §9 headline metric).
    pub fn texels_per_mib(&self) -> f64 {
        if self.resident_bytes == 0 {
            return 0.0;
        }
        self.resident_texels as f64 / (self.resident_bytes as f64 / 1_048_576.0)
    }

    /// Hit rate over all lookups, `None` before the first lookup.
    pub fn hit_rate(&self) -> Option<f64> {
        let total = self.hits + self.misses;
        if total == 0 {
            return None;
        }
        Some(self.hits as f64 / total as f64)
    }
}

impl ResidencyCache {
    /// Empty cache with a soft byte budget.
    pub fn new(budget_bytes: u64) -> Self {
        Self {
            budget_bytes,
            slots: BTreeMap::new(),
            clock: 0,
            hits: 0,
            misses: 0,
            evictions: 0,
            evicted_bytes: 0,
            uploaded_bytes: 0,
        }
    }

    /// Current telemetry snapshot.
    pub fn telemetry(&self) -> CacheTelemetry {
        let resident_bytes = self.resident_bytes();
        let resident_texels = self
            .slots
            .values()
            .map(|s| s.page.width as u64 * s.page.height as u64)
            .sum();
        CacheTelemetry {
            resident_bytes,
            resident_texels,
            pages: self.slots.len(),
            budget_bytes: self.budget_bytes,
            over_budget: resident_bytes > self.budget_bytes,
            hits: self.hits,
            misses: self.misses,
            evictions: self.evictions,
            evicted_bytes: self.evicted_bytes,
            uploaded_bytes: self.uploaded_bytes,
        }
    }

    fn touch(&mut self, key: u64) {
        self.clock += 1;
        if let Some(slot) = self.slots.get_mut(&key) {
            slot.stamp = self.clock;
        }
    }

    /// Oldest `(stamp, key)` victim: least recently used, key order breaks
    /// stamp ties deterministically.
    fn victim(&self) -> Option<u64> {
        self.slots
            .iter()
            .min_by_key(|(key, slot)| (slot.stamp, **key))
            .map(|(key, _)| *key)
    }

    fn resident_bytes(&self) -> u64 {
        self.slots.values().map(|s| s.cost_bytes).sum()
    }

    fn evict_until_fit(&mut self) {
        while self.resident_bytes() > self.budget_bytes {
            let Some(victim) = self.victim() else {
                break;
            };
            // A single oversized page admits itself: evicting everything
            // else still leaves it over budget, so stop instead of
            // pointlessly dropping the only resident.
            if self.slots.len() == 1 {
                break;
            }
            let slot = self.slots.remove(&victim).expect("victim is resident");
            self.evictions += 1;
            self.evicted_bytes += slot.cost_bytes;
        }
    }

    /// Insert (or replace) a page. The whole page counts as fresh upload
    /// bytes; updates via [`ResidencyCache::update`] count dirty ranges
    /// instead. Evicts LRU victims until the budget fits. Cost resets to
    /// the wire image; backends re-report via [`ResidencyCache::set_page_cost`].
    pub fn insert(&mut self, key: u64, page: EncodedPage) {
        self.uploaded_bytes += page.encoded_bytes() as u64;
        self.clock += 1;
        let stamp = self.clock;
        let blocks = page.blocks.len() as u32;
        let cost_bytes = page.encoded_bytes() as u64;
        self.slots.insert(
            key,
            Slot {
                page,
                dirty: (0..blocks).collect(),
                flushed: None,
                cost_bytes,
                stamp,
            },
        );
        self.evict_until_fit();
    }

    /// Override the eviction cost of one resident page with the backend's
    /// true residency number (wire + tables + padding + decode targets).
    /// The cache evicts and reports in this currency until the next
    /// [`ResidencyCache::update`], which resets it to the fresh wire image.
    pub fn set_page_cost(&mut self, key: u64, cost_bytes: u64) -> Result<(), ResidencyError> {
        let slot = self
            .slots
            .get_mut(&key)
            .ok_or(ResidencyError::NotResident(key))?;
        slot.cost_bytes = cost_bytes;
        self.evict_until_fit();
        Ok(())
    }

    /// Look up a resident page. Hits refresh LRU; misses only count.
    pub fn get(&mut self, key: u64) -> Option<&EncodedPage> {
        if self.slots.contains_key(&key) {
            self.hits += 1;
            self.touch(key);
            self.slots.get(&key).map(|slot| &slot.page)
        } else {
            self.misses += 1;
            None
        }
    }

    /// Re-encode one resident field in place and diff blocks against the
    /// stored page. Returns the count of newly dirty blocks; the byte
    /// ranges come out of [`ResidencyCache::flush`]. Refreshes LRU.
    pub fn update(
        &mut self,
        key: u64,
        field: &ScalarField,
        mode: EncodeMode,
    ) -> Result<usize, ResidencyError> {
        let slot = self
            .slots
            .get_mut(&key)
            .ok_or(ResidencyError::NotResident(key))?;
        let fresh = EncodedPage::encode(field, mode);
        let mut dirty: Vec<u32> = Vec::new();
        for (i, (old, new)) in slot.page.blocks.iter().zip(fresh.blocks.iter()).enumerate() {
            if old != new {
                dirty.push(i as u32);
            }
        }
        // Extent or grid changes re-address every block.
        if slot.page.blocks.len() != fresh.blocks.len() {
            dirty = (0..fresh.blocks.len() as u32).collect();
        }
        let count = dirty.len();
        slot.page = fresh;
        slot.dirty = dirty;
        // A new wire image invalidates any backend-reported cost: the
        // backend re-reports after the next flush.
        slot.cost_bytes = slot.page.encoded_bytes() as u64;
        self.touch(key);
        self.evict_until_fit();
        Ok(count)
    }

    /// Drain pending changes for one page.
    ///
    /// Returns [`FlushPayload::Full`] after insert and whenever the block
    /// layout changed (codec-rung or extent change shifts every later wire
    /// offset, so per-block patches would land on the wrong bytes);
    /// otherwise returns [`FlushPayload::Incremental`] patches against the
    /// previously uploaded layout. Refreshes LRU.
    pub fn flush(&mut self, key: u64) -> Result<FlushPayload, ResidencyError> {
        let slot = self
            .slots
            .get_mut(&key)
            .ok_or(ResidencyError::NotResident(key))?;
        let layout = layout_of(&slot.page);
        let layout_changed = slot.flushed.as_ref() != Some(&layout);
        let payload = if layout_changed {
            let page_bytes = slot.page.to_bytes();
            let table = block_base_table(&slot.page);
            self.uploaded_bytes += page_bytes.len() as u64 + table.len() as u64 * 4;
            FlushPayload::Full { page_bytes, table }
        } else {
            let wire = slot.page.to_bytes();
            let table = block_base_table(&slot.page);
            let mut patches = Vec::with_capacity(slot.dirty.len());
            for index in slot.dirty.drain(..) {
                let block = &slot.page.blocks[index as usize];
                let base = table[index as usize] as usize;
                let len = 3 + block.codec.payload_len();
                patches.push(DirtyRange {
                    byte_offset: base as u32,
                    bytes: wire[base..base + len].to_vec(),
                });
            }
            self.uploaded_bytes += patches.iter().map(|r| r.bytes.len() as u64).sum::<u64>();
            FlushPayload::Incremental { patches }
        };
        slot.dirty.clear();
        slot.flushed = Some(layout);
        self.touch(key);
        Ok(payload)
    }

    /// Drop pending dirty flags without uploading (caller already holds
    /// the bytes, e.g. right after [`ResidencyCache::insert`]). Records
    /// the current layout as uploaded.
    pub fn mark_clean(&mut self, key: u64) -> Result<(), ResidencyError> {
        let slot = self
            .slots
            .get_mut(&key)
            .ok_or(ResidencyError::NotResident(key))?;
        slot.dirty.clear();
        slot.flushed = Some(layout_of(&slot.page));
        Ok(())
    }

    /// Remove one page, returning its bytes to the budget.
    pub fn evict(&mut self, key: u64) -> Result<EncodedPage, ResidencyError> {
        let slot = self
            .slots
            .remove(&key)
            .ok_or(ResidencyError::NotResident(key))?;
        self.evictions += 1;
        self.evicted_bytes += slot.cost_bytes;
        Ok(slot.page)
    }

    /// Resident keys in deterministic order.
    pub fn keys(&self) -> Vec<u64> {
        self.slots.keys().copied().collect()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{EncodeMode, fixtures};

    fn page_at(seed: u64) -> EncodedPage {
        EncodedPage::encode(
            &fixtures::noise(16, 16, seed),
            EncodeMode::Adaptive { max_abs_error: 2.0 },
        )
    }

    #[test]
    fn hits_misses_and_telemetry() {
        let mut cache = ResidencyCache::new(1 << 20);
        assert_eq!(cache.telemetry().hit_rate(), None);
        let page = page_at(1);
        let bytes = page.encoded_bytes();
        cache.insert(7, page);
        assert!(cache.get(7).is_some());
        assert!(cache.get(8).is_none());
        let t = cache.telemetry();
        assert_eq!((t.hits, t.misses), (1, 1));
        assert_eq!(t.hit_rate(), Some(0.5));
        assert_eq!(t.pages, 1);
        assert_eq!(t.resident_bytes, bytes as u64);
        assert_eq!(t.resident_texels, 256);
        assert!(!t.over_budget);
        assert_eq!(t.uploaded_bytes, bytes as u64);
        assert!(t.texels_per_mib() > 0.0);
    }

    #[test]
    fn eviction_is_lru_with_key_tiebreak() {
        let mut cache = ResidencyCache::new(u64::MAX);
        for key in [3u64, 1, 2] {
            cache.insert(key, page_at(key));
        }
        // Access order 2, 1 makes 3 the oldest; equal stamps fall back to
        // ascending key. Shrink the budget to hold exactly two pages.
        cache.get(2);
        cache.get(1);
        let two = (page_at(1).encoded_bytes() + page_at(2).encoded_bytes()) as u64;
        cache.budget_bytes = two;
        cache.evict_until_fit();
        assert_eq!(cache.keys(), vec![1, 2]);
        assert_eq!(cache.telemetry().evictions, 1);
    }

    #[test]
    fn insert_evicts_until_fit() {
        let a = page_at(11);
        let b = page_at(22);
        let budget = a.encoded_bytes() + b.encoded_bytes();
        let mut cache = ResidencyCache::new(budget as u64);
        cache.insert(1, page_at(11));
        cache.insert(2, page_at(22));
        cache.insert(3, page_at(33)); // Must evict key 1 (oldest).
        assert_eq!(cache.keys(), vec![2, 3]);
        assert!(!cache.telemetry().over_budget);
    }

    #[test]
    fn oversized_single_page_admits_and_flags() {
        let page = page_at(5);
        let mut cache = ResidencyCache::new(page.encoded_bytes() as u64 - 1);
        cache.insert(9, page);
        assert_eq!(cache.keys(), vec![9]);
        assert!(cache.telemetry().over_budget);
        // A second insert evicts everything else but keeps the oversized
        // page rather than dropping the only resident.
        cache.insert(10, page_at(6));
        assert!(cache.keys().contains(&9) || cache.keys().contains(&10));
    }

    #[test]
    fn identical_update_dirties_nothing() {
        let mut cache = ResidencyCache::new(1 << 20);
        let field = fixtures::noise(16, 16, 77);
        cache.insert(
            1,
            EncodedPage::encode(&field, EncodeMode::Adaptive { max_abs_error: 2.0 }),
        );
        cache.mark_clean(1).unwrap();
        let changed = cache
            .update(1, &field, EncodeMode::Adaptive { max_abs_error: 2.0 })
            .unwrap();
        assert_eq!(changed, 0);
        assert_eq!(
            cache.flush(1).unwrap(),
            FlushPayload::Incremental { patches: vec![] }
        );
    }

    #[test]
    fn single_texel_change_dirties_exactly_its_block() {
        let mut cache = ResidencyCache::new(1 << 20);
        let mut data = fixtures::uniform(16, 16, 100).data;
        cache.insert(
            1,
            EncodedPage::encode(
                &crate::ScalarField::new(16, 16, data.clone()).unwrap(),
                EncodeMode::Residual4,
            ),
        );
        cache.mark_clean(1).unwrap();
        // Flip one texel in block (bx=2, by=1) hard enough to move it:
        // texel (x=8, y=4) sits in the middle of that block.
        data[4 * 16 + 8] = 200;
        let field = crate::ScalarField::new(16, 16, data).unwrap();
        let changed = cache.update(1, &field, EncodeMode::Residual4).unwrap();
        assert_eq!(changed, 1);
        let ranges = match cache.flush(1).unwrap() {
            FlushPayload::Incremental { patches } => patches,
            FlushPayload::Full { .. } => panic!("same layout must patch incrementally"),
        };
        assert_eq!(ranges.len(), 1);
        // Range bytes match the wire image at the reported offset.
        let wire = cache.get(1).unwrap().to_bytes();
        assert_eq!(
            wire[ranges[0].byte_offset as usize
                ..ranges[0].byte_offset as usize + ranges[0].bytes.len()],
            ranges[0].bytes[..]
        );
        // Second flush is empty: flags drained, layout unchanged.
        assert_eq!(
            cache.flush(1).unwrap(),
            FlushPayload::Incremental { patches: vec![] }
        );
    }

    #[test]
    fn insert_reports_full_page_then_mark_clean_skips() {
        let mut cache = ResidencyCache::new(1 << 20);
        cache.insert(4, page_at(4));
        match cache.flush(4).unwrap() {
            FlushPayload::Full { page_bytes, table } => {
                let page = cache.get(4).unwrap();
                assert_eq!(page_bytes, page.to_bytes());
                assert_eq!(table, crate::wgsl::block_base_table(page));
            }
            FlushPayload::Incremental { .. } => panic!("first flush after insert is Full"),
        }
        // Re-insert and skip the initial upload instead.
        cache.insert(4, page_at(4));
        cache.mark_clean(4).unwrap();
        assert_eq!(
            cache.flush(4).unwrap(),
            FlushPayload::Incremental { patches: vec![] }
        );
    }

    #[test]
    fn rung_change_forces_full_page_and_table_reupload() {
        // Insert lossless Residual8, then re-encode lossy: every block
        // changes size, so per-block patches would land on shifted offsets.
        let mut cache = ResidencyCache::new(1 << 20);
        let field = fixtures::noise(16, 16, 0xBEAD);
        cache.insert(1, EncodedPage::encode(&field, EncodeMode::Residual8));
        cache.mark_clean(1).unwrap();
        let changed = cache.update(1, &field, EncodeMode::Residual4).unwrap();
        assert_eq!(changed, 16);
        match cache.flush(1).unwrap() {
            FlushPayload::Full { page_bytes, table } => {
                let page = cache.get(1).unwrap();
                assert_eq!(page_bytes, page.to_bytes());
                assert_eq!(table.len(), page.blocks.len());
                assert_eq!(table, crate::wgsl::block_base_table(page));
            }
            FlushPayload::Incremental { .. } => panic!("rung change must go Full"),
        }
        // Steady state again: the next identical update patches nothing.
        cache
            .update(1, &field, EncodeMode::Residual4)
            .expect("resident");
        assert_eq!(
            cache.flush(1).unwrap(),
            FlushPayload::Incremental { patches: vec![] }
        );
    }

    #[test]
    fn extent_change_forces_full_reupload() {
        let mut cache = ResidencyCache::new(1 << 20);
        cache.insert(2, page_at(4));
        cache.mark_clean(2).unwrap();
        cache
            .update(2, &fixtures::noise(8, 8, 9), EncodeMode::Residual4)
            .expect("resident");
        assert!(matches!(cache.flush(2).unwrap(), FlushPayload::Full { .. }));
        assert_eq!(cache.get(2).unwrap().width, 8);
    }

    #[test]
    fn missing_keys_error_but_misses_count() {
        let mut cache = ResidencyCache::new(1 << 20);
        assert_eq!(
            cache.update(1, &fixtures::uniform(4, 4, 0), EncodeMode::Raw8),
            Err(ResidencyError::NotResident(1))
        );
        assert_eq!(cache.flush(1), Err(ResidencyError::NotResident(1)));
        assert_eq!(cache.mark_clean(1), Err(ResidencyError::NotResident(1)));
        assert_eq!(
            cache.evict(1).map(|_| ()),
            Err(ResidencyError::NotResident(1))
        );
        assert!(cache.get(1).is_none());
        assert_eq!(cache.telemetry().misses, 1);
        assert_eq!(
            ResidencyError::NotResident(3).to_string(),
            "no resident page 3"
        );
    }

    #[test]
    fn backend_cost_override_drives_eviction_and_telemetry() {
        let mut cache = ResidencyCache::new(u64::MAX);
        let page = page_at(31);
        let wire = page.encoded_bytes() as u64;
        cache.insert(1, page.clone());
        cache.insert(2, page);
        // Backend reports wire + table + decode target as its true cost.
        let gpu_cost = wire + 1024 + 4096;
        cache.set_page_cost(1, gpu_cost).unwrap();
        cache.set_page_cost(2, gpu_cost).unwrap();
        assert_eq!(cache.telemetry().resident_bytes, 2 * gpu_cost);
        // A budget holding one gpu-cost page but not two evicts the LRU
        // victim on cost, not on wire bytes.
        cache.budget_bytes = gpu_cost + wire;
        cache.evict_until_fit();
        assert_eq!(cache.keys(), vec![2]);
        assert_eq!(cache.telemetry().evicted_bytes, gpu_cost);
        assert_eq!(
            cache.set_page_cost(9, 10),
            Err(ResidencyError::NotResident(9))
        );
    }

    #[test]
    fn update_resets_cost_to_fresh_wire() {
        let mut cache = ResidencyCache::new(u64::MAX);
        let field = fixtures::noise(16, 16, 0xC0DE);
        cache.insert(1, EncodedPage::encode(&field, EncodeMode::Residual8));
        cache.set_page_cost(1, 1_000_000).unwrap();
        cache
            .update(1, &field, EncodeMode::Residual8)
            .expect("resident");
        let wire = cache.get(1).unwrap().encoded_bytes() as u64;
        assert_eq!(cache.telemetry().resident_bytes, wire);
    }

    #[test]
    fn explicit_evict_returns_page_and_frees_budget() {
        let mut cache = ResidencyCache::new(1 << 20);
        let page = page_at(21);
        let bytes = page.encoded_bytes();
        cache.insert(21, page.clone());
        let back = cache.evict(21).unwrap();
        assert_eq!(back, page);
        let t = cache.telemetry();
        assert_eq!((t.pages, t.resident_bytes), (0, 0));
        assert_eq!((t.evictions, t.evicted_bytes), (1, bytes as u64));
    }

    #[test]
    fn same_operation_sequence_is_deterministic() {
        let run = || {
            let mut cache = ResidencyCache::new(50_000);
            for (i, seed) in [1u64, 2, 3, 4, 5, 1, 2, 6, 7, 3].iter().enumerate() {
                cache.insert(i as u64, page_at(*seed));
                cache.get(i as u64);
            }
            (cache.keys(), cache.telemetry())
        };
        let (keys_a, tel_a) = run();
        let (keys_b, tel_b) = run();
        assert_eq!(keys_a, keys_b);
        assert_eq!(tel_a, tel_b);
    }

    #[test]
    fn telemetry_texels_per_mib_math() {
        let t = CacheTelemetry {
            resident_bytes: 1_048_576,
            resident_texels: 1 << 20,
            pages: 1,
            budget_bytes: 1 << 24,
            over_budget: false,
            hits: 3,
            misses: 1,
            evictions: 0,
            evicted_bytes: 0,
            uploaded_bytes: 0,
        };
        assert_eq!(t.texels_per_mib(), 1_048_576.0);
        assert_eq!(t.hit_rate(), Some(0.75));
        let empty = CacheTelemetry {
            resident_bytes: 0,
            ..t
        };
        assert_eq!(empty.texels_per_mib(), 0.0);
    }
}
