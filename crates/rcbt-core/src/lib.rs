//! Backend-agnostic logical state and contracts for adaptive terrain trees.
//!
//! This crate deliberately knows nothing about Bevy, wgpu, Vulkan, or a
//! terrain height field. A [`Tree`] is only a deterministic partition of a
//! binary domain. Renderers decide how a leaf maps to geometry; authoritative
//! surface queries remain outside this crate.

#![forbid(unsafe_code)]

use std::collections::BTreeSet;
use std::collections::HashMap;
use std::fmt;

pub const MAX_SUPPORTED_DEPTH: u8 = 58;
const ENCODING_VERSION: u8 = 1;
const PAGE_ENCODING_VERSION: u8 = 1;

pub mod packed;
pub mod compact;

/// A heap-addressed binary-tree node.
///
/// The root has id `1` and depth `0`; children are `id << 1` and
/// `(id << 1) | 1`. The representation matches the observable addressing
/// semantics of `libcbt`, without exposing its heap bitfield layout.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct Node {
    id: u64,
    depth: u8,
}

impl Node {
    pub const fn root() -> Self {
        Self { id: 1, depth: 0 }
    }

    pub fn new(id: u64, depth: u8) -> Result<Self, NodeError> {
        if depth > MAX_SUPPORTED_DEPTH {
            return Err(NodeError::DepthTooLarge { depth });
        }
        let minimum = 1_u64 << depth;
        let maximum = 1_u64 << (depth + 1);
        if id < minimum || id >= maximum {
            return Err(NodeError::InvalidHeapId { id, depth });
        }
        Ok(Self { id, depth })
    }

    pub fn from_heap_id(id: u64) -> Result<Self, NodeError> {
        if id == 0 {
            return Err(NodeError::NullNode);
        }
        let depth = (u64::BITS - id.leading_zeros() - 1) as u8;
        Self::new(id, depth)
    }

    pub const fn id(self) -> u64 {
        self.id
    }

    pub const fn depth(self) -> u8 {
        self.depth
    }

    pub const fn is_root(self) -> bool {
        self.id == 1
    }

    pub fn parent(self) -> Option<Self> {
        (!self.is_root()).then(|| Self {
            id: self.id >> 1,
            depth: self.depth - 1,
        })
    }

    pub fn left_child(self) -> Option<Self> {
        (self.depth < MAX_SUPPORTED_DEPTH).then(|| Self {
            id: self.id << 1,
            depth: self.depth + 1,
        })
    }

    pub fn right_child(self) -> Option<Self> {
        (self.depth < MAX_SUPPORTED_DEPTH).then(|| Self {
            id: (self.id << 1) | 1,
            depth: self.depth + 1,
        })
    }

    pub fn children(self) -> Option<[Self; 2]> {
        Some([self.left_child()?, self.right_child()?])
    }

    pub fn sibling(self) -> Option<Self> {
        (!self.is_root()).then_some(Self {
            id: self.id ^ 1,
            depth: self.depth,
        })
    }
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum NodeError {
    NullNode,
    DepthTooLarge { depth: u8 },
    InvalidHeapId { id: u64, depth: u8 },
}

impl fmt::Display for NodeError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::NullNode => f.write_str("node id 0 is reserved for the null node"),
            Self::DepthTooLarge { depth } => {
                write!(f, "node depth {depth} exceeds the supported maximum")
            }
            Self::InvalidHeapId { id, depth } => {
                write!(f, "heap id {id} is invalid at depth {depth}")
            }
        }
    }
}

impl std::error::Error for NodeError {}

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum TreeError {
    InvalidMaxDepth { depth: u8 },
    InvalidNode(NodeError),
    NotALeaf(Node),
    AtMaximumDepth(Node),
    CannotMergeRoot,
    ChildrenNotLeaves(Node),
    BatchConflict,
    InvalidEncoding(&'static str),
}

impl fmt::Display for TreeError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::InvalidMaxDepth { depth } => write!(f, "tree max depth {depth} is unsupported"),
            Self::InvalidNode(error) => error.fmt(f),
            Self::NotALeaf(node) => write!(f, "node {:?} is not a leaf", node),
            Self::AtMaximumDepth(node) => write!(f, "node {:?} is already at maximum depth", node),
            Self::CannotMergeRoot => f.write_str("the root cannot be merged"),
            Self::ChildrenNotLeaves(node) => {
                write!(f, "children of {:?} are not both leaves", node)
            }
            Self::BatchConflict => {
                f.write_str("batch contains conflicting or overlapping operations")
            }
            Self::InvalidEncoding(message) => f.write_str(message),
        }
    }
}

impl std::error::Error for TreeError {}

impl From<NodeError> for TreeError {
    fn from(value: NodeError) -> Self {
        Self::InvalidNode(value)
    }
}

/// A requested topology operation. Operations are applied in the supplied
/// order to a private copy and committed only if the whole batch succeeds.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Update {
    Split(Node),
    Merge(Node),
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
pub enum WorkClass {
    CoverageRepair,
    VisibleGeometry,
    PredictedGeometry,
    Cosmetic,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct LeafCandidate {
    pub node: Node,
    pub action: CandidateAction,
    pub class: WorkClass,
    pub projected_error_px: f32,
    pub predicted_error_px: f32,
    pub time_to_needed_s: f32,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CandidateAction {
    Split,
    Merge,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct FrameBudget {
    pub max_operations: usize,
}

impl Default for FrameBudget {
    fn default() -> Self {
        Self { max_operations: 64 }
    }
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct UpdatePlan {
    updates: Vec<Update>,
}

impl UpdatePlan {
    pub fn updates(&self) -> &[Update] {
        &self.updates
    }

    pub fn into_updates(self) -> Vec<Update> {
        self.updates
    }

    pub fn is_empty(&self) -> bool {
        self.updates.is_empty()
    }
}

/// Deterministically select a bounded set of topology mutations for one
/// render frame. Candidate scoring belongs to the terrain-facing adapter; the
/// core only orders declared work and rejects conflicting mutations.
pub fn plan_frame<I>(tree: &Tree, candidates: I, budget: FrameBudget) -> UpdatePlan
where
    I: IntoIterator<Item = LeafCandidate>,
{
    let mut candidates: Vec<_> = candidates.into_iter().collect();
    candidates.sort_by(|a, b| {
        a.class
            .cmp(&b.class)
            .then_with(|| score(*b).total_cmp(&score(*a)))
            .then_with(|| a.node.cmp(&b.node))
    });
    let mut working = tree.clone();
    let mut updates = Vec::with_capacity(budget.max_operations);
    for candidate in candidates {
        if updates.len() == budget.max_operations {
            break;
        }
        let update = match candidate.action {
            CandidateAction::Split => Update::Split(candidate.node),
            CandidateAction::Merge => Update::Merge(candidate.node),
        };
        if working.apply_one(update).is_ok() {
            updates.push(update);
        }
    }
    UpdatePlan { updates }
}

/// Supplies same-domain neighbors for a leaf. Cube-sphere edge transforms and
/// longest-edge-bisection adjacency belong in the terrain adapter, not here.
pub trait NeighborProvider {
    fn for_each_neighbor(&self, node: Node, visit: &mut dyn FnMut(Node));
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct BalanceReport {
    pub splits: usize,
    pub passes: usize,
    pub budget_exhausted: bool,
}

/// Refine coarse neighbors until no reported pair differs by more than one
/// level, or the per-frame split budget is exhausted. This is the shared
/// topology safety pass used before emitting adaptive render geometry.
pub fn enforce_two_to_one<P: NeighborProvider>(
    tree: &mut Tree,
    neighbors: &P,
    max_splits: usize,
) -> Result<BalanceReport, TreeError> {
    let mut report = BalanceReport::default();
    for _ in 0..=tree.max_depth() {
        report.passes += 1;
        let leaves = tree.leaves();
        let mut required = BTreeSet::new();
        for node in leaves {
            let mut visit = |neighbor: Node| {
                if tree.contains(neighbor)
                    && node.depth() > neighbor.depth().saturating_add(1)
                    && neighbor.depth() < tree.max_depth()
                {
                    required.insert(neighbor);
                }
            };
            neighbors.for_each_neighbor(node, &mut visit);
        }
        if required.is_empty() {
            return Ok(report);
        }
        let remaining = max_splits.saturating_sub(report.splits);
        if remaining == 0 {
            report.budget_exhausted = true;
            return Ok(report);
        }
        let required_len = required.len();
        let updates: Vec<_> = required
            .into_iter()
            .take(remaining)
            .map(Update::Split)
            .collect();
        report.splits += updates.len();
        tree.apply_batch(&updates)?;
        if updates.len() < required_len {
            report.budget_exhausted = true;
            return Ok(report);
        }
    }
    Ok(report)
}

fn score(candidate: LeafCandidate) -> f32 {
    let deadline_weight =
        if candidate.time_to_needed_s.is_finite() && candidate.time_to_needed_s > 0.0 {
            1.0 / candidate.time_to_needed_s
        } else {
            0.0
        };
    candidate.projected_error_px.max(0.0)
        + candidate.predicted_error_px.max(0.0) * 0.5
        + deadline_weight.min(1000.0)
}

fn conservative_min(value: f64) -> f32 {
    let result = value as f32;
    if (result as f64) > value {
        f32::from_bits(result.to_bits() - 1)
    } else {
        result
    }
}

fn conservative_max(value: f64) -> f32 {
    let result = value as f32;
    if (result as f64) < value {
        f32::from_bits(result.to_bits() + 1)
    } else {
        result
    }
}

/// A logical adaptive binary tree.
///
/// The initial implementation uses a sorted leaf set because it makes the
/// correctness contract explicit and keeps serialization deterministic. The
/// public operations do not depend on that representation, so packed
/// bitplanes can replace it after workload measurements without changing the
/// terrain or backend APIs.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Tree {
    max_depth: u8,
    leaves: BTreeSet<Node>,
}

impl Tree {
    pub fn new(max_depth: u8) -> Result<Self, TreeError> {
        if max_depth > MAX_SUPPORTED_DEPTH {
            return Err(TreeError::InvalidMaxDepth { depth: max_depth });
        }
        Ok(Self {
            max_depth,
            leaves: BTreeSet::from([Node::root()]),
        })
    }

    pub fn at_depth(max_depth: u8, depth: u8) -> Result<Self, TreeError> {
        let mut tree = Self::new(max_depth)?;
        tree.reset_to_depth(depth)?;
        Ok(tree)
    }

    pub const fn max_depth(&self) -> u8 {
        self.max_depth
    }

    pub fn reset_to_root(&mut self) {
        self.leaves.clear();
        self.leaves.insert(Node::root());
    }

    pub fn reset_to_depth(&mut self, depth: u8) -> Result<(), TreeError> {
        if depth > self.max_depth {
            return Err(TreeError::InvalidMaxDepth { depth });
        }
        self.leaves.clear();
        let first = 1_u64 << depth;
        let count = 1_u64 << depth;
        self.leaves
            .extend((first..first + count).map(|id| Node { id, depth }));
        Ok(())
    }

    pub fn leaf_count(&self) -> usize {
        self.leaves.len()
    }

    pub fn contains(&self, node: Node) -> bool {
        self.leaves.contains(&node)
    }

    /// Returns leaves in the same left-to-right traversal order used by CBT
    /// decode/encode operations, rather than numeric heap-id order.
    pub fn leaves(&self) -> Vec<Node> {
        let mut result = Vec::with_capacity(self.leaves.len());
        self.collect_leaves(Node::root(), &mut result);
        result
    }

    pub fn leaf_at(&self, mut index: usize) -> Option<Node> {
        // Correct but O(leaves) per level: each step recounts a whole subtree.
        // Frame-rate indexed access must go through `LeafList::snapshot`,
        // which pays one listing pass and then serves O(1) index/encode.
        if index >= self.leaf_count() {
            return None;
        }
        let mut node = Node::root();
        loop {
            if self.leaves.contains(&node) {
                return Some(node);
            }
            let [left, right] = node.children()?;
            let left_count = self.count_under(left);
            if index < left_count {
                node = left;
            } else {
                index -= left_count;
                node = right;
            }
        }
    }

    pub fn encode_leaf(&self, node: Node) -> Result<usize, TreeError> {
        // Same note as `leaf_at`: O(subtree) per level. Use `LeafList` for
        // frame-rate encode traffic.
        if !self.contains(node) {
            return Err(TreeError::NotALeaf(node));
        }
        let mut result = 0;
        let mut current = node;
        while let Some(parent) = current.parent() {
            if current.id() & 1 == 1 {
                result += self.count_under(parent.left_child().expect("non-root parent"));
            }
            current = parent;
        }
        Ok(result)
    }

    /// One listing pass that then serves O(1) indexed access and O(1) encode
    /// for the rest of the frame. This is the intended bridge to draw-list
    /// construction: snapshot once, index many times.
    pub fn snapshot(&self) -> LeafList {
        LeafList::snapshot(self)
    }

    pub fn split(&mut self, node: Node) -> Result<[Node; 2], TreeError> {
        if node.depth() >= self.max_depth {
            return Err(TreeError::AtMaximumDepth(node));
        }
        if !self.leaves.remove(&node) {
            return Err(TreeError::NotALeaf(node));
        }
        let children = node.children().expect("depth checked above");
        self.leaves.extend(children);
        Ok(children)
    }

    /// Merge the two leaf children of `parent` into their parent leaf.
    pub fn merge(&mut self, parent: Node) -> Result<(), TreeError> {
        if parent.is_root() {
            return Err(TreeError::CannotMergeRoot);
        }
        let Some([left, right]) = parent.children() else {
            return Err(TreeError::ChildrenNotLeaves(parent));
        };
        if !self.leaves.remove(&left) {
            return Err(TreeError::ChildrenNotLeaves(parent));
        }
        if !self.leaves.remove(&right) {
            self.leaves.insert(left);
            return Err(TreeError::ChildrenNotLeaves(parent));
        }
        self.leaves.insert(parent);
        Ok(())
    }

    pub fn apply_batch(&mut self, updates: &[Update]) -> Result<(), TreeError> {
        // Atomic commit: validate the whole batch on a private copy so a
        // mid-batch conflict leaves `self` untouched.
        let mut candidate = self.clone();
        for update in updates {
            candidate.apply_one(*update)?;
        }
        *self = candidate;
        Ok(())
    }

    /// Single validated mutation without cloning. Used by `plan_frame`, which
    /// already works on a private copy and must not pay O(leaves) per op.
    /// Fallible so a conflicting candidate is skipped while earlier accepted
    /// updates stay committed in the working copy.
    pub fn apply_one(&mut self, update: Update) -> Result<(), TreeError> {
        match update {
            Update::Split(node) => {
                self.split(node)?;
            }
            Update::Merge(node) => {
                self.merge(node)?;
            }
        }
        Ok(())
    }

    pub fn leaf_count_under(&self, node: Node) -> usize {
        self.count_under(node)
    }

    fn count_under(&self, node: Node) -> usize {
        if self.leaves.contains(&node) {
            return 1;
        }
        node.children()
            .map(|[left, right]| self.count_under(left) + self.count_under(right))
            .unwrap_or(0)
    }

    fn collect_leaves(&self, node: Node, result: &mut Vec<Node>) {
        if self.leaves.contains(&node) {
            result.push(node);
        } else if let Some([left, right]) = node.children() {
            self.collect_leaves(left, result);
            self.collect_leaves(right, result);
        }
    }

    /// Stable compact encoding of the observable topology, not the internal
    /// implementation. It is suitable for cache keys and backend snapshots.
    pub fn to_bytes(&self) -> Vec<u8> {
        let leaves = self.leaves();
        let mut bytes = Vec::with_capacity(16 + leaves.len() * 9);
        bytes.extend_from_slice(b"RCBT");
        bytes.push(ENCODING_VERSION);
        bytes.push(self.max_depth);
        bytes.extend_from_slice(&[0, 0]);
        bytes.extend_from_slice(&(leaves.len() as u64).to_le_bytes());
        for node in leaves {
            bytes.extend_from_slice(&node.id().to_le_bytes());
            bytes.push(node.depth());
        }
        bytes
    }

    pub fn from_bytes(bytes: &[u8]) -> Result<Self, TreeError> {
        if bytes.len() < 16 || &bytes[..4] != b"RCBT" {
            return Err(TreeError::InvalidEncoding("invalid RCBT encoding header"));
        }
        if bytes[4] != ENCODING_VERSION {
            return Err(TreeError::InvalidEncoding(
                "unsupported RCBT encoding version",
            ));
        }
        let max_depth = bytes[5];
        let count = u64::from_le_bytes(bytes[8..16].try_into().expect("fixed header"));
        let count = usize::try_from(count)
            .map_err(|_| TreeError::InvalidEncoding("leaf count does not fit usize"))?;
        let expected = 16usize
            .checked_add(
                count
                    .checked_mul(9)
                    .ok_or(TreeError::InvalidEncoding("encoding size overflow"))?,
            )
            .ok_or(TreeError::InvalidEncoding("encoding size overflow"))?;
        if bytes.len() != expected {
            return Err(TreeError::InvalidEncoding(
                "truncated or trailing RCBT topology data",
            ));
        }
        if max_depth > MAX_SUPPORTED_DEPTH {
            return Err(TreeError::InvalidMaxDepth { depth: max_depth });
        }
        let mut leaves = BTreeSet::new();
        for chunk in bytes[16..].as_chunks::<9>().0 {
            let id = u64::from_le_bytes(chunk[..8].try_into().expect("fixed node"));
            let node = Node::new(id, chunk[8])?;
            if !leaves.insert(node) {
                return Err(TreeError::InvalidEncoding("duplicate RCBT leaf"));
            }
        }
        let tree = Self { max_depth, leaves };
        if !tree.is_valid_partition() {
            return Err(TreeError::InvalidEncoding(
                "leaves do not form a complete partition",
            ));
        }
        Ok(tree)
    }

    fn is_valid_partition(&self) -> bool {
        if self.leaves.iter().any(|leaf| leaf.depth() > self.max_depth) {
            return false;
        }
        self.partition_is_complete(Node::root())
    }

    fn partition_is_complete(&self, node: Node) -> bool {
        if self.leaves.contains(&node) {
            return true;
        }
        let Some([left, right]) = node.children() else {
            return false;
        };
        self.has_descendant(left)
            && self.has_descendant(right)
            && self.partition_is_complete(left)
            && self.partition_is_complete(right)
    }

    fn has_descendant(&self, node: Node) -> bool {
        self.leaves.iter().any(|leaf| {
            leaf.depth() >= node.depth()
                && (leaf.id() >> (leaf.depth() - node.depth())) == node.id()
        })
    }
}

/// Compact per-frame leaf index: one `leaves()` pass, then O(1) indexed
/// access and O(1) encode for the rest of the frame. Build once per topology
/// commit, not once per query.
///
/// The rank map is keyed by a single packed `u64` (`id << 6 | depth`), so no
/// hasher ever sees a struct: one integer multiply per lookup.
#[derive(Debug, Clone, Default)]
pub struct LeafList {
    ordered: Vec<Node>,
    rank: HashMap<u64, usize>,
}

fn leaf_key(node: Node) -> u64 {
    (node.id() << 6) | node.depth() as u64
}

impl LeafList {
    pub fn snapshot(tree: &Tree) -> Self {
        let ordered = tree.leaves();
        let mut rank = HashMap::with_capacity(ordered.len());
        for (index, node) in ordered.iter().copied().enumerate() {
            rank.insert(leaf_key(node), index);
        }
        Self { ordered, rank }
    }

    pub fn len(&self) -> usize {
        self.ordered.len()
    }

    pub fn is_empty(&self) -> bool {
        self.ordered.is_empty()
    }

    pub fn index(&self, index: usize) -> Option<Node> {
        self.ordered.get(index).copied()
    }

    pub fn encode(&self, node: Node) -> Option<usize> {
        self.rank.get(&leaf_key(node)).copied()
    }

    pub fn iter(&self) -> impl Iterator<Item = Node> + '_ {
        self.ordered.iter().copied()
    }

    pub fn into_vec(self) -> Vec<Node> {
        self.ordered
    }
}

/// A compact baked height page with quantized residual samples.
///
/// The page has no cube-sphere address on purpose: address ownership belongs to
/// the terrain layer. It can therefore be used by a spherical terrain,
/// heightfield test scene, or another continuous surface provider.
#[derive(Debug, Clone, PartialEq)]
pub struct HeightPage {
    grid_size: u32,
    base_height_m: f32,
    residual_scale_m: f32,
    min_height_m: f32,
    max_height_m: f32,
    max_residual_error_m: f32,
    max_slope_bound: f32,
    residuals: Vec<i16>,
}

#[derive(Debug, Clone, PartialEq)]
pub enum HeightPageError {
    InvalidGrid,
    SampleCountMismatch { expected: usize, actual: usize },
    InvalidErrorBudget,
    InvalidEncoding(&'static str),
}

impl fmt::Display for HeightPageError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::InvalidGrid => f.write_str("height page grid must be non-zero"),
            Self::SampleCountMismatch { expected, actual } => {
                write!(f, "height page needs {expected} samples, got {actual}")
            }
            Self::InvalidErrorBudget => {
                f.write_str("height page error budget must be finite and non-negative")
            }
            Self::InvalidEncoding(message) => f.write_str(message),
        }
    }
}

impl std::error::Error for HeightPageError {}

impl HeightPage {
    /// Bake a square page. The resulting quantization error is checked against
    /// the requested absolute budget, including f32 storage round-off.
    pub fn bake(
        samples: &[f64],
        grid_size: u32,
        max_error_m: f64,
    ) -> Result<Self, HeightPageError> {
        if grid_size == 0 {
            return Err(HeightPageError::InvalidGrid);
        }
        if !max_error_m.is_finite() || max_error_m < 0.0 {
            return Err(HeightPageError::InvalidErrorBudget);
        }
        let grid = usize::try_from(grid_size).map_err(|_| HeightPageError::InvalidGrid)?;
        let expected = grid.checked_mul(grid).ok_or(HeightPageError::InvalidGrid)?;
        if samples.len() != expected {
            return Err(HeightPageError::SampleCountMismatch {
                expected,
                actual: samples.len(),
            });
        }
        if samples.iter().any(|sample| !sample.is_finite()) {
            return Err(HeightPageError::InvalidEncoding(
                "height page samples must be finite",
            ));
        }
        if samples.iter().any(|sample| !(*sample as f32).is_finite()) {
            return Err(HeightPageError::InvalidEncoding(
                "height page samples must fit f32 storage",
            ));
        }
        let base = samples.iter().sum::<f64>() / samples.len() as f64;
        let max_abs_residual = samples
            .iter()
            .map(|sample| (sample - base).abs())
            .fold(0.0, f64::max);
        let scale = max_abs_residual / i16::MAX as f64;
        let residuals: Vec<i16> = samples
            .iter()
            .map(|sample| {
                if scale == 0.0 {
                    0
                } else {
                    ((sample - base) / scale)
                        .round()
                        .clamp(i16::MIN as f64, i16::MAX as f64) as i16
                }
            })
            .collect();
        let base_height_m = base as f32;
        let residual_scale_m = scale as f32;
        let max_residual_error_m = samples
            .iter()
            .zip(&residuals)
            .map(|(sample, residual)| {
                (sample - (base_height_m as f64 + *residual as f64 * residual_scale_m as f64)).abs()
            })
            .fold(0.0, f64::max);
        if max_residual_error_m > max_error_m + f64::EPSILON.max(max_error_m * 1e-6) {
            return Err(HeightPageError::InvalidErrorBudget);
        }
        let min_height_m = conservative_min(samples.iter().copied().fold(f64::INFINITY, f64::min));
        let max_height_m =
            conservative_max(samples.iter().copied().fold(f64::NEG_INFINITY, f64::max));
        let max_slope_bound = samples
            .iter()
            .enumerate()
            .map(|(index, sample)| {
                let x = index % grid;
                let y = index / grid;
                let right = if x + 1 < grid {
                    samples[index + 1]
                } else {
                    *sample
                };
                let down = if y + 1 < grid {
                    samples[index + grid]
                } else {
                    *sample
                };
                (right - sample).abs().max((down - sample).abs())
            })
            .fold(0.0, f64::max);
        let max_slope_bound = conservative_max(max_slope_bound);
        let max_residual_error_m = conservative_max(max_residual_error_m);
        Ok(Self {
            grid_size,
            base_height_m,
            residual_scale_m,
            min_height_m,
            max_height_m,
            max_residual_error_m,
            max_slope_bound,
            residuals,
        })
    }

    pub const fn grid_size(&self) -> u32 {
        self.grid_size
    }

    pub const fn base_height_m(&self) -> f32 {
        self.base_height_m
    }

    pub const fn residual_scale_m(&self) -> f32 {
        self.residual_scale_m
    }

    pub const fn min_height_m(&self) -> f32 {
        self.min_height_m
    }

    pub const fn max_height_m(&self) -> f32 {
        self.max_height_m
    }

    pub const fn max_residual_error_m(&self) -> f32 {
        self.max_residual_error_m
    }

    pub const fn max_slope_bound(&self) -> f32 {
        self.max_slope_bound
    }

    pub fn residuals(&self) -> &[i16] {
        &self.residuals
    }

    pub fn residual_bytes(&self) -> Vec<u8> {
        let mut bytes = Vec::with_capacity(self.residuals.len() * 2);
        for residual in &self.residuals {
            bytes.extend_from_slice(&residual.to_le_bytes());
        }
        bytes
    }

    pub fn decoded_sample(&self, x: u32, y: u32) -> Option<f32> {
        if x >= self.grid_size || y >= self.grid_size {
            return None;
        }
        let index = y as usize * self.grid_size as usize + x as usize;
        Some(self.base_height_m + self.residuals[index] as f32 * self.residual_scale_m)
    }

    /// Bilinear query in normalized page coordinates. The returned value is
    /// the baked representation and is never an analytic residual query.
    pub fn sample(&self, u: f32, v: f32) -> f32 {
        let max = self.grid_size.saturating_sub(1) as f32;
        let x = (u.clamp(0.0, 1.0) * max).floor() as u32;
        let y = (v.clamp(0.0, 1.0) * max).floor() as u32;
        let x1 = (x + 1).min(self.grid_size - 1);
        let y1 = (y + 1).min(self.grid_size - 1);
        let tx = (u.clamp(0.0, 1.0) * max - x as f32).clamp(0.0, 1.0);
        let ty = (v.clamp(0.0, 1.0) * max - y as f32).clamp(0.0, 1.0);
        let h00 = self.decoded_sample(x, y).unwrap_or(self.base_height_m);
        let h10 = self.decoded_sample(x1, y).unwrap_or(h00);
        let h01 = self.decoded_sample(x, y1).unwrap_or(h00);
        let h11 = self.decoded_sample(x1, y1).unwrap_or(h00);
        let top = h00 + (h10 - h00) * tx;
        let bottom = h01 + (h11 - h01) * tx;
        top + (bottom - top) * ty
    }

    pub fn to_bytes(&self) -> Vec<u8> {
        let mut bytes = Vec::with_capacity(40 + self.residuals.len() * 2);
        bytes.extend_from_slice(b"RHPG");
        bytes.push(PAGE_ENCODING_VERSION);
        bytes.extend_from_slice(&[0, 0, 0]);
        bytes.extend_from_slice(&self.grid_size.to_le_bytes());
        for value in [
            self.base_height_m,
            self.residual_scale_m,
            self.min_height_m,
            self.max_height_m,
            self.max_residual_error_m,
            self.max_slope_bound,
        ] {
            bytes.extend_from_slice(&value.to_le_bytes());
        }
        for residual in &self.residuals {
            bytes.extend_from_slice(&residual.to_le_bytes());
        }
        bytes
    }

    pub fn from_bytes(bytes: &[u8]) -> Result<Self, HeightPageError> {
        const HEADER: usize = 36;
        if bytes.len() < HEADER || &bytes[..4] != b"RHPG" || bytes[4] != PAGE_ENCODING_VERSION {
            return Err(HeightPageError::InvalidEncoding(
                "invalid height page header",
            ));
        }
        let grid_size = u32::from_le_bytes(bytes[8..12].try_into().expect("fixed header"));
        if grid_size == 0 {
            return Err(HeightPageError::InvalidGrid);
        }
        let count = (grid_size as usize)
            .checked_mul(grid_size as usize)
            .ok_or(HeightPageError::InvalidGrid)?;
        let expected = HEADER
            .checked_add(count.checked_mul(2).ok_or(HeightPageError::InvalidGrid)?)
            .ok_or(HeightPageError::InvalidGrid)?;
        if bytes.len() != expected {
            return Err(HeightPageError::InvalidEncoding(
                "invalid height page length",
            ));
        }
        let read_f32 = |offset: usize| {
            f32::from_le_bytes(bytes[offset..offset + 4].try_into().expect("fixed field"))
        };
        let residuals = bytes[HEADER..]
            .as_chunks::<2>()
            .0
            .iter()
            .map(|chunk| i16::from_le_bytes(*chunk))
            .collect();
        let page = Self {
            grid_size,
            base_height_m: read_f32(12),
            residual_scale_m: read_f32(16),
            min_height_m: read_f32(20),
            max_height_m: read_f32(24),
            max_residual_error_m: read_f32(28),
            max_slope_bound: read_f32(32),
            residuals,
        };
        if !page.base_height_m.is_finite()
            || !page.residual_scale_m.is_finite()
            || page.residual_scale_m < 0.0
            || !page.min_height_m.is_finite()
            || !page.max_height_m.is_finite()
            || page.min_height_m > page.max_height_m
            || !page.max_residual_error_m.is_finite()
            || page.max_residual_error_m < 0.0
            || !page.max_slope_bound.is_finite()
            || page.max_slope_bound < 0.0
        {
            return Err(HeightPageError::InvalidEncoding(
                "invalid height page metadata",
            ));
        }
        Ok(page)
    }
}

/// Logical kernels understood by backend adapters. These are semantic stages,
/// not shader names or a particular graphics API's pipeline objects.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum CbtKernel {
    Classify,
    Bisect,
    Simplify,
    Reduce,
    CompactLeaves,
    BuildDrawList,
    GenerateVertices,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct CbtCapabilities {
    pub subgroup_size: Option<u32>,
    pub subgroup_ballot: bool,
    pub storage_u64: bool,
    pub atomic_u64: bool,
    pub indirect_draw: bool,
    pub indirect_count: bool,
    pub persistent_mapping: bool,
    pub device_address: bool,
    pub cooperative_matrix: bool,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum BufferUsage {
    Storage,
    Uniform,
    Indirect,
    Readback,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct BufferDesc {
    pub size_bytes: u64,
    pub usage: BufferUsage,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct BufferBinding<B = ()> {
    pub slot: u32,
    pub offset_bytes: u64,
    pub size_bytes: u64,
    pub resource: B,
}

#[derive(Debug, Clone, PartialEq, Eq, Default)]
pub struct CbtBindings<B = ()> {
    pub buffers: Vec<BufferBinding<B>>,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CbtBarrier {
    StorageToStorage,
    StorageToIndirect,
    StorageToVertex,
    HostToStorage,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct DispatchSize {
    pub x: u32,
    pub y: u32,
    pub z: u32,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct CbtMetrics {
    pub dispatches: u64,
    pub active_leaves: u64,
    pub bytes_touched: u64,
    pub uploaded_bytes: u64,
}

/// Backend boundary for CBT work. Associated types keep graphics handles in
/// the adapter crate while the logical API remains reusable and portable.
pub trait CbtBackend {
    type Buffer;
    type Pipeline;
    type Commands;
    type Error: std::error::Error + Send + Sync + 'static;

    fn capabilities(&self) -> CbtCapabilities;
    fn create_buffer(&self, desc: BufferDesc) -> Result<Self::Buffer, Self::Error>;
    fn create_pipeline(&self, kernel: CbtKernel) -> Result<Self::Pipeline, Self::Error>;
    fn begin_commands(&self) -> Self::Commands;
    fn dispatch(
        &self,
        commands: &mut Self::Commands,
        pipeline: &Self::Pipeline,
        bindings: &CbtBindings<Self::Buffer>,
        groups: DispatchSize,
    ) -> Result<(), Self::Error>;
    fn barrier(&self, commands: &mut Self::Commands, barrier: CbtBarrier);
    fn submit(&self, commands: Self::Commands) -> Result<CbtMetrics, Self::Error>;
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn split_merge_and_traversal_are_inverse_operations() {
        let mut tree = Tree::new(8).unwrap();
        let [left, right] = tree.split(Node::root()).unwrap();
        tree.split(left).unwrap();
        assert_eq!(tree.leaf_count(), 3);
        let leaves = tree.leaves();
        for (index, node) in leaves.iter().copied().enumerate() {
            assert_eq!(tree.leaf_at(index), Some(node));
            assert_eq!(tree.encode_leaf(node), Ok(index));
        }
        tree.merge(left).unwrap();
        assert_eq!(
            tree.leaves(),
            vec![Node::root().left_child().unwrap(), right]
        );
        tree.merge(Node::root()).unwrap_err();
    }

    #[test]
    fn batch_is_atomic_on_failure() {
        let mut tree = Tree::new(4).unwrap();
        let before = tree.to_bytes();
        let result = tree.apply_batch(&[Update::Split(Node::root()), Update::Split(Node::root())]);
        assert!(result.is_err());
        assert_eq!(tree.to_bytes(), before);
    }

    #[test]
    fn encoding_round_trip_is_deterministic() {
        let mut tree = Tree::at_depth(6, 2).unwrap();
        tree.merge(Node::new(2, 1).unwrap()).unwrap();
        tree.split(Node::new(6, 2).unwrap()).unwrap();
        let bytes = tree.to_bytes();
        assert_eq!(Tree::from_bytes(&bytes).unwrap(), tree);
        assert_eq!(bytes, tree.to_bytes());
    }

    #[test]
    fn differential_sequence_matches_reference_oracle() {
        let mut actual = Tree::new(10).unwrap();
        let mut reference = thessa_rcbt_ref::ReferenceTree::new(10).unwrap();
        let sequence = [
            Update::Split(Node::root()),
            Update::Split(Node::new(2, 1).unwrap()),
            Update::Split(Node::new(3, 1).unwrap()),
            Update::Merge(Node::new(2, 1).unwrap()),
            Update::Split(Node::new(2, 1).unwrap()),
        ];
        for update in sequence {
            match update {
                Update::Split(node) => {
                    actual.split(node).unwrap();
                    reference.split(node.id(), node.depth()).unwrap();
                }
                Update::Merge(node) => {
                    actual.merge(node).unwrap();
                    reference.merge(node.id(), node.depth()).unwrap();
                }
            }
            let expected: Vec<_> = reference
                .leaves()
                .into_iter()
                .map(|(id, depth)| Node::new(id, depth).unwrap())
                .collect();
            assert_eq!(actual.leaves(), expected);
        }
    }

    #[test]
    fn frame_plan_is_bounded_and_deterministic() {
        let tree = Tree::new(6).unwrap();
        let candidates = [
            LeafCandidate {
                node: Node::root(),
                action: CandidateAction::Split,
                class: WorkClass::PredictedGeometry,
                projected_error_px: 2.0,
                predicted_error_px: 1.0,
                time_to_needed_s: 1.0,
            },
            LeafCandidate {
                node: Node::root(),
                action: CandidateAction::Split,
                class: WorkClass::CoverageRepair,
                projected_error_px: 1.0,
                predicted_error_px: 1.0,
                time_to_needed_s: 1.0,
            },
        ];
        let plan = plan_frame(&tree, candidates, FrameBudget { max_operations: 1 });
        assert_eq!(plan.updates(), &[Update::Split(Node::root())]);
        assert_eq!(
            plan.updates(),
            plan_frame(&tree, candidates, FrameBudget { max_operations: 1 }).updates()
        );
    }

    #[test]
    fn leaf_list_snapshot_matches_walk_queries() {
        let mut tree = Tree::at_depth(8, 3).unwrap();
        tree.split(Node::new(8, 3).unwrap()).unwrap();
        tree.split(Node::new(9, 3).unwrap()).unwrap();
        let list = tree.snapshot();
        assert_eq!(list.len(), tree.leaf_count());
        for (index, node) in tree.leaves().iter().copied().enumerate() {
            assert_eq!(list.index(index), Some(node));
            assert_eq!(tree.leaf_at(index), Some(node));
            assert_eq!(list.encode(node), Some(index));
            assert_eq!(tree.encode_leaf(node), Ok(index));
        }
        assert_eq!(list.index(list.len()), None);
    }

    #[test]
    fn height_page_respects_absolute_error_and_round_trips() {
        let source = [0.0, 10.0, 20.0, 30.0, 40.0, 50.0, 60.0, 70.0, 80.0];
        let page = HeightPage::bake(&source, 3, 0.01).unwrap();
        assert!(page.max_residual_error_m() <= 0.01);
        assert_eq!(page.min_height_m(), 0.0);
        assert_eq!(page.max_height_m(), 80.0);
        assert!((page.sample(0.5, 0.5) - 40.0).abs() < 0.01);
        assert_eq!(HeightPage::from_bytes(&page.to_bytes()).unwrap(), page);
    }

    #[test]
    fn balance_pass_refines_coarse_neighbors_with_a_bounded_budget() {
        struct Siblings;
        impl NeighborProvider for Siblings {
            fn for_each_neighbor(&self, node: Node, visit: &mut dyn FnMut(Node)) {
                if let Some(sibling) = node
                    .parent()
                    .and_then(|parent| parent.parent())
                    .and_then(Node::sibling)
                {
                    visit(sibling);
                }
            }
        }

        let mut tree = Tree::new(6).unwrap();
        let left = Node::root().left_child().unwrap();
        tree.split(Node::root()).unwrap();
        tree.split(left).unwrap();
        tree.split(left.left_child().unwrap()).unwrap();
        let report = enforce_two_to_one(&mut tree, &Siblings, 0).unwrap();
        assert!(report.budget_exhausted);
        assert_eq!(report.splits, 0);
        assert!(tree.contains(Node::root().right_child().unwrap()));
        enforce_two_to_one(&mut tree, &Siblings, 8).unwrap();
        assert!(!tree.contains(Node::root().right_child().unwrap()));
    }

    #[test]
    fn malformed_page_is_rejected_without_allocating_unbounded_data() {
        let mut bytes = vec![0; 36];
        bytes[..4].copy_from_slice(b"RHPG");
        bytes[4] = PAGE_ENCODING_VERSION;
        bytes[8..12].copy_from_slice(&u32::MAX.to_le_bytes());
        assert!(HeightPage::from_bytes(&bytes).is_err());
    }
}
