//! Fixed-capacity allocation for active CBT bisectors.
//!
//! The large_cbt design uses a CBT as a memory-pool manager: subdivision
//! allocates child primitives and simplification releases them again.  The
//! regular [`crate::Tree`] intentionally keeps only topology, so it cannot
//! provide stable allocation handles to a renderer or a concurrent adapter.
//! This pool supplies that missing contract without copying the upstream
//! implementation or tying the core to a graphics API.
//!
//! A pool contains the current leaf partition.  Splitting a bisector reuses
//! its slot for the left child and allocates exactly one additional slot for
//! the right child.  Merging does the inverse.  Consequently a pool with
//! capacity `N` can represent at most `N` active leaves and never grows after
//! construction.  Handles carry a generation, so a released or replaced slot
//! cannot be accidentally used by a later operation.

use std::fmt;

use crate::{MAX_SUPPORTED_DEPTH, Node};

/// A stable reference to one pool slot.
///
/// The slot index is stable while the generation changes whenever the slot's
/// node is replaced or released.  Keep the handle instead of the slot index
/// when an adapter caches per-bisector data.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct BisectorHandle {
    slot: u32,
    generation: u32,
}

impl BisectorHandle {
    pub const fn slot(self) -> u32 {
        self.slot
    }

    pub const fn generation(self) -> u32 {
        self.generation
    }
}

/// The value stored for an active bisector.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Bisector {
    node: Node,
}

impl Bisector {
    pub const fn node(self) -> Node {
        self.node
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
struct Slot {
    generation: u32,
    value: Option<Bisector>,
}

/// Errors returned by [`BisectorPool`] operations.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum BisectorPoolError {
    InvalidCapacity,
    CapacityTooLarge { capacity: usize },
    InvalidMaxDepth { depth: u8 },
    CapacityExhausted { capacity: usize },
    InvalidHandle(BisectorHandle),
    NotALeaf(Node),
    AtMaximumDepth(Node),
    CannotMergeRoot,
    ChildrenNotLeaves(Node),
    GenerationExhausted(BisectorHandle),
}

impl fmt::Display for BisectorPoolError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::InvalidCapacity => f.write_str("bisector pool capacity must be non-zero"),
            Self::CapacityTooLarge { capacity } => {
                write!(
                    f,
                    "bisector pool capacity {capacity} exceeds u32 handle space"
                )
            }
            Self::InvalidMaxDepth { depth } => {
                write!(f, "bisector pool max depth {depth} is unsupported")
            }
            Self::CapacityExhausted { capacity } => {
                write!(f, "bisector pool capacity {capacity} is exhausted")
            }
            Self::InvalidHandle(handle) => write!(f, "invalid bisector handle {handle:?}"),
            Self::NotALeaf(node) => write!(f, "node {node:?} is not an active bisector"),
            Self::AtMaximumDepth(node) => write!(f, "node {node:?} is at maximum depth"),
            Self::CannotMergeRoot => f.write_str("the root bisector cannot be merged"),
            Self::ChildrenNotLeaves(node) => {
                write!(f, "children of {node:?} are not both active bisectors")
            }
            Self::GenerationExhausted(handle) => {
                write!(f, "generation exhausted for bisector handle {handle:?}")
            }
        }
    }
}

impl std::error::Error for BisectorPoolError {}

/// A fixed-capacity pool containing the active leaf bisectors of a CBT.
///
/// The allocation vectors are sized once by [`Self::new`].  `split` and
/// `merge` only update existing slots and the free-index stack; they never
/// allocate or grow a collection.  Active handles are returned in slot order
/// by [`Self::handles`], which is deterministic but intentionally independent
/// of left-to-right leaf order.
#[derive(Debug, Clone)]
pub struct BisectorPool {
    max_depth: u8,
    slots: Vec<Slot>,
    free: Vec<u32>,
    active_count: usize,
}

impl BisectorPool {
    /// Creates a pool containing the root bisector.
    pub fn new(max_depth: u8, capacity: usize) -> Result<Self, BisectorPoolError> {
        if capacity == 0 {
            return Err(BisectorPoolError::InvalidCapacity);
        }
        if max_depth > MAX_SUPPORTED_DEPTH {
            return Err(BisectorPoolError::InvalidMaxDepth { depth: max_depth });
        }
        let capacity_u32 = u32::try_from(capacity)
            .map_err(|_| BisectorPoolError::CapacityTooLarge { capacity })?;
        let slots = vec![
            Slot {
                generation: 1,
                value: None,
            };
            capacity
        ];
        // Pop returns the lowest free slot, which makes allocations
        // deterministic while keeping the stack itself fixed-capacity.
        let free = (1..capacity_u32).rev().collect();
        let mut pool = Self {
            max_depth,
            slots,
            free,
            active_count: 1,
        };
        pool.slots[0].value = Some(Bisector { node: Node::root() });
        Ok(pool)
    }

    /// Creates a pool with the same leaf partition as an existing tree.
    ///
    /// This is the hand-off point for an adapter migrating from the topology
    /// only [`crate::Tree`]. The capacity check is enforced by the same split
    /// path used for later updates; a failed build simply returns the partial
    /// pool and leaves the caller's tree untouched.
    pub fn from_tree(tree: &crate::Tree, capacity: usize) -> Result<Self, BisectorPoolError> {
        let mut pool = Self::new(tree.max_depth(), capacity)?;
        materialize_pool_from_tree(&mut pool, tree, Node::root())?;
        Ok(pool)
    }

    /// Replaces the active partition while retaining the pool allocation.
    ///
    /// Every slot generation is advanced before the replacement topology is
    /// materialized, including currently free slots. This makes handles from
    /// before the reset unusable even if a slot is immediately reused. The
    /// operation is transactional: generation exhaustion or an over-capacity
    /// target leaves this pool unchanged.
    pub fn reset_to_tree(&mut self, tree: &crate::Tree) -> Result<(), BisectorPoolError> {
        if tree.max_depth() != self.max_depth {
            return Err(BisectorPoolError::InvalidMaxDepth {
                depth: tree.max_depth(),
            });
        }
        if tree.leaf_count() > self.capacity() {
            return Err(BisectorPoolError::CapacityExhausted {
                capacity: self.capacity(),
            });
        }
        for index in 0..self.capacity() {
            if self.slots[index].generation == u32::MAX {
                return Err(BisectorPoolError::GenerationExhausted(
                    self.handle_at(index),
                ));
            }
        }

        let mut candidate = self.clone();
        for slot in &mut candidate.slots {
            slot.generation += 1;
            slot.value = None;
        }
        candidate.free.clear();
        candidate
            .free
            .extend((1..candidate.capacity() as u32).rev());
        candidate.active_count = 1;
        candidate.slots[0].value = Some(Bisector { node: Node::root() });

        materialize_pool_from_tree(&mut candidate, tree, Node::root())?;
        debug_assert!(candidate.is_valid_partition());
        *self = candidate;
        Ok(())
    }

    pub const fn max_depth(&self) -> u8 {
        self.max_depth
    }

    pub const fn capacity(&self) -> usize {
        self.slots.len()
    }

    pub const fn active_count(&self) -> usize {
        self.active_count
    }

    pub const fn free_count(&self) -> usize {
        self.slots.len() - self.active_count
    }

    pub const fn is_full(&self) -> bool {
        self.active_count == self.slots.len()
    }

    pub fn root_handle(&self) -> BisectorHandle {
        self.handle_at(0)
    }

    /// Finds the active handle for a node by scanning the fixed slot array.
    /// Adapters that cache handles should use [`Self::get`] and avoid this
    /// lookup on the frame-rate path.
    pub fn handle_for(&self, node: Node) -> Option<BisectorHandle> {
        self.slots.iter().enumerate().find_map(|(slot, entry)| {
            (entry.value == Some(Bisector { node })).then(|| self.handle_at(slot))
        })
    }

    /// Returns a bisector only when the handle still names its generation.
    pub fn get(&self, handle: BisectorHandle) -> Option<Bisector> {
        let slot = self.slots.get(handle.slot as usize)?;
        (slot.generation == handle.generation).then_some(slot.value?)
    }

    /// Returns all active handles in deterministic slot order.
    pub fn handles(&self) -> Vec<BisectorHandle> {
        self.slots
            .iter()
            .enumerate()
            .filter(|(_, entry)| entry.value.is_some())
            .map(|(slot, _)| self.handle_at(slot))
            .collect()
    }

    /// Returns all active nodes sorted in CBT traversal order.
    pub fn nodes(&self) -> Vec<Node> {
        let mut nodes: Vec<_> = self
            .slots
            .iter()
            .filter_map(|slot| slot.value.map(Bisector::node))
            .collect();
        nodes.sort_unstable_by_key(|node| {
            // A leaf occupies one interval at max depth. Interval starts are
            // therefore exactly the depth-first left-to-right order used by
            // `Tree::leaves`, even when neighboring leaves have different
            // depths.
            node.id() << (self.max_depth - node.depth())
        });
        nodes
    }

    /// Checks that active slots form one complete, non-overlapping partition
    /// of the root domain and that the cached active count is exact.
    pub fn is_valid_partition(&self) -> bool {
        if self.active_count == 0 || self.active_count > self.capacity() {
            return false;
        }
        let active_count = self
            .slots
            .iter()
            .filter(|slot| slot.value.is_some())
            .count();
        if active_count != self.active_count {
            return false;
        }
        fn complete(pool: &BisectorPool, node: Node) -> Option<usize> {
            if pool.handle_for(node).is_some() {
                return Some(1);
            }
            let [left, right] = node.children()?;
            Some(complete(pool, left)? + complete(pool, right)?)
        }
        complete(self, Node::root()) == Some(self.active_count)
    }

    /// Splits the bisector addressed by `handle` and returns both child
    /// handles. The parent slot becomes the left child; one free slot stores
    /// the right child. Capacity exhaustion leaves the pool unchanged.
    pub fn split(
        &mut self,
        handle: BisectorHandle,
    ) -> Result<[BisectorHandle; 2], BisectorPoolError> {
        let index = self.validate(handle)?;
        let parent = self.slots[index]
            .value
            .expect("validated active bisector")
            .node;
        if parent.depth() >= self.max_depth {
            return Err(BisectorPoolError::AtMaximumDepth(parent));
        }
        if self.free.is_empty() {
            return Err(BisectorPoolError::CapacityExhausted {
                capacity: self.capacity(),
            });
        }
        let [left, right] = parent.children().expect("depth checked above");
        let next_generation = Self::next_generation(handle)?;
        let right_index = self.free.pop().expect("checked free slot");
        self.slots[index] = Slot {
            generation: next_generation,
            value: Some(Bisector { node: left }),
        };
        self.slots[right_index as usize].value = Some(Bisector { node: right });
        self.active_count += 1;
        Ok([self.handle_at(index), self.handle_at(right_index as usize)])
    }

    /// Splits an active node after resolving its current handle.
    pub fn split_node(&mut self, node: Node) -> Result<[BisectorHandle; 2], BisectorPoolError> {
        let handle = self
            .handle_for(node)
            .ok_or(BisectorPoolError::NotALeaf(node))?;
        self.split(handle)
    }

    /// Merges two active child bisectors into their parent. The left child
    /// slot is reused for the parent and the right child slot is released.
    pub fn merge(&mut self, parent: Node) -> Result<BisectorHandle, BisectorPoolError> {
        if parent.is_root() {
            return Err(BisectorPoolError::CannotMergeRoot);
        }
        let [left, right] = parent
            .children()
            .ok_or(BisectorPoolError::ChildrenNotLeaves(parent))?;
        let left_handle = self
            .handle_for(left)
            .ok_or(BisectorPoolError::ChildrenNotLeaves(parent))?;
        let right_handle = self
            .handle_for(right)
            .ok_or(BisectorPoolError::ChildrenNotLeaves(parent))?;
        self.merge_handles(parent, left_handle, right_handle)
    }

    /// Merges children using cached handles, keeping the mutation itself
    /// O(1). `merge` remains available for node-based callers that do not yet
    /// retain handles between topology commits.
    pub fn merge_handles(
        &mut self,
        parent: Node,
        left_handle: BisectorHandle,
        right_handle: BisectorHandle,
    ) -> Result<BisectorHandle, BisectorPoolError> {
        if parent.is_root() {
            return Err(BisectorPoolError::CannotMergeRoot);
        }
        let [left, right] = parent
            .children()
            .ok_or(BisectorPoolError::ChildrenNotLeaves(parent))?;
        let left_value = self
            .get(left_handle)
            .ok_or(BisectorPoolError::InvalidHandle(left_handle))?;
        let right_value = self
            .get(right_handle)
            .ok_or(BisectorPoolError::InvalidHandle(right_handle))?;
        if left_value.node() != left || right_value.node() != right {
            return Err(BisectorPoolError::ChildrenNotLeaves(parent));
        }
        let left_index = left_handle.slot as usize;
        let right_index = right_handle.slot as usize;
        let parent_generation = Self::next_generation(left_handle)?;
        let right_generation = Self::next_generation(right_handle)?;
        self.slots[left_index] = Slot {
            generation: parent_generation,
            value: Some(Bisector { node: parent }),
        };
        self.slots[right_index] = Slot {
            generation: right_generation,
            value: None,
        };
        self.free.push(right_handle.slot);
        self.active_count -= 1;
        Ok(self.handle_at(left_index))
    }

    fn validate(&self, handle: BisectorHandle) -> Result<usize, BisectorPoolError> {
        let index = handle.slot as usize;
        let Some(slot) = self.slots.get(index) else {
            return Err(BisectorPoolError::InvalidHandle(handle));
        };
        if slot.generation != handle.generation || slot.value.is_none() {
            return Err(BisectorPoolError::InvalidHandle(handle));
        }
        Ok(index)
    }

    fn handle_at(&self, slot: usize) -> BisectorHandle {
        BisectorHandle {
            slot: slot as u32,
            generation: self.slots[slot].generation,
        }
    }

    fn next_generation(handle: BisectorHandle) -> Result<u32, BisectorPoolError> {
        handle
            .generation
            .checked_add(1)
            .ok_or(BisectorPoolError::GenerationExhausted(handle))
    }
}

fn materialize_pool_from_tree(
    pool: &mut BisectorPool,
    tree: &crate::Tree,
    node: Node,
) -> Result<(), BisectorPoolError> {
    if tree.contains(node) {
        return Ok(());
    }
    let children = pool.split_node(node)?;
    let left = pool.get(children[0]).expect("new child handle").node();
    let right = pool.get(children[1]).expect("new child handle").node();
    materialize_pool_from_tree(pool, tree, left)?;
    materialize_pool_from_tree(pool, tree, right)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::Tree;

    #[test]
    fn fixed_pool_tracks_tree_topology_and_reuses_slots() {
        let mut tree = Tree::new(6).unwrap();
        let mut pool = BisectorPool::new(6, 8).unwrap();
        let root = pool.root_handle();
        let [left, right] = pool.split(root).unwrap();
        tree.split(Node::root()).unwrap();
        assert_eq!(pool.nodes(), tree.leaves());
        assert_eq!(pool.active_count(), 2);

        let [left_left, left_right] = pool.split(left).unwrap();
        tree.split(Node::new(2, 1).unwrap()).unwrap();
        assert_eq!(pool.nodes(), tree.leaves());
        assert_eq!(pool.get(root), None);
        assert_eq!(pool.get(left), None);
        assert_eq!(pool.get(right).unwrap().node(), Node::new(3, 1).unwrap());

        let merged = pool
            .merge_handles(Node::new(2, 1).unwrap(), left_left, left_right)
            .unwrap();
        tree.merge(Node::new(2, 1).unwrap()).unwrap();
        assert_eq!(pool.nodes(), tree.leaves());
        assert_eq!(pool.get(left_left), None);
        assert_eq!(pool.get(left_right), None);
        assert_eq!(pool.get(merged).unwrap().node(), Node::new(2, 1).unwrap());

        // The released right-child slot is reused, but its generation keeps
        // the old handle invalid.
        let [new_left, new_right] = pool.split_node(Node::new(3, 1).unwrap()).unwrap();
        assert_eq!(pool.get(right), None);
        assert_eq!(new_left.slot(), right.slot());
        assert_eq!(new_right.slot(), left_right.slot());
        assert_ne!(new_right.generation(), left_right.generation());
        assert_eq!(pool.active_count(), 3);
        assert!(pool.is_valid_partition());
    }

    #[test]
    fn from_tree_rebuilds_a_mixed_depth_partition() {
        let mut tree = Tree::new(6).unwrap();
        tree.split(Node::root()).unwrap();
        tree.split(Node::new(2, 1).unwrap()).unwrap();
        tree.split(Node::new(4, 2).unwrap()).unwrap();
        let pool = BisectorPool::from_tree(&tree, tree.leaf_count()).unwrap();
        assert_eq!(pool.nodes(), tree.leaves());
        assert_eq!(pool.active_count(), tree.leaf_count());
        assert_eq!(pool.free_count(), 0);
        assert!(pool.is_valid_partition());
    }

    #[test]
    fn reset_invalidates_all_handles_and_rebuilds_without_growth() {
        let mut initial = Tree::new(6).unwrap();
        initial.split(Node::root()).unwrap();
        initial.split(Node::new(2, 1).unwrap()).unwrap();
        let mut pool = BisectorPool::from_tree(&initial, 8).unwrap();
        let stale = pool.handles();

        let mut replacement = Tree::at_depth(6, 2).unwrap();
        replacement.merge(Node::new(2, 1).unwrap()).unwrap();
        pool.reset_to_tree(&replacement).unwrap();

        assert_eq!(pool.nodes(), replacement.leaves());
        assert!(pool.is_valid_partition());
        assert_eq!(pool.capacity(), 8);
        assert!(stale.iter().all(|handle| pool.get(*handle).is_none()));
    }

    #[test]
    fn reset_rejects_over_capacity_without_changing_pool() {
        let mut pool = BisectorPool::new(5, 2).unwrap();
        pool.split(pool.root_handle()).unwrap();
        let before = pool.nodes();
        let target = Tree::at_depth(5, 2).unwrap();
        assert_eq!(
            pool.reset_to_tree(&target),
            Err(BisectorPoolError::CapacityExhausted { capacity: 2 })
        );
        assert_eq!(pool.nodes(), before);
        assert!(pool.is_valid_partition());
    }

    #[test]
    fn capacity_exhaustion_is_transactional() {
        let mut pool = BisectorPool::new(4, 1).unwrap();
        let root = pool.root_handle();
        assert_eq!(
            pool.split(root),
            Err(BisectorPoolError::CapacityExhausted { capacity: 1 })
        );
        assert_eq!(pool.active_count(), 1);
        assert_eq!(pool.get(root).unwrap().node(), Node::root());
    }

    #[test]
    fn constructor_rejects_zero_capacity_and_unsupported_depth() {
        assert_eq!(
            BisectorPool::new(4, 0).unwrap_err(),
            BisectorPoolError::InvalidCapacity
        );
        assert_eq!(
            BisectorPool::new(MAX_SUPPORTED_DEPTH + 1, 1).unwrap_err(),
            BisectorPoolError::InvalidMaxDepth {
                depth: MAX_SUPPORTED_DEPTH + 1
            }
        );
    }

    #[test]
    fn stale_handle_cannot_mutate_a_reused_slot() {
        let mut pool = BisectorPool::new(4, 3).unwrap();
        let [left, right] = pool.split(pool.root_handle()).unwrap();
        let old_right = right;
        pool.split(left).unwrap();
        pool.merge(Node::new(2, 1).unwrap()).unwrap();
        let [new_left, new_right] = pool.split(right).unwrap();
        assert_eq!(pool.get(old_right), None);
        assert_ne!(new_left, old_right);
        assert_ne!(new_right, old_right);
        assert!(pool.split(old_right).is_err());
    }
}
