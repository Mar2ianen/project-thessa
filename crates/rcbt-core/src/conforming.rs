//! Experimental closed cube-sphere newest-vertex bisection reference.
//!
//! Independent implementation of the refinement/decimation ideas in Benyoub &
//! Dupuy, HPG 2024, §2 (https://arxiv.org/abs/2407.02215). No upstream code.
//! Reuses `BisectorPool`; this deliberately slow transactional CPU oracle is
//! not the production terrain scheduler. Addresses and allocation are separate.

use crate::{BisectorHandle, BisectorPool, Node, Tree};

/// Integer cube coordinates, exact through the supported 54 local bisections.
pub const CUBE_SCALE: i64 = 1 << 28;
pub const ROOT_DEPTH: u8 = 4;
pub const NONE: u32 = u32::MAX;

/// Fixed GPU transport: heap low/high, depth, generation, three edge neighbors,
/// reserved. Edges are a→b, b→c and c→a; the last is the refinement edge.
#[repr(C)]
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Record {
    pub address: [u32; 4],
    pub neighbors: [u32; 4],
}

#[derive(Clone, Debug)]
pub struct SphereTopology {
    pool: BisectorPool,
    neighbors: Vec<[u32; 3]>,
}

impl SphereTopology {
    pub fn new(capacity: usize, max_depth: u8) -> Result<Self, &'static str> {
        if capacity < 16 || !(ROOT_DEPTH..=crate::MAX_SUPPORTED_DEPTH).contains(&max_depth) {
            return Err("sphere needs capacity >=16 and total heap depth 4..=58");
        }
        // Preserve the existing single-domain allocator contract: 12 sphere
        // roots + four reserved virtual roots form one complete binary domain.
        let tree = Tree::at_depth(max_depth, ROOT_DEPTH).map_err(|_| "invalid depth")?;
        let pool = BisectorPool::from_tree(&tree, capacity).map_err(|_| "invalid capacity")?;
        let mut mesh = Self {
            pool,
            neighbors: vec![[NONE; 3]; capacity],
        };
        let handles = mesh.handles();
        for h in &handles {
            let a = cube_vertices(mesh.node(*h)).ok_or("invalid root")?;
            for e in 0..3 {
                let matches: Vec<_> = handles
                    .iter()
                    .filter(|other| {
                        let b = cube_vertices(mesh.node(**other)).unwrap();
                        (0..3).any(|f| a[e] == b[(f + 1) % 3] && a[(e + 1) % 3] == b[f])
                    })
                    .collect();
                if matches.len() != 1 {
                    return Err("root surface is not a closed oriented manifold");
                }
                mesh.neighbors[h.slot() as usize][e] = matches[0].slot();
            }
        }
        Ok(mesh)
    }

    pub fn capacity(&self) -> usize {
        self.pool.capacity()
    }
    pub fn max_depth(&self) -> u8 {
        self.pool.max_depth()
    }
    pub fn triangle_count(&self) -> usize {
        self.pool.active_count() - 4
    }
    pub fn node(&self, handle: BisectorHandle) -> Node {
        self.pool.get(handle).expect("live oracle handle").node()
    }
    pub fn handles(&self) -> Vec<BisectorHandle> {
        self.pool
            .handles()
            .into_iter()
            .filter(|h| root_index(self.node(*h)).is_some())
            .collect()
    }

    pub fn records(&self) -> Vec<Record> {
        let mut records = (0..self.capacity())
            .map(|slot| Record {
                address: [0, 0, 0, self.pool.slot_generation(slot).unwrap()],
                neighbors: [NONE; 4],
            })
            .collect::<Vec<_>>();
        for h in self.pool.handles() {
            let n = self.node(h);
            let e = self.neighbors[h.slot() as usize];
            records[h.slot() as usize] = Record {
                address: [
                    n.id() as u32,
                    (n.id() >> 32) as u32,
                    u32::from(n.depth()),
                    h.generation(),
                ],
                neighbors: [e[0], e[1], e[2], 0],
            };
        }
        records
    }

    fn handle_at(&self, slot: u32) -> Result<BisectorHandle, &'static str> {
        self.pool
            .handles()
            .into_iter()
            .find(|h| h.slot() == slot)
            .ok_or("inactive neighbor")
    }
    fn replace(&mut self, neighbor: u32, old: u32, new: u32) {
        for e in &mut self.neighbors[neighbor as usize] {
            if *e == old {
                *e = new;
            }
        }
    }

    /// All compatibility-chain reservations succeed, or the live state is
    /// unchanged. CPU cloning is intentional in the independent oracle only.
    pub fn split(&mut self, node: Node) -> Result<usize, &'static str> {
        let mut candidate = self.clone();
        let count = candidate.split_inner(node)?;
        *self = candidate;
        Ok(count)
    }

    fn split_inner(&mut self, node: Node) -> Result<usize, &'static str> {
        if root_index(node).is_none() {
            return Err("not a sphere address");
        }
        let mut slot = self.pool.handle_for(node).ok_or("inactive address")?.slot();
        let mut chain = Vec::new();
        loop {
            let h = self.handle_at(slot)?;
            if self.node(h).depth() >= self.max_depth() {
                return Err("maximum depth");
            }
            chain.push(slot);
            let twin = self.neighbors[slot as usize][2];
            if self.neighbors[twin as usize][2] == slot {
                if self.node(self.handle_at(twin)?).depth() >= self.max_depth() {
                    return Err("maximum depth");
                }
                break;
            }
            if chain.len() > 64 {
                return Err("invalid compatibility cycle");
            }
            slot = twin;
        }
        if self.pool.free_count() < chain.len() * 2 {
            return Err("capacity exhausted");
        }
        for &a in chain.iter().rev() {
            self.split_pair(a)?;
        }
        Ok(chain.len() * 2)
    }

    fn split_pair(&mut self, a: u32) -> Result<(), &'static str> {
        let b = self.neighbors[a as usize][2];
        if self.neighbors[b as usize][2] != a {
            return Err("non-mutual refinement pair");
        }
        let an = self.neighbors[a as usize];
        let bn = self.neighbors[b as usize];
        let ah = self.handle_at(a)?;
        let bh = self.handle_at(b)?;
        let [_, ar] = self.pool.split(ah).map_err(|_| "pool split rejected")?;
        let [_, br] = self.pool.split(bh).map_err(|_| "pool split rejected")?;
        let ar = ar.slot();
        let br = br.slot();
        self.replace(an[1], a, ar);
        self.replace(bn[1], b, br);
        self.neighbors[a as usize] = [ar, br, an[0]];
        self.neighbors[ar as usize] = [b, a, an[1]];
        self.neighbors[b as usize] = [br, ar, bn[0]];
        self.neighbors[br as usize] = [a, b, bn[1]];
        Ok(())
    }

    /// Conservative four-leaf diamond decimation, never a whole chain merge.
    pub fn merge(&mut self, parent: Node) -> Result<(), &'static str> {
        let mut candidate = self.clone();
        candidate.merge_inner(parent)?;
        *self = candidate;
        Ok(())
    }

    fn merge_inner(&mut self, parent: Node) -> Result<(), &'static str> {
        if root_index(parent).is_none() {
            return Err("not a sphere parent");
        }
        let [left, right] = parent.children().ok_or("maximum depth")?;
        let ah = self
            .pool
            .handle_for(left)
            .ok_or("left child is not a leaf")?;
        let ar = self
            .pool
            .handle_for(right)
            .ok_or("right child is not a leaf")?;
        let a = ah.slot();
        let r = ar.slot();
        let br = self.handle_at(self.neighbors[a as usize][1])?;
        let bh = self.handle_at(self.neighbors[br.slot() as usize][1])?;
        let other = self.node(bh).parent().ok_or("invalid diamond")?;
        if self.node(bh).id() & 1 != 0
            || self.node(br).id() != self.node(bh).id() + 1
            || self.node(bh).depth() != left.depth()
            || self.neighbors[a as usize][0] != r
            || self.neighbors[r as usize][0] != bh.slot()
            || self.neighbors[bh.slot() as usize][0] != br.slot()
            || self.neighbors[br.slot() as usize][0] != a
        {
            return Err("not a four-leaf diamond");
        }
        let an = [
            self.neighbors[a as usize][2],
            self.neighbors[r as usize][2],
            bh.slot(),
        ];
        let bn = [
            self.neighbors[bh.slot() as usize][2],
            self.neighbors[br.slot() as usize][2],
            a,
        ];
        self.replace(an[1], r, a);
        self.replace(bn[1], br.slot(), bh.slot());
        self.pool
            .merge_handles(parent, ah, ar)
            .map_err(|_| "pool merge rejected")?;
        self.pool
            .merge_handles(other, bh, br)
            .map_err(|_| "pool merge rejected")?;
        self.neighbors[a as usize] = an;
        self.neighbors[bh.slot() as usize] = bn;
        self.neighbors[r as usize] = [NONE; 3];
        self.neighbors[br.slot() as usize] = [NONE; 3];
        Ok(())
    }

    /// Independent geometric/topological oracle: exact reversed edge matching,
    /// complete per-root area, Euler characteristic and allocator partition.
    pub fn validate(&self) -> Result<(), &'static str> {
        use std::collections::{BTreeMap, BTreeSet};
        if !self.pool.is_valid_partition() {
            return Err("invalid allocator partition");
        }
        let mut edges = BTreeMap::new();
        let mut vertices = BTreeSet::new();
        let mut areas = [0u128; 12];
        let denominator = 1u128 << (self.max_depth() - ROOT_DEPTH);
        for h in self.handles() {
            let n = self.node(h);
            let v = cube_vertices(n).ok_or("invalid geometry")?;
            areas[root_index(n).unwrap() as usize] += denominator >> (n.depth() - ROOT_DEPTH);
            for e in 0..3 {
                vertices.insert(v[e]);
                if edges.insert((v[e], v[(e + 1) % 3]), h.slot()).is_some() {
                    return Err("duplicate directed edge");
                }
            }
        }
        for (&(a, b), &slot) in &edges {
            let Some(&other) = edges.get(&(b, a)) else {
                return Err("open edge or T-junction");
            };
            let n = self.node(self.handle_at(slot)?);
            let v = cube_vertices(n).unwrap();
            let e = (0..3).find(|&e| v[e] == a && v[(e + 1) % 3] == b).unwrap();
            if self.neighbors[slot as usize][e] != other {
                return Err("neighbor does not match geometric reverse edge");
            }
        }
        if areas.iter().any(|&a| a != denominator) {
            return Err("incomplete root coverage");
        }
        if vertices.len() as isize - (edges.len() / 2) as isize + self.triangle_count() as isize
            != 2
        {
            return Err("not a closed sphere");
        }
        Ok(())
    }
}

pub fn root_index(node: Node) -> Option<u32> {
    if node.depth() < ROOT_DEPTH {
        return None;
    }
    let root = node.id() >> (node.depth() - ROOT_DEPTH);
    (16..28).contains(&root).then_some((root - 16) as u32)
}

/// Exact integer decoder, independent of GPU floating-point vertex evaluation.
pub fn cube_vertices(node: Node) -> Option<[[i64; 3]; 3]> {
    let root = root_index(node)?;
    let face = root / 2;
    let (n, u, v): ([i64; 3], [i64; 3], [i64; 3]) = match face {
        0 => ([1, 0, 0], [0, 1, 0], [0, 0, 1]),
        1 => ([-1, 0, 0], [0, -1, 0], [0, 0, 1]),
        2 => ([0, 1, 0], [0, 0, 1], [1, 0, 0]),
        3 => ([0, -1, 0], [0, 0, -1], [1, 0, 0]),
        4 => ([0, 0, 1], [1, 0, 0], [0, 1, 0]),
        _ => ([0, 0, -1], [-1, 0, 0], [0, 1, 0]),
    };
    let point = |a, b| std::array::from_fn(|i| (n[i] + a * u[i] + b * v[i]) * CUBE_SCALE);
    let mut triangle = if root & 1 == 0 {
        [point(1, 1), point(-1, 1), point(-1, -1)]
    } else {
        [point(-1, -1), point(1, -1), point(1, 1)]
    };
    for depth in (0..node.depth() - ROOT_DEPTH).rev() {
        let [a, b, c] = triangle;
        let mid = std::array::from_fn(|i| (a[i] + c[i]) / 2);
        triangle = if node.id() >> depth & 1 == 0 {
            [b, mid, a]
        } else {
            [c, mid, b]
        };
    }
    Some(triangle)
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn roots_and_compatibility_chains_remain_closed_across_cube_faces() {
        let mut mesh = SphereTopology::new(512, 30).unwrap();
        mesh.validate().unwrap();
        let mut seed = 17u64;
        let mut saw_chain = false;
        for _ in 0..600 {
            seed = seed.wrapping_mul(6364136223846793005).wrapping_add(1);
            let leaves = mesh.handles();
            let n = mesh.node(leaves[(seed as usize >> 8) % leaves.len()]);
            let before = mesh.records();
            let result = if seed & 3 != 0 {
                mesh.split(n).map(|count| {
                    saw_chain |= count > 2;
                })
            } else {
                n.parent().ok_or("root").and_then(|p| mesh.merge(p))
            };
            if result.is_err() {
                assert_eq!(before, mesh.records());
            }
            mesh.validate().unwrap();
        }
        assert!(saw_chain);
        loop {
            let mut progress = false;
            for h in mesh.handles() {
                if let Some(p) = mesh.node(h).parent()
                    && mesh.merge(p).is_ok()
                {
                    progress = true;
                    break;
                }
            }
            if !progress {
                break;
            }
        }
        assert_eq!(mesh.triangle_count(), 12);
        mesh.validate().unwrap();
    }
    #[test]
    fn exhaustion_and_depth_limits_are_transactional_and_recoverable() {
        let mut mesh = SphereTopology::new(18, 5).unwrap();
        let root = Node::new(16, 4).unwrap();
        mesh.split(root).unwrap();
        let full = mesh.records();
        assert!(mesh.split(Node::new(18, 4).unwrap()).is_err());
        assert_eq!(full, mesh.records());
        assert!(mesh.split(root.left_child().unwrap()).is_err());
        assert_eq!(full, mesh.records());
        mesh.merge(root).unwrap();
        mesh.split(Node::new(18, 4).unwrap()).unwrap();
        mesh.validate().unwrap();
        assert!(SphereTopology::new(15, 20).is_err());
        assert_eq!(std::mem::size_of::<Record>(), 32);
    }
    #[test]
    fn address_depth_is_independent_of_pool_size_and_integer_edges_remain_exact() {
        let mut mesh = SphereTopology::new(1024, 58).unwrap();
        let mut node = Node::new(16, 4).unwrap();
        for _ in 4..58 {
            mesh.split(node).unwrap();
            node = node.left_child().unwrap();
        }
        assert!(node.id() > u64::from(u32::MAX));
        mesh.validate().unwrap();
        let before = mesh.records();
        assert!(mesh.split(node).is_err());
        assert_eq!(before, mesh.records());
    }
}
