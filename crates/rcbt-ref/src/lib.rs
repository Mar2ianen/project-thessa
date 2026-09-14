//! Independent observable-semantics oracle for `rcbt-core` tests.
//!
//! The production tree must not depend on this crate. The upstream `libcbt`
//! revision and license are recorded in `third_party/libcbt.REVISION`; this
//! compact Rust model keeps workspace tests reproducible without requiring a C
//! compiler or making the reference implementation part of the runtime.

#![forbid(unsafe_code)]

use std::collections::BTreeSet;

#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
struct Node {
    id: u64,
    depth: u8,
}

pub struct ReferenceTree {
    max_depth: u8,
    leaves: BTreeSet<Node>,
}

impl ReferenceTree {
    pub fn new(max_depth: u8) -> Result<Self, &'static str> {
        if max_depth > 58 {
            return Err("maximum depth exceeds libcbt's 58-bit node id contract");
        }
        Ok(Self {
            max_depth,
            leaves: BTreeSet::from([Node { id: 1, depth: 0 }]),
        })
    }

    pub fn split(&mut self, id: u64, depth: u8) -> Result<(), &'static str> {
        let node = Node { id, depth };
        if depth >= self.max_depth || !self.leaves.remove(&node) {
            return Err("node cannot be split");
        }
        self.leaves.insert(Node {
            id: id << 1,
            depth: depth + 1,
        });
        self.leaves.insert(Node {
            id: (id << 1) | 1,
            depth: depth + 1,
        });
        Ok(())
    }

    pub fn merge(&mut self, id: u64, depth: u8) -> Result<(), &'static str> {
        if id == 1 || depth == 0 {
            return Err("root cannot be merged");
        }
        let left = Node {
            id: id << 1,
            depth: depth + 1,
        };
        let right = Node {
            id: (id << 1) | 1,
            depth: depth + 1,
        };
        if !self.leaves.remove(&left) {
            return Err("children are not leaves");
        }
        if !self.leaves.remove(&right) {
            self.leaves.insert(left);
            return Err("children are not leaves");
        }
        self.leaves.insert(Node { id, depth });
        Ok(())
    }

    pub fn leaves(&self) -> Vec<(u64, u8)> {
        let mut output = Vec::with_capacity(self.leaves.len());
        self.collect(1, 0, &mut output);
        output
    }

    fn collect(&self, id: u64, depth: u8, output: &mut Vec<(u64, u8)>) {
        let node = Node { id, depth };
        if self.leaves.contains(&node) {
            output.push((id, depth));
        } else if depth < self.max_depth {
            self.collect(id << 1, depth + 1, output);
            self.collect((id << 1) | 1, depth + 1, output);
        }
    }
}
