//! Part-assembly connectivity (`docs/details/06`).
//!
//! Index-based crew/air groups and fuel reachability for runtime hatch
//! toggles. Hangar-side validation (diameters, tree shape, volume
//! inventory) lives in `thessa-fuselage`; this module recomputes pure
//! connectivity from link states with no geometry, so opening or sealing
//! a hatch updates crew, air, and fuel domains through one code path.

use serde::{Deserialize, Serialize};
use std::{error::Error, fmt};

/// Assembly connectivity failure modes.
#[derive(Debug, Clone, PartialEq)]
pub enum AssemblyError {
    InvalidLink(String),
}

impl fmt::Display for AssemblyError {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::InvalidLink(message) => write!(formatter, "invalid assembly link: {message}"),
        }
    }
}

impl Error for AssemblyError {}

/// One runtime assembly link between two volume/body indices. Stack
/// links have no door (`hatch = false`) and stay resource-open; hatch
/// links carry the live open state.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub struct AssemblyLinkState {
    pub a: usize,
    pub b: usize,
    pub hatch: bool,
    pub open: bool,
}

impl AssemblyLinkState {
    pub fn validate(&self, node_count: usize) -> Result<(), AssemblyError> {
        if self.a >= node_count || self.b >= node_count {
            return Err(AssemblyError::InvalidLink(format!(
                "link endpoint out of range for {node_count} nodes"
            )));
        }
        if self.a == self.b {
            return Err(AssemblyError::InvalidLink(
                "link connects a node to itself".into(),
            ));
        }
        Ok(())
    }

    /// Crew passes only through open links (stack sides are doorless,
    /// so their stored state is open).
    pub fn crew_open(&self) -> bool {
        self.open
    }

    /// Fuel passes through stack links always, hatch links only when open.
    pub fn resource_open(&self) -> bool {
        self.open || !self.hatch
    }
}

fn validate_all(node_count: usize, links: &[AssemblyLinkState]) -> Result<(), AssemblyError> {
    for link in links {
        link.validate(node_count)?;
    }
    Ok(())
}

struct UnionFind {
    parent: Vec<usize>,
}

impl UnionFind {
    fn new(count: usize) -> Self {
        Self {
            parent: (0..count).collect(),
        }
    }

    fn find(&mut self, mut index: usize) -> usize {
        while self.parent[index] != index {
            self.parent[index] = self.parent[self.parent[index]];
            index = self.parent[index];
        }
        index
    }

    fn union(&mut self, a: usize, b: usize) {
        let (root_a, root_b) = (self.find(a), self.find(b));
        if root_a != root_b {
            self.parent[root_a] = root_b;
        }
    }

    fn groups(&mut self, count: usize) -> Vec<Vec<usize>> {
        let mut groups: std::collections::HashMap<usize, Vec<usize>> =
            std::collections::HashMap::new();
        for index in 0..count {
            groups.entry(self.find(index)).or_default().push(index);
        }
        let mut groups: Vec<Vec<usize>> = groups.into_values().collect();
        for group in &mut groups {
            group.sort_unstable();
        }
        groups.sort_unstable();
        groups
    }
}

/// Crew-passable volume groups through open links.
pub fn crew_groups(
    volume_count: usize,
    links: &[AssemblyLinkState],
) -> Result<Vec<Vec<usize>>, AssemblyError> {
    validate_all(volume_count, links)?;
    let mut union = UnionFind::new(volume_count);
    for link in links {
        if link.crew_open() {
            union.union(link.a, link.b);
        }
    }
    Ok(union.groups(volume_count))
}

/// Shared-air domains: open links between pressurized volumes only.
pub fn air_groups(
    volume_count: usize,
    links: &[AssemblyLinkState],
    pressurized: &[bool],
) -> Result<Vec<Vec<usize>>, AssemblyError> {
    if pressurized.len() != volume_count {
        return Err(AssemblyError::InvalidLink(
            "pressurized mask must cover every volume".into(),
        ));
    }
    validate_all(volume_count, links)?;
    let mut union = UnionFind::new(volume_count);
    for link in links {
        if link.crew_open() && pressurized[link.a] && pressurized[link.b] {
            union.union(link.a, link.b);
        }
    }
    Ok(union.groups(volume_count))
}

/// Tank-to-port fuel reachability through resource-open links.
/// `tanks`/`ports` hold body indices; returns `(tank, port)` pairs.
pub fn feed_reachable(
    body_count: usize,
    links: &[AssemblyLinkState],
    tanks: &[usize],
    ports: &[usize],
) -> Result<Vec<(usize, usize)>, AssemblyError> {
    validate_all(body_count, links)?;
    let mut adjacency: Vec<Vec<usize>> = vec![Vec::new(); body_count];
    for link in links {
        if link.resource_open() {
            adjacency[link.a].push(link.b);
            adjacency[link.b].push(link.a);
        }
    }
    let mut pairs = Vec::new();
    for tank in tanks {
        if *tank >= body_count {
            return Err(AssemblyError::InvalidLink(
                "tank body index out of range".into(),
            ));
        }
        let mut seen = vec![false; body_count];
        let mut stack = vec![*tank];
        seen[*tank] = true;
        while let Some(next) = stack.pop() {
            for neighbor in &adjacency[next] {
                if !seen[*neighbor] {
                    seen[*neighbor] = true;
                    stack.push(*neighbor);
                }
            }
        }
        for port in ports {
            if *port >= body_count {
                return Err(AssemblyError::InvalidLink(
                    "port body index out of range".into(),
                ));
            }
            if seen[*port] {
                pairs.push((*tank, *port));
            }
        }
    }
    pairs.sort_unstable();
    Ok(pairs)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn link(a: usize, b: usize, hatch: bool, open: bool) -> AssemblyLinkState {
        AssemblyLinkState { a, b, hatch, open }
    }

    #[test]
    fn open_hatch_shares_crew_and_air() {
        let links = vec![link(0, 1, true, true)];
        assert_eq!(crew_groups(2, &links).unwrap(), vec![vec![0, 1]]);
        assert_eq!(
            air_groups(2, &links, &[true, true]).unwrap(),
            vec![vec![0, 1]]
        );
        // Dry volume on one side: crew passes, air does not mix.
        assert_eq!(
            air_groups(2, &links, &[true, false]).unwrap(),
            vec![vec![0], vec![1]]
        );
    }

    #[test]
    fn sealed_hatch_splits_domains() {
        let links = vec![link(0, 1, true, false)];
        assert_eq!(crew_groups(2, &links).unwrap(), vec![vec![0], vec![1]]);
        assert_eq!(
            air_groups(2, &links, &[true, true]).unwrap(),
            vec![vec![0], vec![1]]
        );
        // Sealed hatch still blocks fuel (no crossfeed through doors).
        assert!(feed_reachable(2, &links, &[0], &[1]).unwrap().is_empty());
        assert_eq!(feed_reachable(2, &links, &[0], &[0]).unwrap(), vec![(0, 0)]);
    }

    #[test]
    fn stack_links_always_flow() {
        let links = vec![link(0, 1, false, true)];
        assert_eq!(crew_groups(2, &links).unwrap(), vec![vec![0, 1]]);
        assert_eq!(feed_reachable(2, &links, &[0], &[1]).unwrap(), vec![(0, 1)]);
    }

    #[test]
    fn chain_reaches_through_middle_bodies() {
        let links = vec![link(0, 1, false, true), link(1, 2, true, true)];
        assert_eq!(crew_groups(3, &links).unwrap(), vec![vec![0, 1, 2]]);
        assert_eq!(feed_reachable(3, &links, &[0], &[2]).unwrap(), vec![(0, 2)]);
    }

    #[test]
    fn bad_links_fail_closed() {
        assert!(crew_groups(2, &[link(0, 2, true, true)]).is_err());
        assert!(crew_groups(2, &[link(1, 1, true, true)]).is_err());
        assert!(air_groups(2, &[], &[true]).is_err());
        assert!(feed_reachable(2, &[link(0, 1, false, true)], &[5], &[1]).is_err());
    }
}
