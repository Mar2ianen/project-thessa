use std::collections::BTreeMap;
use thessa_rcbt_core::{
    Node,
    conforming::{Record, cube_vertices, root_index},
};

// Target-size raw GPU snapshot oracle, independent of the planner's links.
pub fn assert_closed_snapshot(snapshot: &[Record]) {
    let mut edges = BTreeMap::new();
    let mut vertices = std::collections::BTreeSet::new();
    let mut areas = [0u128; 12];
    let mut triangles = 0;
    for (slot, &r) in snapshot.iter().enumerate() {
        let Ok(node) =
            Node::from_heap_id(u64::from(r.address[0]) | (u64::from(r.address[1]) << 32))
        else {
            continue;
        };
        let Some(v) = cube_vertices(node) else {
            continue;
        };
        let root = root_index(node).unwrap();
        areas[root as usize] += 1u128 << (58 - node.depth());
        triangles += 1;
        for e in 0..3 {
            vertices.insert(v[e]);
            assert!(edges.insert((v[e], v[(e + 1) % 3]), (slot, e)).is_none());
        }
    }
    for (&(a, b), &(slot, e)) in &edges {
        let &(other, _) = edges
            .get(&(b, a))
            .expect("GPU snapshot must not contain open edges or T-junctions");
        assert_eq!(snapshot[slot].neighbors[e] as usize, other);
    }
    assert_eq!(areas, [1u128 << 54; 12]);
    assert_eq!(
        vertices.len() as isize - (edges.len() / 2) as isize + triangles,
        2
    );
}
