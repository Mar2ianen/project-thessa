//! Cached depth-only plant forms and conservative two-sided surface error.
use crate::offline::{PlantMesh, plant_prototype_forms, shadow_plant_prototype_forms};
use parry3d_f64::{math::Vector, query::PointQuery, shape::TriMesh};
use std::{collections::BTreeSet, ops::Range, sync::OnceLock};

#[derive(Clone)]
pub struct CertifiedPlant {
    pub mesh: PlantMesh,
    pub form: u8,
    /// Stem, living foliage, senescent foliage; absent organs have no bound.
    pub errors_m: [Option<f64>; 3],
}

pub fn organ_ranges(mesh: &PlantMesh) -> [Range<usize>; 3] {
    let stem = mesh.stem.as_ref().map_or(0, |s| s.vertex_count as usize);
    let dry = mesh
        .senescent
        .as_ref()
        .map_or(mesh.positions.len(), |s| s.first_vertex as usize);
    [0..stem, stem..dry, dry..mesh.positions.len()]
}

pub fn prototypes(reduced: bool) -> &'static [CertifiedPlant] {
    static FULL: OnceLock<Vec<CertifiedPlant>> = OnceLock::new();
    static REDUCED: OnceLock<Vec<CertifiedPlant>> = OnceLock::new();
    (if reduced { &REDUCED } else { &FULL })
        .get_or_init(|| certify_forms(reduced, shadow_plant_prototype_forms(), false))
}

/// Additional coarse surfaces use the same continuous surface certificate.
pub fn far_prototypes(reduced: bool) -> &'static [CertifiedPlant] {
    static FULL: OnceLock<Vec<CertifiedPlant>> = OnceLock::new();
    static REDUCED: OnceLock<Vec<CertifiedPlant>> = OnceLock::new();
    (if reduced { &REDUCED } else { &FULL }).get_or_init(|| {
        certify_forms(
            reduced,
            crate::offline::far_shadow_plant_prototype_forms(),
            true,
        )
    })
}

fn certify_forms(
    reduced: bool,
    forms: Vec<(PlantMesh, u8)>,
    unresolved_only: bool,
) -> Vec<CertifiedPlant> {
    plant_prototype_forms(reduced)
        .iter()
        .zip(forms)
        .map(|((source, form), (mesh, shadow_form))| {
            assert_eq!(*form, shadow_form);
            assert_eq!(source.kind, mesh.kind);
            let errors_m = std::array::from_fn(|i| {
                let a = organ_triangles(source, organ_ranges(source)[i].clone());
                let b = organ_triangles(&mesh, organ_ranges(&mesh)[i].clone());
                let error = surface_error_bound(&source.positions, &a, &mesh.positions, &b)?;
                if !unresolved_only {
                    return Some(error);
                }
                // Charge the complete organ's enclosing sphere as well as its
                // surface error: the far tier requires sub-texel support.
                let mut lo = [f64::INFINITY; 3];
                let mut hi = [f64::NEG_INFINITY; 3];
                for p in source.positions[organ_ranges(source)[i].clone()]
                    .iter()
                    .chain(&mesh.positions[organ_ranges(&mesh)[i].clone()])
                {
                    for k in 0..3 {
                        lo[k] = lo[k].min(f64::from(p[k]));
                        hi[k] = hi[k].max(f64::from(p[k]));
                    }
                }
                let radius = 0.5 * (0..3).map(|k| (hi[k] - lo[k]).powi(2)).sum::<f64>().sqrt();
                let rounding = 128.0
                    * f64::EPSILON
                    * hi.into_iter().chain(lo).map(f64::abs).fold(1.0, f64::max);
                Some(error.max(radius + rounding))
            });
            CertifiedPlant {
                mesh,
                form: *form,
                errors_m,
            }
        })
        .collect()
}

pub fn organ_triangles(mesh: &PlantMesh, range: Range<usize>) -> Vec<[u32; 3]> {
    mesh.indices
        .as_chunks::<3>()
        .0
        .iter()
        .copied()
        .filter(|t| t.iter().all(|&i| range.contains(&(i as usize))))
        .collect()
}

/// Centroid distance plus triangle radius bounds every point by the 1-Lipschitz
/// property of distance to a closed set. Subdivision tightens that upper bound;
/// a work/depth limit keeps the remaining upper bound, never a sampled estimate.
pub fn surface_error_bound(
    a: &[[f32; 3]],
    ai: &[[u32; 3]],
    b: &[[f32; 3]],
    bi: &[[u32; 3]],
) -> Option<f64> {
    fn directed(a: &[[f32; 3]], ai: &[[u32; 3]], b: &[[f32; 3]], bi: &[[u32; 3]]) -> Option<f64> {
        let target = TriMesh::new(
            b.iter()
                .map(|p| Vector::from_array(p.map(f64::from)))
                .collect(),
            bi.to_vec(),
        )
        .ok()?;
        let key = |t: &[u32; 3], p: &[[f32; 3]]| {
            let mut k = t.map(|i| p[i as usize].map(|v| if v == 0.0 { 0 } else { v.to_bits() }));
            k.sort_unstable();
            k
        };
        let exact: BTreeSet<_> = bi.iter().map(|t| key(t, b)).collect();
        let mut lower = 0.0_f64;
        let mut upper = 0.0_f64;
        let mut queries = 0_usize;
        for tri in ai {
            if exact.contains(&key(tri, a)) {
                continue;
            }
            let mut stack = vec![(
                tri.map(|i| Vector::from_array(a[i as usize].map(f64::from))),
                0,
            )];
            while let Some((p, depth)) = stack.pop() {
                let c = (p[0] + p[1] + p[2]) / 3.0;
                let d = target.distance_to_local_point(c, false);
                queries += 1;
                if !d.is_finite() {
                    return None;
                }
                lower = lower.max(d);
                let bound = d + p.iter().map(|v| v.distance(c)).fold(0.0_f64, f64::max);
                if bound <= lower + 0.01 || depth >= 24 || queries >= 2_000_000 {
                    upper = upper.max(bound);
                } else {
                    let i = (0..3)
                        .max_by(|&i, &j| {
                            p[i].distance_squared(p[(i + 1) % 3])
                                .total_cmp(&p[j].distance_squared(p[(j + 1) % 3]))
                        })
                        .unwrap();
                    let j = (i + 1) % 3;
                    let k = (i + 2) % 3;
                    let mid = (p[i] + p[j]) * 0.5;
                    stack.push(([p[i], mid, p[k]], depth + 1));
                    stack.push(([mid, p[j], p[k]], depth + 1));
                }
            }
        }
        Some(upper)
    }
    for (p, indices) in [(a, ai), (b, bi)] {
        if indices.is_empty()
            || p.iter().flatten().any(|v| !v.is_finite())
            || indices.iter().flatten().any(|&i| i as usize >= p.len())
        {
            return None;
        }
    }
    let magnitude = a
        .iter()
        .chain(b)
        .flatten()
        .map(|v| f64::from(*v).abs())
        .fold(1.0_f64, f64::max);
    let numeric_envelope = 128.0 * f64::EPSILON * magnitude + 1e-6;
    Some(directed(a, ai, b, bi)?.max(directed(b, bi, a, ai)?) + numeric_envelope)
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn bound_covers_triangle_interiors_in_both_directions() {
        let a = [[0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0]];
        let b = a.map(|p| [p[0], p[1], 0.1]);
        let bound = surface_error_bound(&a, &[[0, 1, 2]], &b, &[[0, 1, 2]]).unwrap();
        assert!((0.1..0.111).contains(&bound), "{bound}");
        assert!(surface_error_bound(&a, &[[0, 1, 3]], &b, &[[0, 1, 2]]).is_none());
        // Equal corners do not imply equal surfaces: the large triangle spans
        // the empty interior between three tiny corner triangles.
        let corners = [
            [0.0, 0.0, 0.0],
            [0.01, 0.0, 0.0],
            [0.0, 0.01, 0.0],
            [1.0, 0.0, 0.0],
            [0.99, 0.0, 0.0],
            [0.99, 0.01, 0.0],
            [0.0, 1.0, 0.0],
            [0.0, 0.99, 0.0],
            [0.01, 0.99, 0.0],
        ];
        assert!(
            surface_error_bound(
                &a,
                &[[0, 1, 2]],
                &corners,
                &[[0, 1, 2], [3, 4, 5], [6, 7, 8]]
            )
            .unwrap()
                > 0.4
        );
    }
    #[test]
    fn far_tiers_bound_entire_organ_surfaces_and_preserve_all_forms() {
        for reduced in [false, true] {
            let sources = plant_prototype_forms(reduced);
            for (((a, form), near), far) in sources
                .iter()
                .zip(prototypes(reduced))
                .zip(far_prototypes(reduced))
            {
                assert_eq!(*form, far.form);
                assert_eq!(a.kind, far.mesh.kind);
                assert!(far.mesh.indices.len() <= near.mesh.indices.len());
                for organ in 0..3 {
                    let ai = organ_triangles(a, organ_ranges(a)[organ].clone());
                    let bi = organ_triangles(&far.mesh, organ_ranges(&far.mesh)[organ].clone());
                    if ai.is_empty() {
                        assert!(bi.is_empty() && far.errors_m[organ].is_none());
                        continue;
                    }
                    assert!(!bi.is_empty());
                    let bound = far.errors_m[organ].unwrap();
                    assert!(bound.is_finite() && bound > 0.0);
                    let anchor = a.positions[ai[0][0] as usize];
                    for &i in ai.iter().flatten() {
                        let diameter = (0..3)
                            .map(|k| {
                                (f64::from(a.positions[i as usize][k]) - f64::from(anchor[k]))
                                    .powi(2)
                            })
                            .sum::<f64>()
                            .sqrt();
                        assert!(
                            diameter <= 2.0 * bound,
                            "far tier lost whole-organ support charge"
                        );
                    }
                    for (points, triangles, target, ti) in [
                        (&a.positions, &ai, &far.mesh.positions, &bi),
                        (&far.mesh.positions, &bi, &a.positions, &ai),
                    ] {
                        let target = TriMesh::new(
                            target
                                .iter()
                                .map(|p| Vector::from_array(p.map(f64::from)))
                                .collect(),
                            ti.clone(),
                        )
                        .unwrap();
                        for triangle in triangles {
                            let p = triangle
                                .map(|i| Vector::from_array(points[i as usize].map(f64::from)));
                            for v in [p[0], p[1], p[2], (p[0] + p[1] + p[2]) / 3.0] {
                                assert!(
                                    target.distance_to_local_point(v, false) <= bound,
                                    "{form} {organ} {reduced}"
                                );
                            }
                        }
                    }
                }
            }
        }
    }
    #[test]
    fn certified_shadow_forms_keep_organs_and_reduce_geometry() {
        for reduced in [false, true] {
            let source = plant_prototype_forms(reduced);
            for ((a, form), b) in source.iter().zip(prototypes(reduced)) {
                assert_eq!(*form, b.form);
                for i in 0..3 {
                    let indices = organ_triangles(a, organ_ranges(a)[i].clone());
                    if indices.is_empty() {
                        assert!(b.errors_m[i].is_none());
                    } else {
                        assert!(
                            b.errors_m[i].is_some_and(|v| v.is_finite() && v < 0.1),
                            "{:?} {form} {reduced} {:?}",
                            a.kind,
                            b.errors_m
                        );
                    }
                }
                if a.kind != crate::scatter::ScatterKind::ReedClump {
                    // Small coarse juvenile fronds already use the minimum division count.
                    assert!(b.mesh.indices.len() <= a.indices.len());
                }
                if a.kind == crate::scatter::ScatterKind::LycopsidTree {
                    assert!(b.mesh.indices.len() * 3 < a.indices.len());
                }
                eprintln!(
                    "{:?} form={form} reduced={reduced}: {} -> {} triangles; errors {:?}",
                    a.kind,
                    a.indices.len() / 3,
                    b.mesh.indices.len() / 3,
                    b.errors_m
                );
            }
        }
    }
}
