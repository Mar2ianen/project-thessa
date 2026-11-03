//! Local ecological eligibility from canonical tree organs, never a shading AO.
use super::*;
use thessa_worldgen_rocky::{offline, rng, scatter::ScatterKind};

const CELL_M: f64 = 32.0;

#[derive(Clone, Copy)]
struct Tree {
    root: DVec3,
    up: DVec3,
    base_radius_m: f64,
    crowns: [(DVec3, f64); 2],
    crown_count: usize,
}

#[derive(Default)]
pub(super) struct CanopyHabitat {
    cells: BTreeMap<[i64; 3], Vec<Tree>>,
    reach_m: f64,
}

pub(super) struct Niche {
    pub shelter: f64,
    pub root_clearance_m: f64,
}

fn cell(p: DVec3) -> [i64; 3] {
    p.to_array().map(|v| (v / CELL_M).floor() as i64)
}

impl CanopyHabitat {
    pub fn new(items: &[ScatterItem], radius_m: f64) -> Self {
        let mut habitat = Self::default();
        if !items.iter().any(|i| i.kind == ScatterKind::LycopsidTree) {
            return habitat;
        }
        let forms = offline::plant_scaffold_forms(false);
        let base_radii: [f64; 3] = std::array::from_fn(|form| {
            let mesh = &forms
                .iter()
                .find(|(m, f)| m.kind == ScatterKind::LycopsidTree && usize::from(*f) == form)
                .unwrap()
                .0;
            mesh.positions
                .iter()
                .filter(|p| p[1] <= 0.10)
                .map(|p| f64::from(p[0]).hypot(f64::from(p[2])))
                .fold(0.0, f64::max)
        });
        for item in items.iter().filter(|i| i.kind == ScatterKind::LycopsidTree) {
            let form = offline::plant_form(item.id, item.kind);
            let pose = instance(item, radius_m);
            let sockets = offline::plant_crown_attachments(form);
            let mut tree = Tree {
                root: pose.position_m,
                up: DVec3::from(item.direction),
                base_radius_m: base_radii[form as usize] * item.scale,
                crowns: [(DVec3::ZERO, 0.0); 2],
                crown_count: sockets.len(),
            };
            for (i, socket) in sockets.into_iter().enumerate() {
                let center = pose.position_m
                    + pose.rotation * DVec3::from(socket.map(f64::from)) * item.scale;
                let radius = f64::from(super::foliage::dimensions(true, form, i).0) * item.scale;
                tree.crowns[i] = (center, radius);
                // Index by the root, including offset fork sockets and a soft edge.
                habitat.reach_m = habitat
                    .reach_m
                    .max(center.distance(tree.root) + radius * 1.2);
            }
            habitat.reach_m = habitat.reach_m.max(tree.base_radius_m + 4.0);
            habitat.cells.entry(cell(tree.root)).or_default().push(tree);
        }
        habitat
    }

    pub fn tree_count(&self) -> usize {
        self.cells.values().map(Vec::len).sum()
    }

    pub fn at(&self, point: DVec3) -> Niche {
        let mut niche = Niche {
            shelter: 0.0,
            root_clearance_m: f64::INFINITY,
        };
        if self.cells.is_empty() {
            return niche;
        }
        let origin = cell(point);
        let halo = (self.reach_m / CELL_M).ceil() as i64;
        for z in -halo..=halo {
            for y in -halo..=halo {
                for x in -halo..=halo {
                    let Some(trees) =
                        self.cells
                            .get(&[origin[0] + x, origin[1] + y, origin[2] + z])
                    else {
                        continue;
                    };
                    for tree in trees {
                        let delta = point - tree.root;
                        let lateral = (delta - tree.up * delta.dot(tree.up)).length();
                        niche.root_clearance_m =
                            niche.root_clearance_m.min(lateral - tree.base_radius_m);
                        for (center, radius) in tree.crowns.iter().take(tree.crown_count) {
                            let delta = point - center;
                            let distance = (delta - tree.up * delta.dot(tree.up)).length();
                            let t = ((distance - radius * 0.35) / (radius * 0.85)).clamp(0.0, 1.0);
                            let proximity = 1.0 - t * t * (3.0 - 2.0 * t);
                            niche.shelter = niche.shelter.max(proximity);
                        }
                    }
                }
            }
        }
        niche
    }

    pub fn accepts_understory(&self, item: &ScatterItem, seed: u64, radius_m: f64) -> bool {
        let niche = self.at(DVec3::from(item.direction) * (radius_m + item.ground_height_m));
        let form = offline::plant_form(item.id, item.kind);
        let radius = if item.kind == ScatterKind::FernClump {
            f64::from(super::foliage::dimensions(false, form, 0).0) * item.scale
        } else {
            0.15 * item.scale
        };
        if niche.root_clearance_m < radius {
            return false;
        }
        // Authored shade tolerance; crown proximity is not an irradiance solver.
        let eligibility = if item.kind == ScatterKind::ReedClump {
            1.0 - 0.75 * niche.shelter
        } else {
            0.60 + 0.40 * niche.shelter
        };
        rng::hash01(seed, 7841, item.id.0 as i64, 0) < eligibility
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn tree(dir: DVec3) -> ScatterItem {
        ScatterItem {
            id: thessa_worldgen_rocky::scatter::ScatterId(7),
            kind: ScatterKind::LycopsidTree,
            direction: dir.to_array(),
            ground_height_m: 10.0,
            scale: 1.0,
            yaw_rad: 0.4,
        }
    }

    #[test]
    fn actual_crowns_create_shelter_and_root_exclusion_without_a_cube_seam() {
        let radius = 3_200_000.0;
        for up in [DVec3::Y, DVec3::new(1.0, 1.0, 0.2).normalize(), -DVec3::Z] {
            let item = tree(up);
            let habitat = CanopyHabitat::new(&[item], radius);
            let pose = instance(&item, radius);
            assert!(habitat.at(pose.position_m).root_clearance_m < 0.0);
            let form = offline::plant_form(item.id, item.kind);
            for socket in offline::plant_crown_attachments(form) {
                let crown = pose.position_m + pose.rotation * DVec3::from(socket.map(f64::from));
                let ground = crown - up * (crown - pose.position_m).dot(up);
                assert!(habitat.at(ground).shelter > 0.999);
                assert!(habitat.at(ground + pose.rotation * DVec3::X * 30.0).shelter == 0.0);
                assert!(
                    (habitat.at(ground).shelter
                        - habitat
                            .at(ground + pose.rotation * DVec3::X * 0.001)
                            .shelter)
                        .abs()
                        < 0.001
                );
            }
        }
    }

    #[test]
    fn fern_and_reed_niches_respond_differently_to_actual_neighbor_crowns() {
        let radius = 3_200_000.0;
        let item = tree(DVec3::Y);
        let habitat = CanopyHabitat::new(&[item], radius);
        let pose = instance(&item, radius);
        let mut counts = [[0_usize; 2]; 2];
        for (k, kind) in [ScatterKind::FernClump, ScatterKind::ReedClump]
            .into_iter()
            .enumerate()
        {
            for (p, distance) in [2.5, 30.0].into_iter().enumerate() {
                let position = pose.position_m + pose.rotation * DVec3::X * distance;
                for id in 1..8193 {
                    let plant = ScatterItem {
                        id: thessa_worldgen_rocky::scatter::ScatterId(id),
                        kind,
                        direction: position.normalize().to_array(),
                        ground_height_m: position.length() - radius,
                        scale: 0.25,
                        yaw_rad: 0.0,
                    };
                    counts[k][p] += usize::from(habitat.accepts_understory(&plant, 42, radius));
                }
            }
            let plant = ScatterItem { kind, ..item };
            assert!(
                !habitat.accepts_understory(&plant, 42, radius),
                "understory must not intersect a stem base"
            );
        }
        assert!(
            counts[0][0] > counts[0][1] + 1000,
            "ferns prefer shelter: {counts:?}"
        );
        assert!(
            counts[1][1] > counts[1][0] + 3000,
            "reeds prefer open sky: {counts:?}"
        );
    }

    #[test]
    fn source_order_does_not_change_habitat() {
        let radius = 3_200_000.0;
        let items = [
            tree(DVec3::Y),
            tree(DVec3::new(0.000003, 1.0, 0.0).normalize()),
        ];
        let a = CanopyHabitat::new(&items, radius);
        let b = CanopyHabitat::new(&[items[1], items[0]], radius);
        for x in -80..80 {
            let p = DVec3::Y * (radius + 10.0) + DVec3::X * x as f64;
            assert_eq!(a.at(p).shelter, b.at(p).shelter);
            assert_eq!(a.at(p).root_clearance_m, b.at(p).root_clearance_m);
        }
    }
}
