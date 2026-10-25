//! Near-eye pebble representation of shared world-addressed descriptors.
//! Reuses the parent's preparation job, mesh publication and shadow path.
use super::*;
use glam::Vec3;
use thessa_worldgen_rocky::scatter::ScatterKind;

pub(super) fn append(
    prepared: &mut Prepared,
    items: &[ScatterItem],
    radius_m: f64,
) -> Result<(), String> {
    for palette in 0..3 {
        let start = u32::try_from(prepared.instances.len()).map_err(|_| "ground range overflow")?;
        for item in items
            .iter()
            .filter(|item| item.kind == ScatterKind::Pebble && variant(item.id) == palette)
        {
            prepared.instances.push(instance(item, radius_m));
            prepared.descriptors.push(*item);
        }
        let end = u32::try_from(prepared.instances.len()).map_err(|_| "ground range overflow")?;
        if start == end {
            continue;
        }
        let material = prepared.asset.materials.len();
        let (color, roughness) = match palette {
            0 => ([0.038, 0.036, 0.035, 1.0], 0.40),
            1 => ([0.062, 0.058, 0.052, 1.0], 0.44),
            _ => ([0.088, 0.078, 0.068, 1.0], 0.48),
        };
        prepared.asset.materials.push(Material {
            base_color: color,
            metallic: 0.0,
            roughness,
            double_sided: false,
            ..Default::default()
        });
        let mut primitive = pebble(material);
        let stretch = match palette {
            0 => [1.0, 0.75, 1.15],
            1 => [0.85, 1.0, 1.0],
            _ => [1.05, 0.85, 0.85],
        };
        for vertex in &mut primitive.vertices {
            vertex.position = std::array::from_fn(|i| vertex.position[i] * stretch[i]);
            vertex.normal =
                Vec3::from_array(std::array::from_fn(|i| vertex.normal[i] / stretch[i]))
                    .normalize()
                    .to_array();
        }
        prepared.asset.primitives.push(primitive);
        prepared.ranges.push(start..end);
        prepared.reduced_primitives.push(false);
    }
    Ok(())
}

pub(super) fn variant(id: thessa_worldgen_rocky::scatter::ScatterId) -> u8 {
    ((thessa_worldgen_rocky::rng::hash01(0, 6181, id.0 as i64, 0) * 3.0) as u8).min(2)
}

fn pebble(material: usize) -> Primitive {
    // Authored metre-scale unit cobble, partly embedded at its descriptor's
    // ground height. Instance scale produces 4..30 cm diameters.
    let mut positions = Vec::with_capacity(18);
    for ring in 0..2 {
        for i in 0..8 {
            let angle = i as f32 * std::f32::consts::TAU / 8.0;
            let radius = if ring == 0 { 0.48 } else { 0.34 };
            let irregular = 1.0 + 0.10 * (angle * 3.0 + ring as f32).sin();
            positions.push(Vec3::new(
                angle.cos() * radius * irregular,
                if ring == 0 {
                    0.015
                } else {
                    0.24 + angle.sin() * 0.025
                },
                angle.sin() * radius * irregular * 0.85,
            ));
        }
    }
    positions.push(Vec3::new(0.035, 0.36, -0.025));
    positions.push(Vec3::new(-0.025, -0.12, 0.015));
    let mut triangles = Vec::with_capacity(32);
    for i in 0..8_u32 {
        let j = (i + 1) % 8;
        triangles.extend([
            [i, j, 8 + j],
            [i, 8 + j, 8 + i],
            [8 + i, 8 + j, 16],
            [j, i, 17],
        ]);
    }
    let center = Vec3::new(0.0, 0.1, 0.0);
    let mut normals = vec![Vec3::ZERO; positions.len()];
    for tri in &mut triangles {
        let [a, b, c] = tri.map(|i| positions[i as usize]);
        if (b - a).cross(c - a).dot((a + b + c) / 3.0 - center) < 0.0 {
            tri.swap(1, 2);
        }
        let [a, b, c] = tri.map(|i| positions[i as usize]);
        let area_normal = (b - a).cross(c - a);
        for i in *tri {
            normals[i as usize] += area_normal;
        }
    }
    Primitive {
        vertices: positions
            .into_iter()
            .zip(normals)
            .map(|(p, n)| Vertex {
                position: p.to_array(),
                normal: n.normalize().to_array(),
                uv: [0.0; 2],
            })
            .collect(),
        indices: triangles.into_iter().flatten().collect(),
        material,
        ambient_visibility: vec![1.0; 18],
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn closed_pebble_has_outward_faces_and_no_extra_material_images() {
        let p = pebble(0);
        assert_eq!((p.vertices.len(), p.indices.len()), (18, 96));
        let mut volume = 0.0;
        for tri in p.indices.as_chunks::<3>().0 {
            let [a, b, c] = tri.map(|i| Vec3::from_array(p.vertices[i as usize].position));
            let face = (b - a).cross(c - a);
            assert!(face.length() > 1e-4);
            assert!(face.dot((a + b + c) / 3.0 - Vec3::new(0.0, 0.1, 0.0)) > 0.0);
            volume += a.dot(b.cross(c)) / 6.0;
        }
        assert!(volume > 0.05 && volume < 0.5);
        assert!(
            p.vertices
                .iter()
                .all(|v| Vec3::from_array(v.normal).is_normalized())
        );
        let prepared = convert(
            vec![ScatterItem {
                id: thessa_worldgen_rocky::scatter::ScatterId(1),
                kind: ScatterKind::Pebble,
                direction: [0.0, 1.0, 0.0],
                ground_height_m: 12.0,
                scale: 0.1,
                yaw_rad: 0.5,
            }],
            3_200_000.0,
        )
        .unwrap();
        assert_eq!(prepared.instances.len(), 1);
        assert_eq!(
            prepared.instances[0].position_m,
            DVec3::new(0.0, 3_200_012.0, 0.0)
        );
        assert_eq!(prepared.asset.primitives.len(), 1);
        assert!(prepared.asset.images.is_empty());
    }
    #[test]
    fn near_ground_preparation_is_independent_of_plants_and_rejects_invalid_windows() {
        let dir =
            thessa_worldgen_rocky::sphere::dir_from_latlon(23.565679258039204, 165.13881581863882);
        let mut requested = thessa_graphics::RequestedGraphics::default();
        requested.renderer.terrain_ground_detail = true;
        requested.renderer.surface_quality = thessa_graphics::Quality::High;
        let settings = ResolvedGraphicsSettings::from_requested(
            &requested,
            &thessa_graphics::Capabilities::unknown(),
        );
        let context = Context::SurveyWithUnderstory {
            canopy: survey_window(dir.into()),
            understory: understory_window(dir.into()),
        };
        let field = thessa_flight_authority::launch_site::canonical_world_field().unwrap();
        let (prepared, stats) = prepare(context, &settings, &field).unwrap();
        let prepared = prepared.unwrap();
        assert_eq!(stats.retained, 0);
        assert!(
            stats.ground_pebbles > 0
                && stats.ground_pebbles <= settings.terrain_ground_pebbles_max_items as usize
        );
        assert_eq!(stats.ground_candidates, 9 * 4096);
        assert_eq!(prepared.instances.len(), stats.ground_pebbles);
        assert!(prepared.asset.images.is_empty());
        for item in prepared.descriptors.iter().take(16) {
            assert_eq!(
                item.ground_height_m,
                field.height_m(item.direction, 32.0).max(0.0)
                    + thessa_worldgen_rocky::lod::ground_relief_height_m(
                        &field,
                        item.direction,
                        0.012
                    )
            );
        }
        eprintln!(
            "ground_pebbles count={} candidates={} instances={} bytes={} triangles_per_prototype=32",
            stats.ground_pebbles,
            stats.ground_candidates,
            prepared.instances.len(),
            stats.instance_buffer_bytes
        );
        requested.renderer.terrain_ground_detail = false;
        let disabled = ResolvedGraphicsSettings::from_requested(
            &requested,
            &thessa_graphics::Capabilities::unknown(),
        );
        let (none, stats) = prepare(context, &disabled, &field).unwrap();
        assert!(none.is_none());
        assert_eq!(
            (
                stats.ground_candidates,
                stats.samples,
                stats.instance_buffer_bytes
            ),
            (0, 0, 0)
        );
        let bad = Context::SurveyWithUnderstory {
            canopy: survey_window(dir.into()),
            understory: TileKey {
                face: 6,
                level: 18,
                x: 0,
                y: 0,
            },
        };
        assert!(prepare(bad, &settings, &field).is_err());
    }
    #[test]
    fn different_pebble_palettes_cannot_reuse_the_wrong_immutable_source() {
        let make = |id| {
            convert(
                vec![ScatterItem {
                    id: thessa_worldgen_rocky::scatter::ScatterId(id),
                    kind: ScatterKind::Pebble,
                    direction: [0.0, 1.0, 0.0],
                    ground_height_m: 12.0,
                    scale: 0.1,
                    yaw_rad: 0.0,
                }],
                3_200_000.0,
            )
            .unwrap()
        };
        let first = make(1);
        let different = (2..100)
            .find(|id| {
                variant(thessa_worldgen_rocky::scatter::ScatterId(*id))
                    != variant(thessa_worldgen_rocky::scatter::ScatterId(1))
            })
            .unwrap();
        assert!(!first.same_prototypes(&make(different)));
        let same = (2..100)
            .find(|id| {
                variant(thessa_worldgen_rocky::scatter::ScatterId(*id))
                    == variant(thessa_worldgen_rocky::scatter::ScatterId(1))
            })
            .unwrap();
        assert!(first.same_prototypes(&make(same)));
    }
}
