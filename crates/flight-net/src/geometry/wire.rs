//! Mesh payload epochs and lightweight pose/contact descriptors. A complete
//! bounded payload epoch is sent only when its active mesh set changes; COM
//! changes send descriptors alone. IDs are epoch-local, never asset paths.
use super::*;
use std::sync::Arc;
use thessa_sim_core::{RenderMesh, RenderPart};

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
struct MeshCatalogue {
    revision: u64,
    meshes: Vec<Arc<RenderMesh>>,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
struct PartReference {
    name: String,
    position_body_m: glam::DVec3,
    orientation_mesh_to_body: glam::DQuat,
    mesh: usize,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
enum AppearanceReference {
    Absent,
    StarterX15 { origin_body_m: glam::DVec3 },
    Available(Vec<PartReference>),
    OmittedBudget,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
struct VehicleDescriptor {
    vehicle_id: VehicleId,
    display_name: String,
    contact: ContactGeometry,
    render: AppearanceReference,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
struct GeometryDescriptor {
    revision: u64,
    mesh_revision: u64,
    vehicles: Vec<VehicleDescriptor>,
}

/// Cached complete publication. Transports keep independent, coalesced slots
/// for `mesh_frame` and `frame`, updating their revision markers atomically.
pub struct GeometryPublication {
    pub geometry: FleetGeometry,
    pub frame: Arc<Vec<u8>>,
    pub mesh_revision: u64,
    pub mesh_frame: Arc<Vec<u8>>,
    /// PRIMARY appearance/body-frame discontinuity only, not unrelated fleet
    /// metadata. Used by snapshot interpolation, independently of mesh epochs.
    pub primary_revision: u64,
    meshes: Vec<Arc<RenderMesh>>,
}

fn same_mesh(a: &Arc<RenderMesh>, b: &Arc<RenderMesh>) -> bool {
    Arc::ptr_eq(a, b) || a == b
}

impl GeometryPublication {
    pub fn new(geometry: FleetGeometry, previous: Option<&Self>) -> Result<Self, String> {
        geometry.validate()?;
        if previous.is_some_and(|p| geometry.revision <= p.geometry.revision) {
            return Err("geometry publication revision did not advance".into());
        }
        let primary_revision = if let Some(previous) = previous {
            if previous.geometry.primary.as_ref().map(|p| &p.render)
                == geometry.primary.as_ref().map(|p| &p.render)
            {
                previous.primary_revision
            } else {
                previous
                    .primary_revision
                    .checked_add(1)
                    .ok_or("PRIMARY frame revision overflow")?
            }
        } else {
            1
        };
        let mut active = Vec::new();
        for vehicle in geometry.primary.iter().chain(&geometry.vehicles) {
            if let RenderAppearance::Available(render) = &vehicle.render {
                for part in &render.parts {
                    if !active.iter().any(|mesh| same_mesh(mesh, &part.mesh)) {
                        active.push(part.mesh.clone());
                    }
                }
            }
        }
        // Preserve order across split/reordered parts. A COM/pose/name change
        // must not renumber or retransmit an otherwise identical mesh set.
        let mut meshes = previous.map_or_else(Vec::new, |p| {
            p.meshes
                .iter()
                .filter(|mesh| active.iter().any(|a| same_mesh(mesh, a)))
                .cloned()
                .collect()
        });
        for mesh in active {
            if !meshes.iter().any(|old| same_mesh(old, &mesh)) {
                meshes.push(mesh);
            }
        }
        let unchanged = previous.is_some_and(|p| {
            p.meshes.len() == meshes.len()
                && p.meshes.iter().zip(&meshes).all(|(a, b)| same_mesh(a, b))
        });
        let (mesh_revision, mesh_frame) = if unchanged {
            let previous = previous.expect("unchanged publication exists");
            (previous.mesh_revision, previous.mesh_frame.clone())
        } else {
            let revision = previous.map_or(Ok(1), |p| {
                p.mesh_revision
                    .checked_add(1)
                    .ok_or("mesh revision overflow")
            })?;
            let frame = crate::encode_frame(
                thessa_protocol::kind::FLEET_MESHES,
                &MeshCatalogue {
                    revision,
                    meshes: meshes.clone(),
                },
            )
            .map_err(|e| e.to_string())?;
            (revision, Arc::new(frame))
        };
        let descriptor = GeometryDescriptor {
            revision: geometry.revision,
            mesh_revision,
            vehicles: geometry
                .primary
                .iter()
                .chain(&geometry.vehicles)
                .map(|vehicle| VehicleDescriptor {
                    vehicle_id: vehicle.vehicle_id,
                    display_name: vehicle.display_name.clone(),
                    contact: vehicle.contact.clone(),
                    render: match &vehicle.render {
                        RenderAppearance::Absent => AppearanceReference::Absent,
                        RenderAppearance::StarterX15 { origin_body_m } => {
                            AppearanceReference::StarterX15 {
                                origin_body_m: *origin_body_m,
                            }
                        }
                        RenderAppearance::OmittedBudget => AppearanceReference::OmittedBudget,
                        RenderAppearance::Available(render) => AppearanceReference::Available(
                            render
                                .parts
                                .iter()
                                .map(|part| PartReference {
                                    name: part.name.clone(),
                                    position_body_m: part.position_body_m,
                                    orientation_mesh_to_body: part.orientation_mesh_to_body,
                                    mesh: meshes
                                        .iter()
                                        .position(|mesh| same_mesh(mesh, &part.mesh))
                                        .expect("active mesh retained"),
                                })
                                .collect(),
                        ),
                    },
                })
                .collect(),
        };
        let frame = Arc::new(
            crate::encode_frame(thessa_protocol::kind::FLEET_GEOMETRY, &descriptor)
                .map_err(|e| e.to_string())?,
        );
        Ok(Self {
            primary_revision,
            geometry,
            frame,
            mesh_revision,
            mesh_frame,
            meshes,
        })
    }
}

/// One bounded active payload epoch and one latest descriptor. Resolves either
/// arrival order before publishing the existing retained-geometry application
/// contract; a skipped epoch cannot attach another epoch's integer mesh IDs.
#[derive(Default)]
pub struct GeometryReceiver {
    meshes: Option<MeshCatalogue>,
    descriptor: Option<GeometryDescriptor>,
    delivered_revision: u64,
}

impl GeometryReceiver {
    pub fn accept(
        &mut self,
        envelope: &thessa_protocol::Envelope,
    ) -> Result<Option<Arc<FleetGeometry>>, String> {
        match envelope.kind {
            thessa_protocol::kind::FLEET_MESHES => {
                let incoming: MeshCatalogue =
                    crate::decode_payload(envelope).map_err(|e| e.to_string())?;
                if incoming.revision == 0
                    || incoming.meshes.len() > crate::MAX_FLEET_VEHICLES * MAX_RENDER_PARTS
                    || incoming
                        .meshes
                        .iter()
                        .map(|m| m.positions.len())
                        .sum::<usize>()
                        > crate::MAX_FLEET_VEHICLES * MAX_RENDER_VERTICES
                    || incoming
                        .meshes
                        .iter()
                        .map(|m| m.indices.len())
                        .sum::<usize>()
                        > crate::MAX_FLEET_VEHICLES * MAX_RENDER_INDICES
                {
                    return Err("invalid mesh catalogue revision/budget".into());
                }
                for mesh in &incoming.meshes {
                    if mesh.positions.len() > MAX_RENDER_VERTICES
                        || mesh.indices.len() > MAX_RENDER_INDICES
                        || mesh
                            .positions
                            .iter()
                            .any(|v| v.abs().max_element() > MAX_RENDER_EXTENT_M)
                    {
                        return Err("invalid mesh payload budget/extent".into());
                    }
                    mesh.validate()?;
                }
                if let Some(old) = &self.meshes {
                    if incoming.revision < old.revision
                        || (incoming.revision == old.revision && incoming != *old)
                    {
                        return Err("mesh catalogue revision reused or moved backwards".into());
                    }
                    if incoming == *old {
                        return Ok(None);
                    }
                }
                self.meshes = Some(incoming);
            }
            thessa_protocol::kind::FLEET_GEOMETRY => {
                let incoming: GeometryDescriptor =
                    crate::decode_payload(envelope).map_err(|e| e.to_string())?;
                if incoming.revision == 0 || incoming.mesh_revision == 0
                    || incoming.vehicles.len() > crate::MAX_FLEET_VEHICLES
                    || incoming.vehicles.iter().any(|v| matches!(&v.render, AppearanceReference::Available(parts) if parts.len() > MAX_RENDER_PARTS)) {
                    return Err("invalid geometry descriptor revision/budget".into());
                }
                if let Some(old) = &self.descriptor {
                    if incoming.revision < old.revision
                        || (incoming.revision == old.revision && incoming != *old)
                    {
                        return Err("geometry descriptor revision reused or moved backwards".into());
                    }
                    if incoming == *old {
                        return Ok(None);
                    }
                }
                self.descriptor = Some(incoming);
            }
            _ => return Err("not a geometry publication kind".into()),
        }
        let (Some(meshes), Some(descriptor)) = (&self.meshes, &self.descriptor) else {
            return Ok(None);
        };
        if meshes.revision != descriptor.mesh_revision
            || descriptor.revision == self.delivered_revision
        {
            return Ok(None);
        }
        let mut vehicles: Vec<_> = descriptor
            .vehicles
            .iter()
            .map(|vehicle| -> Result<_, String> {
                Ok(FleetVehicleGeometry {
                    vehicle_id: vehicle.vehicle_id,
                    display_name: vehicle.display_name.clone(),
                    contact: vehicle.contact.clone(),
                    render: match &vehicle.render {
                        AppearanceReference::Absent => RenderAppearance::Absent,
                        AppearanceReference::StarterX15 { origin_body_m } => {
                            RenderAppearance::StarterX15 {
                                origin_body_m: *origin_body_m,
                            }
                        }
                        AppearanceReference::OmittedBudget => RenderAppearance::OmittedBudget,
                        AppearanceReference::Available(parts) => {
                            RenderAppearance::Available(RenderGeometry {
                                parts: parts
                                    .iter()
                                    .map(|part| -> Result<_, String> {
                                        Ok(RenderPart {
                                            name: part.name.clone(),
                                            assembly_body: None,
                                            position_body_m: part.position_body_m,
                                            orientation_mesh_to_body: part.orientation_mesh_to_body,
                                            mesh: meshes
                                                .meshes
                                                .get(part.mesh)
                                                .ok_or("unknown mesh ID in matching epoch")?
                                                .clone(),
                                        })
                                    })
                                    .collect::<Result<_, _>>()?,
                            })
                        }
                    },
                })
            })
            .collect::<Result<_, _>>()?;
        let primary = if vehicles.first().is_some_and(|v| v.vehicle_id.is_primary()) {
            Some(vehicles.remove(0))
        } else {
            None
        };
        let geometry = FleetGeometry {
            revision: descriptor.revision,
            primary,
            vehicles,
        };
        geometry.validate()?;
        self.delivered_revision = descriptor.revision;
        Ok(Some(Arc::new(geometry)))
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use glam::{DQuat, DVec3};

    fn catalogue(revision: u64) -> FleetGeometry {
        let mesh = Arc::new(RenderMesh {
            positions: vec![DVec3::ZERO, DVec3::X, DVec3::Y],
            normals: vec![DVec3::Z; 3],
            indices: vec![0, 1, 2],
        });
        FleetGeometry {
            primary: None,
            revision,
            vehicles: (1..=2)
                .map(|id| FleetVehicleGeometry {
                    vehicle_id: VehicleId::new(id),
                    display_name: "shell".into(),
                    contact: ContactGeometry::Absent,
                    render: RenderAppearance::Available(RenderGeometry {
                        parts: vec![RenderPart {
                            name: "body".into(),
                            assembly_body: None,
                            position_body_m: DVec3::ZERO,
                            orientation_mesh_to_body: DQuat::IDENTITY,
                            mesh: mesh.clone(),
                        }],
                    }),
                })
                .collect(),
        }
    }
    fn envelope(bytes: &[u8]) -> thessa_protocol::Envelope {
        crate::decode_frame(&thessa_protocol::FrameDecoder::new().push(bytes).unwrap()[0]).unwrap()
    }
    fn mesh(geometry: &FleetGeometry) -> &Arc<RenderMesh> {
        let RenderAppearance::Available(render) = &geometry.vehicles[0].render else {
            unreachable!()
        };
        &render.parts[0].mesh
    }

    #[test]
    fn com_and_relative_pose_updates_send_no_mesh_payload_and_keep_received_arc_identity() {
        let a = GeometryPublication::new(catalogue(1), None).unwrap();
        assert_eq!(
            a.meshes.len(),
            1,
            "identical meshes across vehicles share an ID"
        );
        let mut receiver = GeometryReceiver::default();
        assert!(receiver.accept(&envelope(&a.mesh_frame)).unwrap().is_none());
        let first = receiver.accept(&envelope(&a.frame)).unwrap().unwrap();
        let mut shifted = catalogue(2);
        for v in &mut shifted.vehicles {
            let RenderAppearance::Available(render) = &mut v.render else {
                unreachable!()
            };
            render.parts[0].position_body_m = DVec3::new(1.0, -2.0, 3.0);
            render.parts[0].orientation_mesh_to_body = DQuat::from_rotation_z(0.3);
        }
        let b = GeometryPublication::new(shifted, Some(&a)).unwrap();
        assert_eq!(b.mesh_revision, a.mesh_revision);
        assert!(Arc::ptr_eq(&a.mesh_frame, &b.mesh_frame));
        let second = receiver.accept(&envelope(&b.frame)).unwrap().unwrap();
        assert!(Arc::ptr_eq(mesh(&first), mesh(&second)));
        assert_eq!(*second, b.geometry);
        assert!(receiver.accept(&envelope(&b.frame)).unwrap().is_none());
    }

    #[test]
    fn primary_frame_counter_is_independent_of_fleet_and_label_revisions() {
        let mut geometry = catalogue(1);
        let mut primary = geometry.vehicles[0].clone();
        primary.vehicle_id = VehicleId::PRIMARY;
        geometry.primary = Some(primary);
        let a = GeometryPublication::new(geometry, None).unwrap();
        let mut geometry = a.geometry.clone();
        geometry.revision += 1;
        geometry.vehicles[0].display_name = "fleet metadata change".into();
        geometry.primary.as_mut().unwrap().display_name = "PRIMARY label change".into();
        let b = GeometryPublication::new(geometry, Some(&a)).unwrap();
        assert_eq!(b.primary_revision, a.primary_revision);
        let mut geometry = b.geometry.clone();
        geometry.revision += 1;
        let RenderAppearance::Available(render) = &mut geometry.primary.as_mut().unwrap().render
        else {
            unreachable!()
        };
        render.parts[0].position_body_m.x -= 0.125;
        let c = GeometryPublication::new(geometry, Some(&b)).unwrap();
        assert_eq!(c.primary_revision, b.primary_revision + 1);
        assert!(Arc::ptr_eq(&c.mesh_frame, &a.mesh_frame));
        let mut receiver = GeometryReceiver::default();
        receiver.accept(&envelope(&c.mesh_frame)).unwrap();
        assert_eq!(
            *receiver.accept(&envelope(&c.frame)).unwrap().unwrap(),
            c.geometry
        );
    }

    #[test]
    fn either_arrival_order_skipped_epochs_and_late_join_resolve_only_matching_mesh_ids() {
        let a = GeometryPublication::new(catalogue(1), None).unwrap();
        let mut changed = catalogue(2);
        let RenderAppearance::Available(render) = &mut changed.vehicles[0].render else {
            unreachable!()
        };
        Arc::make_mut(&mut render.parts[0].mesh).positions[0].x = -0.25;
        let b = GeometryPublication::new(changed, Some(&a)).unwrap();
        assert_eq!(b.mesh_revision, 2);
        let mut changed = b.geometry.clone();
        changed.revision = 4;
        changed.vehicles.pop();
        let c = GeometryPublication::new(changed, Some(&b)).unwrap();
        assert_eq!(c.mesh_revision, 3);
        let mut receiver = GeometryReceiver::default();
        assert!(receiver.accept(&envelope(&a.frame)).unwrap().is_none());
        // Skip the old dependency; old descriptor cannot resolve with epoch 3.
        assert!(receiver.accept(&envelope(&c.mesh_frame)).unwrap().is_none());
        assert_eq!(
            *receiver.accept(&envelope(&c.frame)).unwrap().unwrap(),
            c.geometry
        );
        let mut late_join = GeometryReceiver::default();
        assert!(late_join.accept(&envelope(&c.frame)).unwrap().is_none());
        assert_eq!(
            *late_join.accept(&envelope(&c.mesh_frame)).unwrap().unwrap(),
            c.geometry
        );
        assert!(late_join.accept(&envelope(&a.mesh_frame)).is_err());
        assert!(late_join.accept(&envelope(&a.frame)).is_err());
    }

    #[test]
    fn malformed_or_reused_payloads_unknown_ids_and_reference_budgets_fail_closed() {
        let a = GeometryPublication::new(catalogue(1), None).unwrap();
        let mut receiver = GeometryReceiver::default();
        receiver.accept(&envelope(&a.mesh_frame)).unwrap();
        let mut bad: MeshCatalogue = crate::decode_payload(&envelope(&a.mesh_frame)).unwrap();
        bad.revision += 1;
        Arc::make_mut(&mut bad.meshes[0]).indices[0] = 999;
        assert!(
            receiver
                .accept(&envelope(
                    &crate::encode_frame(thessa_protocol::kind::FLEET_MESHES, &bad).unwrap()
                ))
                .is_err()
        );
        let mut reused: MeshCatalogue = crate::decode_payload(&envelope(&a.mesh_frame)).unwrap();
        Arc::make_mut(&mut reused.meshes[0]).positions[0].x = 0.25;
        assert!(
            receiver
                .accept(&envelope(
                    &crate::encode_frame(thessa_protocol::kind::FLEET_MESHES, &reused).unwrap()
                ))
                .is_err()
        );
        let mut bad: GeometryDescriptor = crate::decode_payload(&envelope(&a.frame)).unwrap();
        let AppearanceReference::Available(parts) = &mut bad.vehicles[0].render else {
            unreachable!()
        };
        parts[0].mesh = 999;
        assert!(
            receiver
                .accept(&envelope(
                    &crate::encode_frame(thessa_protocol::kind::FLEET_GEOMETRY, &bad).unwrap()
                ))
                .is_err()
        );
        let mut receiver = GeometryReceiver::default();
        receiver.accept(&envelope(&a.mesh_frame)).unwrap();
        let mut bad: GeometryDescriptor = crate::decode_payload(&envelope(&a.frame)).unwrap();
        let AppearanceReference::Available(parts) = &mut bad.vehicles[0].render else {
            unreachable!()
        };
        parts.resize(MAX_RENDER_PARTS + 1, parts[0].clone());
        assert!(
            receiver
                .accept(&envelope(
                    &crate::encode_frame(thessa_protocol::kind::FLEET_GEOMETRY, &bad).unwrap()
                ))
                .is_err()
        );
    }
}
