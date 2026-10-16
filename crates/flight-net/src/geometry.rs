//! Bounded, revisioned fleet contact/retained-appearance catalogue (game policy).
//! Not a full vehicle design/material registry; no paths, executable asset loaders,
//! renderer handles, or inference of a model from a display name.
use serde::{Deserialize, Serialize};
use thessa_sim_core::{CollisionGeometry, RenderGeometry, VehicleDefinition, VehicleId};

use crate::MAX_FLEET_VEHICLES;

mod wire;
pub use wire::{GeometryPublication, GeometryReceiver};

pub const MAX_GEOMETRY_PARTS_PER_VEHICLE: usize = 64;
pub const MAX_DISPLAY_NAME_BYTES: usize = 128;
pub const MAX_RENDER_PARTS: usize = 8;
pub const MAX_RENDER_VERTICES: usize = 1024;
pub const MAX_RENDER_INDICES: usize = 4096;
pub const MAX_RENDER_EXTENT_M: f64 = 1024.0;

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub enum RenderAppearance {
    Absent,
    /// Explicit authority-owned compatibility asset, PRIMARY only. This is a
    /// closed built-in identity, not a path, display-name guess or fallback.
    StarterX15 {
        origin_body_m: glam::DVec3,
    },
    /// Actual retained meshes. Assembly ownership has already been resolved
    /// by authority; only body-local part poses cross this display boundary.
    Available(RenderGeometry),
    OmittedBudget,
}

fn render_fits(geometry: &RenderGeometry) -> bool {
    geometry.parts.len() <= MAX_RENDER_PARTS
        && geometry
            .parts
            .iter()
            .map(|p| p.mesh.positions.len())
            .sum::<usize>()
            <= MAX_RENDER_VERTICES
        && geometry
            .parts
            .iter()
            .map(|p| p.mesh.indices.len())
            .sum::<usize>()
            <= MAX_RENDER_INDICES
        && geometry.parts.iter().all(|p| {
            p.position_body_m.is_finite()
                && p.position_body_m.abs().max_element() <= MAX_RENDER_EXTENT_M
                && p.mesh
                    .positions
                    .iter()
                    .all(|v| v.is_finite() && v.abs().max_element() <= MAX_RENDER_EXTENT_M)
        })
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub enum ContactGeometry {
    /// The authoritative definition has no contact parts.
    Absent,
    /// Exact authoritative part-local poses/dimensions in vehicle body metres.
    Available(CollisionGeometry),
    /// No partial hull is substituted when the wire-policy budget is exceeded.
    OmittedBudget,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct FleetVehicleGeometry {
    pub vehicle_id: VehicleId,
    /// Bounded label only. Stable identity is VehicleId, not this string.
    pub display_name: String,
    pub contact: ContactGeometry,
    pub render: RenderAppearance,
}

fn display_name(name: &str) -> &str {
    let mut end = name.len().min(MAX_DISPLAY_NAME_BYTES);
    while !name.is_char_boundary(end) {
        end -= 1;
    }
    &name[..end]
}

fn part_name_matches(label: &str, name: &str) -> bool {
    let clean = display_name(name).chars().filter(|c| !c.is_control());
    if clean.clone().all(char::is_whitespace) {
        label == "unnamed"
    } else {
        label.chars().eq(clean)
    }
}

impl FleetVehicleGeometry {
    pub fn primary_from_authority(authority: &thessa_flight_authority::FlightAuthority) -> Self {
        let mut descriptor = Self::from_vehicle(VehicleId::PRIMARY, &authority.vehicle);
        if matches!(descriptor.render, RenderAppearance::Absent)
            && let Some(origin_body_m) = authority.starter_render_origin_body_m()
        {
            descriptor.render = RenderAppearance::StarterX15 { origin_body_m };
        }
        descriptor
    }

    pub fn matches_primary(&self, authority: &thessa_flight_authority::FlightAuthority) -> bool {
        self.matches_vehicle(VehicleId::PRIMARY, &authority.vehicle)
            && match self.render {
                RenderAppearance::StarterX15 { origin_body_m } => {
                    authority.starter_render_origin_body_m() == Some(origin_body_m)
                }
                RenderAppearance::Absent => authority.starter_render_origin_body_m().is_none(),
                _ => true,
            }
    }
    pub fn from_vehicle(vehicle_id: VehicleId, vehicle: &VehicleDefinition) -> Self {
        let parts = &vehicle.collision_geometry.parts;
        Self {
            vehicle_id,
            display_name: display_name(&vehicle.name)
                .chars()
                .filter(|c| !c.is_control())
                .collect(),
            contact: if parts.is_empty() {
                ContactGeometry::Absent
            } else if parts.len() > MAX_GEOMETRY_PARTS_PER_VEHICLE {
                ContactGeometry::OmittedBudget
            } else {
                ContactGeometry::Available(vehicle.collision_geometry.clone())
            },
            render: if vehicle.render_geometry.parts.is_empty() {
                RenderAppearance::Absent
            } else if !render_fits(&vehicle.render_geometry) {
                RenderAppearance::OmittedBudget
            } else {
                let mut geometry = vehicle.render_geometry.clone();
                for part in &mut geometry.parts {
                    part.assembly_body = None;
                    part.name = display_name(&part.name)
                        .chars()
                        .filter(|c| !c.is_control())
                        .collect();
                    if part.name.trim().is_empty() {
                        part.name = "unnamed".into();
                    }
                }
                RenderAppearance::Available(geometry)
            },
        }
    }

    /// Borrowed comparison avoids rebuilding/copying/encoding an unchanged
    /// catalogue at snapshot cadence. Includes recentering and shape changes.
    pub fn matches_vehicle(&self, vehicle_id: VehicleId, vehicle: &VehicleDefinition) -> bool {
        self.vehicle_id == vehicle_id
            && self.display_name.chars().eq(display_name(&vehicle.name)
                .chars()
                .filter(|c| !c.is_control()))
            && match &self.contact {
                ContactGeometry::Absent => vehicle.collision_geometry.parts.is_empty(),
                ContactGeometry::Available(geometry) => *geometry == vehicle.collision_geometry,
                ContactGeometry::OmittedBudget => {
                    vehicle.collision_geometry.parts.len() > MAX_GEOMETRY_PARTS_PER_VEHICLE
                }
            }
            && match &self.render {
                RenderAppearance::Absent => vehicle.render_geometry.parts.is_empty(),
                RenderAppearance::StarterX15 { .. } => vehicle.render_geometry.parts.is_empty(),
                RenderAppearance::OmittedBudget => {
                    !vehicle.render_geometry.parts.is_empty()
                        && !render_fits(&vehicle.render_geometry)
                }
                RenderAppearance::Available(geometry) => {
                    geometry.parts.len() == vehicle.render_geometry.parts.len()
                        && geometry
                            .parts
                            .iter()
                            .zip(&vehicle.render_geometry.parts)
                            .all(|(a, b)| {
                                part_name_matches(&a.name, &b.name)
                                    && a.position_body_m == b.position_body_m
                                    && a.orientation_mesh_to_body == b.orientation_mesh_to_body
                                    && (std::sync::Arc::ptr_eq(&a.mesh, &b.mesh)
                                        || a.mesh == b.mesh)
                            })
                }
            }
    }
}

/// Complete secondary set plus explicit PRIMARY appearance. Revisions increase
/// within one link; snapshots name the exact revision they require. Sent on change and
/// to newly subscribed peers, not as bulk geometry on every kinematic sample.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct FleetGeometry {
    pub revision: u64,
    /// Missing means appearance unknown: consumers must not invent PRIMARY.
    pub primary: Option<FleetVehicleGeometry>,
    pub vehicles: Vec<FleetVehicleGeometry>,
}

impl FleetGeometry {
    /// Independent latest-wins slots may temporarily disagree. Only an exact
    /// revision match can attach geometry; same-revision ID mismatch is corrupt.
    pub fn matches_snapshot(&self, snapshot: &crate::FleetSnapshot) -> Result<bool, String> {
        if self.revision != snapshot.geometry_revision {
            return Ok(false);
        }
        if self.vehicles.len() != snapshot.vehicles.len()
            || !self
                .vehicles
                .iter()
                .zip(&snapshot.vehicles)
                .all(|(g, s)| g.vehicle_id == s.vehicle_id)
        {
            return Err("fleet geometry/snapshot membership mismatch".into());
        }
        Ok(true)
    }

    pub fn validate(&self) -> Result<(), String> {
        if self.revision == 0
            || self.vehicles.len() >= MAX_FLEET_VEHICLES
            || self
                .vehicles
                .first()
                .is_some_and(|v| v.vehicle_id.is_primary())
            || self
                .vehicles
                .windows(2)
                .any(|p| p[0].vehicle_id >= p[1].vehicle_id)
        {
            return Err("invalid fleet geometry revision/capacity/IDs".into());
        }
        if self
            .primary
            .as_ref()
            .is_some_and(|p| !p.vehicle_id.is_primary())
        {
            return Err("primary appearance has a non-primary ID".into());
        }
        for vehicle in self.primary.iter().chain(&self.vehicles) {
            if vehicle.display_name.len() > MAX_DISPLAY_NAME_BYTES
                || vehicle.display_name.chars().any(char::is_control)
            {
                return Err("invalid fleet display name".into());
            }
            if let ContactGeometry::Available(geometry) = &vehicle.contact {
                if geometry.parts.is_empty()
                    || geometry.parts.len() > MAX_GEOMETRY_PARTS_PER_VEHICLE
                {
                    return Err("invalid contact geometry part budget".into());
                }
                geometry.validate().map_err(|e| e.to_string())?;
            }
            if let RenderAppearance::Available(geometry) = &vehicle.render {
                if geometry.parts.is_empty()
                    || !render_fits(geometry)
                    || geometry.parts.iter().any(|p| {
                        p.name.len() > MAX_DISPLAY_NAME_BYTES
                            || p.name.chars().any(char::is_control)
                    })
                {
                    return Err("invalid fleet render geometry budget/name".into());
                }
                geometry.validate(None)?;
            }
            if let RenderAppearance::StarterX15 { origin_body_m } = vehicle.render
                && (!vehicle.vehicle_id.is_primary()
                    || !origin_body_m.is_finite()
                    || origin_body_m.abs().max_element() > MAX_RENDER_EXTENT_M)
            {
                return Err("invalid built-in PRIMARY identity/origin".into());
            }
        }
        Ok(())
    }
}

pub fn encode_fleet_geometry(
    geometry: &FleetGeometry,
) -> Result<Vec<u8>, thessa_protocol::CodecError> {
    // Self-contained initial stream bundle for fixtures/one-shot consumers.
    // Live publishers retain GeometryPublication to reuse payload epochs.
    let publication = GeometryPublication::new(geometry.clone(), None)
        .map_err(thessa_protocol::CodecError::Codec)?;
    let mut bytes = (*publication.mesh_frame).clone();
    bytes.extend_from_slice(&publication.frame);
    Ok(bytes)
}

#[cfg(test)]
mod tests {
    use super::*;
    use glam::{DQuat, DVec3};
    use thessa_sim_core::{CollisionMaterial, CollisionPart, CollisionShape, X15StarterProfile};

    fn roundtrip(bytes: &[u8]) -> FleetGeometry {
        let mut receiver = GeometryReceiver::default();
        let mut result = None;
        for frame in thessa_protocol::FrameDecoder::new().push(bytes).unwrap() {
            if let Some(geometry) = receiver
                .accept(&crate::decode_frame(&frame).unwrap())
                .unwrap()
            {
                result = Some(geometry);
            }
        }
        (*result.unwrap()).clone()
    }

    #[test]
    fn primary_asset_identity_is_authority_declared_not_a_name_fallback() {
        let system: thessa_sim_core::SystemConfig =
            toml::from_str(include_str!("../../../data/system.toml")).unwrap();
        let ephemeris = system.bake().unwrap();
        let body = ephemeris.body_id("thessa").unwrap();
        let starter = thessa_flight_authority::FlightAuthority::new(&ephemeris, body).unwrap();
        let declared = FleetVehicleGeometry::primary_from_authority(&starter);
        assert!(matches!(
            declared.render,
            RenderAppearance::StarterX15 { .. }
        ));
        assert!(declared.matches_primary(&starter));
        let custom = thessa_flight_authority::FlightAuthority::new_with_vehicle(
            &ephemeris,
            body,
            starter.vehicle.clone(),
        )
        .unwrap();
        assert!(!declared.matches_primary(&custom));
        let custom_descriptor = FleetVehicleGeometry::primary_from_authority(&custom);
        assert_eq!(
            custom_descriptor.render,
            RenderAppearance::Absent,
            "identical X-15 name/physical parameters do not authorize an asset"
        );
        let mut catalogue = FleetGeometry {
            revision: 1,
            primary: Some(declared.clone()),
            vehicles: vec![],
        };
        assert_eq!(
            roundtrip(&encode_fleet_geometry(&catalogue).unwrap()),
            catalogue
        );
        let mut invalid = declared;
        invalid.vehicle_id = VehicleId::new(1);
        catalogue.vehicles.push(invalid);
        assert!(
            catalogue.validate().is_err(),
            "built-in identity is PRIMARY-only"
        );
    }

    #[test]
    fn render_roundtrip_keeps_authored_sources_and_bounds_the_complete_frame() {
        use std::sync::Arc;
        use thessa_sim_core::{RenderMesh, RenderPart};
        let mut vehicle = vehicle();
        let mesh = Arc::new(RenderMesh {
            positions: vec![DVec3::new(0.1, 0.2, 0.3); MAX_RENDER_VERTICES],
            normals: vec![DVec3::Z; MAX_RENDER_VERTICES],
            indices: vec![1023; 4095],
        });
        vehicle.render_geometry.parts.push(RenderPart {
            name: "\n".into(),
            assembly_body: Some(99),
            position_body_m: DVec3::new(2.0, 3.0, 4.0),
            orientation_mesh_to_body: DQuat::from_rotation_z(0.7),
            mesh: mesh.clone(),
        });
        let descriptor = FleetVehicleGeometry::from_vehicle(VehicleId::new(1), &vehicle);
        assert!(descriptor.matches_vehicle(VehicleId::new(1), &vehicle));
        let RenderAppearance::Available(render) = &descriptor.render else {
            panic!("retained source");
        };
        assert!(Arc::ptr_eq(&mesh, &render.parts[0].mesh));
        assert_eq!(render.parts[0].assembly_body, None);
        assert_eq!(render.parts[0].name, "unnamed");
        let catalogue = FleetGeometry {
            revision: 2,
            primary: None,
            vehicles: (1..MAX_FLEET_VEHICLES as u32)
                .map(|id| FleetVehicleGeometry::from_vehicle(VehicleId::new(id), &vehicle))
                .collect(),
        };
        catalogue.validate().unwrap();
        let bytes = encode_fleet_geometry(&catalogue).unwrap();
        assert!(bytes.len() < thessa_protocol::MAX_FRAME_BYTES);
        let decoded = roundtrip(&bytes);
        assert_eq!(decoded, catalogue);
        vehicle.render_geometry.parts[0].position_body_m.x += 0.125;
        assert!(!descriptor.matches_vehicle(VehicleId::new(1), &vehicle));
        let mut bad = catalogue.clone();
        let RenderAppearance::Available(render) = &mut bad.vehicles[0].render else {
            unreachable!()
        };
        Arc::make_mut(&mut render.parts[0].mesh).indices[0] = 99999;
        assert!(bad.validate().is_err());
        vehicle.render_geometry.parts[0].mesh = Arc::new(RenderMesh {
            positions: vec![DVec3::ZERO; MAX_RENDER_VERTICES + 1],
            normals: vec![DVec3::Z; MAX_RENDER_VERTICES + 1],
            indices: vec![0, 1, 2],
        });
        let omitted = FleetVehicleGeometry::from_vehicle(VehicleId::new(1), &vehicle);
        assert_eq!(omitted.render, RenderAppearance::OmittedBudget);
        assert!(omitted.matches_vehicle(VehicleId::new(1), &vehicle));
    }

    fn vehicle() -> VehicleDefinition {
        let mut vehicle = X15StarterProfile::new().unwrap().vehicle;
        vehicle.name = "物理\n船".repeat(64);
        vehicle.collision_geometry = CollisionGeometry::new(vec![
            CollisionPart::new(
                DVec3::new(-4.0, 1.0, 2.0),
                DQuat::from_rotation_z(0.4),
                CollisionShape::Cuboid {
                    half_extents_m: DVec3::new(2.0, 0.5, 0.75),
                },
                CollisionMaterial::default(),
            )
            .unwrap(),
        ])
        .unwrap();
        vehicle
    }

    #[test]
    fn exact_authoritative_geometry_and_bounded_label_roundtrip_without_asset_paths() {
        let vehicle = vehicle();
        let descriptor = FleetVehicleGeometry::from_vehicle(VehicleId::new(1), &vehicle);
        assert!(descriptor.matches_vehicle(VehicleId::new(1), &vehicle));
        assert!(descriptor.display_name.len() <= MAX_DISPLAY_NAME_BYTES);
        assert!(!descriptor.display_name.chars().any(char::is_control));
        assert_eq!(
            descriptor.contact,
            ContactGeometry::Available(vehicle.collision_geometry.clone())
        );
        let catalogue = FleetGeometry {
            revision: 1,
            primary: None,
            vehicles: vec![descriptor],
        };
        catalogue.validate().unwrap();
        let frame = encode_fleet_geometry(&catalogue).unwrap();
        assert_eq!(roundtrip(&frame), catalogue);
        let mut shifted = vehicle.clone();
        shifted.collision_geometry.parts[0].local_position_m.x += 0.01;
        assert!(!catalogue.vehicles[0].matches_vehicle(VehicleId::new(1), &shifted));
        shifted = vehicle.clone();
        shifted.name = "different label".into();
        assert!(!catalogue.vehicles[0].matches_vehicle(VehicleId::new(1), &shifted));
    }

    #[test]
    fn absent_and_budget_omission_never_substitute_a_partial_hull() {
        let mut vehicle = vehicle();
        vehicle.collision_geometry.parts.resize(
            MAX_GEOMETRY_PARTS_PER_VEHICLE,
            vehicle.collision_geometry.parts[0],
        );
        assert!(matches!(
            FleetVehicleGeometry::from_vehicle(VehicleId::new(1), &vehicle).contact,
            ContactGeometry::Available(_)
        ));
        vehicle
            .collision_geometry
            .parts
            .push(vehicle.collision_geometry.parts[0]);
        let omitted = FleetVehicleGeometry::from_vehicle(VehicleId::new(1), &vehicle);
        assert_eq!(omitted.contact, ContactGeometry::OmittedBudget);
        assert!(omitted.matches_vehicle(VehicleId::new(1), &vehicle));
        vehicle.collision_geometry.parts.clear();
        assert_eq!(
            FleetVehicleGeometry::from_vehicle(VehicleId::new(1), &vehicle).contact,
            ContactGeometry::Absent
        );
    }

    #[test]
    fn reject_nonfinite_shapes_poses_ids_names_and_unbounded_catalogues() {
        let descriptor = FleetVehicleGeometry::from_vehicle(VehicleId::new(1), &vehicle());
        let valid = FleetGeometry {
            primary: None,
            revision: 1,
            vehicles: vec![descriptor.clone()],
        };
        let mut bad = valid.clone();
        bad.revision = 0;
        assert!(bad.validate().is_err());
        let mut bad = valid.clone();
        bad.vehicles[0].vehicle_id = VehicleId::PRIMARY;
        assert!(bad.validate().is_err());
        let mut bad = valid.clone();
        bad.vehicles.push(descriptor);
        assert!(bad.validate().is_err());
        for name in ["x".repeat(129), "line\nbreak".into()] {
            let mut bad = valid.clone();
            bad.vehicles[0].display_name = name;
            assert!(bad.validate().is_err());
        }
        for count in [0, MAX_GEOMETRY_PARTS_PER_VEHICLE + 1] {
            let mut bad = valid.clone();
            if let ContactGeometry::Available(g) = &mut bad.vehicles[0].contact {
                g.parts.resize(count, g.parts[0]);
            }
            assert!(bad.validate().is_err());
        }
        for position in [DVec3::NAN, DVec3::INFINITY] {
            let mut bad = valid.clone();
            if let ContactGeometry::Available(g) = &mut bad.vehicles[0].contact {
                g.parts[0].local_position_m = position;
            }
            assert!(bad.validate().is_err());
        }
        let mut bad = valid.clone();
        if let ContactGeometry::Available(g) = &mut bad.vehicles[0].contact {
            g.parts[0].shape = CollisionShape::Sphere { radius_m: -1.0 };
        }
        assert!(bad.validate().is_err());
        let mut bad = valid;
        bad.vehicles = (1..=64)
            .map(|id| {
                let mut d = bad.vehicles[0].clone();
                d.vehicle_id = VehicleId::new(id);
                d
            })
            .collect();
        assert!(bad.validate().is_err());
    }
}
