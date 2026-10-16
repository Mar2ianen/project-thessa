//! Retained appearance geometry. Never used for mass, contact or aerodynamic forces.
use std::sync::Arc;

use glam::{DQuat, DVec3};
use serde::{Deserialize, Serialize};

/// Renderer-neutral indexed triangles in mesh-local metres, CCW outward.
/// Shared immutable payloads survive definition clones and cluster partitioning.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct RenderMesh {
    pub positions: Vec<DVec3>,
    pub normals: Vec<DVec3>,
    pub indices: Vec<u32>,
}

impl RenderMesh {
    pub fn validate(&self) -> Result<(), String> {
        if self.positions.is_empty()
            || self.positions.len() != self.normals.len()
            || self.indices.is_empty()
            || !self.indices.len().is_multiple_of(3)
            || !self.positions.iter().all(|p| p.is_finite())
            || !self
                .normals
                .iter()
                .all(|n| n.is_finite() && (n.length_squared() - 1.0).abs() <= 1e-6)
            || self
                .indices
                .iter()
                .any(|i| *i as usize >= self.positions.len())
        {
            return Err(
                "render mesh needs finite vertices, unit normals and in-range triangles".into(),
            );
        }
        Ok(())
    }
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct RenderPart {
    /// Display/debug name, not a path, loader instruction or global model key.
    pub name: String,
    /// Index in VehicleAssembly::body_names. None is valid only without an assembly.
    pub assembly_body: Option<usize>,
    pub position_body_m: DVec3,
    pub orientation_mesh_to_body: DQuat,
    pub mesh: Arc<RenderMesh>,
}

#[derive(Debug, Clone, Default, PartialEq, Serialize, Deserialize)]
pub struct RenderGeometry {
    pub parts: Vec<RenderPart>,
}

impl RenderGeometry {
    pub fn validate(&self, body_count: Option<usize>) -> Result<(), String> {
        for part in &self.parts {
            if part.name.trim().is_empty()
                || !part.position_body_m.is_finite()
                || !part.orientation_mesh_to_body.is_finite()
                || (part.orientation_mesh_to_body.length_squared() - 1.0).abs() > 1e-6
                || match (body_count, part.assembly_body) {
                    (Some(count), Some(body)) => body >= count,
                    (None, None) => false,
                    _ => true,
                }
            {
                return Err(
                    "render part needs a name, finite rigid pose and valid assembly ownership"
                        .into(),
                );
            }
            part.mesh.validate()?;
        }
        Ok(())
    }

    /// Partition without copying vertex/index buffers; keep the mesh frame and
    /// move its f64 body-local origin exactly as the physical cluster does.
    pub(crate) fn for_cluster(&self, bodies: &[usize], shift: DVec3) -> Self {
        Self {
            parts: self
                .parts
                .iter()
                .filter_map(|part| {
                    let local_body = bodies
                        .iter()
                        .position(|body| Some(*body) == part.assembly_body)?;
                    let mut part = part.clone();
                    part.assembly_body = Some(local_body);
                    part.position_body_m += shift;
                    Some(part)
                })
                .collect(),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    pub(crate) fn triangle() -> Arc<RenderMesh> {
        Arc::new(RenderMesh {
            positions: vec![DVec3::ZERO, DVec3::X, DVec3::Y],
            normals: vec![DVec3::Z; 3],
            indices: vec![0, 1, 2],
        })
    }

    #[test]
    fn validation_rejects_bad_ownership_pose_and_mesh() {
        let part = RenderPart {
            name: "body".into(),
            assembly_body: Some(2),
            position_body_m: DVec3::ZERO,
            orientation_mesh_to_body: DQuat::IDENTITY,
            mesh: triangle(),
        };
        let mut geometry = RenderGeometry { parts: vec![part] };
        geometry.validate(Some(3)).unwrap();
        assert!(geometry.validate(Some(2)).is_err());
        assert!(geometry.validate(None).is_err());
        geometry.parts[0].assembly_body = None;
        geometry.validate(None).unwrap();
        geometry.parts[0].orientation_mesh_to_body = DQuat::from_xyzw(0.0, 0.0, 0.0, 2.0);
        assert!(geometry.validate(None).is_err());
        let mut mesh = (*triangle()).clone();
        mesh.indices[2] = 3;
        assert!(mesh.validate().is_err());
        mesh.indices[2] = 2;
        mesh.normals[0] = DVec3::ZERO;
        assert!(mesh.validate().is_err());
        mesh.normals[0] = DVec3::Z;
        mesh.positions[0].x = f64::NAN;
        assert!(mesh.validate().is_err());
    }

    #[test]
    fn partition_remaps_owner_and_shares_payload_roundtrip_is_exact() {
        let mesh = triangle();
        let geometry = RenderGeometry {
            parts: vec![RenderPart {
                name: "third".into(),
                assembly_body: Some(2),
                position_body_m: DVec3::new(1.0, 2.0, 3.0),
                orientation_mesh_to_body: DQuat::from_rotation_x(0.3),
                mesh: mesh.clone(),
            }],
        };
        let split = geometry.for_cluster(&[1, 2], DVec3::new(-4.0, 0.0, 0.0));
        split.validate(Some(2)).unwrap();
        assert_eq!(split.parts[0].assembly_body, Some(1));
        assert_eq!(split.parts[0].position_body_m, DVec3::new(-3.0, 2.0, 3.0));
        assert!(Arc::ptr_eq(&mesh, &split.parts[0].mesh));
        assert!(geometry.for_cluster(&[0], DVec3::ZERO).parts.is_empty());
        let decoded: RenderGeometry =
            serde_json::from_str(&serde_json::to_string(&split).unwrap()).unwrap();
        assert_eq!(decoded, split);
    }
}
