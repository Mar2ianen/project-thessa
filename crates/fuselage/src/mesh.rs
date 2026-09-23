//! Renderer-independent triangle mesh tessellation for procedural fuselages.

use glam::DVec3;

use crate::{BodyStation, FuselageError, ProceduralBody, outline_point};

/// An indexed triangle mesh in body/vehicle metres.
///
/// Indices are grouped in triples. Positions and normals use `f64` and
/// `glam` only; this type has no renderer or graphics API semantics.
#[derive(Debug, Clone, PartialEq)]
pub struct FuselageMesh {
    /// Vertex positions in the vehicle body frame, including body origin.
    pub positions: Vec<DVec3>,
    /// Unit vertex normals, aligned one-to-one with `positions`.
    pub normals: Vec<DVec3>,
    /// Counter-clockwise triangle indices for outward-facing surfaces.
    pub indices: Vec<u32>,
}

/// Tessellate a fuselage loft into a closed, indexed triangular mesh.
///
/// `axial_samples_per_station` is the number of subdivisions in each interval
/// between authored stations (and must be at least one). Authored stations
/// are always emitted exactly. `radial_samples` is the number of outline
/// vertices per ring and must be at least eight. The first and last cross
/// sections are closed with flat caps.
pub fn tessellate_fuselage(
    body: &ProceduralBody,
    axial_samples_per_station: usize,
    radial_samples: usize,
) -> Result<FuselageMesh, FuselageError> {
    body.validate()?;
    if axial_samples_per_station == 0 {
        return Err(FuselageError::InvalidOptions(
            "axial samples per station interval must be >= 1".into(),
        ));
    }
    if radial_samples < 8 {
        return Err(FuselageError::InvalidOptions(format!(
            "radial samples must be >= 8 (got {radial_samples})"
        )));
    }

    let interval_count = body.stations.len() - 1;
    let ring_count = interval_count
        .checked_mul(axial_samples_per_station)
        .and_then(|subdivisions| subdivisions.checked_add(1))
        .ok_or_else(|| mesh_size_error("axial ring count overflows usize"))?;
    let side_vertex_count = ring_count
        .checked_mul(radial_samples)
        .ok_or_else(|| mesh_size_error("side vertex count overflows usize"))?;
    let cap_vertex_count = radial_samples
        .checked_add(1)
        .and_then(|per_cap| per_cap.checked_mul(2))
        .ok_or_else(|| mesh_size_error("cap vertex count overflows usize"))?;
    let vertex_count = side_vertex_count
        .checked_add(cap_vertex_count)
        .ok_or_else(|| mesh_size_error("vertex count overflows usize"))?;
    if vertex_count > u32::MAX as usize {
        return Err(mesh_size_error("mesh exceeds the u32 index range"));
    }

    let side_index_count = (ring_count - 1)
        .checked_mul(radial_samples)
        .and_then(|quads| quads.checked_mul(6))
        .ok_or_else(|| mesh_size_error("side index count overflows usize"))?;
    let index_count = side_index_count
        .checked_add(
            radial_samples
                .checked_mul(6)
                .ok_or_else(|| mesh_size_error("cap triangle index count overflows usize"))?,
        )
        .ok_or_else(|| mesh_size_error("mesh index count overflows usize"))?;

    let mut positions = Vec::with_capacity(vertex_count);
    let angular_step = std::f64::consts::TAU / radial_samples as f64;
    positions.extend(sample_ring(
        body,
        body.stations[0],
        radial_samples,
        angular_step,
    ));
    for pair in body.stations.windows(2) {
        for sample in 1..=axial_samples_per_station {
            let t = sample as f64 / axial_samples_per_station as f64;
            let station = pair[0].lerp(pair[1], t);
            positions.extend(sample_ring(body, station, radial_samples, angular_step));
        }
    }
    if positions.iter().any(|position| !position.is_finite()) {
        return Err(mesh_geometry_error("sampled vertex is not finite"));
    }

    let mut indices = Vec::with_capacity(index_count);
    let mut side_normals = vec![DVec3::ZERO; side_vertex_count];
    for ring in 0..ring_count - 1 {
        let lower = ring * radial_samples;
        let upper = (ring + 1) * radial_samples;
        for radial in 0..radial_samples {
            let next = (radial + 1) % radial_samples;
            let a = lower + radial;
            let b = lower + next;
            let c = upper + radial;
            let d = upper + next;
            // Increasing clock angle runs +Y toward +Z. These triangles
            // therefore face outward for the +X tail-to-nose axis.
            push_side_triangle(&positions, &mut indices, &mut side_normals, a, b, c)?;
            push_side_triangle(&positions, &mut indices, &mut side_normals, b, d, c)?;
        }
    }

    let mut normals = Vec::with_capacity(vertex_count);
    for normal in side_normals {
        normals.push(normalize_checked(normal)?);
    }

    append_cap(
        body,
        body.stations[0],
        0,
        radial_samples,
        DVec3::NEG_X,
        false,
        &mut positions,
        &mut normals,
        &mut indices,
    )?;
    append_cap(
        body,
        *body.stations.last().expect("validated body has stations"),
        (ring_count - 1) * radial_samples,
        radial_samples,
        DVec3::X,
        true,
        &mut positions,
        &mut normals,
        &mut indices,
    )?;

    debug_assert_eq!(positions.len(), vertex_count);
    debug_assert_eq!(normals.len(), vertex_count);
    debug_assert_eq!(indices.len(), index_count);
    Ok(FuselageMesh {
        positions,
        normals,
        indices,
    })
}

fn sample_ring(
    body: &ProceduralBody,
    station: BodyStation,
    radial_samples: usize,
    angular_step: f64,
) -> Vec<DVec3> {
    (0..radial_samples)
        .map(|radial| {
            let (y, z) = outline_point(
                station.half_width_m,
                station.top_height_m,
                station.bottom_height_m,
                station.top_exponent,
                station.bottom_exponent,
                radial as f64 * angular_step,
            );
            body.origin_body_m
                + DVec3::new(station.x_m, station.offset_y_m + y, station.offset_z_m + z)
        })
        .collect()
}

#[allow(clippy::too_many_arguments)]
fn append_cap(
    body: &ProceduralBody,
    station: BodyStation,
    side_ring_start: usize,
    radial_samples: usize,
    normal: DVec3,
    nose_cap: bool,
    positions: &mut Vec<DVec3>,
    normals: &mut Vec<DVec3>,
    indices: &mut Vec<u32>,
) -> Result<(), FuselageError> {
    let center =
        body.origin_body_m + DVec3::new(station.x_m, station.offset_y_m, station.offset_z_m);
    if !center.is_finite() {
        return Err(mesh_geometry_error("cap center is not finite"));
    }
    let cap_start = u32::try_from(positions.len())
        .map_err(|_| mesh_size_error("mesh exceeds the u32 index range"))?;
    let boundary = positions[side_ring_start..side_ring_start + radial_samples].to_vec();
    positions.push(center);
    positions.extend(boundary);
    normals.extend(std::iter::repeat(normal).take(radial_samples + 1));

    // Caps own their boundary vertices so their normals remain flat and
    // independent from the smoothly averaged loft normals.
    for radial in 0..radial_samples {
        let current = cap_start + 1 + radial as u32;
        let next = cap_start + 1 + ((radial + 1) % radial_samples) as u32;
        let (b, c) = if nose_cap {
            (current, next)
        } else {
            (next, current)
        };
        triangle_normal(
            positions[cap_start as usize],
            positions[b as usize],
            positions[c as usize],
        )?;
        indices.extend([cap_start, b, c]);
    }
    Ok(())
}

fn push_side_triangle(
    positions: &[DVec3],
    indices: &mut Vec<u32>,
    normals: &mut [DVec3],
    a: usize,
    b: usize,
    c: usize,
) -> Result<(), FuselageError> {
    let face_normal = triangle_normal(positions[a], positions[b], positions[c])?;
    for vertex in [a, b, c] {
        normals[vertex] += face_normal;
    }
    indices.extend([a as u32, b as u32, c as u32]);
    Ok(())
}

fn triangle_normal(a: DVec3, b: DVec3, c: DVec3) -> Result<DVec3, FuselageError> {
    let cross = (b - a).cross(c - a);
    if !cross.is_finite() || cross == DVec3::ZERO {
        return Err(mesh_geometry_error(
            "tessellation produced a degenerate triangle",
        ));
    }
    Ok(cross)
}

fn normalize_checked(vector: DVec3) -> Result<DVec3, FuselageError> {
    let scale = vector.x.abs().max(vector.y.abs()).max(vector.z.abs());
    if !scale.is_finite() || scale == 0.0 {
        return Err(mesh_geometry_error(
            "tessellation produced an invalid vertex normal",
        ));
    }
    let scaled = vector / scale;
    let length = scaled.length();
    if !length.is_finite() || length == 0.0 {
        return Err(mesh_geometry_error(
            "tessellation produced an invalid vertex normal",
        ));
    }
    Ok(scaled / length)
}

fn mesh_size_error(message: &str) -> FuselageError {
    FuselageError::InvalidOptions(message.into())
}

fn mesh_geometry_error(message: &str) -> FuselageError {
    FuselageError::InvalidBody(message.into())
}

#[cfg(test)]
#[path = "mesh_tests.rs"]
mod tests;
