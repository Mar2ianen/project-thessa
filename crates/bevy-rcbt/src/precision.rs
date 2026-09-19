//! Precision boundary for cube-sphere CBT geometry.
//!
//! The render shader must stay in portable WGSL `f32`.  A planet-sized
//! position is therefore never reconstructed by subtracting two nearly equal
//! radius-sized `f32` values.  The CPU computes the tile centre in `f64`,
//! subtracts the floating-origin camera in `f64`, and sends a camera-relative
//! anchor plus a small, normalized-cube frame to the shader.
//!
//! ## GPU contract
//!
//! [`GpuTileFrame`] is seven 16-byte records, in this order:
//!
//! 1. `anchor_hi_m`, camera-relative base-radius anchor, high words;
//! 2. `anchor_lo_m`, residual words for the same anchor;
//! 3. `raw_center`, the unnormalized cube-face coordinate at tile centre;
//! 4. `raw_axis_u`, cube-face derivative for increasing tile `u`;
//! 5. `raw_axis_v`, cube-face derivative for increasing tile `v`;
//! 6. `normal`, normalized `raw_center`;
//! 7. `radius_half_extent_len`, `(radius_m, 1 / 2^level, |raw_center|, 0)`.
//!
//! A portable WGSL consumer computes the vertex position as follows.  All
//! terms named `delta`, `radial`, `tangent`, and `bend` are small local values;
//! the only large value is added once, from the camera-relative anchor.
//!
//! ```text
//! delta = raw_axis_u * (2 * (u - .5) * half_extent)
//!       + raw_axis_v * (2 * (v - .5) * half_extent)
//! radial = dot(normal, delta)
//! tangent = delta - normal * radial
//! q = raw_center_length + radial
//! t2 = dot(tangent, tangent)
//! q_length = sqrt(q*q + t2)
//! bend = -t2 / (q_length * (q_length + q))
//! offset = tangent * ((radius + height) / q_length)
//!        + normal * (height + (radius + height) * bend)
//! position_camera_relative = anchor_hi_m + anchor_lo_m + offset
//! ```
//!
//! The `bend` form avoids subtracting two almost equal unit vectors.  For a
//! tile centre, `delta == 0`, so the output is exactly the anchor plus radial
//! terrain height.  The frame is intended for the current CBT depth-17
//! leaves; the raw-coordinate local span remains representable in `f32` up to
//! [`GPU_STABLE_LEVEL_MAX`].

/// Current CBT encoding uses levels 0..=17 (heap depths 3..=37).  This is a
/// conservative portable-WGSL limit for the local raw-cube span, rather than
/// a limit on the CPU-side address representation.
pub const GPU_STABLE_LEVEL_MAX: u8 = 22;

/// Number of 16-byte records in [`GpuTileFrame`].
pub const GPU_TILE_FRAME_VEC4S: usize = 7;
/// Byte stride of one [`GpuTileFrame`] when copied to a storage buffer.
pub const GPU_TILE_FRAME_STRIDE_BYTES: usize = GPU_TILE_FRAME_VEC4S * 16;

/// Cube face and integer tile coordinates.  Face numbering matches the
/// existing CBT WGSL `face_direction` function in `render.rs`.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct TileKey {
    pub face: u8,
    pub level: u8,
    pub x: u32,
    pub y: u32,
}

impl TileKey {
    /// Construct a valid tile.  `level` is allowed through 31 for CPU use;
    /// [`GpuTileFrame`] documents the smaller portable local-coordinate bound.
    pub fn new(face: u8, level: u8, x: u32, y: u32) -> Option<Self> {
        if face >= 6 || level > 31 {
            return None;
        }
        let side = 1_u32.checked_shl(u32::from(level))?;
        (x < side && y < side).then_some(Self { face, level, x, y })
    }
}

/// CPU-computed frame for one CBT tile.
#[derive(Clone, Copy, Debug)]
pub struct TileAnchor {
    pub tile: TileKey,
    /// Base-radius point at the tile centre, in body metres.
    pub anchor_body_m: [f64; 3],
    /// Unnormalized cube coordinate at the tile centre and its face axes.
    pub raw_center: [f64; 3],
    pub raw_axis_u: [f64; 3],
    pub raw_axis_v: [f64; 3],
    /// Normalized direction at the tile centre.
    pub normal: [f64; 3],
    pub radius_m: f64,
    pub half_extent: f64,
    pub raw_center_length: f64,
}

/// Camera-relative, f32-compatible transport record for the WGSL consumer.
/// Each field is a 16-byte record so an array has no vec3 alignment ambiguity.
#[repr(C)]
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct GpuTileFrame {
    pub anchor_hi_m: [f32; 4],
    pub anchor_lo_m: [f32; 4],
    pub raw_center: [f32; 4],
    pub raw_axis_u: [f32; 4],
    pub raw_axis_v: [f32; 4],
    pub normal: [f32; 4],
    pub radius_half_extent_len: [f32; 4],
}

impl TileAnchor {
    /// Compute the f64 anchor and normalized-cube frame for a tile.
    pub fn new(tile: TileKey, radius_m: f64) -> Option<Self> {
        if !radius_m.is_finite() || radius_m <= 0.0 {
            return None;
        }
        let scale = 2_f64.powi(i32::from(tile.level));
        let a = 2.0 * (f64::from(tile.x) + 0.5) / scale - 1.0;
        let b = 2.0 * (f64::from(tile.y) + 0.5) / scale - 1.0;
        let (raw_center, raw_axis_u, raw_axis_v) = face_frame(tile.face, a, b);
        let raw_center_length = length(raw_center);
        let normal = scale_vec(raw_center, 1.0 / raw_center_length);
        Some(Self {
            tile,
            anchor_body_m: scale_vec(normal, radius_m),
            raw_center,
            raw_axis_u,
            raw_axis_v,
            normal,
            radius_m,
            half_extent: 1.0 / scale,
            raw_center_length,
        })
    }

    /// Project one page sample in body metres using the stable local formula.
    pub fn project_body_m(&self, uv: [f64; 2], height_m: f64) -> [f64; 3] {
        let offset = stable_offset(
            self.raw_axis_u,
            self.raw_axis_v,
            self.normal,
            self.raw_center_length,
            self.half_extent,
            uv,
            self.radius_m,
            height_m,
        );
        add(self.anchor_body_m, offset)
    }

    /// Build the camera-relative transport record.  The subtraction happens
    /// in f64 before either word is converted to f32.
    pub fn to_gpu(&self, camera_body_m: [f64; 3]) -> Option<GpuTileFrame> {
        if camera_body_m.iter().any(|value| !value.is_finite()) {
            return None;
        }
        let relative = sub(self.anchor_body_m, camera_body_m);
        let (anchor_hi, anchor_lo) = split_f64_vec(relative);
        let f = |v: [f64; 3]| [v[0] as f32, v[1] as f32, v[2] as f32, 0.0];
        Some(GpuTileFrame {
            anchor_hi_m: [anchor_hi[0], anchor_hi[1], anchor_hi[2], 0.0],
            anchor_lo_m: [anchor_lo[0], anchor_lo[1], anchor_lo[2], 0.0],
            raw_center: f(self.raw_center),
            raw_axis_u: f(self.raw_axis_u),
            raw_axis_v: f(self.raw_axis_v),
            normal: f(self.normal),
            radius_half_extent_len: [
                self.radius_m as f32,
                self.half_extent as f32,
                self.raw_center_length as f32,
                0.0,
            ],
        })
    }
}

impl GpuTileFrame {
    /// Return the exact seven-vec4 storage layout documented at the top of
    /// this module.  Keeping this explicit avoids relying on Rust struct
    /// layout or a bytemuck dependency at the render boundary.
    pub fn words(&self) -> [[f32; 4]; GPU_TILE_FRAME_VEC4S] {
        [
            self.anchor_hi_m,
            self.anchor_lo_m,
            self.raw_center,
            self.raw_axis_u,
            self.raw_axis_v,
            self.normal,
            self.radius_half_extent_len,
        ]
    }

    /// CPU mirror of the WGSL contract, useful for numerical regression tests.
    pub fn project_local_f32(&self, uv: [f32; 2], height_m: f32) -> [f32; 3] {
        let delta_u = 2.0 * (uv[0] - 0.5) * self.radius_half_extent_len[1];
        let delta_v = 2.0 * (uv[1] - 0.5) * self.radius_half_extent_len[1];
        let delta = [
            self.raw_axis_u[0] * delta_u + self.raw_axis_v[0] * delta_v,
            self.raw_axis_u[1] * delta_u + self.raw_axis_v[1] * delta_v,
            self.raw_axis_u[2] * delta_u + self.raw_axis_v[2] * delta_v,
        ];
        let normal = [self.normal[0], self.normal[1], self.normal[2]];
        let radial = dot32(normal, delta);
        let tangent = sub32(delta, scale_vec32(normal, radial));
        let q = self.radius_half_extent_len[2] + radial;
        let t2 = dot32(tangent, tangent);
        let q_length = (q * q + t2).sqrt();
        let bend = if t2 == 0.0 {
            0.0
        } else {
            -t2 / (q_length * (q_length + q))
        };
        let radius = self.radius_half_extent_len[0];
        let radial_offset = height_m + (radius + height_m) * bend;
        let tangent_scale = (radius + height_m) / q_length;
        let offset = add32(
            scale_vec32(tangent, tangent_scale),
            scale_vec32(normal, radial_offset),
        );
        let anchor = [
            self.anchor_hi_m[0] + self.anchor_lo_m[0],
            self.anchor_hi_m[1] + self.anchor_lo_m[1],
            self.anchor_hi_m[2] + self.anchor_lo_m[2],
        ];
        [
            anchor[0] + offset[0],
            anchor[1] + offset[1],
            anchor[2] + offset[2],
        ]
    }
}

fn face_frame(face: u8, a: f64, b: f64) -> ([f64; 3], [f64; 3], [f64; 3]) {
    match face {
        0 => ([1.0, b, -a], [0.0, 0.0, -1.0], [0.0, 1.0, 0.0]),
        1 => ([-1.0, b, a], [0.0, 0.0, 1.0], [0.0, 1.0, 0.0]),
        2 => ([a, 1.0, -b], [1.0, 0.0, 0.0], [0.0, 0.0, -1.0]),
        3 => ([a, -1.0, b], [1.0, 0.0, 0.0], [0.0, 0.0, 1.0]),
        4 => ([a, b, 1.0], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0]),
        5 => ([-a, b, -1.0], [-1.0, 0.0, 0.0], [0.0, 1.0, 0.0]),
        _ => unreachable!("TileKey validates face"),
    }
}

#[allow(clippy::too_many_arguments)]
fn stable_offset(
    axis_u: [f64; 3],
    axis_v: [f64; 3],
    normal: [f64; 3],
    raw_center_length: f64,
    half_extent: f64,
    uv: [f64; 2],
    radius_m: f64,
    height_m: f64,
) -> [f64; 3] {
    let delta_u = 2.0 * (uv[0] - 0.5) * half_extent;
    let delta_v = 2.0 * (uv[1] - 0.5) * half_extent;
    let delta = add(scale_vec(axis_u, delta_u), scale_vec(axis_v, delta_v));
    let radial = dot(normal, delta);
    let tangent = sub(delta, scale_vec(normal, radial));
    let q = raw_center_length + radial;
    let t2 = dot(tangent, tangent);
    let q_length = (q * q + t2).sqrt();
    let bend = if t2 == 0.0 {
        0.0
    } else {
        -t2 / (q_length * (q_length + q))
    };
    add(
        scale_vec(tangent, (radius_m + height_m) / q_length),
        scale_vec(normal, height_m + (radius_m + height_m) * bend),
    )
}

fn split_f64(value: f64) -> (f32, f32) {
    let hi = value as f32;
    let lo = (value - f64::from(hi)) as f32;
    (hi, lo)
}

fn split_f64_vec(value: [f64; 3]) -> ([f32; 3], [f32; 3]) {
    let (x_hi, x_lo) = split_f64(value[0]);
    let (y_hi, y_lo) = split_f64(value[1]);
    let (z_hi, z_lo) = split_f64(value[2]);
    ([x_hi, y_hi, z_hi], [x_lo, y_lo, z_lo])
}

fn length(v: [f64; 3]) -> f64 {
    dot(v, v).sqrt()
}

fn dot(a: [f64; 3], b: [f64; 3]) -> f64 {
    a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
}

fn add(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
    [a[0] + b[0], a[1] + b[1], a[2] + b[2]]
}

fn sub(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
    [a[0] - b[0], a[1] - b[1], a[2] - b[2]]
}

fn scale_vec(v: [f64; 3], scalar: f64) -> [f64; 3] {
    [v[0] * scalar, v[1] * scalar, v[2] * scalar]
}

fn dot32(a: [f32; 3], b: [f32; 3]) -> f32 {
    a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
}

fn sub32(a: [f32; 3], b: [f32; 3]) -> [f32; 3] {
    [a[0] - b[0], a[1] - b[1], a[2] - b[2]]
}

fn add32(a: [f32; 3], b: [f32; 3]) -> [f32; 3] {
    [a[0] + b[0], a[1] + b[1], a[2] + b[2]]
}

fn scale_vec32(v: [f32; 3], scalar: f32) -> [f32; 3] {
    [v[0] * scalar, v[1] * scalar, v[2] * scalar]
}

#[cfg(test)]
mod tests {
    use super::*;

    const CASES: &[(f64, f64, f64)] = &[
        (6_371_000.0, -11_000.0, 11_000.0), // Earth
        (1_737_400.0, -9_000.0, 11_000.0),  // Moon
        (3_200_000.0, -8_000.0, 12_000.0),  // Thessa
    ];

    fn max_error(a: [f32; 3], b: [f64; 3]) -> f64 {
        a.into_iter()
            .zip(b)
            .map(|(x, y)| (f64::from(x) - y).abs())
            .fold(0.0, f64::max)
    }

    #[test]
    fn all_faces_have_matching_anchor_and_center_projection() {
        for face in 0..6 {
            for level in [0, 17] {
                let side = 1_u32 << level;
                let tile = TileKey::new(face, level, side / 3, side / 2).unwrap();
                let frame = TileAnchor::new(tile, 3_200_000.0).unwrap();
                let centre = frame.project_body_m([0.5, 0.5], 0.0);
                assert!(
                    centre
                        .into_iter()
                        .zip(frame.anchor_body_m)
                        .all(|(actual, expected)| (actual - expected).abs() < 1.0e-8)
                );
            }
        }
    }

    #[test]
    fn f64_camera_subtraction_survives_large_body_coordinates() {
        let tile = TileKey::new(4, 17, 65_537, 65_535).unwrap();
        let frame = TileAnchor::new(tile, 6_371_000.0).unwrap();
        let camera = [
            frame.anchor_body_m[0] - 17.375,
            frame.anchor_body_m[1] + 3.25,
            frame.anchor_body_m[2] + 0.125,
        ];
        let gpu = frame.to_gpu(camera).unwrap();
        let reconstructed = [
            f64::from(gpu.anchor_hi_m[0]) + f64::from(gpu.anchor_lo_m[0]),
            f64::from(gpu.anchor_hi_m[1]) + f64::from(gpu.anchor_lo_m[1]),
            f64::from(gpu.anchor_hi_m[2]) + f64::from(gpu.anchor_lo_m[2]),
        ];
        let expected = sub(frame.anchor_body_m, camera);
        assert!(
            reconstructed
                .into_iter()
                .zip(expected)
                .all(|(actual, expected)| (actual - expected).abs() < 1.0e-5)
        );
        assert_eq!(gpu.words().len(), GPU_TILE_FRAME_VEC4S);
        assert_eq!(gpu.words()[0], gpu.anchor_hi_m);
        assert_eq!(gpu.words()[6], gpu.radius_half_extent_len);
    }

    #[test]
    fn portable_f32_projection_is_bounded_for_earth_moon_and_thessa() {
        for &(radius, min_height, max_height) in CASES {
            for level in [0, GPU_STABLE_LEVEL_MAX.min(17)] {
                let side = 1_u32 << level;
                let tile = TileKey::new(4, level, side / 2, side / 2).unwrap();
                let frame = TileAnchor::new(tile, radius).unwrap();
                let camera = [
                    frame.anchor_body_m[0] - 113.25,
                    frame.anchor_body_m[1] + 41.5,
                    frame.anchor_body_m[2] - 7.0,
                ];
                let gpu = frame.to_gpu(camera).unwrap();
                for uv in [[0.0, 0.0], [0.5, 0.5], [1.0, 1.0], [0.17, 0.83]] {
                    for height in [min_height, 0.0, max_height] {
                        let expected = sub(direct_body_projection(&frame, uv, height), camera);
                        let actual =
                            gpu.project_local_f32([uv[0] as f32, uv[1] as f32], height as f32);
                        let error = max_error(actual, expected);
                        // L0 has deliberately broad geometry and is allowed a
                        // larger absolute f32 error.  L17 remains sub-metre.
                        let bound = if level == 0 { 4.0 } else { 1.0 };
                        assert!(
                            error <= bound,
                            "radius={radius} level={level} uv={uv:?} height={height}: error={error}"
                        );
                    }
                }
            }
        }
    }

    fn direct_body_projection(frame: &TileAnchor, uv: [f64; 2], height_m: f64) -> [f64; 3] {
        let delta = add(
            scale_vec(frame.raw_axis_u, 2.0 * (uv[0] - 0.5) * frame.half_extent),
            scale_vec(frame.raw_axis_v, 2.0 * (uv[1] - 0.5) * frame.half_extent),
        );
        let raw = add(frame.raw_center, delta);
        let direction = scale_vec(raw, 1.0 / length(raw));
        scale_vec(direction, frame.radius_m + height_m)
    }
}
