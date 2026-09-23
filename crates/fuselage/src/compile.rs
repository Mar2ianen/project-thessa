//! Hangar compiler: authoring body to solver-ready zones and mass data.
//!
//! The compiler samples the station loft, splits at every authored station
//! and subdivides long or fast-changing intervals for solver locality,
//! then derives per-zone solver inputs. Section area and first moments are
//! adaptively integrated along the actual interpolated loft, so changing
//! width and height together is not mistaken for a linear-area frustum.
//! Lateral skin area and its material moments come from a converged
//! triangulated loft integral, including changing section shape and
//! centerline offsets.
//!
//! Body aerodynamics follows Munk/slender-body strip logic: each axial
//! zone carries the potential-flow normal force of its signed section-area
//! change (`2 * |dA|`, with force direction following the sign) through a
//! geometry-derived interference value. A cylinder gets (correctly)
//! almost no potential normal force, ogive noses carry nose suction, and
//! boat-tails retain their opposite-sign contribution. Centers of pressure
//! follow the first moment of area change. Viscous crossflow at high alpha
//! arrives through the solver's separated `sin^2` branch, not a body knob.

use glam::{DMat3, DVec3, DVec4};
use serde::{Deserialize, Serialize};
use thessa_sim_core::{
    AeroPanel, ControlSurfaceDefinition, Propellant, TankMount, TankShape, TankSpec,
    diederich_lift_slope,
};

use crate::summary::CompiledBodySummary;
use crate::{
    BodyControlPlane, BodyStation, CompiledHull, FuselageError, InteriorRegion, ProceduralBody,
    RegionKind, outline_point, point_inertia,
};

/// Subdivision tolerances for one compilation.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct BodyCompileOptions {
    /// Split an axial interval while it exceeds this length (solver
    /// locality for CP travel and local flow).
    pub max_zone_length_m: f64,
    /// Split while sampled quarter/midpoint section area deviates from
    /// linear area interpolation by more than this fraction. Area reversals
    /// always split so signed Munk contributions remain local.
    pub max_area_change_frac: f64,
    /// Hard recursion cap per base interval.
    pub max_depth: u32,
    /// Relative estimated tolerance for triangulated loft skin area.
    #[serde(default = "default_surface_area_tolerance")]
    pub surface_area_tolerance: f64,
    /// Angular samples for loft skin, end-cap, and frame polygons (>= 64).
    pub radial_samples: usize,
}

fn default_surface_area_tolerance() -> f64 {
    1.0e-6
}

impl Default for BodyCompileOptions {
    fn default() -> Self {
        Self {
            max_zone_length_m: 1.5,
            max_area_change_frac: 0.15,
            max_depth: 12,
            surface_area_tolerance: default_surface_area_tolerance(),
            radial_samples: 1024,
        }
    }
}

impl BodyCompileOptions {
    fn validate(&self) -> Result<(), FuselageError> {
        if !self.max_zone_length_m.is_finite() || self.max_zone_length_m <= 0.0 {
            return Err(FuselageError::InvalidOptions(
                "max_zone_length_m must be finite and > 0".into(),
            ));
        }
        if !self.max_area_change_frac.is_finite()
            || self.max_area_change_frac <= 0.0
            || self.max_area_change_frac >= 1.0
        {
            return Err(FuselageError::InvalidOptions(
                "max_area_change_frac must be finite and in (0, 1)".into(),
            ));
        }
        if self.max_depth == 0 || self.max_depth > 20 {
            return Err(FuselageError::InvalidOptions(
                "max_depth must be in [1, 20]".into(),
            ));
        }
        if !self.surface_area_tolerance.is_finite()
            || self.surface_area_tolerance <= 0.0
            || self.surface_area_tolerance >= 0.1
        {
            return Err(FuselageError::InvalidOptions(
                "surface_area_tolerance must be finite and in (0, 0.1)".into(),
            ));
        }
        if self.radial_samples < 64 {
            return Err(FuselageError::InvalidOptions(
                "radial_samples must be >= 64".into(),
            ));
        }
        Ok(())
    }
}

/// One tank region compiled into the feed pipeline.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct CompiledBodyTank {
    /// Region name from authoring.
    pub region_name: String,
    /// Feed-pipeline mount (equivalent-cylinder shell, real inner volume
    /// capacity, authored initial fill kept separate from capacity).
    pub mount: TankMount,
    /// Real inner-mold region volume (m^3).
    pub inner_volume_m3: f64,
    /// Estimated absolute integration error in the inner volume (m^3).
    #[serde(default)]
    pub inner_volume_error_m3: f64,
    /// Propellant mass at the authored fill (kg).
    pub propellant_kg: f64,
}

/// One interior region with compiled volume data.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct CompiledRegion {
    pub name: String,
    pub kind: RegionKind,
    /// Usable inner-mold volume (m^3).
    pub volume_m3: f64,
    /// Estimated absolute integration error in the usable volume (m^3).
    #[serde(default)]
    pub volume_error_m3: f64,
    /// Volume centroid in body-local metres.
    pub centroid_body_m: DVec3,
    /// Declared cargo mass (kg, cargo regions only).
    pub payload_mass_kg: f64,
}

/// One interface anchor in body-local metres.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct BodyPortCompiled {
    pub name: String,
    pub kind: crate::PortKind,
    /// Anchor position in body-local metres (on the outer mold).
    pub position_body_m: DVec3,
    /// Interface axis in body coordinates (unit).
    pub axis_body_m: DVec3,
    /// Interface diameter in metres.
    pub diameter_m: f64,
}

/// Compiled fuselage: runtime-consumable output of the hangar step.
///
/// Panels plug into [`thessa_sim_core::AeroGeometry`]; tanks plug into
/// [`thessa_sim_core::VehicleDefinition`] mounts; mass aggregates like
/// wing structure in the vehicle baker.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct CompiledBody {
    /// Solver panels: two per axial zone (pitch + yaw strips).
    pub panels: Vec<AeroPanel>,
    /// User control channels resolved to this body's compiled panel indices.
    pub controls: Vec<ControlSurfaceDefinition>,
    /// Hull shell/frames mass data, present when the body authors a
    /// [`BodyStructuralLayout`].
    pub structure: Option<CompiledHull>,
    /// Tank regions compiled into feed-pipeline mounts.
    pub tanks: Vec<CompiledBodyTank>,
    /// All interior regions with usable volumes.
    pub interior: Vec<CompiledRegion>,
    /// Interface anchors in body-local metres.
    pub ports: Vec<BodyPortCompiled>,
    /// Geometry-only telemetry for goldens and debug views.
    pub summary: CompiledBodySummary,
}

/// Compile one body with options.
///
/// Result panels plug straight into `AeroGeometry`; tanks plug into the
/// baker's tank mounts; everything else is data for assembly and debug.
pub fn compile_body(
    body: &ProceduralBody,
    options: &BodyCompileOptions,
) -> Result<CompiledBody, FuselageError> {
    body.validate()?;
    options.validate()?;
    let compiler = Compiler::new(body, options)?;
    compiler.compile()
}

struct Compiler<'a> {
    body: &'a ProceduralBody,
    options: &'a BodyCompileOptions,
    fineness_thickness: f64,
}

#[derive(Debug, Clone, Copy)]
struct AxialLeaf {
    a: f64,
    b: f64,
}

/// Geometric quantities of one axial leaf, backing panels, mass, and the
/// subdivision error path from a single code path.
#[derive(Debug, Clone, Copy)]
struct LeafQuantities {
    length_m: f64,
    area0_m2: f64,
    area1_m2: f64,
    /// Signed area shrinkage along the tail-to-nose station direction.
    munk_delta_area_m2: f64,
    volume_m3: f64,
    volume_error_m3: f64,
    centroid_x_m: f64,
    /// First-moment center of the monotone section-area change.
    munk_cp_x_m: f64,
    centroid_y_m: f64,
    centroid_z_m: f64,
    lateral_m2: f64,
    lateral_area_error_m2: f64,
    lateral_first_moment_m3: DVec3,
    lateral_second_moment_m4: DMat3,
    avg_half_width_m: f64,
    avg_top_height_m: f64,
    avg_bottom_height_m: f64,
    /// Centerline slope (centroid differences over length): strip chord
    /// axes tilt with it, so bent/drooped bodies carry camber physics.
    slope_y: f64,
    slope_z: f64,
}

#[derive(Debug, Clone, Copy)]
struct SurfaceMoments {
    area_m2: f64,
    first_moment_m3: DVec3,
    /// `integral(r r^T dA)` in the body-local frame.
    second_moment_m4: DMat3,
    estimated_area_error_m2: f64,
}

impl SurfaceMoments {
    fn zero() -> Self {
        Self {
            area_m2: 0.0,
            first_moment_m3: DVec3::ZERO,
            second_moment_m4: DMat3::ZERO,
            estimated_area_error_m2: 0.0,
        }
    }

    fn add(mut self, other: Self) -> Self {
        self.area_m2 += other.area_m2;
        self.first_moment_m3 += other.first_moment_m3;
        self.second_moment_m4 += other.second_moment_m4;
        self.estimated_area_error_m2 += other.estimated_area_error_m2;
        self
    }

    fn add_triangle(&mut self, a: DVec3, b: DVec3, c: DVec3) {
        let area = 0.5 * (b - a).cross(c - a).length();
        if area <= 0.0 || !area.is_finite() {
            return;
        }
        let sum = a + b + c;
        let vertex_outer = outer_product(a, a) + outer_product(b, b) + outer_product(c, c);
        self.area_m2 += area;
        self.first_moment_m3 += sum * (area / 3.0);
        self.second_moment_m4 += (vertex_outer + outer_product(sum, sum)) * (area / 12.0);
    }
}

#[derive(Debug, Clone, Copy)]
struct IntegratedSection {
    moments: DVec4,
    estimated_error: DVec4,
}

impl<'a> Compiler<'a> {
    fn new(
        body: &'a ProceduralBody,
        options: &'a BodyCompileOptions,
    ) -> Result<Self, FuselageError> {
        let length = body.stations.last().unwrap().x_m - body.stations[0].x_m;
        let max_d = body
            .stations
            .iter()
            .map(|station| {
                2.0 * station
                    .half_width_m
                    .max(station.top_height_m)
                    .max(station.bottom_height_m)
            })
            .fold(0.0_f64, f64::max);
        if length <= 0.0 || max_d <= 0.0 {
            return Err(FuselageError::InvalidBody(
                "body needs positive length and diameter".into(),
            ));
        }
        Ok(Self {
            body,
            options,
            fineness_thickness: (max_d / length).min(0.3),
        })
    }

    fn section_pair(&self, a: f64, b: f64) -> (BodyStation, BodyStation) {
        (self.body.section_at(a), self.body.section_at(b))
    }

    /// Full section center (offsets plus camber) in body-local frame.
    fn section_center(&self, station: BodyStation) -> Result<DVec3, FuselageError> {
        let (cy, cz) = crate::section_centroid_yz(
            station.half_width_m,
            station.top_height_m,
            station.bottom_height_m,
            station.top_exponent,
            station.bottom_exponent,
        );
        Ok(DVec3::new(
            station.x_m,
            station.offset_y_m + cy,
            station.offset_z_m + cz,
        ))
    }

    fn sample_ring(&self, station: BodyStation, radial_samples: usize) -> Vec<DVec3> {
        let angular_step = std::f64::consts::TAU / radial_samples as f64;
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
                DVec3::new(station.x_m, station.offset_y_m + y, station.offset_z_m + z)
            })
            .collect()
    }

    fn surface_patch_from_rings(&self, ring_a: &[DVec3], ring_b: &[DVec3]) -> SurfaceMoments {
        debug_assert_eq!(ring_a.len(), ring_b.len());
        let radial_samples = ring_a.len();
        let mut surface = SurfaceMoments::zero();
        for radial in 0..radial_samples {
            let next = (radial + 1) % radial_samples;
            let a0 = ring_a[radial];
            let a1 = ring_a[next];
            let b0 = ring_b[radial];
            let b1 = ring_b[next];
            // Triangulate the ruled quadrilateral consistently around the
            // loft. Triangle moments yield the actual polygonal-surface
            // centroid and inertia instead of borrowing volume centroids.
            surface.add_triangle(a0, a1, b1);
            surface.add_triangle(a0, b1, b0);
        }
        surface
    }

    fn integrate_surface(
        &self,
        a: f64,
        b: f64,
        depth: u32,
    ) -> Result<SurfaceMoments, FuselageError> {
        let radial = self.options.radial_samples;
        let half_radial = (radial / 2).max(32);
        let ring_a = self.sample_ring(self.body.section_at(a), radial);
        let ring_b = self.sample_ring(self.body.section_at(b), radial);
        self.refine_surface(a, b, &ring_a, &ring_b, half_radial, depth)
    }

    fn refine_surface(
        &self,
        a: f64,
        b: f64,
        ring_a: &[DVec3],
        ring_b: &[DVec3],
        half_radial: usize,
        depth: u32,
    ) -> Result<SurfaceMoments, FuselageError> {
        let coarse_axial = self.surface_patch_from_rings(ring_a, ring_b);
        let mid = 0.5 * (a + b);
        let ring_mid = self.sample_ring(self.body.section_at(mid), ring_a.len());
        let left = self.surface_patch_from_rings(ring_a, &ring_mid);
        let right = self.surface_patch_from_rings(&ring_mid, ring_b);
        let fine = left.add(right);
        let axial_error = (fine.area_m2 - coarse_axial.area_m2).abs() / 3.0;
        let tolerance = self.options.surface_area_tolerance * fine.area_m2.max(1e-12);
        if axial_error > tolerance {
            if depth >= self.options.max_depth {
                return Err(FuselageError::InvalidOptions(format!(
                    "surface-area integration did not meet tolerance on [{a}, {b}]"
                )));
            }
            let left = self.refine_surface(a, mid, ring_a, &ring_mid, half_radial, depth + 1)?;
            let right = self.refine_surface(mid, b, &ring_mid, ring_b, half_radial, depth + 1)?;
            return Ok(left.add(right));
        }

        let low_ring_a = self.sample_ring(self.body.section_at(a), half_radial);
        let low_ring_mid = self.sample_ring(self.body.section_at(mid), half_radial);
        let low_ring_b = self.sample_ring(self.body.section_at(b), half_radial);
        let angular_coarse = self
            .surface_patch_from_rings(&low_ring_a, &low_ring_mid)
            .add(self.surface_patch_from_rings(&low_ring_mid, &low_ring_b));
        let radial_error = (fine.area_m2 - angular_coarse.area_m2).abs() / 3.0;
        let corrected_area = fine.area_m2
            + (fine.area_m2 - coarse_axial.area_m2) / 3.0
            + (fine.area_m2 - angular_coarse.area_m2) / 3.0;
        let moment_scale = corrected_area / fine.area_m2;
        Ok(SurfaceMoments {
            area_m2: corrected_area,
            first_moment_m3: fine.first_moment_m3 * moment_scale,
            second_moment_m4: fine.second_moment_m4 * moment_scale,
            estimated_area_error_m2: axial_error + radial_error,
        })
    }

    fn end_cap_surface(&self, station: BodyStation) -> SurfaceMoments {
        let radial = self.options.radial_samples;
        let angular_step = std::f64::consts::TAU / radial as f64;
        let center = DVec3::new(station.x_m, station.offset_y_m, station.offset_z_m);
        let mut surface = SurfaceMoments::zero();
        let mut previous = DVec3::ZERO;
        for index in 0..=radial {
            let angle = index as f64 * angular_step;
            let (y, z) = outline_point(
                station.half_width_m,
                station.top_height_m,
                station.bottom_height_m,
                station.top_exponent,
                station.bottom_exponent,
                angle,
            );
            let point = DVec3::new(station.x_m, station.offset_y_m + y, station.offset_z_m + z);
            if index > 0 {
                surface.add_triangle(center, previous, point);
            }
            previous = point;
        }
        let polygon_area = surface.area_m2;
        let exact_area = station.area_m2();
        if polygon_area > 0.0 {
            let scale = exact_area / polygon_area;
            let (centroid_y, centroid_z) = crate::section_centroid_yz(
                station.half_width_m,
                station.top_height_m,
                station.bottom_height_m,
                station.top_exponent,
                station.bottom_exponent,
            );
            surface.area_m2 = exact_area;
            surface.first_moment_m3 = DVec3::new(
                station.x_m,
                station.offset_y_m + centroid_y,
                station.offset_z_m + centroid_z,
            ) * exact_area;
            surface.second_moment_m4 *= scale;
            let low_radial = self.surface_cap_area(station, (radial / 2).max(32));
            surface.estimated_area_error_m2 =
                (exact_area - polygon_area).abs() + (polygon_area - low_radial).abs() / 3.0;
        }
        surface
    }

    fn surface_cap_area(&self, station: BodyStation, radial_samples: usize) -> f64 {
        let step = std::f64::consts::TAU / radial_samples as f64;
        let center = DVec3::new(station.x_m, station.offset_y_m, station.offset_z_m);
        let mut area = 0.0;
        let mut previous = DVec3::ZERO;
        for index in 0..=radial_samples {
            let (y, z) = outline_point(
                station.half_width_m,
                station.top_height_m,
                station.bottom_height_m,
                station.top_exponent,
                station.bottom_exponent,
                index as f64 * step,
            );
            let point = DVec3::new(station.x_m, station.offset_y_m + y, station.offset_z_m + z);
            if index > 0 {
                area += 0.5 * (previous - center).cross(point - center).length();
            }
            previous = point;
        }
        area
    }

    fn frame_mass_properties(
        &self,
        station: BodyStation,
        width_m: f64,
        areal_density_kg_m2: f64,
    ) -> (f64, DVec3, DMat3) {
        let step = std::f64::consts::TAU / self.options.radial_samples as f64;
        let mut mass = 0.0;
        let mut first_moment = DVec3::ZERO;
        let mut second_moment = DMat3::ZERO;
        let axial_variance = width_m.powi(2) / 12.0;
        let linear_density = width_m * areal_density_kg_m2;
        let mut previous: Option<DVec3> = None;
        for index in 0..=self.options.radial_samples {
            let (y, z) = outline_point(
                station.half_width_m,
                station.top_height_m,
                station.bottom_height_m,
                station.top_exponent,
                station.bottom_exponent,
                index as f64 * step,
            );
            let point = DVec3::new(station.x_m, station.offset_y_m + y, station.offset_z_m + z);
            if index > 0
                && let Some(a) = previous
            {
                let segment = point - a;
                let length = segment.length();
                let segment_mass = length * linear_density;
                let center = 0.5 * (a + point);
                let axial_spread = DMat3::from_diagonal(DVec3::new(axial_variance, 0.0, 0.0));
                mass += segment_mass;
                first_moment += center * segment_mass;
                second_moment += (outer_product(center, center)
                    + outer_product(segment, segment) / 12.0
                    + axial_spread)
                    * segment_mass;
            }
            previous = Some(point);
        }
        (
            mass,
            first_moment,
            inertia_from_second_moment(second_moment),
        )
    }

    fn leaf_quantities(&self, a: f64, b: f64) -> Result<LeafQuantities, FuselageError> {
        let (s0, s1) = self.section_pair(a, b);
        let length = b - a;
        let area0 = s0.area_m2();
        let area1 = s1.area_m2();
        let integration = self.integrate_section(a, b, 0.0)?;
        let integrated = integration.moments;
        let volume = integrated.x;
        if !volume.is_finite() || volume <= 0.0 {
            return Err(FuselageError::InvalidBody(
                "loft integration produced non-positive volume".into(),
            ));
        }
        let centroid_x = integrated.y / volume;
        let centroid_y = integrated.z / volume;
        let centroid_z = integrated.w / volume;
        let delta_area = area0 - area1;
        // For a monotone area schedule, integration by parts gives the
        // centroid of |dA/dx| without differentiating a sampled loft:
        // x_cp = (b*A1 - a*A0 - integral(A dx)) / (A1 - A0).
        let munk_cp_x = if delta_area.abs() > 1e-14 * area0.max(area1).max(f64::MIN_POSITIVE) {
            ((b * area1 - a * area0 - volume) / (area1 - area0)).clamp(a, b)
        } else {
            centroid_x
        };
        let lateral = self.integrate_surface(a, b, 0)?;
        let c0 = self.section_center(s0)?;
        let c1 = self.section_center(s1)?;
        Ok(LeafQuantities {
            length_m: length,
            area0_m2: area0,
            area1_m2: area1,
            munk_delta_area_m2: delta_area,
            volume_m3: volume,
            volume_error_m3: integration.estimated_error.x,
            centroid_x_m: centroid_x,
            munk_cp_x_m: munk_cp_x,
            centroid_y_m: centroid_y,
            centroid_z_m: centroid_z,
            lateral_m2: lateral.area_m2,
            lateral_area_error_m2: lateral.estimated_area_error_m2,
            lateral_first_moment_m3: lateral.first_moment_m3,
            lateral_second_moment_m4: lateral.second_moment_m4,
            avg_half_width_m: 0.5 * (s0.half_width_m + s1.half_width_m),
            avg_top_height_m: 0.5 * (s0.top_height_m + s1.top_height_m),
            avg_bottom_height_m: 0.5 * (s0.bottom_height_m + s1.bottom_height_m),
            slope_y: (c1.y - c0.y) / length.max(1e-12),
            slope_z: (c1.z - c0.z) / length.max(1e-12),
        })
    }

    /// Integrate area and its three first moments over the actual loft.
    /// The returned moments are `(V, ∫x dV, ∫y dV, ∫z dV)` with a
    /// componentwise adaptive-Simpson absolute-error estimate.
    fn integrate_section(
        &self,
        a: f64,
        b: f64,
        wall_m: f64,
    ) -> Result<IntegratedSection, FuselageError> {
        let evaluate = |x: f64| -> Result<DVec4, FuselageError> {
            let mut section = self.body.section_at(x);
            if wall_m > 0.0 {
                section = shrink_section(section, wall_m)?;
            }
            let area = section.area_m2();
            let (camber_y, camber_z) = crate::section_centroid_yz(
                section.half_width_m,
                section.top_height_m,
                section.bottom_height_m,
                section.top_exponent,
                section.bottom_exponent,
            );
            let center_y = section.offset_y_m + camber_y;
            let center_z = section.offset_z_m + camber_z;
            let value = DVec4::new(area, x * area, center_y * area, center_z * area);
            if value.is_finite() {
                Ok(value)
            } else {
                Err(FuselageError::InvalidBody(
                    "loft integration sampled a non-finite section".into(),
                ))
            }
        };
        let mid = 0.5 * (a + b);
        let fa = evaluate(a)?;
        let fm = evaluate(mid)?;
        let fb = evaluate(b)?;
        let whole = simpson(a, b, fa, fm, fb);
        let mut max_y_extent: f64 = 0.0;
        let mut max_z_extent: f64 = 0.0;
        for x in [a, mid, b] {
            let mut section = self.body.section_at(x);
            if wall_m > 0.0 {
                section = shrink_section(section, wall_m)?;
            }
            max_y_extent = max_y_extent.max(section.offset_y_m.abs() + section.half_width_m);
            max_z_extent = max_z_extent
                .max(section.offset_z_m.abs() + section.top_height_m.max(section.bottom_height_m));
        }
        let volume_scale = whole.x.abs().max(f64::MIN_POSITIVE);
        let x_scale = a
            .abs()
            .max(b.abs())
            .max((b - a).abs())
            .max(f64::MIN_POSITIVE);
        let scale = DVec4::new(
            volume_scale,
            volume_scale * x_scale,
            volume_scale * max_y_extent.max(f64::MIN_POSITIVE),
            volume_scale * max_z_extent.max(f64::MIN_POSITIVE),
        );
        adaptive_simpson(&evaluate, a, b, fa, fm, fb, whole, scale * 1e-11, 16)
    }

    fn needs_split(&self, a: f64, b: f64) -> Result<bool, FuselageError> {
        if b - a <= 1e-12 {
            return Ok(false);
        }
        if b - a > self.options.max_zone_length_m {
            return Ok(true);
        }
        let (s0, s1) = self.section_pair(a, b);
        // Ensure a leaf never hides a local reversal of dA/dx. Reversed
        // contributions need separate panels because Munk normal-force
        // direction follows the sign of the area derivative.
        let x = [
            a,
            0.25 * (3.0 * a + b),
            0.5 * (a + b),
            0.25 * (a + 3.0 * b),
            b,
        ];
        let areas = x.map(|sample_x| self.body.section_at(sample_x).area_m2());
        let mut direction = 0_i8;
        for pair in areas.windows(2) {
            let difference = pair[1] - pair[0];
            let epsilon = 1e-12 * pair[0].abs().max(pair[1].abs()).max(f64::MIN_POSITIVE);
            let current = if difference > epsilon {
                1
            } else if difference < -epsilon {
                -1
            } else {
                0
            };
            if current != 0 {
                if direction != 0 && direction != current {
                    return Ok(true);
                }
                direction = current;
            }
        }
        // Curvature split: refine true nonlinear area schedules based on
        // quarter, midpoint and three-quarter samples, not one midpoint
        // that can miss a shallow internal maximum.
        if b - a < self.options.max_zone_length_m / 8.0 {
            return Ok(false);
        }
        for (index, fraction) in [0.25, 0.5, 0.75].into_iter().enumerate() {
            let actual = areas[index + 1];
            let linear = s0.area_m2() + (s1.area_m2() - s0.area_m2()) * fraction;
            if (actual - linear).abs() / linear.abs().max(f64::MIN_POSITIVE)
                > self.options.max_area_change_frac
            {
                return Ok(true);
            }
        }
        Ok(false)
    }

    fn subdivide(
        &self,
        a: f64,
        b: f64,
        depth: u32,
        leaves: &mut Vec<AxialLeaf>,
    ) -> Result<(), FuselageError> {
        if !self.needs_split(a, b)? {
            leaves.push(AxialLeaf { a, b });
            return Ok(());
        }
        if depth >= self.options.max_depth {
            return Err(FuselageError::InvalidOptions(format!(
                "max_depth {} reached before interval [{a}, {b}] met subdivision tolerances",
                self.options.max_depth
            )));
        }
        let mid = 0.5 * (a + b);
        self.subdivide(a, mid, depth + 1, leaves)?;
        self.subdivide(mid, b, depth + 1, leaves)?;
        Ok(())
    }

    fn compile(&self) -> Result<CompiledBody, FuselageError> {
        let x_first = self.body.stations[0].x_m;
        let x_last = self.body.stations.last().unwrap().x_m;
        let mut leaves = Vec::new();
        let mut splits = vec![x_first, x_last];
        for station in &self.body.stations {
            splits.push(station.x_m);
        }
        for control in &self.body.controls {
            splits.push(control.x0_m);
            splits.push(control.x1_m);
        }
        splits.sort_by(|a, b| a.partial_cmp(b).expect("validated finite"));
        splits.dedup_by(|a, b| (*a - *b).abs() <= 1e-12);
        for pair in splits.windows(2) {
            self.subdivide(pair[0], pair[1], 0, &mut leaves)?;
        }
        if leaves.is_empty() {
            return Err(FuselageError::PanelRejected(
                "compilation produced no axial zones".into(),
            ));
        }
        let leaf_data = leaves
            .iter()
            .map(|leaf| self.leaf_quantities(leaf.a, leaf.b))
            .collect::<Result<Vec<_>, _>>()?;

        let mut compiled = CompiledBody {
            panels: Vec::new(),
            controls: Vec::new(),
            structure: None,
            tanks: Vec::new(),
            interior: Vec::new(),
            ports: Vec::new(),
            summary: CompiledBodySummary::default(),
        };

        // Geometry accumulators (outer mold, body-local frame).
        let mut wet_m2 = 0.0;
        let mut volume_m3 = 0.0;
        let mut volume_error_m3 = 0.0;
        let mut moment = DVec3::ZERO;
        let mut lateral_area_error_est = 0.0;
        let mut max_section_m2: f64 = 0.0;
        let mut control_panels = vec![Vec::new(); self.body.controls.len()];
        for (leaf, q) in leaves.iter().zip(&leaf_data) {
            max_section_m2 = max_section_m2.max(q.area0_m2).max(q.area1_m2);
            for sample in 1..16 {
                let x = leaf.a + (leaf.b - leaf.a) * sample as f64 / 16.0;
                max_section_m2 = max_section_m2.max(self.body.section_at(x).area_m2());
            }
            wet_m2 += q.lateral_m2;
            lateral_area_error_est += q.lateral_area_error_m2;
            volume_m3 += q.volume_m3;
            volume_error_m3 += q.volume_error_m3;
            moment += DVec3::new(q.centroid_x_m, q.centroid_y_m, q.centroid_z_m) * q.volume_m3;
            let zone_panel_start = compiled.panels.len();
            self.emit_zone_panels(&mut compiled, &q)?;
            for (control_index, control) in self.body.controls.iter().enumerate() {
                if leaf.a >= control.x0_m - 1e-12 && leaf.b <= control.x1_m + 1e-12 {
                    // `emit_zone_panels` emits pitch then yaw for each zone.
                    let plane_offset = match control.plane {
                        BodyControlPlane::Pitch => 0,
                        BodyControlPlane::Yaw => 1,
                    };
                    control_panels[control_index].push(zone_panel_start + plane_offset);
                }
            }
        }
        for (control, panel_indices) in self.body.controls.iter().zip(control_panels) {
            if panel_indices.is_empty() {
                return Err(FuselageError::PanelRejected(format!(
                    "body control '{}' did not resolve to any aerodynamic panels",
                    control.name
                )));
            }
            compiled.controls.push(
                ControlSurfaceDefinition::new(
                    control.name.clone(),
                    panel_indices,
                    control.minimum_deflection_rad,
                    control.maximum_deflection_rad,
                )
                .map_err(|error| FuselageError::PanelRejected(error.to_string()))?,
            );
        }

        // Flat-disc end caps for open ends (documented closure rule).
        let mut cap_surfaces = Vec::with_capacity(2);
        for station in [self.body.stations[0], *self.body.stations.last().unwrap()] {
            let cap = self.end_cap_surface(station);
            if cap.area_m2 > 1e-9 {
                wet_m2 += cap.area_m2;
                lateral_area_error_est += cap.estimated_area_error_m2;
                cap_surfaces.push(cap);
            }
        }

        // Structure + fuel + interior over the same leaves.
        let layout = self.body.structure.clone();
        let mut hull_mass_kg = 0.0;
        let mut hull_moment = DVec3::ZERO;
        let mut hull_inertia = DMat3::ZERO;
        let mut skin_mass_kg = 0.0;
        let mut frame_mass_kg = 0.0;
        if let Some(layout) = &layout {
            let gauge_m = layout.skin_gauge_mm / 1000.0;
            let rho = layout.skin_material.density_kg_m3;
            let areal_density = gauge_m * rho;
            for q in &leaf_data {
                let mass = q.lateral_m2 * areal_density;
                let first_moment = q.lateral_first_moment_m3 * areal_density;
                skin_mass_kg += mass;
                hull_mass_kg += mass;
                hull_moment += first_moment;
                hull_inertia +=
                    inertia_from_second_moment(q.lateral_second_moment_m4 * areal_density);
            }
            // End discs use their in-plane area moments and exact outline
            // centroid, rather than point-mass approximations.
            for cap in &cap_surfaces {
                let mass = cap.area_m2 * areal_density;
                skin_mass_kg += mass;
                hull_mass_kg += mass;
                hull_moment += cap.first_moment_m3 * areal_density;
                hull_inertia += inertia_from_second_moment(cap.second_moment_m4 * areal_density);
            }
            // Ring frames use the authored maximum pitch, independently
            // of how densely geometry stations happen to be authored.
            let frame_gauge_m = layout.frame_gauge_mm / 1000.0;
            let frame_width_m = layout.frame_width_mm / 1000.0;
            let length_m = x_last - x_first;
            let frame_count = (length_m / layout.frame_spacing_m).ceil().max(1.0);
            if frame_count > 100_000.0 {
                return Err(FuselageError::InvalidOptions(
                    "frame spacing would generate more than 100000 ring frames".into(),
                ));
            }
            let frame_count = frame_count as usize;
            let actual_spacing_m = length_m / frame_count as f64;
            if frame_width_m > actual_spacing_m {
                return Err(FuselageError::InvalidBody(format!(
                    "frame width {:.3} m exceeds generated frame pitch {:.3} m",
                    frame_width_m, actual_spacing_m
                )));
            }
            for frame_index in 0..frame_count {
                let x = x_first + (frame_index as f64 + 0.5) * actual_spacing_m;
                let station = self.body.section_at(x);
                let (mass, first_moment, inertia) =
                    self.frame_mass_properties(station, frame_width_m, frame_gauge_m * rho);
                frame_mass_kg += mass;
                hull_mass_kg += mass;
                hull_moment += first_moment;
                hull_inertia += inertia;
            }
        }

        // Interior regions: usable inner-mold volumes clipped from leaves.
        let wall_m = layout
            .as_ref()
            .map(|layout| layout.wall_inset_mm / 1000.0)
            .unwrap_or(0.0);
        for region in &self.body.regions {
            let (volume, volume_error, centroid) = self.region_volume(region, &leaves, wall_m)?;
            let payload = match region.kind {
                RegionKind::Cargo { payload_mass_kg } => payload_mass_kg,
                _ => 0.0,
            };
            if payload > 0.0 {
                hull_mass_kg += payload;
                hull_moment += centroid * payload;
                hull_inertia += point_inertia(payload, centroid);
            }
            if let RegionKind::Tank {
                propellant,
                fill_fraction,
            } = region.kind
            {
                let layout = layout.as_ref().ok_or_else(|| {
                    FuselageError::InvalidBody(format!(
                        "tank region '{}' needs a structural layout",
                        region.name
                    ))
                })?;
                let mean_area = volume / (region.x1_m - region.x0_m).max(1e-12);
                let equiv_d = 2.0 * (mean_area / std::f64::consts::PI).sqrt();
                let spec = TankSpec {
                    shape: TankShape::Cylinder {
                        diameter_m: equiv_d,
                        length_m: region.x1_m - region.x0_m,
                    },
                    pressure_pa: layout.tank_pressure_pa,
                    material: layout.tank_material,
                };
                let density = propellant_density(propellant)?;
                let mut tank = spec.compile(density).map_err(|error| {
                    FuselageError::InvalidBody(format!("tank region '{}': {error}", region.name))
                })?;
                // Real inner volume is the capacity (equivalent cylinder
                // sizes the shell); preserve capacity separately from the
                // authored initial fill in the installed mount.
                tank.volume_m3 = volume;
                tank.full_propellant_kg = volume * density;
                let initial_propellant_kg = tank.full_propellant_kg * fill_fraction;
                let intrinsic_inertia_body_kg_m2 = spec
                    .shape
                    .intrinsic_inertia_body_kg_m2(tank.dry_mass_kg, initial_propellant_kg)
                    .map_err(|error| {
                        FuselageError::InvalidBody(format!(
                            "tank region '{}': {error}",
                            region.name
                        ))
                    })?;
                let position = centroid + self.body.origin_body_m;
                let mount = TankMount {
                    tank,
                    position_body_m: position.to_array(),
                    intrinsic_inertia_body_kg_m2,
                    initial_propellant_kg: Some(initial_propellant_kg),
                };
                mount
                    .validate()
                    .map_err(|error| FuselageError::InvalidBody(error.to_string()))?;
                compiled.tanks.push(CompiledBodyTank {
                    region_name: region.name.clone(),
                    mount,
                    inner_volume_m3: volume,
                    inner_volume_error_m3: volume_error,
                    propellant_kg: initial_propellant_kg,
                });
            }
            compiled.interior.push(CompiledRegion {
                name: region.name.clone(),
                kind: region.kind,
                volume_m3: volume,
                volume_error_m3: volume_error,
                centroid_body_m: centroid,
                payload_mass_kg: payload,
            });
        }

        // The structure record exists whenever there is mass to own
        // (shell, frames, or manifest): a massless shell stays `None`
        // for pure-aero bodies, but payload is never silently dropped.
        if layout.is_some() || hull_mass_kg > 0.0 {
            let com = if hull_mass_kg > 0.0 {
                hull_moment / hull_mass_kg
            } else {
                DVec3::ZERO
            };
            compiled.structure = Some(self.mount_hull(CompiledHull {
                mass_kg: hull_mass_kg,
                skin_mass_kg,
                frame_mass_kg,
                center_of_mass_body_m: com,
                inertia_body_kg_m2: hull_inertia,
                wetted_area_m2: wet_m2,
            }));
        }

        // Ports resolve onto the outer mold at their clock angle.
        for port in &self.body.ports {
            compiled.ports.push(self.compile_port(port)?);
        }

        let center_of_volume = if volume_m3 > 0.0 {
            moment / volume_m3
        } else {
            DVec3::ZERO
        };
        let tail_area = self.body.stations[0].area_m2();
        let max_diameter = self
            .body
            .stations
            .iter()
            .map(|station| {
                2.0 * station
                    .half_width_m
                    .max(station.top_height_m)
                    .max(station.bottom_height_m)
            })
            .fold(0.0_f64, f64::max);
        compiled.summary = CompiledBodySummary::build(
            &self.body.name,
            x_last - x_first,
            max_diameter,
            max_section_m2,
            tail_area,
            wet_m2,
            volume_m3,
            volume_error_m3,
            center_of_volume,
            hull_mass_kg,
            compiled.tanks.iter().map(|tank| tank.inner_volume_m3).sum(),
            compiled.panels.len() / 2,
            compiled.panels.len(),
            lateral_area_error_est,
        );
        Ok(self.mount(compiled))
    }

    /// Pitch + yaw strip panels for one axial zone (Munk distribution).
    /// The signed section-area change is preserved: ogives and boat-tails
    /// contribute with opposite force directions as slender-body theory
    /// requires.
    fn emit_zone_panels(
        &self,
        compiled: &mut CompiledBody,
        q: &LeafQuantities,
    ) -> Result<(), FuselageError> {
        let cp_section = self.body.section_at(q.munk_cp_x_m);
        let cp_center = self.section_center(cp_section)? + self.body.origin_body_m;
        let delta_area = q.munk_delta_area_m2;
        // Strip chord follows the local centerline (bent/drooped bodies
        // carry camber physics); the constructor orthogonalizes lift.
        let chord_axis = DVec3::new(1.0, q.slope_y, q.slope_z);
        let two_pi = 2.0 * std::f64::consts::PI;
        let mean_height_m = 0.5 * (q.avg_top_height_m + q.avg_bottom_height_m);
        for (half_span_m, lift_axis) in [(q.avg_half_width_m, DVec3::Z), (mean_height_m, DVec3::Y)]
        {
            let area = 2.0 * half_span_m * q.length_m;
            if area <= 0.0 {
                return Err(FuselageError::PanelRejected(
                    "body zone has non-positive projected area".into(),
                ));
            }
            let span = 2.0 * half_span_m;
            let aspect = span.powi(2) / area;
            // Geometry-derived interference: the zone slope on its own
            // projected area equals the Munk strip value 2*dA/S, so the
            // body total recovers 2*S_base/S_ref for pointed noses with
            // zero tuning per vehicle. Unit Diederich at 2-D slope 2π
            // keeps the factor Mach-independent at compile time.
            let unit = diederich_lift_slope(two_pi, aspect, 1.0);
            if !unit.is_finite() || unit <= 0.0 {
                return Err(FuselageError::PanelRejected(
                    "body zone unit slope is non-finite".into(),
                ));
            }
            let interference = (2.0 * delta_area.abs() / area) / unit;
            let lift_sign = if delta_area < 0.0 { -1.0 } else { 1.0 };
            let panel = AeroPanel::new(cp_center, chord_axis, lift_axis, area, q.length_m)
                .and_then(|panel| panel.with_planform(span, aspect, 0.0, interference))
                .and_then(|panel| panel.with_lift_sign(lift_sign))
                .and_then(|panel| panel.with_center_of_pressure(cp_center))
                .and_then(|panel| panel.with_thickness_ratio(self.fineness_thickness))
                // Lateral response arrives through the lift path of the
                // orthogonal strips; the shared sideslip convention would
                // otherwise double-count pitch-plane crossflow as sideslip
                // on yaw-normal strips (and vice versa).
                .and_then(|panel| panel.with_side_force_scale(0.0))
                .map_err(|error| FuselageError::PanelRejected(error.to_string()))?;
            compiled.panels.push(panel);
        }
        Ok(())
    }

    /// Usable inner-mold volume of a region by clipping leaves.
    fn region_volume(
        &self,
        region: &InteriorRegion,
        leaves: &[AxialLeaf],
        wall_m: f64,
    ) -> Result<(f64, f64, DVec3), FuselageError> {
        let layout_wall = wall_m;
        let mut volume = 0.0;
        let mut volume_error = 0.0;
        let mut moment = DVec3::ZERO;
        for leaf in leaves {
            let a = leaf.a.max(region.x0_m);
            let b = leaf.b.min(region.x1_m);
            if b - a <= 1e-12 {
                continue;
            }
            let integrated = self.integrate_section(a, b, layout_wall)?;
            volume += integrated.moments.x;
            volume_error += integrated.estimated_error.x;
            moment += DVec3::new(
                integrated.moments.y,
                integrated.moments.z,
                integrated.moments.w,
            );
        }
        if volume <= 0.0 {
            return Err(FuselageError::InvalidInterior(format!(
                "region '{}' has no usable volume (wall exceeds section?)",
                region.name
            )));
        }
        Ok((volume, volume_error, moment / volume))
    }

    fn compile_port(&self, port: &crate::BodyPort) -> Result<BodyPortCompiled, FuselageError> {
        let section = self.body.section_at(port.x_m);
        let (dy, dz) = crate::outline_point(
            section.half_width_m,
            section.top_height_m,
            section.bottom_height_m,
            section.top_exponent,
            section.bottom_exponent,
            port.clock_rad,
        );
        if !dy.is_finite() || !dz.is_finite() {
            return Err(FuselageError::InvalidInterior(format!(
                "port '{}' outline point is non-finite",
                port.name
            )));
        }
        // The port sits on the mold by construction (outline point).
        let local = DVec3::new(port.x_m, section.offset_y_m + dy, section.offset_z_m + dz);
        // Docking faces the nearer end (nose berthing forward, aft ports
        // face back); engines face aft; attachments face radial-out.
        let x_first = self.body.stations[0].x_m;
        let x_last = self.body.stations.last().unwrap().x_m;
        let axis = match port.kind {
            crate::PortKind::Docking => {
                if port.x_m - x_first >= x_last - port.x_m {
                    DVec3::X
                } else {
                    DVec3::NEG_X
                }
            }
            crate::PortKind::EngineMount => DVec3::NEG_X,
            crate::PortKind::Attachment => {
                DVec3::new(0.0, port.clock_rad.cos(), port.clock_rad.sin())
            }
        };
        // Radial ports must clear the mold: the interface diameter has
        // to fit inside the local section diagonal.
        let fit = 2.0
            * section
                .half_width_m
                .min(section.top_height_m)
                .min(section.bottom_height_m);
        if port.diameter_m >= fit {
            return Err(FuselageError::InvalidInterior(format!(
                "port '{}' diameter {:.3} exceeds local section fit {:.3}",
                port.name, port.diameter_m, fit
            )));
        }
        Ok(BodyPortCompiled {
            name: port.name.clone(),
            kind: port.kind,
            position_body_m: local,
            axis_body_m: axis,
            diameter_m: port.diameter_m,
        })
    }

    /// Shift a local-frame hull into the mounted vehicle frame.
    fn mount_hull(&self, mut hull: CompiledHull) -> CompiledHull {
        let origin = self.body.origin_body_m;
        let local_com = hull.center_of_mass_body_m;
        // The stored tensor is about the vehicle-local origin. Recover its
        // centroidal tensor, then shift that tensor to the mounted origin;
        // this preserves COM * mount-offset cross terms.
        hull.inertia_body_kg_m2 -= point_inertia(hull.mass_kg, local_com);
        hull.center_of_mass_body_m += origin;
        hull.inertia_body_kg_m2 += point_inertia(hull.mass_kg, hull.center_of_mass_body_m);
        hull
    }

    /// Shift a compiled body into the mounted vehicle frame.
    ///
    /// Panels and tank mounts already carry the origin from emission;
    /// ports, interior centroids, and the summary volume center shift
    /// here exactly once.
    fn mount(&self, mut compiled: CompiledBody) -> CompiledBody {
        let origin = self.body.origin_body_m;
        for port in &mut compiled.ports {
            port.position_body_m += origin;
        }
        for region in &mut compiled.interior {
            region.centroid_body_m += origin;
        }
        compiled.summary.center_of_volume_m += origin;
        compiled
    }
}

fn simpson(a: f64, b: f64, fa: DVec4, fm: DVec4, fb: DVec4) -> DVec4 {
    (fa + 4.0 * fm + fb) * ((b - a) / 6.0)
}

fn outer_product(a: DVec3, b: DVec3) -> DMat3 {
    DMat3::from_cols(a * b.x, a * b.y, a * b.z)
}

fn inertia_from_second_moment(second_moment: DMat3) -> DMat3 {
    let trace = second_moment.x_axis.x + second_moment.y_axis.y + second_moment.z_axis.z;
    DMat3::IDENTITY * trace - second_moment
}

#[allow(clippy::too_many_arguments)]
fn adaptive_simpson<F>(
    evaluate: &F,
    a: f64,
    b: f64,
    fa: DVec4,
    fm: DVec4,
    fb: DVec4,
    whole: DVec4,
    tolerance: DVec4,
    depth_remaining: u32,
) -> Result<IntegratedSection, FuselageError>
where
    F: Fn(f64) -> Result<DVec4, FuselageError>,
{
    let mid = 0.5 * (a + b);
    let left_mid = 0.5 * (a + mid);
    let right_mid = 0.5 * (mid + b);
    let flm = evaluate(left_mid)?;
    let frm = evaluate(right_mid)?;
    let left = simpson(a, mid, fa, flm, fm);
    let right = simpson(mid, b, fm, frm, fb);
    let split = left + right;
    let difference = split - whole;
    let error = difference.abs() / 15.0;
    let converged = error.x <= tolerance.x
        && error.y <= tolerance.y
        && error.z <= tolerance.z
        && error.w <= tolerance.w;
    if converged {
        return Ok(IntegratedSection {
            moments: split + difference / 15.0,
            estimated_error: error,
        });
    }
    if depth_remaining == 0 {
        return Err(FuselageError::InvalidBody(
            "adaptive loft integration did not meet its error tolerance".into(),
        ));
    }
    let half_tolerance = tolerance * 0.5;
    let left_integral = adaptive_simpson(
        evaluate,
        a,
        mid,
        fa,
        flm,
        fm,
        left,
        half_tolerance,
        depth_remaining - 1,
    )?;
    let right_integral = adaptive_simpson(
        evaluate,
        mid,
        b,
        fm,
        frm,
        fb,
        right,
        half_tolerance,
        depth_remaining - 1,
    )?;
    Ok(IntegratedSection {
        moments: left_integral.moments + right_integral.moments,
        estimated_error: left_integral.estimated_error + right_integral.estimated_error,
    })
}

/// Inner-mold section after wall inset (uniform shrink of semi-axes).
fn shrink_section(section: BodyStation, wall_m: f64) -> Result<BodyStation, FuselageError> {
    let inner = BodyStation::new(
        section.x_m,
        section.half_width_m - wall_m,
        section.top_height_m - wall_m,
        section.bottom_height_m - wall_m,
        section.top_exponent,
        section.bottom_exponent,
        section.offset_y_m,
        section.offset_z_m,
    )
    .map_err(|_| {
        FuselageError::InvalidInterior(format!(
            "wall inset {:.1} mm exceeds section at x={}",
            wall_m * 1000.0,
            section.x_m
        ))
    })?;
    Ok(inner)
}

/// Propellant bulk density from the sim-core thermo tables (the same
/// source the tank pipeline uses, never a fuselage-side constant).
fn propellant_density(propellant: Propellant) -> Result<f64, FuselageError> {
    let density = propellant.thermo().bulk_density_kg_m3;
    if !density.is_finite() || density <= 0.0 {
        return Err(FuselageError::InvalidInterior(
            "tank propellant needs a positive bulk density (cold-gas/solid have none)".into(),
        ));
    }
    Ok(density)
}
