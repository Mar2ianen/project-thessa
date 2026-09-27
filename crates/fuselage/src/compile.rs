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

use glam::{DMat3, DQuat, DVec3, DVec4};
use serde::{Deserialize, Serialize};
use thessa_sim_core::{
    AeroPanel, ControlHinge, ControlSurfaceDefinition, Propellant, TankMount, TankShape, TankSpec,
    diederich_lift_slope,
};

use crate::summary::CompiledBodySummary;
use crate::{
    AttachKind, AttachSite, BodyControlPlane, BodyStation, CabinAtmosphere, CabinSeatRole,
    CompiledHull, DoorSide, ExitType, FuselageError, InteriorRegion, MonumentKind, ProceduralBody,
    RegionKind, SeatClass, SeatStyle, SuitType, TankShell, outline_point, point_inertia,
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

/// Which propellant load a compiled tank carries.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "kebab-case")]
pub enum TankComponent {
    /// Single mixed/bulk tank (legacy path).
    Bulk,
    /// Oxidizer side of a split bipropellant region.
    Oxidizer,
    /// Fuel side of a split bipropellant region.
    Fuel,
    /// Standalone pure-fluid tank.
    Stored,
}

/// Tank contents: a chamber pair (bulk mixed load) or a pure stored fluid.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "kebab-case")]
pub enum TankContents {
    Pair(Propellant),
    Fluid(crate::StoredFluid),
}

/// Dry-air gas constant in J/kg/K for cabin air inventory.
const R_DRY_AIR_J_KG_K: f64 = 287.05;
/// Molar masses in g/mol for the oxygen mass split.
const MOLAR_MASS_AIR_G_MOL: f64 = 28.97;
const MOLAR_MASS_O2_G_MOL: f64 = 32.0;
/// Shared safety factor for tank and cabin pressure shells.
const PRESSURE_SAFETY_FACTOR: f64 = 1.5;
/// Loft samples for the pressurized-region radius screening.
const PRESSURE_SHELL_SAMPLES: usize = 16;

/// One tank region compiled into the feed pipeline.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct CompiledBodyTank {
    /// Region name from authoring (`{name}-ox` / `{name}-fuel` for splits).
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
    /// What the tank stores (pair or pure fluid).
    pub contents: TankContents,
    /// Bulk vs split-tank side vs standalone stored fluid.
    #[serde(default = "default_tank_component")]
    pub component: TankComponent,
}

fn default_tank_component() -> TankComponent {
    TankComponent::Bulk
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
    /// Declared cargo/crew mass (kg; cargo manifest, seats + occupants).
    pub payload_mass_kg: f64,
    /// Installed seat places (crew regions only).
    #[serde(default)]
    pub seats: u32,
    /// Upright seats vs reclined couches.
    #[serde(default)]
    pub seat_style: crate::SeatStyle,
    /// Seat anchors in body-local metres, forward-facing on the section
    /// centerline (crew regions only; empty otherwise).
    #[serde(default)]
    pub seat_positions_body_m: Vec<DVec3>,
    /// Cabin air mass in kg at the authored atmosphere (0 unpressurized).
    #[serde(default)]
    pub air_mass_kg: f64,
    /// Oxygen mass within the cabin air in kg (0 unpressurized).
    #[serde(default)]
    pub o2_mass_kg: f64,
    /// Authored atmosphere (pressure/temp/O2 setpoints for runtime cabins).
    #[serde(default)]
    pub atmosphere: Option<crate::CabinAtmosphere>,
    /// Autopilot core hosted here, if any (runtime control authority).
    #[serde(default)]
    pub control_core: Option<thessa_sim_core::AutopilotTier>,
    /// Advanced cabin seats with per-place class, role, suit and mass data.
    #[serde(default)]
    pub cabin_seats: Vec<CompiledCabinSeat>,
    /// Fitted mass-only volumes compiled from deck monuments.
    #[serde(default)]
    pub cabin_monuments: Vec<CompiledCabinMonument>,
    /// Exit records retained for hangar diagnostics and future evacuation.
    #[serde(default)]
    pub cabin_doors: Vec<CompiledCabinDoor>,
}

/// One compiled place after loft fitting, including its baked mass manifest.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct CompiledCabinSeat {
    pub name: String,
    pub position_body_m: DVec3,
    pub class: SeatClass,
    pub role: CabinSeatRole,
    pub seat_style: SeatStyle,
    pub occupied: bool,
    pub suited: bool,
    pub suit_type: SuitType,
    pub seat_mass_kg: f64,
    pub occupant_mass_kg: f64,
    pub carry_on_mass_kg: f64,
    pub suit_mass_kg: f64,
}

impl CompiledCabinSeat {
    pub fn mass_kg(&self) -> f64 {
        self.seat_mass_kg
            + if self.occupied {
                self.occupant_mass_kg + self.carry_on_mass_kg + self.suit_mass_kg
            } else {
                0.0
            }
    }
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct CompiledCabinMonument {
    pub name: String,
    pub kind: MonumentKind,
    pub position_body_m: DVec3,
    pub mass_kg: f64,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct CompiledCabinDoor {
    pub name: String,
    pub pair_id: String,
    pub position_body_m: DVec3,
    pub side: DoorSide,
    pub rating: ExitType,
    pub opening_width_m: f64,
    pub opening_height_m: f64,
}

struct CompiledCabinLayout {
    seats: Vec<CompiledCabinSeat>,
    monuments: Vec<CompiledCabinMonument>,
    doors: Vec<CompiledCabinDoor>,
    seat_positions_body_m: Vec<DVec3>,
    seat_count: u32,
    payload_mass_kg: f64,
    seat_style: SeatStyle,
}

/// One detachable heat shield with compiled mass data.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct CompiledHeatShield {
    pub name: String,
    /// Mount position in body-local metres (end-section center).
    pub position_body_m: DVec3,
    /// Shield diameter in metres (from the end section).
    pub diameter_m: f64,
    /// Shield mass in kg.
    pub mass_kg: f64,
    /// Ablative shell material.
    pub material: thessa_sim_core::ChamberMaterial,
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
    /// Detachable heat shields with mass data.
    #[serde(default)]
    pub heat_shields: Vec<CompiledHeatShield>,
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
            heat_shields: Vec::new(),
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
            self.emit_zone_panels(&mut compiled, q)?;
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
            // Body stations run tail-to-nose with +X forward, so the
            // control's forward/leading axial boundary is its larger x.
            let section = self.body.section_at(control.x1_m);
            let hinge_point = self.section_center(section)? + self.body.origin_body_m;
            // Positive pitch/yaw commands preserve the existing body-control
            // sign convention while now rotating the strip geometry itself.
            let hinge_axis = match control.plane {
                BodyControlPlane::Pitch => -DVec3::Y,
                BodyControlPlane::Yaw => DVec3::Z,
            };
            let hinge = ControlHinge::new(hinge_point, hinge_axis)
                .map_err(|error| FuselageError::PanelRejected(error.to_string()))?;
            let mut definition = ControlSurfaceDefinition::new(
                control.name.clone(),
                panel_indices,
                control.minimum_deflection_rad,
                control.maximum_deflection_rad,
            )
            .map_err(|error| FuselageError::PanelRejected(error.to_string()))?
            .with_hinge(hinge);
            if let Some(actuator) = control.actuator {
                definition = definition.with_actuator(actuator);
            }
            compiled.controls.push(definition);
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
            let cabin_layout = if let Some(layout) = &region.cabin_layout {
                Some(self.compile_cabin_layout(region, layout, wall_m)?)
            } else if let Some(preset) = region.cabin_layout_preset {
                let layout = preset.build_for_region(region.x0_m, region.x1_m)?;
                Some(self.compile_cabin_layout(region, &layout, wall_m)?)
            } else {
                None
            };
            let (mut payload, mut seats, mut seat_style, mut seat_anchors) = match region.kind {
                RegionKind::Cargo { payload_mass_kg } => {
                    (payload_mass_kg, 0, crate::SeatStyle::Upright, Vec::new())
                }
                RegionKind::Crew {
                    seats,
                    seat_mass_kg_each,
                    occupant_mass_kg_each,
                    seat_pitch_m,
                    abreast,
                    seat_style,
                    suited,
                    suit_mass_kg_each,
                    ..
                } => (
                    seats as f64
                        * (seat_mass_kg_each
                            + occupant_mass_kg_each
                            + if suited { suit_mass_kg_each } else { 0.0 }),
                    seats,
                    seat_style,
                    self.seat_anchors(
                        region.x0_m,
                        region.x1_m,
                        seats,
                        seat_pitch_m,
                        abreast.unwrap_or(1),
                    )?,
                ),
                _ => (0.0, 0, crate::SeatStyle::Upright, Vec::new()),
            };
            if let Some(cabin) = &cabin_layout {
                payload = cabin.payload_mass_kg;
                seats = cabin.seat_count;
                seat_style = cabin.seat_style;
                seat_anchors = cabin.seat_positions_body_m.clone();
                for seat in &cabin.seats {
                    let mass = seat.mass_kg();
                    hull_mass_kg += mass;
                    hull_moment += seat.position_body_m * mass;
                    hull_inertia += point_inertia(mass, seat.position_body_m);
                }
                for monument in &cabin.monuments {
                    hull_mass_kg += monument.mass_kg;
                    hull_moment += monument.position_body_m * monument.mass_kg;
                    hull_inertia += point_inertia(monument.mass_kg, monument.position_body_m);
                }
            } else if payload > 0.0 {
                hull_mass_kg += payload;
                hull_moment += centroid * payload;
                hull_inertia += point_inertia(payload, centroid);
            }
            if let RegionKind::Tank {
                propellant,
                fill_fraction,
                pressure_pa,
                material,
                shell,
            } = region.kind
            {
                let layout = layout.as_ref().ok_or_else(|| {
                    FuselageError::InvalidBody(format!(
                        "tank region '{}' needs a structural layout",
                        region.name
                    ))
                })?;
                let spec = Self::tank_spec(
                    shell,
                    volume,
                    region.x1_m - region.x0_m,
                    pressure_pa.unwrap_or(layout.tank_pressure_pa),
                    material.unwrap_or(layout.tank_material),
                );
                let compiled_tank = self.compile_single_tank(
                    &region.name,
                    TankContents::Pair(propellant),
                    TankComponent::Bulk,
                    volume,
                    volume_error,
                    centroid,
                    fill_fraction,
                    &spec,
                )?;
                compiled.tanks.push(compiled_tank);
            }
            if let RegionKind::FluidTank {
                fluid,
                fill_fraction,
                pressure_pa,
                material,
                shell,
            } = region.kind
            {
                let layout = layout.as_ref().ok_or_else(|| {
                    FuselageError::InvalidBody(format!(
                        "tank region '{}' needs a structural layout",
                        region.name
                    ))
                })?;
                let spec = Self::tank_spec(
                    shell,
                    volume,
                    region.x1_m - region.x0_m,
                    pressure_pa.unwrap_or(layout.tank_pressure_pa),
                    material.unwrap_or(layout.tank_material),
                );
                let compiled_tank = self.compile_split_tank(
                    &region.name,
                    TankContents::Fluid(fluid),
                    TankComponent::Stored,
                    volume,
                    volume_error,
                    centroid,
                    fluid.density_kg_m3(),
                    fill_fraction,
                    &spec,
                )?;
                compiled.tanks.push(compiled_tank);
            }
            if let RegionKind::Bipropellant {
                propellant,
                fill_fraction,
                mixture_ratio,
                pressure_pa,
                oxidizer_pressure_pa,
                fuel_pressure_pa,
                material,
                oxidizer_material,
                fuel_material,
                shell,
            } = region.kind
            {
                let layout = layout.as_ref().ok_or_else(|| {
                    FuselageError::InvalidBody(format!(
                        "tank region '{}' needs a structural layout",
                        region.name
                    ))
                })?;
                let ratio = mixture_ratio.unwrap_or_else(|| {
                    propellant
                        .reference_mixture_ratio()
                        .expect("validated bipropellant pair")
                });
                let (rho_ox, rho_fuel) = propellant.split_densities().ok_or_else(|| {
                    FuselageError::InvalidInterior(format!(
                        "region '{}' needs a bipropellant pair for a split tank",
                        region.name
                    ))
                })?;
                // Mass ratio MR = m_ox / m_f; volume ratio follows densities.
                let volume_ratio = ratio * rho_fuel / rho_ox;
                let ox_volume = volume * volume_ratio / (1.0 + volume_ratio);
                let split_x = self.find_split_x(region.x0_m, region.x1_m, ox_volume, wall_m)?;
                let (ox_vol, ox_err, ox_centroid) =
                    self.range_volume(region.x0_m, split_x, wall_m)?;
                let (fuel_vol, fuel_err, fuel_centroid) =
                    self.range_volume(split_x, region.x1_m, wall_m)?;
                let base_pressure = pressure_pa.unwrap_or(layout.tank_pressure_pa);
                let base_material = material.unwrap_or(layout.tank_material);
                let ox_spec = Self::tank_spec(
                    shell,
                    ox_vol,
                    (split_x - region.x0_m).max(1e-12),
                    oxidizer_pressure_pa
                        .or(pressure_pa)
                        .unwrap_or(base_pressure),
                    oxidizer_material.or(material).unwrap_or(base_material),
                );
                let fuel_spec = Self::tank_spec(
                    shell,
                    fuel_vol,
                    (region.x1_m - split_x).max(1e-12),
                    fuel_pressure_pa.or(pressure_pa).unwrap_or(base_pressure),
                    fuel_material.or(material).unwrap_or(base_material),
                );
                // Oxidizer aft, fuel forward: denser load near the tail.
                let ox_tank = self.compile_split_tank(
                    &format!("{}-ox", region.name),
                    TankContents::Pair(propellant),
                    TankComponent::Oxidizer,
                    ox_vol,
                    ox_err,
                    ox_centroid,
                    rho_ox,
                    fill_fraction,
                    &ox_spec,
                )?;
                let fuel_tank = self.compile_split_tank(
                    &format!("{}-fuel", region.name),
                    TankContents::Pair(propellant),
                    TankComponent::Fuel,
                    fuel_vol,
                    fuel_err,
                    fuel_centroid,
                    rho_fuel,
                    fill_fraction,
                    &fuel_spec,
                )?;
                compiled.tanks.push(ox_tank);
                compiled.tanks.push(fuel_tank);
            }
            // Pressurized atmosphere (first ECLSS brick): ideal-gas air
            // inventory rides the hull at the region centroid, and the
            // skin must hold the full differential against vacuum.
            let (air_mass_kg, o2_mass_kg) = match region.atmosphere {
                Some(atmosphere) => {
                    let considering = layout.as_ref().ok_or_else(|| {
                        FuselageError::InvalidBody(format!(
                            "pressurized region '{}' needs a structural layout",
                            region.name
                        ))
                    })?;
                    self.check_pressure_shell(
                        &region.name,
                        region.x0_m,
                        region.x1_m,
                        atmosphere,
                        considering,
                    )?;
                    let density_kg_m3 =
                        atmosphere.pressure_kpa * 1000.0 / (R_DRY_AIR_J_KG_K * atmosphere.temp_k);
                    let air = density_kg_m3 * volume;
                    let o2 =
                        air * atmosphere.o2_fraction * MOLAR_MASS_O2_G_MOL / MOLAR_MASS_AIR_G_MOL;
                    hull_mass_kg += air;
                    hull_moment += centroid * air;
                    hull_inertia += point_inertia(air, centroid);
                    (air, o2)
                }
                None => (0.0, 0.0),
            };
            compiled.interior.push(CompiledRegion {
                name: region.name.clone(),
                kind: region.kind,
                volume_m3: volume,
                volume_error_m3: volume_error,
                centroid_body_m: centroid,
                payload_mass_kg: payload,
                seats,
                seat_style,
                seat_positions_body_m: seat_anchors,
                air_mass_kg,
                o2_mass_kg,
                atmosphere: region.atmosphere,
                control_core: region.control_core,
                cabin_seats: cabin_layout
                    .as_ref()
                    .map(|layout| layout.seats.clone())
                    .unwrap_or_default(),
                cabin_monuments: cabin_layout
                    .as_ref()
                    .map(|layout| layout.monuments.clone())
                    .unwrap_or_default(),
                cabin_doors: cabin_layout.map(|layout| layout.doors).unwrap_or_default(),
            });
        }

        // Detachable heat shields: disc mass from the end-section area,
        // thin-disc intrinsic inertia plus the mount offset term.
        for shield in &self.body.heat_shields {
            let end_station = match shield.end {
                crate::BodyEnd::Aft => self.body.stations[0],
                crate::BodyEnd::Forward => *self.body.stations.last().unwrap(),
            };
            let area = end_station.area_m2();
            let radius = (area / std::f64::consts::PI).sqrt();
            let mass = area * (shield.thickness_mm / 1000.0) * shield.material.density_kg_m3;
            let center = self.section_center(end_station)?;
            let intrinsic = DMat3::from_diagonal(DVec3::new(
                0.5 * mass * radius.powi(2),
                0.25 * mass * radius.powi(2),
                0.25 * mass * radius.powi(2),
            ));
            hull_mass_kg += mass;
            hull_moment += center * mass;
            hull_inertia += intrinsic + point_inertia(mass, center);
            compiled.heat_shields.push(CompiledHeatShield {
                name: shield.name.clone(),
                position_body_m: center,
                diameter_m: 2.0 * radius,
                mass_kg: mass,
                material: shield.material,
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

    fn compile_cabin_layout(
        &self,
        region: &InteriorRegion,
        layout: &crate::CabinLayout,
        wall_m: f64,
    ) -> Result<CompiledCabinLayout, FuselageError> {
        let mut seats = Vec::new();
        let mut monuments = Vec::new();
        let mut doors = Vec::new();
        let mut seat_positions_body_m = Vec::new();
        let mut payload_mass_kg = 0.0;
        let mut first_style = None;

        for deck in &layout.decks {
            for block in &deck.blocks {
                first_style.get_or_insert(block.seat_style);
                let suit_overrides: std::collections::HashMap<_, _> = block
                    .suit_overrides
                    .iter()
                    .map(|suit_override| (suit_override.seat_index, suit_override))
                    .collect();
                let seat_width_m = block.seat_width_m();
                let seat_count_per_row: u32 = block.columns.iter().sum();
                let seat_width_total_m = seat_count_per_row as f64 * seat_width_m;
                let aisle_width_total_m: f64 = block.aisle_widths_m.iter().sum();
                let required_width_m =
                    seat_width_total_m + aisle_width_total_m + block.wall_clearance_m;

                for row in 0..block.rows {
                    let x_m = block.x0_m + (row as f64 + 0.5) * block.pitch_m;
                    let section = self.body.section_at(x_m);
                    let top_inner_z_m = section.offset_z_m + section.top_height_m - wall_m;
                    let bottom_inner_z_m = section.offset_z_m - section.bottom_height_m + wall_m;
                    if deck.floor_z_m < bottom_inner_z_m - 1e-9
                        || deck.floor_z_m + deck.min_headroom_m > top_inner_z_m + 1e-9
                    {
                        return Err(FuselageError::InvalidInterior(format!(
                            "deck '{}' at row x={x_m:.2} m has insufficient headroom or its floor lies outside the loft",
                            deck.name
                        )));
                    }
                    let relative_z_m = deck.floor_z_m - section.offset_z_m;
                    let (height_m, exponent) = if relative_z_m >= 0.0 {
                        (section.top_height_m, section.top_exponent)
                    } else {
                        (section.bottom_height_m, section.bottom_exponent)
                    };
                    let z_fraction: f64 = (relative_z_m.abs() / height_m).clamp(0.0, 1.0);
                    let outer_half_width_m: f64 = section.half_width_m
                        * (1.0 - z_fraction.powf(exponent))
                            .max(0.0)
                            .powf(1.0 / exponent);
                    let inner_half_width_m = outer_half_width_m - wall_m;
                    let usable_width_m = 2.0 * inner_half_width_m;
                    if !usable_width_m.is_finite() || required_width_m > usable_width_m + 1e-9 {
                        return Err(FuselageError::InvalidInterior(format!(
                            "seat block '{}' row at x={x_m:.2} m needs {:.2} m width, loft provides {:.2} m",
                            block.name,
                            required_width_m,
                            usable_width_m.max(0.0)
                        )));
                    }

                    let lateral_span_m = seat_width_total_m + aisle_width_total_m;
                    let mut lateral_cursor_m = section.offset_y_m - 0.5 * lateral_span_m;
                    let mut column_index = 0_u32;
                    for (group_index, &group_places) in block.columns.iter().enumerate() {
                        for place_in_group in 0..group_places {
                            let position = DVec3::new(
                                x_m,
                                lateral_cursor_m + (place_in_group as f64 + 0.5) * seat_width_m,
                                deck.floor_z_m,
                            );
                            let occupied =
                                row * seat_count_per_row + column_index < block.occupants;
                            let seat_index = row * seat_count_per_row + column_index;
                            let (suited, suit_mass_kg, suit_type) = suit_overrides
                                .get(&seat_index)
                                .map(|suit_override| {
                                    (
                                        suit_override.suited,
                                        suit_override.suit_mass_kg_each,
                                        suit_override.suit_type,
                                    )
                                })
                                .unwrap_or((
                                    block.suited,
                                    block.suit_mass_kg_each,
                                    block.suit_type,
                                ));
                            let seat = CompiledCabinSeat {
                                name: format!(
                                    "{}.{}.{}.r{}.c{}",
                                    region.name,
                                    deck.name,
                                    block.name,
                                    row + 1,
                                    column_index + 1
                                ),
                                position_body_m: position,
                                class: block.class,
                                role: block.role,
                                seat_style: block.seat_style,
                                occupied,
                                suited,
                                suit_type,
                                seat_mass_kg: block.seat_mass_kg_each(),
                                occupant_mass_kg: block.occupant_mass_kg_each,
                                carry_on_mass_kg: block.carry_on_kg_each,
                                suit_mass_kg,
                            };
                            payload_mass_kg += seat.mass_kg();
                            seat_positions_body_m.push(position);
                            seats.push(seat);
                            column_index += 1;
                        }
                        lateral_cursor_m += group_places as f64 * seat_width_m;
                        if let Some(&aisle_width_m) = block.aisle_widths_m.get(group_index) {
                            lateral_cursor_m += aisle_width_m;
                        }
                    }
                }
            }

            for monument in &deck.monuments {
                let x_m = 0.5 * (monument.x0_m + monument.x1_m);
                let section = self.body.section_at(x_m);
                let position = DVec3::new(x_m, section.offset_y_m, deck.floor_z_m);
                monuments.push(CompiledCabinMonument {
                    name: format!("{}.{}.{}", region.name, deck.name, monument.name),
                    kind: monument.kind,
                    position_body_m: position,
                    mass_kg: monument.mass_kg,
                });
                payload_mass_kg += monument.mass_kg;
            }

            for door in &deck.doors {
                let section = self.body.section_at(door.x_m);
                let spec = door.rating.spec();
                let top_inner_z_m = section.offset_z_m + section.top_height_m - wall_m;
                let bottom_inner_z_m = section.offset_z_m - section.bottom_height_m + wall_m;
                let (door_height, door_exponent) = if deck.floor_z_m >= section.offset_z_m {
                    (section.top_height_m, section.top_exponent)
                } else {
                    (section.bottom_height_m, section.bottom_exponent)
                };
                let door_z_fraction: f64 =
                    ((deck.floor_z_m - section.offset_z_m).abs() / door_height).clamp(0.0, 1.0);
                let floor_half_width_m: f64 = section.half_width_m
                    * (1.0 - door_z_fraction.powf(door_exponent))
                        .max(0.0)
                        .powf(1.0 / door_exponent);
                if deck.floor_z_m < bottom_inner_z_m - 1e-9
                    || deck.floor_z_m + spec.opening_height_m > top_inner_z_m + 1e-9
                    || floor_half_width_m <= wall_m
                {
                    return Err(FuselageError::InvalidInterior(format!(
                        "door '{}' ({:?}) does not fit the loft at x={:.2} m on deck '{}'",
                        door.name, door.rating, door.x_m, deck.name
                    )));
                }
                let side_sign = match door.side {
                    DoorSide::Left => -1.0,
                    DoorSide::Right => 1.0,
                };
                doors.push(CompiledCabinDoor {
                    name: format!("{}.{}.{}", region.name, deck.name, door.name),
                    pair_id: format!("{}.{}.{}", region.name, deck.name, door.pair_id),
                    position_body_m: DVec3::new(
                        door.x_m,
                        section.offset_y_m + side_sign * floor_half_width_m,
                        deck.floor_z_m,
                    ),
                    side: door.side,
                    rating: door.rating,
                    opening_width_m: spec.opening_width_m,
                    opening_height_m: spec.opening_height_m,
                });
            }
        }

        let seat_count = u32::try_from(seats.len()).map_err(|_| {
            FuselageError::InvalidInterior(format!(
                "cabin '{}' has too many compiled seat places",
                region.name
            ))
        })?;
        if !payload_mass_kg.is_finite()
            || seats.iter().any(|seat| !seat.position_body_m.is_finite())
            || monuments
                .iter()
                .any(|monument| !monument.position_body_m.is_finite())
            || doors.iter().any(|door| !door.position_body_m.is_finite())
        {
            return Err(FuselageError::InvalidInterior(format!(
                "cabin '{}' compiled non-finite layout data",
                region.name
            )));
        }
        Ok(CompiledCabinLayout {
            seats,
            monuments,
            doors,
            seat_positions_body_m,
            seat_count,
            payload_mass_kg,
            seat_style: first_style.unwrap_or(SeatStyle::Upright),
        })
    }

    /// Usable inner-mold volume over an explicit axial range (split tanks).
    fn range_volume(
        &self,
        x0_m: f64,
        x1_m: f64,
        wall_m: f64,
    ) -> Result<(f64, f64, DVec3), FuselageError> {
        if x1_m - x0_m <= 1e-12 {
            return Err(FuselageError::InvalidInterior(
                "split tank needs a non-empty axial sub-range".into(),
            ));
        }
        let integrated = self.integrate_section(x0_m, x1_m, wall_m)?;
        let volume = integrated.moments.x;
        if volume <= 0.0 {
            return Err(FuselageError::InvalidInterior(
                "split tank sub-range has no usable volume".into(),
            ));
        }
        let centroid = DVec3::new(
            integrated.moments.y / volume,
            integrated.moments.z / volume,
            integrated.moments.w / volume,
        );
        Ok((volume, integrated.estimated_error.x, centroid))
    }

    /// Axial station where `[x0, split]` holds `target_volume_m3`.
    fn find_split_x(
        &self,
        x0_m: f64,
        x1_m: f64,
        target_volume_m3: f64,
        wall_m: f64,
    ) -> Result<f64, FuselageError> {
        let (total, _, _) = self.range_volume(x0_m, x1_m, wall_m)?;
        if target_volume_m3 <= 0.0 || target_volume_m3 >= total {
            return Err(FuselageError::InvalidInterior(
                "split tank target volume lies outside the region".into(),
            ));
        }
        let mut low = x0_m;
        let mut high = x1_m;
        for _ in 0..80 {
            let mid = 0.5 * (low + high);
            let integrated = self.integrate_section(x0_m, mid, wall_m)?;
            if integrated.moments.x < target_volume_m3 {
                low = mid;
            } else {
                high = mid;
            }
        }
        Ok(0.5 * (low + high))
    }

    fn equiv_diameter(volume_m3: f64, length_m: f64) -> f64 {
        let mean_area = volume_m3 / length_m.max(1e-12);
        2.0 * (mean_area / std::f64::consts::PI).sqrt()
    }

    fn sphere_diameter(volume_m3: f64) -> f64 {
        (6.0 * volume_m3 / std::f64::consts::PI).cbrt()
    }

    /// Pressure-shell spec for one tank: equivalent cylinder over the
    /// axial length, or a volume-sized sphere.
    fn tank_spec(
        shell: Option<TankShell>,
        volume_m3: f64,
        length_m: f64,
        pressure_pa: f64,
        material: thessa_sim_core::ChamberMaterial,
    ) -> TankSpec {
        let shape = match shell.unwrap_or(TankShell::Cylinder) {
            TankShell::Cylinder => TankShape::Cylinder {
                diameter_m: Self::equiv_diameter(volume_m3, length_m),
                length_m: length_m.max(1e-12),
            },
            TankShell::Sphere => TankShape::Sphere {
                diameter_m: Self::sphere_diameter(volume_m3),
            },
        };
        TankSpec {
            shape,
            pressure_pa,
            material,
        }
    }

    /// Seat anchors for a crew region: transverse rows of forward-facing
    /// places on the loft centerline. Rows spread evenly along the axis
    /// by default; explicit pitch centers them (validated to fit at
    /// authoring time). Within a row, places spread across the local
    /// section width and refuse if the row overflows the loft — the same
    /// row rule will serve multi-aisle airplane cabins later.
    fn seat_anchors(
        &self,
        x0_m: f64,
        x1_m: f64,
        seats: u32,
        seat_pitch_m: Option<f64>,
        abreast: u32,
    ) -> Result<Vec<DVec3>, FuselageError> {
        let abreast = abreast.max(1);
        let rows = seats.div_ceil(abreast);
        let length = x1_m - x0_m;
        let mut anchors = Vec::with_capacity(seats as usize);
        for row in 0..rows {
            let x = match seat_pitch_m {
                Some(pitch) => {
                    let block = (rows as f64 - 1.0) * pitch;
                    0.5 * (x0_m + x1_m) - 0.5 * block + row as f64 * pitch
                }
                None => x0_m + (row as f64 + 0.5) * length / rows as f64,
            };
            let section = self.body.section_at(x);
            let center = self.section_center(section)?;
            let in_row = (seats - row * abreast).min(abreast);
            // Shoulder room per place, capped so the outer place stays
            // inside the local section.
            let spacing = if in_row > 1 {
                (0.55_f64).min(1.6 * section.half_width_m / (in_row as f64 - 1.0))
            } else {
                0.0
            };
            for place in 0..in_row {
                let y = (place as f64 - (in_row as f64 - 1.0) / 2.0) * spacing;
                if y.abs() > section.half_width_m * 0.95 + 1e-9 {
                    return Err(FuselageError::InvalidInterior(format!(
                        "seat row at x={x:.2} m overflows the {:.2} m half-width",
                        section.half_width_m,
                    )));
                }
                anchors.push(DVec3::new(x, center.y + y, center.z));
            }
        }
        Ok(anchors)
    }

    /// Pressure-membrane screening for one pressurized region: thin-hoop
    /// stress `p*r/t` at the largest loft radius in the region must stay
    /// under yield with the shared safety factor. Conservative on purpose
    /// (vacuum outside, frames ignored); a failure names the numbers so
    /// the author thickens the skin, derates pressure, or picks a
    /// stronger alloy instead of flying an unverified shell.
    fn check_pressure_shell(
        &self,
        region_name: &str,
        x0_m: f64,
        x1_m: f64,
        atmosphere: CabinAtmosphere,
        layout: &crate::BodyStructuralLayout,
    ) -> Result<(), FuselageError> {
        let yield_mpa = layout.skin_material.yield_strength_mpa.ok_or_else(|| {
            FuselageError::InvalidBody(format!(
                "pressurized region '{region_name}' needs a skin yield strength for '{}'",
                layout.skin_material.name
            ))
        })?;
        let mut radius_m = 0.0_f64;
        for sample in 0..=PRESSURE_SHELL_SAMPLES {
            let x = x0_m + (x1_m - x0_m) * sample as f64 / PRESSURE_SHELL_SAMPLES as f64;
            let section = self.body.section_at(x);
            radius_m = radius_m
                .max(section.half_width_m)
                .max(section.top_height_m)
                .max(section.bottom_height_m);
        }
        let pressure_pa = atmosphere.pressure_kpa * 1000.0;
        let skin_m = layout.skin_gauge_mm / 1000.0;
        let required_m = pressure_pa * radius_m / (yield_mpa * 1.0e6 / PRESSURE_SAFETY_FACTOR);
        if skin_m < required_m {
            return Err(FuselageError::InvalidBody(format!(
                "pressurized region '{region_name}' needs {:.2} mm skin for {:.1} kPa at {:.2} m radius (has {:.2} mm)",
                required_m * 1000.0,
                atmosphere.pressure_kpa,
                radius_m,
                skin_m * 1000.0,
            )));
        }
        Ok(())
    }

    #[allow(clippy::too_many_arguments)]
    fn compile_single_tank(
        &self,
        name: &str,
        contents: TankContents,
        component: TankComponent,
        volume_m3: f64,
        volume_error_m3: f64,
        centroid: DVec3,
        fill_fraction: f64,
        spec: &TankSpec,
    ) -> Result<CompiledBodyTank, FuselageError> {
        let TankContents::Pair(propellant) = contents else {
            return Err(FuselageError::InvalidInterior(
                "single tank needs a propellant pair".into(),
            ));
        };
        let density = propellant_density(propellant)?;
        self.compile_tank_with_density(
            name,
            contents,
            component,
            volume_m3,
            volume_error_m3,
            centroid,
            density,
            fill_fraction,
            spec,
        )
    }

    #[allow(clippy::too_many_arguments)]
    fn compile_split_tank(
        &self,
        name: &str,
        contents: TankContents,
        component: TankComponent,
        volume_m3: f64,
        volume_error_m3: f64,
        centroid: DVec3,
        component_density: f64,
        fill_fraction: f64,
        spec: &TankSpec,
    ) -> Result<CompiledBodyTank, FuselageError> {
        if !component_density.is_finite() || component_density <= 0.0 {
            return Err(FuselageError::InvalidInterior(
                "split tank component needs a positive density".into(),
            ));
        }
        self.compile_tank_with_density(
            name,
            contents,
            component,
            volume_m3,
            volume_error_m3,
            centroid,
            component_density,
            fill_fraction,
            spec,
        )
    }

    #[allow(clippy::too_many_arguments)]
    fn compile_tank_with_density(
        &self,
        name: &str,
        contents: TankContents,
        component: TankComponent,
        volume_m3: f64,
        volume_error_m3: f64,
        centroid: DVec3,
        density_kg_m3: f64,
        fill_fraction: f64,
        spec: &TankSpec,
    ) -> Result<CompiledBodyTank, FuselageError> {
        let mut tank = spec.compile(density_kg_m3).map_err(|error| {
            FuselageError::InvalidBody(format!("tank region '{name}': {error}"))
        })?;
        // Real inner volume is the capacity (equivalent cylinder sizes
        // the shell); preserve capacity separately from the authored
        // initial fill in the installed mount.
        tank.volume_m3 = volume_m3;
        tank.full_propellant_kg = volume_m3 * density_kg_m3;
        let initial_propellant_kg = tank.full_propellant_kg * fill_fraction;
        let intrinsic_inertia_body_kg_m2 = spec
            .shape
            .intrinsic_inertia_body_kg_m2(tank.dry_mass_kg, initial_propellant_kg)
            .map_err(|error| {
                FuselageError::InvalidBody(format!("tank region '{name}': {error}"))
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
        Ok(CompiledBodyTank {
            region_name: name.into(),
            mount,
            inner_volume_m3: volume_m3,
            inner_volume_error_m3: volume_error_m3,
            propellant_kg: initial_propellant_kg,
            contents,
            component,
        })
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
    /// ports, interior centroids, seat anchors, and the summary volume
    /// center shift here exactly once.
    fn mount(&self, mut compiled: CompiledBody) -> CompiledBody {
        let origin = self.body.origin_body_m;
        for port in &mut compiled.ports {
            port.position_body_m += origin;
        }
        for region in &mut compiled.interior {
            region.centroid_body_m += origin;
            for seat in &mut region.seat_positions_body_m {
                *seat += origin;
            }
            for seat in &mut region.cabin_seats {
                seat.position_body_m += origin;
            }
            for monument in &mut region.cabin_monuments {
                monument.position_body_m += origin;
            }
            for door in &mut region.cabin_doors {
                door.position_body_m += origin;
            }
        }
        for shield in &mut compiled.heat_shields {
            shield.position_body_m += origin;
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

/// Stack interface diameters must agree this closely (relative) or the
/// author fits an adapter part (future) instead of forcing the joint.
const STACK_DIAMETER_TOLERANCE: f64 = 0.05;
/// A hatch below this diameter cannot pass crew (documented minimum).
const HATCH_MIN_DIAMETER_M: f64 = 0.5;

/// One assembly link between two attach nodes on different bodies.
/// Stack links are permanently open for resources; hatch links carry an
/// initial open state (crew/air/fuel cross only when open).
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct AssemblyLink {
    pub name: String,
    pub parent_body: String,
    pub parent_node: String,
    pub child_body: String,
    pub child_node: String,
    /// Initial hatch state; forced open on stack links.
    #[serde(default = "default_hatch_open")]
    pub hatch_open: bool,
}

fn default_hatch_open() -> bool {
    true
}

/// One habitable volume: a non-tank interior region on one body.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub struct VolumeId {
    pub body: usize,
    pub region: usize,
}

/// One fuel path: a tank region that can reach an engine-mount port
/// through open resource links.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct FeedPath {
    /// `body.region` qualified tank name.
    pub tank: String,
    /// `body.port` qualified engine-mount port name.
    pub engine_port: String,
}

/// Rigid transform from one part's authored body frame into the assembled
/// vehicle frame. Points include the translation; directions do not.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct BodyTransform {
    pub rotation_body: DQuat,
    pub translation_body_m: DVec3,
}

impl BodyTransform {
    pub fn transform_point(self, point_body_m: DVec3) -> DVec3 {
        self.rotation_body * point_body_m + self.translation_body_m
    }

    pub fn transform_direction(self, direction_body: DVec3) -> DVec3 {
        self.rotation_body * direction_body
    }

    /// Rotate a centroidal tensor into the vehicle axes. Translation of
    /// the tensor reference point uses a separate parallel-axis term.
    pub fn rotate_inertia(self, inertia_body_kg_m2: DMat3) -> DMat3 {
        let rotation = DMat3::from_quat(self.rotation_body);
        rotation * inertia_body_kg_m2 * rotation.transpose()
    }
}

/// Compiled part assembly: validated topology plus crew/air sharing
/// domains and fuel reachability. Merged physics (transforms, joints)
/// arrives in a later slice; this record owns topology truth.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct CompiledAssembly {
    /// Root body name (the one never used as a child).
    pub root: String,
    /// Part-to-vehicle transforms indexed exactly like the `bodies`
    /// argument to [`compile_assembly`]. The root keeps its authored
    /// translation; linked children are placed by mating attach frames.
    pub body_transforms: Vec<BodyTransform>,
    /// Crew-passable volume groups (open hatch links; suits and air
    /// checked at runtime, not here).
    pub crew_groups: Vec<Vec<VolumeId>>,
    /// Shared-air domains (open hatch links between pressurized volumes).
    pub air_groups: Vec<Vec<VolumeId>>,
    /// Tank-to-engine-port fuel reachability through open links.
    pub feed_paths: Vec<FeedPath>,
}

/// Compile a part assembly from bodies plus links (KSP-style tree).
///
/// Fails closed on: unknown bodies/nodes, a node used twice, diameter
/// mismatch beyond tolerance, hatch links below crew-passage diameter,
/// cycles, and forests (more than one root). A single body with no
/// links compiles to one root with isolated volumes.
pub fn compile_assembly(
    bodies: &[ProceduralBody],
    links: &[AssemblyLink],
) -> Result<CompiledAssembly, FuselageError> {
    for body in bodies {
        body.validate()?;
    }
    if bodies.is_empty() {
        return Err(FuselageError::InvalidBody(
            "assembly needs at least one body".into(),
        ));
    }
    let mut body_names = std::collections::HashSet::new();
    if bodies
        .iter()
        .any(|body| !body_names.insert(body.name.as_str()))
    {
        return Err(FuselageError::InvalidBody(
            "assembly body names must be unique".into(),
        ));
    }
    let body_index = |name: &str| {
        bodies
            .iter()
            .position(|body| body.name == name)
            .ok_or_else(|| {
                FuselageError::InvalidBody(format!("assembly references unknown body '{name}'"))
            })
    };
    let node_diameter = |body: &ProceduralBody, node: &crate::AttachNode| -> f64 {
        if let Some(diameter) = node.diameter_m {
            return diameter;
        }
        let station = match node.site {
            AttachSite::AftEnd => body.stations[0],
            AttachSite::ForwardEnd => *body.stations.last().unwrap(),
            AttachSite::Station { x_m, .. } => body.section_at(x_m),
        };
        2.0 * station
            .half_width_m
            .max(station.top_height_m)
            .max(station.bottom_height_m)
    };
    // Resolve endpoints, check single use and diameter match.
    struct Resolved {
        parent: usize,
        child: usize,
        parent_node: usize,
        child_node: usize,
        hatch: bool,
        open: bool,
    }
    let mut resolved = Vec::with_capacity(links.len());
    let mut link_names = std::collections::HashSet::new();
    let mut used: std::collections::HashSet<(usize, &str)> = std::collections::HashSet::new();
    for link in links {
        if link.name.trim().is_empty() || !link_names.insert(link.name.as_str()) {
            return Err(FuselageError::InvalidBody(
                "assembly link names must be non-empty and unique".into(),
            ));
        }
        let parent = body_index(&link.parent_body)?;
        let child = body_index(&link.child_body)?;
        if parent == child {
            return Err(FuselageError::InvalidBody(format!(
                "assembly link '{}' connects a body to itself",
                link.name
            )));
        }
        let parent_node = bodies[parent]
            .attach_nodes
            .iter()
            .find(|node| node.name == link.parent_node)
            .ok_or_else(|| {
                FuselageError::InvalidBody(format!(
                    "assembly link '{}' references unknown node '{}.{}'",
                    link.name, link.parent_body, link.parent_node
                ))
            })?;
        let parent_node_index = bodies[parent]
            .attach_nodes
            .iter()
            .position(|node| node.name == link.parent_node)
            .expect("resolved parent node exists");
        let child_node = bodies[child]
            .attach_nodes
            .iter()
            .find(|node| node.name == link.child_node)
            .ok_or_else(|| {
                FuselageError::InvalidBody(format!(
                    "assembly link '{}' references unknown node '{}.{}'",
                    link.name, link.child_body, link.child_node
                ))
            })?;
        let child_node_index = bodies[child]
            .attach_nodes
            .iter()
            .position(|node| node.name == link.child_node)
            .expect("resolved child node exists");
        for (index, node) in [
            (parent, parent_node.name.as_str()),
            (child, child_node.name.as_str()),
        ] {
            if !used.insert((index, node)) {
                return Err(FuselageError::InvalidBody(format!(
                    "attach node '{node}' is used by more than one link"
                )));
            }
        }
        let parent_d = node_diameter(&bodies[parent], parent_node);
        let child_d = node_diameter(&bodies[child], child_node);
        if !parent_d.is_finite() || !child_d.is_finite() || parent_d <= 0.0 || child_d <= 0.0 {
            return Err(FuselageError::InvalidBody(format!(
                "assembly link '{}' requires positive finite interface diameters",
                link.name
            )));
        }
        let mismatch = (parent_d - child_d).abs() / parent_d.max(child_d).max(f64::MIN_POSITIVE);
        if mismatch > STACK_DIAMETER_TOLERANCE {
            return Err(FuselageError::InvalidBody(format!(
                "assembly link '{}' joins {:.2} m to {:.2} m (beyond {:.0}% tolerance)",
                link.name,
                parent_d,
                child_d,
                STACK_DIAMETER_TOLERANCE * 100.0,
            )));
        }
        let hatch = parent_node.kind == AttachKind::Hatch || child_node.kind == AttachKind::Hatch;
        if hatch && parent_d.min(child_d) < HATCH_MIN_DIAMETER_M {
            return Err(FuselageError::InvalidBody(format!(
                "assembly link '{}' hatch is {:.2} m (crew passage needs {:.1} m)",
                link.name,
                parent_d.min(child_d),
                HATCH_MIN_DIAMETER_M,
            )));
        }
        resolved.push(Resolved {
            parent,
            child,
            parent_node: parent_node_index,
            child_node: child_node_index,
            hatch,
            open: if hatch { link.hatch_open } else { true },
        });
    }
    // Tree check: acyclic (union-find rejects the closing edge) with
    // exactly one root and one connected set (no forests).
    let mut union = UnionFind::new(bodies.len());
    for link in &resolved {
        if !union.union(link.parent, link.child) {
            return Err(FuselageError::InvalidBody(format!(
                "assembly link between '{}' and '{}' closes a cycle (trees only)",
                bodies[link.parent].name, bodies[link.child].name
            )));
        }
    }
    // The root is the one body never used as a child; every body must
    // share its connected set (single tree, no forests).
    let is_child = |index: usize| resolved.iter().any(|link| link.child == index);
    let roots: Vec<usize> = (0..bodies.len())
        .filter(|index| !is_child(*index))
        .collect();
    if roots.len() != 1 {
        return Err(FuselageError::InvalidBody(format!(
            "assembly needs exactly one root body (found {})",
            roots.len()
        )));
    }
    let main = union.find(roots[0]);
    if !(0..bodies.len()).all(|index| union.find(index) == main) {
        return Err(FuselageError::InvalidBody(
            "assembly is a forest, not one connected tree".into(),
        ));
    }
    // Resolve each part pose from attach-frame coincidence. The mating
    // half-turn makes node outward axes oppose while preserving an
    // authored stack's axial roll convention.
    let mut body_transforms = vec![None; bodies.len()];
    body_transforms[roots[0]] = Some(BodyTransform {
        rotation_body: DQuat::IDENTITY,
        translation_body_m: bodies[roots[0]].origin_body_m,
    });
    let mut pose_stack = vec![roots[0]];
    while let Some(parent_index) = pose_stack.pop() {
        let parent_transform = body_transforms[parent_index].expect("rooted traversal");
        for link in resolved.iter().filter(|link| link.parent == parent_index) {
            let parent_body = &bodies[link.parent];
            let child_body = &bodies[link.child];
            let parent_node = &parent_body.attach_nodes[link.parent_node];
            let child_node = &child_body.attach_nodes[link.child_node];
            let (parent_local_position, parent_local_rotation) =
                attach_node_pose(parent_body, parent_node);
            let (child_local_position, child_local_rotation) =
                attach_node_pose(child_body, child_node);
            let mating_rotation = DQuat::from_rotation_y(std::f64::consts::PI);
            let child_rotation = (parent_transform.rotation_body
                * parent_local_rotation
                * mating_rotation
                * child_local_rotation.inverse())
            .normalize();
            let parent_node_world = parent_transform.transform_point(parent_local_position);
            let child_translation = parent_node_world - child_rotation * child_local_position;
            body_transforms[link.child] = Some(BodyTransform {
                rotation_body: child_rotation,
                translation_body_m: child_translation,
            });
            pose_stack.push(link.child);
        }
    }
    let body_transforms = body_transforms
        .into_iter()
        .map(|transform| transform.expect("connected tree assigns every part pose"))
        .collect::<Vec<_>>();
    // Habitable volumes per body (non-tank regions).
    let is_volume = |kind: &RegionKind| {
        !matches!(
            kind,
            RegionKind::Tank { .. }
                | RegionKind::FluidTank { .. }
                | RegionKind::Bipropellant { .. }
        )
    };
    let mut volumes: Vec<VolumeId> = Vec::new();
    for (body_index, body) in bodies.iter().enumerate() {
        for (region_index, region) in body.regions.iter().enumerate() {
            if is_volume(&region.kind) {
                volumes.push(VolumeId {
                    body: body_index,
                    region: region_index,
                });
            }
        }
    }
    let volume_index = |id: VolumeId| {
        volumes
            .iter()
            .position(|volume| *volume == id)
            .expect("volume inventoried above")
    };
    // Crew groups: open hatch links only; a structural stack joint is not
    // an interior passage.
    let mut crew_union = UnionFind::new(volumes.len());
    // Air groups: open links between pressurized volumes only.
    let mut air_union = UnionFind::new(volumes.len());
    let pressurized = |id: VolumeId| bodies[id.body].regions[id.region].atmosphere.is_some();
    // Crew passes only through open hatches. Resource paths cross stack
    // joints unconditionally and hatches only while open.
    let mut crew_adj: Vec<Vec<usize>> = vec![Vec::new(); bodies.len()];
    let mut resource_adj: Vec<Vec<usize>> = vec![Vec::new(); bodies.len()];
    for link in &resolved {
        if link.hatch && link.open {
            crew_adj[link.parent].push(link.child);
            crew_adj[link.child].push(link.parent);
        }
        if !link.hatch || link.open {
            resource_adj[link.parent].push(link.child);
            resource_adj[link.child].push(link.parent);
        }
    }
    // Volumes in one body share the open interior (documented rule).
    for body_volumes in volumes.chunk_by(|a, b| a.body == b.body) {
        for pair in body_volumes.windows(2) {
            crew_union.union(volume_index(pair[0]), volume_index(pair[1]));
            if pressurized(pair[0]) && pressurized(pair[1]) {
                air_union.union(volume_index(pair[0]), volume_index(pair[1]));
            }
        }
    }
    for (body_index, neighbors) in crew_adj.iter().enumerate() {
        for neighbor in neighbors {
            let a: Vec<VolumeId> = volumes
                .iter()
                .copied()
                .filter(|volume| volume.body == body_index)
                .collect();
            let b: Vec<VolumeId> = volumes
                .iter()
                .copied()
                .filter(|volume| volume.body == *neighbor)
                .collect();
            for x in &a {
                for y in &b {
                    crew_union.union(volume_index(*x), volume_index(*y));
                    if pressurized(*x) && pressurized(*y) {
                        air_union.union(volume_index(*x), volume_index(*y));
                    }
                }
            }
        }
    }
    let groups = |union: &mut UnionFind| {
        let mut groups: std::collections::HashMap<usize, Vec<VolumeId>> =
            std::collections::HashMap::new();
        for (index, volume) in volumes.iter().enumerate() {
            groups.entry(union.find(index)).or_default().push(*volume);
        }
        let mut groups: Vec<Vec<VolumeId>> = groups.into_values().collect();
        for group in &mut groups {
            group.sort_by_key(|volume| (volume.body, volume.region));
        }
        groups.sort_by_key(|group| (group[0].body, group[0].region));
        groups
    };
    // Fuel reachability: tanks to engine-mount ports through resource-open
    // links (stack joints always flow; sealed hatches block crossfeed).
    let mut tanks: Vec<(usize, String)> = Vec::new();
    for (body_index, body) in bodies.iter().enumerate() {
        for region in &body.regions {
            let tank_kind = matches!(
                region.kind,
                RegionKind::Tank { .. }
                    | RegionKind::FluidTank { .. }
                    | RegionKind::Bipropellant { .. }
            );
            if tank_kind {
                tanks.push((body_index, format!("{}.{}", body.name, region.name)));
            }
        }
    }
    let mut engine_ports: Vec<(usize, String)> = Vec::new();
    for (body_index, body) in bodies.iter().enumerate() {
        for port in &body.ports {
            if port.kind == crate::PortKind::EngineMount {
                engine_ports.push((body_index, format!("{}.{}", body.name, port.name)));
            }
        }
    }
    let reachable = |from: usize| {
        let mut seen = vec![false; bodies.len()];
        let mut stack = vec![from];
        seen[from] = true;
        while let Some(next) = stack.pop() {
            for neighbor in &resource_adj[next] {
                if !seen[*neighbor] {
                    seen[*neighbor] = true;
                    stack.push(*neighbor);
                }
            }
        }
        seen
    };
    let mut feed_paths = Vec::new();
    for (tank_body, tank) in &tanks {
        let seen = reachable(*tank_body);
        for (port_body, engine_port) in &engine_ports {
            if seen[*port_body] {
                feed_paths.push(FeedPath {
                    tank: tank.clone(),
                    engine_port: engine_port.clone(),
                });
            }
        }
    }
    feed_paths.sort_by(|a, b| (&a.tank, &a.engine_port).cmp(&(&b.tank, &b.engine_port)));
    Ok(CompiledAssembly {
        root: bodies[roots[0]].name.clone(),
        body_transforms,
        crew_groups: groups(&mut crew_union),
        air_groups: groups(&mut air_union),
        feed_paths,
    })
}

fn attach_node_pose(body: &ProceduralBody, node: &crate::AttachNode) -> (DVec3, DQuat) {
    match node.site {
        AttachSite::AftEnd => (
            DVec3::new(
                body.stations[0].x_m,
                body.stations[0].offset_y_m,
                body.stations[0].offset_z_m,
            ),
            DQuat::from_rotation_y(std::f64::consts::PI),
        ),
        AttachSite::ForwardEnd => {
            let station = *body.stations.last().expect("validated stations");
            (
                DVec3::new(station.x_m, station.offset_y_m, station.offset_z_m),
                DQuat::IDENTITY,
            )
        }
        AttachSite::Station { x_m, clock_rad } => {
            let station = body.section_at(x_m);
            let (y, z) = outline_point(
                station.half_width_m,
                station.top_height_m,
                station.bottom_height_m,
                station.top_exponent,
                station.bottom_exponent,
                clock_rad,
            );
            let sin = clock_rad.sin();
            let (height, exponent) = if sin >= 0.0 {
                (station.top_height_m, station.top_exponent)
            } else {
                (station.bottom_height_m, station.bottom_exponent)
            };
            let normal_y = y.signum() * (y.abs() / station.half_width_m).powf(exponent - 1.0)
                / station.half_width_m;
            let normal_z = z.signum() * (z.abs() / height).powf(exponent - 1.0) / height;
            let outward = DVec3::new(0.0, normal_y, normal_z).normalize();
            (
                DVec3::new(station.x_m, station.offset_y_m + y, station.offset_z_m + z),
                DQuat::from_rotation_arc(DVec3::X, outward),
            )
        }
    }
}

/// Disjoint-set union for tree and group computation.
struct UnionFind {
    parent: Vec<usize>,
}

impl UnionFind {
    fn new(count: usize) -> Self {
        Self {
            parent: (0..count).collect(),
        }
    }

    fn find(&mut self, mut index: usize) -> usize {
        while self.parent[index] != index {
            self.parent[index] = self.parent[self.parent[index]];
            index = self.parent[index];
        }
        index
    }

    /// Returns false when already united (cycle edge).
    fn union(&mut self, a: usize, b: usize) -> bool {
        let (root_a, root_b) = (self.find(a), self.find(b));
        if root_a == root_b {
            return false;
        }
        self.parent[root_a] = root_b;
        true
    }
}
