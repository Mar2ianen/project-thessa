//! Solid motors: BATES cylindrical-port grains (uniform or stepped channel)
//! plus numerically regressed star/finocyl ports, with equilibrium pressure
//! coupled to the instantaneous burning perimeter.

use serde::{Deserialize, Serialize};

use super::{
    AEROSPIKE_BASE_FRACTION, ChamberMaterial, MASS_FIT_FEED_KG_PER_N, MASS_FIT_GIMBAL_BASE_KG,
    MASS_FIT_GIMBAL_KG_PER_N, MASS_FIT_MOUNT_KG_PER_N, MASS_FIT_SOLID_IGNITER_KG, NozzleContour,
    NozzleExitState, PRESSURE_SAFETY_FACTOR, Propellant, PropellantThermo, PropulsionError,
    STANDARD_GRAVITY_MPS2, characteristic_velocity, nozzle_exit, require_non_negative,
    require_positive, thrust_coefficient,
};

const GRAIN_BURNBACK_GRID_SIDE: usize = 128;
const GRAIN_BURNBACK_CURVE_STATIONS: usize = 400;

/// Cross-section of the initial solid-grain port. `Circular` is the legacy
/// BATES geometry; star and finocyl profiles regress by Euclidean distance
/// into the grain and use a numerically sampled port perimeter.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize, Default)]
#[serde(tag = "kind", rename_all = "kebab-case")]
pub enum SolidGrainGeometry {
    /// Circular BATES port; `core_radius_m` and optional stepped radii apply.
    #[default]
    Circular,
    /// Alternating root/tip polygon. `core_radius_m` is the root radius.
    Star { tip_count: u8, tip_radius_m: f64 },
    /// Central circular port with radial fin slots. Width is angular at the
    /// fin tip; `core_radius_m` is the central-port/root radius.
    Finocyl {
        fin_count: u8,
        fin_tip_radius_m: f64,
        fin_width_rad: f64,
    },
}

/// BATES cylindrical-port grain authoring (Juno fuel-grain equivalent).
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct SolidMotorSpec {
    pub name: String,
    /// Ammonium-perchlorate composite today; the grain solver reads thermo
    /// from the pair so future chemistries plug in without a new solver.
    pub propellant: Propellant,
    /// Grain outer radius (m).
    pub outer_radius_m: f64,
    /// Initial port (core) radius (m).
    pub core_radius_m: f64,
    /// Initial grain-port cross-section (omitted authoring data defaults to
    /// the legacy circular BATES grain).
    #[serde(default)]
    pub grain_geometry: SolidGrainGeometry,
    /// Grain length per segment (m).
    pub segment_length_m: f64,
    /// Number of segments.
    pub segments: u32,
    /// Burn-rate coefficient a in r = a * Pc^n (m/s/Pa^n).
    pub burn_rate_coeff: f64,
    /// Burn-rate exponent n (must be < 1 for stable equilibrium).
    pub burn_rate_exponent: f64,
    pub throat_radius_m: f64,
    pub expansion_ratio: f64,
    pub nozzle_length_m: f64,
    pub contour: NozzleContour,
    pub casing_material: ChamberMaterial,
    /// Inhibited segment ends (neutral trace); exposed ends burn too.
    pub inhibited_ends: bool,
    /// Per-segment initial port radii (m) for a stepped channel
    /// (boost-sustain shaping); `None` = uniform `core_radius_m`.
    pub segment_core_radii_m: Option<Vec<f64>>,
    pub gimbal_range_rad: f64,
    /// Ignition shots carried (solids are single-shot by default).
    pub ignition_shots: u32,
}

#[derive(Debug, Clone, Copy, Default)]
struct PortSectionSample {
    area_m2: f64,
    perimeter_m: f64,
}

#[derive(Debug, Clone, Copy, Default)]
struct BurnGeometry {
    burning_area_m2: f64,
    port_area_m2: f64,
    port_perimeter_m: f64,
}

/// Tabulated Euclidean-distance burnback for one polygonal port section.
/// The signed-distance grid is reduced once at bake time to a compact curve;
/// runtime still consumes the ordinary compiled burn trace.
#[derive(Debug, Clone)]
struct NumericalPortCurve {
    max_web_m: f64,
    samples: Vec<PortSectionSample>,
}

#[derive(Debug, Clone, Copy)]
struct PerimeterEvent {
    threshold_m: f64,
    delta_m: f64,
}

impl NumericalPortCurve {
    fn compile(geometry: SolidGrainGeometry, root_radius_m: f64, outer_radius_m: f64) -> Self {
        let vertices = port_polygon(geometry, root_radius_m);
        let side = GRAIN_BURNBACK_GRID_SIDE;
        let spacing_m = 2.0 * outer_radius_m / side as f64;
        let mut signed_distance = vec![f64::INFINITY; side * side];
        let mut distances = Vec::new();
        for y in 0..side {
            let py = -outer_radius_m + (y as f64 + 0.5) * spacing_m;
            for x in 0..side {
                let px = -outer_radius_m + (x as f64 + 0.5) * spacing_m;
                if px * px + py * py <= outer_radius_m * outer_radius_m {
                    let distance = signed_distance_to_polygon([px, py], &vertices);
                    signed_distance[y * side + x] = distance;
                    distances.push(distance);
                }
            }
        }
        distances.sort_by(f64::total_cmp);
        let max_web_m = distances.last().copied().unwrap_or(0.0).max(0.0);

        // Crofton perimeter estimate from grid crossings in four directions.
        // The diagonal directions use their perpendicular line spacing.
        let base_weight_m = std::f64::consts::PI / 8.0 * spacing_m;
        let directions = [
            (1_isize, 0_isize, base_weight_m),
            (0, 1, base_weight_m),
            (1, 1, base_weight_m / std::f64::consts::SQRT_2),
            (1, -1, base_weight_m / std::f64::consts::SQRT_2),
        ];
        let mut events = Vec::new();
        for y in 0..side {
            for x in 0..side {
                let first = signed_distance[y * side + x];
                if !first.is_finite() {
                    continue;
                }
                for (dx, dy, weight_m) in directions {
                    let nx = x as isize + dx;
                    let ny = y as isize + dy;
                    if !(0..side as isize).contains(&nx) || !(0..side as isize).contains(&ny) {
                        continue;
                    }
                    let second = signed_distance[ny as usize * side + nx as usize];
                    if !second.is_finite() || first == second {
                        continue;
                    }
                    let (low, high) = if first < second {
                        (first, second)
                    } else {
                        (second, first)
                    };
                    events.push(PerimeterEvent {
                        threshold_m: low,
                        delta_m: weight_m,
                    });
                    events.push(PerimeterEvent {
                        threshold_m: high,
                        delta_m: -weight_m,
                    });
                }
            }
        }
        events.sort_by(|left, right| left.threshold_m.total_cmp(&right.threshold_m));

        let cell_area_m2 = spacing_m * spacing_m;
        let mut samples = Vec::with_capacity(GRAIN_BURNBACK_CURVE_STATIONS + 1);
        let mut area_cursor = 0;
        let mut event_cursor = 0;
        let mut occupied_cells = 0usize;
        let mut perimeter_m = 0.0;
        for station in 0..=GRAIN_BURNBACK_CURVE_STATIONS {
            let depth_m = max_web_m * station as f64 / GRAIN_BURNBACK_CURVE_STATIONS as f64;
            while area_cursor < distances.len() && distances[area_cursor] <= depth_m {
                occupied_cells += 1;
                area_cursor += 1;
            }
            while event_cursor < events.len() && events[event_cursor].threshold_m <= depth_m {
                perimeter_m += events[event_cursor].delta_m;
                event_cursor += 1;
            }
            samples.push(PortSectionSample {
                area_m2: occupied_cells as f64 * cell_area_m2,
                perimeter_m: perimeter_m.max(0.0),
            });
        }
        Self { max_web_m, samples }
    }

    fn sample(&self, web_m: f64) -> PortSectionSample {
        let fraction = (web_m / self.max_web_m).clamp(0.0, 1.0);
        let station = fraction * GRAIN_BURNBACK_CURVE_STATIONS as f64;
        let lower = (station.floor() as usize).min(GRAIN_BURNBACK_CURVE_STATIONS);
        let upper = (lower + 1).min(GRAIN_BURNBACK_CURVE_STATIONS);
        let t = station - lower as f64;
        let a = self.samples[lower];
        let b = self.samples[upper];
        PortSectionSample {
            area_m2: a.area_m2 + (b.area_m2 - a.area_m2) * t,
            perimeter_m: a.perimeter_m + (b.perimeter_m - a.perimeter_m) * t,
        }
    }
}

fn port_polygon(geometry: SolidGrainGeometry, root_radius_m: f64) -> Vec<[f64; 2]> {
    let mut vertices = Vec::new();
    match geometry {
        SolidGrainGeometry::Circular => unreachable!("circular ports use the analytic path"),
        SolidGrainGeometry::Star {
            tip_count,
            tip_radius_m,
        } => {
            let pitch_rad = std::f64::consts::TAU / f64::from(tip_count);
            for tip in 0..tip_count {
                vertices.push(polar_point(root_radius_m, f64::from(tip) * pitch_rad));
                vertices.push(polar_point(
                    tip_radius_m,
                    (f64::from(tip) + 0.5) * pitch_rad,
                ));
            }
        }
        SolidGrainGeometry::Finocyl {
            fin_count,
            fin_tip_radius_m,
            fin_width_rad,
        } => {
            let pitch_rad = std::f64::consts::TAU / f64::from(fin_count);
            const FIN_TIP_ARC_SUBDIVISIONS: usize = 4;
            for fin in 0..fin_count {
                let center = (f64::from(fin) + 0.5) * pitch_rad;
                let start = center - 0.5 * fin_width_rad;
                let end = center + 0.5 * fin_width_rad;
                vertices.push(polar_point(root_radius_m, start));
                vertices.push(polar_point(fin_tip_radius_m, start));
                for subdivision in 1..=FIN_TIP_ARC_SUBDIVISIONS {
                    let fraction = subdivision as f64 / FIN_TIP_ARC_SUBDIVISIONS as f64;
                    vertices.push(polar_point(
                        fin_tip_radius_m,
                        start + fraction * (end - start),
                    ));
                }
                vertices.push(polar_point(root_radius_m, end));
            }
        }
    }
    vertices
}

fn polar_point(radius_m: f64, angle_rad: f64) -> [f64; 2] {
    [radius_m * angle_rad.cos(), radius_m * angle_rad.sin()]
}

fn signed_distance_to_polygon(point: [f64; 2], vertices: &[[f64; 2]]) -> f64 {
    let mut inside = false;
    let mut min_distance_sq = f64::INFINITY;
    for index in 0..vertices.len() {
        let a = vertices[index];
        let b = vertices[(index + 1) % vertices.len()];
        let edge = [b[0] - a[0], b[1] - a[1]];
        let length_sq = edge[0] * edge[0] + edge[1] * edge[1];
        let projection = if length_sq > 0.0 {
            (((point[0] - a[0]) * edge[0] + (point[1] - a[1]) * edge[1]) / length_sq)
                .clamp(0.0, 1.0)
        } else {
            0.0
        };
        let nearest = [a[0] + projection * edge[0], a[1] + projection * edge[1]];
        let dx = point[0] - nearest[0];
        let dy = point[1] - nearest[1];
        min_distance_sq = min_distance_sq.min(dx * dx + dy * dy);

        if (a[1] > point[1]) != (b[1] > point[1]) {
            let crossing_x = a[0] + (point[1] - a[1]) * edge[0] / edge[1];
            if point[0] < crossing_x {
                inside = !inside;
            }
        }
    }
    let distance_m = min_distance_sq.sqrt();
    if inside { -distance_m } else { distance_m }
}

fn burn_geometry_at(
    outer_radius_m: f64,
    segment_length_m: f64,
    inhibited_ends: bool,
    core_radii_m: &[f64],
    webs_m: &[f64],
    burned_m: &[f64],
    port_curves: &[Option<NumericalPortCurve>],
) -> BurnGeometry {
    let grain_area_m2 = std::f64::consts::PI * outer_radius_m * outer_radius_m;
    let mut geometry = BurnGeometry::default();
    for index in 0..core_radii_m.len() {
        if burned_m[index] >= webs_m[index] {
            geometry.port_area_m2 += grain_area_m2;
            continue;
        }
        let section = if let Some(curve) = &port_curves[index] {
            curve.sample(burned_m[index])
        } else {
            let radius_m = core_radii_m[index] + burned_m[index];
            PortSectionSample {
                area_m2: std::f64::consts::PI * radius_m * radius_m,
                perimeter_m: std::f64::consts::TAU * radius_m,
            }
        };
        let port_area_m2 = section.area_m2.clamp(0.0, grain_area_m2);
        let perimeter_m = section.perimeter_m.max(0.0);
        geometry.port_area_m2 += port_area_m2;
        geometry.port_perimeter_m += perimeter_m;
        geometry.burning_area_m2 += perimeter_m * segment_length_m;
        if !inhibited_ends {
            geometry.burning_area_m2 += 2.0 * (grain_area_m2 - port_area_m2).max(0.0);
        }
    }
    geometry
}

impl SolidMotorSpec {
    /// Reference APCP ballistics (documented typical: ~10 mm/s at 7 MPa).
    pub fn apcp_ballistics() -> (f64, f64) {
        (4.0e-5, 0.35)
    }

    /// Validate authoring values (NaN fails closed).
    #[allow(clippy::neg_cmp_op_on_partial_ord)]
    pub fn validate(&self) -> Result<(), PropulsionError> {
        if self.name.trim().is_empty() {
            return Err(PropulsionError::InvalidSpec(
                "motor name must not be empty".into(),
            ));
        }
        if !self.propellant.is_solid() {
            return Err(PropulsionError::InvalidSpec(
                "grain ballistics need a solid propellant".into(),
            ));
        }
        require_positive(self.outer_radius_m, "grain outer radius")?;
        require_positive(self.core_radius_m, "grain core radius")?;
        if !(self.core_radius_m < self.outer_radius_m) {
            return Err(PropulsionError::InvalidSpec(
                "grain core must be smaller than the outer radius".into(),
            ));
        }
        require_positive(self.segment_length_m, "grain segment length")?;
        if self.segments == 0 || self.segments > 16 {
            return Err(PropulsionError::InvalidSpec(
                "segments must be in 1..=16".into(),
            ));
        }
        require_positive(self.burn_rate_coeff, "burn-rate coefficient")?;
        if !(self.burn_rate_exponent > 0.0) || !(self.burn_rate_exponent < 1.0) {
            return Err(PropulsionError::InvalidSpec(
                "burn-rate exponent must be in (0, 1): n >= 1 has no stable pressure equilibrium"
                    .into(),
            ));
        }
        require_positive(self.throat_radius_m, "throat radius")?;
        if !(self.expansion_ratio >= 1.0) {
            return Err(PropulsionError::InvalidSpec(
                "expansion ratio must be finite and >= 1".into(),
            ));
        }
        require_positive(self.nozzle_length_m, "nozzle length")?;
        self.casing_material.validate()?;
        require_non_negative(self.gimbal_range_rad, "gimbal range")?;
        if let Some(radii) = &self.segment_core_radii_m {
            if radii.len() != self.segments as usize {
                return Err(PropulsionError::InvalidSpec(
                    "segment port radii must match the segment count".into(),
                ));
            }
            for radius_m in radii {
                require_positive(*radius_m, "segment port radius")?;
                if !(*radius_m < self.outer_radius_m) {
                    return Err(PropulsionError::InvalidSpec(
                        "segment port must be smaller than the outer radius".into(),
                    ));
                }
            }
        }
        let minimum_resolved_feature_m =
            4.0 * self.outer_radius_m / GRAIN_BURNBACK_GRID_SIDE as f64;
        match self.grain_geometry {
            SolidGrainGeometry::Circular => {}
            SolidGrainGeometry::Star {
                tip_count,
                tip_radius_m,
            } => {
                if !(3..=16).contains(&tip_count) {
                    return Err(PropulsionError::InvalidSpec(
                        "star grain tip count must be in 3..=16".into(),
                    ));
                }
                require_positive(tip_radius_m, "star grain tip radius")?;
                let roots_fit = self
                    .segment_core_radii_m
                    .as_deref()
                    .unwrap_or(&[self.core_radius_m])
                    .iter()
                    .all(|root| root < &tip_radius_m);
                if !(tip_radius_m < self.outer_radius_m) || !roots_fit {
                    return Err(PropulsionError::InvalidSpec(
                        "star tip radius must exceed every root radius and stay inside the grain"
                            .into(),
                    ));
                }
                let largest_root_m = self
                    .segment_core_radii_m
                    .as_deref()
                    .map_or(self.core_radius_m, |radii| {
                        radii.iter().copied().fold(0.0, f64::max)
                    });
                if tip_radius_m - largest_root_m < minimum_resolved_feature_m
                    || self.outer_radius_m - tip_radius_m < minimum_resolved_feature_m
                {
                    return Err(PropulsionError::InvalidSpec(
                        "star lobes and outer web must each span at least two burnback cells"
                            .into(),
                    ));
                }
            }
            SolidGrainGeometry::Finocyl {
                fin_count,
                fin_tip_radius_m,
                fin_width_rad,
            } => {
                if !(3..=16).contains(&fin_count) {
                    return Err(PropulsionError::InvalidSpec(
                        "finocyl fin count must be in 3..=16".into(),
                    ));
                }
                require_positive(fin_tip_radius_m, "finocyl fin-tip radius")?;
                require_positive(fin_width_rad, "finocyl fin width")?;
                let pitch_rad = std::f64::consts::TAU / f64::from(fin_count);
                if fin_width_rad >= 0.9 * pitch_rad {
                    return Err(PropulsionError::InvalidSpec(
                        "finocyl fin width must be less than 90% of its angular pitch".into(),
                    ));
                }
                let roots_fit = self
                    .segment_core_radii_m
                    .as_deref()
                    .unwrap_or(&[self.core_radius_m])
                    .iter()
                    .all(|root| root < &fin_tip_radius_m);
                if !(fin_tip_radius_m < self.outer_radius_m) || !roots_fit {
                    return Err(PropulsionError::InvalidSpec(
                        "fin tip radius must exceed every core radius and stay inside the grain"
                            .into(),
                    ));
                }
                let largest_root_m = self
                    .segment_core_radii_m
                    .as_deref()
                    .map_or(self.core_radius_m, |radii| {
                        radii.iter().copied().fold(0.0, f64::max)
                    });
                if fin_tip_radius_m - largest_root_m < minimum_resolved_feature_m
                    || self.outer_radius_m - fin_tip_radius_m < minimum_resolved_feature_m
                    || fin_tip_radius_m * fin_width_rad < minimum_resolved_feature_m
                {
                    return Err(PropulsionError::InvalidSpec(
                        "fin depth, tip web, and angular width must each span at least two burnback cells"
                            .into(),
                    ));
                }
            }
        }
        if self.ignition_shots == 0 {
            return Err(PropulsionError::InvalidSpec(
                "a solid motor with zero shots can never ignite".into(),
            ));
        }
        Ok(())
    }

    /// Hangar compile: solve the equilibrium burn trace over the web.
    pub fn compile(&self) -> Result<CompiledSolid, PropulsionError> {
        self.validate()?;
        let thermo = self.propellant.thermo();
        let c_star = characteristic_velocity(&thermo);
        let throat_area_m2 = std::f64::consts::PI * self.throat_radius_m * self.throat_radius_m;
        let exit_radius_m = self.throat_radius_m * self.expansion_ratio.sqrt();
        let exit = nozzle_exit(&thermo, self.expansion_ratio)?;
        let wall_slope = (exit_radius_m - self.throat_radius_m) / self.nozzle_length_m;
        let conical_divergence = 0.5 * (1.0 + wall_slope.atan().cos());
        let divergence = match self.contour {
            NozzleContour::Conical => conical_divergence,
            NozzleContour::Bell => conical_divergence + (1.0 - conical_divergence) * 0.5,
            NozzleContour::Aerospike => 0.99,
        };

        // Web-burn trace: time-stepped equilibrium. Each segment regresses
        // its own port (stepped channels shape boost-sustain traces); star
        // and finocyl profiles use a signed-distance burnback curve. The
        // common chamber pressure couples them through total burning area:
        // Pc = [Ab(t) a rho c* / At]^(1/(1-n)).
        let density = thermo.bulk_density_kg_m3;
        let core_radii: Vec<f64> = match &self.segment_core_radii_m {
            Some(radii) => radii.clone(),
            None => vec![self.core_radius_m; self.segments as usize],
        };
        let mut unique_curves: Vec<(u64, NumericalPortCurve)> = Vec::new();
        let port_curves: Vec<Option<NumericalPortCurve>> = core_radii
            .iter()
            .map(|root_radius_m| match self.grain_geometry {
                SolidGrainGeometry::Circular => None,
                geometry => {
                    let radius_key = root_radius_m.to_bits();
                    let curve = if let Some((_, curve)) =
                        unique_curves.iter().find(|(key, _)| *key == radius_key)
                    {
                        curve.clone()
                    } else {
                        let curve = NumericalPortCurve::compile(
                            geometry,
                            *root_radius_m,
                            self.outer_radius_m,
                        );
                        unique_curves.push((radius_key, curve.clone()));
                        curve
                    };
                    Some(curve)
                }
            })
            .collect();
        let webs: Vec<f64> = core_radii
            .iter()
            .enumerate()
            .map(|(index, core_m)| {
                port_curves[index]
                    .as_ref()
                    .map_or(self.outer_radius_m - core_m, |curve| curve.max_web_m)
            })
            .collect();
        let geometry_at = |burned: &[f64]| {
            burn_geometry_at(
                self.outer_radius_m,
                self.segment_length_m,
                self.inhibited_ends,
                &core_radii,
                &webs,
                burned,
                &port_curves,
            )
        };
        let pressure_at = |area_m2: f64| -> Result<f64, PropulsionError> {
            let chamber_pa = (area_m2 * self.burn_rate_coeff * density * c_star / throat_area_m2)
                .powf(1.0 / (1.0 - self.burn_rate_exponent));
            if !chamber_pa.is_finite() || chamber_pa <= 0.0 {
                return Err(PropulsionError::InvalidSpec(
                    "grain equilibrium pressure is non-physical".into(),
                ));
            }
            Ok(chamber_pa)
        };
        let point_at = |time_s: f64,
                        burned_now: &[f64],
                        burned_max_m: f64|
         -> Result<BurnPoint, PropulsionError> {
            let burn_geometry = geometry_at(burned_now);
            let chamber_pa = pressure_at(burn_geometry.burning_area_m2)?;
            let burn_rate_mps = self.burn_rate_coeff * chamber_pa.powf(self.burn_rate_exponent);
            let mass_flow_kg_s = density * burn_geometry.burning_area_m2 * burn_rate_mps;
            let thrust_sl_n = thrust_coefficient(
                &thermo,
                chamber_pa,
                &exit,
                self.expansion_ratio,
                101_325.0,
                divergence,
            ) * chamber_pa
                * throat_area_m2;
            let thrust_vac_n = thrust_coefficient(
                &thermo,
                chamber_pa,
                &exit,
                self.expansion_ratio,
                0.0,
                divergence,
            ) * chamber_pa
                * throat_area_m2;
            Ok(BurnPoint {
                time_s,
                web_burned_m: burned_max_m,
                burn_surface_area_m2: burn_geometry.burning_area_m2,
                port_area_m2: burn_geometry.port_area_m2,
                port_perimeter_m: burn_geometry.port_perimeter_m,
                chamber_pa,
                mass_flow_kg_s,
                thrust_sl_n,
                thrust_vac_n,
            })
        };
        let initial_area_m2 = geometry_at(&vec![0.0; core_radii.len()]).burning_area_m2;
        let initial_rate_mps =
            self.burn_rate_coeff * pressure_at(initial_area_m2)?.powf(self.burn_rate_exponent);
        let max_web_m = webs.iter().cloned().fold(0.0_f64, f64::max);
        let dt_s = max_web_m / initial_rate_mps / 400.0;
        if !dt_s.is_finite() || dt_s <= 0.0 {
            return Err(PropulsionError::InvalidSpec(
                "grain step sizing is non-physical".into(),
            ));
        }
        let mut burned = vec![0.0; core_radii.len()];
        let mut time_s = 0.0;
        let mut points = vec![point_at(0.0, &burned, 0.0)?];
        let mut peak_pressure_pa = 0.0_f64;
        let mut peak_thrust_n = 0.0_f64;
        loop {
            let burn_geometry = geometry_at(&burned);
            if burn_geometry.burning_area_m2 <= 0.0 {
                break;
            }
            let chamber_pa = pressure_at(burn_geometry.burning_area_m2)?;
            let burn_rate_mps = self.burn_rate_coeff * chamber_pa.powf(self.burn_rate_exponent);
            let mut all_done = true;
            let mut burned_max_m = 0.0_f64;
            for (index, web_m) in webs.iter().enumerate() {
                if burned[index] < *web_m {
                    burned[index] = (burned[index] + burn_rate_mps * dt_s).min(*web_m);
                }
                burned_max_m = burned_max_m.max(burned[index]);
                if burned[index] < *web_m {
                    all_done = false;
                }
            }
            time_s += dt_s;
            // Burn-through: the trace terminates at zero thrust rather
            // than erroring on the empty grain.
            let next_geometry = geometry_at(&burned);
            let point = if next_geometry.burning_area_m2 <= 0.0 {
                BurnPoint {
                    time_s,
                    web_burned_m: burned_max_m,
                    burn_surface_area_m2: 0.0,
                    port_area_m2: std::f64::consts::PI
                        * self.outer_radius_m
                        * self.outer_radius_m
                        * core_radii.len() as f64,
                    port_perimeter_m: 0.0,
                    chamber_pa: 0.0,
                    mass_flow_kg_s: 0.0,
                    thrust_sl_n: 0.0,
                    thrust_vac_n: 0.0,
                }
            } else {
                point_at(time_s, &burned, burned_max_m)?
            };
            peak_pressure_pa = peak_pressure_pa.max(point.chamber_pa);
            peak_thrust_n = peak_thrust_n.max(point.thrust_sl_n);
            points.push(point);
            if all_done {
                break;
            }
            if points.len() > 100_000 {
                return Err(PropulsionError::InvalidSpec(
                    "grain does not burn through".into(),
                ));
            }
        }
        peak_pressure_pa = peak_pressure_pa.max(points[0].chamber_pa);
        peak_thrust_n = peak_thrust_n.max(points[0].thrust_sl_n);
        // Integrate impulse and propellant over the recorded trace.
        let mut total_impulse_ns = 0.0;
        let mut propellant_kg = 0.0;
        for window in points.windows(2) {
            let (a, b) = (window[0], window[1]);
            let dt = b.time_s - a.time_s;
            total_impulse_ns += 0.5 * (a.thrust_vac_n + b.thrust_vac_n) * dt;
            propellant_kg += 0.5 * (a.mass_flow_kg_s + b.mass_flow_kg_s) * dt;
        }
        let burn_time_s = time_s;
        let avg_isp_s = total_impulse_ns / (propellant_kg * STANDARD_GRAVITY_MPS2);

        // Casing: cylinder + hemispherical-cap allowance + shared nozzle path.
        let grain_length_m = self.segment_length_m * self.segments as f64;
        let case_thickness_m = peak_pressure_pa * 2.0 * self.outer_radius_m
            / (2.0 * self.casing_material.yield_strength_pa)
            * PRESSURE_SAFETY_FACTOR;
        let case_kg = 2.0
            * std::f64::consts::PI
            * self.outer_radius_m
            * grain_length_m
            * case_thickness_m
            * self.casing_material.density_kg_m3
            * 1.3;
        let slant_m = (self.nozzle_length_m * self.nozzle_length_m
            + (exit_radius_m - self.throat_radius_m).powi(2))
        .sqrt();
        let mut nozzle_kg = std::f64::consts::PI
            * (self.throat_radius_m + exit_radius_m)
            * slant_m
            * case_thickness_m
            * 0.6
            * self.casing_material.density_kg_m3;
        if self.contour == NozzleContour::Bell {
            nozzle_kg *= 0.85;
        }
        if self.contour == NozzleContour::Aerospike {
            let spike_base_m = 0.6 * exit_radius_m;
            let spike_slant_m =
                (self.nozzle_length_m * self.nozzle_length_m + spike_base_m * spike_base_m).sqrt();
            nozzle_kg = nozzle_kg * 0.7
                + std::f64::consts::PI
                    * spike_base_m
                    * spike_slant_m
                    * case_thickness_m
                    * 0.6
                    * self.casing_material.density_kg_m3;
        }
        let gimbal_kg = if self.gimbal_range_rad > 0.0 {
            MASS_FIT_GIMBAL_BASE_KG + MASS_FIT_GIMBAL_KG_PER_N * peak_thrust_n
        } else {
            0.0
        };
        let dry_mass_kg = case_kg
            + nozzle_kg
            + gimbal_kg
            + MASS_FIT_SOLID_IGNITER_KG
            + MASS_FIT_MOUNT_KG_PER_N * peak_thrust_n
            + MASS_FIT_FEED_KG_PER_N * peak_thrust_n;

        Ok(CompiledSolid {
            name: self.name.clone(),
            propellant: self.propellant,
            throat_area_m2,
            throat_radius_m: self.throat_radius_m,
            expansion_ratio: self.expansion_ratio,
            exit_radius_m,
            divergence_factor: divergence,
            gamma: thermo.gamma,
            chamber_temp_k: thermo.chamber_temp_k,
            gas_constant_j_kg_k: thermo.gas_constant_j_kg_k,
            c_star_mps: c_star,
            exit_mach: exit.exit_mach,
            exit_temp_k: exit.exit_temp_k,
            exhaust_velocity_mps: exit.exhaust_velocity_mps,
            burn_curve: points,
            burn_time_s,
            total_impulse_ns,
            propellant_mass_kg: propellant_kg,
            avg_isp_s,
            peak_pressure_pa,
            peak_thrust_sl_n: peak_thrust_n,
            dry_mass_kg,
            grain_outer_radius_m: self.outer_radius_m,
            grain_length_m,
            grain_geometry: self.grain_geometry,
            ignition_shots: self.ignition_shots,
            gimbal_range_rad: self.gimbal_range_rad,
            contour: self.contour,
            aerospike_base_area_m2: match self.contour {
                NozzleContour::Aerospike => {
                    AEROSPIKE_BASE_FRACTION * throat_area_m2 * self.expansion_ratio
                }
                _ => 0.0,
            },
        })
    }
}

/// One web station of a compiled solid burn trace (sea-level + vacuum
/// thrust stored; altitude replay varies only the pressure term, which is
/// exact for a choked motor at frozen chamber pressure).
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct BurnPoint {
    pub time_s: f64,
    pub web_burned_m: f64,
    /// Total exposed propellant surface area driving equilibrium pressure.
    #[serde(default)]
    pub burn_surface_area_m2: f64,
    /// Sum of open port cross-sectional areas across the segments.
    #[serde(default)]
    pub port_area_m2: f64,
    /// Sum of open-port perimeters across the segments.
    #[serde(default)]
    pub port_perimeter_m: f64,
    pub chamber_pa: f64,
    pub mass_flow_kg_s: f64,
    pub thrust_sl_n: f64,
    pub thrust_vac_n: f64,
}

/// Hangar-compiled solid motor with its equilibrium burn trace.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct CompiledSolid {
    pub name: String,
    pub propellant: Propellant,
    pub throat_area_m2: f64,
    pub throat_radius_m: f64,
    pub expansion_ratio: f64,
    pub exit_radius_m: f64,
    pub divergence_factor: f64,
    pub gamma: f64,
    pub chamber_temp_k: f64,
    pub gas_constant_j_kg_k: f64,
    pub c_star_mps: f64,
    pub exit_mach: f64,
    pub exit_temp_k: f64,
    pub exhaust_velocity_mps: f64,
    pub burn_curve: Vec<BurnPoint>,
    pub burn_time_s: f64,
    pub total_impulse_ns: f64,
    pub propellant_mass_kg: f64,
    pub avg_isp_s: f64,
    pub peak_pressure_pa: f64,
    pub peak_thrust_sl_n: f64,
    pub dry_mass_kg: f64,
    pub grain_outer_radius_m: f64,
    pub grain_length_m: f64,
    /// Port profile used to compile the burn curve.
    #[serde(default)]
    pub grain_geometry: SolidGrainGeometry,
    pub ignition_shots: u32,
    pub gimbal_range_rad: f64,
    pub contour: NozzleContour,
    /// Aerospike base area exposed to ambient (m^2, 0 for bell/cone).
    pub aerospike_base_area_m2: f64,
}

impl CompiledSolid {
    pub(crate) fn thermo_ref(&self) -> PropellantThermo {
        PropellantThermo {
            gamma: self.gamma,
            chamber_temp_k: self.chamber_temp_k,
            gas_constant_j_kg_k: self.gas_constant_j_kg_k,
            bulk_density_kg_m3: 0.0,
            characteristic_length_m: 0.0,
            reference_c_star_mps: 0.0,
        }
    }

    /// Design-point exit pressure ratio (frozen nozzle geometry: the ratio
    /// depends only on gamma and expansion ratio, never on chamber pressure
    /// or ambient).
    pub(crate) fn exit_pressure_ratio(&self) -> f64 {
        (1.0 + (self.gamma - 1.0) / 2.0 * self.exit_mach * self.exit_mach)
            .powf(-self.gamma / (self.gamma - 1.0))
    }

    pub(crate) fn exit_state(&self) -> NozzleExitState {
        NozzleExitState {
            exit_mach: self.exit_mach,
            exit_pressure_ratio: self.exit_pressure_ratio(),
            exit_temp_k: self.exit_temp_k,
            exhaust_velocity_mps: self.exhaust_velocity_mps,
        }
    }

    pub(crate) fn interpolate(&self, burn_time_s: f64) -> BurnPoint {
        let curve = &self.burn_curve;
        if curve.is_empty() {
            return BurnPoint {
                time_s: 0.0,
                web_burned_m: 0.0,
                burn_surface_area_m2: 0.0,
                port_area_m2: 0.0,
                port_perimeter_m: 0.0,
                chamber_pa: 0.0,
                mass_flow_kg_s: 0.0,
                thrust_sl_n: 0.0,
                thrust_vac_n: 0.0,
            };
        }
        if burn_time_s <= 0.0 {
            return curve[0];
        }
        for window in curve.windows(2) {
            let (a, b) = (window[0], window[1]);
            if burn_time_s <= b.time_s {
                let span = (b.time_s - a.time_s).max(1e-12);
                let fraction = ((burn_time_s - a.time_s) / span).clamp(0.0, 1.0);
                return BurnPoint {
                    time_s: burn_time_s,
                    web_burned_m: a.web_burned_m + (b.web_burned_m - a.web_burned_m) * fraction,
                    burn_surface_area_m2: a.burn_surface_area_m2
                        + (b.burn_surface_area_m2 - a.burn_surface_area_m2) * fraction,
                    port_area_m2: a.port_area_m2 + (b.port_area_m2 - a.port_area_m2) * fraction,
                    port_perimeter_m: a.port_perimeter_m
                        + (b.port_perimeter_m - a.port_perimeter_m) * fraction,
                    chamber_pa: a.chamber_pa + (b.chamber_pa - a.chamber_pa) * fraction,
                    mass_flow_kg_s: a.mass_flow_kg_s
                        + (b.mass_flow_kg_s - a.mass_flow_kg_s) * fraction,
                    thrust_sl_n: a.thrust_sl_n + (b.thrust_sl_n - a.thrust_sl_n) * fraction,
                    thrust_vac_n: a.thrust_vac_n + (b.thrust_vac_n - a.thrust_vac_n) * fraction,
                };
            }
        }
        *curve.last().expect("non-empty burn curve")
    }
}

#[cfg(test)]
mod tests {
    use super::super::{CompiledEngine, STANDARD_GRAVITY_MPS2};
    use super::*;

    #[test]
    fn solid_burn_trace_is_physical() {
        // Circular BATES: a cylindrical port opens as it burns, so the trace
        // is inherently progressive. This pins that signature, impulse/mass/
        // Isp consistency, and choked altitude replay.
        let (a, n) = SolidMotorSpec::apcp_ballistics();
        let spec = SolidMotorSpec {
            name: "APCP booster".into(),
            propellant: Propellant::SolidApcp,
            outer_radius_m: 0.5,
            core_radius_m: 0.32,
            grain_geometry: SolidGrainGeometry::Circular,
            segment_length_m: 1.5,
            segments: 4,
            burn_rate_coeff: a,
            burn_rate_exponent: n,
            throat_radius_m: 0.12,
            expansion_ratio: 10.0,
            nozzle_length_m: 0.9,
            contour: NozzleContour::Conical,
            casing_material: ChamberMaterial::nickel_superalloy(),
            inhibited_ends: true,
            segment_core_radii_m: None,
            gimbal_range_rad: 0.0,
            ignition_shots: 1,
        };
        let motor = spec.compile().expect("solid compiles");
        assert!(motor.burn_time_s > 1.0 && motor.burn_time_s < 60.0);
        // Time-stepped trace: resolved (hundreds of stations) with
        // strictly increasing time.
        assert!(motor.burn_curve.len() >= 100);
        for window in motor.burn_curve.windows(2) {
            assert!(window[1].time_s > window[0].time_s);
        }
        // Progressive signature: vacuum thrust never decreases along the
        // web (the terminal point is the zero-thrust burn-through).
        let live = &motor.burn_curve[..motor.burn_curve.len() - 1];
        for window in live.windows(2) {
            assert!(
                window[1].thrust_vac_n >= window[0].thrust_vac_n,
                "cylindrical-port trace must be progressive"
            );
        }
        let mean_vac = motor.total_impulse_ns / motor.burn_time_s;
        let peak_vac = motor
            .burn_curve
            .iter()
            .map(|point| point.thrust_vac_n)
            .fold(0.0_f64, f64::max);
        assert!(
            peak_vac / mean_vac < 1.8,
            "chunky-port progressivity must stay bounded"
        );
        // Impulse = Isp * m_prop * g0 (internal consistency).
        let check = motor.avg_isp_s * motor.propellant_mass_kg * STANDARD_GRAVITY_MPS2;
        assert!((check - motor.total_impulse_ns).abs() / motor.total_impulse_ns < 1e-9);
        // Chamber pressure frozen vs ambient: SL and vacuum evaluation at
        // mid-burn differ only by the (Pe - Pa) Ae term.
        let engine = CompiledEngine::Solid(motor);
        let mid = engine
            .operating_point(1.0, 101_325.0, 1.0)
            .expect("mid-burn SL");
        let mid_vac = engine.operating_point(1.0, 0.0, 1.0).expect("mid-burn vac");
        assert!((mid.mass_flow_kg_s - mid_vac.mass_flow_kg_s).abs() / mid.mass_flow_kg_s < 1e-12);
        assert!(engine.operating_point(0.5, 0.0, 1.0).is_err());
        // Past burnout: zero thrust, no error.
        let done = engine.operating_point(1.0, 0.0, 1.0e6).expect("burned out");
        assert_eq!(done.thrust_n, 0.0);
    }

    #[test]
    fn stepped_ports_shape_boost_sustain() {
        // Stepped channel: a thin-web segment burns out early (boost) and
        // the thick-web segment sustains. The test pins the two-phase
        // signature plus integrated-vs-geometric propellant agreement.
        let (a, n) = SolidMotorSpec::apcp_ballistics();
        let spec = SolidMotorSpec {
            name: "boost-sustain".into(),
            propellant: Propellant::SolidApcp,
            outer_radius_m: 0.5,
            core_radius_m: 0.15,
            grain_geometry: SolidGrainGeometry::Circular,
            segment_length_m: 1.5,
            segments: 2,
            burn_rate_coeff: a,
            burn_rate_exponent: n,
            throat_radius_m: 0.12,
            expansion_ratio: 10.0,
            nozzle_length_m: 0.9,
            contour: NozzleContour::Conical,
            casing_material: ChamberMaterial::nickel_superalloy(),
            inhibited_ends: true,
            segment_core_radii_m: Some(vec![0.15, 0.35]),
            gimbal_range_rad: 0.0,
            ignition_shots: 1,
        };
        let motor = spec.compile().expect("stepped grain compiles");
        let engine = CompiledEngine::Solid(motor.clone());
        let peak = motor
            .burn_curve
            .iter()
            .map(|point| point.thrust_vac_n)
            .fold(0.0_f64, f64::max);
        let peak_time = motor
            .burn_curve
            .iter()
            .find(|point| point.thrust_vac_n >= peak * (1.0 - 1e-9))
            .expect("peak station")
            .time_s;
        assert!(
            peak_time < 0.4 * motor.burn_time_s,
            "boost peak must sit in the first 40% of the burn"
        );
        let late = engine
            .operating_point(1.0, 0.0, 0.85 * motor.burn_time_s)
            .expect("late sustain")
            .thrust_n;
        assert!(
            late < 0.55 * peak,
            "sustain phase must drop below 55% of boost peak"
        );
        // Integrated propellant agrees with the geometric grain mass.
        let geometric: f64 = [0.15, 0.35]
            .iter()
            .map(|core_m| std::f64::consts::PI * (0.5 * 0.5 - core_m * core_m) * 1.5 * 1770.0)
            .sum();
        let drift = (motor.propellant_mass_kg - geometric).abs() / geometric;
        assert!(drift < 0.02, "propellant drift {drift:e} too large");
        // Port-count mismatch is refused, not silently broadcast.
        let bad = SolidMotorSpec {
            segment_core_radii_m: Some(vec![0.2]),
            ..spec
        };
        assert!(bad.compile().is_err());
    }

    #[test]
    fn unstable_burn_exponent_is_rejected() {
        let (a, _) = SolidMotorSpec::apcp_ballistics();
        let spec = SolidMotorSpec {
            name: "unstable".into(),
            propellant: Propellant::SolidApcp,
            outer_radius_m: 0.5,
            core_radius_m: 0.15,
            grain_geometry: SolidGrainGeometry::Circular,
            segment_length_m: 1.5,
            segments: 2,
            burn_rate_coeff: a,
            burn_rate_exponent: 1.2,
            throat_radius_m: 0.12,
            expansion_ratio: 8.0,
            nozzle_length_m: 0.8,
            contour: NozzleContour::Conical,
            casing_material: ChamberMaterial::nickel_superalloy(),
            inhibited_ends: true,
            segment_core_radii_m: None,
            gimbal_range_rad: 0.0,
            ignition_shots: 1,
        };
        assert!(spec.compile().is_err());
    }

    fn polygon_area_and_perimeter(vertices: &[[f64; 2]]) -> (f64, f64) {
        let mut twice_area = 0.0;
        let mut perimeter = 0.0;
        for index in 0..vertices.len() {
            let a = vertices[index];
            let b = vertices[(index + 1) % vertices.len()];
            twice_area += a[0] * b[1] - b[0] * a[1];
            perimeter += ((b[0] - a[0]).powi(2) + (b[1] - a[1]).powi(2)).sqrt();
        }
        (0.5 * twice_area.abs(), perimeter)
    }

    fn shaped_grain_spec(grain_geometry: SolidGrainGeometry) -> SolidMotorSpec {
        let (burn_rate_coeff, burn_rate_exponent) = SolidMotorSpec::apcp_ballistics();
        SolidMotorSpec {
            name: "shaped-grain".into(),
            propellant: Propellant::SolidApcp,
            outer_radius_m: 0.5,
            core_radius_m: 0.16,
            grain_geometry,
            segment_length_m: 1.0,
            segments: 2,
            burn_rate_coeff,
            burn_rate_exponent,
            throat_radius_m: 0.12,
            expansion_ratio: 10.0,
            nozzle_length_m: 0.9,
            contour: NozzleContour::Conical,
            casing_material: ChamberMaterial::nickel_superalloy(),
            inhibited_ends: true,
            segment_core_radii_m: None,
            gimbal_range_rad: 0.0,
            ignition_shots: 1,
        }
    }

    #[test]
    fn star_and_finocyl_ports_burn_back_with_bounded_geometry_error() {
        let profiles = [
            SolidGrainGeometry::Star {
                tip_count: 6,
                tip_radius_m: 0.30,
            },
            SolidGrainGeometry::Finocyl {
                fin_count: 8,
                fin_tip_radius_m: 0.32,
                fin_width_rad: 0.24,
            },
        ];
        let grain_area_m2 = std::f64::consts::PI * 0.5_f64.powi(2);
        for profile in profiles {
            let spec = shaped_grain_spec(profile);
            let polygon = port_polygon(profile, spec.core_radius_m);
            let (exact_port_area_m2, exact_port_perimeter_m) = polygon_area_and_perimeter(&polygon);
            let motor = spec.compile().expect("shaped grain compiles");
            assert_eq!(motor.grain_geometry, profile);
            assert!(motor.burn_curve.len() >= 100);

            let initial = motor.burn_curve.first().expect("initial station");
            let area_error = (initial.port_area_m2 - exact_port_area_m2 * 2.0).abs()
                / (exact_port_area_m2 * 2.0);
            let perimeter_error = (initial.port_perimeter_m - exact_port_perimeter_m * 2.0).abs()
                / (exact_port_perimeter_m * 2.0);
            assert!(
                area_error < 0.025,
                "port area raster error {:.3}%",
                area_error * 100.0
            );
            assert!(
                perimeter_error < 0.08,
                "Crofton perimeter error {:.3}%",
                perimeter_error * 100.0
            );
            assert!(initial.burn_surface_area_m2 > 0.0);
            assert!(
                initial.port_perimeter_m
                    > std::f64::consts::TAU * spec.core_radius_m * spec.segments as f64
            );

            for window in motor.burn_curve.windows(2) {
                assert!(window[1].time_s > window[0].time_s);
                assert!(window[1].port_area_m2 + 1e-12 >= window[0].port_area_m2);
                assert!(window[1].burn_surface_area_m2.is_finite());
            }
            let final_point = motor.burn_curve.last().expect("burnout station");
            assert_eq!(final_point.burn_surface_area_m2, 0.0);
            assert_eq!(final_point.port_perimeter_m, 0.0);
            assert!(
                (final_point.port_area_m2 - grain_area_m2 * 2.0).abs() / (grain_area_m2 * 2.0)
                    < 1e-12
            );

            let density_kg_m3 = spec.propellant.thermo().bulk_density_kg_m3;
            let geometric_propellant_kg =
                (grain_area_m2 - exact_port_area_m2) * spec.segment_length_m * 2.0 * density_kg_m3;
            let propellant_error = (motor.propellant_mass_kg - geometric_propellant_kg).abs()
                / geometric_propellant_kg;
            assert!(
                propellant_error < 0.07,
                "integrated propellant error {:.3}%",
                propellant_error * 100.0
            );
            let impulse_check = motor.avg_isp_s * motor.propellant_mass_kg * STANDARD_GRAVITY_MPS2;
            assert!((impulse_check - motor.total_impulse_ns).abs() / motor.total_impulse_ns < 1e-9);

            if matches!(profile, SolidGrainGeometry::Star { .. }) {
                let end_burning = SolidMotorSpec {
                    inhibited_ends: false,
                    ..spec.clone()
                }
                .compile()
                .expect("exposed grain ends compile");
                let initial = &end_burning.burn_curve[0];
                let expected_area = initial.port_perimeter_m * spec.segment_length_m
                    + 2.0 * (grain_area_m2 * spec.segments as f64 - initial.port_area_m2);
                assert!((initial.burn_surface_area_m2 - expected_area).abs() < 1e-12);
            }
        }
    }

    #[test]
    fn shaped_grains_reject_invalid_lobe_geometry() {
        let star = shaped_grain_spec(SolidGrainGeometry::Star {
            tip_count: 6,
            tip_radius_m: 0.16,
        });
        assert!(star.compile().is_err(), "tip must extend beyond the root");
        let under_resolved_star = shaped_grain_spec(SolidGrainGeometry::Star {
            tip_count: 6,
            tip_radius_m: 0.17,
        });
        assert!(
            under_resolved_star.compile().is_err(),
            "lobe depth must resolve to two cells"
        );
        let finocyl = shaped_grain_spec(SolidGrainGeometry::Finocyl {
            fin_count: 8,
            fin_tip_radius_m: 0.32,
            fin_width_rad: 1.0,
        });
        assert!(finocyl.compile().is_err(), "fin width must fit its pitch");
        let under_resolved_fin = shaped_grain_spec(SolidGrainGeometry::Finocyl {
            fin_count: 8,
            fin_tip_radius_m: 0.32,
            fin_width_rad: 0.01,
        });
        assert!(
            under_resolved_fin.compile().is_err(),
            "fin width must resolve to two cells"
        );
    }
}
