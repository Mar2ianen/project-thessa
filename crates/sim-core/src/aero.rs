use std::io::BufRead;
use std::{error::Error, fmt};

use glam::DVec3;
use rayon::prelude::*;
use serde::{Deserialize, Serialize};

const EPS_SPEED_MPS: f64 = 1.0e-9;

/// Freestream properties expressed in the vehicle body frame.
///
/// Body axes are `+X` forward, `+Y` right and `+Z` up. All values are SI and
/// authoritative `f64`; a renderer may down-convert the resulting forces or
/// debug vectors separately.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct AeroEnvironment {
    pub density_kg_m3: f64,
    pub speed_of_sound_mps: f64,
    pub dynamic_viscosity_pa_s: f64,
    pub wind_velocity_body_mps: DVec3,
}

impl AeroEnvironment {
    pub const fn new(
        density_kg_m3: f64,
        speed_of_sound_mps: f64,
        dynamic_viscosity_pa_s: f64,
        wind_velocity_body_mps: DVec3,
    ) -> Self {
        Self {
            density_kg_m3,
            speed_of_sound_mps,
            dynamic_viscosity_pa_s,
            wind_velocity_body_mps,
        }
    }

    /// ISA-like sea-level reference used by unit tests and the first lab.
    pub const fn standard_sea_level() -> Self {
        Self::new(1.225, 340.294, 1.81e-5, DVec3::ZERO)
    }

    fn validate(self) -> Result<(), AeroError> {
        if !self.density_kg_m3.is_finite() || self.density_kg_m3 < 0.0 {
            return Err(AeroError::InvalidEnvironment(
                "density must be finite and non-negative".into(),
            ));
        }
        if !self.speed_of_sound_mps.is_finite() || self.speed_of_sound_mps <= 0.0 {
            return Err(AeroError::InvalidEnvironment(
                "speed of sound must be positive and finite".into(),
            ));
        }
        if !self.dynamic_viscosity_pa_s.is_finite() || self.dynamic_viscosity_pa_s < 0.0 {
            return Err(AeroError::InvalidEnvironment(
                "dynamic viscosity must be finite and non-negative".into(),
            ));
        }
        if !self.wind_velocity_body_mps.is_finite() {
            return Err(AeroError::InvalidEnvironment(
                "wind velocity must be finite".into(),
            ));
        }
        Ok(())
    }
}

/// Translational and angular state sampled by the aerodynamic solver.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct AeroState {
    pub velocity_body_mps: DVec3,
    pub angular_velocity_body_rps: DVec3,
}

impl AeroState {
    pub const fn new(velocity_body_mps: DVec3, angular_velocity_body_rps: DVec3) -> Self {
        Self {
            velocity_body_mps,
            angular_velocity_body_rps,
        }
    }

    fn validate(self) -> Result<(), AeroError> {
        if !self.velocity_body_mps.is_finite() {
            return Err(AeroError::InvalidState(
                "vehicle velocity must be finite".into(),
            ));
        }
        if !self.angular_velocity_body_rps.is_finite() {
            return Err(AeroError::InvalidState(
                "vehicle angular velocity must be finite".into(),
            ));
        }
        Ok(())
    }
}

/// One local aerodynamic zone. The panel is intentionally a force primitive,
/// not a mesh triangle: a vehicle compiler can aggregate render geometry into
/// a small number of zones without changing the runtime equations.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct AeroPanel {
    pub position_body_m: DVec3,
    /// Point at which the integrated panel force is applied. It defaults to
    /// `position_body_m`, but should be set to a geometry-derived center of
    /// pressure when the panel is large enough for the moment arm to matter.
    pub center_of_pressure_body_m: DVec3,
    pub chord_axis_body: DVec3,
    pub lift_axis_body: DVec3,
    pub area_m2: f64,
    pub chord_m: f64,
    pub span_m: f64,
    /// Effective planform aspect ratio used by the finite-surface lift model.
    /// The default constructor assumes a rectangular panel. Vehicle
    /// compilers should provide the actual value for tapered or body-mounted
    /// surfaces with [`AeroPanel::with_planform`].
    pub planform_aspect_ratio: f64,
    /// Mean aerodynamic sweep angle used by the Diederich planform
    /// correlation.
    pub planform_sweep_rad: f64,
    /// Lift interference multiplier for a surface mounted near a body. A
    /// value of `1` is isolated-flow; rocket fin sets can use a documented
    /// Barrowman/body-interference value.
    pub lift_interference_factor: f64,
    /// Sign/authority multiplier for lift on surfaces such as an inverted
    /// horizontal tail. `1` is conventional; `-1` produces restoring lift.
    pub lift_coefficient_sign: f64,
    /// Maximum thickness divided by chord. This is used only by the
    /// supersonic thin-airfoil wave-drag term; `0` keeps a mathematical flat
    /// plate and is the default for legacy panel definitions.
    pub thickness_to_chord_ratio: f64,
    pub control_deflection_rad: f64,
    /// Fraction of the panel exposed to the incoming flow after an optional
    /// occlusion/wake query. `1` is fully exposed and `0` contributes nothing.
    pub exposure: f64,
}

impl AeroPanel {
    pub fn new(
        position_body_m: DVec3,
        chord_axis_body: DVec3,
        lift_axis_body: DVec3,
        area_m2: f64,
        chord_m: f64,
    ) -> Result<Self, AeroError> {
        let chord_axis_body = normalize_axis(chord_axis_body, "chord axis")?;
        let lift_axis_body = orthogonal_axis(lift_axis_body, chord_axis_body, "lift axis")?;
        if !position_body_m.is_finite() {
            return Err(AeroError::InvalidGeometry(
                "panel position must be finite".into(),
            ));
        }
        if !area_m2.is_finite() || area_m2 <= 0.0 {
            return Err(AeroError::InvalidGeometry(
                "panel area must be positive and finite".into(),
            ));
        }
        if !chord_m.is_finite() || chord_m <= 0.0 {
            return Err(AeroError::InvalidGeometry(
                "panel chord must be positive and finite".into(),
            ));
        }
        Ok(Self {
            position_body_m,
            center_of_pressure_body_m: position_body_m,
            chord_axis_body,
            lift_axis_body,
            area_m2,
            chord_m,
            span_m: area_m2 / chord_m,
            planform_aspect_ratio: area_m2 / chord_m.powi(2),
            planform_sweep_rad: 0.0,
            lift_interference_factor: 1.0,
            lift_coefficient_sign: 1.0,
            thickness_to_chord_ratio: 0.0,
            control_deflection_rad: 0.0,
            exposure: 1.0,
        })
    }

    /// Override the geometric span/aspect ratio for a non-rectangular or
    /// body-mounted surface. `aspect_ratio` is the effective ratio used by
    /// the finite-planform correlation and may therefore differ from the
    /// textbook rectangular `span² / area` value.
    pub fn with_planform(
        mut self,
        span_m: f64,
        aspect_ratio: f64,
        sweep_rad: f64,
        lift_interference_factor: f64,
    ) -> Result<Self, AeroError> {
        self.span_m = span_m;
        self.planform_aspect_ratio = aspect_ratio;
        self.planform_sweep_rad = sweep_rad;
        self.lift_interference_factor = lift_interference_factor;
        self.validate()?;
        Ok(self)
    }

    /// Override only the span while retaining a rectangular planform model.
    pub fn with_span(self, span_m: f64) -> Result<Self, AeroError> {
        let aspect_ratio = span_m.powi(2) / self.area_m2;
        self.with_planform(
            span_m,
            aspect_ratio,
            self.planform_sweep_rad,
            self.lift_interference_factor,
        )
    }

    /// Set the force application point independently from the flow sample
    /// point. This is useful for tapered fins and imported coefficient
    /// surfaces whose center of pressure is not at their zone origin.
    pub fn with_center_of_pressure(
        mut self,
        center_of_pressure_body_m: DVec3,
    ) -> Result<Self, AeroError> {
        self.center_of_pressure_body_m = center_of_pressure_body_m;
        self.validate()?;
        Ok(self)
    }

    /// Set signed lift authority while retaining the common body AoA
    /// convention. Stabilizers commonly use `-1` here.
    pub fn with_lift_sign(mut self, lift_coefficient_sign: f64) -> Result<Self, AeroError> {
        self.lift_coefficient_sign = lift_coefficient_sign;
        self.validate()?;
        Ok(self)
    }

    /// Set the maximum thickness-to-chord ratio used by the supersonic
    /// thickness wave-drag approximation. A symmetric double-wedge proxy
    /// uses this together with angle of attack to estimate zero-lift and
    /// lift-dependent wave drag.
    pub fn with_thickness_ratio(
        mut self,
        thickness_to_chord_ratio: f64,
    ) -> Result<Self, AeroError> {
        self.thickness_to_chord_ratio = thickness_to_chord_ratio;
        self.validate()?;
        Ok(self)
    }

    /// A convenient symmetric flat-plate zone for tests and early vehicle
    /// prototypes. Its normal/lift axis is `+Z`.
    pub fn flat_plate(
        position_body_m: DVec3,
        area_m2: f64,
        chord_m: f64,
    ) -> Result<Self, AeroError> {
        Self::new(position_body_m, DVec3::X, DVec3::Z, area_m2, chord_m)
    }

    fn validate(self) -> Result<(), AeroError> {
        if !self.position_body_m.is_finite()
            || !self.center_of_pressure_body_m.is_finite()
            || !self.chord_axis_body.is_finite()
            || !self.lift_axis_body.is_finite()
        {
            return Err(AeroError::InvalidGeometry(
                "panel vectors must be finite".into(),
            ));
        }
        if !self.area_m2.is_finite()
            || self.area_m2 <= 0.0
            || !self.chord_m.is_finite()
            || self.chord_m <= 0.0
            || !self.span_m.is_finite()
            || self.span_m <= 0.0
            || !self.planform_aspect_ratio.is_finite()
            || self.planform_aspect_ratio <= 0.0
            || !self.planform_sweep_rad.is_finite()
            || self.planform_sweep_rad.abs() >= std::f64::consts::FRAC_PI_2
            || !self.lift_interference_factor.is_finite()
            || self.lift_interference_factor < 0.0
            || !self.lift_coefficient_sign.is_finite()
            || self.lift_coefficient_sign.abs() > 1.0
            || self.lift_coefficient_sign.abs() <= f64::EPSILON
            || !self.thickness_to_chord_ratio.is_finite()
            || self.thickness_to_chord_ratio < 0.0
            || self.thickness_to_chord_ratio > 0.5
        {
            return Err(AeroError::InvalidGeometry(
                "panel dimensions and planform parameters must be valid and finite".into(),
            ));
        }
        if !self.control_deflection_rad.is_finite() {
            return Err(AeroError::InvalidGeometry(
                "control deflection must be finite".into(),
            ));
        }
        if !self.exposure.is_finite() || !(0.0..=1.0).contains(&self.exposure) {
            return Err(AeroError::InvalidGeometry(
                "panel exposure must be in [0, 1]".into(),
            ));
        }
        let chord = normalize_axis(self.chord_axis_body, "chord axis")?;
        orthogonal_axis(self.lift_axis_body, chord, "lift axis")?;
        Ok(())
    }
}

#[derive(Debug, Clone, Default, PartialEq, Serialize, Deserialize)]
pub struct AeroGeometry {
    pub panels: Vec<AeroPanel>,
}

impl AeroGeometry {
    pub fn new(panels: Vec<AeroPanel>) -> Result<Self, AeroError> {
        if panels.is_empty() {
            return Err(AeroError::InvalidGeometry(
                "aero geometry needs at least one panel".into(),
            ));
        }
        let geometry = Self { panels };
        geometry.validate()?;
        Ok(geometry)
    }

    pub fn validate(&self) -> Result<(), AeroError> {
        for panel in &self.panels {
            panel.validate()?;
        }
        Ok(())
    }
}

/// Analytic panel-model knobs. This is a compact engineering model rather
/// than CFD: it captures local flow, stall smoothing, induced drag,
/// compressibility, transonic drag rise and a thin-body supersonic trend while
/// keeping evaluation O(number of panels).
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct AeroConfig {
    pub lift_slope_per_rad: f64,
    pub control_effectiveness: f64,
    pub zero_lift_angle_rad: f64,
    pub stall_angle_rad: f64,
    pub max_lift_coefficient: f64,
    pub base_drag_coefficient: f64,
    pub induced_drag_factor: f64,
    pub wave_drag_coefficient: f64,
    pub side_force_slope_per_rad: f64,
    /// Static pitching-moment coefficient about the panel's local side axis.
    /// This is kept separate from the center-of-pressure moment arm so an
    /// imported aircraft polar can represent tail/body moments at zero lift.
    pub pitching_moment_coefficient: f64,
    /// Dynamic roll damping derivative per non-dimensional roll rate.
    pub roll_damping_coefficient: f64,
    /// Dynamic pitch damping derivative per non-dimensional pitch rate.
    pub pitch_damping_coefficient: f64,
    /// Dynamic yaw damping derivative per non-dimensional yaw rate.
    pub yaw_damping_coefficient: f64,
    pub supersonic_lift_slope_factor: f64,
    /// Multiplier for the linearized supersonic wave-drag approximation.
    /// Keep it separate from lift slope so aircraft thickness/drag calibration
    /// does not silently change stability derivatives.
    pub supersonic_wave_drag_factor: f64,
    /// Minimum Prandtl-Glauert beta retained through the transonic plateau.
    /// The default follows the Barrowman/RocketPy convention of freezing the
    /// subsonic correction at M=0.8 until the supersonic branch takes over.
    pub transonic_beta_floor: f64,
}

impl Default for AeroConfig {
    fn default() -> Self {
        Self {
            lift_slope_per_rad: 2.0 * std::f64::consts::PI,
            control_effectiveness: 0.65,
            zero_lift_angle_rad: 0.0,
            stall_angle_rad: 18.0_f64.to_radians(),
            max_lift_coefficient: 1.35,
            base_drag_coefficient: 0.025,
            induced_drag_factor: 0.085,
            wave_drag_coefficient: 0.16,
            side_force_slope_per_rad: 0.9,
            pitching_moment_coefficient: 0.0,
            roll_damping_coefficient: 0.0,
            pitch_damping_coefficient: 0.0,
            yaw_damping_coefficient: 0.0,
            supersonic_lift_slope_factor: 4.0,
            supersonic_wave_drag_factor: 1.0,
            transonic_beta_floor: 0.6,
        }
    }
}

impl AeroConfig {
    fn validate(self) -> Result<(), AeroError> {
        let finite = [
            self.lift_slope_per_rad,
            self.control_effectiveness,
            self.zero_lift_angle_rad,
            self.stall_angle_rad,
            self.max_lift_coefficient,
            self.base_drag_coefficient,
            self.induced_drag_factor,
            self.wave_drag_coefficient,
            self.side_force_slope_per_rad,
            self.pitching_moment_coefficient,
            self.roll_damping_coefficient,
            self.pitch_damping_coefficient,
            self.yaw_damping_coefficient,
            self.supersonic_lift_slope_factor,
            self.supersonic_wave_drag_factor,
            self.transonic_beta_floor,
        ];
        if finite.iter().any(|value| !value.is_finite()) {
            return Err(AeroError::InvalidModel(
                "aero configuration contains a non-finite value".into(),
            ));
        }
        if self.lift_slope_per_rad <= 0.0
            || self.control_effectiveness < 0.0
            || self.stall_angle_rad <= 0.0
            || self.max_lift_coefficient <= 0.0
            || self.base_drag_coefficient < 0.0
            || self.induced_drag_factor < 0.0
            || self.wave_drag_coefficient < 0.0
            || self.side_force_slope_per_rad < 0.0
            || self.supersonic_lift_slope_factor <= 0.0
            || self.supersonic_wave_drag_factor < 0.0
            || self.transonic_beta_floor <= 0.0
            || self.transonic_beta_floor > 1.0
        {
            return Err(AeroError::InvalidModel(
                "aero configuration has an invalid range".into(),
            ));
        }
        Ok(())
    }

    fn analytic_coefficients(
        self,
        mach: f64,
        alpha_rad: f64,
        beta_rad: f64,
        control_deflection_rad: f64,
        panel: &AeroPanel,
    ) -> AeroCoefficients {
        let control_alpha = alpha_rad + self.control_effectiveness * control_deflection_rad;
        let alpha_eff = control_alpha - self.zero_lift_angle_rad;
        let mach = mach.max(0.0);
        // For a swept surface the shock-forming component is the velocity
        // normal to the leading edge. Below M=1 retain the conventional
        // freestream correction; above it, use the normal Mach number for the
        // supersonic branch and leave highly swept surfaces finite when their
        // normal flow is still subsonic.
        let normal_mach = if mach > 1.0 {
            mach * panel.planform_sweep_rad.cos()
        } else {
            mach
        };
        let subsonic_slope = self.lift_slope_per_rad
            / (1.0 - mach * mach)
                .max(self.transonic_beta_floor.powi(2))
                .sqrt();
        let transonic_slope = self.lift_slope_per_rad / self.transonic_beta_floor;
        let supersonic_slope = if normal_mach > 1.0 {
            (self.supersonic_lift_slope_factor / (normal_mach * normal_mach - 1.0).max(0.05).sqrt())
                .min(self.lift_slope_per_rad * 2.5)
        } else {
            transonic_slope
        };
        let compressible_lift_slope = if normal_mach <= 0.80 {
            subsonic_slope
        } else if normal_mach <= 1.0 {
            transonic_slope
        } else if normal_mach >= 1.20 {
            supersonic_slope
        } else {
            lerp(
                transonic_slope,
                supersonic_slope,
                smoothstep(1.0, 1.20, normal_mach),
            )
        };
        let lift_slope = finite_planform_lift_slope(compressible_lift_slope, panel);

        // Preserve a continuous lift curve past the stall angle and let the
        // tanh saturation below provide the final bounded CL. This is a
        // reduced-order post-stall trend, not a claim of separated-flow CFD.
        let alpha_abs = alpha_eff.abs();
        let post_stall_scale = if alpha_abs <= self.stall_angle_rad {
            1.0
        } else {
            (self.stall_angle_rad / alpha_abs).powf(0.35)
        };
        let linear_lift = lift_slope * alpha_eff * post_stall_scale;
        let cl = self.max_lift_coefficient * (linear_lift / self.max_lift_coefficient).tanh();
        let reynolds_independent_drag =
            self.base_drag_coefficient + self.induced_drag_factor * cl * cl;

        let transonic_rise = smoothstep(0.78, 1.18, mach);
        // Linearized supersonic thin-airfoil theory gives the same pressure
        // slope used above and a wave-drag term Cdw = 4 alpha^2 / beta.
        // Blend it in over M=1..1.2 so the realtime model stays finite and
        // does not introduce an artificial drag discontinuity at sonic speed.
        let supersonic_wave_drag = if normal_mach > 1.0 {
            let sweep_cos = panel.planform_sweep_rad.cos();
            let normal_alpha = alpha_eff * sweep_cos;
            let beta = (normal_mach * normal_mach - 1.0).max(0.05).sqrt();
            self.supersonic_wave_drag_factor
                * 4.0
                * (normal_alpha * normal_alpha + panel.thickness_to_chord_ratio.powi(2))
                / beta
        } else {
            0.0
        };
        let wave_drag = self.wave_drag_coefficient
            * (transonic_rise * (0.12 + 1.5 * alpha_eff * alpha_eff))
            + smoothstep(1.0, 1.20, normal_mach) * supersonic_wave_drag;

        AeroCoefficients {
            lift: cl,
            drag: reynolds_independent_drag + wave_drag,
            side_force: self.side_force_slope_per_rad * beta_rad,
            pitching_moment: self.pitching_moment_coefficient,
        }
    }
}

/// Dimensionless panel coefficients. Positive drag is converted into a force
/// opposite the local relative velocity; positive lift follows the panel's
/// configured lift axis projection.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct AeroCoefficients {
    pub lift: f64,
    pub drag: f64,
    pub side_force: f64,
    /// Dimensionless moment about the panel's local side axis. Positive is
    /// the right-handed rotation around `lift_axis × chord_axis`.
    pub pitching_moment: f64,
}

/// Small bilinear Mach/AoA table for coefficients generated by an offline
/// solver. Runtime interpolation is constant-time and clamps outside the
/// supplied domain, which makes imported VLM/CFD data safe for game flight.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct AeroCoefficientTable {
    pub mach_grid: Vec<f64>,
    pub alpha_grid_rad: Vec<f64>,
    pub samples: Vec<AeroCoefficients>,
}

impl AeroCoefficientTable {
    pub fn new(
        mach_grid: Vec<f64>,
        alpha_grid_rad: Vec<f64>,
        samples: Vec<AeroCoefficients>,
    ) -> Result<Self, AeroError> {
        if mach_grid.is_empty() || alpha_grid_rad.is_empty() {
            return Err(AeroError::InvalidModel(
                "coefficient table needs non-empty Mach and alpha grids".into(),
            ));
        }
        if samples.len() != mach_grid.len() * alpha_grid_rad.len() {
            return Err(AeroError::InvalidModel(
                "coefficient table sample count does not match its grids".into(),
            ));
        }
        validate_grid(&mach_grid, "Mach")?;
        validate_grid(&alpha_grid_rad, "alpha")?;
        if samples.iter().any(|sample| {
            !sample.lift.is_finite()
                || !sample.drag.is_finite()
                || !sample.side_force.is_finite()
                || !sample.pitching_moment.is_finite()
        }) {
            return Err(AeroError::InvalidModel(
                "coefficient table contains a non-finite sample".into(),
            ));
        }
        if samples.iter().any(|sample| sample.drag < 0.0) {
            return Err(AeroError::InvalidModel(
                "coefficient table contains negative drag".into(),
            ));
        }
        Ok(Self {
            mach_grid,
            alpha_grid_rad,
            samples,
        })
    }

    pub fn sample(&self, mach: f64, alpha_rad: f64) -> AeroCoefficients {
        let (mach_lo, mach_hi, mach_t) = bracket(&self.mach_grid, mach);
        let (alpha_lo, alpha_hi, alpha_t) = bracket(&self.alpha_grid_rad, alpha_rad);
        let alpha_count = self.alpha_grid_rad.len();
        let at = |mach_index: usize, alpha_index: usize| {
            self.samples[mach_index * alpha_count + alpha_index]
        };
        let low = interpolate_coefficients(at(mach_lo, alpha_lo), at(mach_lo, alpha_hi), alpha_t);
        let high = interpolate_coefficients(at(mach_hi, alpha_lo), at(mach_hi, alpha_hi), alpha_t);
        interpolate_coefficients(low, high, mach_t)
    }

    /// Import a small deterministic polar table from CSV.
    ///
    /// The format is `mach,alpha_deg,cl,cd,cy,cm`; a header and lines starting
    /// with `#` are optional. Rows may arrive in any order, but every Mach/AoA
    /// grid point must occur exactly once. The parser belongs to the data
    /// boundary, while runtime evaluation remains a constant-time lookup.
    pub fn from_csv<R: BufRead>(reader: R) -> Result<Self, AeroError> {
        let mut rows = Vec::new();
        for (line_number, line) in reader.lines().enumerate() {
            let line_number = line_number + 1;
            let line = line.map_err(|error| {
                AeroError::InvalidModel(format!("failed to read coefficient CSV line: {error}"))
            })?;
            let line = line.trim();
            if line.is_empty() || line.starts_with('#') {
                continue;
            }
            let fields = line.split(',').map(str::trim).collect::<Vec<_>>();
            if fields
                .first()
                .is_some_and(|field| field.eq_ignore_ascii_case("mach"))
            {
                continue;
            }
            if fields.len() != 6 {
                return Err(AeroError::InvalidModel(format!(
                    "coefficient CSV line {line_number} needs 6 columns"
                )));
            }
            let parse = |index: usize, name: &str| {
                fields[index].parse::<f64>().map_err(|error| {
                    AeroError::InvalidModel(format!(
                        "invalid {name} on coefficient CSV line {line_number}: {error}"
                    ))
                })
            };
            let mach = parse(0, "Mach")?;
            let alpha_deg = parse(1, "alpha")?;
            let coefficients = AeroCoefficients {
                lift: parse(2, "CL")?,
                drag: parse(3, "CD")?,
                side_force: parse(4, "CY")?,
                pitching_moment: parse(5, "Cm")?,
            };
            if !mach.is_finite() || mach < 0.0 || !alpha_deg.is_finite() {
                return Err(AeroError::InvalidModel(format!(
                    "invalid Mach/AoA on coefficient CSV line {line_number}"
                )));
            }
            rows.push((mach, alpha_deg.to_radians(), coefficients));
        }
        if rows.is_empty() {
            return Err(AeroError::InvalidModel(
                "coefficient CSV contains no data rows".into(),
            ));
        }
        let mut mach_grid = rows.iter().map(|row| row.0).collect::<Vec<_>>();
        let mut alpha_grid_rad = rows.iter().map(|row| row.1).collect::<Vec<_>>();
        mach_grid.sort_by(f64::total_cmp);
        mach_grid.dedup_by(|left, right| *left == *right);
        alpha_grid_rad.sort_by(f64::total_cmp);
        alpha_grid_rad.dedup_by(|left, right| *left == *right);
        let mut samples = vec![None; mach_grid.len() * alpha_grid_rad.len()];
        for (mach, alpha_rad, coefficients) in rows {
            let mach_index = mach_grid
                .binary_search_by(|value| value.total_cmp(&mach))
                .expect("Mach was inserted into the grid");
            let alpha_index = alpha_grid_rad
                .binary_search_by(|value| value.total_cmp(&alpha_rad))
                .expect("alpha was inserted into the grid");
            let slot = &mut samples[mach_index * alpha_grid_rad.len() + alpha_index];
            if slot.replace(coefficients).is_some() {
                return Err(AeroError::InvalidModel(
                    "coefficient CSV contains a duplicate Mach/AoA point".into(),
                ));
            }
        }
        let samples = samples
            .into_iter()
            .enumerate()
            .map(|(index, sample)| {
                sample.ok_or_else(|| {
                    AeroError::InvalidModel(format!(
                        "coefficient CSV is missing grid point at flattened index {index}"
                    ))
                })
            })
            .collect::<Result<Vec<_>, _>>()?;
        Self::new(mach_grid, alpha_grid_rad, samples)
    }
}

/// One complete local aerodynamic solution. `panel_loads` is intentionally
/// optional: the authoritative flight path can avoid per-frame allocations,
/// while debug/telemetry can request the detailed panel breakdown.
#[derive(Debug, Clone, PartialEq)]
pub struct AeroResult {
    pub force_body_n: DVec3,
    pub moment_body_nm: DVec3,
    pub dynamic_pressure_pa: f64,
    pub mach: f64,
    pub reynolds_number: f64,
    pub panel_count: usize,
    pub panel_loads: Option<Vec<AeroPanelLoad>>,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct AeroPanelLoad {
    pub force_body_n: DVec3,
    pub moment_body_nm: DVec3,
    pub local_velocity_body_mps: DVec3,
    pub dynamic_pressure_pa: f64,
    pub mach: f64,
    pub reynolds_number: f64,
    pub angle_of_attack_rad: f64,
    pub sideslip_rad: f64,
    pub coefficients: AeroCoefficients,
}

#[derive(Debug, Clone, PartialEq)]
pub struct AeroCase {
    pub state: AeroState,
    pub environment: AeroEnvironment,
    pub geometry: AeroGeometry,
}

impl AeroCase {
    pub fn new(
        state: AeroState,
        environment: AeroEnvironment,
        geometry: AeroGeometry,
    ) -> Result<Self, AeroError> {
        state.validate()?;
        environment.validate()?;
        geometry.validate()?;
        Ok(Self {
            state,
            environment,
            geometry,
        })
    }
}

/// The runtime model boundary. Additional implementations can consume a
/// fitted table or an external solver-backed test adapter without changing the
/// force/moment contract used by vehicle dynamics.
pub trait AeroModel: Sync {
    fn evaluate(&self, case: &AeroCase) -> Result<AeroResult, AeroError>;

    fn evaluate_detailed(&self, case: &AeroCase) -> Result<AeroResult, AeroError>;

    /// Evaluate a reusable geometry without forcing callers to clone its
    /// panel vector on every frame. Custom models retain a safe default; the
    /// built-in panel model overrides this with its allocation-free path.
    fn evaluate_state(
        &self,
        state: AeroState,
        environment: AeroEnvironment,
        geometry: &AeroGeometry,
    ) -> Result<AeroResult, AeroError> {
        self.evaluate(&AeroCase {
            state,
            environment,
            geometry: geometry.clone(),
        })
    }
}

/// Fast deterministic panel solver. It is deliberately allocation-free in its
/// normal result path and is suitable for fixed-rate realtime evaluation.
#[derive(Debug, Clone, PartialEq)]
pub struct PanelAeroModel {
    pub config: AeroConfig,
    pub coefficient_table: Option<AeroCoefficientTable>,
}

impl PanelAeroModel {
    pub fn new(config: AeroConfig) -> Result<Self, AeroError> {
        config.validate()?;
        Ok(Self {
            config,
            coefficient_table: None,
        })
    }

    pub fn from_table(
        config: AeroConfig,
        coefficient_table: AeroCoefficientTable,
    ) -> Result<Self, AeroError> {
        config.validate()?;
        Ok(Self {
            config,
            coefficient_table: Some(coefficient_table),
        })
    }

    fn evaluate_internal(
        &self,
        case: &AeroCase,
        record_panel_loads: bool,
    ) -> Result<AeroResult, AeroError> {
        self.evaluate_parts(
            case.state,
            case.environment,
            &case.geometry,
            record_panel_loads,
        )
    }

    fn evaluate_parts(
        &self,
        state: AeroState,
        environment: AeroEnvironment,
        geometry: &AeroGeometry,
        record_panel_loads: bool,
    ) -> Result<AeroResult, AeroError> {
        state.validate()?;
        environment.validate()?;
        geometry.validate()?;
        self.config.validate()?;

        let freestream_velocity = state.velocity_body_mps - environment.wind_velocity_body_mps;
        let freestream_speed = finite_length(freestream_velocity, "freestream velocity")?;
        let freestream_dynamic_pressure =
            dynamic_pressure(environment.density_kg_m3, freestream_speed)?;
        let mach = freestream_speed / environment.speed_of_sound_mps;
        if !mach.is_finite() {
            return Err(AeroError::InvalidState(
                "freestream Mach number is non-finite".into(),
            ));
        }
        let reynolds_number = reynolds(
            environment.density_kg_m3,
            freestream_speed,
            average_chord(geometry),
            environment.dynamic_viscosity_pa_s,
        );
        if !reynolds_number.is_finite() {
            return Err(AeroError::InvalidState(
                "freestream Reynolds number is non-finite".into(),
            ));
        }
        let mut force_body_n = DVec3::ZERO;
        let mut moment_body_nm = DVec3::ZERO;
        let mut panel_loads = record_panel_loads.then(Vec::new);

        for panel in &geometry.panels {
            let local_velocity = state.velocity_body_mps
                + state.angular_velocity_body_rps.cross(panel.position_body_m)
                - environment.wind_velocity_body_mps;
            let local_speed = finite_length(local_velocity, "local velocity")?;
            if local_speed <= EPS_SPEED_MPS || environment.density_kg_m3 == 0.0 {
                if let Some(loads) = &mut panel_loads {
                    loads.push(AeroPanelLoad {
                        force_body_n: DVec3::ZERO,
                        moment_body_nm: DVec3::ZERO,
                        local_velocity_body_mps: local_velocity,
                        dynamic_pressure_pa: 0.0,
                        mach: 0.0,
                        reynolds_number: 0.0,
                        angle_of_attack_rad: 0.0,
                        sideslip_rad: 0.0,
                        coefficients: AeroCoefficients {
                            lift: 0.0,
                            drag: 0.0,
                            side_force: 0.0,
                            pitching_moment: 0.0,
                        },
                    });
                }
                continue;
            }

            let chord_axis = normalize_axis(panel.chord_axis_body, "chord axis")?;
            let lift_axis = orthogonal_axis(panel.lift_axis_body, chord_axis, "lift axis")?;
            let side_axis = lift_axis.cross(chord_axis).normalize();
            let velocity_direction = local_velocity / local_speed;
            let forward = local_velocity.dot(chord_axis);
            let alpha = local_velocity.dot(lift_axis).atan2(forward);
            let beta = local_velocity
                .dot(side_axis)
                .atan2(forward.abs().max(EPS_SPEED_MPS));
            let local_mach = local_speed / environment.speed_of_sound_mps;
            if !local_mach.is_finite() {
                return Err(AeroError::InvalidState(
                    "local Mach number is non-finite".into(),
                ));
            }
            let local_reynolds = reynolds(
                environment.density_kg_m3,
                local_speed,
                panel.chord_m,
                environment.dynamic_viscosity_pa_s,
            );
            if !local_reynolds.is_finite() {
                return Err(AeroError::InvalidState(
                    "local Reynolds number is non-finite".into(),
                ));
            }
            let coefficients =
                self.coefficients(panel, local_mach, alpha, beta, panel.control_deflection_rad);
            if !coefficients.lift.is_finite()
                || !coefficients.drag.is_finite()
                || coefficients.drag < 0.0
                || !coefficients.side_force.is_finite()
                || !coefficients.pitching_moment.is_finite()
            {
                return Err(AeroError::InvalidModel(
                    "aero coefficients must be finite with non-negative drag".into(),
                ));
            }
            let local_q = dynamic_pressure(environment.density_kg_m3, local_speed)?;
            let lift_direction = project_perpendicular(lift_axis, velocity_direction);
            let side_direction = project_perpendicular(side_axis, velocity_direction);
            let force = panel.exposure
                * local_q
                * panel.area_m2
                * (-velocity_direction * coefficients.drag + lift_direction * coefficients.lift
                    - side_direction * coefficients.side_force);
            let aerodynamic_moment = side_axis
                * (local_q * panel.area_m2 * panel.chord_m * coefficients.pitching_moment);
            let reduced_rates = DVec3::new(
                state.angular_velocity_body_rps.x * panel.span_m / (2.0 * local_speed),
                state.angular_velocity_body_rps.y * panel.chord_m / (2.0 * local_speed),
                state.angular_velocity_body_rps.z * panel.span_m / (2.0 * local_speed),
            );
            let dynamic_moment = chord_axis
                * (local_q
                    * panel.area_m2
                    * panel.span_m
                    * self.config.roll_damping_coefficient
                    * reduced_rates.x)
                + side_axis
                    * (local_q
                        * panel.area_m2
                        * panel.chord_m
                        * self.config.pitch_damping_coefficient
                        * reduced_rates.y)
                + lift_axis
                    * (local_q
                        * panel.area_m2
                        * panel.span_m
                        * self.config.yaw_damping_coefficient
                        * reduced_rates.z);
            let moment =
                panel.center_of_pressure_body_m.cross(force) + aerodynamic_moment + dynamic_moment;
            if !force.is_finite() || !moment.is_finite() {
                return Err(AeroError::InvalidState(
                    "aero force or moment is non-finite".into(),
                ));
            }
            force_body_n += force;
            moment_body_nm += moment;
            if let Some(loads) = &mut panel_loads {
                loads.push(AeroPanelLoad {
                    force_body_n: force,
                    moment_body_nm: moment,
                    local_velocity_body_mps: local_velocity,
                    dynamic_pressure_pa: local_q,
                    mach: local_mach,
                    reynolds_number: local_reynolds,
                    angle_of_attack_rad: alpha,
                    sideslip_rad: beta,
                    coefficients,
                });
            }
        }

        if !force_body_n.is_finite() || !moment_body_nm.is_finite() {
            return Err(AeroError::InvalidState(
                "summed aero force or moment is non-finite".into(),
            ));
        }
        Ok(AeroResult {
            force_body_n,
            moment_body_nm,
            dynamic_pressure_pa: freestream_dynamic_pressure,
            mach,
            reynolds_number,
            panel_count: geometry.panels.len(),
            panel_loads,
        })
    }

    fn coefficients(
        &self,
        panel: &AeroPanel,
        mach: f64,
        alpha: f64,
        beta: f64,
        control_deflection: f64,
    ) -> AeroCoefficients {
        if let Some(table) = &self.coefficient_table {
            let mut coefficients = table.sample(
                mach,
                alpha + self.config.control_effectiveness * control_deflection,
            );
            coefficients.lift *= panel.lift_coefficient_sign;
            coefficients.side_force += self.config.side_force_slope_per_rad * beta;
            coefficients
        } else {
            let mut coefficients =
                self.config
                    .analytic_coefficients(mach, alpha, beta, control_deflection, panel);
            coefficients.lift *= panel.lift_coefficient_sign;
            coefficients
        }
    }
}

impl AeroModel for PanelAeroModel {
    fn evaluate(&self, case: &AeroCase) -> Result<AeroResult, AeroError> {
        self.evaluate_internal(case, false)
    }

    fn evaluate_detailed(&self, case: &AeroCase) -> Result<AeroResult, AeroError> {
        self.evaluate_internal(case, true)
    }

    fn evaluate_state(
        &self,
        state: AeroState,
        environment: AeroEnvironment,
        geometry: &AeroGeometry,
    ) -> Result<AeroResult, AeroError> {
        self.evaluate_parts(state, environment, geometry, false)
    }
}

/// Evaluate independent vehicles in parallel while preserving input order.
/// The per-case summation order remains the geometry order, so replay does not
/// depend on Rayon worker scheduling.
pub fn evaluate_batch<M: AeroModel>(
    model: &M,
    cases: &[AeroCase],
) -> Vec<Result<AeroResult, AeroError>> {
    cases.par_iter().map(|case| model.evaluate(case)).collect()
}

#[derive(Debug, Clone, PartialEq)]
pub enum AeroError {
    InvalidEnvironment(String),
    InvalidState(String),
    InvalidGeometry(String),
    InvalidModel(String),
}

impl fmt::Display for AeroError {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::InvalidEnvironment(message) => {
                write!(formatter, "invalid aero environment: {message}")
            }
            Self::InvalidState(message) => write!(formatter, "invalid aero state: {message}"),
            Self::InvalidGeometry(message) => write!(formatter, "invalid aero geometry: {message}"),
            Self::InvalidModel(message) => write!(formatter, "invalid aero model: {message}"),
        }
    }
}

impl Error for AeroError {}

fn normalize_axis(axis: DVec3, name: &str) -> Result<DVec3, AeroError> {
    if !axis.is_finite() || axis.length_squared() <= EPS_SPEED_MPS {
        return Err(AeroError::InvalidGeometry(format!(
            "{name} must be finite and non-zero"
        )));
    }
    Ok(axis.normalize())
}

fn orthogonal_axis(axis: DVec3, reference: DVec3, name: &str) -> Result<DVec3, AeroError> {
    let axis = normalize_axis(axis, name)?;
    let projected = axis - reference * axis.dot(reference);
    normalize_axis(projected, name)
}

fn project_perpendicular(axis: DVec3, direction: DVec3) -> DVec3 {
    let projected = axis - direction * axis.dot(direction);
    if projected.length_squared() <= EPS_SPEED_MPS {
        DVec3::ZERO
    } else {
        projected.normalize()
    }
}

/// Apply the Diederich finite-planform correlation to a local 2-D lift slope.
/// This keeps the runtime O(number of panels) while avoiding the common error
/// of applying an airfoil's infinite-span slope to an entire low-aspect-ratio
/// fin. The optional body-interference multiplier is part of the compiled
/// geometry, not a global vehicle tuning constant.
fn finite_planform_lift_slope(compressible_2d_slope: f64, panel: &AeroPanel) -> f64 {
    let sweep_cos = panel.planform_sweep_rad.cos();
    let correlation_parameter = 2.0 * std::f64::consts::PI * panel.planform_aspect_ratio
        / (compressible_2d_slope * sweep_cos);
    let denominator =
        2.0 + correlation_parameter * (1.0 + (2.0 / correlation_parameter).powi(2)).sqrt();
    panel.lift_interference_factor
        * (compressible_2d_slope * correlation_parameter * sweep_cos / denominator)
}

fn average_chord(geometry: &AeroGeometry) -> f64 {
    geometry
        .panels
        .iter()
        .map(|panel| panel.chord_m)
        .sum::<f64>()
        / geometry.panels.len() as f64
}

fn reynolds(density: f64, speed: f64, chord: f64, viscosity: f64) -> f64 {
    if viscosity <= 0.0 {
        0.0
    } else {
        density * speed * chord / viscosity
    }
}

/// Compute a vector norm without overflowing while squaring a large finite
/// component. A finite input is not enough here: `DVec3::length()` can still
/// become `inf` for a very large but finite velocity.
fn finite_length(value: DVec3, name: &str) -> Result<f64, AeroError> {
    if !value.is_finite() {
        return Err(AeroError::InvalidState(format!(
            "{name} contains a non-finite value"
        )));
    }
    let scale = value.abs().max_element();
    if scale == 0.0 {
        return Ok(0.0);
    }
    let length = scale * (value / scale).length();
    if length.is_finite() {
        Ok(length)
    } else {
        Err(AeroError::InvalidState(format!(
            "{name} magnitude is non-finite"
        )))
    }
}

fn dynamic_pressure(density: f64, speed: f64) -> Result<f64, AeroError> {
    let q = 0.5 * density * speed * speed;
    if q.is_finite() {
        Ok(q)
    } else {
        Err(AeroError::InvalidState(
            "dynamic pressure is non-finite".into(),
        ))
    }
}

fn validate_grid(grid: &[f64], name: &str) -> Result<(), AeroError> {
    if grid.iter().any(|value| !value.is_finite()) {
        return Err(AeroError::InvalidModel(format!(
            "{name} grid contains a non-finite value"
        )));
    }
    if grid.windows(2).any(|window| window[0] >= window[1]) {
        return Err(AeroError::InvalidModel(format!(
            "{name} grid must be strictly increasing"
        )));
    }
    Ok(())
}

fn bracket(grid: &[f64], value: f64) -> (usize, usize, f64) {
    if grid.len() == 1 || value <= grid[0] {
        return (0, 0, 0.0);
    }
    if value >= grid[grid.len() - 1] {
        let last = grid.len() - 1;
        return (last, last, 0.0);
    }
    let upper = grid.partition_point(|entry| *entry < value);
    let lower = upper - 1;
    let t = (value - grid[lower]) / (grid[upper] - grid[lower]);
    (lower, upper, t)
}

fn interpolate_coefficients(
    low: AeroCoefficients,
    high: AeroCoefficients,
    t: f64,
) -> AeroCoefficients {
    AeroCoefficients {
        lift: lerp(low.lift, high.lift, t),
        drag: lerp(low.drag, high.drag, t),
        side_force: lerp(low.side_force, high.side_force, t),
        pitching_moment: lerp(low.pitching_moment, high.pitching_moment, t),
    }
}

fn lerp(low: f64, high: f64, t: f64) -> f64 {
    low + (high - low) * t
}

fn smoothstep(edge0: f64, edge1: f64, value: f64) -> f64 {
    let t = ((value - edge0) / (edge1 - edge0)).clamp(0.0, 1.0);
    t * t * (3.0 - 2.0 * t)
}
