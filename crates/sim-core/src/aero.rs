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
    /// Multiplier of the entire lift curve, including its slope. `1` is
    /// conventional; `-1` reverses static stability and local-flow damping.
    /// Negative tail trim lift should use incidence/deflection, not this sign.
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
    /// Angular width past stall over which attached flow hands over to
    /// separated flow. Must be positive; ~6 deg keeps the lift peak within
    /// a few degrees past the critical angle (thin-wing-like break) while
    /// staying C1-smooth for trim solvers. Wider blends push the peak
    /// further out and suit gentle low-aspect-ratio stall.
    /// The handoff uses smoothstep (SIMD-friendly), never tanh/powf.
    pub post_stall_blend_rad: f64,
    /// Peak of the separated-flow flat-plate lift term
    /// (`separated_lift * sin(2*alpha)`). 1.0 recovers thin flat-plate
    /// theory; ordinary finite surfaces may calibrate lower.
    pub separated_lift_coefficient: f64,
    /// Flat-plate drag at 90 deg AoA (`separated_drag * sin^2(alpha)`).
    /// ~1.9 matches normal-force data for thin plates.
    pub separated_drag_coefficient: f64,
    /// Pitching-moment coefficient about the panel side axis once fully
    /// separated. Models the aft center-of-pressure shift without moving
    /// geometry; 0.0 keeps the attached value blended to zero.
    pub separated_pitching_moment: f64,
    /// Control-surface effectiveness retained in fully separated flow
    /// (0 = dead stick in the wake, 1 = unaffected). Blended by the same
    /// separation factor as lift.
    pub separated_control_factor: f64,
    /// Polhamus-style leading-edge vortex lift, added on top of both
    /// branches: `vortex * |sin(alpha)| * sin(alpha) * cos(alpha)`. This is
    /// the dedicated delta-wing mechanism and never weakens the general
    /// stall; 0.0 disables it (all legacy panels).
    pub vortex_lift_factor: f64,
    /// Hypersonic drag-only cutoff: above this Mach number lift (attached,
    /// separated and vortex) fades to zero over a fixed 1-Mach band,
    /// leaving pressure drag. INFINITY disables the cutoff. For ascent
    /// work where hypersonic lift errors dwarf the trajectory but drag
    /// still matters.
    pub drag_only_above_mach: f64,
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
            post_stall_blend_rad: 6.0_f64.to_radians(),
            separated_lift_coefficient: 1.0,
            separated_drag_coefficient: 1.9,
            separated_pitching_moment: 0.0,
            separated_control_factor: 0.3,
            vortex_lift_factor: 0.0,
            drag_only_above_mach: f64::INFINITY,
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
            self.post_stall_blend_rad,
            self.separated_lift_coefficient,
            self.separated_drag_coefficient,
            self.separated_pitching_moment,
            self.separated_control_factor,
            self.vortex_lift_factor,
        ];
        if finite.iter().any(|value| !value.is_finite()) {
            return Err(AeroError::InvalidModel(
                "aero configuration contains a non-finite value".into(),
            ));
        }
        // drag_only_above_mach may be INFINITY (disabled); anything else
        // must be a positive finite cutoff.
        if !(self.drag_only_above_mach.is_infinite() || self.drag_only_above_mach > 0.0) {
            return Err(AeroError::InvalidModel(
                "aero configuration has an invalid range".into(),
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
            || self.post_stall_blend_rad <= 0.0
            || self.separated_lift_coefficient < 0.0
            || self.separated_drag_coefficient < 0.0
            || self.separated_control_factor < 0.0
            || self.separated_control_factor > 1.0
            || self.vortex_lift_factor < 0.0
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

        // Attached/separated handoff driven by the AERODYNAMIC angle, so a
        // deflected surface cannot de-stall itself by moving alpha_eff.
        // smoothstep keeps the whole core to mul/add/min/max (SIMD-friendly);
        // no tanh/powf anywhere on this path.
        let alpha_aero = alpha_rad - self.zero_lift_angle_rad;
        let separation = smoothstep(
            self.stall_angle_rad,
            self.stall_angle_rad + self.post_stall_blend_rad,
            alpha_aero.abs(),
        );
        // Control surfaces lose authority in the wake: full gain attached,
        // fading to the separated factor once stalled.
        let control_gain = lerp(1.0, self.separated_control_factor, separation);
        let alpha_eff =
            alpha_aero + self.control_effectiveness * control_gain * control_deflection_rad;
        // Attached branch peaks smoothly at max_lift via a rational cap
        // (C-infinity, one sqrt) rather than tanh saturation.
        let linear_lift = lift_slope * alpha_eff;
        let capped_lift = self.max_lift_coefficient * (linear_lift / self.max_lift_coefficient)
            / (1.0 + (linear_lift / self.max_lift_coefficient).powi(2)).sqrt();
        // Separated branch is flat-plate-like: sin(2a) lift peaking at 45
        // deg and vanishing at 0/90 deg, sin^2 drag peaking at 90 deg.
        // sin(2a) keeps the correct odd symmetry past 90 deg for symmetric
        // sections; cambered sections treat it as an approximation.
        let (sin_alpha, cos_alpha) = alpha_eff.sin_cos();
        let cl_separated = self.separated_lift_coefficient * 2.0 * sin_alpha * cos_alpha;
        let cl = lerp(capped_lift, cl_separated, separation);
        // Polhamus-style leading-edge vortex lift: a DEDICATED delta-wing
        // term, never a weakening of the general stall. Odd in alpha,
        // vanishing at 0 and 90 deg by construction.
        let cl_vortex = self.vortex_lift_factor * sin_alpha.abs() * sin_alpha * cos_alpha;
        // Induced drag follows attached lift only; separation drag takes
        // over through the blend, rising sharply past stall.
        let reynolds_independent_drag = self.base_drag_coefficient
            + self.induced_drag_factor * capped_lift * capped_lift * (1.0 - separation)
            + self.separated_drag_coefficient * separation * sin_alpha * sin_alpha;

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

        // Hypersonic drag-only regime: above the cutoff, lift (attached,
        // separated and vortex alike) fades out over a fixed 1-Mach band
        // while pressure drag is kept. Off by default (INFINITY, which must
        // never reach smoothstep: INF-INF is NaN).
        let lift_fade = if self.drag_only_above_mach.is_infinite() {
            1.0
        } else {
            1.0 - smoothstep(
                self.drag_only_above_mach,
                self.drag_only_above_mach + 1.0,
                mach,
            )
        };
        // Center of pressure drifts aft with separation: blend the moment
        // coefficient the same way as the forces.
        let pitching_moment = lerp(
            self.pitching_moment_coefficient,
            self.separated_pitching_moment,
            separation,
        );

        AeroCoefficients {
            lift: (cl + cl_vortex) * lift_fade,
            drag: reynolds_independent_drag + wave_drag,
            side_force: self.side_force_slope_per_rad * beta_rad,
            pitching_moment,
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

/// Reusable scratch buffers for [`PanelAeroModel::evaluate_soa_simd_scratch`].
/// Kept by the caller across ticks: after the first evaluation no heap
/// allocation happens on the fast path, preserving the model's
/// allocation-free realtime property. Stale entries for inactive
/// (parked) lanes are always finite, and their kernel outputs are ignored
/// by force assembly, so reuse is sound.
#[derive(Debug, Default)]
pub struct AeroSimdScratch {
    lanes: Vec<LaneFlow>,
    alpha_eff: Vec<f64>,
    beta_k: Vec<f64>,
    sin_e: Vec<f64>,
    cos_e: Vec<f64>,
    sep: Vec<f64>,
    mach_k: Vec<f64>,
    sweep_cos: Vec<f64>,
    cl: Vec<f64>,
    cd: Vec<f64>,
    cy: Vec<f64>,
    cm: Vec<f64>,
}

impl AeroSimdScratch {
    fn ensure(&mut self, panels: usize) {
        self.lanes.clear();
        self.lanes.reserve(panels);
        self.alpha_eff.resize(panels, 0.0);
        self.beta_k.resize(panels, 0.0);
        self.sin_e.resize(panels, 0.0);
        self.cos_e.resize(panels, 1.0);
        self.sep.resize(panels, 0.0);
        self.mach_k.resize(panels, 0.3);
        self.sweep_cos.resize(panels, 1.0);
        self.cl.resize(panels, 0.0);
        self.cd.resize(panels, 0.0);
        self.cy.resize(panels, 0.0);
        self.cm.resize(panels, 0.0);
    }
}

/// Per-lane flow state from the scalar prologue shared by the SoA oracle and
/// the SIMD fast path. Trig (atan2) and control folding stay scalar here;
/// the kernels consume only `alpha_eff`/`separation`/sin-cos plus Mach.
/// Inactive lanes (parked panels in vacuum or still air) carry the real
/// local velocity for load recording but assemble zero force.
#[derive(Debug, Clone, Copy)]
struct LaneFlow {
    chord_axis: DVec3,
    lift_axis: DVec3,
    side_axis: DVec3,
    velocity_direction: DVec3,
    local_velocity: DVec3,
    local_speed: f64,
    local_q: f64,
    area: f64,
    chord: f64,
    span: f64,
    exposure: f64,
    center_of_pressure: DVec3,
    alpha: f64,
    beta: f64,
    local_mach: f64,
    local_reynolds: f64,
    active: bool,
}

/// Scalar prologue for one SoA lane: local flow angles and dynamic pressure.
/// Identical inputs feed the oracle's shared coefficient path and the SIMD
/// kernels, so any divergence between them is kernel math, never geometry.
fn lane_flow(
    state: AeroState,
    environment: AeroEnvironment,
    panels: &PanelSoA,
    index: usize,
) -> Result<LaneFlow, AeroError> {
    let position = DVec3::new(
        panels.pos_x[index],
        panels.pos_y[index],
        panels.pos_z[index],
    );
    let local_velocity = state.velocity_body_mps + state.angular_velocity_body_rps.cross(position)
        - environment.wind_velocity_body_mps;
    let local_speed = finite_length(local_velocity, "local velocity")?;
    let inactive = LaneFlow {
        chord_axis: DVec3::X,
        lift_axis: DVec3::Z,
        side_axis: DVec3::Y,
        velocity_direction: DVec3::X,
        local_velocity,
        local_speed,
        local_q: 0.0,
        area: 0.0,
        chord: 0.0,
        span: 0.0,
        exposure: 0.0,
        center_of_pressure: DVec3::ZERO,
        alpha: 0.0,
        beta: 0.0,
        local_mach: 0.0,
        local_reynolds: 0.0,
        active: false,
    };
    if local_speed <= EPS_SPEED_MPS || environment.density_kg_m3 == 0.0 {
        return Ok(inactive);
    }
    let chord_axis = DVec3::new(
        panels.chord_x[index],
        panels.chord_y[index],
        panels.chord_z[index],
    );
    let lift_axis = DVec3::new(
        panels.lift_x[index],
        panels.lift_y[index],
        panels.lift_z[index],
    );
    let side_axis = lift_axis.cross(chord_axis).normalize();
    let velocity_direction = local_velocity / local_speed;
    let forward = local_velocity.dot(chord_axis);
    let alpha = (-local_velocity.dot(lift_axis)).atan2(forward);
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
        panels.chord[index],
        environment.dynamic_viscosity_pa_s,
    );
    if !local_reynolds.is_finite() {
        return Err(AeroError::InvalidState(
            "local Reynolds number is non-finite".into(),
        ));
    }
    let local_q = dynamic_pressure(environment.density_kg_m3, local_speed)?;
    Ok(LaneFlow {
        chord_axis,
        lift_axis,
        side_axis,
        velocity_direction,
        local_velocity,
        local_speed,
        local_q,
        area: panels.area[index],
        chord: panels.chord[index],
        span: panels.span[index],
        exposure: panels.exposure[index],
        center_of_pressure: DVec3::new(
            panels.cop_x[index],
            panels.cop_y[index],
            panels.cop_z[index],
        ),
        alpha,
        beta,
        local_mach,
        local_reynolds,
        active: true,
    })
}

/// Zero-load record for inactive lanes, matching the oracle's parked output.
fn parked_load(lane: &LaneFlow) -> AeroPanelLoad {
    AeroPanelLoad {
        force_body_n: DVec3::ZERO,
        moment_body_nm: DVec3::ZERO,
        local_velocity_body_mps: lane.local_velocity,
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
            // Conventional vehicle angle of attack is measured from the
            // incoming flow: a nose-up attitude has a negative body-frame
            // vertical velocity and therefore a positive alpha.
            let alpha = (-local_velocity.dot(lift_axis)).atan2(forward);
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
                state.angular_velocity_body_rps.dot(chord_axis) * panel.span_m
                    / (2.0 * local_speed),
                state.angular_velocity_body_rps.dot(side_axis) * panel.chord_m
                    / (2.0 * local_speed),
                state.angular_velocity_body_rps.dot(lift_axis) * panel.span_m / (2.0 * local_speed),
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

    /// Scalar oracle over [`PanelSoA`] lanes: same equations as
    /// [`evaluate_parts`](Self::evaluate_parts) (shared coefficient path),
    /// transcribed to structure-of-arrays reads. The equivalence test pins
    /// it against the AoS path; vector kernels in `thessa-simd` target this
    /// exact data flow with a tolerance test.
    pub fn evaluate_soa_parts(
        &self,
        state: AeroState,
        environment: AeroEnvironment,
        panels: &PanelSoA,
        record_panel_loads: bool,
    ) -> Result<AeroResult, AeroError> {
        state.validate()?;
        environment.validate()?;
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
        let mut chord_sum = 0.0;
        for index in 0..panels.count {
            chord_sum += panels.chord[index];
        }
        let reynolds_number = if panels.count > 0 {
            reynolds(
                environment.density_kg_m3,
                freestream_speed,
                chord_sum / panels.count as f64,
                environment.dynamic_viscosity_pa_s,
            )
        } else {
            0.0
        };
        if !reynolds_number.is_finite() {
            return Err(AeroError::InvalidState(
                "freestream Reynolds number is non-finite".into(),
            ));
        }
        let mut force_body_n = DVec3::ZERO;
        let mut moment_body_nm = DVec3::ZERO;
        let mut panel_loads = record_panel_loads.then(Vec::new);

        for index in 0..panels.count {
            let lane = lane_flow(state, environment, panels, index)?;
            if !lane.active {
                if let Some(loads) = &mut panel_loads {
                    loads.push(parked_load(&lane));
                }
                continue;
            }
            let panel = panels.panel_at(index);
            let coefficients = self.coefficients(
                &panel,
                lane.local_mach,
                lane.alpha,
                lane.beta,
                panels.deflection[index],
            );
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
            let (force, moment) =
                self.assemble_lane(state.angular_velocity_body_rps, &lane, &coefficients)?;
            force_body_n += force;
            moment_body_nm += moment;
            if let Some(loads) = &mut panel_loads {
                loads.push(AeroPanelLoad {
                    force_body_n: force,
                    moment_body_nm: moment,
                    local_velocity_body_mps: lane.local_velocity,
                    dynamic_pressure_pa: lane.local_q,
                    mach: lane.local_mach,
                    reynolds_number: lane.local_reynolds,
                    angle_of_attack_rad: lane.alpha,
                    sideslip_rad: lane.beta,
                    coefficients,
                });
            }
        }
        Ok(AeroResult {
            force_body_n,
            moment_body_nm,
            dynamic_pressure_pa: freestream_dynamic_pressure,
            mach,
            reynolds_number,
            panel_count: panels.count,
            panel_loads,
        })
    }

    /// SIMD fast path over [`PanelSoA`] lanes: scalar prologue (flow angles,
    /// separation, control folding, sin/cos) then the 8/4-wide coefficient
    /// kernels from `thessa-simd`, then the shared assembly below. Short
    /// tails and machines without AVX-512 evaluate the same shared scalar
    /// coefficient path per lane, so the result is deterministic for a fixed
    /// lane count and feature set (cross-machine bits may differ). A
    /// coefficient table forces delegation to [`evaluate_soa_parts`](Self::evaluate_soa_parts):
    /// the kernels implement the analytic model only.
    pub fn evaluate_soa_simd(
        &self,
        state: AeroState,
        environment: AeroEnvironment,
        panels: &PanelSoA,
        record_panel_loads: bool,
    ) -> Result<AeroResult, AeroError> {
        self.evaluate_soa_simd_scratch(
            state,
            environment,
            panels,
            record_panel_loads,
            &mut AeroSimdScratch::default(),
        )
    }

    /// SIMD fast path with caller-owned scratch (see [`AeroSimdScratch`]).
    /// Identical results to [`evaluate_soa_simd`](Self::evaluate_soa_simd)
    /// without per-tick heap allocation once the scratch is warm.
    pub fn evaluate_soa_simd_scratch(
        &self,
        state: AeroState,
        environment: AeroEnvironment,
        panels: &PanelSoA,
        record_panel_loads: bool,
        scratch: &mut AeroSimdScratch,
    ) -> Result<AeroResult, AeroError> {
        if self.coefficient_table.is_some() {
            return self.evaluate_soa_parts(state, environment, panels, record_panel_loads);
        }
        state.validate()?;
        environment.validate()?;
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
        let mut chord_sum = 0.0;
        for index in 0..panels.count {
            chord_sum += panels.chord[index];
        }
        let reynolds_number = if panels.count > 0 {
            reynolds(
                environment.density_kg_m3,
                freestream_speed,
                chord_sum / panels.count as f64,
                environment.dynamic_viscosity_pa_s,
            )
        } else {
            0.0
        };
        if !reynolds_number.is_finite() {
            return Err(AeroError::InvalidState(
                "freestream Reynolds number is non-finite".into(),
            ));
        }

        let n = panels.count;
        scratch.ensure(n);
        let s = &mut *scratch;
        for index in 0..n {
            let lane = lane_flow(state, environment, panels, index)?;
            if lane.active {
                // Analytic-model head, transcribed: the handoff is driven by
                // the aerodynamic angle so a deflected surface cannot
                // de-stall itself; control gain fades with separation.
                let alpha_aero = lane.alpha - self.config.zero_lift_angle_rad;
                let separation = smoothstep(
                    self.config.stall_angle_rad,
                    self.config.stall_angle_rad + self.config.post_stall_blend_rad,
                    alpha_aero.abs(),
                );
                let gain = lerp(1.0, self.config.separated_control_factor, separation);
                let effective = alpha_aero
                    + self.config.control_effectiveness * gain * panels.deflection[index];
                let (sn, cs) = effective.sin_cos();
                s.alpha_eff[index] = effective;
                s.beta_k[index] = lane.beta;
                s.sin_e[index] = sn;
                s.cos_e[index] = cs;
                s.sep[index] = separation;
                s.mach_k[index] = lane.local_mach;
                s.sweep_cos[index] = panels.sweep_rad[index].cos();
            }
            s.lanes.push(lane);
        }
        let params = thessa_simd::AeroKernelParams {
            lift_slope: self.config.lift_slope_per_rad,
            max_lift: self.config.max_lift_coefficient,
            base_drag: self.config.base_drag_coefficient,
            induced: self.config.induced_drag_factor,
            wave_coeff: self.config.wave_drag_coefficient,
            side_slope: self.config.side_force_slope_per_rad,
            cm_att: self.config.pitching_moment_coefficient,
            sep_lift: self.config.separated_lift_coefficient,
            sep_drag: self.config.separated_drag_coefficient,
            cm_sep: self.config.separated_pitching_moment,
            vortex: self.config.vortex_lift_factor,
            ss_factor: self.config.supersonic_lift_slope_factor,
            wave_factor: self.config.supersonic_wave_drag_factor,
            beta_floor: self.config.transonic_beta_floor,
            drag_cut: self.config.drag_only_above_mach,
        };
        let mut body = 0;
        while body + 8 <= n {
            let done = thessa_simd::aero_coefficients_chunk(
                &s.alpha_eff,
                &s.beta_k,
                &s.sin_e,
                &s.cos_e,
                &s.sep,
                &s.mach_k,
                &s.sweep_cos,
                &panels.aspect_ratio,
                &panels.interference,
                &panels.thickness_ratio,
                body,
                &params,
                &mut s.cl,
                &mut s.cd,
                &mut s.cy,
                &mut s.cm,
            );
            if !done {
                for half in [body, body + 4] {
                    let done4 = thessa_simd::aero_coefficients_quad(
                        &s.alpha_eff,
                        &s.beta_k,
                        &s.sin_e,
                        &s.cos_e,
                        &s.sep,
                        &s.mach_k,
                        &s.sweep_cos,
                        &panels.aspect_ratio,
                        &panels.interference,
                        &panels.thickness_ratio,
                        half,
                        &params,
                        &mut s.cl,
                        &mut s.cd,
                        &mut s.cy,
                        &mut s.cm,
                    );
                    if !done4 {
                        for index in half..half + 4 {
                            let coefficients = self.analytic_lane(panels, &s.lanes[index], index);
                            s.cl[index] = coefficients.lift;
                            s.cd[index] = coefficients.drag;
                            s.cy[index] = coefficients.side_force;
                            s.cm[index] = coefficients.pitching_moment;
                        }
                    }
                }
            }
            body += 8;
        }
        while body + 4 <= n {
            let done4 = thessa_simd::aero_coefficients_quad(
                &s.alpha_eff,
                &s.beta_k,
                &s.sin_e,
                &s.cos_e,
                &s.sep,
                &s.mach_k,
                &s.sweep_cos,
                &panels.aspect_ratio,
                &panels.interference,
                &panels.thickness_ratio,
                body,
                &params,
                &mut s.cl,
                &mut s.cd,
                &mut s.cy,
                &mut s.cm,
            );
            if !done4 {
                for index in body..body + 4 {
                    let coefficients = self.analytic_lane(panels, &s.lanes[index], index);
                    s.cl[index] = coefficients.lift;
                    s.cd[index] = coefficients.drag;
                    s.cy[index] = coefficients.side_force;
                    s.cm[index] = coefficients.pitching_moment;
                }
            }
            body += 4;
        }
        for index in body..n {
            let coefficients = self.analytic_lane(panels, &s.lanes[index], index);
            s.cl[index] = coefficients.lift;
            s.cd[index] = coefficients.drag;
            s.cy[index] = coefficients.side_force;
            s.cm[index] = coefficients.pitching_moment;
        }

        let mut force_body_n = DVec3::ZERO;
        let mut moment_body_nm = DVec3::ZERO;
        let mut panel_loads = record_panel_loads.then(Vec::new);
        for index in 0..n {
            let lane = &s.lanes[index];
            if !lane.active {
                if let Some(loads) = &mut panel_loads {
                    loads.push(parked_load(lane));
                }
                continue;
            }
            if !s.cl[index].is_finite()
                || !s.cd[index].is_finite()
                || s.cd[index] < 0.0
                || !s.cy[index].is_finite()
                || !s.cm[index].is_finite()
            {
                return Err(AeroError::InvalidModel(
                    "aero coefficients must be finite with non-negative drag".into(),
                ));
            }
            // Lift sign lives in force assembly, outside the kernels.
            let coefficients = AeroCoefficients {
                lift: s.cl[index] * panels.lift_sign[index],
                drag: s.cd[index],
                side_force: s.cy[index],
                pitching_moment: s.cm[index],
            };
            let (force, moment) =
                self.assemble_lane(state.angular_velocity_body_rps, lane, &coefficients)?;
            force_body_n += force;
            moment_body_nm += moment;
            if let Some(loads) = &mut panel_loads {
                loads.push(AeroPanelLoad {
                    force_body_n: force,
                    moment_body_nm: moment,
                    local_velocity_body_mps: lane.local_velocity,
                    dynamic_pressure_pa: lane.local_q,
                    mach: lane.local_mach,
                    reynolds_number: lane.local_reynolds,
                    angle_of_attack_rad: lane.alpha,
                    sideslip_rad: lane.beta,
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
            panel_count: panels.count,
            panel_loads,
        })
    }

    /// Shared scalar coefficient lane for SIMD tails and fallback machines:
    /// the analytic model without lift sign (kernels never see the sign;
    /// assembly applies it uniformly).
    fn analytic_lane(&self, panels: &PanelSoA, lane: &LaneFlow, index: usize) -> AeroCoefficients {
        if !lane.active {
            return AeroCoefficients {
                lift: 0.0,
                drag: 0.0,
                side_force: 0.0,
                pitching_moment: 0.0,
            };
        }
        let panel = panels.panel_at(index);
        self.config.analytic_coefficients(
            lane.local_mach,
            lane.alpha,
            lane.beta,
            panels.deflection[index],
            &panel,
        )
    }

    /// Force/moment assembly for one lane, shared by the SoA oracle and the
    /// SIMD fast path: exposure-scaled pressure force along the projected
    /// axes plus pitching moment about the side axis, CP cross term and the
    /// rotational-damping moment. Inactive lanes assemble zero.
    fn assemble_lane(
        &self,
        angular_velocity_body_rps: DVec3,
        lane: &LaneFlow,
        coefficients: &AeroCoefficients,
    ) -> Result<(DVec3, DVec3), AeroError> {
        if !lane.active {
            return Ok((DVec3::ZERO, DVec3::ZERO));
        }
        let lift_direction = project_perpendicular(lane.lift_axis, lane.velocity_direction);
        let side_direction = project_perpendicular(lane.side_axis, lane.velocity_direction);
        let force = lane.exposure
            * lane.local_q
            * lane.area
            * (-lane.velocity_direction * coefficients.drag + lift_direction * coefficients.lift
                - side_direction * coefficients.side_force);
        let aerodynamic_moment =
            lane.side_axis * (lane.local_q * lane.area * lane.chord * coefficients.pitching_moment);
        let reduced_rates = DVec3::new(
            angular_velocity_body_rps.dot(lane.chord_axis) * lane.span / (2.0 * lane.local_speed),
            angular_velocity_body_rps.dot(lane.side_axis) * lane.chord / (2.0 * lane.local_speed),
            angular_velocity_body_rps.dot(lane.lift_axis) * lane.span / (2.0 * lane.local_speed),
        );
        let dynamic_moment = lane.chord_axis
            * (lane.local_q
                * lane.area
                * lane.span
                * self.config.roll_damping_coefficient
                * reduced_rates.x)
            + lane.side_axis
                * (lane.local_q
                    * lane.area
                    * lane.chord
                    * self.config.pitch_damping_coefficient
                    * reduced_rates.y)
            + lane.lift_axis
                * (lane.local_q
                    * lane.area
                    * lane.span
                    * self.config.yaw_damping_coefficient
                    * reduced_rates.z);
        let moment = lane.center_of_pressure.cross(force) + aerodynamic_moment + dynamic_moment;
        if !force.is_finite() || !moment.is_finite() {
            return Err(AeroError::InvalidState(
                "aero force or moment is non-finite".into(),
            ));
        }
        Ok((force, moment))
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

/// Structure-of-arrays panel layout for vectorized evaluation: one
/// contiguous run per field, so an 8-wide kernel loads 8 panels with plain
/// `loadu` and no gathers. Built once per geometry (vehicle compile time),
/// consumed every tick. Field order mirrors [`AeroPanel`]; the scalar oracle
/// below reads the same arrays the kernels will consume.
#[derive(Debug, Clone, PartialEq)]
pub struct PanelSoA {
    pub count: usize,
    pub pos_x: Vec<f64>,
    pub pos_y: Vec<f64>,
    pub pos_z: Vec<f64>,
    pub cop_x: Vec<f64>,
    pub cop_y: Vec<f64>,
    pub cop_z: Vec<f64>,
    pub chord_x: Vec<f64>,
    pub chord_y: Vec<f64>,
    pub chord_z: Vec<f64>,
    pub lift_x: Vec<f64>,
    pub lift_y: Vec<f64>,
    pub lift_z: Vec<f64>,
    pub area: Vec<f64>,
    pub chord: Vec<f64>,
    pub span: Vec<f64>,
    pub aspect_ratio: Vec<f64>,
    pub sweep_rad: Vec<f64>,
    pub interference: Vec<f64>,
    pub lift_sign: Vec<f64>,
    pub thickness_ratio: Vec<f64>,
    pub deflection: Vec<f64>,
    pub exposure: Vec<f64>,
}

/// Push helper keeping every lane aligned; a partial push on error would
/// desynchronize the whole layout.
macro_rules! push_panel_lane {
    ($soa:expr, $panel:expr, $chord:expr, $lift:expr) => {{
        $soa.pos_x.push($panel.position_body_m.x);
        $soa.pos_y.push($panel.position_body_m.y);
        $soa.pos_z.push($panel.position_body_m.z);
        $soa.cop_x.push($panel.center_of_pressure_body_m.x);
        $soa.cop_y.push($panel.center_of_pressure_body_m.y);
        $soa.cop_z.push($panel.center_of_pressure_body_m.z);
        $soa.chord_x.push($chord.x);
        $soa.chord_y.push($chord.y);
        $soa.chord_z.push($chord.z);
        $soa.lift_x.push($lift.x);
        $soa.lift_y.push($lift.y);
        $soa.lift_z.push($lift.z);
        $soa.area.push($panel.area_m2);
        $soa.chord.push($panel.chord_m);
        $soa.span.push($panel.span_m);
        $soa.aspect_ratio.push($panel.planform_aspect_ratio);
        $soa.sweep_rad.push($panel.planform_sweep_rad);
        $soa.interference.push($panel.lift_interference_factor);
        $soa.lift_sign.push($panel.lift_coefficient_sign);
        $soa.thickness_ratio.push($panel.thickness_to_chord_ratio);
        $soa.deflection.push($panel.control_deflection_rad);
        $soa.exposure.push($panel.exposure);
    }};
}

impl PanelSoA {
    /// Compile-once layout from a validated geometry. Axes are normalized
    /// and orthogonalized exactly like the scalar path, so the oracle below
    /// and the kernels consume identical inputs.
    pub fn from_geometry(geometry: &AeroGeometry) -> Result<Self, AeroError> {
        geometry.validate()?;
        let mut soa = Self {
            count: geometry.panels.len(),
            pos_x: Vec::with_capacity(geometry.panels.len()),
            pos_y: Vec::with_capacity(geometry.panels.len()),
            pos_z: Vec::with_capacity(geometry.panels.len()),
            cop_x: Vec::with_capacity(geometry.panels.len()),
            cop_y: Vec::with_capacity(geometry.panels.len()),
            cop_z: Vec::with_capacity(geometry.panels.len()),
            chord_x: Vec::with_capacity(geometry.panels.len()),
            chord_y: Vec::with_capacity(geometry.panels.len()),
            chord_z: Vec::with_capacity(geometry.panels.len()),
            lift_x: Vec::with_capacity(geometry.panels.len()),
            lift_y: Vec::with_capacity(geometry.panels.len()),
            lift_z: Vec::with_capacity(geometry.panels.len()),
            area: Vec::with_capacity(geometry.panels.len()),
            chord: Vec::with_capacity(geometry.panels.len()),
            span: Vec::with_capacity(geometry.panels.len()),
            aspect_ratio: Vec::with_capacity(geometry.panels.len()),
            sweep_rad: Vec::with_capacity(geometry.panels.len()),
            interference: Vec::with_capacity(geometry.panels.len()),
            lift_sign: Vec::with_capacity(geometry.panels.len()),
            thickness_ratio: Vec::with_capacity(geometry.panels.len()),
            deflection: Vec::with_capacity(geometry.panels.len()),
            exposure: Vec::with_capacity(geometry.panels.len()),
        };
        for panel in &geometry.panels {
            let chord = normalize_axis(panel.chord_axis_body, "chord axis")?;
            let lift = orthogonal_axis(panel.lift_axis_body, chord, "lift axis")?;
            push_panel_lane!(soa, panel, chord, lift);
        }
        Ok(soa)
    }

    /// Rehydrate one lane for the shared coefficient path. Copy cost is
    /// trivial next to a coefficient evaluation; the kernels bypass this.
    fn panel_at(&self, index: usize) -> AeroPanel {
        AeroPanel {
            position_body_m: DVec3::new(self.pos_x[index], self.pos_y[index], self.pos_z[index]),
            center_of_pressure_body_m: DVec3::new(
                self.cop_x[index],
                self.cop_y[index],
                self.cop_z[index],
            ),
            chord_axis_body: DVec3::new(
                self.chord_x[index],
                self.chord_y[index],
                self.chord_z[index],
            ),
            lift_axis_body: DVec3::new(self.lift_x[index], self.lift_y[index], self.lift_z[index]),
            area_m2: self.area[index],
            chord_m: self.chord[index],
            span_m: self.span[index],
            planform_aspect_ratio: self.aspect_ratio[index],
            planform_sweep_rad: self.sweep_rad[index],
            lift_interference_factor: self.interference[index],
            lift_coefficient_sign: self.lift_sign[index],
            thickness_to_chord_ratio: self.thickness_ratio[index],
            control_deflection_rad: self.deflection[index],
            exposure: self.exposure[index],
        }
    }
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
