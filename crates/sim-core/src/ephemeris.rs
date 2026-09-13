use std::{error::Error, fmt};

use glam::{DQuat, DVec3};
use serde::{Deserialize, Serialize};

use crate::{SimTime, TAU};

/// Stable index into a baked ephemeris. IDs are numeric at runtime so source
/// ordering can be explicit and independent of hash-map iteration.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash, Serialize, Deserialize)]
pub struct BodyId(pub u32);

impl BodyId {
    pub const fn index(self) -> usize {
        self.0 as usize
    }
}

/// Authoritative state returned by an on-rails celestial ephemeris.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct BodyState {
    pub position_inertial: DVec3,
    pub velocity_inertial: DVec3,
    pub orientation: DQuat,
    pub angular_velocity: DVec3,
}

impl BodyState {
    pub const ORIGIN: Self = Self {
        position_inertial: DVec3::ZERO,
        velocity_inertial: DVec3::ZERO,
        orientation: DQuat::IDENTITY,
        angular_velocity: DVec3::ZERO,
    };
}

/// Elliptic Kepler elements that form one deterministic on-rails segment.
///
/// The segment is analytic rather than a runtime-integrated planet state. This
/// gives constant-time random access and exactly repeatable evaluation while
/// the system baker remains free to replace it with fitted Chebyshev segments
/// later without changing the gravity API.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct KeplerOrbit {
    pub central_mu: f64,
    pub semi_major_axis_m: f64,
    pub eccentricity: f64,
    pub inclination_rad: f64,
    pub longitude_of_ascending_node_rad: f64,
    pub argument_of_periapsis_rad: f64,
    pub mean_anomaly_at_epoch_rad: f64,
    /// Override for a component's barycentric orbit. For an ordinary
    /// two-body orbit this is `None` and mean motion is derived from `a`.
    pub mean_motion_rad_s: Option<f64>,
}

impl KeplerOrbit {
    pub fn new(
        central_mu: f64,
        semi_major_axis_m: f64,
        eccentricity: f64,
        inclination_rad: f64,
        longitude_of_ascending_node_rad: f64,
        argument_of_periapsis_rad: f64,
        mean_anomaly_at_epoch_rad: f64,
    ) -> Result<Self, EphemerisError> {
        if !central_mu.is_finite() || central_mu <= 0.0 {
            return Err(EphemerisError::InvalidOrbit(
                "central gravitational parameter must be positive and finite".into(),
            ));
        }
        if !semi_major_axis_m.is_finite() || semi_major_axis_m <= 0.0 {
            return Err(EphemerisError::InvalidOrbit(
                "semi-major axis must be positive and finite".into(),
            ));
        }
        if !eccentricity.is_finite() || !(0.0..1.0).contains(&eccentricity) {
            return Err(EphemerisError::InvalidOrbit(
                "this MVP supports elliptic eccentricity 0 <= e < 1".into(),
            ));
        }
        Ok(Self {
            central_mu,
            semi_major_axis_m,
            eccentricity,
            inclination_rad,
            longitude_of_ascending_node_rad,
            argument_of_periapsis_rad,
            mean_anomaly_at_epoch_rad,
            mean_motion_rad_s: None,
        })
    }

    pub fn with_mean_motion(mut self, mean_motion_rad_s: f64) -> Result<Self, EphemerisError> {
        if !mean_motion_rad_s.is_finite() || mean_motion_rad_s <= 0.0 {
            return Err(EphemerisError::InvalidOrbit(
                "mean motion must be positive and finite".into(),
            ));
        }
        self.mean_motion_rad_s = Some(mean_motion_rad_s);
        Ok(self)
    }

    pub fn period_s(self) -> f64 {
        TAU / self.mean_motion()
    }

    pub fn state_relative_at(self, time: SimTime) -> Result<(DVec3, DVec3), EphemerisError> {
        let mean_motion = self.mean_motion();
        let mean_anomaly = self.mean_anomaly_at_epoch_rad + mean_motion * time.seconds();
        let eccentric_anomaly = solve_kepler(mean_anomaly, self.eccentricity);
        let (sin_e, cos_e) = eccentric_anomaly.sin_cos();
        let one_minus_e2 = 1.0 - self.eccentricity * self.eccentricity;
        let x = self.semi_major_axis_m * (cos_e - self.eccentricity);
        let y = self.semi_major_axis_m * one_minus_e2.sqrt() * sin_e;
        let velocity_factor =
            self.mean_motion() * self.semi_major_axis_m / (1.0 - self.eccentricity * cos_e);
        let vx = -velocity_factor * sin_e;
        let vy = velocity_factor * one_minus_e2.sqrt() * cos_e;

        let (p_hat, q_hat) = perifocal_basis(
            self.longitude_of_ascending_node_rad,
            self.argument_of_periapsis_rad,
            self.inclination_rad,
        );
        Ok((p_hat * x + q_hat * y, p_hat * vx + q_hat * vy))
    }

    fn mean_motion(self) -> f64 {
        self.mean_motion_rad_s
            .unwrap_or_else(|| (self.central_mu / self.semi_major_axis_m.powi(3)).sqrt())
    }
}

/// Osculating two-body elements at one epoch, derived from an instantaneous
/// state vector. Unlike [`KeplerOrbit`] this covers hyperbolic escape paths
/// (`e >= 1`, negative semi-major axis) so a departing craft still reports a
/// meaningful trajectory instead of an error.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct OsculatingElements {
    pub central_mu: f64,
    pub semi_major_axis_m: f64,
    pub eccentricity: f64,
    pub inclination_rad: f64,
    pub longitude_of_ascending_node_rad: f64,
    pub argument_of_periapsis_rad: f64,
    pub true_anomaly_rad: f64,
}

impl OsculatingElements {
    /// Classical state-to-elements conversion (Vallado algorithm 2).
    /// Positions/velocities are central-body-relative SI vectors.
    pub fn from_state(
        relative_position_m: DVec3,
        relative_velocity_mps: DVec3,
        central_mu: f64,
    ) -> Result<Self, EphemerisError> {
        if !central_mu.is_finite() || central_mu <= 0.0 {
            return Err(EphemerisError::InvalidOrbit(
                "central gravitational parameter must be positive and finite".into(),
            ));
        }
        let r_norm = relative_position_m.length();
        let v_norm_sq = relative_velocity_mps.length_squared();
        if !r_norm.is_finite() || r_norm <= 0.0 || !v_norm_sq.is_finite() {
            return Err(EphemerisError::InvalidOrbit(
                "state vectors must be finite with nonzero radius".into(),
            ));
        }
        let h = relative_position_m.cross(relative_velocity_mps);
        let h_norm = h.length();
        if !h_norm.is_finite() || h_norm <= r_norm * v_norm_sq.sqrt() * 1e-12 {
            return Err(EphemerisError::InvalidOrbit(
                "radial plunge has no defined orbital plane".into(),
            ));
        }
        let energy = v_norm_sq / 2.0 - central_mu / r_norm;
        // Parabolic boundary carries no stable elements in this form.
        if energy.abs() < central_mu / r_norm * 1e-9 {
            return Err(EphemerisError::InvalidOrbit(
                "parabolic energy has no stable elements".into(),
            ));
        }
        let n = DVec3::Z.cross(h);
        let n_norm = n.length();
        let r_dot_v = relative_position_m.dot(relative_velocity_mps);
        let e_vec = (relative_position_m * (v_norm_sq - central_mu / r_norm)
            - relative_velocity_mps * r_dot_v)
            / central_mu;
        let e = e_vec.length();
        if !e.is_finite() {
            return Err(EphemerisError::InvalidOrbit(
                "eccentricity vector is not finite".into(),
            ));
        }
        let h_hat = h / h_norm;
        let inclination = h.x.hypot(h.y).atan2(h.z);
        let inclined = n_norm > h_norm * 1e-9;
        let reference = if inclined { n / n_norm } else { DVec3::X };
        let raan = if inclined {
            n.y.atan2(n.x).rem_euclid(TAU)
        } else {
            0.0
        };
        // Signed angles around the actual angular momentum retain the
        // quadrant for circular/equatorial and retrograde states alike.
        let angle = |from: DVec3, to: DVec3| {
            h_hat
                .dot(from.cross(to))
                .atan2(from.dot(to))
                .rem_euclid(TAU)
        };
        let (arg_periapsis, true_anomaly) = if e > 1e-9 {
            let e_hat = e_vec / e;
            (
                angle(reference, e_hat),
                angle(e_hat, relative_position_m / r_norm),
            )
        } else {
            (0.0, angle(reference, relative_position_m / r_norm))
        };
        Ok(Self {
            central_mu,
            semi_major_axis_m: -central_mu / (2.0 * energy),
            eccentricity: e,
            inclination_rad: inclination,
            longitude_of_ascending_node_rad: raan,
            argument_of_periapsis_rad: arg_periapsis,
            true_anomaly_rad: true_anomaly,
        })
    }

    pub fn is_escape(self) -> bool {
        self.eccentricity >= 1.0
    }

    /// Periapsis radius, valid for elliptic and hyperbolic paths.
    pub fn periapsis_m(self) -> f64 {
        self.semi_major_axis_m * (1.0 - self.eccentricity)
    }

    /// Apoapsis radius, or `None` on escape trajectories.
    pub fn apoapsis_m(self) -> Option<f64> {
        (!self.is_escape()).then_some(self.semi_major_axis_m * (1.0 + self.eccentricity))
    }

    /// Keplerian period, or `None` on escape trajectories.
    pub fn period_s(self) -> Option<f64> {
        (!self.is_escape()).then(|| TAU * (self.semi_major_axis_m.powi(3) / self.central_mu).sqrt())
    }

    /// Inertial position at a true anomaly, central-body-relative.
    pub fn position_at_nu(self, true_anomaly_rad: f64) -> DVec3 {
        let semi_latus = self.semi_major_axis_m * (1.0 - self.eccentricity.powi(2));
        let radius = semi_latus / (1.0 + self.eccentricity * true_anomaly_rad.cos()).max(1e-9);
        let (p_hat, q_hat) = perifocal_basis(
            self.longitude_of_ascending_node_rad,
            self.argument_of_periapsis_rad,
            self.inclination_rad,
        );
        let (sin_nu, cos_nu) = true_anomaly_rad.sin_cos();
        p_hat * (radius * cos_nu) + q_hat * (radius * sin_nu)
    }
}

/// A single body entry in a deterministic baked ephemeris.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct BakedBody {
    pub id: BodyId,
    pub name: String,
    pub mu: f64,
    pub radius_m: f64,
    pub parent: Option<BodyId>,
    pub orbit: Option<KeplerOrbit>,
    pub design_period_s: Option<f64>,
    /// Sidereal spin period. `None` means the visual/runtime consumer may use
    /// its documented fallback for bodies without a spin target yet.
    #[serde(default)]
    pub rotation_period_s: Option<f64>,
    /// Keep the same body-facing longitude pointed at the immediate parent.
    #[serde(default)]
    pub tidal_lock: bool,
    /// Rotation-axis tilt relative to the engine's reference plane.
    #[serde(default)]
    pub axial_tilt_rad: f64,
    /// Synthetic barycentres are useful for kinematics but must not be added
    /// to the gravity source list alongside their component bodies.
    pub gravity_source: bool,
}

impl BakedBody {
    pub fn fixed(id: BodyId, name: impl Into<String>, mu: f64, radius_m: f64) -> Self {
        Self {
            id,
            name: name.into(),
            mu,
            radius_m,
            parent: None,
            orbit: None,
            design_period_s: None,
            rotation_period_s: None,
            tidal_lock: false,
            axial_tilt_rad: 0.0,
            gravity_source: true,
        }
    }

    pub fn orbital(
        id: BodyId,
        name: impl Into<String>,
        mu: f64,
        radius_m: f64,
        parent: BodyId,
        orbit: KeplerOrbit,
    ) -> Self {
        Self {
            id,
            name: name.into(),
            mu,
            radius_m,
            parent: Some(parent),
            orbit: Some(orbit),
            design_period_s: None,
            rotation_period_s: None,
            tidal_lock: false,
            axial_tilt_rad: 0.0,
            gravity_source: true,
        }
    }

    pub fn synthetic_barycenter(
        id: BodyId,
        name: impl Into<String>,
        mu: f64,
        parent: Option<BodyId>,
        orbit: Option<KeplerOrbit>,
    ) -> Self {
        Self {
            id,
            name: name.into(),
            mu,
            radius_m: 0.0,
            parent,
            orbit,
            design_period_s: None,
            rotation_period_s: None,
            tidal_lock: false,
            axial_tilt_rad: 0.0,
            gravity_source: false,
        }
    }
}

/// Versioned analytic ephemeris consumed by runtime gravity.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct BakedEphemeris {
    pub format_version: u32,
    pub epoch_name: String,
    pub bodies: Vec<BakedBody>,
}

impl BakedEphemeris {
    pub fn new(
        epoch_name: impl Into<String>,
        bodies: Vec<BakedBody>,
    ) -> Result<Self, EphemerisError> {
        let ephemeris = Self {
            format_version: 1,
            epoch_name: epoch_name.into(),
            bodies,
        };
        ephemeris.validate()?;
        Ok(ephemeris)
    }

    pub fn body(&self, id: BodyId) -> Result<&BakedBody, EphemerisError> {
        self.bodies
            .get(id.index())
            .filter(|body| body.id == id)
            .ok_or(EphemerisError::UnknownBody(id))
    }

    pub fn body_id(&self, name: &str) -> Option<BodyId> {
        self.bodies
            .iter()
            .find(|body| body.name == name)
            .map(|body| body.id)
    }

    pub fn body_state(&self, id: BodyId, time: SimTime) -> Result<BodyState, EphemerisError> {
        self.body_state_with_stack(id, time, &mut Vec::new())
    }

    /// Conservative inertial speed bound over every orbital phase. Summing
    /// parent/periapsis bounds also covers nested Kepler/binary trajectories.
    pub fn maximum_body_speed(&self, id: BodyId) -> Result<f64, EphemerisError> {
        let mut id = Some(id);
        let mut speed = 0.0;
        for _ in 0..=self.bodies.len() {
            let Some(current) = id else {
                return Ok(speed);
            };
            let body = self.body(current)?;
            if let Some(orbit) = body.orbit {
                speed += orbit.mean_motion().abs()
                    * orbit.semi_major_axis_m
                    * ((1.0 + orbit.eccentricity) / (1.0 - orbit.eccentricity)).sqrt();
            }
            id = body.parent;
        }
        Err(EphemerisError::InvalidOrbit("cyclic body hierarchy".into()))
    }

    /// Inertial acceleration bound, summed over nested Kepler orbits.
    pub fn maximum_body_acceleration(&self, id: BodyId) -> Result<f64, EphemerisError> {
        let mut id = Some(id);
        let mut acceleration = 0.0;
        for _ in 0..=self.bodies.len() {
            let Some(current) = id else {
                return Ok(acceleration);
            };
            let body = self.body(current)?;
            if let Some(orbit) = body.orbit {
                acceleration += orbit.mean_motion().powi(2) * orbit.semi_major_axis_m
                    / (1.0 - orbit.eccentricity).powi(2);
            }
            id = body.parent;
        }
        Err(EphemerisError::InvalidOrbit("cyclic body hierarchy".into()))
    }

    pub fn gravity_sources(&self) -> impl Iterator<Item = &BakedBody> {
        self.bodies.iter().filter(|body| body.gravity_source)
    }

    /// Display-only dominant body at a position: the gravity source with the
    /// largest local `mu / r^2`. This is a readout hint (which world the speed
    /// and orbit lines are relative to), never a physics switch: gravity stays
    /// a summed field, there is no SOI transition.
    pub fn dominant_body(&self, position_inertial_m: DVec3, time: SimTime) -> Option<BodyId> {
        let mut best: Option<(BodyId, f64)> = None;
        for body in self.gravity_sources() {
            if !body.mu.is_finite() || body.mu <= 0.0 {
                continue;
            }
            let Ok(state) = self.body_state(body.id, time) else {
                continue;
            };
            let distance_squared = (state.position_inertial - position_inertial_m).length_squared();
            if !distance_squared.is_finite() || distance_squared <= 0.0 {
                continue;
            }
            let pull = body.mu / distance_squared;
            if best.is_none_or(|(_, best_pull)| pull > best_pull) {
                best = Some((body.id, pull));
            }
        }
        best.map(|(id, _)| id)
    }

    pub fn validate(&self) -> Result<(), EphemerisError> {
        for (index, body) in self.bodies.iter().enumerate() {
            if body.id.index() != index {
                return Err(EphemerisError::InvalidBody(format!(
                    "body {} has id {:?}; IDs must match vector order",
                    body.name, body.id
                )));
            }
            if !body.mu.is_finite() || body.mu < 0.0 {
                return Err(EphemerisError::InvalidBody(format!(
                    "body {} has invalid mu {}",
                    body.name, body.mu
                )));
            }
            if body
                .rotation_period_s
                .is_some_and(|period| !period.is_finite() || period <= 0.0)
            {
                return Err(EphemerisError::InvalidBody(format!(
                    "body {} has invalid rotation period",
                    body.name
                )));
            }
            if !body.axial_tilt_rad.is_finite() {
                return Err(EphemerisError::InvalidBody(format!(
                    "body {} has invalid axial tilt",
                    body.name
                )));
            }
            if let Some(parent) = body.parent {
                self.body(parent)?;
                if body.orbit.is_none() {
                    return Err(EphemerisError::InvalidBody(format!(
                        "body {} has a parent but no orbit",
                        body.name
                    )));
                }
            }
        }
        Ok(())
    }

    fn body_state_with_stack(
        &self,
        id: BodyId,
        time: SimTime,
        stack: &mut Vec<BodyId>,
    ) -> Result<BodyState, EphemerisError> {
        if stack.contains(&id) {
            return Err(EphemerisError::Cycle(id));
        }
        let body = self.body(id)?.clone();
        stack.push(id);
        let result = match (body.parent, body.orbit) {
            (Some(parent), Some(orbit)) => {
                let parent_state = self.body_state_with_stack(parent, time, stack)?;
                let (relative_position, relative_velocity) = orbit.state_relative_at(time)?;
                Ok(BodyState {
                    position_inertial: parent_state.position_inertial + relative_position,
                    velocity_inertial: parent_state.velocity_inertial + relative_velocity,
                    orientation: DQuat::IDENTITY,
                    angular_velocity: DVec3::ZERO,
                })
            }
            (None, None) => Ok(BodyState::ORIGIN),
            _ => Err(EphemerisError::InvalidBody(format!(
                "body {} has an incomplete parent/orbit pair",
                body.name
            ))),
        };
        stack.pop();
        result
    }
}

fn solve_kepler(mean_anomaly: f64, eccentricity: f64) -> f64 {
    let reduced_mean_anomaly = mean_anomaly.rem_euclid(TAU);
    let mut eccentric_anomaly = if eccentricity < 0.8 {
        reduced_mean_anomaly
    } else {
        std::f64::consts::PI
    };
    for _ in 0..32 {
        let (sin_e, cos_e) = eccentric_anomaly.sin_cos();
        let correction = (eccentric_anomaly - eccentricity * sin_e - reduced_mean_anomaly)
            / (1.0 - eccentricity * cos_e);
        eccentric_anomaly -= correction;
        if correction.abs() < 1.0e-14 {
            break;
        }
    }
    eccentric_anomaly + mean_anomaly - reduced_mean_anomaly
}

fn perifocal_basis(raan: f64, argument_of_periapsis: f64, inclination: f64) -> (DVec3, DVec3) {
    let (sin_raan, cos_raan) = raan.sin_cos();
    let (sin_arg, cos_arg) = argument_of_periapsis.sin_cos();
    let (sin_inc, cos_inc) = inclination.sin_cos();
    let p_hat = DVec3::new(
        cos_raan * cos_arg - sin_raan * sin_arg * cos_inc,
        sin_raan * cos_arg + cos_raan * sin_arg * cos_inc,
        sin_arg * sin_inc,
    );
    let q_hat = DVec3::new(
        -cos_raan * sin_arg - sin_raan * cos_arg * cos_inc,
        -sin_raan * sin_arg + cos_raan * cos_arg * cos_inc,
        cos_arg * sin_inc,
    );
    (p_hat, q_hat)
}

#[derive(Debug, Clone, PartialEq)]
pub enum EphemerisError {
    UnknownBody(BodyId),
    Cycle(BodyId),
    InvalidOrbit(String),
    InvalidBody(String),
}

impl fmt::Display for EphemerisError {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::UnknownBody(id) => write!(formatter, "unknown body {id:?}"),
            Self::Cycle(id) => write!(formatter, "ephemeris parent cycle through {id:?}"),
            Self::InvalidOrbit(message) => write!(formatter, "invalid orbit: {message}"),
            Self::InvalidBody(message) => write!(formatter, "invalid body: {message}"),
        }
    }
}

impl Error for EphemerisError {}
