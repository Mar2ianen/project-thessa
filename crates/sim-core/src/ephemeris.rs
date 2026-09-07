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

    pub fn gravity_sources(&self) -> impl Iterator<Item = &BakedBody> {
        self.bodies.iter().filter(|body| body.gravity_source)
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
