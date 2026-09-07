use std::{error::Error, fmt};

use glam::DVec3;
use rayon::prelude::*;

use crate::{BakedEphemeris, BodyId, SimTime};

/// Runtime gravity view. The ephemeris remains the sole source of moving-body
/// positions; no SOI switching is performed.
pub struct GravityField<'a> {
    ephemeris: &'a BakedEphemeris,
    source_ids: Vec<BodyId>,
}

impl<'a> GravityField<'a> {
    pub fn from_ephemeris(ephemeris: &'a BakedEphemeris) -> Self {
        let mut source_ids: Vec<_> = ephemeris.gravity_sources().map(|body| body.id).collect();
        source_ids.sort_unstable();
        Self {
            ephemeris,
            source_ids,
        }
    }

    pub fn source_count(&self) -> usize {
        self.source_ids.len()
    }

    pub fn acceleration(&self, position: DVec3, time: SimTime) -> Result<DVec3, GravityError> {
        let mut total = DVec3::ZERO;
        for body_id in &self.source_ids {
            let body = self.ephemeris.body(*body_id)?;
            let state = self.ephemeris.body_state(*body_id, time)?;
            let offset = state.position_inertial - position;
            let distance_squared = offset.length_squared();
            if !distance_squared.is_finite() {
                return Err(GravityError::NonFinite { body_id: *body_id });
            }
            if distance_squared == 0.0 {
                return Err(GravityError::Singularity { body_id: *body_id });
            }
            let inverse_distance = distance_squared.sqrt().recip();
            total += offset * (body.mu * inverse_distance.powi(3));
        }
        if total.is_finite() {
            Ok(total)
        } else {
            Err(GravityError::NonFinite {
                body_id: self.source_ids[0],
            })
        }
    }

    pub fn potential(&self, position: DVec3, time: SimTime) -> Result<f64, GravityError> {
        let mut total = 0.0;
        for body_id in &self.source_ids {
            let body = self.ephemeris.body(*body_id)?;
            let state = self.ephemeris.body_state(*body_id, time)?;
            let distance = (state.position_inertial - position).length();
            if !distance.is_finite() {
                return Err(GravityError::NonFinite { body_id: *body_id });
            }
            if distance == 0.0 {
                return Err(GravityError::Singularity { body_id: *body_id });
            }
            total -= body.mu / distance;
        }
        Ok(total)
    }

    /// Ordered output is retained even though each independent state is
    /// evaluated in parallel. Per-state source accumulation order is stable,
    /// making replay behaviour independent of the worker count.
    pub fn accelerations(
        &self,
        positions: &[DVec3],
        time: SimTime,
    ) -> Result<Vec<DVec3>, GravityError> {
        positions
            .par_iter()
            .map(|position| self.acceleration(*position, time))
            .collect()
    }
}

#[derive(Debug, Clone, PartialEq)]
pub enum GravityError {
    Ephemeris(crate::EphemerisError),
    Singularity { body_id: BodyId },
    NonFinite { body_id: BodyId },
}

impl fmt::Display for GravityError {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Ephemeris(error) => error.fmt(formatter),
            Self::Singularity { body_id } => {
                write!(formatter, "gravity singularity at {body_id:?}")
            }
            Self::NonFinite { body_id } => {
                write!(
                    formatter,
                    "non-finite gravity contribution from {body_id:?}"
                )
            }
        }
    }
}

impl Error for GravityError {}

impl From<crate::EphemerisError> for GravityError {
    fn from(error: crate::EphemerisError) -> Self {
        Self::Ephemeris(error)
    }
}
