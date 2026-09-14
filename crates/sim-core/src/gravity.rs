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

    /// Gravity from a precomputed [`EphemerisFrame`] slice instead of fresh
    /// per-body lookups. Same source order, same checks, same summation —
    /// bitwise identical to [`GravityField::acceleration`] for the same
    /// timestamp. A short slice reports the missing body as unknown rather
    /// than panicking on indexing.
    pub fn acceleration_from_states(
        &self,
        position: DVec3,
        states: &[crate::BodyState],
    ) -> Result<DVec3, GravityError> {
        let mut total = DVec3::ZERO;
        for body_id in &self.source_ids {
            let body = self.ephemeris.body(*body_id)?;
            let state = states
                .get(body_id.index())
                .ok_or(crate::EphemerisError::UnknownBody(*body_id))?;
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

    /// Batch twin of [`GravityField::accelerations`] over one precomputed
    /// [`EphemerisFrame`](crate::EphemerisFrame): the caller evaluates the
    /// frame once per tick, then every target accumulates from the same
    /// slice instead of re-walking parent chains per target per source.
    /// Source `(mu, state index)` pairs resolve once per call, not once per
    /// target: N×S ephemeris lookups become S. Same order, same checks, same
    /// summation — bitwise identical to [`GravityField::accelerations`] for
    /// the same timestamp.
    ///
    /// Granularity note: Rayon dispatches per task, not per flop, and a
    /// thousand 15 ns closures never amortize eighty task spawns. Targets
    /// therefore run in one chunk per worker with an infallible kernel;
    /// outputs are finite-scanned once and only failed lanes pay the
    /// fallible path for their exact error identity.
    pub fn accelerations_from_frame(
        &self,
        positions: &[DVec3],
        states: &[crate::BodyState],
    ) -> Result<Vec<DVec3>, GravityError> {
        let sources = self.resolved_frame_sources(states.len())?;
        if positions.is_empty() {
            return Ok(Vec::new());
        }
        let workers = rayon::current_num_threads().max(1);
        let chunk = positions.len().div_ceil(workers).max(1);
        // Direct slice writes: no per-chunk allocation, no collect-concat.
        let mut out = vec![DVec3::ZERO; positions.len()];
        out.par_chunks_mut(chunk)
            .zip(positions.par_chunks(chunk))
            .for_each(|(out_block, pos_block)| {
                for (slot, position) in out_block.iter_mut().zip(pos_block.iter()) {
                    *slot = accumulate_frame_unchecked(*position, states, &sources);
                }
            });
        for (position, acceleration) in positions.iter().zip(out.iter_mut()) {
            if !acceleration.is_finite() {
                // Rare lane (singularity / non-finite input): rerun fallibly
                // for the exact error the checked path reports.
                *acceleration = accumulate_frame(*position, states, &sources)?;
            }
        }
        Ok(out)
    }

    /// Resolve `(mu, state index)` for every source in accumulation order.
    /// Reports the same unknown-body error as the per-target path for a
    /// short states slice.
    fn resolved_frame_sources(
        &self,
        states_len: usize,
    ) -> Result<Vec<(f64, BodyId)>, GravityError> {
        let mut sources = Vec::with_capacity(self.source_ids.len());
        for body_id in &self.source_ids {
            if body_id.index() >= states_len {
                return Err(crate::EphemerisError::UnknownBody(*body_id).into());
            }
            sources.push((self.ephemeris.body(*body_id)?.mu, *body_id));
        }
        Ok(sources)
    }
}

/// Accumulate point-mass terms from pre-resolved `(mu, state index)` pairs.
/// Same per-source order, checks and summation as
/// [`GravityField::acceleration`]; the caller guarantees every index is in
/// bounds, so a short slice is reported before the parallel loop starts.
fn accumulate_frame(
    position: DVec3,
    states: &[crate::BodyState],
    sources: &[(f64, BodyId)],
) -> Result<DVec3, GravityError> {
    let mut total = DVec3::ZERO;
    for (mu, body_id) in sources {
        let state = &states[body_id.index()];
        let offset = state.position_inertial - position;
        let distance_squared = offset.length_squared();
        if !distance_squared.is_finite() {
            return Err(GravityError::NonFinite { body_id: *body_id });
        }
        if distance_squared == 0.0 {
            return Err(GravityError::Singularity { body_id: *body_id });
        }
        let inverse_distance = distance_squared.sqrt().recip();
        total += offset * (*mu * inverse_distance.powi(3));
    }
    if total.is_finite() {
        Ok(total)
    } else {
        Err(GravityError::NonFinite {
            body_id: sources[0].1,
        })
    }
}

/// Infallible twin of [`accumulate_frame`]: identical operations in
/// identical order, minus the branches. A singular lane evaluates to NaN
/// (`0 * inf`) and non-finite inputs propagate, so callers finite-scan the
/// outputs once and rerun only failed lanes fallibly. Bit-identical output
/// wherever the checked path succeeds.
fn accumulate_frame_unchecked(
    position: DVec3,
    states: &[crate::BodyState],
    sources: &[(f64, BodyId)],
) -> DVec3 {
    let mut total = DVec3::ZERO;
    for (mu, body_id) in sources {
        let offset = states[body_id.index()].position_inertial - position;
        let distance_squared = offset.length_squared();
        let inverse_distance = distance_squared.sqrt().recip();
        total += offset * (*mu * inverse_distance.powi(3));
    }
    total
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
