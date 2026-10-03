//! Backend-neutral internal reaction-wheel actuators.
//!
//! Wheel banks are game-oriented moment actuators: their authored torque
//! ratings are available continuously up to the motor rating. Banks with an
//! authored momentum capacity integrate stored rotor momentum, saturate per
//! axis when the reservoir fills, and drain continuously through biased
//! wheel output (RCS carries the external compensation); banks without one
//! keep the legacy unlimited model. Contact, renderer and power-system
//! backends are intentionally outside this module.

use std::{error::Error, fmt};

use glam::{DMat3, DVec3};
use serde::{Deserialize, Serialize};

use crate::RigidBodyProperties;

/// Authored three-axis reaction-wheel assembly, with wheel axes aligned to the
/// vehicle body axes. Multiple assemblies are allowed and their torque
/// authority is combined per axis.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct ReactionWheelBankSpec {
    pub name: String,
    /// Installed torque rating about body X/Y/Z, in N·m.
    pub max_torque_body_nm: DVec3,
    /// Installed assembly mass, included in vehicle COM and inertia baking.
    pub mass_kg: f64,
    /// Assembly center of mass in the vehicle authoring frame.
    pub position_body_m: DVec3,
    /// Inertia tensor about the assembly center of mass, in body axes.
    pub inertia_body_kg_m2: DMat3,
    /// Quiescent electronics/bearing draw while the bank is enabled (W).
    #[serde(default)]
    pub idle_power_w: f64,
    /// Marginal motor draw per N·m of delivered torque (W/N·m, L1 norm).
    #[serde(default)]
    pub torque_power_w_per_nm: f64,
    /// Rotor momentum capacity about body X/Y/Z (N·m·s). `None` (default)
    /// keeps the legacy KSP-style unlimited model; `Some` integrates stored
    /// momentum and saturates per axis when the reservoir fills.
    #[serde(default)]
    pub momentum_capacity_nms: Option<DVec3>,
    /// Rotor spin inertia per body axis (kg·m²). Enables rotor-speed
    /// telemetry (`stored_momentum / inertia`); `None` (default) reports
    /// no speeds. Independent of the assembly inertia tensor above,
    /// which describes the mounted box, not the spinning rotor.
    #[serde(default)]
    pub rotor_inertia_kg_m2: Option<DVec3>,
}

impl ReactionWheelBankSpec {
    pub fn validate(&self) -> Result<(), ReactionWheelError> {
        if self.name.trim().is_empty()
            || !self.max_torque_body_nm.is_finite()
            || !self.mass_kg.is_finite()
            || self.mass_kg <= 0.0
            || !self.position_body_m.is_finite()
            || self.max_torque_body_nm.min_element() < 0.0
        {
            return Err(ReactionWheelError::InvalidConfiguration(format!(
                "'{}' has a missing name or non-finite/negative rating, mass, or position",
                self.name
            )));
        }
        if !self.idle_power_w.is_finite() || self.idle_power_w < 0.0 {
            return Err(ReactionWheelError::InvalidConfiguration(format!(
                "'{}' idle power must be finite and non-negative",
                self.name
            )));
        }
        if !self.torque_power_w_per_nm.is_finite() || self.torque_power_w_per_nm < 0.0 {
            return Err(ReactionWheelError::InvalidConfiguration(format!(
                "'{}' torque power coefficient must be finite and non-negative",
                self.name
            )));
        }
        if let Some(capacity) = self.momentum_capacity_nms
            && (!capacity.is_finite() || capacity.min_element() < 0.0)
        {
            return Err(ReactionWheelError::InvalidConfiguration(format!(
                "'{}' momentum capacity must be finite and non-negative",
                self.name
            )));
        }
        if let Some(inertia) = self.rotor_inertia_kg_m2
            && (!inertia.is_finite() || inertia.min_element() <= 0.0)
        {
            return Err(ReactionWheelError::InvalidConfiguration(format!(
                "'{}' rotor inertia must be finite and positive",
                self.name
            )));
        }
        RigidBodyProperties::new(self.mass_kg, self.inertia_body_kg_m2).map_err(|error| {
            ReactionWheelError::InvalidConfiguration(format!(
                "'{}' has invalid mass properties: {error}",
                self.name
            ))
        })?;
        Ok(())
    }

    /// Bus load for one step's delivered torque: quiescent draw plus a
    /// marginal motor term linear in the L1 torque magnitude. Both
    /// coefficients are authored per bank, so this is actuator data rather
    /// than a whole-craft tuning constant.
    pub fn electrical_power_w(&self, delivered_torque_body_nm: DVec3) -> f64 {
        debug_assert!(delivered_torque_body_nm.is_finite());
        (self.idle_power_w
            + self.torque_power_w_per_nm
                * (delivered_torque_body_nm.x.abs()
                    + delivered_torque_body_nm.y.abs()
                    + delivered_torque_body_nm.z.abs()))
        .max(0.0)
    }

    /// Rotor angular velocity per body axis (rad/s) from stored momentum:
    /// `ω = H / I_rotor`. `None` without authored rotor inertia — speeds
    /// are telemetry only and never feed back into the allocation.
    pub fn rotor_speed_body_rps(&self, state: &ReactionWheelState) -> Option<DVec3> {
        self.rotor_inertia_kg_m2.map(|inertia| {
            DVec3::new(
                state.stored_momentum_body_nms.x / inertia.x,
                state.stored_momentum_body_nms.y / inertia.y,
                state.stored_momentum_body_nms.z / inertia.z,
            )
        })
    }

    /// Saturation fraction of the momentum reservoir in [0, 1] per axis
    /// (`|H| / capacity`); 0.0 without an authored capacity.
    pub fn momentum_saturation_fraction(&self, state: &ReactionWheelState) -> DVec3 {
        match self.momentum_capacity_nms {
            None => DVec3::ZERO,
            Some(capacity) => DVec3::new(
                saturation_axis(state.stored_momentum_body_nms.x, capacity.x),
                saturation_axis(state.stored_momentum_body_nms.y, capacity.y),
                saturation_axis(state.stored_momentum_body_nms.z, capacity.z),
            ),
        }
    }
}

/// One axis of reservoir fill, clamped to [0, 1]; zero capacity reads full
/// only when momentum is actually stored there (avoids divide-by-zero
/// while still flagging a wedged bank).
fn saturation_axis(stored_nms: f64, capacity_nms: f64) -> f64 {
    if capacity_nms <= 0.0 {
        return if stored_nms == 0.0 { 0.0 } else { 1.0 };
    }
    (stored_nms.abs() / capacity_nms).clamp(0.0, 1.0)
}

/// Per-bank rotor telemetry snapshot. Speeds are `None` without authored
/// rotor inertia; saturation is zero without a momentum capacity.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ReactionWheelBankTelemetry {
    pub stored_momentum_body_nms: DVec3,
    pub rotor_speed_body_rps: Option<DVec3>,
    pub momentum_saturation_fraction: DVec3,
}

/// Telemetry for every installed bank, parallel to `specs`/`states`.
pub fn reaction_wheel_bank_telemetry(
    specs: &[ReactionWheelBankSpec],
    states: &[ReactionWheelState],
) -> Result<Vec<ReactionWheelBankTelemetry>, ReactionWheelError> {
    if states.len() != specs.len() {
        return Err(ReactionWheelError::InvalidState(
            "momentum states must match the installed banks".into(),
        ));
    }
    specs
        .iter()
        .zip(states.iter())
        .map(|(spec, state)| {
            spec.validate()?;
            state.validate()?;
            Ok(ReactionWheelBankTelemetry {
                stored_momentum_body_nms: state.stored_momentum_body_nms,
                rotor_speed_body_rps: spec.rotor_speed_body_rps(state),
                momentum_saturation_fraction: spec.momentum_saturation_fraction(state),
            })
        })
        .collect()
}

/// Motor heat rejected by one bank over a step (W): bus electrical draw
/// minus the rotor kinetic-energy rate, `KE = Σ H²/(2·I)` over axes with
/// authored inertia. Axes without rotor inertia (or banks without momentum
/// tracking) treat the full draw as heat — conservative and documented.
/// Clamped at zero: this model has no regenerative braking, so negative
/// dissipation would mean unphysical energy creation.
///
/// Pure function of the pre-step state plus the delivered torque; does not
/// mutate momentum (the allocator owns integration).
pub fn reaction_wheel_step_heat_w(
    spec: &ReactionWheelBankSpec,
    state: &ReactionWheelState,
    delivered_torque_body_nm: DVec3,
    dt_s: f64,
    electrical_power_w: f64,
) -> Result<f64, ReactionWheelError> {
    spec.validate()?;
    state.validate()?;
    if !delivered_torque_body_nm.is_finite() {
        return Err(ReactionWheelError::InvalidState(
            "delivered torque must be finite".into(),
        ));
    }
    if !dt_s.is_finite() || dt_s <= 0.0 {
        return Err(ReactionWheelError::InvalidState(
            "step duration must be finite and positive".into(),
        ));
    }
    if !electrical_power_w.is_finite() || electrical_power_w < 0.0 {
        return Err(ReactionWheelError::InvalidState(
            "electrical draw must be finite and non-negative".into(),
        ));
    }
    let kinetic_j = |momentum: DVec3| -> f64 {
        match spec.rotor_inertia_kg_m2 {
            None => 0.0,
            Some(inertia) => {
                0.5 * (momentum.x * momentum.x / inertia.x
                    + momentum.y * momentum.y / inertia.y
                    + momentum.z * momentum.z / inertia.z)
            }
        }
    };
    let before_j = kinetic_j(state.stored_momentum_body_nms);
    let after_j = kinetic_j(state.stored_momentum_body_nms + delivered_torque_body_nm * dt_s);
    let heat_w = electrical_power_w - (after_j - before_j) / dt_s;
    if !heat_w.is_finite() {
        return Err(ReactionWheelError::InvalidState(
            "wheel heat balance is non-finite".into(),
        ));
    }
    Ok(heat_w.max(0.0))
}

/// One reaction-wheel allocation result for a physics step.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ReactionWheelAllocation {
    pub delivered_torque_body_nm: DVec3,
    pub saturated: bool,
    /// True when at least one bank hit its momentum capacity this step
    /// (as opposed to the instantaneous motor-torque rating).
    pub momentum_saturated: bool,
}

/// Authoritative stored rotor momentum for one bank (N·m·s, body axes).
/// Integrated by the momentum-aware allocator; zeroed on bank disable is
/// NOT automatic — a disabled bank holds its momentum until desaturated.
#[derive(Debug, Clone, Copy, Default, PartialEq, Serialize, Deserialize)]
pub struct ReactionWheelState {
    pub stored_momentum_body_nms: DVec3,
}

impl ReactionWheelState {
    pub fn validate(&self) -> Result<(), ReactionWheelError> {
        if !self.stored_momentum_body_nms.is_finite() {
            return Err(ReactionWheelError::InvalidState(
                "wheel stored momentum must be finite".into(),
            ));
        }
        Ok(())
    }
}

/// Zero-momentum states for a bank list, parallel to `specs`.
pub fn initial_reaction_wheel_states(
    specs: &[ReactionWheelBankSpec],
) -> Result<Vec<ReactionWheelState>, ReactionWheelError> {
    for spec in specs {
        spec.validate()?;
    }
    Ok(vec![ReactionWheelState::default(); specs.len()])
}

/// Allocate requested body moment to aligned reaction-wheel banks. A bank's
/// rating is a per-axis actuator capability, not a consumable momentum
/// reservoir; as in KSP's gameplay model, sustained rotation does not saturate.
pub fn allocate_reaction_wheels(
    specs: &[ReactionWheelBankSpec],
    requested_torque_body_nm: DVec3,
) -> Result<ReactionWheelAllocation, ReactionWheelError> {
    allocate_reaction_wheels_masked(specs, None, requested_torque_body_nm)
}

/// Allocate to only the enabled reaction-wheel banks. The mask is parallel to
/// `specs`; disabled banks remain installed but provide no torque authority.
pub fn allocate_reaction_wheels_with_enabled_banks(
    specs: &[ReactionWheelBankSpec],
    enabled_banks: &[bool],
    requested_torque_body_nm: DVec3,
) -> Result<ReactionWheelAllocation, ReactionWheelError> {
    allocate_reaction_wheels_masked(specs, Some(enabled_banks), requested_torque_body_nm)
}

fn allocate_reaction_wheels_masked(
    specs: &[ReactionWheelBankSpec],
    enabled_banks: Option<&[bool]>,
    requested_torque_body_nm: DVec3,
) -> Result<ReactionWheelAllocation, ReactionWheelError> {
    if !requested_torque_body_nm.is_finite() {
        return Err(ReactionWheelError::InvalidState(
            "requested torque must be finite".into(),
        ));
    }
    if let Some(enabled) = enabled_banks
        && enabled.len() != specs.len()
    {
        return Err(ReactionWheelError::InvalidState(
            "reaction-wheel enable mask must match the installed banks".into(),
        ));
    }
    for spec in specs {
        spec.validate()?;
    }
    let total_rating = specs
        .iter()
        .enumerate()
        .filter(|(index, _)| enabled_banks.is_none_or(|enabled| enabled[*index]))
        .map(|(_, spec)| spec.max_torque_body_nm)
        .sum::<DVec3>();
    let delivered = DVec3::new(
        requested_torque_body_nm
            .x
            .clamp(-total_rating.x, total_rating.x),
        requested_torque_body_nm
            .y
            .clamp(-total_rating.y, total_rating.y),
        requested_torque_body_nm
            .z
            .clamp(-total_rating.z, total_rating.z),
    );
    let residual = requested_torque_body_nm - delivered;
    Ok(ReactionWheelAllocation {
        delivered_torque_body_nm: delivered,
        saturated: residual.length_squared() > 1.0e-12,
        momentum_saturated: false,
    })
}

/// Allocate with per-bank rotor-momentum integration. Banks with
/// `momentum_capacity_nms: None` behave exactly like the stateless path;
/// banks with a capacity clamp their share so
/// `|stored + delivered_share * dt| <= capacity` per axis, and the stored
/// momentum integrates `H += delivered * dt`.
///
/// `states` is parallel to `specs` and updated in place; disabled banks hold
/// their momentum. `dt_s` must be finite and positive.
#[allow(clippy::too_many_arguments)]
pub fn allocate_reaction_wheels_with_momentum(
    specs: &[ReactionWheelBankSpec],
    enabled_banks: &[bool],
    states: &mut [ReactionWheelState],
    requested_torque_body_nm: DVec3,
    dt_s: f64,
) -> Result<ReactionWheelAllocation, ReactionWheelError> {
    if !requested_torque_body_nm.is_finite() {
        return Err(ReactionWheelError::InvalidState(
            "requested torque must be finite".into(),
        ));
    }
    if !dt_s.is_finite() || dt_s <= 0.0 {
        return Err(ReactionWheelError::InvalidState(
            "momentum step duration must be finite and positive".into(),
        ));
    }
    if enabled_banks.len() != specs.len() || states.len() != specs.len() {
        return Err(ReactionWheelError::InvalidState(
            "enable mask and momentum states must match the installed banks".into(),
        ));
    }
    for (spec, state) in specs.iter().zip(states.iter()) {
        spec.validate()?;
        state.validate()?;
        // Fail closed on over-capacity momentum (reachable via deserialized
        // state or a capacity-lowering asset edit): without this, the
        // reservoir clamp below would emit uncommanded torque up to |H|/dt.
        if let Some(capacity) = spec.momentum_capacity_nms {
            let stored = state.stored_momentum_body_nms.abs();
            if stored.x > capacity.x || stored.y > capacity.y || stored.z > capacity.z {
                return Err(ReactionWheelError::InvalidState(
                    "stored momentum exceeds bank capacity".into(),
                ));
            }
        }
    }
    // Rating clamp on the aggregate first (same motor authority as the
    // stateless path), then split across enabled banks in proportion to
    // per-axis ratings.
    let total_rating = specs
        .iter()
        .enumerate()
        .filter(|(index, _)| enabled_banks[*index])
        .map(|(_, spec)| spec.max_torque_body_nm)
        .sum::<DVec3>();
    let rated = DVec3::new(
        requested_torque_body_nm
            .x
            .clamp(-total_rating.x, total_rating.x),
        requested_torque_body_nm
            .y
            .clamp(-total_rating.y, total_rating.y),
        requested_torque_body_nm
            .z
            .clamp(-total_rating.z, total_rating.z),
    );
    let mut delivered = DVec3::ZERO;
    let mut momentum_saturated = false;
    for ((spec, enabled), state) in specs.iter().zip(enabled_banks).zip(states.iter_mut()) {
        if !enabled {
            continue;
        }
        let share = DVec3::new(
            share_of(spec.max_torque_body_nm.x, total_rating.x),
            share_of(spec.max_torque_body_nm.y, total_rating.y),
            share_of(spec.max_torque_body_nm.z, total_rating.z),
        );
        let mut bank_torque = rated * share;
        if let Some(capacity) = spec.momentum_capacity_nms {
            let headroom = capacity - state.stored_momentum_body_nms;
            let footroom = -capacity - state.stored_momentum_body_nms;
            // Clamp the bank share so the integrated momentum stays inside
            // the reservoir: footroom/dt <= T <= headroom/dt per axis.
            let limited = DVec3::new(
                (bank_torque.x * dt_s).clamp(footroom.x, headroom.x) / dt_s,
                (bank_torque.y * dt_s).clamp(footroom.y, headroom.y) / dt_s,
                (bank_torque.z * dt_s).clamp(footroom.z, headroom.z) / dt_s,
            );
            if limited != bank_torque {
                momentum_saturated = true;
            }
            // The reservoir clamp can push outside motor authority when the
            // state sits at the wall; re-clamp so delivered torque never
            // exceeds the per-axis rating.
            bank_torque = DVec3::new(
                limited
                    .x
                    .clamp(-spec.max_torque_body_nm.x, spec.max_torque_body_nm.x),
                limited
                    .y
                    .clamp(-spec.max_torque_body_nm.y, spec.max_torque_body_nm.y),
                limited
                    .z
                    .clamp(-spec.max_torque_body_nm.z, spec.max_torque_body_nm.z),
            );
        }
        if !bank_torque.is_finite() {
            return Err(ReactionWheelError::InvalidState(
                "momentum-clamped bank torque is non-finite".into(),
            ));
        }
        state.stored_momentum_body_nms += bank_torque * dt_s;
        delivered += bank_torque;
    }
    let residual = requested_torque_body_nm - delivered;
    Ok(ReactionWheelAllocation {
        delivered_torque_body_nm: delivered,
        saturated: residual.length_squared() > 1.0e-12,
        momentum_saturated,
    })
}

/// Proportional rating share for splitting an aggregate torque across
/// banks. A zero total rating carries no torque on that axis.
fn share_of(rating: f64, total: f64) -> f64 {
    if total > 0.0 && rating.is_finite() && total.is_finite() {
        (rating / total).clamp(0.0, 1.0)
    } else {
        0.0
    }
}

/// Desaturation demand: body torque that would drive stored rotor momentum
/// toward zero. The caller biases the WHEEL target by this demand (so the
/// reservoir drains through the wheels) and routes the remaining residual
/// through RCS; net vehicle torque stays as commanded. Only banks with a
/// momentum capacity participate; unlimited banks never need dumping.
/// `max_unload_torque_nm` bounds the TOTAL unload authority per axis: it is
/// split evenly across participating banks so multi-bank totals cannot
/// exceed the caller's bound.
pub fn momentum_unload_demand(
    specs: &[ReactionWheelBankSpec],
    states: &[ReactionWheelState],
    dt_s: f64,
    max_unload_torque_nm: f64,
) -> Result<DVec3, ReactionWheelError> {
    if !dt_s.is_finite() || dt_s <= 0.0 {
        return Err(ReactionWheelError::InvalidState(
            "momentum step duration must be finite and positive".into(),
        ));
    }
    if !max_unload_torque_nm.is_finite() || max_unload_torque_nm < 0.0 {
        return Err(ReactionWheelError::InvalidState(
            "unload torque limit must be finite and non-negative".into(),
        ));
    }
    if states.len() != specs.len() {
        return Err(ReactionWheelError::InvalidState(
            "momentum states must match the installed banks".into(),
        ));
    }
    let mut demand = DVec3::ZERO;
    let participants = specs
        .iter()
        .filter(|spec| spec.momentum_capacity_nms.is_some())
        .count();
    // Split the total unload authority across participating banks so the
    // summed demand respects the caller's bound on every axis.
    let per_bank_limit = if participants > 0 {
        max_unload_torque_nm / participants as f64
    } else {
        0.0
    };
    for (spec, state) in specs.iter().zip(states.iter()) {
        spec.validate()?;
        state.validate()?;
        if spec.momentum_capacity_nms.is_none() {
            continue;
        }
        let stored = state.stored_momentum_body_nms;
        // Torque opposing the stored momentum, sized to zero it in one
        // step but capped at this bank's share of the unload authority.
        let axis_demand = DVec3::new(
            (-stored.x / dt_s).clamp(-per_bank_limit, per_bank_limit),
            (-stored.y / dt_s).clamp(-per_bank_limit, per_bank_limit),
            (-stored.z / dt_s).clamp(-per_bank_limit, per_bank_limit),
        );
        demand += axis_demand;
    }
    Ok(demand)
}

#[derive(Debug, Clone, PartialEq)]
pub enum ReactionWheelError {
    InvalidConfiguration(String),
    InvalidState(String),
}

impl fmt::Display for ReactionWheelError {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::InvalidConfiguration(message) => {
                write!(formatter, "invalid reaction-wheel configuration: {message}")
            }
            Self::InvalidState(message) => {
                write!(formatter, "invalid reaction-wheel state: {message}")
            }
        }
    }
}

impl Error for ReactionWheelError {}

#[cfg(test)]
mod tests {
    use super::*;

    fn bank(name: &str, torque: DVec3) -> ReactionWheelBankSpec {
        ReactionWheelBankSpec {
            name: name.into(),
            max_torque_body_nm: torque,
            mass_kg: 10.0,
            position_body_m: DVec3::ZERO,
            inertia_body_kg_m2: DMat3::IDENTITY,
            idle_power_w: 0.0,
            torque_power_w_per_nm: 0.0,
            momentum_capacity_nms: None,
            rotor_inertia_kg_m2: None,
        }
    }

    #[test]
    fn rated_torque_is_available_indefinitely_without_momentum_saturation() {
        let specs = [bank("main", DVec3::splat(20.0))];
        for _ in 0..10_000 {
            let result = allocate_reaction_wheels(&specs, DVec3::X * 15.0).unwrap();
            assert_eq!(result.delivered_torque_body_nm, DVec3::X * 15.0);
            assert!(!result.saturated);
        }
    }

    #[test]
    fn multiple_banks_combine_torque_ratings_per_axis() {
        let specs = [
            bank("small", DVec3::X * 10.0),
            bank("large", DVec3::X * 30.0),
        ];
        let result = allocate_reaction_wheels(&specs, DVec3::X * 20.0).unwrap();
        assert_eq!(result.delivered_torque_body_nm, DVec3::X * 20.0);
        assert!(!result.saturated);
    }

    #[test]
    fn disabled_bank_does_not_contribute_torque_rating() {
        let specs = [
            bank("offline", DVec3::X * 10.0),
            bank("online", DVec3::X * 30.0),
        ];
        let result =
            allocate_reaction_wheels_with_enabled_banks(&specs, &[false, true], DVec3::X * 20.0)
                .unwrap();
        assert_eq!(result.delivered_torque_body_nm, DVec3::X * 20.0);
        assert!(!result.saturated);

        let saturated =
            allocate_reaction_wheels_with_enabled_banks(&specs, &[false, true], DVec3::X * 50.0)
                .unwrap();
        assert_eq!(saturated.delivered_torque_body_nm, DVec3::X * 30.0);
        assert!(saturated.saturated);
    }

    #[test]
    fn requested_moment_saturates_only_at_the_configured_motor_rating() {
        let specs = [bank("main", DVec3::new(8.0, 12.0, 20.0))];
        let result = allocate_reaction_wheels(&specs, DVec3::new(100.0, -100.0, 10.0)).unwrap();
        assert_eq!(
            result.delivered_torque_body_nm,
            DVec3::new(8.0, -12.0, 10.0)
        );
        assert!(result.saturated);
    }

    #[test]
    fn bus_load_is_idle_plus_marginal_torque_draw() {
        let mut spec = bank("main", DVec3::splat(20.0));
        spec.idle_power_w = 5.0;
        spec.torque_power_w_per_nm = 0.5;
        // L1(|10| + |-4| + |0|) = 14 N·m -> 5 + 7 = 12 W.
        assert!((spec.electrical_power_w(DVec3::new(10.0, -4.0, 0.0)) - 12.0).abs() < 1.0e-12);
        assert_eq!(spec.electrical_power_w(DVec3::ZERO), 5.0);
    }

    #[test]
    fn negative_power_authoring_fails_closed() {
        let mut spec = bank("main", DVec3::splat(20.0));
        spec.idle_power_w = -1.0;
        assert!(spec.validate().is_err());
        spec.idle_power_w = 0.0;
        spec.torque_power_w_per_nm = f64::NAN;
        assert!(spec.validate().is_err());
    }

    #[test]
    fn unlimited_banks_match_the_stateless_path_exactly() {
        let specs = [
            bank("small", DVec3::X * 10.0),
            bank("large", DVec3::X * 30.0),
        ];
        let mut states = initial_reaction_wheel_states(&specs).unwrap();
        let result = allocate_reaction_wheels_with_momentum(
            &specs,
            &[true, true],
            &mut states,
            DVec3::X * 20.0,
            0.02,
        )
        .unwrap();
        assert_eq!(result.delivered_torque_body_nm, DVec3::X * 20.0);
        assert!(!result.saturated);
        assert!(!result.momentum_saturated);
        // 20 N·m over dt=0.02 s stores 0.4 N·m·s split 0.1/0.3 by rating.
        assert!((states[0].stored_momentum_body_nms.x - 0.1).abs() < 1.0e-12);
        assert!((states[1].stored_momentum_body_nms.x - 0.3).abs() < 1.0e-12);
    }

    #[test]
    fn sustained_torque_fills_the_reservoir_then_saturates() {
        let mut spec = bank("main", DVec3::splat(100.0));
        spec.momentum_capacity_nms = Some(DVec3::splat(1.0));
        let specs = [spec];
        let mut states = initial_reaction_wheel_states(&specs).unwrap();
        // 8 N·m for 0.125 s stores exactly 1.0 N·m·s: one full step, then
        // the reservoir is full and the next step delivers nothing.
        let full = allocate_reaction_wheels_with_momentum(
            &specs,
            &[true],
            &mut states,
            DVec3::X * 8.0,
            0.125,
        )
        .unwrap();
        assert_eq!(full.delivered_torque_body_nm, DVec3::X * 8.0);
        assert!(!full.saturated);
        assert!(!full.momentum_saturated);
        assert_eq!(states[0].stored_momentum_body_nms, DVec3::X);
        let saturated = allocate_reaction_wheels_with_momentum(
            &specs,
            &[true],
            &mut states,
            DVec3::X * 8.0,
            0.125,
        )
        .unwrap();
        assert_eq!(saturated.delivered_torque_body_nm, DVec3::ZERO);
        assert!(saturated.saturated);
        assert!(saturated.momentum_saturated);
        // Opposite torque drains the reservoir back toward zero.
        let drained = allocate_reaction_wheels_with_momentum(
            &specs,
            &[true],
            &mut states,
            DVec3::NEG_X * 8.0,
            0.125,
        )
        .unwrap();
        assert_eq!(drained.delivered_torque_body_nm, DVec3::NEG_X * 8.0);
        assert!(!drained.momentum_saturated);
        assert_eq!(states[0].stored_momentum_body_nms, DVec3::ZERO);
    }

    #[test]
    fn unload_demand_drives_stored_momentum_toward_zero() {
        let mut spec = bank("main", DVec3::splat(100.0));
        spec.momentum_capacity_nms = Some(DVec3::splat(10.0));
        let specs = [spec];
        let states = [ReactionWheelState {
            stored_momentum_body_nms: DVec3::new(4.0, -1.0, 0.0),
        }];
        let demand = momentum_unload_demand(&specs, &states, 0.02, 50.0).unwrap();
        // -H/dt = (-200, +50, 0), capped at 50 N·m per axis.
        assert_eq!(demand, DVec3::new(-50.0, 50.0, 0.0));
        let gentle = momentum_unload_demand(&specs, &states, 2.0, 50.0).unwrap();
        assert_eq!(gentle, DVec3::new(-2.0, 0.5, 0.0));
    }

    #[test]
    fn unload_authority_is_split_across_capped_banks() {
        let mut first = bank("fore", DVec3::splat(100.0));
        first.momentum_capacity_nms = Some(DVec3::splat(10.0));
        let mut second = bank("aft", DVec3::splat(100.0));
        second.momentum_capacity_nms = Some(DVec3::splat(10.0));
        let specs = [first, second];
        let states = [
            ReactionWheelState {
                stored_momentum_body_nms: DVec3::new(4.0, 0.0, 0.0),
            },
            ReactionWheelState {
                stored_momentum_body_nms: DVec3::new(4.0, 0.0, 0.0),
            },
        ];
        // Each bank wants -200 N·m on X; the 50 N·m total authority splits
        // 25/25 instead of summing to 100.
        let demand = momentum_unload_demand(&specs, &states, 0.02, 50.0).unwrap();
        assert_eq!(demand, DVec3::new(-50.0, 0.0, 0.0));
    }

    #[test]
    fn over_capacity_momentum_and_wall_torque_fail_closed() {
        let mut spec = bank("main", DVec3::splat(100.0));
        spec.momentum_capacity_nms = Some(DVec3::splat(10.0));
        let specs = [spec];
        // Over-capacity state (reachable via deserialization) fails closed
        // instead of emitting uncommanded torque up to |H|/dt.
        let mut states = [ReactionWheelState {
            stored_momentum_body_nms: DVec3::new(12.0, 0.0, 0.0),
        }];
        assert!(
            allocate_reaction_wheels_with_momentum(&specs, &[true], &mut states, DVec3::ZERO, 0.02)
                .is_err()
        );
        // At exactly the wall, delivered torque never exceeds the rating.
        states[0].stored_momentum_body_nms = DVec3::new(10.0, 0.0, 0.0);
        let pinned = allocate_reaction_wheels_with_momentum(
            &specs,
            &[true],
            &mut states,
            DVec3::X * 1_000.0,
            0.02,
        )
        .unwrap();
        assert!(pinned.momentum_saturated);
        assert!(pinned.delivered_torque_body_nm.x.abs() <= 100.0);
    }

    #[test]
    fn bad_momentum_authoring_and_steps_fail_closed() {
        let mut spec = bank("main", DVec3::splat(20.0));
        spec.momentum_capacity_nms = Some(DVec3::new(1.0, -1.0, 1.0));
        assert!(spec.validate().is_err());
        spec.momentum_capacity_nms = Some(DVec3::splat(f64::NAN));
        assert!(spec.validate().is_err());
        let specs = [bank("main", DVec3::splat(20.0))];
        let mut states = initial_reaction_wheel_states(&specs).unwrap();
        assert!(
            allocate_reaction_wheels_with_momentum(&specs, &[true], &mut states, DVec3::X, 0.0)
                .is_err()
        );
        assert!(
            allocate_reaction_wheels_with_momentum(
                &specs,
                &[true],
                &mut states,
                DVec3::X,
                f64::NAN
            )
            .is_err()
        );
        assert!(momentum_unload_demand(&specs, &states, 0.02, -1.0).is_err());
    }

    fn spinning_bank() -> (ReactionWheelBankSpec, ReactionWheelState) {
        let mut spec = bank("main", DVec3::splat(100.0));
        spec.momentum_capacity_nms = Some(DVec3::splat(10.0));
        spec.rotor_inertia_kg_m2 = Some(DVec3::splat(0.5));
        let state = ReactionWheelState {
            stored_momentum_body_nms: DVec3::new(4.0, -2.0, 0.0),
        };
        (spec, state)
    }

    #[test]
    fn rotor_speed_is_momentum_over_inertia() {
        let (spec, state) = spinning_bank();
        assert_eq!(
            spec.rotor_speed_body_rps(&state),
            Some(DVec3::new(8.0, -4.0, 0.0))
        );
        // No authored inertia: no speeds, but saturation still works.
        let mut plain = bank("plain", DVec3::splat(100.0));
        plain.momentum_capacity_nms = Some(DVec3::splat(10.0));
        assert_eq!(plain.rotor_speed_body_rps(&state), None);
        assert_eq!(
            plain.momentum_saturation_fraction(&state),
            DVec3::new(0.4, 0.2, 0.0)
        );
        // Unlimited banks report neither speeds (without inertia) nor fill.
        let free = bank("free", DVec3::splat(100.0));
        assert_eq!(free.rotor_speed_body_rps(&state), None);
        assert_eq!(free.momentum_saturation_fraction(&state), DVec3::ZERO);
    }

    #[test]
    fn bank_telemetry_reports_speeds_and_fill_per_bank() {
        let (spec, state) = spinning_bank();
        let plain = bank("plain", DVec3::splat(100.0));
        let report =
            reaction_wheel_bank_telemetry(&[spec, plain], &[state, ReactionWheelState::default()])
                .unwrap();
        assert_eq!(report.len(), 2);
        assert_eq!(
            report[0].stored_momentum_body_nms,
            DVec3::new(4.0, -2.0, 0.0)
        );
        assert_eq!(
            report[0].rotor_speed_body_rps,
            Some(DVec3::new(8.0, -4.0, 0.0))
        );
        assert_eq!(
            report[0].momentum_saturation_fraction,
            DVec3::new(0.4, 0.2, 0.0)
        );
        assert_eq!(report[1].rotor_speed_body_rps, None);
    }

    #[test]
    fn bad_rotor_inertia_and_mismatched_telemetry_fail_closed() {
        let mut spec = bank("main", DVec3::splat(20.0));
        spec.rotor_inertia_kg_m2 = Some(DVec3::new(0.5, 0.0, 0.5));
        assert!(spec.validate().is_err());
        spec.rotor_inertia_kg_m2 = Some(DVec3::splat(f64::INFINITY));
        assert!(spec.validate().is_err());
        let specs = [bank("main", DVec3::splat(20.0))];
        assert!(reaction_wheel_bank_telemetry(&specs, &[]).is_err());
    }

    #[test]
    fn step_heat_is_draw_minus_rotor_energy_rate() {
        // Rotor I=0.5, H=4 on X: KE=16 J. Delivering 8 N·m for 0.125 s
        // stores 1.0 more (H=5, KE=25 J): rate 72 W. A 100 W draw rejects
        // 28 W; a 50 W draw would imply regen and clamps to zero.
        let (spec, state) = spinning_bank();
        let heat = reaction_wheel_step_heat_w(&spec, &state, DVec3::X * 8.0, 0.125, 100.0).unwrap();
        assert!((heat - 28.0).abs() < 1.0e-9, "heat was {heat}");
        let clamped =
            reaction_wheel_step_heat_w(&spec, &state, DVec3::X * 8.0, 0.125, 50.0).unwrap();
        assert_eq!(clamped, 0.0);
        // No inertia/track: the full draw counts as heat.
        let plain = bank("plain", DVec3::splat(100.0));
        let idle = ReactionWheelState::default();
        assert_eq!(
            reaction_wheel_step_heat_w(&plain, &idle, DVec3::X * 8.0, 0.125, 12.5).unwrap(),
            12.5
        );
        assert!(reaction_wheel_step_heat_w(&spec, &state, DVec3::X * 8.0, 0.0, 100.0).is_err());
        assert!(reaction_wheel_step_heat_w(&spec, &state, DVec3::X * 8.0, 0.125, -1.0).is_err());
    }
}
