//! Contact-active runtime seam for the authoritative flight loop.
//!
//! The authoritative flight equations stay in `thessa-sim-core`; the contact
//! solver (`thessa-collision`, MIT) integrates contact-active ticks. Exactly
//! one integrator owns a body per tick: when this runtime is active for the
//! vehicle, the flight step samples the same external loads through
//! [`FlightAuthority::evaluate_forces`](super::runtime::FlightAuthority)
//! but hands integration to Rapier instead of
//! `integrate_rigid_body_step_soa`.
//!
//! The activation boundary has hysteresis: a body enters contact-active mode
//! at `enter_distance_m` and leaves only past `exit_distance_m`, so it never
//! alternates integrators near one threshold. Uncertain (non-finite)
//! distance evidence keeps the body contact-active — optimisation comes
//! after known-case tests, never before safety.

use glam::{DQuat, DVec3};
use thessa_collision::{
    CollisionBodyId, CollisionDebugSnapshot, CollisionFrame, CollisionWorld, DynamicBodyConfig,
    ExternalWrench, KinematicBodyId, StaticColliderId,
};
use thessa_sim_core::{
    CollisionGeometry, CollisionMaterial, FlightError, FlightForces, RigidBodyProperties,
    RigidBodyState,
};

fn invalid(message: impl Into<String>) -> FlightError {
    FlightError::InvalidInput(message.into())
}

/// Hysteresis policy for the contact-active regime switch.
///
/// `enter_distance_m` is the conservative evidence range at which a body
/// becomes contact-active; `exit_distance_m` (strictly larger) is where it
/// may return to the free-flight integrator. Both are distances from the
/// candidate envelope (terrain certificate, vehicle pair, docking intent) to
/// the craft, in metres.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ContactActivation {
    enter_distance_m: f64,
    exit_distance_m: f64,
    active: bool,
}

impl ContactActivation {
    pub fn new(enter_distance_m: f64, exit_distance_m: f64) -> Result<Self, FlightError> {
        if !enter_distance_m.is_finite()
            || !exit_distance_m.is_finite()
            || enter_distance_m < 0.0
            || exit_distance_m <= enter_distance_m
        {
            return Err(invalid(
                "contact activation needs finite distances with exit > enter >= 0",
            ));
        }
        Ok(Self {
            enter_distance_m,
            exit_distance_m,
            active: false,
        })
    }

    pub const fn is_active(self) -> bool {
        self.active
    }

    pub const fn enter_distance_m(self) -> f64 {
        self.enter_distance_m
    }

    pub const fn exit_distance_m(self) -> f64 {
        self.exit_distance_m
    }

    /// Feed one distance observation, returning whether the body must be
    /// contact-active now. Non-finite evidence means uncertainty, and an
    /// uncertain body stays contact-active per the integration policy.
    pub fn observe(&mut self, distance_m: f64) -> bool {
        if !distance_m.is_finite() {
            self.active = true;
            return true;
        }
        if self.active {
            if distance_m > self.exit_distance_m {
                self.active = false;
            }
        } else if distance_m <= self.enter_distance_m {
            self.active = true;
        }
        self.active
    }

    /// Force the active state, for staging/docking events whose intent is
    /// known without a distance certificate.
    pub fn set_active(&mut self, active: bool) {
        self.active = active;
    }
}

/// Transient contact scene for one authoritative vehicle and its terrain
/// patches. Rebuilt from authoritative simulation state on regime entry;
/// authoritative persistence serializes Thessa state, never this structure.
pub struct ContactRuntime {
    world: CollisionWorld,
    body: Option<ContactBody>,
    activation: ContactActivation,
    static_patches: Vec<StaticColliderId>,
    kinematic_terrain: Vec<KinematicBodyId>,
}

struct ContactBody {
    id: CollisionBodyId,
    geometry: CollisionGeometry,
    properties: RigidBodyProperties,
    config: DynamicBodyConfig,
}

impl ContactRuntime {
    pub fn new(frame: CollisionFrame, activation: ContactActivation) -> Result<Self, FlightError> {
        let world = CollisionWorld::new(frame)
            .map_err(|error| invalid(format!("contact frame: {error}")))?;
        Ok(Self {
            world,
            body: None,
            activation,
            static_patches: Vec::new(),
            kinematic_terrain: Vec::new(),
        })
    }

    pub const fn is_active(&self) -> bool {
        self.activation.is_active()
    }

    pub const fn activation(&self) -> ContactActivation {
        self.activation
    }

    /// Feed terrain/pair distance evidence. Returns `(active,
    /// just_activated)`: a rising edge must invalidate the current rails
    /// coast and any baked forecast before the next physics tick.
    pub fn observe_distance(&mut self, distance_m: f64) -> (bool, bool) {
        let was_active = self.activation.is_active();
        let active = self.activation.observe(distance_m);
        (active, active && !was_active)
    }

    /// Ensure the backend body mirrors the authoritative vehicle.
    ///
    /// A geometry change (structural failure, staging, docking) or a mass /
    /// sleep-policy change rebuilds the backend body so no stale compound
    /// survives a topology change. Fresh state with unchanged geometry,
    /// mass, and policy re-mirrors the backend pose/velocity in place, so a
    /// re-entry never integrates from a stale pose.
    pub fn sync_body(
        &mut self,
        state: RigidBodyState,
        properties: RigidBodyProperties,
        geometry: &CollisionGeometry,
        config: DynamicBodyConfig,
    ) -> Result<CollisionBodyId, FlightError> {
        if geometry.is_empty() {
            return Err(invalid(
                "contact-active body needs compiled collision geometry",
            ));
        }
        let rebuild = match &self.body {
            None => true,
            Some(body) => {
                body.geometry != *geometry || body.properties != properties || body.config != config
            }
        };
        if !rebuild {
            let id = self
                .body
                .as_ref()
                .map(|body| body.id)
                .ok_or_else(|| invalid("contact sync lost its backend body".to_string()))?;
            self.world
                .resync_dynamic_body(id, state, properties, config)
                .map_err(|error| invalid(format!("contact resync: {error}")))?;
            return Ok(id);
        }
        if let Some(previous) = self.body.take() {
            self.world
                .remove_dynamic_body(previous.id)
                .map_err(|error| invalid(format!("contact rebuild: {error}")))?;
        }
        let id = self
            .world
            .insert_dynamic_body(state, properties, geometry, config)
            .map_err(|error| invalid(format!("contact body: {error}")))?;
        self.body = Some(ContactBody {
            id,
            geometry: geometry.clone(),
            properties,
            config,
        });
        Ok(id)
    }

    pub fn body_id(&self) -> Option<CollisionBodyId> {
        self.body.as_ref().map(|body| body.id)
    }

    pub fn remove_body(&mut self) -> Result<(), FlightError> {
        if let Some(previous) = self.body.take() {
            self.world
                .remove_dynamic_body(previous.id)
                .map_err(|error| invalid(format!("contact removal: {error}")))?;
        }
        Ok(())
    }

    /// Convert the flight step's sampled loads into the Rapier wrench. The
    /// caller must supply `FlightForces` from the same `evaluate_forces`
    /// path the free-flight integrator uses; gravity stays a separately
    /// sampled field and is added here, never baked into the flight forces.
    pub fn evaluate_wrench(
        state: RigidBodyState,
        properties: RigidBodyProperties,
        gravity_acceleration_inertial_mps2: DVec3,
        forces: &FlightForces,
    ) -> Result<ExternalWrench, FlightError> {
        if !gravity_acceleration_inertial_mps2.is_finite() {
            return Err(invalid("contact gravity sample must be finite"));
        }
        ExternalWrench::from_flight_forces(
            state,
            properties,
            gravity_acceleration_inertial_mps2,
            forces,
        )
        .map_err(|error| invalid(format!("contact wrench: {error}")))
    }

    /// Advance one contact-active tick and return the authoritative state.
    /// Exactly one integrator owns the body here: the caller must not also
    /// run `integrate_rigid_body_step_soa` for this tick.
    pub fn step(
        &mut self,
        step_s: f64,
        wrench: ExternalWrench,
    ) -> Result<RigidBodyState, FlightError> {
        let id = self
            .body
            .as_ref()
            .map(|body| body.id)
            .ok_or_else(|| invalid("contact step needs a synced body"))?;
        self.world
            .step(step_s, [(id, wrench)])
            .map_err(|error| invalid(format!("contact step: {error}")))?;
        self.world
            .body_state(id)
            .map_err(|error| invalid(format!("contact readback: {error}")))
    }

    /// Attach a static terrain patch (non-rotating test geometry or a
    /// far-field proxy). Rotating planetary terrain must use
    /// [`attach_kinematic_terrain`](Self::attach_kinematic_terrain) so its
    /// ephemeris-derived surface velocity participates in contacts.
    pub fn attach_static_patch(
        &mut self,
        center_local_m: DVec3,
        orientation_local: DQuat,
        half_extents_m: DVec3,
        material: CollisionMaterial,
    ) -> Result<StaticColliderId, FlightError> {
        let id = self
            .world
            .insert_static_cuboid(center_local_m, orientation_local, half_extents_m, material)
            .map_err(|error| invalid(format!("contact terrain patch: {error}")))?;
        self.static_patches.push(id);
        Ok(id)
    }

    /// Attach a kinematic terrain patch whose pose the caller drives from
    /// the canonical ephemeris/body-rotation model through
    /// [`move_kinematic_terrain`](Self::move_kinematic_terrain).
    pub fn attach_kinematic_terrain(
        &mut self,
        center_local_m: DVec3,
        orientation_local: DQuat,
        half_extents_m: DVec3,
        material: CollisionMaterial,
    ) -> Result<KinematicBodyId, FlightError> {
        let id = self
            .world
            .insert_kinematic_cuboid(center_local_m, orientation_local, half_extents_m, material)
            .map_err(|error| invalid(format!("contact kinematic terrain: {error}")))?;
        self.kinematic_terrain.push(id);
        Ok(id)
    }

    /// Publish the ephemeris-derived pose kinematic terrain must reach by
    /// the next step.
    pub fn move_kinematic_terrain(
        &mut self,
        id: KinematicBodyId,
        center_local_m: DVec3,
        orientation_local: DQuat,
    ) -> Result<(), FlightError> {
        self.world
            .set_next_kinematic_pose(id, center_local_m, orientation_local)
            .map_err(|error| invalid(format!("contact terrain motion: {error}")))
    }

    /// Evict a static patch that left the contact-active envelope.
    pub fn evict_static_patch(&mut self, id: StaticColliderId) -> Result<(), FlightError> {
        self.world
            .remove_static_collider(id)
            .map_err(|error| invalid(format!("contact patch eviction: {error}")))?;
        self.static_patches.retain(|patch| *patch != id);
        Ok(())
    }

    /// Evict a kinematic terrain patch that left the contact-active
    /// envelope. Streaming calls this when a patch scrolls out of range so
    /// the backend never accumulates stale moving geometry.
    pub fn evict_kinematic_terrain(&mut self, id: KinematicBodyId) -> Result<(), FlightError> {
        self.world
            .remove_kinematic_body(id)
            .map_err(|error| invalid(format!("contact terrain eviction: {error}")))?;
        self.kinematic_terrain.retain(|terrain| *terrain != id);
        Ok(())
    }

    pub fn debug_snapshot(&self) -> Result<CollisionDebugSnapshot, FlightError> {
        self.world
            .debug_snapshot()
            .map_err(|error| invalid(format!("contact snapshot: {error}")))
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use glam::{DMat3, DQuat, DVec3};
    use thessa_sim_core::{CollisionMaterial, CollisionPart, CollisionShape, x15_contact_geometry};

    fn test_properties() -> RigidBodyProperties {
        RigidBodyProperties::new(100.0, DMat3::from_diagonal(DVec3::splat(500.0))).unwrap()
    }

    fn activation() -> ContactActivation {
        ContactActivation::new(50.0, 80.0).unwrap()
    }

    fn runtime() -> ContactRuntime {
        ContactRuntime::new(
            CollisionFrame::inertial_at(DVec3::ZERO, DVec3::ZERO),
            activation(),
        )
        .unwrap()
    }

    #[test]
    fn activation_has_hysteresis_and_is_conservative_on_uncertainty() {
        let mut policy = activation();
        assert!(!policy.observe(200.0));
        assert!(policy.observe(50.0));
        // Inside the hysteresis band the body stays active.
        assert!(policy.observe(65.0));
        assert!(policy.observe(80.0));
        assert!(!policy.observe(80.0001));
        // Uncertain evidence keeps the body contact-active.
        assert!(policy.observe(f64::NAN));
        assert!(policy.is_active());
    }

    #[test]
    fn rising_edge_is_reported_for_rails_invalidation() {
        let mut runtime = runtime();
        assert_eq!(runtime.observe_distance(200.0), (false, false));
        assert_eq!(runtime.observe_distance(10.0), (true, true));
        assert_eq!(runtime.observe_distance(60.0), (true, false));
        assert_eq!(runtime.observe_distance(200.0), (false, false));
    }

    #[test]
    fn contact_step_uses_the_same_wrench_conversion() {
        let mut runtime = runtime();
        let geometry = x15_contact_geometry().unwrap();
        let state = RigidBodyState::stationary(DVec3::new(0.0, 5.0, 0.0));
        let properties = test_properties();
        let id = runtime
            .sync_body(state, properties, &geometry, DynamicBodyConfig::default())
            .unwrap();
        assert_eq!(runtime.body_id(), Some(id));
        // Same body/geometry re-syncs without a rebuild.
        assert_eq!(
            runtime
                .sync_body(state, properties, &geometry, DynamicBodyConfig::default())
                .unwrap(),
            id
        );
        let forces = thessa_sim_core::FlightForces {
            environment: thessa_sim_core::AeroEnvironment::standard_sea_level(),
            aero: thessa_sim_core::AeroResult {
                force_body_n: DVec3::ZERO,
                moment_body_nm: DVec3::ZERO,
                dynamic_pressure_pa: 0.0,
                mach: 0.0,
                reynolds_number: 0.0,
                panel_count: 0,
                panel_loads: None,
            },
            total_force_body_n: DVec3::new(100.0, 0.0, 0.0),
            total_moment_body_nm: DVec3::ZERO,
            total_force_inertial_n: DVec3::new(100.0, 0.0, 0.0),
            acceleration_inertial_mps2: DVec3::new(1.0, -9.81, 0.0),
            angular_acceleration_body_rps2: DVec3::ZERO,
        };
        let wrench = ContactRuntime::evaluate_wrench(
            state,
            properties,
            DVec3::new(0.0, -9.81, 0.0),
            &forces,
        )
        .unwrap();
        assert!((wrench.force_inertial_n - DVec3::new(100.0, -981.0, 0.0)).length() < 1.0e-9);
        let next = runtime.step(1.0 / 120.0, wrench).unwrap();
        assert!(next.position_inertial_m.is_finite());
        let snapshot = runtime.debug_snapshot().unwrap();
        assert_eq!(snapshot.dynamic_bodies.len(), 1);
    }

    #[test]
    fn geometry_change_rebuilds_the_backend_body() {
        let mut runtime = runtime();
        let geometry = x15_contact_geometry().unwrap();
        let state = RigidBodyState::stationary(DVec3::ZERO);
        let properties = test_properties();
        let first = runtime
            .sync_body(state, properties, &geometry, DynamicBodyConfig::default())
            .unwrap();
        let smaller = CollisionGeometry::new(vec![
            CollisionPart::new(
                DVec3::ZERO,
                DQuat::IDENTITY,
                CollisionShape::Sphere { radius_m: 0.5 },
                CollisionMaterial::default(),
            )
            .unwrap(),
        ])
        .unwrap();
        let second = runtime
            .sync_body(state, properties, &smaller, DynamicBodyConfig::default())
            .unwrap();
        assert_ne!(
            first, second,
            "topology change must rebuild the backend body"
        );
        runtime.remove_body().unwrap();
        assert_eq!(runtime.body_id(), None);
    }

    #[test]
    fn resync_re_mirrors_state_and_rebuilds_on_mass_or_policy_change() {
        let mut runtime = runtime();
        let geometry = x15_contact_geometry().unwrap();
        let properties = test_properties();
        let config = DynamicBodyConfig::default();
        let id = runtime
            .sync_body(
                RigidBodyState::stationary(DVec3::new(0.0, 5.0, 0.0)),
                properties,
                &geometry,
                config,
            )
            .unwrap();
        // Fresh state with unchanged geometry/mass/policy keeps the id but
        // must move the backend body: re-entry never integrates stale pose.
        let moved = RigidBodyState::new(
            DVec3::new(10.0, 6.0, -3.0),
            DVec3::new(1.0, 2.0, 3.0),
            DQuat::from_rotation_z(0.4),
            DVec3::new(0.01, -0.02, 0.03),
        )
        .unwrap();
        assert_eq!(
            runtime
                .sync_body(moved, properties, &geometry, config)
                .unwrap(),
            id
        );
        let mirrored = runtime.world.body_state(id).unwrap();
        assert!((mirrored.position_inertial_m - moved.position_inertial_m).length() < 1.0e-9);
        assert!((mirrored.velocity_inertial_mps - moved.velocity_inertial_mps).length() < 1.0e-9);
        // Changed mass properties rebuild the backend body.
        let heavier =
            RigidBodyProperties::new(250.0, DMat3::from_diagonal(DVec3::splat(900.0))).unwrap();
        let rebuilt = runtime
            .sync_body(moved, heavier, &geometry, config)
            .unwrap();
        assert_ne!(id, rebuilt, "mass change must rebuild the backend body");
        // Changed sleep policy rebuilds as well.
        let no_sleep = runtime
            .sync_body(
                moved,
                heavier,
                &geometry,
                DynamicBodyConfig {
                    full_ccd: true,
                    can_sleep: false,
                },
            )
            .unwrap();
        assert_ne!(
            rebuilt, no_sleep,
            "solver-policy change must rebuild the backend body"
        );
    }

    #[test]
    fn kinematic_terrain_patch_participates_in_contacts() {
        let mut runtime = runtime();
        let patch = runtime
            .attach_static_patch(
                DVec3::new(0.0, -0.5, 0.0),
                DQuat::IDENTITY,
                DVec3::new(10.0, 0.5, 10.0),
                CollisionMaterial::default(),
            )
            .unwrap();
        let platform = runtime
            .attach_kinematic_terrain(
                DVec3::new(20.0, 0.0, 0.0),
                DQuat::IDENTITY,
                DVec3::new(1.0, 0.5, 1.0),
                CollisionMaterial::default(),
            )
            .unwrap();
        runtime
            .move_kinematic_terrain(platform, DVec3::new(20.0, 1.0, 0.0), DQuat::IDENTITY)
            .unwrap();
        let snapshot = runtime.debug_snapshot().unwrap();
        assert_eq!(snapshot.fixed_collider_count, 1);
        assert_eq!(snapshot.kinematic_bodies.len(), 1);
        runtime.evict_static_patch(patch).unwrap();
        runtime.evict_kinematic_terrain(platform).unwrap();
        let snapshot = runtime.debug_snapshot().unwrap();
        assert_eq!(snapshot.fixed_collider_count, 0);
        assert!(snapshot.kinematic_bodies.is_empty());
    }
}
