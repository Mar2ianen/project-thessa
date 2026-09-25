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
    ArticulatedWheelBinding, CollisionBodyId, CollisionDebugSnapshot, CollisionFrame,
    CollisionWorld, ContactSummary, DynamicBodyConfig, ExternalWrench, JointId, KinematicBodyId,
    StaticColliderId, WheelContactResult, WheelContactSample,
};
use thessa_sim_core::{
    CollisionGeometry, CollisionMaterial, CollisionPart, CollisionShape, CompiledWheelChassis,
    FlightError, FlightForces, RigidBodyProperties, RigidBodyState, VehicleDefinition,
    VehicleWheelMassSplit, WheelBodyMassProperties, WheelBrakeState, WheelDrivePoint,
};

fn invalid(message: impl Into<String>) -> FlightError {
    FlightError::InvalidInput(message.into())
}

fn shift_collision_geometry(
    geometry: &CollisionGeometry,
    origin_shift_body_m: DVec3,
) -> Result<CollisionGeometry, FlightError> {
    let mut shifted = geometry.clone();
    for part in &mut shifted.parts {
        part.local_position_m -= origin_shift_body_m;
    }
    shifted
        .validate()
        .map_err(|error| invalid(format!("sprung collision geometry: {error}")))?;
    Ok(shifted)
}

fn add_wrench(
    wrenches: &mut std::collections::BTreeMap<CollisionBodyId, ExternalWrench>,
    body: CollisionBodyId,
    wrench: ExternalWrench,
) {
    let accumulated = wrenches.entry(body).or_insert(ExternalWrench::ZERO);
    accumulated.force_inertial_n += wrench.force_inertial_n;
    accumulated.torque_inertial_nm += wrench.torque_inertial_nm;
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

/// One body in the vehicle-vehicle broad phase: authoritative position,
/// bounding radius, and speed. All SI/f64; the scheduler owns the values,
/// this policy only reads them.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ContactCandidate {
    pub tag: u64,
    pub position_inertial_m: DVec3,
    pub bounding_radius_m: f64,
    pub speed_mps: f64,
}

/// One pair under broad-phase observation.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct BroadPhasePair {
    pub first_tag: u64,
    pub second_tag: u64,
    pub active: bool,
    pub just_activated: bool,
}

/// Pairwise contact-activation screen for a vehicle fleet. Surface-to-surface
/// distance minus the closing distance over a bounded horizon decides; each
/// pair carries hysteresis so neighbours never flap the integrator. Kept
/// solver-free on purpose: the scheduler runs this before touching any
/// collision backend, and only active pairs earn one.
#[derive(Debug, Clone, PartialEq)]
pub struct ContactBroadPhase {
    enter_distance_m: f64,
    exit_distance_m: f64,
    horizon_s: f64,
    pairs: std::collections::BTreeMap<(u64, u64), bool>,
}

impl ContactBroadPhase {
    pub fn new(
        enter_distance_m: f64,
        exit_distance_m: f64,
        horizon_s: f64,
    ) -> Result<Self, FlightError> {
        if !enter_distance_m.is_finite()
            || !exit_distance_m.is_finite()
            || enter_distance_m < 0.0
            || exit_distance_m <= enter_distance_m
        {
            return Err(invalid(
                "broad phase needs finite distances with exit > enter >= 0",
            ));
        }
        if !horizon_s.is_finite() || horizon_s < 0.0 {
            return Err(invalid(
                "broad-phase horizon must be finite and non-negative",
            ));
        }
        Ok(Self {
            enter_distance_m,
            exit_distance_m,
            horizon_s,
            pairs: std::collections::BTreeMap::new(),
        })
    }

    /// Observe one candidate set, returning every pair worth tracking. A pair
    /// is reported while it is active or on the tick it deactivates (so the
    /// owner can drop the backend body); long-separated pairs disappear.
    pub fn observe(&mut self, candidates: &[ContactCandidate]) -> Vec<BroadPhasePair> {
        let mut observed = std::collections::BTreeSet::new();
        let mut report = Vec::new();
        for (index, a) in candidates.iter().enumerate() {
            for b in &candidates[index + 1..] {
                let (first, second) = if a.tag <= b.tag {
                    (a.tag, b.tag)
                } else {
                    (b.tag, a.tag)
                };
                observed.insert((first, second));
                let was_active = self.pairs.get(&(first, second)).copied().unwrap_or(false);
                let distance = pair_distance_m(a, b, self.horizon_s);
                let active = match distance {
                    // Uncertain evidence keeps the pair active.
                    None => true,
                    Some(distance) if was_active => distance <= self.exit_distance_m,
                    Some(distance) => distance <= self.enter_distance_m,
                };
                self.pairs.insert((first, second), active);
                if active || was_active {
                    report.push(BroadPhasePair {
                        first_tag: first,
                        second_tag: second,
                        active,
                        just_activated: active && !was_active,
                    });
                }
            }
        }
        self.pairs.retain(|pair, _| observed.contains(pair));
        report.sort_by_key(|pair| (pair.first_tag, pair.second_tag));
        report
    }
}

/// Conservative surface-to-surface distance over the horizon, or `None` on
/// non-finite evidence. Closing speed is the sum of speeds (worst case:
/// head-on), never a relative-velocity projection that could hide a turn.
fn pair_distance_m(a: &ContactCandidate, b: &ContactCandidate, horizon_s: f64) -> Option<f64> {
    if !a.position_inertial_m.is_finite()
        || !b.position_inertial_m.is_finite()
        || !a.bounding_radius_m.is_finite()
        || !b.bounding_radius_m.is_finite()
        || !a.speed_mps.is_finite()
        || !b.speed_mps.is_finite()
        || a.bounding_radius_m < 0.0
        || b.bounding_radius_m < 0.0
        || a.speed_mps < 0.0
        || b.speed_mps < 0.0
    {
        return None;
    }
    let distance = (a.position_inertial_m - b.position_inertial_m).length()
        - a.bounding_radius_m
        - b.bounding_radius_m
        - (a.speed_mps + b.speed_mps) * horizon_s;
    distance.is_finite().then_some(distance)
}

/// Transient contact scene for one authoritative vehicle and its terrain
/// patches. Rebuilt from authoritative simulation state on regime entry;
/// authoritative persistence serializes Thessa state, never this structure.
pub struct ContactRuntime {
    world: CollisionWorld,
    body: Option<ContactBody>,
    wheel_assembly: Option<ContactWheelAssembly>,
    /// Docking partners (upper stage, visiting vehicle) sharing this scene.
    /// The primary `body` keeps the single-vehicle fast path; partners join
    /// for joint/docking ticks and multi-body contact.
    partners: std::collections::BTreeMap<u64, ContactBody>,
    activation: ContactActivation,
    static_patches: Vec<StaticColliderId>,
    kinematic_terrain: Vec<KinematicBodyId>,
}

#[derive(Clone)]
struct ContactBody {
    id: CollisionBodyId,
    geometry: CollisionGeometry,
    properties: RigidBodyProperties,
    config: DynamicBodyConfig,
}

struct ContactWheelBody {
    id: CollisionBodyId,
    chassis_index: usize,
    wheel_mass: WheelBodyMassProperties,
}

struct ContactWheelAssembly {
    total_properties: RigidBodyProperties,
    mass_split: VehicleWheelMassSplit,
    chassis: Vec<CompiledWheelChassis>,
    source_geometry: CollisionGeometry,
    config: DynamicBodyConfig,
    wheels: Vec<ContactWheelBody>,
    bindings_by_chassis: Vec<Vec<ArticulatedWheelBinding>>,
}

impl ContactRuntime {
    pub fn new(frame: CollisionFrame, activation: ContactActivation) -> Result<Self, FlightError> {
        let world = CollisionWorld::new(frame)
            .map_err(|error| invalid(format!("contact frame: {error}")))?;
        Ok(Self {
            world,
            body: None,
            wheel_assembly: None,
            partners: std::collections::BTreeMap::new(),
            activation,
            static_patches: Vec::new(),
            kinematic_terrain: Vec::new(),
        })
    }

    pub const fn is_active(&self) -> bool {
        self.activation.is_active()
    }

    pub const fn frame(&self) -> CollisionFrame {
        self.world.frame()
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
        self.world
            .validate_dynamic_body_inputs(state, properties, geometry)
            .map_err(|error| invalid(format!("contact body: {error}")))?;
        self.clear_wheel_assembly()?;
        Self::sync_slot(
            &mut self.world,
            &mut self.body,
            state,
            properties,
            geometry,
            config,
        )
    }

    pub fn body_id(&self) -> Option<CollisionBodyId> {
        self.body.as_ref().map(|body| body.id)
    }

    pub fn remove_body(&mut self) -> Result<(), FlightError> {
        self.clear_wheel_assembly()?;
        if let Some(previous) = self.body.take() {
            self.world
                .remove_dynamic_body(previous.id)
                .map_err(|error| invalid(format!("contact removal: {error}")))?;
        }
        Ok(())
    }

    /// Synchronize a vehicle as one sprung body plus one sensor-only dynamic
    /// wheel body per station. The total vehicle state remains COM-based;
    /// unsprung wheel poses are retained between ticks and the sprung pose is
    /// reconciled from total COM and the live wheel states.
    fn sync_articulated_vehicle(
        &mut self,
        state: RigidBodyState,
        vehicle: &VehicleDefinition,
        wheel_spin_rad_s: &[Vec<f64>],
        config: DynamicBodyConfig,
    ) -> Result<CollisionBodyId, FlightError> {
        if vehicle.wheel_chassis.is_empty() {
            return Err(invalid("articulated sync needs at least one wheel chassis"));
        }
        if vehicle.collision_geometry.is_empty() {
            return Err(invalid(
                "articulated contact needs compiled sprung-body collision geometry",
            ));
        }
        if wheel_spin_rad_s.len() != vehicle.wheel_chassis.len()
            || wheel_spin_rad_s
                .iter()
                .zip(&vehicle.wheel_chassis)
                .any(|(rates, chassis)| {
                    rates.len() != chassis.wheel_stations.len()
                        || rates.iter().any(|rate| !rate.is_finite())
                })
        {
            return Err(invalid(
                "wheel spin state must match every compiled wheel station",
            ));
        }
        let needs_rebuild = self.body.is_none()
            || self.wheel_assembly.as_ref().is_none_or(|assembly| {
                assembly.total_properties != vehicle.mass_properties
                    || assembly.chassis != vehicle.wheel_chassis
                    || assembly.source_geometry != vehicle.collision_geometry
                    || assembly.config != config
            });
        if needs_rebuild {
            self.clear_wheel_assembly()?;
            let mass_split = vehicle
                .wheel_mass_split()
                .map_err(|error| invalid(format!("wheel mass split: {error}")))?;
            let sprung_geometry = shift_collision_geometry(
                &vehicle.collision_geometry,
                mass_split.sprung_center_of_mass_body_m,
            )?;
            let orientation = state.orientation_body_to_inertial;
            let angular_inertial = orientation * state.angular_velocity_body_rps;
            let sprung_offset = orientation * mass_split.sprung_center_of_mass_body_m;
            let sprung_state = RigidBodyState::new(
                state.position_inertial_m + sprung_offset,
                state.velocity_inertial_mps + angular_inertial.cross(sprung_offset),
                orientation,
                state.angular_velocity_body_rps,
            )
            .map_err(|error| invalid(format!("sprung body initialization: {error}")))?;
            let root_id = Self::sync_slot(
                &mut self.world,
                &mut self.body,
                sprung_state,
                mass_split.sprung_properties,
                &sprung_geometry,
                config,
            )?;

            let mut wheel_bodies: Vec<ContactWheelBody> =
                Vec::with_capacity(mass_split.wheels.len());
            let mut bindings_by_chassis: Vec<Vec<ArticulatedWheelBinding>> = vehicle
                .wheel_chassis
                .iter()
                .map(|chassis| Vec::with_capacity(chassis.wheel_stations.len()))
                .collect();
            for (chassis_index, chassis) in vehicle.wheel_chassis.iter().enumerate() {
                let slide_axis_body = chassis.spec.mount_orientation_body * DVec3::NEG_Z;
                for station in &chassis.wheel_stations {
                    let wheel_mass = mass_split
                        .wheels
                        .iter()
                        .find(|wheel| {
                            wheel.chassis_index == chassis_index
                                && wheel.wheel_index == station.index
                        })
                        .copied()
                        .ok_or_else(|| invalid("wheel mass split lost a compiled station"))?;
                    let wheel_properties =
                        RigidBodyProperties::new(wheel_mass.mass_kg, wheel_mass.inertia_body_kg_m2)
                            .map_err(|error| invalid(format!("wheel mass properties: {error}")))?;
                    let wheel_offset = orientation * wheel_mass.center_of_mass_body_m;
                    let axle_spin = station.axle_axis_body
                        * wheel_spin_rad_s[chassis_index][usize::from(station.index)];
                    let wheel_state = RigidBodyState::new(
                        state.position_inertial_m + wheel_offset,
                        state.velocity_inertial_mps + angular_inertial.cross(wheel_offset),
                        orientation,
                        state.angular_velocity_body_rps + axle_spin,
                    )
                    .map_err(|error| invalid(format!("wheel body initialization: {error}")))?;
                    let wheel_geometry = CollisionGeometry::new(vec![
                        CollisionPart::new(
                            DVec3::ZERO,
                            DQuat::IDENTITY,
                            CollisionShape::Sphere {
                                radius_m: chassis.spec.tire.radius_m,
                            },
                            CollisionMaterial::new(chassis.spec.tire.surface_friction, 0.0)
                                .map_err(|error| invalid(format!("wheel material: {error}")))?,
                        )
                        .map_err(|error| invalid(format!("wheel sensor geometry: {error}")))?,
                    ])
                    .map_err(|error| invalid(format!("wheel sensor geometry: {error}")))?;
                    let wheel_id = self
                        .world
                        .insert_dynamic_sensor_body(
                            wheel_state,
                            wheel_properties,
                            &wheel_geometry,
                            config,
                        )
                        .map_err(|error| invalid(format!("wheel body: {error}")))?;
                    let nominal_center_sprung_local_m =
                        station.position_body_m - mass_split.sprung_center_of_mass_body_m;
                    let anchor_sprung_local_m = nominal_center_sprung_local_m;
                    let binding = ArticulatedWheelBinding {
                        wheel_body: wheel_id,
                        wheel_index: station.index,
                        nominal_center_sprung_local_m,
                        slide_axis_sprung_local: slide_axis_body,
                        axle_axis_sprung_local: station.axle_axis_body,
                    };
                    if let Err(error) = self.world.attach_suspension_wheel_joint(
                        root_id,
                        wheel_id,
                        anchor_sprung_local_m,
                        DVec3::ZERO,
                        slide_axis_body,
                        station.axle_axis_body,
                        slide_axis_body,
                        station.axle_axis_body,
                        [-chassis.spec.strut.stroke_m, 0.0],
                    ) {
                        self.world.remove_dynamic_body(wheel_id).ok();
                        for previous in &wheel_bodies {
                            self.world.remove_dynamic_body(previous.id).ok();
                        }
                        self.body = None;
                        self.world.remove_dynamic_body(root_id).ok();
                        return Err(invalid(format!("wheel suspension joint: {error}")));
                    }
                    wheel_bodies.push(ContactWheelBody {
                        id: wheel_id,
                        chassis_index,
                        wheel_mass,
                    });
                    bindings_by_chassis[chassis_index].push(binding);
                }
            }
            self.wheel_assembly = Some(ContactWheelAssembly {
                total_properties: vehicle.mass_properties,
                mass_split,
                chassis: vehicle.wheel_chassis.clone(),
                source_geometry: vehicle.collision_geometry.clone(),
                config,
                wheels: wheel_bodies,
                bindings_by_chassis,
            });
            return Ok(root_id);
        }

        let assembly = self
            .wheel_assembly
            .as_ref()
            .ok_or_else(|| invalid("wheel assembly disappeared during sync"))?;
        let total_mass = vehicle.mass_properties.mass_kg;
        let sprung_mass = assembly.mass_split.sprung_properties.mass_kg;
        let mut wheel_position_moment = DVec3::ZERO;
        let mut wheel_velocity_moment = DVec3::ZERO;
        for wheel in &assembly.wheels {
            let wheel_state = self
                .world
                .body_state(wheel.id)
                .map_err(|error| invalid(format!("wheel state sync: {error}")))?;
            wheel_position_moment += wheel_state.position_inertial_m * wheel.wheel_mass.mass_kg;
            wheel_velocity_moment += wheel_state.velocity_inertial_mps * wheel.wheel_mass.mass_kg;
        }
        let sprung_state = RigidBodyState::new(
            (state.position_inertial_m * total_mass - wheel_position_moment) / sprung_mass,
            (state.velocity_inertial_mps * total_mass - wheel_velocity_moment) / sprung_mass,
            state.orientation_body_to_inertial,
            state.angular_velocity_body_rps,
        )
        .map_err(|error| invalid(format!("sprung body reconciliation: {error}")))?;
        let root_id = self
            .body
            .as_ref()
            .map(|body| body.id)
            .ok_or_else(|| invalid("articulated sync lost its sprung body"))?;
        self.world
            .resync_dynamic_body(
                root_id,
                sprung_state,
                assembly.mass_split.sprung_properties,
                config,
            )
            .map_err(|error| invalid(format!("sprung body resync: {error}")))?;
        Ok(root_id)
    }

    fn clear_wheel_assembly(&mut self) -> Result<(), FlightError> {
        if let Some(assembly) = self.wheel_assembly.take() {
            for wheel in assembly.wheels {
                self.world
                    .remove_dynamic_body(wheel.id)
                    .map_err(|error| invalid(format!("wheel removal: {error}")))?;
            }
        }
        Ok(())
    }

    /// Sync a docking partner (upper stage, visiting vehicle) into the same
    /// scene. Partners share terrain, broad phase, and joints with the
    /// primary body; each keeps its own authoritative state mapping.
    pub fn sync_partner(
        &mut self,
        tag: u64,
        state: RigidBodyState,
        properties: RigidBodyProperties,
        geometry: &CollisionGeometry,
        config: DynamicBodyConfig,
    ) -> Result<CollisionBodyId, FlightError> {
        let mut slot = self.partners.get(&tag).cloned();
        let id = Self::sync_slot(
            &mut self.world,
            &mut slot,
            state,
            properties,
            geometry,
            config,
        )?;
        self.partners.insert(
            tag,
            slot.ok_or_else(|| invalid("contact sync lost its partner body".to_string()))?,
        );
        Ok(id)
    }

    pub fn partner_id(&self, tag: u64) -> Option<CollisionBodyId> {
        self.partners.get(&tag).map(|partner| partner.id)
    }

    pub fn remove_partner(&mut self, tag: u64) -> Result<(), FlightError> {
        if let Some(previous) = self.partners.remove(&tag) {
            self.world
                .remove_dynamic_body(previous.id)
                .map_err(|error| invalid(format!("contact partner removal: {error}")))?;
        }
        Ok(())
    }

    fn sync_slot(
        world: &mut CollisionWorld,
        slot: &mut Option<ContactBody>,
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
        let rebuild = match slot.as_ref() {
            None => true,
            Some(body) => {
                body.geometry != *geometry || body.properties != properties || body.config != config
            }
        };
        if !rebuild {
            let id = slot
                .as_ref()
                .map(|body| body.id)
                .ok_or_else(|| invalid("contact sync lost its backend body".to_string()))?;
            world
                .resync_dynamic_body(id, state, properties, config)
                .map_err(|error| invalid(format!("contact resync: {error}")))?;
            return Ok(id);
        }
        let id = world
            .insert_dynamic_body(state, properties, geometry, config)
            .map_err(|error| invalid(format!("contact body: {error}")))?;
        if let Some(previous) = slot.as_ref()
            && let Err(error) = world.remove_dynamic_body(previous.id)
        {
            let _ = world.remove_dynamic_body(id);
            return Err(invalid(format!("contact rebuild: {error}")));
        }
        *slot = Some(ContactBody {
            id,
            geometry: geometry.clone(),
            properties,
            config,
        });
        Ok(id)
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

    /// Advance one contact-active tick with compiled wheel chassis. Tire and
    /// strut contact wrenches are evaluated from Rapier terrain queries and
    /// combined with the caller's external wrench before the sole integration
    /// step. Wheel-spin rates remain caller-owned state.
    pub fn step_with_wheel_contacts(
        &mut self,
        step_s: f64,
        wrench: ExternalWrench,
        wheel_chassis: &[(&CompiledWheelChassis, &[f64])],
    ) -> Result<(RigidBodyState, Vec<WheelContactResult>), FlightError> {
        let id = self
            .body
            .as_ref()
            .map(|body| body.id)
            .ok_or_else(|| invalid("contact step needs a synced body"))?;
        let contacts = self
            .world
            .step_with_wheel_contacts(
                step_s,
                [(id, wrench)],
                wheel_chassis
                    .iter()
                    .map(|(chassis, spin_rates)| (id, *chassis, *spin_rates)),
            )
            .map_err(|error| invalid(format!("wheel contact step: {error}")))?;
        let state = self
            .world
            .body_state(id)
            .map_err(|error| invalid(format!("contact readback: {error}")))?;
        Ok((state, contacts))
    }

    /// Integrate a vehicle whose wheel chassis have been assembled as
    /// sprung/unsprung Rapier bodies. External flight loads are applied at the
    /// total vehicle COM, gravity is distributed by mass, suspension/tire
    /// forces are applied at their respective bodies, and actuator torques are
    /// paired so they cannot create net angular momentum.
    #[allow(clippy::too_many_arguments, clippy::type_complexity)]
    pub fn step_articulated_vehicle(
        &mut self,
        step_s: f64,
        state: RigidBodyState,
        gravity_acceleration_inertial_mps2: DVec3,
        forces: &FlightForces,
        vehicle: &VehicleDefinition,
        brake_command: f64,
        drive_command: f64,
        wheel_spin_rad_s: &mut [Vec<f64>],
        wheel_brake_states: &mut [Vec<WheelBrakeState>],
    ) -> Result<
        (
            RigidBodyState,
            Vec<WheelContactSample>,
            Vec<(usize, u16, WheelDrivePoint)>,
        ),
        FlightError,
    > {
        if !step_s.is_finite()
            || step_s <= 0.0
            || !gravity_acceleration_inertial_mps2.is_finite()
            || !brake_command.is_finite()
            || !(0.0..=1.0).contains(&brake_command)
            || !drive_command.is_finite()
            || !(-1.0..=1.0).contains(&drive_command)
            || !forces.total_force_inertial_n.is_finite()
            || !forces.total_moment_body_nm.is_finite()
            || !(state.orientation_body_to_inertial * forces.total_moment_body_nm).is_finite()
        {
            return Err(invalid(
                "articulated step, wrench, commands and gravity must be finite and in range",
            ));
        }
        if wheel_brake_states.len() != vehicle.wheel_chassis.len()
            || wheel_brake_states
                .iter()
                .zip(&vehicle.wheel_chassis)
                .any(|(states, chassis)| states.len() != chassis.wheel_stations.len())
        {
            return Err(invalid(
                "wheel brake state must match every compiled wheel station",
            ));
        }
        let root_id = self.sync_articulated_vehicle(
            state,
            vehicle,
            wheel_spin_rad_s,
            DynamicBodyConfig::default(),
        )?;
        let assembly = self
            .wheel_assembly
            .as_ref()
            .ok_or_else(|| invalid("articulated contact lost its wheel assembly"))?;
        let sprung_state = self
            .world
            .body_state(root_id)
            .map_err(|error| invalid(format!("sprung state readback: {error}")))?;

        let external_force = forces.total_force_inertial_n;
        let external_torque = state.orientation_body_to_inertial * forces.total_moment_body_nm;
        let sprung_offset_from_total = state.position_inertial_m - sprung_state.position_inertial_m;
        let mut wrenches = std::collections::BTreeMap::new();
        wrenches.insert(
            root_id,
            ExternalWrench {
                force_inertial_n: external_force
                    + gravity_acceleration_inertial_mps2
                        * assembly.mass_split.sprung_properties.mass_kg,
                torque_inertial_nm: external_torque
                    + sprung_offset_from_total.cross(external_force),
            },
        );
        for wheel in &assembly.wheels {
            add_wrench(
                &mut wrenches,
                wheel.id,
                ExternalWrench {
                    force_inertial_n: gravity_acceleration_inertial_mps2 * wheel.wheel_mass.mass_kg,
                    torque_inertial_nm: DVec3::ZERO,
                },
            );
        }

        let mut contacts = Vec::new();
        let mut drive_points = Vec::new();
        for (chassis_index, chassis) in assembly.chassis.iter().enumerate() {
            let bindings = &assembly.bindings_by_chassis[chassis_index];
            let wheel_forces = self
                .world
                .evaluate_articulated_wheel_contacts(root_id, chassis, bindings)
                .map_err(|error| invalid(format!("articulated wheel contact: {error}")))?;
            for wheel_force in wheel_forces {
                let wheel = assembly
                    .wheels
                    .iter()
                    .find(|wheel| wheel.id == wheel_force.wheel_body)
                    .ok_or_else(|| invalid("wheel contact returned an unknown body"))?;
                let wheel_index = usize::from(wheel_force.wheel_index);
                wheel_spin_rad_s[chassis_index][wheel_index] = wheel_force.spin_rate_rad_s;
                if let Some(contact) = wheel_force.contact {
                    contacts.push(contact);
                }
                add_wrench(&mut wrenches, root_id, wheel_force.sprung_wrench);

                let wheel_state = self
                    .world
                    .body_state(wheel.id)
                    .map_err(|error| invalid(format!("wheel actuator state: {error}")))?;
                let axle_axis_inertial =
                    wheel_state.orientation_body_to_inertial * wheel.wheel_mass.axle_axis_body;
                let mut drive_torque_nm = 0.0;
                if let Some(drive) = chassis.drive
                    && wheel_index < usize::from(drive.driven_wheel_count)
                {
                    let point = drive
                        .operating_point(
                            wheel_force.spin_rate_rad_s * 60.0 / std::f64::consts::TAU,
                            drive_command,
                        )
                        .map_err(|error| invalid(format!("wheel drive: {error}")))?;
                    drive_torque_nm = point.requested_wheel_torque_per_driven_wheel_nm;
                    drive_points.push((chassis_index, wheel_force.wheel_index, point));
                }

                let brake_state = chassis
                    .spec
                    .brake
                    .advance(
                        wheel_brake_states[chassis_index][wheel_index],
                        brake_command,
                        step_s,
                    )
                    .map_err(|error| invalid(format!("wheel brake actuator: {error}")))?;
                wheel_brake_states[chassis_index][wheel_index] = brake_state;
                let contact_friction = wheel_force
                    .contact
                    .map(|contact| contact.contact_friction)
                    .unwrap_or(chassis.spec.tire.surface_friction);
                let normal_load_n = wheel_force
                    .contact
                    .map(|contact| contact.normal_load_n)
                    .unwrap_or(0.0);
                let brake = chassis
                    .spec
                    .brake
                    .braking_force(
                        brake_state.applied_fraction,
                        chassis.spec.tire.radius_m,
                        normal_load_n,
                        contact_friction,
                    )
                    .map_err(|error| invalid(format!("wheel brake load: {error}")))?;
                let axle_inertia = wheel
                    .wheel_mass
                    .axle_axis_body
                    .dot(wheel.wheel_mass.inertia_body_kg_m2 * wheel.wheel_mass.axle_axis_body);
                let tire_torque_nm = wheel_force
                    .wheel_wrench
                    .torque_inertial_nm
                    .dot(axle_axis_inertial);
                let requested_brake_torque_nm = -axle_inertia * wheel_force.spin_rate_rad_s
                    / step_s
                    - tire_torque_nm
                    - drive_torque_nm;
                let brake_torque_nm =
                    requested_brake_torque_nm.clamp(-brake.brake_torque_nm, brake.brake_torque_nm);
                let actuator_torque = axle_axis_inertial * (drive_torque_nm + brake_torque_nm);
                let mut wheel_wrench = wheel_force.wheel_wrench;
                wheel_wrench.torque_inertial_nm += actuator_torque;
                add_wrench(&mut wrenches, wheel.id, wheel_wrench);
                add_wrench(
                    &mut wrenches,
                    root_id,
                    ExternalWrench {
                        force_inertial_n: DVec3::ZERO,
                        torque_inertial_nm: -actuator_torque,
                    },
                );
            }
        }

        self.world
            .step(step_s, wrenches)
            .map_err(|error| invalid(format!("articulated contact step: {error}")))?;
        let sprung_next = self
            .world
            .body_state(root_id)
            .map_err(|error| invalid(format!("sprung contact readback: {error}")))?;
        let assembly = self
            .wheel_assembly
            .as_ref()
            .ok_or_else(|| invalid("articulated contact lost its wheel assembly after step"))?;
        let total_mass = vehicle.mass_properties.mass_kg;
        let mut center_position_m =
            sprung_next.position_inertial_m * assembly.mass_split.sprung_properties.mass_kg;
        let mut center_velocity_mps =
            sprung_next.velocity_inertial_mps * assembly.mass_split.sprung_properties.mass_kg;
        for wheel in &assembly.wheels {
            let wheel_state = self
                .world
                .body_state(wheel.id)
                .map_err(|error| invalid(format!("wheel contact readback: {error}")))?;
            center_position_m += wheel_state.position_inertial_m * wheel.wheel_mass.mass_kg;
            center_velocity_mps += wheel_state.velocity_inertial_mps * wheel.wheel_mass.mass_kg;
            let wheel_orientation = wheel_state.orientation_body_to_inertial;
            let sprung_angular_inertial =
                sprung_next.orientation_body_to_inertial * sprung_next.angular_velocity_body_rps;
            let wheel_angular_inertial = wheel_orientation * wheel_state.angular_velocity_body_rps;
            let axle_axis_inertial = wheel_orientation * wheel.wheel_mass.axle_axis_body;
            wheel_spin_rad_s[wheel.chassis_index][usize::from(wheel.wheel_mass.wheel_index)] =
                (wheel_angular_inertial - sprung_angular_inertial).dot(axle_axis_inertial);
        }
        let next_state = RigidBodyState::new(
            center_position_m / total_mass,
            center_velocity_mps / total_mass,
            sprung_next.orientation_body_to_inertial,
            sprung_next.angular_velocity_body_rps,
        )
        .map_err(|error| invalid(format!("vehicle contact readback: {error}")))?;
        Ok((next_state, contacts, drive_points))
    }

    /// Advance one tick for the primary body plus any synced partners,
    /// returning the primary state. Partners are read back through
    /// [`partner_state`](Self::partner_state). Docking/staging ticks use
    /// this; single-vehicle ticks keep [`step`](Self::step).
    pub fn step_many(
        &mut self,
        step_s: f64,
        wrenches: &[(CollisionBodyId, ExternalWrench)],
    ) -> Result<RigidBodyState, FlightError> {
        let id = self
            .body
            .as_ref()
            .map(|body| body.id)
            .ok_or_else(|| invalid("contact step needs a synced body"))?;
        self.world
            .step(step_s, wrenches.iter().copied())
            .map_err(|error| invalid(format!("contact step: {error}")))?;
        self.world
            .body_state(id)
            .map_err(|error| invalid(format!("contact readback: {error}")))
    }

    /// Read a synced partner back into authoritative state.
    pub fn partner_state(&self, tag: u64) -> Result<RigidBodyState, FlightError> {
        let id = self
            .partners
            .get(&tag)
            .map(|partner| partner.id)
            .ok_or_else(|| invalid("contact partner is not synced".to_string()))?;
        self.world
            .body_state(id)
            .map_err(|error| invalid(format!("contact partner readback: {error}")))
    }

    /// Rigidly dock the primary body to a synced partner at their local port
    /// frames (staging separation in reverse). Undock with
    /// [`undock`](Self::undock); removing either body drops the joint.
    #[allow(clippy::too_many_arguments)]
    pub fn dock_partner(
        &mut self,
        tag: u64,
        frame_primary_local_position_m: DVec3,
        frame_primary_local_orientation: DQuat,
        frame_partner_local_position_m: DVec3,
        frame_partner_local_orientation: DQuat,
    ) -> Result<JointId, FlightError> {
        let a = self
            .body
            .as_ref()
            .map(|body| body.id)
            .ok_or_else(|| invalid("docking needs a synced primary body".to_string()))?;
        let b = self
            .partners
            .get(&tag)
            .map(|partner| partner.id)
            .ok_or_else(|| invalid("docking needs a synced partner".to_string()))?;
        self.world
            .attach_fixed_joint(
                a,
                b,
                frame_primary_local_position_m,
                frame_primary_local_orientation,
                frame_partner_local_position_m,
                frame_partner_local_orientation,
            )
            .map_err(|error| invalid(format!("contact docking: {error}")))
    }

    /// Undock a fixed joint. Both clusters keep their solved state.
    pub fn undock(&mut self, id: JointId) -> Result<(), FlightError> {
        self.world
            .remove_joint(id)
            .map_err(|error| invalid(format!("contact undocking: {error}")))
    }

    /// Take this tick's contact load evidence for the damage/telemetry
    /// boundary. Empty until the first step, and replaced every step.
    pub fn drain_contact_events(&mut self) -> Vec<ContactSummary> {
        self.world.drain_contact_events()
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
    use thessa_sim_core::{
        AeroGeometry, AeroResult, AirlessWheelStructure, CollisionMaterial, CollisionPart,
        CollisionShape, ElectricMotorSpec, TireConstruction, WheelBrakeSpec, WheelChassisSpec,
        WheelDriveSpec, WheelLayout, WheelStrutSpec, WheelTireSpec, x15_contact_geometry,
    };

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

    fn zero_forces() -> FlightForces {
        FlightForces {
            environment: thessa_sim_core::AeroEnvironment::standard_sea_level(),
            aero: AeroResult {
                force_body_n: DVec3::ZERO,
                moment_body_nm: DVec3::ZERO,
                dynamic_pressure_pa: 0.0,
                mach: 0.0,
                reynolds_number: 0.0,
                panel_count: 0,
                panel_loads: None,
            },
            total_force_body_n: DVec3::ZERO,
            total_moment_body_nm: DVec3::ZERO,
            total_force_inertial_n: DVec3::ZERO,
            acceleration_inertial_mps2: DVec3::ZERO,
            angular_acceleration_body_rps2: DVec3::ZERO,
        }
    }

    fn one_wheel_vehicle(with_drive: bool) -> VehicleDefinition {
        let body_geometry = CollisionGeometry::new(vec![
            CollisionPart::new(
                DVec3::new(0.0, 0.0, 20.0),
                DQuat::IDENTITY,
                CollisionShape::Sphere { radius_m: 0.1 },
                CollisionMaterial::default(),
            )
            .unwrap(),
        ])
        .unwrap();
        let mut chassis_spec = WheelChassisSpec {
            name: "test-wheel".into(),
            mount_position_body_m: DVec3::ZERO,
            mount_orientation_body: DQuat::IDENTITY,
            length_m: 1.0,
            layout: WheelLayout::Inline,
            wheel_count: 1,
            structural_mass_kg: 10.0,
            structural_inertia_local_kg_m2: DMat3::from_diagonal(DVec3::splat(0.2)),
            tire: WheelTireSpec {
                construction: TireConstruction::Airless {
                    structure: AirlessWheelStructure::Spoked { spoke_count: 24 },
                    structure_density_kg_m3: 4_400.0,
                    minimum_temperature_k: 80.0,
                    maximum_temperature_k: 500.0,
                },
                radius_m: 0.32,
                width_m: 0.2,
                mass_kg: 2.0,
                spin_inertia_kg_m2: 0.08,
                radial_stiffness_n_m: 200_000.0,
                radial_damping_n_s_m: 5_000.0,
                longitudinal_slip_stiffness_n_per_mps: 12_000.0,
                lateral_slip_stiffness_n_per_mps: 9_000.0,
                maximum_deflection_m: 0.08,
                maximum_load_n: 8_000.0,
                surface_friction: 0.9,
            },
            strut: WheelStrutSpec {
                extended_length_m: 0.4,
                stroke_m: 0.15,
                spring_rate_n_m: 30_000.0,
                damping_n_s_m: 2_000.0,
                preload_n: 0.0,
                minimum_force_n: 0.0,
                maximum_force_n: 10_000.0,
                mass_per_wheel_kg: 0.5,
            },
            brake: WheelBrakeSpec {
                maximum_torque_nm: 200.0,
                response_time_s: 0.0,
                mass_per_wheel_kg: 0.3,
            },
            drive: with_drive.then_some(WheelDriveSpec {
                motor: ElectricMotorSpec {
                    rated_power_w: 100_000.0,
                    peak_torque_nm: 1_000.0,
                    maximum_rpm: 5_000.0,
                    efficiency: 0.9,
                    cooling_capacity_w: 10_000.0,
                    dry_mass_kg: 10.0,
                },
                stall_copper_loss_w: 100.0,
                rotor_inertia_kg_m2: 0.02,
                final_drive_ratio: 4.0,
                drivetrain_efficiency: 0.9,
                driven_wheel_count: 1,
            }),
        };
        let uncentered = chassis_spec.clone().compile().unwrap();
        chassis_spec.mount_position_body_m = -uncentered.mass_properties.center_of_mass_body_m;
        let mut vehicle = VehicleDefinition::new(
            "articulated-contact-test",
            AeroGeometry::default(),
            RigidBodyProperties::new(100.0, DMat3::from_diagonal(DVec3::splat(100.0))).unwrap(),
            Vec::new(),
        )
        .unwrap()
        .with_collision_geometry(body_geometry)
        .unwrap()
        .with_wheel_chassis(vec![chassis_spec])
        .unwrap();
        vehicle.bake_wheel_chassis_masses().unwrap();
        vehicle
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
    fn articulated_wheel_contacts_ground_and_brake_actuator_stops_spin() {
        let mut runtime = runtime();
        runtime
            .attach_static_patch(
                DVec3::new(0.0, 0.0, -0.5),
                DQuat::IDENTITY,
                DVec3::new(20.0, 20.0, 0.5),
                CollisionMaterial::new(0.25, 0.0).unwrap(),
            )
            .unwrap();
        let vehicle = one_wheel_vehicle(false);
        let station = vehicle.wheel_chassis[0].wheel_stations[0];
        let state = RigidBodyState::new(
            DVec3::new(
                0.0,
                0.0,
                vehicle.wheel_chassis[0].spec.tire.radius_m + 0.03 - station.position_body_m.z,
            ),
            DVec3::new(vehicle.wheel_chassis[0].spec.tire.radius_m * 5.0, 0.0, 0.0),
            DQuat::IDENTITY,
            DVec3::ZERO,
        )
        .unwrap();
        let mut wheel_spin = vec![vec![5.0]];
        let mut brake_states = vec![vec![WheelBrakeState::default()]];
        let forces = zero_forces();
        let gravity = DVec3::new(0.0, 0.0, -9.81);
        let mut next = state;
        for _ in 0..120 {
            (next, _, _) = runtime
                .step_articulated_vehicle(
                    1.0 / 120.0,
                    next,
                    gravity,
                    &forces,
                    &vehicle,
                    0.0,
                    0.0,
                    &mut wheel_spin,
                    &mut brake_states,
                )
                .unwrap();
        }
        let spin_before_braking = wheel_spin[0][0].abs();
        assert!(spin_before_braking > 1.0, "wheel should still be rolling");
        let mut saw_ground_contact = false;
        for _ in 0..60 {
            let (solved, contacts, _) = runtime
                .step_articulated_vehicle(
                    1.0 / 120.0,
                    next,
                    gravity,
                    &forces,
                    &vehicle,
                    1.0,
                    0.0,
                    &mut wheel_spin,
                    &mut brake_states,
                )
                .unwrap();
            next = solved;
            saw_ground_contact |= contacts.iter().any(|contact| contact.normal_load_n > 0.0);
        }
        assert!(saw_ground_contact, "tire law should report terrain load");
        assert!(
            wheel_spin[0][0].abs() < spin_before_braking,
            "service brake should reduce wheel spin"
        );
        assert!(next.position_inertial_m.is_finite());
        assert_eq!(runtime.world.dynamic_body_count(), 2);
        assert_eq!(runtime.world.joint_count(), 1);
    }

    #[test]
    fn articulated_electric_drive_produces_lunar_regolith_traction() {
        let mut runtime = runtime();
        runtime
            .attach_static_patch(
                DVec3::new(0.0, 0.0, -0.5),
                DQuat::IDENTITY,
                DVec3::new(20.0, 20.0, 0.5),
                CollisionMaterial::new(0.25, 0.0).unwrap(),
            )
            .unwrap();
        let vehicle = one_wheel_vehicle(true);
        let station = vehicle.wheel_chassis[0].wheel_stations[0];
        let state = RigidBodyState::new(
            DVec3::new(
                0.0,
                0.0,
                vehicle.wheel_chassis[0].spec.tire.radius_m - 0.005 - station.position_body_m.z,
            ),
            DVec3::ZERO,
            DQuat::IDENTITY,
            DVec3::ZERO,
        )
        .unwrap();
        let mut wheel_spin = vec![vec![0.0]];
        let mut brake_states = vec![vec![WheelBrakeState::default()]];
        let forces = zero_forces();
        let gravity = DVec3::new(0.0, 0.0, -1.62);
        let mut next = state;
        let mut last_drive_points = Vec::new();
        let mut regolith_contact = None;
        for _ in 0..24 {
            let (solved, contacts, drive_points) = runtime
                .step_articulated_vehicle(
                    1.0 / 120.0,
                    next,
                    gravity,
                    &forces,
                    &vehicle,
                    0.0,
                    1.0,
                    &mut wheel_spin,
                    &mut brake_states,
                )
                .unwrap();
            next = solved;
            last_drive_points = drive_points;
            if let Some(contact) = contacts.iter().find(|contact| contact.normal_load_n > 0.0) {
                regolith_contact = Some(*contact);
            }
        }
        assert!(next.velocity_inertial_mps.x > 0.05, "state={next:?}");
        assert!(wheel_spin[0][0] > 0.1, "spin={wheel_spin:?}");
        assert_eq!(last_drive_points.len(), 1);
        assert!(
            last_drive_points[0]
                .2
                .requested_wheel_torque_per_driven_wheel_nm
                > 0.0
        );
        let contact = regolith_contact.expect("airless wheel should contact regolith");
        assert!(contact.longitudinal_force_n > 0.0);
        assert!((contact.contact_friction - 0.575).abs() < 1.0e-12);
        assert!(
            contact.longitudinal_force_n.hypot(contact.lateral_force_n)
                <= contact.contact_friction * contact.normal_load_n + 1.0e-9
        );
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
    fn rejected_sync_keeps_existing_body_and_partner_registered() {
        let mut contact_runtime = runtime();
        let geometry = x15_contact_geometry().unwrap();
        let properties = test_properties();
        let config = DynamicBodyConfig::default();
        let state = RigidBodyState::stationary(DVec3::new(0.0, 5.0, 0.0));
        let body = contact_runtime
            .sync_body(state, properties, &geometry, config)
            .unwrap();
        let empty_geometry = CollisionGeometry::default();

        assert!(
            contact_runtime
                .sync_body(state, properties, &empty_geometry, config)
                .is_err()
        );
        assert_eq!(contact_runtime.body_id(), Some(body));
        assert_eq!(contact_runtime.world.dynamic_body_count(), 1);
        assert!(contact_runtime.world.body_state(body).is_ok());

        let partner = contact_runtime
            .sync_partner(9, state, properties, &geometry, config)
            .unwrap();
        assert!(
            contact_runtime
                .sync_partner(9, state, properties, &empty_geometry, config)
                .is_err()
        );
        assert_eq!(contact_runtime.partner_id(9), Some(partner));
        assert_eq!(contact_runtime.world.dynamic_body_count(), 2);
        assert!(contact_runtime.world.body_state(partner).is_ok());

        let mut articulated = runtime();
        let vehicle = one_wheel_vehicle(false);
        let articulated_state = RigidBodyState::stationary(DVec3::new(0.0, 0.0, 10.0));
        let sprung = articulated
            .sync_articulated_vehicle(articulated_state, &vehicle, &[vec![0.0]], config)
            .unwrap();
        assert_eq!(articulated.world.dynamic_body_count(), 2);
        assert!(
            articulated
                .sync_body(
                    articulated_state,
                    vehicle.mass_properties,
                    &empty_geometry,
                    config,
                )
                .is_err()
        );
        assert_eq!(articulated.body_id(), Some(sprung));
        assert!(articulated.wheel_assembly.is_some());
        assert_eq!(articulated.world.dynamic_body_count(), 2);
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

#[cfg(test)]
mod broadphase_tests {
    use super::*;
    use glam::{DMat3, DVec3};
    use thessa_sim_core::x15_contact_geometry;

    fn candidate(tag: u64, x: f64, radius: f64, speed: f64) -> ContactCandidate {
        ContactCandidate {
            tag,
            position_inertial_m: DVec3::new(x, 0.0, 0.0),
            bounding_radius_m: radius,
            speed_mps: speed,
        }
    }

    #[test]
    fn pairs_activate_with_hysteresis_and_report_edges() {
        let mut screen = ContactBroadPhase::new(50.0, 80.0, 0.0).unwrap();
        let far = [candidate(1, 0.0, 5.0, 0.0), candidate(2, 500.0, 5.0, 0.0)];
        assert!(screen.observe(&far).is_empty());
        // Surface distance 40 m: inside enter.
        let near = [candidate(1, 0.0, 5.0, 0.0), candidate(2, 50.0, 5.0, 0.0)];
        let report = screen.observe(&near);
        assert_eq!(report.len(), 1);
        assert!(report[0].active && report[0].just_activated);
        // Hysteresis band: 65 m stays active without a rising edge.
        let band = [candidate(1, 0.0, 5.0, 0.0), candidate(2, 75.0, 5.0, 0.0)];
        let report = screen.observe(&band);
        assert!(report[0].active && !report[0].just_activated);
        // Past exit: reported once as inactive, then disappears.
        let away = [candidate(1, 0.0, 5.0, 0.0), candidate(2, 200.0, 5.0, 0.0)];
        let report = screen.observe(&away);
        assert_eq!(report.len(), 1);
        assert!(!report[0].active);
        assert!(screen.observe(&away).is_empty());
    }

    #[test]
    fn horizon_and_uncertainty_are_conservative() {
        let mut screen = ContactBroadPhase::new(50.0, 80.0, 10.0).unwrap();
        // 150 m apart closing at 10 m/s each: 150 - 200 = -50 <= enter.
        let closing = [candidate(1, 0.0, 5.0, 10.0), candidate(2, 160.0, 5.0, 10.0)];
        assert!(screen.observe(&closing)[0].active);
        // Non-finite evidence never deactivates.
        let mut screen = ContactBroadPhase::new(50.0, 80.0, 0.0).unwrap();
        let bad = [
            candidate(1, 0.0, 5.0, 0.0),
            ContactCandidate {
                tag: 2,
                position_inertial_m: DVec3::new(f64::NAN, 0.0, 0.0),
                bounding_radius_m: 5.0,
                speed_mps: 0.0,
            },
        ];
        assert!(screen.observe(&bad)[0].active);
    }

    #[test]
    fn docked_partners_share_a_scene_and_drain_events() {
        let mut runtime = ContactRuntime::new(
            CollisionFrame::inertial_at(DVec3::ZERO, DVec3::ZERO),
            ContactActivation::new(50.0, 80.0).unwrap(),
        )
        .unwrap();
        let geometry = x15_contact_geometry().unwrap();
        let properties =
            RigidBodyProperties::new(100.0, DMat3::from_diagonal(DVec3::splat(500.0))).unwrap();
        let primary = runtime
            .sync_body(
                RigidBodyState::stationary(DVec3::new(0.0, 5.0, 0.0)),
                properties,
                &geometry,
                DynamicBodyConfig::default(),
            )
            .unwrap();
        let partner = runtime
            .sync_partner(
                7,
                RigidBodyState::stationary(DVec3::new(0.0, 12.0, 0.0)),
                properties,
                &geometry,
                DynamicBodyConfig::default(),
            )
            .unwrap();
        assert_eq!(runtime.partner_id(7), Some(partner));
        let joint = runtime
            .dock_partner(
                7,
                DVec3::ZERO,
                DQuat::IDENTITY,
                DVec3::ZERO,
                DQuat::IDENTITY,
            )
            .unwrap();
        let wrench = ExternalWrench {
            force_inertial_n: DVec3::new(0.0, -981.0, 0.0),
            torque_inertial_nm: DVec3::ZERO,
        };
        let next = runtime
            .step_many(1.0 / 120.0, &[(primary, wrench), (partner, wrench)])
            .unwrap();
        assert!(next.position_inertial_m.is_finite());
        let partner_state = runtime.partner_state(7).unwrap();
        assert!(partner_state.position_inertial_m.is_finite());
        runtime.undock(joint).unwrap();
        runtime.remove_partner(7).unwrap();
        assert_eq!(runtime.partner_id(7), None);
        // Drain works even with no contacts recorded.
        let _ = runtime.drain_contact_events();
    }
}
