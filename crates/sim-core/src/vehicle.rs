use std::{error::Error, fmt};

use glam::{DQuat, DVec3};
use serde::{Deserialize, Serialize};

use crate::{
    AeroConfig, AeroError, AeroGeometry, AeroPanel, CollisionAxis, CollisionError,
    CollisionGeometry, CollisionMaterial, CollisionPart, CollisionShape, CompiledEngine,
    EngineMount, EstocPoint, FlightCondition, FlightError, JetCommand, JetMount, PropulsionError,
    RigidBodyProperties, SystemMount, TankMount,
};

/// One user-configurable aerodynamic control channel.
///
/// A channel can drive one or more panels, which lets a vehicle compiler
/// represent a conventional elevator, split elevons, rudder, flaps or a
/// procedural control surface without changing the aero solver.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct ControlSurfaceDefinition {
    pub name: String,
    pub panel_indices: Vec<usize>,
    pub minimum_deflection_rad: f64,
    pub maximum_deflection_rad: f64,
    /// Hinge motion vs whole-surface rotation. The force solver treats
    /// both identically today (per-panel deflections); the mechanism
    /// mixer consumes the marker to rotate all-moving surfaces rigidly.
    #[serde(default)]
    pub kind: ControlKind,
    /// Nested-tab parent: index into the vehicle's control-surface list
    /// whose deflection this region rides. `None` for top-level regions.
    #[serde(default)]
    pub parent_index: Option<usize>,
}

/// Hinge motion (panels deflect about the hinge line) vs whole-surface
/// rotation (stabilator: the runtime rotates every addressed panel
/// rigidly instead).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize, Default)]
pub enum ControlKind {
    /// Panels deflect about their hinge lines.
    #[default]
    Hinge,
    /// The whole addressed surface rotates rigidly.
    AllMoving,
}

impl ControlSurfaceDefinition {
    pub fn new(
        name: impl Into<String>,
        panel_indices: Vec<usize>,
        minimum_deflection_rad: f64,
        maximum_deflection_rad: f64,
    ) -> Result<Self, VehicleError> {
        let definition = Self {
            name: name.into(),
            panel_indices,
            minimum_deflection_rad,
            maximum_deflection_rad,
            kind: ControlKind::Hinge,
            parent_index: None,
        };
        definition.validate(usize::MAX)?;
        Ok(definition)
    }

    /// Mark an all-moving surface (stabilator): the runtime rotates the
    /// addressed panels rigidly instead of deflecting them.
    pub fn with_kind(mut self, kind: ControlKind) -> Self {
        self.kind = kind;
        self
    }

    /// Attach a nested-tab parent (index into the vehicle control list).
    pub fn with_parent(mut self, parent_index: usize) -> Self {
        self.parent_index = Some(parent_index);
        self
    }

    fn validate(&self, panel_count: usize) -> Result<(), VehicleError> {
        if self.name.trim().is_empty() || self.panel_indices.is_empty() {
            return Err(VehicleError::InvalidControlSurface(
                "control surface needs a name and at least one panel".into(),
            ));
        }
        // One-sided devices (spoiler/airbrake/slat: minimum exactly 0)
        // are legal; negative commands park at 0 through the mapping
        // below. Strictly positive minima stay rejected.
        if !self.minimum_deflection_rad.is_finite()
            || !self.maximum_deflection_rad.is_finite()
            || self.minimum_deflection_rad > 0.0
            || self.maximum_deflection_rad <= 0.0
            || self.minimum_deflection_rad <= -std::f64::consts::PI
            || self.maximum_deflection_rad >= std::f64::consts::PI
        {
            return Err(VehicleError::InvalidControlSurface(format!(
                "{} has invalid deflection limits",
                self.name
            )));
        }
        if panel_count != usize::MAX
            && self
                .panel_indices
                .iter()
                .any(|panel_index| *panel_index >= panel_count)
        {
            return Err(VehicleError::InvalidControlSurface(format!(
                "{} references a panel outside the vehicle geometry",
                self.name
            )));
        }
        Ok(())
    }

    fn deflection_for_command(&self, command: f64) -> Result<f64, VehicleError> {
        if !command.is_finite() || !(-1.0..=1.0).contains(&command) {
            return Err(VehicleError::InvalidControlCommand {
                surface: self.name.clone(),
                command,
            });
        }
        Ok(if command >= 0.0 {
            command * self.maximum_deflection_rad
        } else {
            -command * self.minimum_deflection_rad
        })
    }
}

/// Generic runtime vehicle asset. It is deliberately agnostic to aircraft,
/// rocket or spacecraft shape: all vehicle-specific geometry stays in the
/// asset while the force/moment and rigid-body solvers remain reusable.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct VehicleDefinition {
    pub name: String,
    pub aero_geometry: AeroGeometry,
    pub mass_properties: RigidBodyProperties,
    pub control_surfaces: Vec<ControlSurfaceDefinition>,
    /// Solver-neutral contact geometry compiled from the same vehicle asset.
    ///
    /// Empty is a supported migration state for legacy assets that have not
    /// received collision geometry yet. Contact-active runtime code must refuse
    /// to create a dynamic collision body until this is populated.
    #[serde(default)]
    pub collision_geometry: CollisionGeometry,
    /// Installed procedural engines (compiled backend data + mount stations).
    /// Empty keeps every legacy asset valid; the baker aggregates engine
    /// masses into `mass_properties` when mounts are present.
    #[serde(default)]
    pub engines: Vec<EngineMount>,
    /// Installed propellant tanks (dry + full-fill mass aggregate at bake;
    /// depletion wiring is future work).
    #[serde(default)]
    pub tanks: Vec<TankMount>,
    /// Installed multi-chamber propulsion systems (chambers carry their
    /// own stations; mass aggregates per chamber at bake).
    #[serde(default)]
    pub systems: Vec<SystemMount>,
    /// Installed air-breathing jets (mass aggregates at bake; thrust
    /// needs a flight condition at query time).
    #[serde(default)]
    pub jets: Vec<JetMount>,
    /// Fold joints compiled from procedural surfaces (hinge placement in
    /// the compiled mechanism state). The force solver ignores them; the
    /// mechanism mixer transforms `fold_index`-tagged panels about these
    /// hinges. Empty keeps every legacy asset valid.
    #[serde(default)]
    pub fold_joints: Vec<FoldJointRecord>,
}

/// One compiled fold joint: hinge placement plus compiled angle, in
/// vehicle body metres. Panels tagged with this joint's index rotate
/// rigidly about the hinge axis; untagged panels stay put.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct FoldJointRecord {
    /// Joint name (`surface.joint` qualified by the baker across surfaces).
    pub name: String,
    /// Hinge point in body metres for the compiled mechanism state.
    pub hinge_body_m: DVec3,
    /// Unit hinge axis in body coordinates.
    pub axis_body: DVec3,
    /// Compiled angle in radians.
    pub angle_rad: f64,
    /// As-drawn flight (deployed) angle in radians: the runtime rotates
    /// tagged panels by `angle_rad - deployed_angle_rad`, so a vehicle
    /// baked deployed starts at the identity transform.
    pub deployed_angle_rad: f64,
    /// Parent joint in the fold hierarchy (index into the same vehicle
    /// joint list): panels tagged with this joint also ride every
    /// ancestor up to the root. `None` for root joints. The runtime
    /// builds the transform chain bone-style instead of storing chains
    /// on panels.
    #[serde(default)]
    pub parent_joint: Option<usize>,
    /// Deployment rate limit in rad/s (actuator data for the runtime).
    pub deployment_rate_rad_s: f64,
    /// Lock engagement window in radians (the lock may only engage
    /// inside it).
    pub lock_window_rad: (f64, f64),
    /// Flight-envelope gate in Pa: folding allowed at or below it,
    /// `None` for no q-gate.
    pub max_dynamic_pressure_pa: Option<f64>,
}

impl FoldJointRecord {
    /// Check finiteness, unit axis, and a named joint.
    pub fn validate(&self) -> Result<(), VehicleError> {
        if self.name.trim().is_empty() {
            return Err(VehicleError::InvalidControlSurface(
                "fold joint needs a non-empty name".into(),
            ));
        }
        if !self.hinge_body_m.is_finite() || !self.axis_body.is_finite() {
            return Err(VehicleError::InvalidControlSurface(format!(
                "fold joint '{}' has non-finite hinge data",
                self.name
            )));
        }
        if (self.axis_body.length() - 1.0).abs() > 1e-9 {
            return Err(VehicleError::InvalidControlSurface(format!(
                "fold joint '{}' axis must be unit length",
                self.name
            )));
        }
        if !self.angle_rad.is_finite() {
            return Err(VehicleError::InvalidControlSurface(format!(
                "fold joint '{}' angle must be finite",
                self.name
            )));
        }
        if !self.deployed_angle_rad.is_finite() {
            return Err(VehicleError::InvalidControlSurface(format!(
                "fold joint '{}' deployed angle must be finite",
                self.name
            )));
        }
        if !self.deployment_rate_rad_s.is_finite() || self.deployment_rate_rad_s <= 0.0 {
            return Err(VehicleError::InvalidControlSurface(format!(
                "fold joint '{}' needs a positive finite deployment rate",
                self.name
            )));
        }
        let (lock_min, lock_max) = self.lock_window_rad;
        if !lock_min.is_finite() || !lock_max.is_finite() || lock_min >= lock_max {
            return Err(VehicleError::InvalidControlSurface(format!(
                "fold joint '{}' lock window must be ordered and finite",
                self.name
            )));
        }
        if let Some(max_q) = self.max_dynamic_pressure_pa
            && (!max_q.is_finite() || max_q <= 0.0)
        {
            return Err(VehicleError::InvalidControlSurface(format!(
                "fold joint '{}' envelope gate must be positive and finite",
                self.name
            )));
        }
        Ok(())
    }
}

/// Physical starter data for the first powered flight profile.
///
/// The profile is deliberately separate from `VehicleDefinition`: geometry
/// and mass are serializable vehicle data, while the aero tuning and engine
/// limit are runtime defaults for the X-15 flight-test slice. The caller still
/// supplies gravity and atmosphere for the world being flown in.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct X15StarterProfile {
    pub vehicle: VehicleDefinition,
    pub aero_config: AeroConfig,
    pub max_thrust_n: f64,
    pub launch_forward_speed_mps: f64,
    pub launch_upward_speed_mps: f64,
}

impl X15StarterProfile {
    pub fn new() -> Result<Self, VehicleError> {
        let wing_left = AeroPanel::new(
            // Main wing ahead of the CG, conventional positive-lift-slope
            // tail aft. Positive alpha must lift the tail to lower the nose;
            // a negative lift slope would instead amplify the disturbance.
            DVec3::new(0.20, -1.40, 0.0),
            DVec3::X,
            DVec3::Z,
            9.29,
            3.10,
        )?
        .with_planform(5.70, 3.50, 35.0_f64.to_radians(), 0.86)?
        .with_thickness_ratio(0.085)?;
        let wing_right =
            AeroPanel::new(DVec3::new(0.20, 1.40, 0.0), DVec3::X, DVec3::Z, 9.29, 3.10)?
                .with_planform(5.70, 3.50, 35.0_f64.to_radians(), 0.86)?
                .with_thickness_ratio(0.085)?;
        let tail_left = AeroPanel::new(
            DVec3::new(-4.15, -0.45, 0.12),
            DVec3::X,
            DVec3::Z,
            1.30,
            1.10,
        )?
        .with_planform(2.25, 3.90, 25.0_f64.to_radians(), 0.94)?;
        let tail_right = AeroPanel::new(
            DVec3::new(-4.15, 0.45, 0.12),
            DVec3::X,
            DVec3::Z,
            1.30,
            1.10,
        )?
        .with_planform(2.25, 3.90, 25.0_f64.to_radians(), 0.94)?;
        let vertical_tail =
            AeroPanel::new(DVec3::new(-3.75, 0.0, 0.72), DVec3::X, DVec3::Y, 2.55, 1.70)?
                .with_planform(2.35, 2.15, 32.0_f64.to_radians(), 0.92)?
                .with_thickness_ratio(0.10)?;
        let geometry = AeroGeometry::new(vec![
            wing_left,
            wing_right,
            tail_left,
            tail_right,
            vertical_tail,
        ])?;
        let mass_properties = RigidBodyProperties::new(
            10_200.0,
            glam::DMat3::from_diagonal(DVec3::new(31_000.0, 115_000.0, 125_000.0)),
        )?;
        let control_surfaces = vec![
            ControlSurfaceDefinition::new(
                "elevator",
                vec![2, 3],
                -25.0_f64.to_radians(),
                25.0_f64.to_radians(),
            )?,
            ControlSurfaceDefinition::new(
                "rudder",
                vec![4],
                -22.0_f64.to_radians(),
                22.0_f64.to_radians(),
            )?,
            ControlSurfaceDefinition::new(
                "aileron-left",
                vec![0],
                -18.0_f64.to_radians(),
                18.0_f64.to_radians(),
            )?,
            ControlSurfaceDefinition::new(
                "aileron-right",
                vec![1],
                -18.0_f64.to_radians(),
                18.0_f64.to_radians(),
            )?,
        ];
        Ok(Self {
            vehicle: VehicleDefinition::new(
                "X-15 / THESSA FLIGHT TEST",
                geometry,
                mass_properties,
                control_surfaces,
            )?
            .with_collision_geometry(x15_contact_geometry()?)?,
            aero_config: AeroConfig {
                lift_slope_per_rad: 4.6,
                control_effectiveness: 0.82,
                stall_angle_rad: 22.0_f64.to_radians(),
                max_lift_coefficient: 1.45,
                base_drag_coefficient: 0.032,
                induced_drag_factor: 0.075,
                wave_drag_coefficient: 0.22,
                side_force_slope_per_rad: 1.10,
                pitching_moment_coefficient: -0.018,
                roll_damping_coefficient: -3.0,
                pitch_damping_coefficient: -4.0,
                yaw_damping_coefficient: -3.0,
                supersonic_lift_slope_factor: 4.0,
                supersonic_wave_drag_factor: 1.15,
                ..AeroConfig::default()
            },
            max_thrust_n: 254_000.0,
            launch_forward_speed_mps: 120.0,
            launch_upward_speed_mps: 8.0,
        })
    }

    pub fn thrust_body_n(&self, throttle: f64) -> Result<DVec3, VehicleError> {
        if !throttle.is_finite() || !(0.0..=1.0).contains(&throttle) {
            return Err(VehicleError::InvalidThrottle { throttle });
        }
        Ok(DVec3::X * (throttle * self.max_thrust_n))
    }
}

/// Solver-neutral contact geometry for the X-15 flight-test article.
///
/// Every primitive is placed at the same body stations as the aerodynamic
/// panels compiled above: the fuselage capsule spans the wing/tail stations
/// with nose/tail overhang from the real 15.45 m airframe length, the wing
/// cuboid covers the 6.8 m span at the wing station, and the tail cuboids sit
/// at the tail stations. No dimension is invented to exercise a solver; the
/// compound is the minimal convex cover of the flown shape for runway and
/// terrain contact.
pub fn x15_contact_geometry() -> Result<CollisionGeometry, VehicleError> {
    let material = CollisionMaterial::new(0.7, 0.0).map_err(VehicleError::Collision)?;
    let fuselage = CollisionPart::new(
        DVec3::new(0.75, 0.0, 0.1),
        DQuat::IDENTITY,
        CollisionShape::Capsule {
            axis: CollisionAxis::X,
            half_segment_m: 5.5,
            radius_m: 0.75,
        },
        material,
    )
    .map_err(VehicleError::Collision)?;
    let wing = CollisionPart::new(
        DVec3::new(0.20, 0.0, 0.0),
        DQuat::IDENTITY,
        CollisionShape::Cuboid {
            half_extents_m: DVec3::new(1.55, 3.4, 0.12),
        },
        material,
    )
    .map_err(VehicleError::Collision)?;
    let horizontal_tail = CollisionPart::new(
        DVec3::new(-4.15, 0.0, 0.12),
        DQuat::IDENTITY,
        CollisionShape::Cuboid {
            half_extents_m: DVec3::new(0.55, 1.6, 0.08),
        },
        material,
    )
    .map_err(VehicleError::Collision)?;
    let vertical_tail = CollisionPart::new(
        DVec3::new(-3.75, 0.0, 0.72),
        DQuat::IDENTITY,
        CollisionShape::Cuboid {
            half_extents_m: DVec3::new(0.85, 0.10, 1.18),
        },
        material,
    )
    .map_err(VehicleError::Collision)?;
    CollisionGeometry::new(vec![fuselage, wing, horizontal_tail, vertical_tail])
        .map_err(VehicleError::Collision)
}

impl VehicleDefinition {
    pub fn new(
        name: impl Into<String>,
        aero_geometry: AeroGeometry,
        mass_properties: RigidBodyProperties,
        control_surfaces: Vec<ControlSurfaceDefinition>,
    ) -> Result<Self, VehicleError> {
        let definition = Self {
            name: name.into(),
            aero_geometry,
            mass_properties,
            control_surfaces,
            collision_geometry: CollisionGeometry::default(),
            engines: Vec::new(),
            tanks: Vec::new(),
            systems: Vec::new(),
            jets: Vec::new(),
            fold_joints: Vec::new(),
        };
        definition.validate()?;
        Ok(definition)
    }

    /// Attach collision geometry produced by a vehicle compiler/baker without
    /// exposing any collision-backend type in the vehicle asset.
    pub fn with_collision_geometry(
        mut self,
        collision_geometry: CollisionGeometry,
    ) -> Result<Self, VehicleError> {
        collision_geometry
            .validate()
            .map_err(VehicleError::Collision)?;
        self.collision_geometry = collision_geometry;
        Ok(self)
    }

    pub fn validate(&self) -> Result<(), VehicleError> {
        if self.name.trim().is_empty() {
            return Err(VehicleError::InvalidVehicle(
                "vehicle name must not be empty".into(),
            ));
        }
        self.aero_geometry
            .validate()
            .map_err(VehicleError::Geometry)?;
        RigidBodyProperties::new(
            self.mass_properties.mass_kg,
            self.mass_properties.inertia_body_kg_m2,
        )
        .map_err(VehicleError::MassProperties)?;
        self.collision_geometry
            .validate()
            .map_err(VehicleError::Collision)?;
        for mount in &self.engines {
            mount.validate().map_err(VehicleError::Propulsion)?;
        }
        for mount in &self.tanks {
            mount.validate().map_err(VehicleError::Propulsion)?;
        }
        for mount in &self.systems {
            mount.validate().map_err(VehicleError::Propulsion)?;
        }
        for mount in &self.jets {
            mount.validate().map_err(VehicleError::Propulsion)?;
        }

        let mut claimed_panels = std::collections::HashSet::new();
        for (surface_index, surface) in self.control_surfaces.iter().enumerate() {
            surface.validate(self.aero_geometry.panels.len())?;
            for panel_index in &surface.panel_indices {
                if !claimed_panels.insert(*panel_index) {
                    return Err(VehicleError::InvalidControlSurface(format!(
                        "panel {panel_index} is assigned to more than one control surface"
                    )));
                }
            }
            // Nested-tab parents must exist, differ, and form no cycles.
            let mut chain = surface.parent_index;
            let mut seen = std::collections::HashSet::from([surface_index]);
            while let Some(parent) = chain {
                if parent >= self.control_surfaces.len() || !seen.insert(parent) {
                    return Err(VehicleError::InvalidControlSurface(format!(
                        "control surface '{}' has an invalid parent chain",
                        surface.name
                    )));
                }
                chain = self.control_surfaces[parent].parent_index;
            }
        }
        for joint in &self.fold_joints {
            joint.validate()?;
        }
        // Fold hierarchy: parents exist, differ, and form no cycles.
        for (joint_index, joint) in self.fold_joints.iter().enumerate() {
            let mut chain = joint.parent_joint;
            let mut seen = std::collections::HashSet::from([joint_index]);
            while let Some(parent) = chain {
                if parent >= self.fold_joints.len() || !seen.insert(parent) {
                    return Err(VehicleError::InvalidControlSurface(format!(
                        "fold joint '{}' has an invalid parent chain",
                        joint.name
                    )));
                }
                chain = self.fold_joints[parent].parent_joint;
            }
        }
        for (panel_index, panel) in self.aero_geometry.panels.iter().enumerate() {
            if let Some(joint) = panel.fold_index
                && joint >= self.fold_joints.len()
            {
                return Err(VehicleError::InvalidControlSurface(format!(
                    "panel {panel_index} references missing fold joint {joint}"
                )));
            }
        }
        Ok(())
    }

    /// Attach compiled engine mounts (baker path; validates the mounts).
    pub fn with_engines(mut self, engines: Vec<EngineMount>) -> Result<Self, VehicleError> {
        for mount in &engines {
            mount.validate().map_err(VehicleError::Propulsion)?;
        }
        self.engines = engines;
        Ok(self)
    }

    /// Attach compiled tank mounts (baker path; validates the mounts).
    pub fn with_tanks(mut self, tanks: Vec<TankMount>) -> Result<Self, VehicleError> {
        for mount in &tanks {
            mount.validate().map_err(VehicleError::Propulsion)?;
        }
        self.tanks = tanks;
        Ok(self)
    }

    /// Aggregate installed tank masses (dry + full propellant fill) as
    /// point masses at their stations. Called after
    /// [`VehicleDefinition::bake_engine_masses`].
    pub fn bake_tank_masses(&mut self) -> Result<(), VehicleError> {
        let mut mass_kg = self.mass_properties.mass_kg;
        let mut inertia = self.mass_properties.inertia_body_kg_m2;
        for mount in &self.tanks {
            let tank_mass_kg = mount.tank.dry_mass_kg + mount.tank.full_propellant_kg;
            let position = DVec3::from_array(mount.position_body_m);
            mass_kg += tank_mass_kg;
            inertia += tank_mass_kg
                * (glam::DMat3::IDENTITY * position.length_squared()
                    - outer_product(position, position));
        }
        self.mass_properties =
            RigidBodyProperties::new(mass_kg, inertia).map_err(VehicleError::MassProperties)?;
        Ok(())
    }

    /// Aggregate installed engine masses into the mass properties. Each
    /// engine rides as a point mass at its mount station (parallel-axis
    /// terms added to the input inertia, which keeps describing the base
    /// structure). The baker calls this after attaching mounts; input mass
    /// semantics are "structure without engines".
    pub fn bake_engine_masses(&mut self) -> Result<(), VehicleError> {
        let mut mass_kg = self.mass_properties.mass_kg;
        let mut inertia = self.mass_properties.inertia_body_kg_m2;
        for mount in &self.engines {
            let engine_mass_kg = mount.engine.bake_mass_kg();
            let position = DVec3::from_array(mount.position_body_m);
            mass_kg += engine_mass_kg;
            inertia += engine_mass_kg
                * (glam::DMat3::IDENTITY * position.length_squared()
                    - outer_product(position, position));
        }
        self.mass_properties =
            RigidBodyProperties::new(mass_kg, inertia).map_err(VehicleError::MassProperties)?;
        Ok(())
    }

    /// Attach compiled multi-chamber systems (baker path).
    pub fn with_systems(mut self, systems: Vec<SystemMount>) -> Result<Self, VehicleError> {
        for mount in &systems {
            mount.validate().map_err(VehicleError::Propulsion)?;
        }
        self.systems = systems;
        Ok(self)
    }

    /// Aggregate installed system masses: shared feed hardware rides at
    /// the system centroid, each chamber at its own station (point masses).
    /// Called after [`VehicleDefinition::bake_tank_masses`].
    pub fn bake_system_masses(&mut self) -> Result<(), VehicleError> {
        let mut mass_kg = self.mass_properties.mass_kg;
        let mut inertia = self.mass_properties.inertia_body_kg_m2;
        for mount in &self.systems {
            let chambers_dry_kg: f64 = mount.system.chambers.iter().map(|c| c.dry_mass_kg).sum();
            let shared_kg = (mount.system.dry_mass_kg - chambers_dry_kg).max(0.0);
            let mut centroid = DVec3::ZERO;
            let mut chamber_mass = 0.0;
            for chamber in &mount.system.chambers {
                let position = DVec3::from_array(chamber.position_body_m);
                mass_kg += chamber.dry_mass_kg;
                inertia += chamber.dry_mass_kg
                    * (glam::DMat3::IDENTITY * position.length_squared()
                        - outer_product(position, position));
                centroid += position * chamber.dry_mass_kg;
                chamber_mass += chamber.dry_mass_kg;
            }
            if shared_kg > 0.0 {
                let at = if chamber_mass > 0.0 {
                    centroid / chamber_mass
                } else {
                    DVec3::ZERO
                };
                mass_kg += shared_kg;
                inertia += shared_kg
                    * (glam::DMat3::IDENTITY * at.length_squared() - outer_product(at, at));
            }
        }
        self.mass_properties =
            RigidBodyProperties::new(mass_kg, inertia).map_err(VehicleError::MassProperties)?;
        Ok(())
    }

    /// Attach installed jets (baker path).
    pub fn with_jets(mut self, jets: Vec<JetMount>) -> Result<Self, VehicleError> {
        for mount in &jets {
            mount.validate().map_err(VehicleError::Propulsion)?;
        }
        self.jets = jets;
        Ok(self)
    }

    /// Attach compiled fold joints (baker path; validates records and
    /// panel/joint references through full validation).
    pub fn with_fold_joints(
        mut self,
        fold_joints: Vec<FoldJointRecord>,
    ) -> Result<Self, VehicleError> {
        self.fold_joints = fold_joints;
        self.validate()?;
        Ok(self)
    }

    /// Aggregate installed jet masses as point masses at their stations.
    /// Called after [`VehicleDefinition::bake_system_masses`].
    pub fn bake_jet_masses(&mut self) -> Result<(), VehicleError> {
        let mut mass_kg = self.mass_properties.mass_kg;
        let mut inertia = self.mass_properties.inertia_body_kg_m2;
        for mount in &self.jets {
            let jet_mass_kg = mount.engine.dry_mass_kg();
            let position = DVec3::from_array(mount.position_body_m);
            mass_kg += jet_mass_kg;
            inertia += jet_mass_kg
                * (glam::DMat3::IDENTITY * position.length_squared()
                    - outer_product(position, position));
        }
        self.mass_properties =
            RigidBodyProperties::new(mass_kg, inertia).map_err(VehicleError::MassProperties)?;
        Ok(())
    }

    /// Thrust vector of one mount in body axes (N).
    pub fn engine_thrust_body_n(
        &self,
        index: usize,
        throttle: f64,
        ambient_pa: f64,
        burn_time_s: f64,
    ) -> Result<DVec3, VehicleError> {
        let mount = self
            .engines
            .get(index)
            .ok_or_else(|| VehicleError::InvalidVehicle(format!("no engine at index {index}")))?;
        mount
            .thrust_vector_body_n(throttle, ambient_pa, burn_time_s)
            .map(DVec3::from_array)
            .map_err(VehicleError::Propulsion)
    }

    /// Total installed thrust in body axes at a uniform command (editor
    /// preview / single-lever path; per-engine allocation is later work).
    /// Multi-chamber systems fire all chambers at the same throttle.
    pub fn total_thrust_body_n(
        &self,
        throttle: f64,
        ambient_pa: f64,
        burn_time_s: f64,
    ) -> Result<DVec3, VehicleError> {
        let mut total = DVec3::ZERO;
        for index in 0..self.engines.len() {
            total += self.engine_thrust_body_n(index, throttle, ambient_pa, burn_time_s)?;
        }
        for mount in &self.systems {
            let throttles = vec![throttle; mount.system.chambers.len()];
            let point = mount
                .system
                .operating_point(&throttles, ambient_pa)
                .map_err(VehicleError::Propulsion)?;
            // The shared GG duct has no station of its own: distribute its
            // thrust across chambers proportionally (documented).
            let main: f64 = point.chambers.iter().map(|c| c.thrust_n).sum();
            let scale = if main > 0.0 {
                point.thrust_n / main
            } else {
                1.0
            };
            for (chamber, chamber_point) in mount.system.chambers.iter().zip(point.chambers.iter())
            {
                total +=
                    DVec3::from_array(chamber.thrust_axis_body) * (chamber_point.thrust_n * scale);
            }
        }
        Ok(total)
    }

    /// Force/moment wrench of one multi-chamber system at per-chamber
    /// throttles (differential authority for the allocator).
    pub fn system_wrench_body_n(
        &self,
        index: usize,
        throttles: &[f64],
        ambient_pa: f64,
    ) -> Result<(DVec3, DVec3), VehicleError> {
        let mount = self.systems.get(index).ok_or_else(|| {
            VehicleError::InvalidVehicle(format!("no propulsion system at index {index}"))
        })?;
        mount
            .system
            .wrench_body_n(throttles, ambient_pa)
            .map_err(VehicleError::Propulsion)
    }

    /// Thrust vector of one jet in body axes (N) at throttle, condition,
    /// and jet command.
    pub fn jet_thrust_body_n(
        &self,
        index: usize,
        throttle: f64,
        condition: &FlightCondition,
        jet: &JetCommand,
    ) -> Result<DVec3, VehicleError> {
        let mount = self
            .jets
            .get(index)
            .ok_or_else(|| VehicleError::InvalidVehicle(format!("no jet at index {index}")))?;
        mount
            .thrust_vector_body_n(throttle, condition, jet)
            .map(DVec3::from_array)
            .map_err(VehicleError::Propulsion)
    }

    /// Evaluate one jet and return the command state for the next world tick.
    /// Plain air-breathers still return an `EstocPoint`-shaped snapshot so a
    /// vehicle caller can use one state-threading path for mixed jet mounts.
    /// The returned command carries the advanced shaft state (spool, lit,
    /// starter charge) alongside mode and transition snapshot.
    pub fn jet_estoc_point(
        &self,
        index: usize,
        throttle: f64,
        condition: &FlightCondition,
        jet: &JetCommand,
    ) -> Result<(EstocPoint, JetCommand), VehicleError> {
        let mount = self
            .jets
            .get(index)
            .ok_or_else(|| VehicleError::InvalidVehicle(format!("no jet at index {index}")))?;
        let (point, transient, shaft) = mount
            .estoc_point(throttle, condition, jet)
            .map_err(VehicleError::Propulsion)?;
        let next = jet.with_state(&point, transient, shaft);
        Ok((point, next))
    }

    /// Force/moment wrench of all jets at per-mount (throttle, jet
    /// command) pairs: engine-out and asymmetric-reheat steering fall out
    /// of the stations for free.
    pub fn jets_wrench_body_n(
        &self,
        commands: &[(f64, JetCommand)],
        condition: &FlightCondition,
    ) -> Result<(DVec3, DVec3), VehicleError> {
        self.jets_wrench_body_n_stateful(commands, condition)
            .map(|(wrench, _)| wrench)
    }

    /// Stateful variant of [`VehicleDefinition::jets_wrench_body_n`]. The
    /// returned commands contain each mount's updated ESTOC mode, full
    /// transition snapshot, and advanced shaft state, and must be fed
    /// into the next world tick.
    pub fn jets_wrench_body_n_stateful(
        &self,
        commands: &[(f64, JetCommand)],
        condition: &FlightCondition,
    ) -> Result<((DVec3, DVec3), Vec<JetCommand>), VehicleError> {
        if commands.len() != self.jets.len() {
            return Err(VehicleError::InvalidVehicle(format!(
                "expected {} jet commands, got {}",
                self.jets.len(),
                commands.len()
            )));
        }
        let mut force = DVec3::ZERO;
        let mut moment = DVec3::ZERO;
        let mut next_commands = Vec::with_capacity(commands.len());
        for (mount, (throttle, jet)) in self.jets.iter().zip(commands) {
            let (point, transient, shaft) = mount
                .estoc_point(*throttle, condition, jet)
                .map_err(VehicleError::Propulsion)?;
            let thrust = DVec3::from_array(mount.thrust_axis_body) * point.thrust_n;
            force += thrust;
            moment += DVec3::from_array(mount.position_body_m).cross(thrust);
            next_commands.push(jet.with_state(&point, transient, shaft));
        }
        Ok(((force, moment), next_commands))
    }

    /// Force/moment wrench in body axes at per-mount commands: force is the
    /// thrust sum, moment the sum of mount-station cross thrust about the
    /// body origin. Covers single engines (`commands` carries one
    /// (throttle, burn clock) per mount); multi-chamber systems ride
    /// [`VehicleDefinition::system_wrench_body_n`].
    pub fn wrench_body_n(
        &self,
        commands: &[(f64, f64)],
        ambient_pa: f64,
    ) -> Result<(DVec3, DVec3), VehicleError> {
        if commands.len() != self.engines.len() {
            return Err(VehicleError::InvalidVehicle(format!(
                "expected {} commands, got {}",
                self.engines.len(),
                commands.len()
            )));
        }
        let mut force = DVec3::ZERO;
        let mut moment = DVec3::ZERO;
        for (mount, (throttle, burn_time_s)) in self.engines.iter().zip(commands) {
            let thrust = DVec3::from_array(
                mount
                    .thrust_vector_body_n(*throttle, ambient_pa, *burn_time_s)
                    .map_err(VehicleError::Propulsion)?,
            );
            force += thrust;
            moment += DVec3::from_array(mount.position_body_m).cross(thrust);
        }
        Ok((force, moment))
    }

    /// Current vehicle mass with solid propellant burned off: baked mass
    /// minus consumed grain per engine burn clock. `burn_times_s` must
    /// carry one clock per mount (liquids ignore theirs). Inertia is left
    /// at the baked value (documented approximation for the flight loop to
    /// refine once off-axis depletion matters).
    pub fn depleted_mass_kg(&self, burn_times_s: &[f64]) -> Result<f64, VehicleError> {
        if burn_times_s.len() != self.engines.len() {
            return Err(VehicleError::InvalidVehicle(format!(
                "expected {} burn clocks, got {}",
                self.engines.len(),
                burn_times_s.len()
            )));
        }
        let mut mass_kg = self.mass_properties.mass_kg;
        for (mount, burn_time_s) in self.engines.iter().zip(burn_times_s) {
            if let Some(remaining) = mount.engine.propellant_remaining_kg(*burn_time_s) {
                let total = match &mount.engine {
                    CompiledEngine::Solid(solid) => solid.propellant_mass_kg,
                    _ => 0.0,
                };
                mass_kg -= (total - remaining).max(0.0);
            }
        }
        RigidBodyProperties::new(mass_kg, self.mass_properties.inertia_body_kg_m2)
            .map(|_| mass_kg)
            .map_err(VehicleError::MassProperties)
    }

    /// Apply normalized control commands in `[-1, 1]` to this vehicle's
    /// panels. It mutates only the asset's control deflections; geometry,
    /// mass and solver configuration remain unchanged.
    pub fn apply_control_inputs(&mut self, commands: &[f64]) -> Result<(), VehicleError> {
        if commands.len() != self.control_surfaces.len() {
            return Err(VehicleError::ControlCount {
                expected: self.control_surfaces.len(),
                actual: commands.len(),
            });
        }
        let deflections = self
            .control_surfaces
            .iter()
            .zip(commands)
            .map(|(surface, command)| surface.deflection_for_command(*command))
            .collect::<Result<Vec<_>, _>>()?;
        for ((surface, _), deflection) in
            self.control_surfaces.iter().zip(commands).zip(deflections)
        {
            for panel_index in &surface.panel_indices {
                self.aero_geometry.panels[*panel_index].control_deflection_rad = deflection;
            }
        }
        self.aero_geometry
            .validate()
            .map_err(VehicleError::Geometry)
    }
}

#[derive(Debug, Clone, PartialEq)]
pub enum VehicleError {
    InvalidVehicle(String),
    Geometry(AeroError),
    Collision(CollisionError),
    MassProperties(FlightError),
    Propulsion(PropulsionError),
    InvalidControlSurface(String),
    InvalidControlCommand { surface: String, command: f64 },
    ControlCount { expected: usize, actual: usize },
    InvalidThrottle { throttle: f64 },
}

impl fmt::Display for VehicleError {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::InvalidVehicle(message) => write!(formatter, "invalid vehicle: {message}"),
            Self::Geometry(error) => write!(formatter, "vehicle geometry error: {error}"),
            Self::Collision(error) => {
                write!(formatter, "vehicle collision geometry error: {error}")
            }
            Self::MassProperties(error) => {
                write!(formatter, "invalid vehicle mass properties: {error}")
            }
            Self::Propulsion(error) => {
                write!(formatter, "vehicle engine error: {error}")
            }
            Self::InvalidControlSurface(message) => {
                write!(formatter, "invalid control surface: {message}")
            }
            Self::InvalidControlCommand { surface, command } => {
                write!(
                    formatter,
                    "invalid command {command} for control surface {surface}"
                )
            }
            Self::ControlCount { expected, actual } => write!(
                formatter,
                "vehicle expected {expected} control commands, received {actual}"
            ),
            Self::InvalidThrottle { throttle } => {
                write!(
                    formatter,
                    "vehicle throttle must be finite and in [0, 1], got {throttle}"
                )
            }
        }
    }
}

impl Error for VehicleError {}

impl From<AeroError> for VehicleError {
    fn from(error: AeroError) -> Self {
        Self::Geometry(error)
    }
}

impl From<CollisionError> for VehicleError {
    fn from(error: CollisionError) -> Self {
        Self::Collision(error)
    }
}

impl From<FlightError> for VehicleError {
    fn from(error: FlightError) -> Self {
        Self::MassProperties(error)
    }
}

/// Rank-one outer product for parallel-axis aggregation.
fn outer_product(a: DVec3, b: DVec3) -> glam::DMat3 {
    glam::DMat3::from_cols(a * b.x, a * b.y, a * b.z)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{
        AirCycle, AirbreathingSpec, ChamberMaterial, EstocMode, EstocSpec, IntakeKind, JetFuel,
    };

    fn test_vehicle() -> VehicleDefinition {
        let geometry = AeroGeometry::new(vec![
            AeroPanel::new(DVec3::ZERO, DVec3::X, DVec3::Z, 1.0, 1.0).expect("panel"),
        ])
        .expect("geometry");
        let mass =
            RigidBodyProperties::new(1000.0, glam::DMat3::from_diagonal(DVec3::splat(1000.0)))
                .expect("mass");
        let vehicle =
            VehicleDefinition::new("jet-state-test", geometry, mass, vec![]).expect("vehicle");
        let engine = EstocSpec {
            name: "state-test-estoc".into(),
            air: AirbreathingSpec {
                name: "state-test-air".into(),
                cycle: AirCycle::Turbojet,
                fuel: JetFuel::Kerosene,
                intake_area_m2: 0.9,
                intake: IntakeKind::Pitot,
                compressor_ratio: 12.0,
                bypass_ratio: 0.0,
                fan_pressure_ratio: 1.0,
                turbine_inlet_temp_k: 1500.0,
                afterburner: false,
                reheat_temp_k: 0.0,
                turbine_material: ChamberMaterial::nickel_superalloy(),
                spool_tau_s: 5.0,
                shaft: crate::ShaftSpec::default(),
            },
            rocket_chamber_pressure_pa: 7.0e6,
            rocket_throat_radius_m: 0.09,
            oxidizer_fuel_ratio: None,
            switch_mach_hi: None,
            switch_mach_lo: None,
            transition_tau_s: None,
        }
        .compile()
        .expect("estoc");
        vehicle
            .with_jets(vec![JetMount {
                name: "state-test-mount".into(),
                engine: crate::CompiledJet::Estoc(Box::new(engine)),
                position_body_m: [0.0, 1.0, 0.0],
                thrust_axis_body: [1.0, 0.0, 0.0],
                gimbal_range_rad: 0.0,
            }])
            .expect("jet mount")
    }

    fn high_mach_condition() -> FlightCondition {
        let temperature_k = 288.15;
        let speed_of_sound = (crate::AIR_GAMMA * 287.0 * temperature_k).sqrt();
        FlightCondition {
            mach: 4.0,
            ambient_pa: 2_000.0,
            ambient_temp_k: temperature_k,
            airspeed_mps: 4.0 * speed_of_sound,
            oxygen_fraction: crate::EARTH_OXYGEN_FRACTION,
        }
    }

    #[test]
    fn stateful_jet_wrench_returns_next_estoc_commands() {
        let vehicle = test_vehicle();
        let command = JetCommand {
            manual: None,
            last_mode: EstocMode::Air,
            prev: None,
            dt_s: 1.0,
            ..JetCommand::fresh()
        };
        let ((force, moment), next) = vehicle
            .jets_wrench_body_n_stateful(&[(1.0, command)], &high_mach_condition())
            .expect("stateful wrench");
        assert_eq!(next.len(), 1);
        assert_eq!(next[0].last_mode, EstocMode::Rocket);
        assert!(next[0].prev.is_some());
        assert!(force.x > 0.0);
        assert!(moment.z.abs() > 0.0);

        let ((continued_force, _), continued) = vehicle
            .jets_wrench_body_n_stateful(&[(1.0, next[0])], &high_mach_condition())
            .expect("continued wrench");
        assert!(continued_force.x > 0.0);
        assert_eq!(continued[0].last_mode, EstocMode::Rocket);
        assert!(continued[0].prev.is_some());
    }
}
