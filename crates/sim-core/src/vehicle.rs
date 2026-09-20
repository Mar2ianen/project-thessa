use std::{error::Error, fmt};

use glam::{DQuat, DVec3};
use serde::{Deserialize, Serialize};

use crate::{
    AeroConfig, AeroError, AeroGeometry, AeroPanel, CollisionAxis, CollisionError,
    CollisionGeometry, CollisionMaterial, CollisionPart, CollisionShape, EngineMount, FlightError,
    PropulsionError, RigidBodyProperties,
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
        };
        definition.validate(usize::MAX)?;
        Ok(definition)
    }

    fn validate(&self, panel_count: usize) -> Result<(), VehicleError> {
        if self.name.trim().is_empty() || self.panel_indices.is_empty() {
            return Err(VehicleError::InvalidControlSurface(
                "control surface needs a name and at least one panel".into(),
            ));
        }
        if !self.minimum_deflection_rad.is_finite()
            || !self.maximum_deflection_rad.is_finite()
            || self.minimum_deflection_rad >= 0.0
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

        let mut claimed_panels = std::collections::HashSet::new();
        for surface in &self.control_surfaces {
            surface.validate(self.aero_geometry.panels.len())?;
            for panel_index in &surface.panel_indices {
                if !claimed_panels.insert(*panel_index) {
                    return Err(VehicleError::InvalidControlSurface(format!(
                        "panel {panel_index} is assigned to more than one control surface"
                    )));
                }
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
        Ok(total)
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
