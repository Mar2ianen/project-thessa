use std::{error::Error, fmt};

use glam::{DMat3, DQuat, DVec3};
use serde::{Deserialize, Serialize};

use crate::{
    AeroConfig, AeroError, AeroGeometry, AeroPanel, AeroResult, AuthorityReason,
    AuxiliaryPowerUnitMount, CabinError, CabinExit, CabinMonument, CabinSeat, CollisionAxis,
    CollisionError, CollisionGeometry, CollisionMaterial, CollisionPart, CollisionShape,
    CompiledEngine, CompiledLandingLeg, CompiledWheelChassis, ControlAuthority, ControlCore,
    ControlStation, CrewSuitMode, ElectricThrusterCommand, ElectricThrusterMount,
    ElectricThrusterPoint, ElectricalPowerCommand, ElectricalPowerError, ElectricalPowerState,
    ElectricalPowerSystem, ElectricalPowerTelemetry, EngineMount, EstocPoint, FlightCondition,
    FlightError, FusionTorchCommand, FusionTorchMount, FusionTorchOperatingPoint, HeatShieldMount,
    JetCommand, JetMount, LandingGearError, LandingLegMassProperties, LandingLegSpec,
    ParachuteError, ParachuteSpec, PressurizedCabin, PropDrivePoint, PropellerDriveCommand,
    PropellerDriveMount, PropulsionError, PulsedFusionCommand, PulsedFusionMount,
    PulsedFusionOperatingPoint, PulsedFusionState, RcsMount, ReactionWheelBankSpec,
    ReactionWheelError, RigidBodyProperties, ShieldError, SolarOccluder, StoredPropellant,
    SystemMount, TankMount, ThermalCommand, ThermalError, ThermalState, ThermalSystem,
    ThermalTelemetry, TurbopropCommand, TurbopropMount, TurbopropOperatingPoint, VehicleAssembly,
    VehicleResourceDemand, VehicleResourceFeedPort, VehicleResourceState, WheelBodyMassProperties,
    WheelChassisMassProperties, WheelChassisSpec, WheelChassisState, control_authority,
};

pub type StatefulTurbopropWrench = (
    (DVec3, DVec3),
    Vec<(TurbopropOperatingPoint, TurbopropCommand)>,
);

pub type StatefulPulsedFusionWrench = (
    (DVec3, DVec3),
    Vec<(PulsedFusionState, PulsedFusionOperatingPoint)>,
);

/// Normalized pilot/control channels consumed by baked surface mixers.
#[derive(Debug, Clone, Copy, Default, PartialEq, Serialize, Deserialize)]
pub struct ControlChannels {
    /// Pitch command (`[-1, 1]`).
    pub pitch: f64,
    /// Roll command (`[-1, 1]`).
    pub roll: f64,
    /// Yaw command (`[-1, 1]`).
    pub yaw: f64,
    /// Flap deployment (`0..=1` typical).
    pub flap: f64,
    /// Airbrake deployment (`0..=1` typical).
    pub airbrake: f64,
}

impl ControlChannels {
    /// Neutral sticks and retracted auxiliary controls.
    pub fn neutral() -> Self {
        Self::default()
    }
}

/// Per-surface gains mapping normalized channels to a normalized actuator
/// command. The result saturates to `[-1, 1]` before surface limits apply.
#[derive(Debug, Clone, Copy, Default, PartialEq, Serialize, Deserialize)]
pub struct ControlMixing {
    /// Pitch channel gain.
    pub pitch: f64,
    /// Roll channel gain.
    pub roll: f64,
    /// Yaw channel gain.
    pub yaw: f64,
    /// Flap channel gain.
    pub flap: f64,
    /// Airbrake channel gain.
    pub airbrake: f64,
}

impl ControlMixing {
    pub fn command(self, channels: ControlChannels) -> f64 {
        (self.pitch * channels.pitch
            + self.roll * channels.roll
            + self.yaw * channels.yaw
            + self.flap * channels.flap
            + self.airbrake * channels.airbrake)
            .clamp(-1.0, 1.0)
    }

    fn validate(self) -> bool {
        [self.pitch, self.roll, self.yaw, self.flap, self.airbrake]
            .into_iter()
            .all(f64::is_finite)
    }
}

/// One user-configurable aerodynamic control surface. A surface can own one
/// or more panels and carries its optional baked channel mixer.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct ControlSurfaceDefinition {
    pub name: String,
    pub panel_indices: Vec<usize>,
    pub minimum_deflection_rad: f64,
    pub maximum_deflection_rad: f64,
    /// Hinge device vs all-moving surface classification. Geometric motion
    /// is enabled by `hinge`; controls without hinge data retain the legacy
    /// incidence-style response.
    #[serde(default)]
    pub kind: ControlKind,
    /// Nested-tab parent: index into the vehicle's control-surface list
    /// whose deflection this region rides. `None` for top-level regions.
    #[serde(default)]
    pub parent_index: Option<usize>,
    /// Body-frame hinge line for a geometrically moving control. `None`
    /// retains the incidence-style panel response.
    #[serde(default)]
    pub hinge: Option<ControlHinge>,
    /// Rated no-load angular rate and stall torque. When present, actuator
    /// rate falls linearly to zero as opposing aerodynamic hinge torque
    /// reaches the stall rating.
    #[serde(default)]
    pub actuator: Option<ControlSurfaceActuator>,
    /// Explicit pitch/roll/yaw/flap/airbrake gains. Missing mixers preserve
    /// the four-channel legacy X-15 mapping for old baked assets; later
    /// unconfigured surfaces remain neutral rather than indexing past it.
    #[serde(default)]
    pub mixing: Option<ControlMixing>,
}

/// A straight hinge line in vehicle body coordinates.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct ControlHinge {
    pub point_body_m: DVec3,
    pub axis_body: DVec3,
}

impl ControlHinge {
    pub fn new(point_body_m: DVec3, axis_body: DVec3) -> Result<Self, VehicleError> {
        let axis_length = axis_body.length();
        if !point_body_m.is_finite()
            || !axis_body.is_finite()
            || !axis_length.is_finite()
            || axis_length <= 1.0e-12
        {
            return Err(VehicleError::InvalidControlSurface(
                "hinge needs a finite point and non-zero finite axis".into(),
            ));
        }
        Ok(Self {
            point_body_m,
            axis_body: axis_body / axis_length,
        })
    }
}

/// Physical actuator ratings used by the control-surface mechanism model.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct ControlSurfaceActuator {
    /// Maximum angular rate at zero aerodynamic load (rad/s).
    pub max_rate_rad_s: f64,
    /// Stall torque about the hinge (N·m).
    pub max_torque_nm: f64,
}

impl ControlSurfaceActuator {
    pub fn validate(self) -> Result<(), VehicleError> {
        if !self.max_rate_rad_s.is_finite()
            || self.max_rate_rad_s <= 0.0
            || !self.max_torque_nm.is_finite()
            || self.max_torque_nm <= 0.0
        {
            return Err(VehicleError::InvalidControlSurface(
                "actuator rate and stall torque must be positive and finite".into(),
            ));
        }
        Ok(())
    }

    /// Advance toward a commanded position using a linear torque-speed
    /// envelope: max rate at zero opposing load, zero rate at stall torque.
    /// Loads that assist motion do not raise speed above the no-load rating.
    pub fn advance(
        self,
        current_rad: f64,
        target_rad: f64,
        aerodynamic_hinge_torque_nm: f64,
        dt_s: f64,
        minimum_rad: f64,
        maximum_rad: f64,
    ) -> Result<f64, VehicleError> {
        self.validate()?;
        if !current_rad.is_finite()
            || !target_rad.is_finite()
            || !aerodynamic_hinge_torque_nm.is_finite()
            || !dt_s.is_finite()
            || dt_s < 0.0
            || !minimum_rad.is_finite()
            || !maximum_rad.is_finite()
            || minimum_rad > maximum_rad
        {
            return Err(VehicleError::InvalidControlSurface(
                "actuator state, load, timestep, or limits are invalid".into(),
            ));
        }
        let current = current_rad.clamp(minimum_rad, maximum_rad);
        let target = target_rad.clamp(minimum_rad, maximum_rad);
        let remaining = target - current;
        if remaining == 0.0 || dt_s == 0.0 {
            return Ok(current);
        }
        let direction = remaining.signum();
        let opposing_torque = (-direction * aerodynamic_hinge_torque_nm).max(0.0);
        let available_rate =
            self.max_rate_rad_s * (1.0 - opposing_torque / self.max_torque_nm).clamp(0.0, 1.0);
        let step = remaining.abs().min(available_rate * dt_s) * direction;
        Ok((current + step).clamp(minimum_rad, maximum_rad))
    }
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
            hinge: None,
            actuator: None,
            mixing: None,
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

    /// Attach body-frame hinge geometry for rigid panel motion.
    pub fn with_hinge(mut self, hinge: ControlHinge) -> Self {
        self.hinge = Some(hinge);
        self
    }

    /// Attach physical no-load rate and stall-torque ratings.
    pub fn with_actuator(mut self, actuator: ControlSurfaceActuator) -> Self {
        self.actuator = Some(actuator);
        self
    }

    /// Attach baked channel gains for this surface.
    pub fn with_mixing(mut self, mixing: ControlMixing) -> Self {
        self.mixing = Some(mixing);
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
        if let Some(hinge) = self.hinge {
            let axis_length = hinge.axis_body.length();
            if !hinge.point_body_m.is_finite()
                || !hinge.axis_body.is_finite()
                || !axis_length.is_finite()
                || (axis_length - 1.0).abs() > 1.0e-6
            {
                return Err(VehicleError::InvalidControlSurface(format!(
                    "{} has invalid hinge geometry",
                    self.name
                )));
            }
        }
        if let Some(actuator) = self.actuator {
            actuator.validate()?;
        }
        if self.mixing.is_some_and(|mixing| !mixing.validate()) {
            return Err(VehicleError::InvalidControlSurface(format!(
                "{} has non-finite control mixing gains",
                self.name
            )));
        }
        Ok(())
    }

    pub fn deflection_for_command(&self, command: f64) -> Result<f64, VehicleError> {
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

/// Build a command for every compiled surface in its current definition
/// order. Explicit baked mixers are order-independent; unmixed legacy assets
/// retain the original X-15 elevator/rudder/left-aileron/right-aileron map.
pub fn control_surface_commands(
    surfaces: &[ControlSurfaceDefinition],
    channels: ControlChannels,
) -> Vec<f64> {
    surfaces
        .iter()
        .enumerate()
        .map(|(index, surface)| {
            surface.mixing.map_or_else(
                || match index {
                    0 => -channels.pitch,
                    1 => channels.yaw,
                    2 => -channels.roll,
                    3 => channels.roll,
                    _ => 0.0,
                },
                |mixing| mixing.command(channels),
            )
        })
        .collect()
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
    /// Installed propellant tanks (dry + initial-load mass aggregate at bake;
    /// rocket and named pure-fluid runtime consumers use this live inventory).
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
    /// Installed turbo-generator auxiliary power units.
    #[serde(default)]
    pub auxiliary_power_units: Vec<AuxiliaryPowerUnitMount>,
    /// Installed electric space thrusters (steady power/flow commands).
    #[serde(default)]
    pub electric_thrusters: Vec<ElectricThrusterMount>,
    /// Installed continuous fusion torches.
    #[serde(default)]
    pub fusion_torches: Vec<FusionTorchMount>,
    /// Installed pulsed fusion systems with event-driven runtime state.
    #[serde(default)]
    pub pulsed_fusion_systems: Vec<PulsedFusionMount>,
    /// Installed piston/electric propeller drives (steady source commands;
    /// dry mass aggregates at bake and thrust requires ambient conditions).
    #[serde(default)]
    pub propeller_drives: Vec<PropellerDriveMount>,
    /// Installed turbine-propeller drives with stateful core-shaft loading.
    #[serde(default)]
    pub turboprops: Vec<TurbopropMount>,
    /// Installed monopropellant and cold-gas reaction-control thrusters.
    #[serde(default)]
    pub rcs_mounts: Vec<RcsMount>,
    /// Compiled parametric landing-gear and rover-wheel assemblies.
    /// Empty retains compatibility with older vehicle assets.
    #[serde(default)]
    pub wheel_chassis: Vec<CompiledWheelChassis>,
    /// Fold-out landing supports with reusable or sacrificial shock absorbers.
    /// Their structural mass is included in the sprung vehicle properties.
    #[serde(default)]
    pub landing_legs: Vec<CompiledLandingLeg>,
    /// Installed internal attitude-control assemblies. Their per-axis torque
    /// ratings are actuator data; assembly mass is included in mass baking.
    #[serde(default)]
    pub reaction_wheels: Vec<ReactionWheelBankSpec>,
    /// Installed atmospheric drag devices. Their packed mass is included in
    /// the vehicle COM and inertia; canopy forces are evaluated at each mount.
    #[serde(default)]
    pub parachutes: Vec<ParachuteSpec>,
    /// Blunt heat-shield discs retained for Newtonian aerodynamics (drag
    /// plus incidence lift). Shield mass already bakes through the fuselage
    /// hull aggregate; mounts carry force-application geometry only and
    /// never re-add mass. Empty keeps legacy assets valid.
    #[serde(default)]
    pub heat_shields: Vec<HeatShieldMount>,
    /// Fold joints compiled from procedural surfaces (hinge placement in
    /// the compiled mechanism state). The force solver ignores them; the
    /// records and panel ownership are retained for mechanism integration,
    /// but flight stepping does not currently animate aerodynamic folds.
    /// Empty keeps every legacy asset valid.
    #[serde(default)]
    pub fold_joints: Vec<FoldJointRecord>,
    /// Pressurized cabin volumes with tracked air inventory (vent/repress
    /// runtime state; empty keeps every legacy asset valid).
    #[serde(default)]
    pub cabins: Vec<PressurizedCabin>,
    /// Validated static exit records compiled from authored cabin layouts.
    #[serde(default)]
    pub cabin_exits: Vec<CabinExit>,
    /// Per-place seat, role, suit, and fitted-mass metadata from cabin layouts.
    /// Mass is already included in `mass_properties`.
    #[serde(default)]
    pub cabin_seats: Vec<CabinSeat>,
    /// Fitted equipment metadata from cabin layouts. Its mass is already
    /// included in `mass_properties`.
    #[serde(default)]
    pub cabin_monuments: Vec<CabinMonument>,
    /// Autopilot cores aboard (capability tiers; empty keeps legacy valid).
    #[serde(default)]
    pub control_cores: Vec<ControlCore>,
    /// Pilot control stations with boarding state (empty keeps legacy valid).
    #[serde(default)]
    pub control_stations: Vec<ControlStation>,
    /// Retained part-link state for crew passage, cabin air domains, and
    /// cross-part resource reachability. None is the legacy single-body path.
    #[serde(default)]
    pub assembly: Option<VehicleAssembly>,
    /// Optional named consumer-to-assembly feed-port routes. Consumers not
    /// listed use the legacy vehicle-level reachable tank set.
    #[serde(default)]
    pub resource_feed_ports: Vec<VehicleResourceFeedPort>,
    /// One ideal shared electrical bus with parameterized sources, storage,
    /// and prioritized part loads. Empty keeps legacy vehicles unpowered.
    #[serde(default)]
    pub electrical_power: ElectricalPowerSystem,
    /// Lumped thermal-node network (conduction, radiation, radiators).
    /// Empty keeps legacy vehicles without a thermal model.
    #[serde(default)]
    pub thermal: ThermalSystem,
}

/// Mass partition for a contact-active sprung chassis and its unsprung wheel
/// bodies. The baked vehicle frame is the total center of mass;
/// `sprung_center_of_mass_body_m` gives the sprung-body COM in that frame.
#[derive(Debug, Clone, PartialEq)]
pub struct VehicleWheelMassSplit {
    pub sprung_properties: RigidBodyProperties,
    pub sprung_center_of_mass_body_m: DVec3,
    pub wheels: Vec<WheelBodyMassProperties>,
}

/// One compiled fold joint: hinge placement plus compiled angle, in
/// vehicle body metres. Panels tagged with this joint's index identify the
/// region that a future runtime fold transform will rotate about the hinge;
/// this record alone does not animate them.
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
    /// As-drawn flight (deployed) angle in radians. A deployed bake has
    /// `angle_rad == deployed_angle_rad`; a future runtime mechanism can use
    /// their difference as the transform from the baked panel geometry.
    pub deployed_angle_rad: f64,
    /// Parent joint in the fold hierarchy (index into the same vehicle
    /// joint list), retained for a future mechanism evaluator. `None` for
    /// root joints.
    #[serde(default)]
    pub parent_joint: Option<usize>,
    /// Deployment rate limit in rad/s (actuator data for future runtime use).
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
            auxiliary_power_units: Vec::new(),
            electric_thrusters: Vec::new(),
            fusion_torches: Vec::new(),
            pulsed_fusion_systems: Vec::new(),
            propeller_drives: Vec::new(),
            turboprops: Vec::new(),
            rcs_mounts: Vec::new(),
            wheel_chassis: Vec::new(),
            landing_legs: Vec::new(),
            reaction_wheels: Vec::new(),
            parachutes: Vec::new(),
            heat_shields: Vec::new(),
            fold_joints: Vec::new(),
            cabins: Vec::new(),
            cabin_exits: Vec::new(),
            cabin_seats: Vec::new(),
            cabin_monuments: Vec::new(),
            control_cores: Vec::new(),
            control_stations: Vec::new(),
            assembly: None,
            resource_feed_ports: Vec::new(),
            electrical_power: ElectricalPowerSystem::default(),
            thermal: ThermalSystem::default(),
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

    /// Attach validated part connectivity compiled from the authored
    /// assembly. Current cabin inventories are retained; the baker resolves
    /// initial open domains before final mass/inertia aggregation.
    pub fn with_assembly(mut self, assembly: VehicleAssembly) -> Result<Self, VehicleError> {
        assembly.validate().map_err(|error| {
            VehicleError::InvalidVehicle(format!("invalid part assembly: {error}"))
        })?;
        for volume in assembly.volumes.iter().filter(|volume| volume.pressurized) {
            if let Some(cabin) = self
                .cabins
                .iter_mut()
                .find(|cabin| cabin.name == volume.name)
            {
                cabin.centroid_body_m = volume.centroid_body_m;
            }
        }
        self.assembly = Some(assembly);
        self.validate()?;
        Ok(self)
    }

    /// Attach named routes from installed resource consumers to assembly
    /// feed ports. Topology state remains live on `assembly`; every planning
    /// step resolves the current reachable tanks.
    pub fn with_resource_feed_ports(
        mut self,
        resource_feed_ports: Vec<VehicleResourceFeedPort>,
    ) -> Result<Self, VehicleError> {
        self.resource_feed_ports = resource_feed_ports;
        self.validate()?;
        Ok(self)
    }

    /// Attach batteries, solar arrays, fission sources, and bus consumers.
    /// Power consumers join the vessel-wide bus implicitly; no wire graph is
    /// authored or required.
    pub fn with_electrical_power(
        mut self,
        electrical_power: ElectricalPowerSystem,
    ) -> Result<Self, VehicleError> {
        electrical_power
            .validate()
            .map_err(VehicleError::ElectricalPower)?;
        self.electrical_power = electrical_power;
        self.validate()?;
        Ok(self)
    }

    /// Create saved runtime energy state from the vehicle's authored initial
    /// battery charges, reactor inventories, and solar-array positions.
    pub fn initial_electrical_power_state(&self) -> Result<ElectricalPowerState, VehicleError> {
        self.electrical_power
            .initial_state()
            .map_err(VehicleError::ElectricalPower)
    }

    /// Advance the vessel-wide electrical network for one simulation step.
    pub fn advance_electrical_power(
        &self,
        state: &ElectricalPowerState,
        command: &ElectricalPowerCommand,
    ) -> Result<(ElectricalPowerState, ElectricalPowerTelemetry), VehicleError> {
        if !self.electrical_power.fuel_cells.is_empty() {
            return Err(VehicleError::InvalidVehicle(
                "fuel-cell power must advance with installed resource inventory".into(),
            ));
        }
        self.electrical_power
            .advance(state, command)
            .map_err(VehicleError::ElectricalPower)
    }

    /// Advance the shared bus with fuel-cell availability bounded by named
    /// installed hydrogen/LOX tanks, then commit its reactant draw through the
    /// common moving-mass resource path. Each cell honors its optional named
    /// feed-port route, while overlapping cells share the same tank allocator.
    pub fn advance_electrical_power_with_resources(
        &mut self,
        resource_state: &mut VehicleResourceState,
        power_state: &ElectricalPowerState,
        command: &ElectricalPowerCommand,
    ) -> Result<(ElectricalPowerState, ElectricalPowerTelemetry, DVec3), VehicleError> {
        self.advance_electrical_power_with_demands(resource_state, power_state, command, &[])
    }

    /// Advance the shared bus and commit all colocated physical source flows
    /// (for example an APU operating point) through the same tank inventory.
    /// Fuel-cell and auxiliary-generator demand therefore share one resource
    /// plan and one center-of-mass/inertia update.
    pub fn advance_electrical_power_with_demands(
        &mut self,
        resource_state: &mut VehicleResourceState,
        power_state: &ElectricalPowerState,
        command: &ElectricalPowerCommand,
        additional_resource_demands: &[VehicleResourceDemand],
    ) -> Result<(ElectricalPowerState, ElectricalPowerTelemetry, DVec3), VehicleError> {
        let mut bus_command = command.clone();
        for demand in additional_resource_demands {
            if demand.consumer_name.starts_with("\u{1f}fuel-cell:") {
                return Err(VehicleError::InvalidVehicle(
                    "resource consumer name uses the reserved fuel-cell namespace".into(),
                ));
            }
        }
        let external_plan =
            self.plan_resource_flows(resource_state, additional_resource_demands, command.dt_s)?;
        if external_plan
            .consumers
            .iter()
            .any(|allocation| allocation.scale < 1.0 - 1.0e-10)
        {
            return Err(VehicleError::InvalidVehicle(
                "external power-source demand exceeds its reachable resource inventory".into(),
            ));
        }
        let mut fuel_cell_inventory = resource_state.clone();
        for (inventory, draw) in fuel_cell_inventory
            .tank_propellant_kg
            .iter_mut()
            .zip(&external_plan.tank_consumption_kg)
        {
            *inventory = (*inventory - draw).max(0.0);
        }

        // Fuel cells are ordinary consumers of the same reachable inventory
        // as jets/thrusters/APUs. Re-evaluate bus dispatch after any per-cell
        // resource cap so the committed source telemetry never claims fuel
        // that the tank transaction cannot draw.
        for _ in 0..=self.electrical_power.fuel_cells.len() {
            let (next_power_state, telemetry) = self
                .electrical_power
                .advance(power_state, &bus_command)
                .map_err(VehicleError::ElectricalPower)?;
            let mut demands = Vec::with_capacity(self.electrical_power.fuel_cells.len() * 2);
            for (cell, output) in self
                .electrical_power
                .fuel_cells
                .iter()
                .zip(&telemetry.fuel_cells)
            {
                let consumer_name = format!("\u{1f}fuel-cell:{}", cell.name);
                let mut hydrogen = VehicleResourceDemand::new(
                    consumer_name.clone(),
                    StoredPropellant::LiquidHydrogen,
                    output.hydrogen_flow_kg_s,
                );
                let mut oxygen = VehicleResourceDemand::new(
                    consumer_name,
                    StoredPropellant::Lox,
                    output.oxygen_flow_kg_s,
                );
                if let Some(port) = &cell.feed_port_name {
                    hydrogen.feed_port_name = Some(port.clone());
                    oxygen.feed_port_name = Some(port.clone());
                }
                demands.extend([hydrogen, oxygen]);
            }
            let fuel_cell_plan =
                self.plan_resource_flows(&fuel_cell_inventory, &demands, command.dt_s)?;
            let mut limited_cell = false;
            for (index, cell) in self.electrical_power.fuel_cells.iter().enumerate() {
                let consumer_name = format!("\u{1f}fuel-cell:{}", cell.name);
                if let Some(allocation) = fuel_cell_plan
                    .consumers
                    .iter()
                    .find(|allocation| allocation.consumer_name == consumer_name)
                    && allocation.scale < 1.0 - 1.0e-10
                {
                    bus_command.fuel_cell_power_fraction[index] *= allocation.scale;
                    limited_cell = true;
                }
            }
            if limited_cell {
                continue;
            }
            let mut combined_plan = external_plan.clone();
            for (draw, fuel_cell_draw) in combined_plan
                .tank_consumption_kg
                .iter_mut()
                .zip(&fuel_cell_plan.tank_consumption_kg)
            {
                *draw += fuel_cell_draw;
            }
            combined_plan
                .consumers
                .extend(fuel_cell_plan.consumers.iter().cloned());
            combined_plan.total_consumption_kg += fuel_cell_plan.total_consumption_kg;
            let frame_shift = self.commit_resource_flows(resource_state, &combined_plan)?;
            return Ok((next_power_state, telemetry, frame_shift));
        }
        Err(VehicleError::InvalidVehicle(
            "fuel-cell dispatch did not converge with reachable tank inventory".into(),
        ))
    }

    /// Attach a lumped thermal-node network (nodes, links, radiators).
    pub fn with_thermal(mut self, thermal: ThermalSystem) -> Result<Self, VehicleError> {
        thermal.validate().map_err(VehicleError::Thermal)?;
        self.thermal = thermal;
        self.validate()?;
        Ok(self)
    }

    /// Create saved runtime node temperatures from authored initial values.
    pub fn initial_thermal_state(&self) -> Result<ThermalState, VehicleError> {
        self.thermal.initial_state().map_err(VehicleError::Thermal)
    }

    /// Advance the vessel-wide thermal network for one simulation step.
    pub fn advance_thermal(
        &self,
        state: &ThermalState,
        command: &ThermalCommand,
    ) -> Result<(ThermalState, ThermalTelemetry), VehicleError> {
        self.thermal
            .advance(state, command)
            .map_err(VehicleError::Thermal)
    }

    /// Own-body solar occluder for one receiver point, derived from the
    /// baked collision geometry by a CPU ray query (no GPU needed).
    /// Feed the result into solar-flux occluders alongside planet and
    /// other-vehicle discs so arrays and thermal nodes share one shadow.
    /// Empty when nothing blocks the sun, or when the inputs are degenerate
    /// (a bad direction claims no shadow rather than a wrong one).
    pub fn own_body_occluder(
        &self,
        receiver_body_m: DVec3,
        sun_direction_body: DVec3,
    ) -> Option<SolarOccluder> {
        self.collision_geometry
            .own_body_occluder(receiver_body_m, sun_direction_body)
            .map(|(direction_body, angular_radius_rad)| SolarOccluder {
                direction_body,
                angular_radius_rad,
            })
    }

    /// Turn bus allocations into commands for installed electric thrusters.
    /// A consumer with the same part name as a thruster is its implicit load;
    /// callers do not connect either part with wires.
    pub fn electric_thruster_commands_from_bus(
        &self,
        telemetry: &ElectricalPowerTelemetry,
        requested_mass_flow_kg_s: &[f64],
    ) -> Result<Vec<ElectricThrusterCommand>, VehicleError> {
        if requested_mass_flow_kg_s.len() != self.electric_thrusters.len() {
            return Err(VehicleError::ControlCount {
                expected: self.electric_thrusters.len(),
                actual: requested_mass_flow_kg_s.len(),
            });
        }
        self.electric_thrusters
            .iter()
            .zip(requested_mass_flow_kg_s)
            .map(|(thruster, mass_flow)| {
                if !mass_flow.is_finite() || *mass_flow < 0.0 {
                    return Err(VehicleError::InvalidVehicle(format!(
                        "electric thruster '{}' mass-flow request must be finite and non-negative",
                        thruster.name
                    )));
                }
                let consumer = self
                    .electrical_power
                    .consumers
                    .iter()
                    .find(|consumer| consumer.name == thruster.name);
                let available_power_w = if let Some(consumer) = consumer {
                    telemetry.supplied_power_w(&consumer.name).ok_or_else(|| {
                        VehicleError::InvalidVehicle(format!(
                            "power telemetry has no allocation for electric thruster '{}'",
                            thruster.name
                        ))
                    })?
                } else {
                    0.0
                };
                Ok(ElectricThrusterCommand {
                    available_power_w,
                    requested_mass_flow_kg_s: *mass_flow,
                })
            })
            .collect()
    }

    /// Resolve each currently connected ideal-gas domain to a common
    /// pressure, temperature, and oxygen fraction. Total air, oxygen, and
    /// sensible thermal energy are conserved; cabin inventories are then
    /// apportioned by volume. This is an instantaneous topology transition,
    /// not a finite-rate hatch-flow integrator. Existing mass properties
    /// must already include the current cabin air inventory, as baked assets
    /// do.
    pub fn equalize_assembly_air_domains(&mut self) -> Result<(), VehicleError> {
        let updated_cabins = self.equalized_assembly_cabins()?;
        self.replace_cabin_states(updated_cabins)
    }

    /// Dump one cabin's air inventory overboard while keeping vehicle mass
    /// properties and all body-frame geometry centered on the updated COM.
    /// Existing mass properties must include the cabin's current air mass.
    pub fn vent_cabin(&mut self, name: &str) -> Result<f64, VehicleError> {
        let mut updated_cabins = self.cabins.clone();
        let cabin = updated_cabins
            .iter_mut()
            .find(|cabin| cabin.name == name)
            .ok_or_else(|| VehicleError::InvalidVehicle(format!("no cabin named '{name}'")))?;
        let dumped_kg = cabin.vent();
        self.replace_cabin_states(updated_cabins)?;
        Ok(dumped_kg)
    }

    /// Repressurize one cabin from a finite air reserve, updating the vehicle
    /// COM and inertia for the added gas. Existing mass properties must
    /// include the cabin's current air mass; insufficient reserve is atomic.
    pub fn repress_cabin(
        &mut self,
        name: &str,
        available_air_kg: f64,
    ) -> Result<f64, VehicleError> {
        let mut updated_cabins = self.cabins.clone();
        let cabin = updated_cabins
            .iter_mut()
            .find(|cabin| cabin.name == name)
            .ok_or_else(|| VehicleError::InvalidVehicle(format!("no cabin named '{name}'")))?;
        let consumed_kg = cabin
            .repress(available_air_kg)
            .map_err(VehicleError::Cabin)?;
        self.replace_cabin_states(updated_cabins)?;
        Ok(consumed_kg)
    }

    fn equalized_assembly_cabins(&self) -> Result<Vec<PressurizedCabin>, VehicleError> {
        let assembly = self.assembly.as_ref().ok_or_else(|| {
            VehicleError::InvalidVehicle("vehicle has no part assembly graph".into())
        })?;
        let mut cabins = self.cabins.clone();
        assembly
            .equalize_cabin_states(&mut cabins)
            .map_err(|error| VehicleError::InvalidVehicle(error.to_string()))?;
        Ok(cabins)
    }

    /// Commit a pressure-state change as one coherent mass-property and
    /// coordinate-frame transition. Air is modeled at each cabin centroid,
    /// matching the baker's initial mass aggregation.
    fn replace_cabin_states(
        &mut self,
        mut updated_cabins: Vec<PressurizedCabin>,
    ) -> Result<(), VehicleError> {
        // Validate and compute every fallible result before mutating the
        // vehicle. This keeps the transition atomic without cloning its
        // potentially large aero/collision geometry.
        self.validate()?;
        if let Some(assembly) = &self.assembly {
            for cabin in &mut updated_cabins {
                if let Some(volume) = assembly
                    .volumes
                    .iter()
                    .find(|volume| volume.pressurized && volume.name == cabin.name)
                {
                    cabin.centroid_body_m = volume.centroid_body_m;
                }
            }
        }
        for cabin in &updated_cabins {
            cabin.validate().map_err(VehicleError::Cabin)?;
        }
        let (mass_properties, frame_shift) =
            self.mass_properties_after_cabin_change(&self.cabins, &updated_cabins)?;
        if !self.body_frame_shift_is_finite(frame_shift)
            || updated_cabins
                .iter()
                .any(|cabin| !(cabin.centroid_body_m + frame_shift).is_finite())
        {
            return Err(VehicleError::InvalidVehicle(
                "cabin transition would move body-frame coordinates out of range".into(),
            ));
        }
        self.shift_body_frame_origin(frame_shift);
        self.mass_properties = mass_properties;
        for cabin in &mut updated_cabins {
            cabin.centroid_body_m += frame_shift;
        }
        self.cabins = updated_cabins;
        Ok(())
    }

    fn mass_properties_after_cabin_change(
        &self,
        old_cabins: &[PressurizedCabin],
        new_cabins: &[PressurizedCabin],
    ) -> Result<(RigidBodyProperties, DVec3), VehicleError> {
        use std::collections::{HashMap, HashSet};

        let mut old_by_name = HashMap::with_capacity(old_cabins.len());
        for cabin in old_cabins {
            if old_by_name.insert(cabin.name.as_str(), cabin).is_some() {
                return Err(VehicleError::InvalidVehicle(format!(
                    "duplicate runtime cabin '{}'",
                    cabin.name
                )));
            }
        }
        if old_by_name.len() != new_cabins.len() {
            return Err(VehicleError::InvalidVehicle(
                "cabin pressure transition changed the cabin inventory".into(),
            ));
        }

        let mut seen = HashSet::with_capacity(new_cabins.len());
        let mut mass_delta_kg = 0.0;
        let mut first_moment_delta = DVec3::ZERO;
        let mut inertia_delta = glam::DMat3::ZERO;
        for new_cabin in new_cabins {
            if !seen.insert(new_cabin.name.as_str()) {
                return Err(VehicleError::InvalidVehicle(format!(
                    "duplicate runtime cabin '{}'",
                    new_cabin.name
                )));
            }
            let old_cabin = old_by_name.get(new_cabin.name.as_str()).ok_or_else(|| {
                VehicleError::InvalidVehicle(format!(
                    "pressure transition introduced unknown cabin '{}'",
                    new_cabin.name
                ))
            })?;
            let centroid_body_m = if let Some(assembly) = &self.assembly {
                assembly
                    .volumes
                    .iter()
                    .find(|volume| volume.pressurized && volume.name == new_cabin.name)
                    .map(|volume| volume.centroid_body_m)
                    .ok_or_else(|| {
                        VehicleError::InvalidVehicle(format!(
                            "assembly has no pressurized volume for cabin '{}'",
                            new_cabin.name
                        ))
                    })?
            } else {
                old_cabin.centroid_body_m
            };
            let delta_kg = new_cabin.air_kg - old_cabin.air_kg;
            mass_delta_kg += delta_kg;
            first_moment_delta += centroid_body_m * delta_kg;
            inertia_delta += parallel_axis(delta_kg, centroid_body_m);
        }

        if !mass_delta_kg.is_finite()
            || !first_moment_delta.is_finite()
            || !inertia_delta.is_finite()
        {
            return Err(VehicleError::InvalidVehicle(
                "cabin mass transition produced non-finite mass properties".into(),
            ));
        }
        let updated_mass_kg = self.mass_properties.mass_kg + mass_delta_kg;
        if !updated_mass_kg.is_finite() || updated_mass_kg <= 0.0 {
            return Err(VehicleError::InvalidVehicle(
                "cabin mass transition produced non-positive vehicle mass".into(),
            ));
        }
        let center_shift = first_moment_delta / updated_mass_kg;
        let updated_inertia = self.mass_properties.inertia_body_kg_m2 + inertia_delta
            - parallel_axis(updated_mass_kg, center_shift);
        let properties = RigidBodyProperties::new(updated_mass_kg, updated_inertia)
            .map_err(VehicleError::MassProperties)?;
        Ok((properties, -center_shift))
    }

    /// Translate every stored point from the old COM frame into a new one.
    pub(crate) fn shift_body_frame_origin(&mut self, shift: DVec3) {
        for panel in &mut self.aero_geometry.panels {
            panel.position_body_m += shift;
            panel.center_of_pressure_body_m += shift;
        }
        for disc in &mut self.aero_geometry.blunt_discs {
            disc.position_body_m += shift;
        }
        for control in &mut self.control_surfaces {
            if let Some(hinge) = &mut control.hinge {
                hinge.point_body_m += shift;
            }
        }
        for joint in &mut self.fold_joints {
            joint.hinge_body_m += shift;
        }
        for part in &mut self.collision_geometry.parts {
            part.local_position_m += shift;
        }
        for mount in &mut self.engines {
            shift_array(&mut mount.position_body_m, shift);
        }
        for mount in &mut self.tanks {
            shift_array(&mut mount.position_body_m, shift);
        }
        for mount in &mut self.systems {
            for chamber in &mut mount.system.chambers {
                shift_array(&mut chamber.position_body_m, shift);
            }
        }
        for mount in &mut self.jets {
            shift_array(&mut mount.position_body_m, shift);
        }
        for mount in &mut self.auxiliary_power_units {
            shift_array(&mut mount.position_body_m, shift);
        }
        for mount in &mut self.electric_thrusters {
            shift_array(&mut mount.position_body_m, shift);
        }
        for mount in &mut self.fusion_torches {
            shift_array(&mut mount.position_body_m, shift);
        }
        for mount in &mut self.pulsed_fusion_systems {
            shift_array(&mut mount.position_body_m, shift);
        }
        for mount in &mut self.propeller_drives {
            shift_array(&mut mount.position_body_m, shift);
        }
        for mount in &mut self.turboprops {
            shift_array(&mut mount.position_body_m, shift);
        }
        for battery in &mut self.electrical_power.batteries {
            battery.position_body_m += shift;
        }
        for capacitor in &mut self.electrical_power.ultracapacitors {
            capacitor.position_body_m += shift;
        }
        for array in &mut self.electrical_power.solar_arrays {
            array.position_body_m += shift;
        }
        for reactor in &mut self.electrical_power.reactors {
            reactor.position_body_m += shift;
        }
        for fuel_cell in &mut self.electrical_power.fuel_cells {
            fuel_cell.position_body_m += shift;
        }
        for node in &mut self.thermal.nodes {
            node.position_body_m += shift;
        }
        for radiator in &mut self.thermal.radiators {
            radiator.position_body_m += shift;
        }
        for shield in &mut self.heat_shields {
            shield.position_body_m += shift;
        }
        for cabin in &mut self.cabins {
            cabin.centroid_body_m += shift;
        }
        for exit in &mut self.cabin_exits {
            exit.position_body_m += shift;
        }
        for seat in &mut self.cabin_seats {
            seat.position_body_m += shift;
        }
        for monument in &mut self.cabin_monuments {
            monument.position_body_m += shift;
        }
        if let Some(assembly) = &mut self.assembly {
            for volume in &mut assembly.volumes {
                volume.centroid_body_m += shift;
                for seat in &mut volume.seat_positions_body_m {
                    *seat += shift;
                }
            }
        }
    }

    pub(crate) fn body_frame_shift_is_finite(&self, shift: DVec3) -> bool {
        let shifted_point_is_finite = |point: DVec3| (point + shift).is_finite();
        let shifted_station_is_finite =
            |station: &[f64; 3]| shifted_point_is_finite(DVec3::from_array(*station));
        self.aero_geometry.panels.iter().all(|panel| {
            shifted_point_is_finite(panel.position_body_m)
                && shifted_point_is_finite(panel.center_of_pressure_body_m)
        }) && self
            .aero_geometry
            .blunt_discs
            .iter()
            .all(|disc| shifted_point_is_finite(disc.position_body_m))
            && self.control_surfaces.iter().all(|control| {
                control
                    .hinge
                    .is_none_or(|hinge| shifted_point_is_finite(hinge.point_body_m))
            })
            && self
                .fold_joints
                .iter()
                .all(|joint| shifted_point_is_finite(joint.hinge_body_m))
            && self
                .collision_geometry
                .parts
                .iter()
                .all(|part| shifted_point_is_finite(part.local_position_m))
            && self
                .engines
                .iter()
                .all(|mount| shifted_station_is_finite(&mount.position_body_m))
            && self
                .tanks
                .iter()
                .all(|mount| shifted_station_is_finite(&mount.position_body_m))
            && self.systems.iter().all(|mount| {
                mount
                    .system
                    .chambers
                    .iter()
                    .all(|chamber| shifted_station_is_finite(&chamber.position_body_m))
            })
            && self
                .jets
                .iter()
                .all(|mount| shifted_station_is_finite(&mount.position_body_m))
            && self
                .auxiliary_power_units
                .iter()
                .all(|mount| shifted_station_is_finite(&mount.position_body_m))
            && self
                .electric_thrusters
                .iter()
                .all(|mount| shifted_station_is_finite(&mount.position_body_m))
            && self
                .fusion_torches
                .iter()
                .all(|mount| shifted_station_is_finite(&mount.position_body_m))
            && self
                .pulsed_fusion_systems
                .iter()
                .all(|mount| shifted_station_is_finite(&mount.position_body_m))
            && self
                .propeller_drives
                .iter()
                .all(|mount| shifted_station_is_finite(&mount.position_body_m))
            && self
                .turboprops
                .iter()
                .all(|mount| shifted_station_is_finite(&mount.position_body_m))
            && self
                .electrical_power
                .batteries
                .iter()
                .all(|part| shifted_point_is_finite(part.position_body_m))
            && self
                .electrical_power
                .ultracapacitors
                .iter()
                .all(|part| shifted_point_is_finite(part.position_body_m))
            && self
                .electrical_power
                .solar_arrays
                .iter()
                .all(|part| shifted_point_is_finite(part.position_body_m))
            && self
                .electrical_power
                .reactors
                .iter()
                .all(|part| shifted_point_is_finite(part.position_body_m))
            && self
                .electrical_power
                .fuel_cells
                .iter()
                .all(|part| shifted_point_is_finite(part.position_body_m))
            && self
                .thermal
                .nodes
                .iter()
                .all(|node| shifted_point_is_finite(node.position_body_m))
            && self
                .thermal
                .radiators
                .iter()
                .all(|radiator| shifted_point_is_finite(radiator.position_body_m))
            && self
                .heat_shields
                .iter()
                .all(|shield| shifted_point_is_finite(shield.position_body_m))
            && self
                .cabins
                .iter()
                .all(|cabin| shifted_point_is_finite(cabin.centroid_body_m))
            && self
                .cabin_exits
                .iter()
                .all(|exit| shifted_point_is_finite(exit.position_body_m))
            && self
                .cabin_seats
                .iter()
                .all(|seat| shifted_point_is_finite(seat.position_body_m))
            && self
                .cabin_monuments
                .iter()
                .all(|monument| shifted_point_is_finite(monument.position_body_m))
            && self.assembly.as_ref().is_none_or(|assembly| {
                assembly.volumes.iter().all(|volume| {
                    shifted_point_is_finite(volume.centroid_body_m)
                        && volume
                            .seat_positions_body_m
                            .iter()
                            .all(|seat| shifted_point_is_finite(*seat))
                })
            })
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
        let mut apu_names = std::collections::HashSet::new();
        for mount in &self.auxiliary_power_units {
            mount.validate().map_err(VehicleError::Propulsion)?;
            if !apu_names.insert(mount.name.as_str()) {
                return Err(VehicleError::InvalidVehicle(format!(
                    "duplicate APU mount name '{}'",
                    mount.name
                )));
            }
        }
        for mount in &self.electric_thrusters {
            mount.validate().map_err(VehicleError::Propulsion)?;
        }
        for mount in &self.fusion_torches {
            mount.validate().map_err(VehicleError::Propulsion)?;
        }
        for mount in &self.pulsed_fusion_systems {
            mount.validate().map_err(VehicleError::Propulsion)?;
        }
        for mount in &self.propeller_drives {
            mount.validate().map_err(VehicleError::Propulsion)?;
        }
        for mount in &self.turboprops {
            mount.validate().map_err(VehicleError::Propulsion)?;
        }
        let mut rcs_names = std::collections::HashSet::new();
        for mount in &self.rcs_mounts {
            mount.validate().map_err(VehicleError::Propulsion)?;
            if !rcs_names.insert(mount.name.as_str()) {
                return Err(VehicleError::InvalidVehicle(format!(
                    "duplicate RCS mount name '{}'",
                    mount.name
                )));
            }
        }
        self.electrical_power
            .validate()
            .map_err(VehicleError::ElectricalPower)?;
        self.thermal.validate().map_err(VehicleError::Thermal)?;
        let mut resource_consumer_names = std::collections::HashSet::new();
        for name in self
            .engines
            .iter()
            .map(|mount| mount.name.as_str())
            .chain(self.systems.iter().map(|mount| mount.name.as_str()))
            .chain(
                self.auxiliary_power_units
                    .iter()
                    .map(|mount| mount.name.as_str()),
            )
            .chain(self.jets.iter().map(|mount| mount.name.as_str()))
            .chain(
                self.electric_thrusters
                    .iter()
                    .map(|mount| mount.name.as_str()),
            )
            .chain(self.fusion_torches.iter().map(|mount| mount.name.as_str()))
            .chain(
                self.pulsed_fusion_systems
                    .iter()
                    .map(|mount| mount.name.as_str()),
            )
            .chain(
                self.propeller_drives
                    .iter()
                    .map(|mount| mount.name.as_str()),
            )
            .chain(self.turboprops.iter().map(|mount| mount.name.as_str()))
            .chain(self.rcs_mounts.iter().map(|mount| mount.name.as_str()))
            .chain(
                self.electrical_power
                    .fuel_cells
                    .iter()
                    .map(|cell| cell.name.as_str()),
            )
        {
            if !resource_consumer_names.insert(name) {
                return Err(VehicleError::InvalidVehicle(format!(
                    "resource consumer name '{name}' is used by more than one installed part"
                )));
            }
        }
        let mut routed_consumers = std::collections::HashSet::new();
        for route in &self.resource_feed_ports {
            if route.consumer_name.trim().is_empty() || route.feed_port_name.trim().is_empty() {
                return Err(VehicleError::InvalidVehicle(
                    "resource feed routes need a consumer and feed-port name".into(),
                ));
            }
            if !routed_consumers.insert(route.consumer_name.as_str()) {
                return Err(VehicleError::InvalidVehicle(format!(
                    "resource consumer '{}' has more than one feed-port route",
                    route.consumer_name
                )));
            }
        }
        for thruster in &self.electric_thrusters {
            if let Some(consumer) = self
                .electrical_power
                .consumers
                .iter()
                .find(|consumer| consumer.name == thruster.name)
                && consumer.rated_power_w < thruster.engine.maximum_power_w
            {
                return Err(VehicleError::InvalidVehicle(format!(
                    "electrical bus consumer '{}' is rated below its electric thruster",
                    thruster.name
                )));
            }
        }
        let mut reaction_wheel_names = std::collections::HashSet::new();
        for bank in &self.reaction_wheels {
            bank.validate().map_err(VehicleError::ReactionWheel)?;
            if !reaction_wheel_names.insert(bank.name.as_str()) {
                return Err(VehicleError::InvalidVehicle(format!(
                    "duplicate reaction-wheel bank name '{}'",
                    bank.name
                )));
            }
        }
        if self.parachutes.len() > crate::MAX_PARACHUTES {
            return Err(VehicleError::InvalidVehicle(format!(
                "parachute count must not exceed {}",
                crate::MAX_PARACHUTES
            )));
        }
        let mut parachute_names = std::collections::HashSet::new();
        for parachute in &self.parachutes {
            parachute.validate().map_err(VehicleError::Parachute)?;
            if !parachute_names.insert(parachute.name.as_str()) {
                return Err(VehicleError::InvalidVehicle(format!(
                    "duplicate parachute name '{}'",
                    parachute.name
                )));
            }
        }
        let mut heat_shield_names = std::collections::HashSet::new();
        for shield in &self.heat_shields {
            shield.validate().map_err(VehicleError::HeatShield)?;
            if !heat_shield_names.insert(shield.name.as_str()) {
                return Err(VehicleError::InvalidVehicle(format!(
                    "duplicate heat-shield mount name '{}'",
                    shield.name
                )));
            }
        }
        let mut chassis_names = std::collections::HashSet::new();
        for chassis in &self.wheel_chassis {
            if !chassis_names.insert(chassis.spec.name.as_str()) {
                return Err(VehicleError::InvalidVehicle(format!(
                    "duplicate wheel chassis name '{}'",
                    chassis.spec.name
                )));
            }
            let compiled = chassis
                .spec
                .clone()
                .compile()
                .map_err(VehicleError::LandingGear)?;
            if !chassis.matches_recompiled(&compiled) {
                return Err(VehicleError::InvalidVehicle(format!(
                    "wheel chassis '{}' has stale compiled data",
                    chassis.spec.name
                )));
            }
        }
        let mut landing_leg_names = std::collections::HashSet::new();
        if self.landing_legs.len() > crate::MAX_LANDING_LEGS {
            return Err(VehicleError::InvalidVehicle(format!(
                "landing-leg count must not exceed {}",
                crate::MAX_LANDING_LEGS
            )));
        }
        for leg in &self.landing_legs {
            if !landing_leg_names.insert(leg.spec.name.as_str()) {
                return Err(VehicleError::InvalidVehicle(format!(
                    "duplicate landing-leg name '{}'",
                    leg.spec.name
                )));
            }
            let compiled = leg
                .spec
                .clone()
                .compile()
                .map_err(VehicleError::LandingGear)?;
            if !leg.matches_recompiled(&compiled) {
                return Err(VehicleError::InvalidVehicle(format!(
                    "landing leg '{}' has stale compiled data",
                    leg.spec.name
                )));
            }
        }
        for cabin in &self.cabins {
            cabin.validate().map_err(VehicleError::Cabin)?;
        }
        let mut cabin_exit_names = std::collections::HashSet::new();
        for exit in &self.cabin_exits {
            exit.validate().map_err(VehicleError::Cabin)?;
            if !cabin_exit_names.insert(exit.name.as_str()) {
                return Err(VehicleError::InvalidVehicle(format!(
                    "duplicate cabin exit '{}'",
                    exit.name
                )));
            }
        }
        let mut cabin_seat_names = std::collections::HashSet::new();
        for seat in &self.cabin_seats {
            seat.validate().map_err(VehicleError::Cabin)?;
            if !cabin_seat_names.insert(seat.name.as_str()) {
                return Err(VehicleError::InvalidVehicle(format!(
                    "duplicate cabin seat '{}'",
                    seat.name
                )));
            }
        }
        let mut cabin_monument_names = std::collections::HashSet::new();
        for monument in &self.cabin_monuments {
            monument.validate().map_err(VehicleError::Cabin)?;
            if !cabin_monument_names.insert(monument.name.as_str()) {
                return Err(VehicleError::InvalidVehicle(format!(
                    "duplicate cabin monument '{}'",
                    monument.name
                )));
            }
        }
        for core in &self.control_cores {
            core.validate().map_err(VehicleError::Cabin)?;
        }
        for station in &self.control_stations {
            station.validate().map_err(VehicleError::Cabin)?;
        }
        if let Some(assembly) = &self.assembly {
            assembly.validate().map_err(|error| {
                VehicleError::InvalidVehicle(format!("invalid part assembly: {error}"))
            })?;
            for route in &self.resource_feed_ports {
                if !assembly
                    .engine_ports
                    .iter()
                    .any(|port| port.name == route.feed_port_name)
                {
                    return Err(VehicleError::InvalidVehicle(format!(
                        "resource consumer '{}' names missing assembly feed port '{}'",
                        route.consumer_name, route.feed_port_name
                    )));
                }
            }
            let mut cabin_names = std::collections::HashSet::new();
            for cabin in &self.cabins {
                if !cabin_names.insert(cabin.name.as_str()) {
                    return Err(VehicleError::InvalidVehicle(format!(
                        "duplicate runtime cabin '{}'",
                        cabin.name
                    )));
                }
                if !assembly
                    .volumes
                    .iter()
                    .any(|volume| volume.pressurized && volume.name == cabin.name)
                {
                    return Err(VehicleError::InvalidVehicle(format!(
                        "runtime cabin '{}' is absent from the assembly volume inventory",
                        cabin.name
                    )));
                }
            }
            for volume in assembly.volumes.iter().filter(|volume| volume.pressurized) {
                let Some(cabin) = self.cabins.iter().find(|cabin| cabin.name == volume.name) else {
                    return Err(VehicleError::InvalidVehicle(format!(
                        "pressurized assembly volume '{}' has no runtime cabin",
                        volume.name
                    )));
                };
                if (volume.volume_m3 - cabin.volume_m3).abs()
                    > 1.0e-9 * volume.volume_m3.max(cabin.volume_m3)
                {
                    return Err(VehicleError::InvalidVehicle(format!(
                        "assembly volume '{}' does not match its runtime cabin volume",
                        volume.name
                    )));
                }
            }
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

    /// Attach authored wheel-chassis specifications after compiling and
    /// validating their wheel stations, component laws, and mass properties.
    pub fn with_wheel_chassis(
        mut self,
        wheel_chassis: Vec<WheelChassisSpec>,
    ) -> Result<Self, VehicleError> {
        let mut compiled = Vec::with_capacity(wheel_chassis.len());
        let mut names = std::collections::HashSet::new();
        for spec in wheel_chassis {
            if !names.insert(spec.name.clone()) {
                return Err(VehicleError::InvalidVehicle(format!(
                    "duplicate wheel chassis name '{}'",
                    spec.name
                )));
            }
            compiled.push(spec.compile().map_err(VehicleError::LandingGear)?);
        }
        self.wheel_chassis = compiled;
        self.validate()?;
        Ok(self)
    }

    /// Add the compiled wheel-chassis dry masses and full inertia tensors to
    /// the vehicle's base structure properties. Component inertias are
    /// translated from each chassis center of mass with the parallel-axis
    /// theorem; wheel spin inertia remains about each wheel axle.
    pub fn bake_wheel_chassis_masses(&mut self) -> Result<(), VehicleError> {
        let mut mass_kg = self.mass_properties.mass_kg;
        let mut inertia_body_kg_m2 = self.mass_properties.inertia_body_kg_m2;
        for chassis in &self.wheel_chassis {
            let WheelChassisMassProperties {
                mass_kg: chassis_mass_kg,
                center_of_mass_body_m,
                inertia_body_kg_m2: chassis_inertia_body_kg_m2,
            } = chassis.mass_properties;
            mass_kg += chassis_mass_kg;
            inertia_body_kg_m2 += chassis_inertia_body_kg_m2
                + chassis_mass_kg
                    * (glam::DMat3::IDENTITY * center_of_mass_body_m.length_squared()
                        - outer_product(center_of_mass_body_m, center_of_mass_body_m));
        }
        self.mass_properties = RigidBodyProperties::new(mass_kg, inertia_body_kg_m2)
            .map_err(VehicleError::MassProperties)?;
        Ok(())
    }

    /// Attach authored fold-out landing legs after compiling their axes,
    /// deployment motion, shock absorber, and mass properties.
    pub fn with_landing_legs(
        mut self,
        landing_legs: Vec<LandingLegSpec>,
    ) -> Result<Self, VehicleError> {
        if landing_legs.len() > crate::MAX_LANDING_LEGS {
            return Err(VehicleError::InvalidVehicle(format!(
                "landing-leg count must not exceed {}",
                crate::MAX_LANDING_LEGS
            )));
        }
        let mut compiled = Vec::with_capacity(landing_legs.len());
        let mut names = std::collections::HashSet::new();
        for spec in landing_legs {
            if !names.insert(spec.name.clone()) {
                return Err(VehicleError::InvalidVehicle(format!(
                    "duplicate landing-leg name '{}'",
                    spec.name
                )));
            }
            compiled.push(spec.compile().map_err(VehicleError::LandingGear)?);
        }
        self.landing_legs = compiled;
        self.validate()?;
        Ok(self)
    }

    /// Add installed landing-leg masses and inertia about the authored vehicle
    /// origin. The vehicle baker performs the final common COM shift later.
    pub fn bake_landing_leg_masses(&mut self) -> Result<(), VehicleError> {
        let mut mass_kg = self.mass_properties.mass_kg;
        let mut inertia_body_kg_m2 = self.mass_properties.inertia_body_kg_m2;
        for leg in &self.landing_legs {
            let LandingLegMassProperties {
                mass_kg: leg_mass_kg,
                center_of_mass_body_m,
                inertia_body_kg_m2: leg_inertia_body_kg_m2,
            } = leg.mass_properties;
            mass_kg += leg_mass_kg;
            inertia_body_kg_m2 += leg_inertia_body_kg_m2
                + leg_mass_kg
                    * (glam::DMat3::IDENTITY * center_of_mass_body_m.length_squared()
                        - outer_product(center_of_mass_body_m, center_of_mass_body_m));
        }
        self.mass_properties = RigidBodyProperties::new(mass_kg, inertia_body_kg_m2)
            .map_err(VehicleError::MassProperties)?;
        Ok(())
    }

    /// Attach authored body-axis reaction-wheel banks. Their torque ratings
    /// are actuator data; their dry mass and local
    /// inertia are added separately by [`Self::bake_reaction_wheel_masses`].
    pub fn with_reaction_wheels(
        mut self,
        reaction_wheels: Vec<ReactionWheelBankSpec>,
    ) -> Result<Self, VehicleError> {
        let mut names = std::collections::HashSet::new();
        for bank in &reaction_wheels {
            bank.validate().map_err(VehicleError::ReactionWheel)?;
            if !names.insert(bank.name.as_str()) {
                return Err(VehicleError::InvalidVehicle(format!(
                    "duplicate reaction-wheel bank name '{}'",
                    bank.name
                )));
            }
        }
        self.reaction_wheels = reaction_wheels;
        self.validate()?;
        Ok(self)
    }

    /// Add installed reaction-wheel dry masses and inertia tensors about the
    /// authored vehicle origin. The vehicle baker applies the final common COM
    /// shift after every mounted component has been attached.
    pub fn bake_reaction_wheel_masses(&mut self) -> Result<(), VehicleError> {
        let mut mass_kg = self.mass_properties.mass_kg;
        let mut inertia_body_kg_m2 = self.mass_properties.inertia_body_kg_m2;
        for bank in &self.reaction_wheels {
            bank.validate().map_err(VehicleError::ReactionWheel)?;
            mass_kg += bank.mass_kg;
            inertia_body_kg_m2 += bank.inertia_body_kg_m2
                + bank.mass_kg
                    * (glam::DMat3::IDENTITY * bank.position_body_m.length_squared()
                        - outer_product(bank.position_body_m, bank.position_body_m));
        }
        self.mass_properties = RigidBodyProperties::new(mass_kg, inertia_body_kg_m2)
            .map_err(VehicleError::MassProperties)?;
        Ok(())
    }

    /// Attach validated, named parachute assemblies.
    pub fn with_parachutes(mut self, parachutes: Vec<ParachuteSpec>) -> Result<Self, VehicleError> {
        if parachutes.len() > crate::MAX_PARACHUTES {
            return Err(VehicleError::InvalidVehicle(format!(
                "parachute count must not exceed {}",
                crate::MAX_PARACHUTES
            )));
        }
        let mut names = std::collections::HashSet::new();
        for parachute in &parachutes {
            parachute.validate().map_err(VehicleError::Parachute)?;
            if !names.insert(parachute.name.as_str()) {
                return Err(VehicleError::InvalidVehicle(format!(
                    "duplicate parachute name '{}'",
                    parachute.name
                )));
            }
        }
        self.parachutes = parachutes;
        self.validate()?;
        Ok(self)
    }

    /// Attach blunt heat-shield discs for Newtonian aerodynamics.
    /// Shield mass already bakes through the fuselage hull aggregate;
    /// mounts carry force-application geometry only.
    pub fn with_heat_shields(
        mut self,
        heat_shields: Vec<HeatShieldMount>,
    ) -> Result<Self, VehicleError> {
        let mut names = std::collections::HashSet::new();
        for shield in &heat_shields {
            shield.validate().map_err(VehicleError::HeatShield)?;
            if !names.insert(shield.name.as_str()) {
                return Err(VehicleError::InvalidVehicle(format!(
                    "duplicate heat-shield mount name '{}'",
                    shield.name
                )));
            }
        }
        self.heat_shields = heat_shields;
        self.validate()?;
        Ok(self)
    }

    /// Add packed parachute mass and local inertia about the authored vehicle
    /// origin. The asset baker applies the final common COM shift afterward.
    pub fn bake_parachute_masses(&mut self) -> Result<(), VehicleError> {
        let mut mass_kg = self.mass_properties.mass_kg;
        let mut inertia_body_kg_m2 = self.mass_properties.inertia_body_kg_m2;
        for parachute in &self.parachutes {
            parachute.validate().map_err(VehicleError::Parachute)?;
            mass_kg += parachute.pack_mass_kg;
            inertia_body_kg_m2 += parachute.inertia_body_kg_m2
                + parachute.pack_mass_kg
                    * (DMat3::IDENTITY * parachute.position_body_m.length_squared()
                        - outer_product(parachute.position_body_m, parachute.position_body_m));
        }
        self.mass_properties = RigidBodyProperties::new(mass_kg, inertia_body_kg_m2)
            .map_err(VehicleError::MassProperties)?;
        Ok(())
    }

    /// Add battery, solar-array, and reactor mass and inertia about the
    /// authored vehicle origin. The baker applies the common COM shift later.
    pub fn bake_electrical_power_masses(&mut self) -> Result<(), VehicleError> {
        let contribution = self
            .electrical_power
            .mass_properties()
            .map_err(VehicleError::ElectricalPower)?;
        let mass_kg = self.mass_properties.mass_kg + contribution.mass_kg;
        let inertia_body_kg_m2 = self.mass_properties.inertia_body_kg_m2
            + contribution.inertia_body_kg_m2
            + parallel_axis(contribution.mass_kg, contribution.center_of_mass_body_m);
        self.mass_properties = RigidBodyProperties::new(mass_kg, inertia_body_kg_m2)
            .map_err(VehicleError::MassProperties)?;
        Ok(())
    }

    /// Add thermal-node and radiator point masses about the authored vehicle
    /// origin. The baker applies the common COM shift later.
    pub fn bake_thermal_masses(&mut self) -> Result<(), VehicleError> {
        let contribution = self
            .thermal
            .mass_properties()
            .map_err(VehicleError::Thermal)?;
        let mass_kg = self.mass_properties.mass_kg + contribution.mass_kg;
        let inertia_body_kg_m2 = self.mass_properties.inertia_body_kg_m2
            + contribution.inertia_body_kg_m2
            + parallel_axis(contribution.mass_kg, contribution.center_of_mass_body_m);
        self.mass_properties = RigidBodyProperties::new(mass_kg, inertia_body_kg_m2)
            .map_err(VehicleError::MassProperties)?;
        Ok(())
    }

    /// Split a center-of-mass-baked vehicle into the sprung chassis and
    /// per-station unsprung wheel bodies. This must be called on the final
    /// baked vehicle definition, after all component masses and the common
    /// COM shift have been applied.
    pub fn wheel_mass_split(&self) -> Result<VehicleWheelMassSplit, VehicleError> {
        let wheels: Vec<_> = self
            .wheel_chassis
            .iter()
            .enumerate()
            .flat_map(|(chassis_index, chassis)| chassis.wheel_body_mass_properties(chassis_index))
            .collect();
        let unsprung_mass_kg: f64 = wheels.iter().map(|wheel| wheel.mass_kg).sum();
        let sprung_mass_kg = self.mass_properties.mass_kg - unsprung_mass_kg;
        if !unsprung_mass_kg.is_finite() || !sprung_mass_kg.is_finite() || sprung_mass_kg <= 1.0e-9
        {
            return Err(VehicleError::InvalidVehicle(
                "wheel mass split leaves no positive sprung chassis mass".into(),
            ));
        }

        // The authored vehicle frame is at total COM. Solve the sprung COM
        // offset from the zero first moment of the complete assembly.
        let wheel_first_moment: DVec3 = wheels
            .iter()
            .map(|wheel| wheel.center_of_mass_body_m * wheel.mass_kg)
            .sum();
        let sprung_center_of_mass_body_m = -wheel_first_moment / sprung_mass_kg;
        let parallel_axis = |mass_kg: f64, position_m: DVec3| {
            mass_kg
                * (glam::DMat3::IDENTITY * position_m.length_squared()
                    - outer_product(position_m, position_m))
        };
        let wheel_inertia_about_total_com = wheels.iter().fold(glam::DMat3::ZERO, |sum, wheel| {
            sum + wheel.inertia_body_kg_m2
                + parallel_axis(wheel.mass_kg, wheel.center_of_mass_body_m)
        });
        let sprung_inertia_about_total_com =
            self.mass_properties.inertia_body_kg_m2 - wheel_inertia_about_total_com;
        let sprung_inertia_at_com = sprung_inertia_about_total_com
            - parallel_axis(sprung_mass_kg, sprung_center_of_mass_body_m);
        let sprung_properties = RigidBodyProperties::new(sprung_mass_kg, sprung_inertia_at_com)
            .map_err(VehicleError::MassProperties)?;
        if !sprung_center_of_mass_body_m.is_finite() {
            return Err(VehicleError::InvalidVehicle(
                "wheel mass split produced a non-finite sprung COM".into(),
            ));
        }
        Ok(VehicleWheelMassSplit {
            sprung_properties,
            sprung_center_of_mass_body_m,
            wheels,
        })
    }

    /// Resolve unsprung wheel positions and the current total-COM offset for
    /// folded gear. The sprung body's intrinsic mass and inertia stay at the
    /// authored structural frame; wheel body positions/inertias follow each
    /// chassis hinge.
    pub fn wheel_mass_split_at_deployment(
        &self,
        chassis_states: &[WheelChassisState],
    ) -> Result<VehicleWheelMassSplit, VehicleError> {
        if chassis_states.len() != self.wheel_chassis.len() {
            return Err(VehicleError::InvalidVehicle(
                "wheel chassis state must match every compiled chassis".into(),
            ));
        }
        let mut split = self.wheel_mass_split()?;
        for (chassis_index, (chassis, state)) in
            self.wheel_chassis.iter().zip(chassis_states).enumerate()
        {
            if !state.deployment_fraction.is_finite()
                || !(0.0..=1.0).contains(&state.deployment_fraction)
            {
                return Err(VehicleError::InvalidVehicle(
                    "wheel chassis deployment fraction must be finite and in [0, 1]".into(),
                ));
            }
            let Some(retraction) = chassis.spec.retraction else {
                continue;
            };
            let rotation = retraction.rotation_at(state.deployment_fraction);
            for wheel in split
                .wheels
                .iter_mut()
                .filter(|wheel| wheel.chassis_index == chassis_index)
            {
                let old_center = wheel.center_of_mass_body_m;
                wheel.center_of_mass_body_m = retraction.pivot_position_body_m
                    + rotation * (old_center - retraction.pivot_position_body_m);
                wheel.inertia_body_kg_m2 = DMat3::from_quat(rotation)
                    * wheel.inertia_body_kg_m2
                    * DMat3::from_quat(rotation).transpose();
                wheel.axle_axis_body = (rotation * wheel.axle_axis_body).normalize();
            }
        }
        let total_mass = self.mass_properties.mass_kg;
        let center_shift_body_m = split
            .wheels
            .iter()
            .map(|wheel| {
                let authored_center = self.wheel_chassis[wheel.chassis_index].wheel_stations
                    [usize::from(wheel.wheel_index)]
                .position_body_m;
                (wheel.center_of_mass_body_m - authored_center) * (wheel.mass_kg / total_mass)
            })
            .sum::<DVec3>();
        split.sprung_center_of_mass_body_m -= center_shift_body_m;
        for wheel in &mut split.wheels {
            wheel.center_of_mass_body_m -= center_shift_body_m;
        }
        Ok(split)
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

    /// Aggregate installed tank masses (dry + initial propellant load), adding
    /// each mount's intrinsic tensor and its parallel-axis term. Called after
    /// [`VehicleDefinition::bake_engine_masses`].
    pub fn bake_tank_masses(&mut self) -> Result<(), VehicleError> {
        let mut mass_kg = self.mass_properties.mass_kg;
        let mut inertia = self.mass_properties.inertia_body_kg_m2;
        for mount in &self.tanks {
            let tank_mass_kg = mount.tank.dry_mass_kg + mount.loaded_propellant_kg();
            let position = DVec3::from_array(mount.position_body_m);
            mass_kg += tank_mass_kg;
            inertia += mount.intrinsic_inertia_body_kg_m2
                + tank_mass_kg
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

    /// Attach compiled auxiliary turbo-generators (baker path).
    pub fn with_auxiliary_power_units(
        mut self,
        auxiliary_power_units: Vec<AuxiliaryPowerUnitMount>,
    ) -> Result<Self, VehicleError> {
        for mount in &auxiliary_power_units {
            mount.validate().map_err(VehicleError::Propulsion)?;
        }
        self.auxiliary_power_units = auxiliary_power_units;
        self.validate()?;
        Ok(self)
    }

    /// Aggregate installed APU dry mass at each generator mount station.
    pub fn bake_auxiliary_power_unit_masses(&mut self) -> Result<(), VehicleError> {
        let mut mass_kg = self.mass_properties.mass_kg;
        let mut inertia = self.mass_properties.inertia_body_kg_m2;
        for mount in &self.auxiliary_power_units {
            let position = DVec3::from_array(mount.position_body_m);
            let apu_mass_kg = mount.unit.dry_mass_kg;
            mass_kg += apu_mass_kg;
            inertia += apu_mass_kg
                * (DMat3::IDENTITY * position.length_squared() - outer_product(position, position));
        }
        self.mass_properties =
            RigidBodyProperties::new(mass_kg, inertia).map_err(VehicleError::MassProperties)?;
        Ok(())
    }

    /// Attach compiled electric spacecraft thrusters (baker path).
    pub fn with_electric_thrusters(
        mut self,
        electric_thrusters: Vec<ElectricThrusterMount>,
    ) -> Result<Self, VehicleError> {
        for mount in &electric_thrusters {
            mount.validate().map_err(VehicleError::Propulsion)?;
        }
        self.electric_thrusters = electric_thrusters;
        Ok(self)
    }

    /// Attach compiled continuous fusion torches.
    pub fn with_fusion_torches(
        mut self,
        fusion_torches: Vec<FusionTorchMount>,
    ) -> Result<Self, VehicleError> {
        for mount in &fusion_torches {
            mount.validate().map_err(VehicleError::Propulsion)?;
        }
        self.fusion_torches = fusion_torches;
        Ok(self)
    }

    /// Attach compiled pulsed fusion systems.
    pub fn with_pulsed_fusion_systems(
        mut self,
        pulsed_fusion_systems: Vec<PulsedFusionMount>,
    ) -> Result<Self, VehicleError> {
        for mount in &pulsed_fusion_systems {
            mount.validate().map_err(VehicleError::Propulsion)?;
        }
        self.pulsed_fusion_systems = pulsed_fusion_systems;
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

    /// Aggregate electric-thruster, power-processing, and radiator mass at
    /// the installed mount stations.
    pub fn bake_electric_thruster_masses(&mut self) -> Result<(), VehicleError> {
        let mut mass_kg = self.mass_properties.mass_kg;
        let mut inertia = self.mass_properties.inertia_body_kg_m2;
        for mount in &self.electric_thrusters {
            let device_mass_kg = mount.engine.dry_mass_kg;
            let position = DVec3::from_array(mount.position_body_m);
            mass_kg += device_mass_kg;
            inertia += device_mass_kg
                * (glam::DMat3::IDENTITY * position.length_squared()
                    - outer_product(position, position));
        }
        self.mass_properties =
            RigidBodyProperties::new(mass_kg, inertia).map_err(VehicleError::MassProperties)?;
        Ok(())
    }

    /// Aggregate continuous and pulsed fusion hardware, driver, coil, and
    /// radiator dry mass at their mount stations.
    pub fn bake_fusion_masses(&mut self) -> Result<(), VehicleError> {
        let mut mass_kg = self.mass_properties.mass_kg;
        let mut inertia = self.mass_properties.inertia_body_kg_m2;
        for (position_body_m, device_mass_kg) in self
            .fusion_torches
            .iter()
            .map(|mount| (mount.position_body_m, mount.engine.dry_mass_kg))
            .chain(
                self.pulsed_fusion_systems
                    .iter()
                    .map(|mount| (mount.position_body_m, mount.engine.dry_mass_kg)),
            )
        {
            let position = DVec3::from_array(position_body_m);
            mass_kg += device_mass_kg;
            inertia += device_mass_kg
                * (glam::DMat3::IDENTITY * position.length_squared()
                    - outer_product(position, position));
        }
        self.mass_properties =
            RigidBodyProperties::new(mass_kg, inertia).map_err(VehicleError::MassProperties)?;
        Ok(())
    }

    /// Attach compiled shaft-power propeller drives (baker path).
    pub fn with_propeller_drives(
        mut self,
        drives: Vec<PropellerDriveMount>,
    ) -> Result<Self, VehicleError> {
        for mount in &drives {
            mount.validate().map_err(VehicleError::Propulsion)?;
        }
        self.propeller_drives = drives;
        Ok(self)
    }

    /// Aggregate source, rotor, and gearbox dry masses at their mount
    /// stations. Call after the other propulsion-family mass aggregators.
    pub fn bake_propeller_drive_masses(&mut self) -> Result<(), VehicleError> {
        let mut mass_kg = self.mass_properties.mass_kg;
        let mut inertia = self.mass_properties.inertia_body_kg_m2;
        for mount in &self.propeller_drives {
            let drive_mass_kg = mount.drive.dry_mass_kg;
            let position = DVec3::from_array(mount.position_body_m);
            mass_kg += drive_mass_kg;
            inertia += drive_mass_kg
                * (glam::DMat3::IDENTITY * position.length_squared()
                    - outer_product(position, position));
        }
        self.mass_properties =
            RigidBodyProperties::new(mass_kg, inertia).map_err(VehicleError::MassProperties)?;
        Ok(())
    }

    /// Attach compiled turboprop mounts (baker path).
    pub fn with_turboprops(
        mut self,
        turboprops: Vec<TurbopropMount>,
    ) -> Result<Self, VehicleError> {
        for mount in &turboprops {
            mount.validate().map_err(VehicleError::Propulsion)?;
        }
        self.turboprops = turboprops;
        Ok(self)
    }

    /// Attach installed reaction-control thrusters.
    pub fn with_rcs_mounts(mut self, rcs_mounts: Vec<RcsMount>) -> Result<Self, VehicleError> {
        for mount in &rcs_mounts {
            mount.validate().map_err(VehicleError::Propulsion)?;
        }
        self.rcs_mounts = rcs_mounts;
        self.validate()?;
        Ok(self)
    }

    /// Attach pressurized cabin volumes (baker path; validates inventory).
    pub fn with_cabins(mut self, cabins: Vec<PressurizedCabin>) -> Result<Self, VehicleError> {
        for cabin in &cabins {
            cabin.validate().map_err(VehicleError::Cabin)?;
        }
        self.cabins = cabins;
        Ok(self)
    }

    /// Attach statically authored emergency-exit records from cabin layouts.
    pub fn with_cabin_exits(mut self, exits: Vec<CabinExit>) -> Result<Self, VehicleError> {
        for exit in &exits {
            exit.validate().map_err(VehicleError::Cabin)?;
        }
        self.cabin_exits = exits;
        Ok(self)
    }

    /// Attach compiled per-place cabin metadata (baker path).
    pub fn with_cabin_seats(mut self, seats: Vec<CabinSeat>) -> Result<Self, VehicleError> {
        for seat in &seats {
            seat.validate().map_err(VehicleError::Cabin)?;
        }
        self.cabin_seats = seats;
        Ok(self)
    }

    /// Attach compiled fitted-equipment metadata (baker path).
    pub fn with_cabin_monuments(
        mut self,
        monuments: Vec<CabinMonument>,
    ) -> Result<Self, VehicleError> {
        for monument in &monuments {
            monument.validate().map_err(VehicleError::Cabin)?;
        }
        self.cabin_monuments = monuments;
        Ok(self)
    }

    /// Attach autopilot cores (baker path).
    pub fn with_control_cores(mut self, cores: Vec<ControlCore>) -> Result<Self, VehicleError> {
        for core in &cores {
            core.validate().map_err(VehicleError::Cabin)?;
        }
        self.control_cores = cores;
        Ok(self)
    }

    /// Attach pilot control stations with boarding state (baker path).
    pub fn with_control_stations(
        mut self,
        stations: Vec<ControlStation>,
    ) -> Result<Self, VehicleError> {
        for station in &stations {
            station.validate().map_err(VehicleError::Cabin)?;
        }
        self.control_stations = stations;
        Ok(self)
    }

    /// Presence-based control authority: pilot at a station wins, else any
    /// core flies, else nobody does. Vehicles that declare no crew systems
    /// at all stay unrestricted (legacy migration, like empty collision
    /// geometry).
    pub fn control_authority(&self) -> ControlAuthority {
        control_authority(
            &self.control_stations,
            &self.control_cores,
            !self.control_stations.is_empty()
                || !self.control_cores.is_empty()
                || !self.cabins.is_empty(),
        )
    }

    /// Board or debark a pilot station by name (EVA clears it, boarding sets it).
    pub fn set_station_occupied(&mut self, name: &str, occupied: bool) -> Result<(), VehicleError> {
        let Some(station) = self
            .control_stations
            .iter_mut()
            .find(|station| station.name == name)
        else {
            return Err(VehicleError::InvalidVehicle(format!(
                "no control station named '{name}'"
            )));
        };
        station.occupied = occupied;
        Ok(())
    }

    /// Change a named assembly hatch without asserting that its occupants are
    /// suited. Opening into an unpressurized region is permitted only after
    /// the connected cabins have been vented.
    pub fn set_assembly_hatch_open(&mut self, name: &str, open: bool) -> Result<(), VehicleError> {
        self.set_assembly_hatch_open_with_safety(name, open, false)
    }

    /// Change a named assembly hatch while declaring that all crew exposed by
    /// opening it are suited. If a pressurized domain opens into an
    /// unpressurized region, its air is dumped and vehicle mass properties
    /// are updated as part of the same transition.
    ///
    /// This is a manifest assertion: callers must only pass `true` after
    /// checking every occupant in the exposed pressure domain.
    pub fn set_assembly_hatch_open_with_safety(
        &mut self,
        name: &str,
        open: bool,
        all_occupants_suited: bool,
    ) -> Result<(), VehicleError> {
        let current_assembly = self.assembly.as_ref().ok_or_else(|| {
            VehicleError::InvalidVehicle("vehicle has no part assembly graph".into())
        })?;
        let was_open = current_assembly
            .links
            .iter()
            .find(|link| link.name == name)
            .map(|link| link.state.open)
            .ok_or_else(|| {
                VehicleError::InvalidVehicle(format!("unknown assembly link '{name}'"))
            })?;
        let exposed_cabins = if open && !was_open {
            Self::hatch_exposed_cabin_names(current_assembly, name)?
        } else {
            None
        };

        let updated_cabins = if let Some(cabin_names) = &exposed_cabins {
            let mut updated = self.cabins.clone();
            for cabin_name in cabin_names {
                let cabin = updated
                    .iter_mut()
                    .find(|cabin| cabin.name == *cabin_name)
                    .ok_or_else(|| {
                        VehicleError::InvalidVehicle(format!(
                            "assembly hatch '{name}' exposes missing runtime cabin '{cabin_name}'"
                        ))
                    })?;
                if !cabin.hatch_may_open(all_occupants_suited) {
                    return Err(VehicleError::InvalidVehicle(format!(
                        "hatch '{name}' cannot expose cabin air to an unpressurized region without suited-occupant confirmation; vent the cabin first or assert suited occupants"
                    )));
                }
                if all_occupants_suited {
                    cabin.vent();
                }
            }
            all_occupants_suited.then_some(updated)
        } else {
            None
        };

        let mut assembly = current_assembly.clone();
        assembly
            .set_hatch_open(name, open)
            .map_err(|error| VehicleError::InvalidVehicle(error.to_string()))?;
        assembly
            .validate()
            .map_err(|error| VehicleError::InvalidVehicle(error.to_string()))?;
        let previous_assembly = self.assembly.replace(assembly);
        if open && !was_open {
            let transition = if let Some(updated_cabins) = updated_cabins {
                self.replace_cabin_states(updated_cabins)
            } else if exposed_cabins.is_none() {
                self.equalize_assembly_air_domains()
            } else {
                Ok(())
            };
            if let Err(error) = transition {
                self.assembly = previous_assembly;
                return Err(error);
            }
        }
        Ok(())
    }

    fn hatch_exposed_cabin_names(
        assembly: &VehicleAssembly,
        name: &str,
    ) -> Result<Option<Vec<String>>, VehicleError> {
        let link = assembly
            .links
            .iter()
            .find(|link| link.name == name)
            .ok_or_else(|| {
                VehicleError::InvalidVehicle(format!("unknown assembly link '{name}'"))
            })?;
        if !link.state.hatch {
            return Ok(None);
        }

        let has_pressure_volume = |body| {
            assembly
                .volumes
                .iter()
                .any(|volume| volume.body == body && volume.pressurized)
        };
        let pressure_a = has_pressure_volume(link.state.a);
        let pressure_b = has_pressure_volume(link.state.b);
        if pressure_a == pressure_b {
            return Ok(None);
        }

        let pressure_body = if pressure_a {
            link.state.a
        } else {
            link.state.b
        };
        let pressure_groups = assembly
            .air_groups()
            .map_err(|error| VehicleError::InvalidVehicle(error.to_string()))?;
        let mut exposed = Vec::new();
        for group in pressure_groups {
            if group.iter().any(|index| {
                let volume = &assembly.volumes[*index];
                volume.body == pressure_body && volume.pressurized
            }) {
                exposed.extend(
                    group
                        .into_iter()
                        .filter(|index| assembly.volumes[*index].pressurized)
                        .map(|index| assembly.volumes[index].name.clone()),
                );
            }
        }
        exposed.sort();
        exposed.dedup();
        Ok(Some(exposed))
    }

    /// Query whether the current attachment graph permits crew passage
    /// between two authored interior-region names.
    pub fn assembly_crew_can_pass(&self, from: &str, to: &str) -> Result<bool, VehicleError> {
        let assembly = self.assembly.as_ref().ok_or_else(|| {
            VehicleError::InvalidVehicle("vehicle has no part assembly graph".into())
        })?;
        assembly
            .crew_can_pass(from, to)
            .map_err(|error| VehicleError::InvalidVehicle(error.to_string()))
    }

    /// Pressure- and suit-aware crew access query for one crew member.
    /// Unsuited and hose-fed crew cannot traverse a dry or vacuum region;
    /// self-contained suits can traverse any currently connected interior.
    pub fn assembly_crew_can_pass_safely(
        &self,
        from: &str,
        to: &str,
        suit: CrewSuitMode,
    ) -> Result<bool, VehicleError> {
        let assembly = self.assembly.as_ref().ok_or_else(|| {
            VehicleError::InvalidVehicle("vehicle has no part assembly graph".into())
        })?;
        assembly
            .crew_can_pass_safely(from, to, &self.cabins, suit)
            .map_err(|error| VehicleError::InvalidVehicle(error.to_string()))
    }

    /// Query whether two pressurized regions share an open gas path.
    pub fn assembly_cabins_share_air(&self, a: &str, b: &str) -> Result<bool, VehicleError> {
        let assembly = self.assembly.as_ref().ok_or_else(|| {
            VehicleError::InvalidVehicle("vehicle has no part assembly graph".into())
        })?;
        assembly
            .cabins_share_air(a, b)
            .map_err(|error| VehicleError::InvalidVehicle(error.to_string()))
    }

    /// Current tank-to-engine-port reachability by qualified endpoint name.
    pub fn assembly_feed_paths(&self) -> Result<Vec<(String, String)>, VehicleError> {
        let assembly = self.assembly.as_ref().ok_or_else(|| {
            VehicleError::InvalidVehicle("vehicle has no part assembly graph".into())
        })?;
        assembly
            .feed_paths()
            .map_err(|error| VehicleError::InvalidVehicle(error.to_string()))
    }

    /// Refuse control intake when nobody aboard can fly the craft.
    /// Legacy assets without crew declarations stay unrestricted, so the
    /// gate is a single authority query with no special cases.
    fn check_control_authority(&self) -> Result<(), VehicleError> {
        let authority = self.control_authority();
        if authority.controllable {
            return Ok(());
        }
        Err(VehicleError::NoControlAuthority {
            reason: authority.reason,
        })
    }

    /// Aggregate gas-path, power-turbine, propeller, and reduction-gear dry
    /// mass at each mount station.
    pub fn bake_turboprop_masses(&mut self) -> Result<(), VehicleError> {
        let mut mass_kg = self.mass_properties.mass_kg;
        let mut inertia = self.mass_properties.inertia_body_kg_m2;
        for mount in &self.turboprops {
            let drive_mass_kg = mount.drive.dry_mass_kg;
            let position = DVec3::from_array(mount.position_body_m);
            mass_kg += drive_mass_kg;
            inertia += drive_mass_kg
                * (glam::DMat3::IDENTITY * position.length_squared()
                    - outer_product(position, position));
        }
        self.mass_properties =
            RigidBodyProperties::new(mass_kg, inertia).map_err(VehicleError::MassProperties)?;
        Ok(())
    }

    /// Aggregate RCS nozzle/valve dry mass at each installed station.
    pub fn bake_rcs_masses(&mut self) -> Result<(), VehicleError> {
        let mut mass_kg = self.mass_properties.mass_kg;
        let mut inertia = self.mass_properties.inertia_body_kg_m2;
        for mount in &self.rcs_mounts {
            let dry_mass_kg = mount.thruster.dry_mass_kg();
            let position = DVec3::from_array(mount.position_body_m);
            mass_kg += dry_mass_kg;
            inertia += dry_mass_kg
                * (DMat3::IDENTITY * position.length_squared() - outer_product(position, position));
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

    /// Evaluate all installed electric thrusters and return their combined
    /// body-frame wrench plus per-mount power, flow, heat, and thrust telemetry.
    pub fn electric_thrusters_wrench_body_n(
        &self,
        commands: &[ElectricThrusterCommand],
    ) -> Result<((DVec3, DVec3), Vec<ElectricThrusterPoint>), VehicleError> {
        if commands.len() != self.electric_thrusters.len() {
            return Err(VehicleError::InvalidVehicle(format!(
                "expected {} electric-thruster commands, got {}",
                self.electric_thrusters.len(),
                commands.len()
            )));
        }
        let mut force = DVec3::ZERO;
        let mut moment = DVec3::ZERO;
        let mut points = Vec::with_capacity(commands.len());
        for (mount, command) in self.electric_thrusters.iter().zip(commands) {
            let point = mount
                .operating_point(*command)
                .map_err(VehicleError::Propulsion)?;
            let thrust = DVec3::from_array(mount.thrust_axis_body) * point.thrust_n;
            force += thrust;
            moment += DVec3::from_array(mount.position_body_m).cross(thrust);
            points.push(point);
        }
        Ok(((force, moment), points))
    }

    /// Evaluate installed continuous fusion torches and return their body
    /// wrench plus per-mount reactor, flow, and energy telemetry.
    pub fn fusion_torches_wrench_body_n(
        &self,
        commands: &[FusionTorchCommand],
    ) -> Result<((DVec3, DVec3), Vec<FusionTorchOperatingPoint>), VehicleError> {
        if commands.len() != self.fusion_torches.len() {
            return Err(VehicleError::InvalidVehicle(format!(
                "expected {} fusion-torch commands, got {}",
                self.fusion_torches.len(),
                commands.len()
            )));
        }
        let mut force = DVec3::ZERO;
        let mut moment = DVec3::ZERO;
        let mut points = Vec::with_capacity(commands.len());
        for (mount, command) in self.fusion_torches.iter().zip(commands) {
            let point = mount
                .operating_point(*command)
                .map_err(VehicleError::Propulsion)?;
            let thrust = DVec3::from_array(mount.thrust_axis_body) * point.thrust_n;
            force += thrust;
            moment += DVec3::from_array(mount.position_body_m).cross(thrust);
            points.push(point);
        }
        Ok(((force, moment), points))
    }

    /// Advance installed pulsed-fusion systems for one physics step, summing
    /// average force/moment over the step and returning each next state.
    pub fn pulsed_fusion_wrench_body_n_stateful(
        &self,
        commands: &[(PulsedFusionState, PulsedFusionCommand)],
        dt_s: f64,
    ) -> Result<StatefulPulsedFusionWrench, VehicleError> {
        if commands.len() != self.pulsed_fusion_systems.len() {
            return Err(VehicleError::InvalidVehicle(format!(
                "expected {} pulsed-fusion commands, got {}",
                self.pulsed_fusion_systems.len(),
                commands.len()
            )));
        }
        let mut force = DVec3::ZERO;
        let mut moment = DVec3::ZERO;
        let mut next = Vec::with_capacity(commands.len());
        for (mount, (state, command)) in self.pulsed_fusion_systems.iter().zip(commands) {
            let (next_state, point) = mount
                .engine
                .advance(*state, *command, dt_s)
                .map_err(VehicleError::Propulsion)?;
            let thrust = DVec3::from_array(mount.thrust_axis_body) * point.average_thrust_n;
            force += thrust;
            moment += DVec3::from_array(mount.position_body_m).cross(thrust);
            next.push((next_state, point));
        }
        Ok(((force, moment), next))
    }

    /// Evaluate every installed shaft-power drive and return the combined
    /// body-frame force/moment plus per-drive telemetry. Commands carry each
    /// source's requested shaft RPM and throttle.
    pub fn propeller_drives_wrench_body_n(
        &self,
        commands: &[PropellerDriveCommand],
        condition: &FlightCondition,
    ) -> Result<((DVec3, DVec3), Vec<PropDrivePoint>), VehicleError> {
        if commands.len() != self.propeller_drives.len() {
            return Err(VehicleError::InvalidVehicle(format!(
                "expected {} propeller-drive commands, got {}",
                self.propeller_drives.len(),
                commands.len()
            )));
        }
        let mut force = DVec3::ZERO;
        let mut moment = DVec3::ZERO;
        let mut points = Vec::with_capacity(commands.len());
        for (mount, command) in self.propeller_drives.iter().zip(commands) {
            let point = mount
                .operating_point(condition, *command)
                .map_err(VehicleError::Propulsion)?;
            let thrust = DVec3::from_array(mount.thrust_axis_body) * point.propeller.thrust_n;
            force += thrust;
            moment += DVec3::from_array(mount.position_body_m).cross(thrust);
            points.push(point);
        }
        Ok(((force, moment), points))
    }

    /// Advance all installed turboprops, summing combined core-nozzle and
    /// propeller force/moment. Each returned command carries the next shaft
    /// state and should be stored for the following physics step.
    pub fn turboprops_wrench_body_n_stateful(
        &self,
        commands: &[TurbopropCommand],
        condition: &FlightCondition,
    ) -> Result<StatefulTurbopropWrench, VehicleError> {
        if commands.len() != self.turboprops.len() {
            return Err(VehicleError::InvalidVehicle(format!(
                "expected {} turboprop commands, got {}",
                self.turboprops.len(),
                commands.len()
            )));
        }
        let mut force = DVec3::ZERO;
        let mut moment = DVec3::ZERO;
        let mut next = Vec::with_capacity(commands.len());
        for (mount, command) in self.turboprops.iter().zip(commands) {
            let (shaft_state, point) = mount
                .advance(condition, command)
                .map_err(VehicleError::Propulsion)?;
            let thrust = DVec3::from_array(mount.thrust_axis_body) * point.total_thrust_n;
            force += thrust;
            moment += DVec3::from_array(mount.position_body_m).cross(thrust);
            next.push((point, command.with_state(shaft_state)));
        }
        Ok(((force, moment), next))
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
    /// mass and solver configuration remain unchanged. Refuses when the
    /// vehicle declares crew systems but has no control authority.
    pub fn apply_control_inputs(&mut self, commands: &[f64]) -> Result<(), VehicleError> {
        self.check_control_authority()?;
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
            let panel_count = self.aero_geometry.panels.len();
            for panel_index in &surface.panel_indices {
                let Some(panel) = self.aero_geometry.panels.get_mut(*panel_index) else {
                    return Err(VehicleError::InvalidVehicle(format!(
                        "control surface '{}' references aero panel {panel_index}, \
                         but the vehicle has {panel_count} aero panels",
                        surface.name
                    )));
                };
                panel.control_deflection_rad = deflection;
            }
        }
        self.aero_geometry
            .validate()
            .map_err(VehicleError::Geometry)
    }

    /// Set absolute control deflections from a stable reference geometry.
    /// Geometric hinges rotate panel sample points, force centers, and axes;
    /// legacy controls without hinge data retain the incidence response.
    /// Like [`VehicleDefinition::apply_control_inputs`], refuses without
    /// control authority on crew-declaring vehicles.
    pub fn apply_control_deflections(
        &mut self,
        reference_geometry: &AeroGeometry,
        deflections_rad: &[f64],
    ) -> Result<(), VehicleError> {
        self.check_control_authority()?;
        if deflections_rad.len() != self.control_surfaces.len() {
            return Err(VehicleError::ControlCount {
                expected: self.control_surfaces.len(),
                actual: deflections_rad.len(),
            });
        }
        if reference_geometry.panels.len() != self.aero_geometry.panels.len() {
            return Err(VehicleError::InvalidVehicle(
                "reference geometry panel count changed".into(),
            ));
        }
        for (surface, deflection) in self.control_surfaces.iter().zip(deflections_rad) {
            if !deflection.is_finite()
                || *deflection < surface.minimum_deflection_rad - 1.0e-12
                || *deflection > surface.maximum_deflection_rad + 1.0e-12
            {
                return Err(VehicleError::InvalidControlCommand {
                    surface: surface.name.clone(),
                    command: *deflection,
                });
            }
        }

        self.aero_geometry
            .panels
            .copy_from_slice(&reference_geometry.panels);
        for (surface, deflection) in self.control_surfaces.iter().zip(deflections_rad) {
            if let Some(hinge) = surface.hinge {
                if *deflection == 0.0 {
                    // Avoid running neutral panels through an identity
                    // quaternion, which can renormalize axes and introduce
                    // drift while another control surface moves.
                    for panel_index in &surface.panel_indices {
                        self.aero_geometry.panels[*panel_index].control_deflection_rad = 0.0;
                    }
                    continue;
                }
                let rotation = DQuat::from_axis_angle(hinge.axis_body, *deflection);
                for panel_index in &surface.panel_indices {
                    let panel = &mut self.aero_geometry.panels[*panel_index];
                    panel.position_body_m = hinge.point_body_m
                        + rotation * (panel.position_body_m - hinge.point_body_m);
                    panel.center_of_pressure_body_m = hinge.point_body_m
                        + rotation * (panel.center_of_pressure_body_m - hinge.point_body_m);
                    panel.chord_axis_body = (rotation * panel.chord_axis_body).normalize();
                    panel.lift_axis_body = (rotation * panel.lift_axis_body).normalize();
                    // The angle is represented by moved geometry; applying
                    // coefficient deflection as well would count it twice.
                    panel.control_deflection_rad = 0.0;
                }
            } else {
                for panel_index in &surface.panel_indices {
                    self.aero_geometry.panels[*panel_index].control_deflection_rad = *deflection;
                }
            }
        }
        self.aero_geometry
            .validate()
            .map_err(VehicleError::Geometry)
    }

    /// Calculate aerodynamic torque about each compiled hinge from detailed
    /// panel loads. Panel moment is translated from the body origin to the
    /// hinge line before projection onto its unit axis.
    pub fn control_hinge_moments(
        &self,
        aero_result: &AeroResult,
    ) -> Result<Vec<f64>, VehicleError> {
        let loads = aero_result.panel_loads.as_ref().ok_or_else(|| {
            VehicleError::InvalidVehicle(
                "detailed panel loads are required for hinge-moment evaluation".into(),
            )
        })?;
        if loads.len() != self.aero_geometry.panels.len() {
            return Err(VehicleError::InvalidVehicle(
                "aerodynamic panel-load count differs from vehicle geometry".into(),
            ));
        }
        Ok(self
            .control_surfaces
            .iter()
            .map(|surface| {
                let Some(hinge) = surface.hinge else {
                    return 0.0;
                };
                surface
                    .panel_indices
                    .iter()
                    .map(|index| {
                        let load = loads[*index];
                        hinge
                            .axis_body
                            .dot(load.moment_body_nm - hinge.point_body_m.cross(load.force_body_n))
                    })
                    .sum()
            })
            .collect())
    }

    /// Advance actuator states from normalized commands and current detailed
    /// aerodynamic hinge loads. Unconfigured actuators preserve instantaneous
    /// legacy response; configured actuators are rate/load limited.
    pub fn advance_control_actuators(
        &self,
        current_deflections_rad: &[f64],
        commands: &[f64],
        hinge_moments_nm: &[f64],
        dt_s: f64,
    ) -> Result<(Vec<f64>, bool), VehicleError> {
        let expected = self.control_surfaces.len();
        for actual in [
            current_deflections_rad.len(),
            commands.len(),
            hinge_moments_nm.len(),
        ] {
            if actual != expected {
                return Err(VehicleError::ControlCount { expected, actual });
            }
        }
        if !dt_s.is_finite()
            || dt_s < 0.0
            || current_deflections_rad
                .iter()
                .any(|value| !value.is_finite())
            || hinge_moments_nm.iter().any(|value| !value.is_finite())
        {
            return Err(VehicleError::InvalidControlSurface(
                "actuator state, timestep, and hinge loads must be finite".into(),
            ));
        }
        let mut saturated = false;
        let mut next = Vec::with_capacity(expected);
        for (((surface, current), command), hinge_moment) in self
            .control_surfaces
            .iter()
            .zip(current_deflections_rad)
            .zip(commands)
            .zip(hinge_moments_nm)
        {
            let target = surface.deflection_for_command(*command)?;
            let actual = match surface.actuator {
                Some(actuator) => actuator.advance(
                    *current,
                    target,
                    *hinge_moment,
                    dt_s,
                    surface.minimum_deflection_rad,
                    surface.maximum_deflection_rad,
                )?,
                None => target,
            };
            saturated |= (target - actual).abs() > 1.0e-12;
            next.push(actual);
        }
        Ok((next, saturated))
    }
}

#[derive(Debug, Clone, PartialEq)]
pub enum VehicleError {
    InvalidVehicle(String),
    Geometry(AeroError),
    Collision(CollisionError),
    MassProperties(FlightError),
    Propulsion(PropulsionError),
    LandingGear(LandingGearError),
    ReactionWheel(ReactionWheelError),
    Parachute(ParachuteError),
    Cabin(CabinError),
    ElectricalPower(ElectricalPowerError),
    Thermal(ThermalError),
    HeatShield(ShieldError),
    InvalidControlSurface(String),
    InvalidControlCommand { surface: String, command: f64 },
    ControlCount { expected: usize, actual: usize },
    InvalidThrottle { throttle: f64 },
    NoControlAuthority { reason: AuthorityReason },
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
            Self::LandingGear(error) => write!(formatter, "vehicle landing-gear error: {error}"),
            Self::ReactionWheel(error) => {
                write!(formatter, "vehicle reaction-wheel error: {error}")
            }
            Self::Parachute(error) => write!(formatter, "vehicle parachute error: {error}"),
            Self::Cabin(error) => {
                write!(formatter, "vehicle cabin error: {error}")
            }
            Self::ElectricalPower(error) => {
                write!(formatter, "vehicle electrical-power error: {error}")
            }
            Self::Thermal(error) => {
                write!(formatter, "vehicle thermal error: {error}")
            }
            Self::HeatShield(error) => {
                write!(formatter, "vehicle heat-shield error: {error}")
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
            Self::NoControlAuthority { reason } => {
                write!(
                    formatter,
                    "no control authority ({reason:?}): no pilot at a station and no autopilot core"
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

/// Point-mass inertia about the body-frame origin (parallel-axis term).
fn parallel_axis(mass_kg: f64, center_body_m: DVec3) -> glam::DMat3 {
    (glam::DMat3::IDENTITY * center_body_m.length_squared()
        - outer_product(center_body_m, center_body_m))
        * mass_kg
}

fn shift_array(station: &mut [f64; 3], shift: DVec3) {
    *station = (DVec3::from_array(*station) + shift).to_array();
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{
        AirCycle, AirbreathingSpec, ChamberMaterial, ElectricMotorSpec, ElectricPropellant,
        ElectricThrusterDesign, ElectricThrusterMount, ElectricThrusterSpec, EstocMode, EstocSpec,
        FusionReaction, FusionTorchCommand, FusionTorchMount, FusionTorchSpec, IntakeKind, JetFuel,
        LandingLegSpec, LandingShockAbsorberSpec, ParachuteSpec, PropellerDriveCommand,
        PropellerDriveMount, PropellerDriveSpec, PropellerSpec, PulsedFusionCommand,
        PulsedFusionMount, PulsedFusionSpec, PulsedFusionState, ReactionWheelBankSpec,
        ShaftPowerSourceSpec, ShaftSpec, TireConstruction, TurbopropCommand, TurbopropDriveSpec,
        TurbopropMount, WheelBrakeSpec, WheelChassisSpec, WheelLayout, WheelStrutSpec,
        WheelTireSpec,
    };
    use thessa_propulsion::{
        CoolingMode, EngineCycle, LiquidEngineSpec, NozzleContour, Propellant,
    };

    #[test]
    fn vehicle_wrench_sums_mount_moments() {
        // Offset engine firing alone: force along +X, moment r x F about
        // the origin; arity mismatch refuses.
        let panel = AeroPanel::new(DVec3::new(0.2, -1.4, 0.0), DVec3::X, DVec3::Z, 9.29, 3.10)
            .expect("panel");
        let geometry = AeroGeometry::new(vec![panel]).expect("geometry");
        let properties = RigidBodyProperties::new(1000.0, DMat3::IDENTITY * 5000.0).expect("mass");
        let engine = CompiledEngine::Liquid(
            LiquidEngineSpec {
                name: "Merlin-1D class".into(),
                propellant: Propellant::LoxRp1,
                cycle: EngineCycle::GasGenerator,
                chamber_pressure_pa: 9.7e6,
                throat_radius_m: 0.134,
                expansion_ratio: 16.0,
                nozzle_length_m: 1.5,
                contour: NozzleContour::Bell,
                chamber_material: ChamberMaterial::nickel_superalloy(),
                cooling: CoolingMode::Regenerative,
                mixture_ratio: None,
                characteristic_length_m: None,
                gimbal_range_rad: 0.09,
                min_throttle: None,
                restartable: true,
            }
            .compile()
            .expect("compile"),
        );
        let full = engine
            .operating_point(1.0, 0.0, 0.0)
            .expect("vacuum point")
            .thrust_n;
        let vehicle = VehicleDefinition::new("wrench probe", geometry, properties, vec![])
            .expect("vehicle")
            .with_engines(vec![
                EngineMount {
                    name: "main".into(),
                    engine: engine.clone(),
                    position_body_m: [-3.0, 0.0, 0.0],
                    thrust_axis_body: [1.0, 0.0, 0.0],
                },
                EngineMount {
                    name: "offset".into(),
                    engine,
                    position_body_m: [-3.0, 0.0, 1.0],
                    thrust_axis_body: [1.0, 0.0, 0.0],
                },
            ])
            .expect("mounts");
        let (force, moment) = vehicle
            .wrench_body_n(&[(0.0, 0.0), (1.0, 0.0)], 0.0)
            .expect("wrench");
        assert!((force - DVec3::new(full, 0.0, 0.0)).length() / full < 1e-12);
        assert!((moment - DVec3::new(0.0, full, 0.0)).length() / full < 1e-12);
        assert!(vehicle.wrench_body_n(&[(1.0, 0.0)], 0.0).is_err());
    }

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
            bulk_fuel: None,
            boost_coolant_fuel: None,
            precooler: None,
            ejector: None,
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

    #[test]
    fn reaction_wheel_bake_adds_mount_mass_and_parallel_axis_inertia() {
        let geometry = AeroGeometry::new(vec![
            AeroPanel::new(DVec3::ZERO, DVec3::X, DVec3::Z, 1.0, 1.0).expect("panel"),
        ])
        .expect("geometry");
        let initial = RigidBodyProperties::new(1_000.0, DMat3::from_diagonal(DVec3::splat(100.0)))
            .expect("base mass");
        let bank = ReactionWheelBankSpec {
            name: "axis-box".into(),
            max_torque_body_nm: DVec3::splat(100.0),
            mass_kg: 20.0,
            position_body_m: DVec3::X,
            inertia_body_kg_m2: DMat3::from_diagonal(DVec3::splat(2.0)),
        };
        let mut vehicle = VehicleDefinition::new("reaction-wheel-bake", geometry, initial, vec![])
            .unwrap()
            .with_reaction_wheels(vec![bank])
            .unwrap();
        vehicle.bake_reaction_wheel_masses().unwrap();
        assert_eq!(vehicle.mass_properties.mass_kg, 1_020.0);
        assert_eq!(
            vehicle.mass_properties.inertia_body_kg_m2,
            DMat3::from_diagonal(DVec3::new(102.0, 122.0, 122.0))
        );
    }

    #[test]
    fn parachute_pack_bake_adds_mount_mass_and_parallel_axis_inertia() {
        let geometry = AeroGeometry::new(vec![
            AeroPanel::new(DVec3::ZERO, DVec3::X, DVec3::Z, 1.0, 1.0).expect("panel"),
        ])
        .expect("geometry");
        let initial = RigidBodyProperties::new(1_000.0, DMat3::from_diagonal(DVec3::splat(100.0)))
            .expect("base mass");
        let parachute = ParachuteSpec {
            name: "main".into(),
            reference_area_m2: 25.0,
            drag_coefficient: 1.5,
            reefed_area_fraction: 0.12,
            inflation_time_s: 2.0,
            deploy_pressure_pa: 8_000.0,
            max_deploy_dynamic_pressure_pa: 1_500.0,
            max_canopy_load_n: 80_000.0,
            pack_mass_kg: 20.0,
            position_body_m: DVec3::X,
            inertia_body_kg_m2: DMat3::from_diagonal(DVec3::splat(2.0)),
        };
        let mut vehicle = VehicleDefinition::new("parachute-bake", geometry, initial, vec![])
            .unwrap()
            .with_parachutes(vec![parachute])
            .unwrap();
        vehicle.bake_parachute_masses().unwrap();
        assert_eq!(vehicle.mass_properties.mass_kg, 1_020.0);
        assert_eq!(
            vehicle.mass_properties.inertia_body_kg_m2,
            DMat3::from_diagonal(DVec3::new(102.0, 122.0, 122.0))
        );
    }

    #[test]
    fn tank_bake_adds_intrinsic_and_parallel_axis_inertia() {
        let geometry = AeroGeometry::new(vec![
            AeroPanel::new(DVec3::ZERO, DVec3::X, DVec3::Z, 1.0, 1.0).expect("panel"),
        ])
        .expect("geometry");
        let initial =
            RigidBodyProperties::new(1000.0, glam::DMat3::from_diagonal(DVec3::splat(1000.0)))
                .expect("mass properties");
        let initial_mass = initial.mass_kg;
        let initial_inertia = initial.inertia_body_kg_m2;
        let mut vehicle =
            VehicleDefinition::new("tank-inertia", geometry, initial, vec![]).expect("vehicle");
        let shape = crate::TankShape::Cylinder {
            diameter_m: 1.0,
            length_m: 2.0,
        };
        let tank = crate::TankSpec {
            shape,
            pressure_pa: 500_000.0,
            material: crate::ChamberMaterial::nickel_superalloy(),
        }
        .compile(800.0)
        .expect("tank");
        let intrinsic = shape
            .intrinsic_inertia_body_kg_m2(tank.dry_mass_kg, tank.full_propellant_kg)
            .expect("tank inertia");
        let position = DVec3::new(1.0, -2.0, 0.5);
        vehicle = vehicle
            .with_tanks(vec![TankMount {
                name: "test-tank".into(),
                tank,
                position_body_m: position.to_array(),
                intrinsic_inertia_body_kg_m2: intrinsic,
                initial_propellant_kg: Some(tank.full_propellant_kg),
                resource: crate::TankResource::Unspecified,
            }])
            .expect("mount");
        vehicle.bake_tank_masses().expect("mass bake");

        let tank_mass = tank.dry_mass_kg + tank.full_propellant_kg;
        let point = (glam::DMat3::IDENTITY * position.length_squared()
            - outer_product(position, position))
            * tank_mass;
        let expected = initial_inertia + intrinsic + point;
        let diff = vehicle.mass_properties.inertia_body_kg_m2 - expected;
        assert!(diff.x_axis.length() < 1e-9);
        assert!(diff.y_axis.length() < 1e-9);
        assert!(diff.z_axis.length() < 1e-9);
        assert!((vehicle.mass_properties.mass_kg - initial_mass - tank_mass).abs() < 1e-9);
    }

    fn high_mach_condition() -> FlightCondition {
        let temperature_k = 288.15;
        let speed_of_sound = (crate::AIR_GAMMA * 287.0 * temperature_k).sqrt();
        FlightCondition {
            mach: 4.0,
            ambient_pa: 2_000.0,
            ambient_temp_k: temperature_k,
            airspeed_mps: 4.0 * speed_of_sound,
            composition: crate::atmosphere::AtmosphereComposition::earth_air(),
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

    #[test]
    fn electric_thruster_mount_bakes_mass_and_applies_force_at_station() {
        let base = VehicleDefinition::new(
            "electric-thruster-test",
            AeroGeometry::new(vec![
                AeroPanel::new(DVec3::ZERO, DVec3::X, DVec3::Z, 1.0, 1.0).expect("panel"),
            ])
            .expect("geometry"),
            RigidBodyProperties::new(1_000.0, glam::DMat3::from_diagonal(DVec3::splat(500.0)))
                .expect("mass"),
            vec![],
        )
        .expect("vehicle");
        let engine = ElectricThrusterSpec {
            name: "xenon-ion".into(),
            propellant: ElectricPropellant::Xenon,
            design: ElectricThrusterDesign::GriddedIon {
                accelerator_voltage_v: 1_000.0,
                grid_diameter_m: 0.4,
                grid_gap_m: 0.002,
                max_beam_current_density_a_m2: 100.0,
                propellant_utilization: 0.95,
                accelerator_efficiency: 0.9,
            },
            maximum_power_w: 5_000.0,
            maximum_mass_flow_kg_s: 1.0e-5,
            power_processor_specific_power_w_kg: 2_000.0,
            structure_density_kg_m3: 2_700.0,
            structure_thickness_m: 0.003,
            radiator_area_m2: 10.0,
            radiator_temperature_k: 700.0,
            radiator_emissivity: 0.9,
            radiator_areal_density_kg_m2: 8.0,
            ionization_efficiency: 0.75,
            inlet_temperature_k: 300.0,
        }
        .compile()
        .expect("ion drive");
        let expected_mass = engine.dry_mass_kg;
        let mut vehicle = base
            .with_electric_thrusters(vec![ElectricThrusterMount {
                name: "aft-ion".into(),
                engine,
                position_body_m: [0.0, 1.0, 0.0],
                thrust_axis_body: [1.0, 0.0, 0.0],
            }])
            .expect("mount");
        vehicle
            .bake_electric_thruster_masses()
            .expect("mass aggregate");
        assert!((vehicle.mass_properties.mass_kg - (1_000.0 + expected_mass)).abs() < 1e-10);

        vehicle = vehicle
            .with_electrical_power(ElectricalPowerSystem {
                batteries: vec![],
                ultracapacitors: vec![],
                solar_arrays: vec![crate::SolarArraySpec {
                    name: "bus-test-array".into(),
                    cell_count_x: 10,
                    cell_count_y: 10,
                    cell_size_x_m: 0.1,
                    cell_size_y_m: 0.1,
                    cell_efficiency: 0.25,
                    cell_areal_density_kg_m2: 2.0,
                    support_areal_density_kg_m2: 1.0,
                    panel_u_axis_body: DVec3::X,
                    panel_v_axis_body: DVec3::Y,
                    position_body_m: DVec3::ZERO,
                    deployment: crate::SolarArrayDeployment::Fixed,
                    tracking: crate::SolarArrayTracking::Fixed,
                }],
                reactors: vec![],
                fuel_cells: vec![],
                consumers: vec![crate::PowerConsumerSpec {
                    name: "aft-ion".into(),
                    rated_power_w: 5_000.0,
                    priority: crate::PowerPriority::Propulsion,
                }],
            })
            .expect("attach shared bus");
        let state = vehicle
            .initial_electrical_power_state()
            .expect("initial electrical state");
        let mut power_step = ElectricalPowerCommand::idle_for(&vehicle.electrical_power, 1.0);
        power_step.solar_flux = vec![
            crate::SolarFluxSource::new(20_000.0, DVec3::Z, 1.0).expect("incident solar flux"),
        ];
        power_step.consumer_power_w = vec![5_000.0];
        let (_, power_telemetry) = vehicle
            .advance_electrical_power(&state, &power_step)
            .expect("allocate solar power");
        let commands = vehicle
            .electric_thruster_commands_from_bus(&power_telemetry, &[1.0e-6])
            .expect("map implicit bus allocation to thruster");
        assert_eq!(commands[0].available_power_w, 5_000.0);
        let ((force, moment), points) = vehicle
            .electric_thrusters_wrench_body_n(&commands)
            .expect("electric-thruster wrench");
        assert_eq!(points.len(), 1);
        assert!(force.x > 0.0);
        assert!(moment.z < 0.0);
        assert_eq!(moment.x, 0.0);
        assert_eq!(moment.y, 0.0);
        assert!(vehicle.electric_thrusters_wrench_body_n(&[]).is_err());
    }

    #[test]
    fn own_hull_shadow_kills_array_output_through_shared_occluders() {
        // Tall hull box at the origin, array panel 5 m to its side facing
        // up: a high sun past the hull face is blocked although incidence
        // alone would generate power.
        let base = VehicleDefinition::new(
            "shadow-test",
            AeroGeometry::new(vec![
                AeroPanel::new(DVec3::ZERO, DVec3::X, DVec3::Z, 1.0, 1.0).expect("panel"),
            ])
            .expect("geometry"),
            RigidBodyProperties::new(1_000.0, glam::DMat3::from_diagonal(DVec3::splat(500.0)))
                .expect("mass"),
            vec![],
        )
        .expect("vehicle");
        let material = crate::CollisionMaterial::default();
        let hull = crate::CollisionPart::new(
            DVec3::ZERO,
            glam::DQuat::IDENTITY,
            crate::CollisionShape::Cuboid {
                half_extents_m: DVec3::new(1.0, 1.0, 10.0),
            },
            material,
        )
        .expect("hull part");
        let panel_position = DVec3::new(5.0, 0.0, 0.0);
        let vehicle = base
            .with_collision_geometry(
                crate::CollisionGeometry::new(vec![hull]).expect("hull geometry"),
            )
            .expect("attach hull")
            .with_electrical_power(ElectricalPowerSystem {
                batteries: vec![],
                ultracapacitors: vec![],
                solar_arrays: vec![crate::SolarArraySpec {
                    name: "deck-array".into(),
                    cell_count_x: 10,
                    cell_count_y: 10,
                    cell_size_x_m: 0.1,
                    cell_size_y_m: 0.1,
                    cell_efficiency: 0.25,
                    cell_areal_density_kg_m2: 2.0,
                    support_areal_density_kg_m2: 1.0,
                    panel_u_axis_body: DVec3::X,
                    panel_v_axis_body: DVec3::Y,
                    position_body_m: panel_position,
                    deployment: crate::SolarArrayDeployment::Fixed,
                    tracking: crate::SolarArrayTracking::Fixed,
                }],
                reactors: vec![],
                fuel_cells: vec![],
                consumers: vec![crate::PowerConsumerSpec {
                    name: "avionics".into(),
                    rated_power_w: 1_000.0,
                    priority: crate::PowerPriority::FlightControl,
                }],
            })
            .expect("attach bus");
        let state = vehicle
            .initial_electrical_power_state()
            .expect("power state");
        // Sun straight up: no own shadow, full 250 W (1 m^2 * 0.25 * 1000).
        let mut step = ElectricalPowerCommand::idle_for(&vehicle.electrical_power, 1.0);
        step.solar_flux = vec![crate::SolarFluxSource::new(1_000.0, DVec3::Z, 1.0).expect("flux")];
        step.consumer_power_w = vec![1_000.0];
        let (_, report) = vehicle
            .advance_electrical_power(&state, &step)
            .expect("lit step");
        assert!((report.solar_available_power_w - 250.0).abs() < 1.0e-9);

        // Tilted high sun past the hull: incidence alone gives 216.5 W.
        let tilted = DVec3::new(-0.5, 0.0, 0.866_025_403_784_438_6);
        let mut bare = crate::SolarFluxSource::new(1_000.0, tilted, 1.0).expect("flux");
        bare.light_angular_radius_rad = 0.005;
        step.solar_flux = vec![bare];
        let (_, unshadowed) = vehicle
            .advance_electrical_power(&state, &step)
            .expect("bare step");
        assert!((unshadowed.solar_available_power_w - 216.506_350_946_109_65).abs() < 1.0e-9);

        // Same sun with the own-body occluder: fully eclipsed.
        let mut dark = crate::SolarFluxSource::new(1_000.0, tilted, 1.0).expect("flux");
        dark.light_angular_radius_rad = 0.005;
        dark.occluders = vec![
            vehicle
                .own_body_occluder(panel_position, tilted)
                .expect("hull must block"),
        ];
        step.solar_flux = vec![dark];
        let (_, shadowed) = vehicle
            .advance_electrical_power(&state, &step)
            .expect("shadow step");
        assert_eq!(shadowed.solar_available_power_w, 0.0);
        assert_eq!(shadowed.unserved_power_w, 1_000.0);
    }

    #[test]
    fn fusion_mounts_bake_mass_and_stateful_pulses_apply_body_wrench() {
        let base = test_vehicle();
        let torch = FusionTorchSpec {
            name: "mounted-dt-torch".into(),
            reaction: FusionReaction::DeuteriumTritium,
            working_fluid: ElectricPropellant::Hydrogen,
            maximum_fusion_power_w: 100.0e6,
            fusion_gain: 10.0,
            maximum_working_flow_kg_s: 1.0e-3,
            reactor_specific_power_w_kg: 10_000.0,
            plasma_coupling_efficiency: 0.9,
            magnetic_nozzle_efficiency: 0.8,
            nozzle_radius_m: 0.5,
            nozzle_length_m: 2.0,
            magnetic_field_t: 1.0,
            coil_current_density_a_m2: 4.0e7,
            structure_density_kg_m3: 2_700.0,
            structure_thickness_m: 0.01,
            radiator_area_m2: 3_000.0,
            radiator_temperature_k: 1_000.0,
            radiator_emissivity: 0.9,
            radiator_areal_density_kg_m2: 8.0,
        }
        .compile()
        .expect("continuous torch");
        let pulse = PulsedFusionSpec {
            name: "mounted-pellet-drive".into(),
            reaction: FusionReaction::DeuteriumTritium,
            working_fluid: ElectricPropellant::Hydrogen,
            fuel_mass_per_pulse_kg: 1.0e-9,
            working_fluid_mass_per_pulse_kg: 1.0e-7,
            fusion_gain: 10.0,
            plasma_coupling_efficiency: 0.9,
            magnetic_nozzle_efficiency: 0.8,
            maximum_pulse_frequency_hz: 0.1,
            pulse_duration_s: 0.01,
            maximum_charge_power_w: 100_000.0,
            energy_buffer_capacity_pulses: 2,
            energy_buffer_specific_energy_j_kg: 1.0e6,
            pulse_system_specific_power_w_kg: 1.0e6,
            chamber_radius_m: 0.1,
            chamber_length_m: 0.5,
            magnetic_field_t: 1.0,
            coil_current_density_a_m2: 4.0e7,
            structure_density_kg_m3: 2_700.0,
            structure_thickness_m: 0.01,
            radiator_area_m2: 10.0,
            radiator_temperature_k: 1_000.0,
            radiator_emissivity: 0.9,
            radiator_areal_density_kg_m2: 8.0,
        }
        .compile()
        .expect("pulsed fusion system");
        let expected_extra_mass = torch.dry_mass_kg + pulse.dry_mass_kg;
        let mut vehicle = base
            .with_fusion_torches(vec![FusionTorchMount {
                name: "torch-aft".into(),
                engine: torch,
                position_body_m: [0.0, 1.0, 0.0],
                thrust_axis_body: [1.0, 0.0, 0.0],
            }])
            .expect("torch mount")
            .with_pulsed_fusion_systems(vec![PulsedFusionMount {
                name: "pulse-aft".into(),
                engine: pulse,
                position_body_m: [0.0, 1.0, 0.0],
                thrust_axis_body: [1.0, 0.0, 0.0],
            }])
            .expect("pulse mount");
        vehicle.bake_fusion_masses().expect("fusion mass");
        assert!((vehicle.mass_properties.mass_kg - 1_000.0 - expected_extra_mass).abs() < 1e-8);

        let ((torch_force, torch_moment), torch_points) = vehicle
            .fusion_torches_wrench_body_n(&[FusionTorchCommand {
                available_driver_power_w: 20.0e6,
                requested_working_flow_kg_s: 1.0e-4,
            }])
            .expect("torch wrench");
        assert_eq!(torch_points.len(), 1);
        assert!(torch_force.x > 0.0);
        assert!(torch_moment.z < 0.0);

        let ((pulse_force, pulse_moment), next) = vehicle
            .pulsed_fusion_wrench_body_n_stateful(
                &[(
                    PulsedFusionState {
                        pulse_phase_s: pulse.pulse_interval_s - 1.0,
                        stored_driver_energy_j: pulse.driver_energy_per_pulse_j,
                        cumulative_shots: 0,
                    },
                    PulsedFusionCommand {
                        available_charge_power_w: 100_000.0,
                        armed: true,
                    },
                )],
                1.0,
            )
            .expect("pulsed fusion wrench");
        assert_eq!(next.len(), 1);
        assert_eq!(next[0].1.pulses_fired, 1);
        assert!(pulse_force.x > 0.0);
        assert!(pulse_moment.z < 0.0);
        assert!(
            vehicle
                .pulsed_fusion_wrench_body_n_stateful(&[], 1.0)
                .is_err()
        );
    }

    #[test]
    fn propeller_drive_wrench_uses_mount_station_and_bakes_dry_mass() {
        let base = VehicleDefinition::new(
            "propeller-drive-test",
            AeroGeometry::new(vec![
                AeroPanel::new(DVec3::ZERO, DVec3::X, DVec3::Z, 1.0, 1.0).expect("panel"),
            ])
            .expect("geometry"),
            RigidBodyProperties::new(1_000.0, glam::DMat3::from_diagonal(DVec3::splat(500.0)))
                .expect("mass"),
            vec![],
        )
        .expect("vehicle");
        let drive = PropellerDriveSpec {
            propeller: PropellerSpec::default(),
            source: ShaftPowerSourceSpec::Electric(ElectricMotorSpec::default()),
            reduction_ratio: 2.0,
        }
        .compile()
        .expect("drive");
        let expected_drive_mass = drive.dry_mass_kg;
        let mut vehicle = base
            .with_propeller_drives(vec![PropellerDriveMount {
                name: "nose-prop".into(),
                drive,
                position_body_m: [0.0, 1.0, 0.0],
                thrust_axis_body: [1.0, 0.0, 0.0],
            }])
            .expect("mount");
        vehicle
            .bake_propeller_drive_masses()
            .expect("mass aggregate");
        assert!((vehicle.mass_properties.mass_kg - (1_000.0 + expected_drive_mass)).abs() < 1e-10);

        let sample = crate::AtmosphereConfig::default()
            .sample(0.0)
            .expect("atmosphere");
        let condition = crate::flight_condition(&sample, 60.0).expect("flight condition");
        let command = PropellerDriveCommand {
            throttle: 1.0,
            source_rpm: 6_000.0,
        };
        let ((force, moment), points) = vehicle
            .propeller_drives_wrench_body_n(&[command], &condition)
            .expect("propeller wrench");
        assert_eq!(points.len(), 1);
        assert!(force.x > 0.0);
        assert!(moment.z.abs() > 0.0);
        assert_eq!(moment.x, 0.0);
        assert_eq!(moment.y, 0.0);
        assert!(
            vehicle
                .propeller_drives_wrench_body_n(&[], &condition)
                .is_err()
        );
    }

    #[test]
    fn turboprop_wrench_advances_shaft_state_and_bakes_the_drive_mass() {
        let base = VehicleDefinition::new(
            "turboprop-test",
            AeroGeometry::new(vec![
                AeroPanel::new(DVec3::ZERO, DVec3::X, DVec3::Z, 1.0, 1.0).expect("panel"),
            ])
            .expect("geometry"),
            RigidBodyProperties::new(1_000.0, glam::DMat3::from_diagonal(DVec3::splat(500.0)))
                .expect("mass"),
            vec![],
        )
        .expect("vehicle");
        let drive = TurbopropDriveSpec {
            air: AirbreathingSpec {
                name: "mounted-turboprop-core".into(),
                cycle: AirCycle::Turbojet,
                fuel: JetFuel::Kerosene,
                intake_area_m2: 0.8,
                intake: IntakeKind::Pitot,
                compressor_ratio: 8.0,
                bypass_ratio: 0.0,
                fan_pressure_ratio: 1.0,
                turbine_inlet_temp_k: 1_400.0,
                afterburner: false,
                reheat_temp_k: 0.0,
                turbine_material: ChamberMaterial::nickel_superalloy(),
                spool_tau_s: 4.0,
                shaft: ShaftSpec {
                    power_turbine_heat_fraction: 0.15,
                    ..ShaftSpec::default()
                },
            },
            propeller: PropellerSpec {
                diameter_m: 2.4,
                ..PropellerSpec::default()
            },
            shaft_rpm_at_full_spool: 12_000.0,
            reduction_ratio: 6.0,
            power_turbine_mass_kg: 40.0,
        }
        .compile()
        .expect("drive");
        let drive_mass = drive.dry_mass_kg;
        let mut vehicle = base
            .with_turboprops(vec![TurbopropMount {
                name: "left-prop".into(),
                drive: drive.clone(),
                position_body_m: [0.0, 1.0, 0.0],
                thrust_axis_body: [1.0, 0.0, 0.0],
            }])
            .expect("mount");
        vehicle.bake_turboprop_masses().expect("mass aggregation");
        assert!((vehicle.mass_properties.mass_kg - (1_000.0 + drive_mass)).abs() < 1e-10);

        let sample = crate::AtmosphereConfig::default()
            .sample(0.0)
            .expect("sea-level atmosphere");
        let condition = crate::flight_condition(&sample, 0.0).expect("static condition");
        let (_, balance) = drive
            .air
            .operating_point_at_spool_loaded(&condition, 1.0, 1.0, true, 0.0)
            .expect("takeoff capacity");
        let mut command = TurbopropCommand::running(&drive);
        command.dt_s = 0.01;
        command.propeller_power_w = balance.power_takeoff_capacity_w * 0.1;
        let ((force, moment), next) = vehicle
            .turboprops_wrench_body_n_stateful(&[command], &condition)
            .expect("turboprop wrench");
        assert_eq!(next.len(), 1);
        assert!(force.x > 0.0);
        assert!(moment.z.abs() > 0.0);
        assert!(next[0].0.propeller.thrust_n > 0.0);
        assert!(next[0].1.shaft_state.spool_n >= command.shaft_state.spool_n);
        assert!(
            vehicle
                .turboprops_wrench_body_n_stateful(&[], &condition)
                .is_err()
        );
    }

    #[test]
    fn wheel_chassis_compiles_into_vehicle_and_bakes_full_mass_inertia() {
        let base_inertia = glam::DMat3::from_diagonal(DVec3::splat(500.0));
        let mut vehicle = VehicleDefinition::new(
            "rover-wheel-mass",
            AeroGeometry::new(vec![
                AeroPanel::new(DVec3::ZERO, DVec3::X, DVec3::Z, 1.0, 1.0).expect("panel"),
            ])
            .expect("geometry"),
            RigidBodyProperties::new(1_000.0, base_inertia).expect("mass"),
            vec![],
        )
        .expect("vehicle");
        let spec = WheelChassisSpec {
            name: "front-rover-bogie".into(),
            mount_position_body_m: DVec3::new(0.2, 0.0, -1.0),
            mount_orientation_body: DQuat::IDENTITY,
            length_m: 1.2,
            layout: WheelLayout::AxlePairs { track_width_m: 0.6 },
            wheel_count: 2,
            structural_mass_kg: 8.0,
            structural_inertia_local_kg_m2: glam::DMat3::from_diagonal(DVec3::splat(0.1)),
            tire: WheelTireSpec {
                construction: TireConstruction::Airless {
                    structure: crate::AirlessWheelStructure::Spoked { spoke_count: 24 },
                    structure_density_kg_m3: 4_400.0,
                    minimum_temperature_k: 80.0,
                    maximum_temperature_k: 500.0,
                },
                radius_m: 0.3,
                width_m: 0.2,
                mass_kg: 1.0,
                spin_inertia_kg_m2: 0.05,
                radial_stiffness_n_m: 20_000.0,
                radial_damping_n_s_m: 500.0,
                longitudinal_slip_stiffness_n_per_mps: 2_000.0,
                lateral_slip_stiffness_n_per_mps: 1_500.0,
                maximum_deflection_m: 0.06,
                maximum_load_n: 1_500.0,
                surface_friction: 0.8,
            },
            strut: WheelStrutSpec {
                extended_length_m: 0.4,
                stroke_m: 0.12,
                spring_rate_n_m: 5_000.0,
                damping_n_s_m: 600.0,
                preload_n: 30.0,
                minimum_force_n: 0.0,
                maximum_force_n: 2_000.0,
                mass_per_wheel_kg: 0.5,
            },
            brake: WheelBrakeSpec {
                maximum_torque_nm: 100.0,
                response_time_s: 0.1,
                mass_per_wheel_kg: 0.2,
            },
            drive: Some(crate::WheelDriveSpec {
                motor: crate::ElectricMotorSpec {
                    rated_power_w: 8_000.0,
                    peak_torque_nm: 80.0,
                    maximum_rpm: 6_000.0,
                    efficiency: 0.93,
                    cooling_capacity_w: 1_000.0,
                    dry_mass_kg: 6.0,
                },
                stall_copper_loss_w: 120.0,
                rotor_inertia_kg_m2: 0.004,
                final_drive_ratio: 18.0,
                drivetrain_efficiency: 0.91,
                driven_wheel_count: 2,
            }),
            retraction: None,
        };
        vehicle = vehicle
            .with_wheel_chassis(vec![spec])
            .expect("wheel chassis compiles");
        let component = &vehicle.wheel_chassis[0].mass_properties;
        let expected_mass = 1_000.0 + component.mass_kg;
        let expected_inertia = base_inertia
            + component.inertia_body_kg_m2
            + component.mass_kg
                * (glam::DMat3::IDENTITY * component.center_of_mass_body_m.length_squared()
                    - outer_product(
                        component.center_of_mass_body_m,
                        component.center_of_mass_body_m,
                    ));
        vehicle
            .bake_wheel_chassis_masses()
            .expect("wheel mass and inertia bake");
        assert!((vehicle.mass_properties.mass_kg - expected_mass).abs() < 1.0e-10);
        for (actual, expected) in vehicle
            .mass_properties
            .inertia_body_kg_m2
            .to_cols_array()
            .into_iter()
            .zip(expected_inertia.to_cols_array())
        {
            assert!((actual - expected).abs() < 1.0e-9);
        }
        vehicle
            .validate()
            .expect("compiled wheel chassis validates");

        let split = vehicle.wheel_mass_split().expect("sprung/unsprung split");
        let parallel_axis = |mass_kg: f64, position_m: DVec3| {
            mass_kg
                * (glam::DMat3::IDENTITY * position_m.length_squared()
                    - outer_product(position_m, position_m))
        };
        let split_mass = split.sprung_properties.mass_kg
            + split.wheels.iter().map(|wheel| wheel.mass_kg).sum::<f64>();
        assert!((split_mass - vehicle.mass_properties.mass_kg).abs() < 1.0e-10);
        let split_inertia = split.wheels.iter().fold(
            split.sprung_properties.inertia_body_kg_m2
                + parallel_axis(
                    split.sprung_properties.mass_kg,
                    split.sprung_center_of_mass_body_m,
                ),
            |sum, wheel| {
                sum + wheel.inertia_body_kg_m2
                    + parallel_axis(wheel.mass_kg, wheel.center_of_mass_body_m)
            },
        );
        for (actual, expected) in split_inertia
            .to_cols_array()
            .into_iter()
            .zip(vehicle.mass_properties.inertia_body_kg_m2.to_cols_array())
        {
            assert!((actual - expected).abs() < 1.0e-9);
        }

        let mut stale_wheel_station = vehicle.clone();
        stale_wheel_station.wheel_chassis[0].wheel_stations[0]
            .position_body_m
            .x += 1.0e-6;
        assert!(matches!(
            stale_wheel_station.validate(),
            Err(VehicleError::InvalidVehicle(message))
                if message.contains("stale compiled data")
        ));

        let mut retractable = vehicle.clone();
        let mut retractable_spec = retractable.wheel_chassis[0].spec.clone();
        retractable_spec.retraction = Some(crate::WheelChassisRetractionSpec {
            pivot_position_body_m: DVec3::ZERO,
            hinge_axis_body: DVec3::Y,
            stowed_angle_rad: -std::f64::consts::FRAC_PI_2,
            deployed_angle_rad: 0.0,
            initially_deployed: false,
            deployment_rate_rad_s: 0.8,
            actuator_max_torque_nm: 1_000.0,
        });
        retractable.wheel_chassis[0] = retractable_spec
            .compile()
            .expect("retractable wheel chassis compiles");
        let stowed_split = retractable
            .wheel_mass_split_at_deployment(&[crate::WheelChassisState {
                deployment_fraction: 0.0,
                actuator_stalled: false,
            }])
            .expect("stowed gear mass split");
        let stowed_first_moment = stowed_split
            .wheels
            .iter()
            .map(|wheel| wheel.center_of_mass_body_m * wheel.mass_kg)
            .sum::<DVec3>()
            + stowed_split.sprung_center_of_mass_body_m * stowed_split.sprung_properties.mass_kg;
        assert!(stowed_first_moment.length() < 1.0e-10);
        assert!(
            (stowed_split.wheels[0].center_of_mass_body_m - split.wheels[0].center_of_mass_body_m)
                .length()
                > 1.0e-4
        );
    }

    #[test]
    fn landing_legs_are_baked_into_sprung_mass_and_survive_asset_roundtrip() {
        let geometry = AeroGeometry::new(vec![
            AeroPanel::new(DVec3::ZERO, DVec3::X, DVec3::Z, 1.0, 1.0).expect("panel"),
        ])
        .expect("geometry");
        let initial_mass =
            RigidBodyProperties::new(1_000.0, glam::DMat3::from_diagonal(DVec3::splat(1_000.0)))
                .expect("mass properties");
        let spec = LandingLegSpec {
            name: "apollo-style-leg".into(),
            mount_position_body_m: DVec3::new(1.0, 0.0, 0.0),
            hinge_axis_body: DVec3::Y,
            stowed_leg_axis_body: DVec3::Z,
            stowed_angle_rad: 0.0,
            deployed_angle_rad: std::f64::consts::PI,
            initially_deployed: false,
            deployment_rate_rad_s: 0.5,
            actuator_max_torque_nm: 20_000.0,
            leg_length_m: 2.0,
            leg_mass_kg: 18.0,
            footpad_radius_m: 0.2,
            footpad_mass_kg: 3.0,
            footpad_friction: 0.8,
            footpad_slip_stiffness_n_per_mps: 5_000.0,
            shock_absorber: LandingShockAbsorberSpec::Reusable {
                stroke_m: 0.2,
                spring_rate_n_m: 40_000.0,
                damping_n_s_m: 1_000.0,
                preload_n: 0.0,
                bottom_out_stiffness_n_m: 250_000.0,
                maximum_force_n: 80_000.0,
            },
        };
        let mut vehicle =
            VehicleDefinition::new("landing-leg-mass", geometry, initial_mass, vec![])
                .expect("vehicle")
                .with_landing_legs(vec![spec])
                .expect("compiled landing leg");
        vehicle.bake_landing_leg_masses().expect("leg mass bake");
        assert_eq!(vehicle.mass_properties.mass_kg, 1_021.0);
        assert!(vehicle.mass_properties.inertia_body_kg_m2.is_finite());
        let split = vehicle.wheel_mass_split().expect("sprung mass split");
        assert_eq!(
            split.sprung_properties.mass_kg,
            vehicle.mass_properties.mass_kg
        );
        assert!(split.wheels.is_empty());

        let json = serde_json::to_string(&vehicle).expect("vehicle serializes");
        let round_trip: VehicleDefinition = serde_json::from_str(&json).expect("vehicle parses");
        round_trip.validate().expect("round-tripped leg recompiles");
        assert_eq!(round_trip.landing_legs.len(), 1);
        assert_eq!(round_trip.landing_legs[0].spec.name, "apollo-style-leg");
        assert_eq!(
            round_trip.landing_legs[0]
                .spec
                .initial_state()
                .deployment_fraction,
            0.0
        );
    }
}

#[cfg(test)]
mod cabin_authority_tests {
    use super::*;
    use crate::{
        AssemblyVolume, AutopilotTier, CabinExit, CabinExitSide, CabinExitType, ControlCore,
        ControlStation, CrewSuitMode, NamedAssemblyLink, PressurizedCabin, R_DRY_AIR_J_KG_K,
    };

    fn bare_vehicle() -> VehicleDefinition {
        VehicleDefinition::new(
            "authority-test",
            AeroGeometry::new(vec![
                AeroPanel::new(DVec3::ZERO, DVec3::X, DVec3::Z, 1.0, 1.0).expect("panel"),
            ])
            .expect("geometry"),
            RigidBodyProperties::new(1_000.0, glam::DMat3::from_diagonal(DVec3::splat(500.0)))
                .expect("mass"),
            vec![],
        )
        .expect("vehicle")
    }

    fn station(name: &str, occupied: bool) -> ControlStation {
        ControlStation {
            name: name.into(),
            occupied,
        }
    }

    fn air_assembly(open: bool) -> VehicleAssembly {
        VehicleAssembly {
            root_body: 0,
            body_names: vec!["service".into(), "capsule".into()],
            links: vec![NamedAssemblyLink {
                name: "hatch".into(),
                state: crate::AssemblyLinkState {
                    a: 0,
                    b: 1,
                    hatch: true,
                    open,
                },
            }],
            volumes: vec![
                AssemblyVolume {
                    name: "service.cabin".into(),
                    body: 0,
                    pressurized: true,
                    volume_m3: 2.0,
                    centroid_body_m: DVec3::ZERO,
                    seats: 0,
                    seat_positions_body_m: vec![],
                },
                AssemblyVolume {
                    name: "capsule.cabin".into(),
                    body: 1,
                    pressurized: true,
                    volume_m3: 1.0,
                    centroid_body_m: DVec3::ZERO,
                    seats: 0,
                    seat_positions_body_m: vec![],
                },
            ],
            tanks: vec![],
            engine_ports: vec![],
        }
    }

    fn positioned_air_assembly() -> VehicleAssembly {
        let mut assembly = air_assembly(false);
        assembly.volumes[0].centroid_body_m = DVec3::new(-2.0, 0.0, 0.0);
        assembly.volumes[1].centroid_body_m = DVec3::new(4.0, 0.0, 0.0);
        assembly
    }

    fn dry_hatch_assembly(open: bool) -> VehicleAssembly {
        let mut assembly = air_assembly(open);
        assembly.volumes[1].name = "capsule.bay".into();
        assembly.volumes[1].pressurized = false;
        assembly
    }

    fn unequal_cabins() -> Vec<PressurizedCabin> {
        vec![
            PressurizedCabin::new(
                "service.cabin",
                2.0,
                200.0,
                300.0,
                0.30,
                2.0 * 200_000.0 / (R_DRY_AIR_J_KG_K * 300.0),
            )
            .expect("service cabin"),
            PressurizedCabin::new(
                "capsule.cabin",
                1.0,
                100.0,
                250.0,
                0.10,
                100_000.0 / (R_DRY_AIR_J_KG_K * 250.0),
            )
            .expect("capsule cabin"),
        ]
    }

    #[test]
    fn legacy_vehicle_without_crew_systems_stays_unrestricted() {
        let mut vehicle = bare_vehicle();
        assert!(vehicle.control_authority().controllable);
        // No stations/cores/cabins: the gate skips entirely.
        vehicle.apply_control_inputs(&[]).expect("legacy intake");
    }

    #[test]
    fn empty_station_locks_control_intake_until_boarding() {
        let mut vehicle = bare_vehicle()
            .with_control_stations(vec![station("left-seat", false)])
            .expect("stations");
        assert!(!vehicle.control_authority().controllable);
        assert!(matches!(
            vehicle.apply_control_inputs(&[]),
            Err(VehicleError::NoControlAuthority { .. })
        ));
        vehicle
            .set_station_occupied("left-seat", true)
            .expect("boarding");
        assert!(vehicle.control_authority().controllable);
        vehicle.apply_control_inputs(&[]).expect("piloted intake");
        // EVA clears the station: lock returns.
        vehicle
            .set_station_occupied("left-seat", false)
            .expect("debark");
        assert!(vehicle.apply_control_inputs(&[]).is_err());
        assert!(vehicle.set_station_occupied("right-seat", true).is_err());
    }

    #[test]
    fn core_flies_uncrewed_craft() {
        let vehicle = bare_vehicle()
            .with_control_cores(vec![ControlCore {
                name: "core".into(),
                tier: AutopilotTier::Fly,
            }])
            .expect("cores");
        assert!(vehicle.control_authority().controllable);
        // Cabins alone (passengers, no pilot, no core) still lock.
        let mut cabin = PressurizedCabin::new("cabin", 2.5, 101.0, 293.0, 0.21, 3.0).unwrap();
        cabin.vent();
        let locked = bare_vehicle().with_cabins(vec![cabin]).expect("cabins");
        assert!(!locked.control_authority().controllable);
    }

    #[test]
    fn opening_hatch_equalizes_ideal_gas_and_conserves_air_oxygen_and_energy() {
        let cabins = unequal_cabins();
        let initial_air: f64 = cabins.iter().map(|cabin| cabin.air_kg).sum();
        let initial_o2: f64 = cabins.iter().map(PressurizedCabin::o2_kg).sum();
        let initial_thermal: f64 = cabins.iter().map(|cabin| cabin.air_kg * cabin.temp_k).sum();
        let expected_common_temp_k = initial_thermal / initial_air;
        let expected_common_pressure_kpa =
            initial_air * R_DRY_AIR_J_KG_K * expected_common_temp_k / 3.0 / 1000.0;
        let initial_pressures: Vec<f64> = cabins
            .iter()
            .map(PressurizedCabin::current_pressure_kpa)
            .collect();
        assert!(initial_pressures[0] > initial_pressures[1]);

        let mut vehicle = bare_vehicle()
            .with_cabins(cabins)
            .expect("cabins")
            .with_assembly(air_assembly(false))
            .expect("sealed assembly");
        assert!((vehicle.cabins[0].current_pressure_kpa() - initial_pressures[0]).abs() < 1e-10);
        assert!((vehicle.cabins[1].current_pressure_kpa() - initial_pressures[1]).abs() < 1e-10);

        vehicle
            .set_assembly_hatch_open("hatch", true)
            .expect("open hatch and equalize");
        let first = &vehicle.cabins[0];
        let second = &vehicle.cabins[1];
        assert!((first.current_pressure_kpa() - second.current_pressure_kpa()).abs() < 1e-10);
        assert!((first.current_pressure_kpa() - expected_common_pressure_kpa).abs() < 1e-10);
        let final_air: f64 = vehicle.cabins.iter().map(|cabin| cabin.air_kg).sum();
        let final_o2: f64 = vehicle.cabins.iter().map(PressurizedCabin::o2_kg).sum();
        let final_thermal: f64 = vehicle
            .cabins
            .iter()
            .map(|cabin| cabin.air_kg * cabin.temp_k)
            .sum();
        assert!((final_air - initial_air).abs() < 1e-12);
        assert!((final_o2 - initial_o2).abs() < 1e-12);
        assert!((final_thermal - initial_thermal).abs() < 1e-10);
        assert!((first.temp_k - second.temp_k).abs() < 1e-12);
        assert!((first.o2_fraction - second.o2_fraction).abs() < 1e-12);
        assert!((first.air_kg / second.air_kg - 2.0).abs() < 1e-12);
        let second_air_before_repress = vehicle.cabins[1].air_kg;
        assert_eq!(vehicle.cabins[1].repress(0.0).unwrap(), 0.0);
        assert!((vehicle.cabins[1].air_kg - second_air_before_repress).abs() < 1e-12);
        vehicle.validate().expect("equalized vehicle remains valid");

        let pressure_before_close = vehicle.cabins[0].current_pressure_kpa();
        vehicle
            .set_assembly_hatch_open("hatch", false)
            .expect("seal hatch");
        assert!((vehicle.cabins[0].current_pressure_kpa() - pressure_before_close).abs() < 1e-12);
    }

    #[test]
    fn pressure_hatch_into_dry_space_requires_suited_crew_or_prior_venting() {
        let cabin = PressurizedCabin::new("service.cabin", 2.0, 101.325, 293.15, 0.21, 2.0)
            .expect("pressurized cabin");
        let air_kg = cabin.air_kg;
        let mut vehicle = bare_vehicle();
        vehicle.mass_properties = RigidBodyProperties::new(
            1_000.0 + air_kg,
            glam::DMat3::from_diagonal(DVec3::splat(500.0)),
        )
        .expect("mass properties including cabin air");
        vehicle = vehicle
            .with_cabins(vec![cabin])
            .expect("cabin")
            .with_assembly(dry_hatch_assembly(false))
            .expect("dry-hatch assembly");

        assert!(vehicle.set_assembly_hatch_open("hatch", true).is_err());
        assert!(!vehicle.assembly.as_ref().unwrap().links[0].state.open);
        assert_eq!(vehicle.cabins[0].air_kg, air_kg);
        assert!((vehicle.mass_properties.mass_kg - (1_000.0 + air_kg)).abs() < 1e-12);

        vehicle
            .set_assembly_hatch_open_with_safety("hatch", true, true)
            .expect("suited crew may open and vent the hatch");
        assert!(vehicle.assembly.as_ref().unwrap().links[0].state.open);
        assert_eq!(vehicle.cabins[0].state, crate::CabinPressureState::Vacuum);
        assert_eq!(vehicle.cabins[0].air_kg, 0.0);
        assert!((vehicle.mass_properties.mass_kg - 1_000.0).abs() < 1e-12);
        assert!(
            vehicle
                .assembly_crew_can_pass("service.cabin", "capsule.bay")
                .expect("open hatch connectivity")
        );
        assert!(
            !vehicle
                .assembly_crew_can_pass_safely(
                    "service.cabin",
                    "capsule.bay",
                    CrewSuitMode::Unsuited,
                )
                .expect("unsuited access query")
        );
        assert!(
            !vehicle
                .assembly_crew_can_pass_safely(
                    "service.cabin",
                    "capsule.bay",
                    CrewSuitMode::HoseFed,
                )
                .expect("hose-fed access query")
        );
        assert!(
            vehicle
                .assembly_crew_can_pass_safely(
                    "service.cabin",
                    "capsule.bay",
                    CrewSuitMode::SelfContained,
                )
                .expect("self-contained access query")
        );
    }

    #[test]
    fn vented_hatch_opens_without_suit_assertion_but_access_still_requires_protection() {
        let cabin = PressurizedCabin::new("service.cabin", 2.0, 101.325, 293.15, 0.21, 2.0)
            .expect("pressurized cabin");
        let air_kg = cabin.air_kg;
        let mut vehicle = bare_vehicle();
        vehicle.mass_properties = RigidBodyProperties::new(
            1_000.0 + air_kg,
            glam::DMat3::from_diagonal(DVec3::splat(500.0)),
        )
        .expect("mass properties including cabin air");
        vehicle = vehicle
            .with_cabins(vec![cabin])
            .expect("cabin")
            .with_assembly(dry_hatch_assembly(false))
            .expect("dry-hatch assembly");

        vehicle.vent_cabin("service.cabin").expect("vent cabin");
        vehicle
            .set_assembly_hatch_open("hatch", true)
            .expect("a vented cabin may open into dry space");
        assert!(
            !vehicle
                .assembly_crew_can_pass_safely(
                    "service.cabin",
                    "capsule.bay",
                    CrewSuitMode::Unsuited,
                )
                .expect("unsuited access query")
        );
        assert!(
            vehicle
                .assembly_crew_can_pass_safely(
                    "service.cabin",
                    "capsule.bay",
                    CrewSuitMode::SelfContained,
                )
                .expect("self-contained access query")
        );
    }

    #[test]
    fn pressurized_route_allows_unsuited_and_hose_fed_crew() {
        let vehicle = bare_vehicle()
            .with_cabins(unequal_cabins())
            .expect("cabins")
            .with_assembly(air_assembly(true))
            .expect("open pressurized assembly");
        assert!(
            vehicle
                .assembly_crew_can_pass_safely(
                    "service.cabin",
                    "capsule.cabin",
                    CrewSuitMode::Unsuited,
                )
                .expect("unsuited access query")
        );
        assert!(
            vehicle
                .assembly_crew_can_pass_safely(
                    "service.cabin",
                    "capsule.cabin",
                    CrewSuitMode::HoseFed,
                )
                .expect("hose-fed access query")
        );
    }

    #[test]
    fn cabin_redistribution_updates_com_inertia_and_body_frame_geometry() {
        let cabins = unequal_cabins();
        let assembly = positioned_air_assembly();
        let initial_air_moment: DVec3 = cabins
            .iter()
            .zip(&assembly.volumes)
            .map(|(cabin, volume)| volume.centroid_body_m * cabin.air_kg)
            .sum();
        let base_mass_kg = 1_000.0;
        let base_center = -initial_air_moment / base_mass_kg;
        let initial_mass_kg = base_mass_kg + cabins.iter().map(|cabin| cabin.air_kg).sum::<f64>();
        let mut initial_inertia = glam::DMat3::from_diagonal(DVec3::splat(10_000.0))
            + parallel_axis(base_mass_kg, base_center);
        for (cabin, volume) in cabins.iter().zip(&assembly.volumes) {
            initial_inertia += parallel_axis(cabin.air_kg, volume.centroid_body_m);
        }

        let mut vehicle = bare_vehicle();
        vehicle.mass_properties =
            RigidBodyProperties::new(initial_mass_kg, initial_inertia).expect("initial properties");
        vehicle.aero_geometry.panels[0].position_body_m = DVec3::new(3.0, 1.0, -2.0);
        vehicle.aero_geometry.panels[0].center_of_pressure_body_m = DVec3::new(3.5, 1.0, -2.0);
        let initial_panel_position = vehicle.aero_geometry.panels[0].position_body_m;
        let initial_center_of_pressure = vehicle.aero_geometry.panels[0].center_of_pressure_body_m;
        let initial_exit_position = DVec3::new(8.0, 2.0, -1.0);
        vehicle = vehicle
            .with_cabins(cabins.clone())
            .expect("cabins")
            .with_cabin_exits(vec![CabinExit {
                name: "cabin.main.exit-left".into(),
                pair_id: "cabin.main.exit-1".into(),
                position_body_m: initial_exit_position,
                side: CabinExitSide::Left,
                exit_type: CabinExitType::TypeIII,
                opening_width_m: 0.508,
                opening_height_m: 0.9144,
            }])
            .expect("cabin exits")
            .with_assembly(assembly.clone())
            .expect("assembly");

        let mut predicted_cabins = cabins;
        let mut predicted_assembly = assembly;
        predicted_assembly
            .set_hatch_open("hatch", true)
            .expect("open predicted hatch");
        predicted_assembly
            .equalize_cabin_states(&mut predicted_cabins)
            .expect("predict cabin equilibrium");
        let mut first_moment_delta = DVec3::ZERO;
        let mut inertia_delta = glam::DMat3::ZERO;
        for ((before, after), volume) in vehicle
            .cabins
            .iter()
            .zip(&predicted_cabins)
            .zip(&vehicle.assembly.as_ref().unwrap().volumes)
        {
            let delta_kg = after.air_kg - before.air_kg;
            first_moment_delta += volume.centroid_body_m * delta_kg;
            inertia_delta += parallel_axis(delta_kg, volume.centroid_body_m);
        }
        let expected_center_shift = first_moment_delta / initial_mass_kg;
        let expected_frame_shift = -expected_center_shift;
        let expected_inertia =
            initial_inertia + inertia_delta - parallel_axis(initial_mass_kg, expected_center_shift);
        assert!(expected_center_shift.length() > 1.0e-6);

        vehicle
            .set_assembly_hatch_open("hatch", true)
            .expect("open hatch");
        assert!((vehicle.mass_properties.mass_kg - initial_mass_kg).abs() < 1e-12);
        let inertia_error = vehicle.mass_properties.inertia_body_kg_m2 - expected_inertia;
        assert!(inertia_error.x_axis.length() < 1e-9);
        assert!(inertia_error.y_axis.length() < 1e-9);
        assert!(inertia_error.z_axis.length() < 1e-9);
        assert!(
            (vehicle.aero_geometry.panels[0].position_body_m
                - (initial_panel_position + expected_frame_shift))
                .length()
                < 1e-12
        );
        assert!(
            (vehicle.aero_geometry.panels[0].center_of_pressure_body_m
                - (initial_center_of_pressure + expected_frame_shift))
                .length()
                < 1e-12
        );
        assert!(
            (vehicle.cabin_exits[0].position_body_m
                - (initial_exit_position + expected_frame_shift))
                .length()
                < 1e-12
        );
        for (cabin, volume) in vehicle
            .cabins
            .iter()
            .zip(&vehicle.assembly.as_ref().unwrap().volumes)
        {
            assert!((cabin.centroid_body_m - volume.centroid_body_m).length() < 1e-12);
        }
        vehicle.validate().expect("recentered vehicle is valid");
    }

    #[test]
    fn vehicle_vent_and_repress_update_mass_properties_atomically() {
        let position = DVec3::new(2.0, -1.0, 0.5);
        let full_charge_kg = 101_325.0 / (R_DRY_AIR_J_KG_K * 293.15) * 2.5;
        let cabin = PressurizedCabin::new("cabin", 2.5, 101.325, 293.15, 0.21, full_charge_kg)
            .unwrap()
            .with_centroid_body_m(position)
            .unwrap();
        let base_mass_kg = 1_000.0;
        let initial_mass_kg = base_mass_kg + cabin.air_kg;
        let base_center = -position * cabin.air_kg / base_mass_kg;
        let initial_inertia = glam::DMat3::from_diagonal(DVec3::splat(2_000.0))
            + parallel_axis(base_mass_kg, base_center)
            + parallel_axis(cabin.air_kg, position);
        let mut vehicle = bare_vehicle().with_cabins(vec![cabin]).expect("cabin");
        vehicle.mass_properties =
            RigidBodyProperties::new(initial_mass_kg, initial_inertia).expect("mass properties");
        vehicle.aero_geometry.panels[0].position_body_m = DVec3::new(-4.0, 3.0, 1.0);
        let initial_panel_position = vehicle.aero_geometry.panels[0].position_body_m;

        let dumped_kg = vehicle.vent_cabin("cabin").expect("vent cabin");
        assert!((dumped_kg - full_charge_kg).abs() < 1e-12);
        assert!((vehicle.mass_properties.mass_kg - (initial_mass_kg - dumped_kg)).abs() < 1e-12);
        assert_eq!(vehicle.cabins[0].air_kg, 0.0);

        let vented_state = vehicle.clone();
        assert!(vehicle.repress_cabin("cabin", 0.0).is_err());
        assert_eq!(vehicle, vented_state, "insufficient air must be atomic");

        assert!((vehicle.repress_cabin("cabin", dumped_kg).unwrap() - dumped_kg).abs() < 1e-12);
        assert!((vehicle.mass_properties.mass_kg - initial_mass_kg).abs() < 1e-12);
        assert!(
            (vehicle.mass_properties.inertia_body_kg_m2.x_axis - initial_inertia.x_axis).length()
                < 1e-9
        );
        assert!(
            (vehicle.mass_properties.inertia_body_kg_m2.y_axis - initial_inertia.y_axis).length()
                < 1e-9
        );
        assert!(
            (vehicle.mass_properties.inertia_body_kg_m2.z_axis - initial_inertia.z_axis).length()
                < 1e-9
        );
        assert!(
            (vehicle.aero_geometry.panels[0].position_body_m - initial_panel_position).length()
                < 1e-12
        );
        assert!((vehicle.cabins[0].centroid_body_m - position).length() < 1e-12);
    }
}
