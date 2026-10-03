//! Backend-neutral vehicle state, rigid-body dynamics, mechanisms and mounts.

#![forbid(unsafe_code)]

use thessa_aero_core::*;
use thessa_propulsion::*;

#[cfg(test)]
pub(crate) mod atmosphere {
    pub use thessa_aero_core::AtmosphereComposition;
}

mod assembly;
mod cabin;
mod collision;
mod docking;
mod fleet;
mod flight;
mod landing_gear;
mod parachute;
mod part_command;
mod power;
mod reaction_wheel;
mod resources;
mod shield;
mod thermal;
mod vehicle;

pub use assembly::{
    AssemblyBodyMassProperties, AssemblyEndpoint, AssemblyError, AssemblyJointLoad,
    AssemblyLinkState, AssemblyOwnership, AssemblyVolume, NamedAssemblyJointStrength,
    NamedAssemblyLink, NamedAssemblyResourceEdge, ReconstructedAssemblyCluster, VehicleAssembly,
    air_groups, crew_groups, feed_reachable, feed_reachable_with_resource_edges,
};
pub use cabin::{
    AuthorityReason, AutopilotTier, CabinError, CabinExit, CabinExitSide, CabinExitType,
    CabinMonument, CabinMonumentKind, CabinPressureState, CabinSeat, CabinSeatClass, CabinSeatRole,
    CabinSeatStyle, CabinSuitType, ControlAuthority, ControlCore, ControlStation, CrewSuitMode,
    MOLAR_MASS_AIR_G_MOL, MOLAR_MASS_O2_G_MOL, PressurizedCabin, R_DRY_AIR_J_KG_K,
    control_authority,
};
pub use collision::{
    CollisionAxis, CollisionError, CollisionGeometry, CollisionMaterial, CollisionPart,
    CollisionShape,
};
pub use docking::{
    DockingError, DockingKinematics, DockingPortClass, DockingPortSpec, DockingPortState,
    DockingSession,
};
pub use fleet::{DockGraph, FleetError, VehicleId};
pub use flight::{
    FlightError, FlightForces, FlightStepInput, RigidBodyProperties, RigidBodyState,
    constant_spin_orientation, evaluate_flight_forces, evaluate_flight_forces_soa,
    evaluate_flight_forces_with_aero_result, integrate_attitude_step,
    integrate_rigid_body_duration, integrate_rigid_body_duration_sampled,
    integrate_rigid_body_step, integrate_rigid_body_step_soa,
    integrate_rigid_body_step_with_aero_result,
};
pub use landing_gear::{
    AirlessWheelStructure, BrakePoint, CompiledLandingLeg, CompiledWheelChassis,
    CompiledWheelDrive, LandingGearActuatorPoint, LandingGearError, LandingLegMassProperties,
    LandingLegSpec, LandingLegState, LandingShockAbsorberSpec, LandingShockPoint, MAX_LANDING_LEGS,
    MAX_WHEELS_PER_CHASSIS, StrutLoadPoint, TireConstruction, TireLoadPoint, TireTangentForcePoint,
    WheelBodyMassProperties, WheelBrakeSpec, WheelBrakeState, WheelChassisActuatorPoint,
    WheelChassisMassProperties, WheelChassisRetractionSpec, WheelChassisSpec, WheelChassisState,
    WheelContactLoadPoint, WheelDrivePoint, WheelDriveSpec, WheelDriveTractionPoint, WheelLayout,
    WheelStation, WheelStrutSpec, WheelTireSpec,
};
pub use parachute::{
    CanopyDeformation, MAX_PARACHUTES, ParachuteCommand, ParachuteEnvironment, ParachuteError,
    ParachuteLines, ParachuteLoad, ParachutePhase, ParachuteSpec, ParachuteState,
    default_repack_time_s,
};
pub use part_command::VehiclePartCommand;
pub use power::{
    BatteryPowerTelemetry, BatterySpec, DockedBusTie, ElectricalPowerCommand, ElectricalPowerError,
    ElectricalPowerState, ElectricalPowerSystem, ElectricalPowerTelemetry,
    FUEL_CELL_HYDROGEN_LHV_J_KG, FUEL_CELL_OXYGEN_HYDROGEN_RATIO, FuelCellPowerTelemetry,
    FuelCellSpec, PowerAllocation, PowerConsumerSpec, PowerPriority, PowerSystemMassProperties,
    ReactorPowerTelemetry, ReactorSpec, SolarArrayDeployment, SolarArrayPowerTelemetry,
    SolarArraySpec, SolarArrayTracking, SolarFluxSource, SolarOccluder,
    UltracapacitorPowerTelemetry, UltracapacitorSpec, infinite_self_discharge_time_s,
};
pub use reaction_wheel::{
    ReactionWheelAllocation, ReactionWheelBankSpec, ReactionWheelBankTelemetry, ReactionWheelError,
    ReactionWheelState, allocate_reaction_wheels, allocate_reaction_wheels_with_enabled_banks,
    allocate_reaction_wheels_with_momentum, initial_reaction_wheel_states, momentum_unload_demand,
    reaction_wheel_bank_telemetry, reaction_wheel_step_heat_w,
};
pub use resources::{
    ConsumerResourceAllocation, FeedResourceProperties, VehicleAuxiliaryPowerUnitStep,
    VehiclePropulsionAllocation, VehicleResourceDemand, VehicleResourceFeedPort,
    VehicleResourcePlan, VehicleResourceState,
};
pub use shield::{HeatShieldMount, ShieldError};
pub use thermal::{
    NamedWasteHeat, RadiatorDeployment, RadiatorSpec, RadiatorTelemetry, RadiatorTracking,
    ThermalCommand, ThermalError, ThermalFlowCondition, ThermalHeatSource, ThermalHeatSourceKind,
    ThermalLinkSpec, ThermalMassProperties, ThermalNodeSpec, ThermalNodeTelemetry, ThermalState,
    ThermalSystem, ThermalTelemetry, default_convective_k, generator_waste_heat_w,
    validate_heat_source_names,
};
pub use vehicle::{
    ControlChannels, ControlHinge, ControlKind, ControlMixing, ControlSurfaceActuator,
    ControlSurfaceDefinition, FoldJointRecord, StatefulPulsedFusionWrench, StatefulTurbopropWrench,
    VehicleDefinition, VehicleError, VehicleWheelMassSplit, X15StarterProfile,
    control_surface_commands, x15_contact_geometry,
};

#[cfg(test)]
mod tests;
