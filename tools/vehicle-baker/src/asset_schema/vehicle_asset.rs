//! Top-level vehicle asset and ordered compilation into runtime geometry, mass, and mounts.

use super::*;

#[derive(Debug, Deserialize)]
pub(crate) struct VehicleAsset {
    pub(super) name: String,
    pub(super) mass_kg: f64,
    /// Matrix is written as rows in the TOML file for readability.
    pub(super) inertia_body_kg_m2: [[f64; 3]; 3],
    /// Hand-authored solver panels. Empty for all-procedural assets;
    /// legacy assets may carry only these while panels are migrated.
    #[serde(default)]
    pub(super) panels: Vec<PanelAsset>,
    #[serde(default)]
    pub(super) control_surfaces: Vec<ControlSurfaceAsset>,
    /// Procedural wing surfaces compiled hangar-side into solver panels.
    /// The compiler never runs in flight: it bakes `AeroPanel` zones plus
    /// control definitions here, and the vehicle asset carries only the
    /// compiled output. Empty keeps legacy hand-panel assets valid.
    #[serde(default)]
    pub(crate) procedural_surfaces: Vec<ProceduralSurface>,
    /// Procedural fuselage bodies compiled hangar-side into strip panels,
    /// hull mass, feed-pipeline tanks, and contact parts. Same boundary
    /// as surfaces: the compiler never runs in flight. Empty keeps
    /// legacy assets valid.
    #[serde(default)]
    pub(crate) procedural_bodies: Vec<ProceduralBody>,
    /// Compile contact boxes from procedural surfaces into the collision
    /// geometry (one body-axis box per mechanism region). Default true:
    /// the documented hangar pipeline; set false to keep hand-authored
    /// contact geometry only.
    #[serde(default = "default_true")]
    pub(super) surface_collision: bool,
    /// Compile per-segment contact parts from procedural bodies into the
    /// collision geometry. Default true; set false to keep hand-authored
    /// contact geometry only.
    #[serde(default = "default_true")]
    pub(super) body_collision: bool,
    /// Solver-neutral contact primitives. Legacy assets may omit this while
    /// collision geometry is migrated; contact-active runtime code must not.
    #[serde(default)]
    pub(super) collision_parts: Vec<CollisionPartAsset>,
    /// Procedural engine mounts (Juno-simple-mode authoring). Empty keeps
    /// engine-less assets valid; input mass semantics are "structure
    /// without engines" once mounts are present.
    #[serde(default)]
    pub(crate) engines: Vec<EngineAsset>,
    /// Propellant tanks (dry + initial-load mass aggregates at bake).
    #[serde(default)]
    pub(super) tanks: Vec<TankAsset>,
    /// Multi-chamber propulsion systems (shared feed, per-chamber nozzles).
    #[serde(default)]
    pub(super) systems: Vec<SystemAsset>,
    /// Air-breathing jets and ESTOCs (mass aggregates at bake; thrust
    /// needs a flight condition at query time).
    #[serde(default)]
    pub(super) jets: Vec<JetAsset>,
    /// Electric spacecraft thrusters, with power processor and radiator mass.
    #[serde(default)]
    pub(super) electric_thrusters: Vec<ElectricThrusterAsset>,
    /// Continuous magnetic-nozzle fusion torches.
    #[serde(default)]
    pub(super) fusion_torches: Vec<FusionTorchAsset>,
    /// Discrete pellet/impulse fusion systems with finite energy buffers.
    #[serde(default)]
    pub(super) pulsed_fusion_systems: Vec<PulsedFusionAsset>,
    /// Piston/electric shaft sources driving reusable ideal propeller disks.
    #[serde(default)]
    pub(super) propeller_drives: Vec<PropellerDriveAsset>,
    /// Gas turbines coupled to propellers through an explicit power turbine.
    #[serde(default)]
    pub(super) turboprops: Vec<TurbopropAsset>,
    /// Parametric aircraft landing gear or rover wheel chassis. Component
    /// masses participate in the same final center-of-mass bake as mounts.
    #[serde(default)]
    pub(super) wheel_chassis: Vec<WheelChassisAsset>,
    /// Fold-out rocket/lander support legs with reusable or crushable shocks.
    #[serde(default)]
    pub(super) landing_legs: Vec<LandingLegAsset>,
    /// Optional body-axis reaction-wheel banks. Their rated torque is the
    /// only attitude-authority limit; no rotor speed/momentum saturation is
    /// modeled, matching the intended KSP-like gameplay behavior.
    #[serde(default)]
    pub(super) reaction_wheels: Vec<ReactionWheelAsset>,
    /// Optional atmospheric drag devices. Their pack mass is included in the
    /// final center-of-mass and inertia bake.
    #[serde(default)]
    pub(super) parachutes: Vec<ParachuteAsset>,
}

impl VehicleAsset {
    pub(crate) fn bake(self) -> Result<VehicleDefinition, Box<dyn Error>> {
        let mut panels = self
            .panels
            .into_iter()
            .map(PanelAsset::bake)
            .collect::<Result<Vec<_>, _>>()?;
        let mut controls = self
            .control_surfaces
            .into_iter()
            .map(ControlSurfaceAsset::bake)
            .collect::<Result<Vec<_>, _>>()?;
        // Hangar-side procedural compilation: each surface contributes
        // its panels (appended after hand panels) with control indices
        // rebased onto the merged panel list. Structural mass, first
        // moments, and inertia aggregate about the authoring origin;
        // fuel volume is reported for tank placement (no TankShape fits a
        // wing box, so mounts are not fabricated).
        let mut surface_mass_kg = 0.0;
        let mut surface_moment = DVec3::ZERO;
        let mut surface_inertia = DMat3::ZERO;
        let mut surface_fuel_m3 = 0.0;
        let mut fold_joints = Vec::new();
        let mut parked_tags: Vec<(usize, usize)> = Vec::new();
        let mut surface_collision_parts = Vec::new();
        for surface in &self.procedural_surfaces {
            let compiled = compile_surface(
                surface,
                &CompileOptions::default(),
                &MechanismState::deployed(),
            )
            .map_err(|error| format!("surface '{}': {error}", surface.name))?;
            println!(
                "surface '{}': {} panels, estimated error {:.3e} m^2",
                surface.name,
                compiled.panels.len(),
                compiled.summary.estimated_error_m2
            );
            if let Some(structure) = &compiled.structure {
                println!(
                    "surface '{}': structure {:.1} kg, fuel {:.3} m^3 at {:?}",
                    surface.name,
                    structure.mass_kg,
                    structure.fuel_volume_m3,
                    structure.fuel_centroid_body_m
                );
                surface_mass_kg += structure.mass_kg;
                surface_moment += structure.center_of_mass_body_m * structure.mass_kg;
                surface_inertia += structure.inertia_body_kg_m2;
                surface_fuel_m3 += structure.fuel_volume_m3;
            }
            let base = panels.len();
            let joint_base = fold_joints.len();
            // Fold tags resolve against joints attached later, so park
            // them aside: VehicleDefinition::new validates eagerly, then
            // with_fold_joints re-validates, then tags restore plus a
            // final validation below.
            for (panel_index, panel) in compiled.panels.iter().enumerate() {
                let mut panel = *panel;
                if let Some(joint) = compiled.tags[panel_index].fold {
                    parked_tags.push((panels.len(), joint_base + joint));
                    panel.fold_index = None;
                }
                panels.push(panel);
            }
            let def_base = controls.len();
            for definition in &compiled.controls {
                let mut rebased = ControlSurfaceDefinition::new(
                    definition.name.clone(),
                    definition
                        .panel_indices
                        .iter()
                        .map(|index| base + index)
                        .collect(),
                    definition.minimum_deflection_rad,
                    definition.maximum_deflection_rad,
                )?;
                rebased = rebased.with_kind(definition.kind);
                if let Some(mixing) = definition.mixing {
                    rebased = rebased.with_mixing(mixing);
                }
                if let Some(hinge) = definition.hinge {
                    rebased = rebased.with_hinge(hinge);
                }
                if let Some(actuator) = definition.actuator {
                    rebased = rebased.with_actuator(actuator);
                }
                if let Some(parent) = definition.parent_index {
                    rebased = rebased.with_parent(def_base + parent);
                }
                controls.push(rebased);
            }
            for (fold, joint) in compiled.folds.iter().zip(surface.folds.iter()) {
                fold_joints.push(FoldJointRecord {
                    name: format!("{}.{}", surface.name, fold.name),
                    hinge_body_m: fold.hinge_body_m,
                    axis_body: fold.axis_body,
                    angle_rad: fold.angle_rad,
                    deployed_angle_rad: fold.deployed_angle_rad,
                    deployment_rate_rad_s: joint.deployment_rate_rad_s,
                    lock_window_rad: joint.lock_window_rad,
                    max_dynamic_pressure_pa: joint.max_dynamic_pressure_pa,
                    parent_joint: fold.parent_joint.map(|parent| joint_base + parent),
                });
            }
            if self.surface_collision {
                let parts = compiled
                    .collision_parts(&CollisionOptions::default())
                    .map_err(|error| format!("surface '{}': {error}", surface.name))?;
                println!("surface '{}': {} contact boxes", surface.name, parts.len());
                surface_collision_parts.extend(parts);
            }
        }
        // Hangar-side body compilation: each body contributes strip
        // panels (appended after hand and surface panels), control channels
        // rebased onto those panels, hull mass and inertia about the
        // authoring origin, feed-pipeline tank mounts from tank regions,
        // and per-segment contact parts. Ports print as anchor data; engine
        // auto-mounting from ports is future work.
        let mut body_tank_mounts = Vec::new();
        let mut body_contact_parts = Vec::new();
        for body in &self.procedural_bodies {
            let compiled = compile_body(body, &BodyCompileOptions::default())
                .map_err(|error| format!("body '{}': {error}", body.name))?;
            println!(
                "body '{}': {} panels in {} zones, volume {:.3} m^3, wet {:.2} m^2",
                body.name,
                compiled.panels.len(),
                compiled.summary.zone_count,
                compiled.summary.enclosed_volume_m3,
                compiled.summary.wetted_area_m2
            );
            if let Some(structure) = &compiled.structure {
                println!(
                    "body '{}': hull {:.1} kg at {:?}, fuel {:.3} m^3",
                    body.name,
                    structure.mass_kg,
                    structure.center_of_mass_body_m,
                    compiled.summary.tank_capacity_m3
                );
                surface_mass_kg += structure.mass_kg;
                surface_moment += structure.center_of_mass_body_m * structure.mass_kg;
                surface_inertia += structure.inertia_body_kg_m2;
            }
            for tank in &compiled.tanks {
                println!(
                    "body '{}': tank '{}' {:.3} m^3, fill {:.0} kg at {:?}",
                    body.name,
                    tank.region_name,
                    tank.inner_volume_m3,
                    tank.propellant_kg,
                    tank.mount.position_body_m
                );
                body_tank_mounts.push(tank.mount);
            }
            for region in &compiled.interior {
                // Manifest mass already rides the hull accumulators above
                // (single ownership: the compiler aggregates, the baker
                // only prints here).
                if region.payload_mass_kg > 0.0 {
                    println!(
                        "body '{}': region '{}' manifest {:.1} kg",
                        body.name, region.name, region.payload_mass_kg
                    );
                }
            }
            for port in &compiled.ports {
                println!(
                    "body '{}': port '{}' ({:?}) at {:?} along {:?}",
                    body.name, port.name, port.kind, port.position_body_m, port.axis_body_m
                );
            }
            let panel_base = panels.len();
            let control_base = controls.len();
            for definition in &compiled.controls {
                let mut rebased = ControlSurfaceDefinition::new(
                    definition.name.clone(),
                    definition
                        .panel_indices
                        .iter()
                        .map(|index| panel_base + index)
                        .collect(),
                    definition.minimum_deflection_rad,
                    definition.maximum_deflection_rad,
                )?;
                rebased = rebased.with_kind(definition.kind);
                if let Some(mixing) = definition.mixing {
                    rebased = rebased.with_mixing(mixing);
                }
                if let Some(hinge) = definition.hinge {
                    rebased = rebased.with_hinge(hinge);
                }
                if let Some(actuator) = definition.actuator {
                    rebased = rebased.with_actuator(actuator);
                }
                if let Some(parent) = definition.parent_index {
                    rebased = rebased.with_parent(control_base + parent);
                }
                controls.push(rebased);
            }
            panels.extend(compiled.panels.iter().cloned());
            if self.body_collision {
                let parts = body_collision_parts(body, &BodyCollisionOptions::default())
                    .map_err(|error| format!("body '{}': {error}", body.name))?;
                println!("body '{}': {} contact parts", body.name, parts.len());
                body_contact_parts.extend(parts);
            }
        }
        // Bake mounts first (authoring stations): the single final COM
        // below needs every mass contributor before anything shifts.
        let mut collision_parts = self
            .collision_parts
            .into_iter()
            .map(CollisionPartAsset::bake)
            .collect::<Result<Vec<_>, _>>()?;
        let mounts = self
            .engines
            .into_iter()
            .map(EngineAsset::bake)
            .collect::<Result<Vec<_>, _>>()?;
        let mut tank_mounts = self
            .tanks
            .into_iter()
            .map(TankAsset::bake)
            .collect::<Result<Vec<TankMount>, _>>()?;
        // Body tank regions join the hand tanks before the COM pass so
        // they ride the same feed-pressure cross-check and recenter.
        tank_mounts.extend(body_tank_mounts);
        let system_mounts = self
            .systems
            .into_iter()
            .map(SystemAsset::bake)
            .collect::<Result<Vec<SystemMount>, _>>()?;
        let jet_mounts = self
            .jets
            .into_iter()
            .map(JetAsset::bake)
            .collect::<Result<Vec<_>, _>>()?;
        let electric_thruster_mounts = self
            .electric_thrusters
            .into_iter()
            .map(ElectricThrusterAsset::bake)
            .collect::<Result<Vec<_>, _>>()?;
        let fusion_torch_mounts = self
            .fusion_torches
            .into_iter()
            .map(FusionTorchAsset::bake)
            .collect::<Result<Vec<_>, _>>()?;
        let pulsed_fusion_mounts = self
            .pulsed_fusion_systems
            .into_iter()
            .map(PulsedFusionAsset::bake)
            .collect::<Result<Vec<_>, _>>()?;
        let propeller_drive_mounts = self
            .propeller_drives
            .into_iter()
            .map(PropellerDriveAsset::bake)
            .collect::<Result<Vec<_>, _>>()?;
        let turboprop_mounts = self
            .turboprops
            .into_iter()
            .map(TurbopropAsset::bake)
            .collect::<Result<Vec<_>, _>>()?;
        let wheel_chassis_specs = self
            .wheel_chassis
            .into_iter()
            .map(WheelChassisAsset::bake)
            .collect::<Result<Vec<_>, _>>()?;
        let wheel_chassis_components = wheel_chassis_specs
            .iter()
            .cloned()
            .map(|spec| spec.compile())
            .collect::<Result<Vec<_>, _>>()?;
        let landing_leg_specs: Vec<_> = self
            .landing_legs
            .into_iter()
            .map(LandingLegAsset::bake)
            .collect();
        let landing_leg_components = landing_leg_specs
            .iter()
            .cloned()
            .map(LandingLegSpec::compile)
            .collect::<Result<Vec<_>, _>>()?;
        let reaction_wheel_banks: Vec<_> = self
            .reaction_wheels
            .into_iter()
            .map(ReactionWheelAsset::bake)
            .collect();
        let parachutes: Vec<_> = self
            .parachutes
            .into_iter()
            .map(ParachuteAsset::bake)
            .collect();
        // Assembly center of mass over EVERYTHING: hand mass rides the
        // authoring origin, surfaces/engine/tank/system/jet masses ride
        // their stations. Flight integrates moments about the body
        // origin, so the baker recenters the whole asset onto the final
        // COM in one shift (legacy hand-only assets sit at zero and
        // shift by nothing). Engine/tank/system/jet mass calls below
        // then add point terms about already-centered stations, and the
        // same accumulator shape serves future fuel-driven COM motion.
        let mut total_mass_kg = self.mass_kg + surface_mass_kg;
        let mut total_moment = surface_moment;
        for mount in &mounts {
            let mass = mount.engine.bake_mass_kg();
            total_mass_kg += mass;
            total_moment += DVec3::from_array(mount.position_body_m) * mass;
        }
        for mount in &tank_mounts {
            let mass = mount.tank.dry_mass_kg + mount.loaded_propellant_kg();
            total_mass_kg += mass;
            total_moment += DVec3::from_array(mount.position_body_m) * mass;
        }
        for mount in &system_mounts {
            let mut chamber_mass = 0.0;
            let mut centroid = DVec3::ZERO;
            for chamber in &mount.system.chambers {
                total_mass_kg += chamber.dry_mass_kg;
                chamber_mass += chamber.dry_mass_kg;
                let at = DVec3::from_array(chamber.position_body_m);
                total_moment += at * chamber.dry_mass_kg;
                centroid += at * chamber.dry_mass_kg;
            }
            let shared = (mount.system.dry_mass_kg - chamber_mass).max(0.0);
            total_mass_kg += shared;
            if chamber_mass > 0.0 {
                total_moment += centroid / chamber_mass * shared;
            }
        }
        for mount in &jet_mounts {
            let mass = mount.engine.dry_mass_kg();
            total_mass_kg += mass;
            total_moment += DVec3::from_array(mount.position_body_m) * mass;
        }
        for mount in &electric_thruster_mounts {
            let mass = mount.engine.dry_mass_kg;
            total_mass_kg += mass;
            total_moment += DVec3::from_array(mount.position_body_m) * mass;
        }
        for mount in &fusion_torch_mounts {
            let mass = mount.engine.dry_mass_kg;
            total_mass_kg += mass;
            total_moment += DVec3::from_array(mount.position_body_m) * mass;
        }
        for mount in &pulsed_fusion_mounts {
            let mass = mount.engine.dry_mass_kg;
            total_mass_kg += mass;
            total_moment += DVec3::from_array(mount.position_body_m) * mass;
        }
        for mount in &propeller_drive_mounts {
            let mass = mount.drive.dry_mass_kg;
            total_mass_kg += mass;
            total_moment += DVec3::from_array(mount.position_body_m) * mass;
        }
        for mount in &turboprop_mounts {
            let mass = mount.drive.dry_mass_kg;
            total_mass_kg += mass;
            total_moment += DVec3::from_array(mount.position_body_m) * mass;
        }
        for chassis in &wheel_chassis_components {
            total_mass_kg += chassis.mass_properties.mass_kg;
            total_moment +=
                chassis.mass_properties.center_of_mass_body_m * chassis.mass_properties.mass_kg;
        }
        for leg in &landing_leg_components {
            total_mass_kg += leg.mass_properties.mass_kg;
            total_moment += leg.mass_properties.center_of_mass_body_m * leg.mass_properties.mass_kg;
        }
        for bank in &reaction_wheel_banks {
            total_mass_kg += bank.mass_kg;
            total_moment += bank.position_body_m * bank.mass_kg;
        }
        for parachute in &parachutes {
            total_mass_kg += parachute.pack_mass_kg;
            total_moment += parachute.position_body_m * parachute.pack_mass_kg;
        }
        let assembly_com = if total_mass_kg > 0.0 {
            total_moment / total_mass_kg
        } else {
            DVec3::ZERO
        };
        if assembly_com.length() > 1e-12 {
            println!(
                "assembly center of mass at [{:.3}, {:.3}, {:.3}], recentering",
                assembly_com.x, assembly_com.y, assembly_com.z
            );
        }
        let shift = -assembly_com;
        let geometry = AeroGeometry::new(panels)?;
        // Properties stay in the authoring frame here (hand plus
        // surfaces, always positive-definite); the single recenter to
        // the final COM happens on the built vehicle below, after every
        // mass contributor is attached. Recentering an intermediate sum
        // that misses mount masses can go indefinite.
        let inertia = rows_to_matrix(self.inertia_body_kg_m2);
        let properties =
            RigidBodyProperties::new(self.mass_kg + surface_mass_kg, inertia + surface_inertia)?;
        if surface_fuel_m3 > 0.0 {
            println!("wing fuel volume: {surface_fuel_m3:.3} m^3");
        }
        collision_parts.extend(surface_collision_parts);
        collision_parts.extend(body_contact_parts);
        let collision_geometry = CollisionGeometry::new(collision_parts)?;
        // Feed cross-check: pressure-fed engines have no pump to hide
        // behind, so a tank must hold their full feed pressure. Pump-fed
        // cycles generate the rise themselves (chamber pressure is already
        // gated by the cycle cap at compile time).
        let strongest_tank_pa = tank_mounts
            .iter()
            .map(|mount| mount.tank.max_pressure_pa)
            .fold(0.0_f64, f64::max);
        for mount in &mounts {
            if let CompiledEngine::Liquid(engine) = &mount.engine
                && engine.cycle == EngineCycle::PressureFed
                && strongest_tank_pa < engine.feed_pressure_required_pa
            {
                return Err(format!(
                    "tank pressure {:.2} MPa cannot pressure-feed {} (needs {:.2} MPa)",
                    strongest_tank_pa / 1.0e6,
                    mount.name,
                    engine.feed_pressure_required_pa / 1.0e6
                )
                .into());
            }
        }
        for mount in &system_mounts {
            if mount.system.cycle == EngineCycle::PressureFed
                && strongest_tank_pa < mount.system.feed_pressure_required_pa
            {
                return Err(format!(
                    "tank pressure {:.2} MPa cannot pressure-feed {} (needs {:.2} MPa)",
                    strongest_tank_pa / 1.0e6,
                    mount.name,
                    mount.system.feed_pressure_required_pa / 1.0e6
                )
                .into());
            }
        }
        let mut vehicle = VehicleDefinition::new(self.name, geometry, properties, controls)?
            .with_collision_geometry(collision_geometry)?
            .with_engines(mounts)?
            .with_tanks(tank_mounts)?
            .with_systems(system_mounts)?
            .with_fold_joints(fold_joints)?
            .with_jets(jet_mounts)?
            .with_electric_thrusters(electric_thruster_mounts)?
            .with_fusion_torches(fusion_torch_mounts)?
            .with_pulsed_fusion_systems(pulsed_fusion_mounts)?
            .with_propeller_drives(propeller_drive_mounts)?
            .with_turboprops(turboprop_mounts)?
            .with_wheel_chassis(wheel_chassis_specs)?
            .with_landing_legs(landing_leg_specs)?
            .with_reaction_wheels(reaction_wheel_banks)?
            .with_parachutes(parachutes)?;
        vehicle.bake_engine_masses()?;
        vehicle.bake_tank_masses()?;
        vehicle.bake_system_masses()?;
        vehicle.bake_jet_masses()?;
        vehicle.bake_electric_thruster_masses()?;
        vehicle.bake_fusion_masses()?;
        vehicle.bake_propeller_drive_masses()?;
        vehicle.bake_turboprop_masses()?;
        vehicle.bake_wheel_chassis_masses()?;
        vehicle.bake_landing_leg_masses()?;
        vehicle.bake_reaction_wheel_masses()?;
        vehicle.bake_parachute_masses()?;
        for (panel_index, joint) in parked_tags {
            vehicle.aero_geometry.panels[panel_index].fold_index = Some(joint);
        }
        // Single final recenter onto the assembly COM: every station
        // rides along, and the total inertia shifts by one parallel-axis
        // term. Doing it here (all masses attached) instead of on an
        // intermediate sum keeps every step positive-definite.
        let shift_point = |point: DVec3| point + shift;
        for panel in &mut vehicle.aero_geometry.panels {
            panel.position_body_m = shift_point(panel.position_body_m);
            panel.center_of_pressure_body_m = shift_point(panel.center_of_pressure_body_m);
        }
        for joint in &mut vehicle.fold_joints {
            joint.hinge_body_m = shift_point(joint.hinge_body_m);
        }
        for control in &mut vehicle.control_surfaces {
            if let Some(hinge) = &mut control.hinge {
                hinge.point_body_m = shift_point(hinge.point_body_m);
            }
        }
        for part in &mut vehicle.collision_geometry.parts {
            part.local_position_m = shift_point(part.local_position_m);
        }
        for mount in &mut vehicle.engines {
            shift_array(&mut mount.position_body_m, shift);
        }
        for mount in &mut vehicle.tanks {
            shift_array(&mut mount.position_body_m, shift);
        }
        for mount in &mut vehicle.systems {
            for chamber in &mut mount.system.chambers {
                shift_array(&mut chamber.position_body_m, shift);
            }
        }
        for mount in &mut vehicle.jets {
            shift_array(&mut mount.position_body_m, shift);
        }
        for mount in &mut vehicle.electric_thrusters {
            shift_array(&mut mount.position_body_m, shift);
        }
        for mount in &mut vehicle.fusion_torches {
            shift_array(&mut mount.position_body_m, shift);
        }
        for mount in &mut vehicle.pulsed_fusion_systems {
            shift_array(&mut mount.position_body_m, shift);
        }
        for mount in &mut vehicle.propeller_drives {
            shift_array(&mut mount.position_body_m, shift);
        }
        for mount in &mut vehicle.turboprops {
            shift_array(&mut mount.position_body_m, shift);
        }
        for chassis in &mut vehicle.wheel_chassis {
            chassis.spec.mount_position_body_m += shift;
            if let Some(retraction) = &mut chassis.spec.retraction {
                retraction.pivot_position_body_m += shift;
            }
            *chassis = chassis
                .spec
                .clone()
                .compile()
                .map_err(|error| format!("wheel chassis '{}': {error}", chassis.spec.name))?;
        }
        for leg in &mut vehicle.landing_legs {
            leg.spec.mount_position_body_m = shift_point(leg.spec.mount_position_body_m);
            *leg = leg
                .spec
                .clone()
                .compile()
                .map_err(|error| format!("landing leg '{}': {error}", leg.spec.name))?;
        }
        for bank in &mut vehicle.reaction_wheels {
            bank.position_body_m = shift_point(bank.position_body_m);
        }
        for parachute in &mut vehicle.parachutes {
            parachute.position_body_m = shift_point(parachute.position_body_m);
        }
        let total = vehicle.mass_properties.mass_kg;
        let recentered =
            vehicle.mass_properties.inertia_body_kg_m2 - parallel_axis(total, assembly_com);
        vehicle.mass_properties = RigidBodyProperties::new(total, recentered)?;
        vehicle.validate()?;
        Ok(vehicle)
    }
}
