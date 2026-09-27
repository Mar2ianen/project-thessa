//! Top-level vehicle asset and ordered compilation into runtime geometry, mass, and mounts.

use super::*;

#[derive(Debug, Deserialize)]
pub(crate) struct VehicleAsset {
    pub(super) name: String,
    pub(crate) mass_kg: f64,
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
    /// Optional rigid assembly of authored body parts with live crew, air,
    /// and resource connectivity.
    #[serde(default)]
    pub(crate) assembly: AssemblyAsset,
    /// Optional consumer-name to assembly feed-port routes.
    #[serde(default)]
    pub(crate) resource_feed_ports: Vec<ResourceFeedPortAsset>,
    /// Optional shared bus with rated loads, storage, solar cells, and reactors.
    #[serde(default)]
    pub(crate) electrical_power: ElectricalPowerAsset,
    /// Optional lumped thermal-node network (nodes, links, radiators).
    #[serde(default)]
    pub(crate) thermal: ThermalAsset,
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
    /// Fuel-burning turbo-generator auxiliary power units.
    #[serde(default)]
    pub(super) auxiliary_power_units: Vec<AuxiliaryPowerUnitAsset>,
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
    /// Mounted monopropellant or cold-gas RCS thrusters.
    #[serde(default)]
    pub(super) rcs_mounts: Vec<RcsMountAsset>,
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
        let mut surface_tile_nodes = Vec::new();
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
            if let (Some(layer), Some(tiles)) = (&surface.tile_layer, &compiled.tile_layer) {
                println!(
                    "surface '{}': {} hex tiles, {:.1} kg at {:?}",
                    surface.name, tiles.tile_count, tiles.mass_kg, tiles.centroid_body_m
                );
                // Tile mass rides the thermal bake below (one lumped node
                // per surface), never the surface accumulators: adding it
                // here would double-count.
                // One lumped tile node per surface (both sides paved, so the
                // node radiates both faces but takes sun/aero on one face
                // through the mean normal; backside solar is future work).
                surface_tile_nodes.push(ThermalNodeSpec {
                    name: format!("{}.tiles", surface.name),
                    mass_kg: tiles.mass_kg,
                    specific_heat_j_kg_k: layer.specific_heat_j_kg_k,
                    initial_temp_k: 280.0,
                    max_temp_k: layer.max_temp_k,
                    emissivity: layer.emissivity,
                    solar_absorptivity: layer.solar_absorptivity,
                    radiating_area_m2: tiles.area_m2,
                    solar_exposed_area_m2: 0.5 * tiles.area_m2,
                    solar_normal_body: tiles.normal_body_m,
                    aero_area_m2: 0.5 * tiles.area_m2,
                    nose_radius_m: layer.nose_radius_m,
                    position_body_m: tiles.centroid_body_m,
                });
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
        let mut body_heat_shield_mounts = Vec::new();
        let mut body_contact_parts = Vec::new();
        let mut body_cabins = Vec::new();
        let mut body_cabin_exits = Vec::new();
        let mut body_cabin_seats = Vec::new();
        let mut body_cabin_monuments = Vec::new();
        let mut body_cores = Vec::new();
        let mut body_stations = Vec::new();
        let mut assembly_volumes = Vec::new();
        let compiled_assembly = if self.assembly.links.is_empty() {
            None
        } else {
            let links = resolve_assembly_links(&self.assembly.links)?;
            let assembly = compile_assembly(&self.procedural_bodies, &links)
                .map_err(|error| format!("assembly: {error}"))?;
            println!("assembly root: {}", assembly.root);
            for (index, group) in assembly.crew_groups.iter().enumerate() {
                println!("assembly crew domain {index}: {} volumes", group.len());
            }
            for (index, group) in assembly.air_groups.iter().enumerate() {
                println!("assembly air domain {index}: {} volumes", group.len());
            }
            for path in &assembly.feed_paths {
                println!("assembly feed: {} -> {}", path.tank, path.engine_port);
            }
            Some(assembly)
        };
        for (body_index, body) in self.procedural_bodies.iter().enumerate() {
            let transform = compiled_assembly
                .as_ref()
                .map(|assembly| assembly.body_transforms[body_index])
                .unwrap_or(BodyTransform {
                    rotation_body: DQuat::IDENTITY,
                    translation_body_m: body.origin_body_m,
                });
            let mut part_frame_body = body.clone();
            // Assembly transforms own part placement; compile in the part's
            // local frame to avoid applying the authored origin twice.
            part_frame_body.origin_body_m = DVec3::ZERO;
            let mut compiled = compile_body(&part_frame_body, &BodyCompileOptions::default())
                .map_err(|error| format!("body '{}': {error}", body.name))?;
            transform_compiled_body(&mut compiled, transform);
            println!(
                "body '{}': assembled at {:?} (rotation {:?})",
                body.name, transform.translation_body_m, transform.rotation_body
            );
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
                let mut mount = tank.mount.clone();
                mount.name = format!("{}.{}", body.name, tank.region_name);
                body_tank_mounts.push(mount);
            }
            for shield in &compiled.heat_shields {
                println!(
                    "body '{}': heat shield '{}' {:.2} m at {:?} along {:?}",
                    body.name,
                    shield.name,
                    shield.diameter_m,
                    shield.position_body_m,
                    shield.normal_body_m
                );
                body_heat_shield_mounts.push(
                    HeatShieldMount::new(
                        shield.name.clone(),
                        shield.position_body_m,
                        shield.normal_body_m,
                        shield.diameter_m,
                    )
                    .map_err(|error| format!("body '{}': {error}", body.name))?,
                );
            }
            for region in &compiled.interior {
                if compiled_assembly.is_some()
                    && !matches!(
                        region.kind,
                        RegionKind::Tank { .. }
                            | RegionKind::FluidTank { .. }
                            | RegionKind::Bipropellant { .. }
                    )
                {
                    assembly_volumes.push(AssemblyVolume {
                        name: format!("{}.{}", body.name, region.name),
                        body: body_index,
                        pressurized: region.atmosphere.is_some(),
                        volume_m3: region.volume_m3,
                        centroid_body_m: region.centroid_body_m,
                        seats: region.seats,
                        seat_positions_body_m: region.seat_positions_body_m.clone(),
                    });
                }
                // Manifest mass already rides the hull accumulators above
                // (single ownership: the compiler aggregates, the baker
                // only prints here).
                if region.payload_mass_kg > 0.0 {
                    println!(
                        "body '{}': region '{}' manifest {:.1} kg",
                        body.name, region.name, region.payload_mass_kg
                    );
                }
                if region.air_mass_kg > 0.0 {
                    println!(
                        "body '{}': region '{}' air {:.2} kg (O2 {:.2} kg)",
                        body.name, region.name, region.air_mass_kg, region.o2_mass_kg
                    );
                }
                if let Some(atmosphere) = region.atmosphere {
                    println!(
                        "body '{}': region '{}' cabin {:.1} kPa, {:.2} kg air",
                        body.name, region.name, atmosphere.pressure_kpa, region.air_mass_kg
                    );
                    body_cabins.push(
                        PressurizedCabin::new(
                            format!("{}.{}", body.name, region.name),
                            region.volume_m3,
                            atmosphere.pressure_kpa,
                            atmosphere.temp_k,
                            atmosphere.o2_fraction,
                            region.air_mass_kg,
                        )
                        .and_then(|cabin| cabin.with_centroid_body_m(region.centroid_body_m))
                        .map_err(|error| format!("body '{}': {error}", body.name))?,
                    );
                }
                if let Some(tier) = region.control_core {
                    println!(
                        "body '{}': region '{}' autopilot core ({tier:?})",
                        body.name, region.name
                    );
                    body_cores.push(ControlCore {
                        name: format!("{}.{}", body.name, region.name),
                        tier,
                    });
                }
                if let RegionKind::Crew {
                    control_station,
                    seats,
                    occupant_mass_kg_each,
                    ..
                } = region.kind
                    && control_station
                {
                    let occupied = seats > 0 && occupant_mass_kg_each > 0.0;
                    println!(
                        "body '{}': region '{}' pilot station ({})",
                        body.name,
                        region.name,
                        if occupied { "occupied" } else { "empty" }
                    );
                    body_stations.push(ControlStation {
                        name: format!("{}.{}", body.name, region.name),
                        occupied,
                    });
                }
                for seat in region
                    .cabin_seats
                    .iter()
                    .filter(|seat| seat.role == CabinSeatRole::FlightCrew)
                {
                    body_stations.push(ControlStation {
                        name: seat.name.clone(),
                        occupied: seat.occupied,
                    });
                }
                for seat in &region.cabin_seats {
                    body_cabin_seats.push(CabinSeat {
                        name: seat.name.clone(),
                        position_body_m: seat.position_body_m,
                        class: match seat.class {
                            thessa_fuselage::SeatClass::Economy => CabinSeatClass::Economy,
                            thessa_fuselage::SeatClass::Premium => CabinSeatClass::Premium,
                            thessa_fuselage::SeatClass::Business => CabinSeatClass::Business,
                            thessa_fuselage::SeatClass::First => CabinSeatClass::First,
                            thessa_fuselage::SeatClass::Ejection => CabinSeatClass::Ejection,
                        },
                        role: match seat.role {
                            CabinSeatRole::Passenger => RuntimeCabinSeatRole::Passenger,
                            CabinSeatRole::FlightCrew => RuntimeCabinSeatRole::FlightCrew,
                            CabinSeatRole::CabinAttendant => RuntimeCabinSeatRole::CabinAttendant,
                        },
                        seat_style: match seat.seat_style {
                            thessa_fuselage::SeatStyle::Upright => CabinSeatStyle::Upright,
                            thessa_fuselage::SeatStyle::Couch => CabinSeatStyle::Couch,
                            thessa_fuselage::SeatStyle::Ejection => CabinSeatStyle::Ejection,
                        },
                        occupied: seat.occupied,
                        suited: seat.suited,
                        suit_type: match seat.suit_type {
                            thessa_fuselage::SuitType::HoseFed => CabinSuitType::HoseFed,
                            thessa_fuselage::SuitType::SelfContained => {
                                CabinSuitType::SelfContained
                            }
                        },
                        seat_mass_kg: seat.seat_mass_kg,
                        occupant_mass_kg: seat.occupant_mass_kg,
                        carry_on_mass_kg: seat.carry_on_mass_kg,
                        suit_mass_kg: seat.suit_mass_kg,
                    });
                }
                for monument in &region.cabin_monuments {
                    body_cabin_monuments.push(CabinMonument {
                        name: monument.name.clone(),
                        kind: match monument.kind {
                            thessa_fuselage::MonumentKind::Galley => CabinMonumentKind::Galley,
                            thessa_fuselage::MonumentKind::Lavatory => CabinMonumentKind::Lavatory,
                            thessa_fuselage::MonumentKind::Closet => CabinMonumentKind::Closet,
                            thessa_fuselage::MonumentKind::FlightDeck => {
                                CabinMonumentKind::FlightDeck
                            }
                            thessa_fuselage::MonumentKind::AvionicsRack => {
                                CabinMonumentKind::AvionicsRack
                            }
                        },
                        position_body_m: monument.position_body_m,
                        mass_kg: monument.mass_kg,
                    });
                }
                for exit in &region.cabin_doors {
                    body_cabin_exits.push(CabinExit {
                        name: exit.name.clone(),
                        pair_id: exit.pair_id.clone(),
                        position_body_m: exit.position_body_m,
                        side: match exit.side {
                            DoorSide::Left => CabinExitSide::Left,
                            DoorSide::Right => CabinExitSide::Right,
                        },
                        exit_type: match exit.rating {
                            ExitType::TypeA => CabinExitType::TypeA,
                            ExitType::TypeB => CabinExitType::TypeB,
                            ExitType::TypeC => CabinExitType::TypeC,
                            ExitType::TypeI => CabinExitType::TypeI,
                            ExitType::TypeII => CabinExitType::TypeII,
                            ExitType::TypeIII => CabinExitType::TypeIII,
                            ExitType::TypeIV => CabinExitType::TypeIV,
                        },
                        opening_width_m: exit.opening_width_m,
                        opening_height_m: exit.opening_height_m,
                    });
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
                let mut parts =
                    body_collision_parts(&part_frame_body, &BodyCollisionOptions::default())
                        .map_err(|error| format!("body '{}': {error}", body.name))?;
                for part in &mut parts {
                    part.local_position_m = transform.transform_point(part.local_position_m);
                    part.local_orientation =
                        (transform.rotation_body * part.local_orientation).normalize();
                }
                println!("body '{}': {} contact parts", body.name, parts.len());
                body_contact_parts.extend(parts);
            }
        }
        let runtime_assembly = compiled_assembly
            .as_ref()
            .map(|assembly| {
                runtime_assembly(
                    &self.procedural_bodies,
                    &self.assembly.links,
                    &assembly.root,
                    assembly_volumes,
                )
            })
            .transpose()?;
        if let Some(assembly) = &runtime_assembly {
            let initial_cabins = body_cabins.clone();
            assembly
                .equalize_cabin_states(&mut body_cabins)
                .map_err(|error| format!("assembly cabin equilibrium: {error}"))?;
            for cabin in &body_cabins {
                let initial = initial_cabins
                    .iter()
                    .find(|initial| initial.name == cabin.name)
                    .ok_or_else(|| {
                        format!(
                            "assembly equilibrium introduced unknown cabin '{}'",
                            cabin.name
                        )
                    })?;
                let delta_mass_kg = cabin.air_kg - initial.air_kg;
                if delta_mass_kg != 0.0 {
                    let volume = assembly
                        .volumes
                        .iter()
                        .find(|volume| volume.name == cabin.name)
                        .ok_or_else(|| {
                            format!("assembly has no volume for cabin '{}'", cabin.name)
                        })?;
                    surface_mass_kg += delta_mass_kg;
                    surface_moment += volume.centroid_body_m * delta_mass_kg;
                    surface_inertia += parallel_axis(delta_mass_kg, volume.centroid_body_m);
                }
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
        let auxiliary_power_unit_mounts = self
            .auxiliary_power_units
            .into_iter()
            .map(AuxiliaryPowerUnitAsset::bake)
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
        let rcs_mounts = self
            .rcs_mounts
            .into_iter()
            .map(RcsMountAsset::bake)
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
        let electrical_power = self.electrical_power.bake();
        let power_mass_properties = electrical_power.mass_properties()?;
        let resource_feed_ports = self
            .resource_feed_ports
            .into_iter()
            .map(ResourceFeedPortAsset::bake)
            .collect();
        let mut thermal = self.thermal.bake();
        // Wing/tail tile layers arrive as lumped nodes (one per surface);
        // duplicate names with authored [thermal] nodes fail closed below.
        thermal.nodes.extend(surface_tile_nodes);
        let thermal_mass_properties = thermal.mass_properties()?;
        // Assembly center of mass over EVERYTHING: hand mass rides the
        // authoring origin; surfaces, propulsion, electrical power hardware,
        // thermal hardware, and other installed masses ride their authored
        // stations. Flight
        // integrates moments about the body origin, so the baker recenters
        // the whole asset onto the final
        // COM in one shift (legacy hand-only assets sit at zero and
        // shift by nothing). Engine/tank/system/jet mass calls below
        // then add point terms about already-centered stations, and the
        // same accumulator shape serves future fuel-driven COM motion.
        let mut total_mass_kg = self.mass_kg
            + surface_mass_kg
            + power_mass_properties.mass_kg
            + thermal_mass_properties.mass_kg;
        let mut total_moment = surface_moment
            + power_mass_properties.center_of_mass_body_m * power_mass_properties.mass_kg
            + thermal_mass_properties.center_of_mass_body_m * thermal_mass_properties.mass_kg;
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
        let mut geometry = AeroGeometry::new(panels)?;
        // Shield discs join the shared aero summation exactly like panels:
        // same geometry object, same flow solution, same result. Mounts
        // stay on the vehicle for identity and validation.
        for mount in &body_heat_shield_mounts {
            geometry.blunt_discs.push(AeroBluntDisc {
                position_body_m: mount.position_body_m,
                normal_body_m: mount.normal_body_m,
                area_m2: mount.area_m2(),
            });
        }
        geometry.validate()?;
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
            .with_auxiliary_power_units(auxiliary_power_unit_mounts)?
            .with_electric_thrusters(electric_thruster_mounts)?
            .with_fusion_torches(fusion_torch_mounts)?
            .with_pulsed_fusion_systems(pulsed_fusion_mounts)?
            .with_propeller_drives(propeller_drive_mounts)?
            .with_turboprops(turboprop_mounts)?
            .with_rcs_mounts(rcs_mounts)?
            .with_cabins(body_cabins)?
            .with_cabin_exits(body_cabin_exits)?
            .with_cabin_seats(body_cabin_seats)?
            .with_cabin_monuments(body_cabin_monuments)?
            .with_control_cores(body_cores)?
            .with_control_stations(body_stations)?
            .with_wheel_chassis(wheel_chassis_specs)?
            .with_landing_legs(landing_leg_specs)?
            .with_reaction_wheels(reaction_wheel_banks)?
            .with_parachutes(parachutes)?
            .with_heat_shields(body_heat_shield_mounts)?
            .with_electrical_power(electrical_power)?
            .with_thermal(thermal)?;
        vehicle = vehicle.with_resource_feed_ports(resource_feed_ports)?;
        if let Some(assembly) = runtime_assembly {
            vehicle = vehicle.with_assembly(assembly)?;
            for cabin in &vehicle.cabins {
                println!(
                    "assembly cabin '{}': equilibrium {:.2} kPa, {:.3} kg air at {:.1} K",
                    cabin.name,
                    cabin.current_pressure_kpa(),
                    cabin.air_kg,
                    cabin.temp_k
                );
            }
        }
        vehicle.bake_engine_masses()?;
        vehicle.bake_tank_masses()?;
        vehicle.bake_system_masses()?;
        vehicle.bake_jet_masses()?;
        vehicle.bake_auxiliary_power_unit_masses()?;
        vehicle.bake_electric_thruster_masses()?;
        vehicle.bake_fusion_masses()?;
        vehicle.bake_propeller_drive_masses()?;
        vehicle.bake_turboprop_masses()?;
        vehicle.bake_rcs_masses()?;
        vehicle.bake_wheel_chassis_masses()?;
        vehicle.bake_landing_leg_masses()?;
        vehicle.bake_reaction_wheel_masses()?;
        vehicle.bake_parachute_masses()?;
        vehicle.bake_electrical_power_masses()?;
        vehicle.bake_thermal_masses()?;
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
        for disc in &mut vehicle.aero_geometry.blunt_discs {
            disc.position_body_m = shift_point(disc.position_body_m);
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
        for mount in &mut vehicle.auxiliary_power_units {
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
        for mount in &mut vehicle.rcs_mounts {
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
        for shield in &mut vehicle.heat_shields {
            shield.position_body_m = shift_point(shield.position_body_m);
        }
        for battery in &mut vehicle.electrical_power.batteries {
            battery.position_body_m = shift_point(battery.position_body_m);
        }
        for capacitor in &mut vehicle.electrical_power.ultracapacitors {
            capacitor.position_body_m = shift_point(capacitor.position_body_m);
        }
        for array in &mut vehicle.electrical_power.solar_arrays {
            array.position_body_m = shift_point(array.position_body_m);
        }
        for reactor in &mut vehicle.electrical_power.reactors {
            reactor.position_body_m = shift_point(reactor.position_body_m);
        }
        for fuel_cell in &mut vehicle.electrical_power.fuel_cells {
            fuel_cell.position_body_m = shift_point(fuel_cell.position_body_m);
        }
        for node in &mut vehicle.thermal.nodes {
            node.position_body_m = shift_point(node.position_body_m);
        }
        for radiator in &mut vehicle.thermal.radiators {
            radiator.position_body_m = shift_point(radiator.position_body_m);
        }
        for cabin in &mut vehicle.cabins {
            cabin.centroid_body_m = shift_point(cabin.centroid_body_m);
        }
        for exit in &mut vehicle.cabin_exits {
            exit.position_body_m = shift_point(exit.position_body_m);
        }
        for seat in &mut vehicle.cabin_seats {
            seat.position_body_m = shift_point(seat.position_body_m);
        }
        for monument in &mut vehicle.cabin_monuments {
            monument.position_body_m = shift_point(monument.position_body_m);
        }
        if let Some(assembly) = &mut vehicle.assembly {
            for volume in &mut assembly.volumes {
                volume.centroid_body_m = shift_point(volume.centroid_body_m);
                for seat in &mut volume.seat_positions_body_m {
                    *seat = shift_point(*seat);
                }
            }
        }
        let total = vehicle.mass_properties.mass_kg;
        let recentered =
            vehicle.mass_properties.inertia_body_kg_m2 - parallel_axis(total, assembly_com);
        vehicle.mass_properties = RigidBodyProperties::new(total, recentered)?;
        vehicle.validate()?;
        Ok(vehicle)
    }
}

fn transform_compiled_body(compiled: &mut CompiledBody, transform: BodyTransform) {
    for panel in &mut compiled.panels {
        panel.position_body_m = transform.transform_point(panel.position_body_m);
        panel.center_of_pressure_body_m =
            transform.transform_point(panel.center_of_pressure_body_m);
        panel.chord_axis_body = transform.transform_direction(panel.chord_axis_body);
        panel.lift_axis_body = transform.transform_direction(panel.lift_axis_body);
    }
    for control in &mut compiled.controls {
        if let Some(hinge) = &mut control.hinge {
            hinge.point_body_m = transform.transform_point(hinge.point_body_m);
            hinge.axis_body = transform.transform_direction(hinge.axis_body).normalize();
        }
    }
    if let Some(structure) = &mut compiled.structure {
        let local_center = structure.center_of_mass_body_m;
        let centroidal_inertia =
            structure.inertia_body_kg_m2 - parallel_axis(structure.mass_kg, local_center);
        structure.center_of_mass_body_m = transform.transform_point(local_center);
        structure.inertia_body_kg_m2 = transform.rotate_inertia(centroidal_inertia)
            + parallel_axis(structure.mass_kg, structure.center_of_mass_body_m);
    }
    for tank in &mut compiled.tanks {
        let mount = &mut tank.mount;
        mount.position_body_m = transform
            .transform_point(DVec3::from_array(mount.position_body_m))
            .to_array();
        mount.intrinsic_inertia_body_kg_m2 =
            transform.rotate_inertia(mount.intrinsic_inertia_body_kg_m2);
    }
    for shield in &mut compiled.heat_shields {
        shield.position_body_m = transform.transform_point(shield.position_body_m);
        shield.normal_body_m = transform.transform_direction(shield.normal_body_m);
    }
    for region in &mut compiled.interior {
        region.centroid_body_m = transform.transform_point(region.centroid_body_m);
        for seat in &mut region.seat_positions_body_m {
            *seat = transform.transform_point(*seat);
        }
        for seat in &mut region.cabin_seats {
            seat.position_body_m = transform.transform_point(seat.position_body_m);
        }
        for monument in &mut region.cabin_monuments {
            monument.position_body_m = transform.transform_point(monument.position_body_m);
        }
        for door in &mut region.cabin_doors {
            door.position_body_m = transform.transform_point(door.position_body_m);
        }
    }
    for port in &mut compiled.ports {
        port.position_body_m = transform.transform_point(port.position_body_m);
        port.axis_body_m = transform.transform_direction(port.axis_body_m).normalize();
    }
    compiled.summary.center_of_volume_m =
        transform.transform_point(compiled.summary.center_of_volume_m);
}
