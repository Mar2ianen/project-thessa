use super::*;
use glam::{DQuat, DVec3};
use thessa_autopilot::PlanExecutionMode;
use thessa_sim_core::{
    ChamberMaterial, CoolingMode, EngineCycle, EngineMount, LiquidEngineSpec, NozzleContour,
    Propellant, TankMount, TankResource, TankShape, TankSpec, VehiclePartCommand,
};

fn input(commands: Vec<Command>) -> ClientInput {
    ClientInput {
        tick: 0,
        control_input: [0.0; 3],
        control_mode: ControlMode::Direct,
        sas_target_xyzw: [0.0, 0.0, 0.0, 1.0],
        throttle: 0.0,
        engine_active: false,
        sas_enabled: false,
        rcs_enabled: false,
        reaction_wheels_enabled: true,
        gear_down: false,
        parachutes_armed: false,
        commands,
    }
}

#[test]
fn ingress_dispatch_decodes_one_payload_selected_by_envelope_kind() {
    let split_wire_frame = |wire: Vec<u8>| {
        FrameDecoder::new()
            .push(&wire)
            .expect("valid framed message")
            .remove(0)
    };
    let client_input = input(Vec::new());
    let client_frame = split_wire_frame(thessa_flight_net::encode_input(&client_input).unwrap());
    assert!(matches!(
        decode_client_message(&client_frame),
        Some(DecodedClientMessage::Input(decoded)) if decoded == client_input
    ));

    let guidance = GuidanceInput {
        tick: 7,
        intent: GuidanceIntent::Attitude {
            target_body_to_inertial: DQuat::from_rotation_y(0.05),
            roll_policy: RollPolicy::Hold,
        },
        propulsion: PropulsionDemand::new(0.25).unwrap(),
    };
    let guidance_frame = split_wire_frame(thessa_flight_net::encode_guidance(&guidance).unwrap());
    assert!(matches!(
        decode_client_message(&guidance_frame),
        Some(DecodedClientMessage::Guidance(decoded)) if decoded == guidance
    ));

    let autopilot = AutopilotInput {
        tick: 9,
        command: AutopilotCommand::Cancel,
    };
    let autopilot_frame =
        split_wire_frame(thessa_flight_net::encode_autopilot(&autopilot).unwrap());
    assert!(matches!(
        decode_client_message(&autopilot_frame),
        Some(DecodedClientMessage::Autopilot(decoded)) if decoded == autopilot
    ));

    let hello = split_wire_frame(
        thessa_flight_net::encode_hello(&thessa_flight_net::Hello {
            client_name: "wrong direction".into(),
        })
        .unwrap(),
    );
    assert!(decode_client_message(&hello).is_none());
}

fn test_driver() -> (Driver, IngressSender) {
    let config: SystemConfig =
        toml::from_str(include_str!("../../../data/system.toml")).expect("system");
    let ephemeris = config.bake().expect("bake");
    let reference_body = ephemeris.body_id("thessa").expect("thessa");
    let sim = Sim::new(ephemeris, reference_body, false, true).expect("sim");
    let (upstream, receiver) = IngressSender::new();
    assert!(upstream.reserve_exact_connection("pilot".into()));
    assert!(upstream.mark_active_for_sender("pilot"));
    (
        Driver {
            sim,
            autopilot: AutopilotHost::new().expect("autopilot host"),
            upstream: receiver,
            subscribers: Vec::new(),
            pacing_target_s: 0.0,
            last_pacing: Instant::now(),
            last_snapshot: Instant::now(),
            last_status: Instant::now(),
            exit_when_empty: false,
        },
        upstream,
    )
}

fn install_resource_test_vehicle(sim: &mut Sim) {
    let engine = LiquidEngineSpec {
        name: "server-resource-engine".into(),
        propellant: Propellant::LoxRp1,
        cycle: EngineCycle::GasGenerator,
        chamber_pressure_pa: 9.7e6,
        throat_radius_m: 0.08,
        expansion_ratio: 18.0,
        nozzle_length_m: 0.9,
        contour: NozzleContour::Bell,
        chamber_material: ChamberMaterial::nickel_superalloy(),
        cooling: CoolingMode::Regenerative,
        mixture_ratio: None,
        characteristic_length_m: None,
        gimbal_range_rad: 0.0,
        min_throttle: None,
        restartable: true,
    }
    .compile()
    .expect("compile server test engine");
    let mount = EngineMount {
        name: "main-engine".into(),
        engine: thessa_sim_core::CompiledEngine::Liquid(engine),
        position_body_m: [0.0; 3],
        thrust_axis_body: [1.0, 0.0, 0.0],
    };
    let shape = TankShape::Sphere { diameter_m: 1.0 };
    let compiled_tank = TankSpec {
        shape,
        pressure_pa: 500_000.0,
        material: ChamberMaterial::nickel_superalloy(),
    }
    .compile(800.0)
    .expect("compile server test tank");
    let tank = TankMount {
        name: "main-tank".into(),
        tank: compiled_tank,
        position_body_m: DVec3::X.to_array(),
        intrinsic_inertia_body_kg_m2: shape
            .intrinsic_inertia_body_kg_m2(
                compiled_tank.dry_mass_kg,
                compiled_tank.full_propellant_kg,
            )
            .expect("test tank inertia"),
        initial_propellant_kg: Some(compiled_tank.full_propellant_kg),
        resource: TankResource::Pair(Propellant::LoxRp1),
    };
    let mut vehicle = sim
        .authority
        .vehicle
        .clone()
        .with_engines(vec![mount])
        .expect("install test engine")
        .with_tanks(vec![tank])
        .expect("install test tank");
    vehicle.bake_engine_masses().expect("bake test engine");
    vehicle.bake_tank_masses().expect("bake test tank");
    let reference_body = sim.authority.reference_body;
    sim.authority = FlightAuthority::new_with_vehicle(&sim.ephemeris, reference_body, vehicle)
        .expect("create resource test authority");
}

fn timed_wait_graph(seconds: f64) -> AutopilotGraph {
    AutopilotGraph {
        nodes: vec![GraphNode {
            id: thessa_autopilot::NodeId(1),
            name: "timed-wait".into(),
            kind: NodeKind::Wait,
            ports: Vec::new(),
            config: Some(GraphNodeConfig::Wait {
                condition: WaitCondition::At(SimTime(seconds)),
            }),
        }],
        edges: Vec::new(),
    }
}

fn event_timed_wait_graph(seconds: f64, event: &str) -> AutopilotGraph {
    AutopilotGraph {
        nodes: vec![GraphNode {
            id: thessa_autopilot::NodeId(1),
            name: "event-timed-wait".into(),
            kind: NodeKind::Wait,
            ports: Vec::new(),
            config: Some(GraphNodeConfig::Wait {
                condition: WaitCondition::All(vec![
                    WaitCondition::At(SimTime(seconds)),
                    WaitCondition::Event(event.into()),
                ]),
            }),
        }],
        edges: Vec::new(),
    }
}

#[test]
fn maneuver_execution_flies_plan_to_completion() {
    use thessa_maneuver::ManeuverNode;

    // Control-subtracted Dv measured AT CUTOFF: the same flight with
    // and without execution, sampled at the same sim time, so orbital
    // dynamics cancel and the burn remains. Raw inertial Dvx is
    // meaningless (orbital rotation adds hundreds of m/s per minute);
    // engine spool-down tail after handoff is vehicle physics,
    // explicitly out of executor scope.
    fn fly(with_execution: bool, until_s: Option<f64>) -> (f64, bool, f64, bool, f64) {
        let config: SystemConfig =
            toml::from_str(include_str!("../../../data/system.toml")).expect("system");
        let ephemeris = config.bake().expect("bake");
        let reference_body = ephemeris.body_id("thessa").expect("thessa");
        // Drift mode (declared vacuum): no aerodynamic forces at all,
        // so attitude differences between the runs cannot leak drag
        // into the Δv metric. The engine auto-arms on first throttle
        // via the guidance path; cold rails bakes stall chunks until
        // the worker serves them (bounded retries below).
        let mut sim = Sim::new(ephemeris, reference_body, false, true).expect("sim");
        sim.register("pilot");
        // One 100 m/s node along +X inertial at t=10 s (settle window
        // covers the initial slew). Chunk size matters: the executor
        // polls once per chunk, so chunks must resolve the burn (tens
        // of ms here) — production driver quanta (20 ms) do; 1 s chunks
        // would overshoot any small burn by a full chunk of thrust.
        let plan = ManeuverPlan::new(
            vec![ManeuverNode::new(SimTime(40.0), glam::DVec3::new(100.0, 0.0, 0.0)).unwrap()],
            sim.authority.state.position_inertial_m,
            sim.authority.state.velocity_inertial_mps,
            SimTime(0.0),
        )
        .unwrap();
        if with_execution {
            sim.start_maneuver_execution(plan).expect("starts");
            assert!(!sim.authority.scheduler.is_empty());
        }
        let initial_vx = sim.authority.state.velocity_inertial_mps.x;
        let mut saw_burn = false;
        let mut max_thrust = 0.0_f64;
        // 70 sim-seconds in 0.05 s chunks: settle, burn, cutoff, latch.
        // Stalled chunks (cold rails bake) retry briefly instead of
        // silently contributing zero time.
        for _ in 0..1400 {
            let mut advanced = 0.0;
            for _ in 0..200 {
                advanced += sim.advance_chunk(0.05).expect("advance");
                if advanced > 0.0
                    || sim.authority.flight_error.is_some()
                    || !sim.authority.bake.has_pending()
                {
                    break;
                }
                std::thread::sleep(std::time::Duration::from_millis(5));
            }
            let t = sim.authority.flight_time_s;
            if let Some((_, propulsion)) = &sim.guidance
                && propulsion.normalized > 0.5
            {
                saw_burn = true;
            }
            max_thrust = max_thrust.max(sim.authority.thrust_n());
            if with_execution && sim.maneuver_execution.is_none() {
                break;
            }
            if let Some(until) = until_s
                && t >= until
            {
                break;
            }
        }
        (
            sim.authority.state.velocity_inertial_mps.x - initial_vx,
            saw_burn,
            max_thrust,
            with_execution && sim.maneuver_execution.is_none(),
            sim.authority.flight_time_s,
        )
    }
    // Exec run first (finds the cutoff time), then the control run
    // sampled at the same sim time.
    let (exec_dvx, saw_burn, max_thrust, done, t_done) = fly(true, None);
    assert!(done, "execution must complete and hand off");
    assert!(
        saw_burn,
        "executor must command full throttle during the burn"
    );
    assert!(max_thrust > 0.0, "engine must produce thrust");
    let (ctrl_dvx, _, _, _, _) = fly(false, Some(t_done));
    let burn_dvx = exec_dvx - ctrl_dvx;
    assert!(
        (85.0..=130.0).contains(&burn_dvx),
        "burn-attributed dvx {burn_dvx} for a 100 m/s node"
    );
}

#[test]
fn maneuver_execution_rejects_bad_plans() {
    use thessa_maneuver::ManeuverNode;

    let config: SystemConfig =
        toml::from_str(include_str!("../../../data/system.toml")).expect("system");
    let ephemeris = config.bake().expect("bake");
    let reference_body = ephemeris.body_id("thessa").expect("thessa");
    let mut sim = Sim::new(ephemeris, reference_body, false, false).expect("sim");
    sim.register("pilot");
    let empty = ManeuverPlan::new(vec![], glam::DVec3::ZERO, glam::DVec3::X, SimTime(0.0)).unwrap();
    assert!(sim.start_maneuver_execution(empty).is_err());
    let stale = ManeuverPlan::new(
        vec![ManeuverNode::new(SimTime(5.0), glam::DVec3::X).unwrap()],
        glam::DVec3::ZERO,
        glam::DVec3::X,
        SimTime(0.0),
    )
    .unwrap();
    for _ in 0..10 {
        sim.advance_chunk(1.0).expect("advance");
    }
    assert!(sim.start_maneuver_execution(stale).is_err());
    assert!(sim.maneuver_execution.is_none());
}

#[test]
fn part_command_reaches_authoritative_runtime() {
    let (mut driver, _) = test_driver();
    driver.sim.register("pilot");
    driver.sim.authority.rcs_enabled = false;
    let command = Command::Part {
        command: thessa_sim_core::VehiclePartCommand::SetRcsEnabled { enabled: true },
    };

    assert!(driver.sim.apply_input("pilot", &input(vec![command])));
    assert!(driver.sim.authority.rcs_enabled);
    // The next full-state packet still echoes the old legacy field. It
    // must not undo the discrete command unless the client changed it.
    driver.sim.apply_input("pilot", &input(Vec::new()));
    assert!(driver.sim.authority.rcs_enabled);
}

#[test]
fn engine_throttle_part_command_survives_unchanged_legacy_input() {
    let (mut driver, _) = test_driver();
    driver.sim.register("pilot");
    install_resource_test_vehicle(&mut driver.sim);

    let command = Command::Part {
        command: VehiclePartCommand::SetEngineThrottle {
            name: "main-engine".into(),
            throttle: 0.5,
        },
    };
    assert!(driver.sim.apply_input("pilot", &input(vec![command])));
    // A normal full-state packet repeats the same legacy throttle fields but
    // carries no engine command. It must not erase the named throttle.
    driver.sim.apply_input("pilot", &input(Vec::new()));
    driver
        .sim
        .advance_chunk(1.0 / 120.0)
        .expect("advance commanded engine");

    assert!(driver.sim.authority.last_propulsion_force_body_n.x > 0.0);
    assert!(driver.sim.authority.last_propellant_flow_kg_s > 0.0);
}

#[test]
fn part_command_cancels_a_waiting_autopilot_script() {
    let (mut driver, _) = test_driver();
    driver.sim.register("pilot");
    let neutral = input(Vec::new());
    driver.apply_client_input("pilot", &neutral);
    assert!(driver.sim.apply_autopilot(
        "pilot",
        &AutopilotInput {
            tick: 0,
            command: AutopilotCommand::StartScript {
                source: "await sim.sleep(1); return Guidance.angularRate(0.1, 0, 0);".into(),
            },
        },
        &mut driver.autopilot,
    ));
    assert_eq!(driver.autopilot.scheduler.pending(), 1);

    let part_input = input(vec![Command::Part {
        command: thessa_sim_core::VehiclePartCommand::SetRcsEnabled { enabled: true },
    }]);
    assert!(driver.apply_client_input("pilot", &part_input));
    assert_eq!(driver.autopilot.scheduler.pending(), 0);
    assert!(driver.sim.authority.rcs_enabled);
}

#[test]
fn execute_maneuver_command_starts_and_rejects() {
    use thessa_flight_net::ManeuverNodeCommand;

    fn fresh_sim() -> Sim {
        let config: SystemConfig =
            toml::from_str(include_str!("../../../data/system.toml")).expect("system");
        let ephemeris = config.bake().expect("bake");
        let reference_body = ephemeris.body_id("thessa").expect("thessa");
        let mut sim = Sim::new(ephemeris, reference_body, false, false).expect("sim");
        sim.register("pilot");
        sim
    }
    // Valid command starts execution and arms the wake.
    let mut sim = fresh_sim();
    let command = Command::ExecuteManeuver {
        nodes: vec![ManeuverNodeCommand {
            epoch_s: 30.0,
            delta_v_mps: [10.0, 0.0, 0.0],
        }],
    };
    assert!(sim.apply_input("pilot", &input(vec![command])));
    assert!(sim.maneuver_execution.is_some());
    assert!(!sim.authority.scheduler.is_empty());
    // Oversize is refused with a wake notice, nothing starts.
    // (apply_input returns its snapshot flag, not acceptance.)
    let mut sim = fresh_sim();
    let big = Command::ExecuteManeuver {
        nodes: vec![
            ManeuverNodeCommand {
                epoch_s: 30.0,
                delta_v_mps: [1.0, 0.0, 0.0],
            };
            17
        ],
    };
    let mut malformed = input(vec![big]);
    malformed.control_input = [0.5, 0.0, 0.0];
    malformed.throttle = 0.75;
    let controls_before = (sim.authority.control_input, sim.authority.throttle);
    assert!(!sim.apply_input("pilot", &malformed));
    assert!(sim.maneuver_execution.is_none());
    assert_eq!(
        (sim.authority.control_input, sim.authority.throttle),
        controls_before
    );
    assert!(sim.last_client_inputs.is_empty());
    // Non-finite node is refused the same way.
    let mut sim = fresh_sim();
    let bad = Command::ExecuteManeuver {
        nodes: vec![ManeuverNodeCommand {
            epoch_s: f64::NAN,
            delta_v_mps: [1.0, 0.0, 0.0],
        }],
    };
    let mut malformed = input(vec![bad]);
    malformed.control_input = [0.5, 0.0, 0.0];
    malformed.throttle = 0.75;
    let controls_before = (sim.authority.control_input, sim.authority.throttle);
    assert!(!sim.apply_input("pilot", &malformed));
    assert!(sim.maneuver_execution.is_none());
    assert_eq!(
        (sim.authority.control_input, sim.authority.throttle),
        controls_before
    );
    assert!(sim.last_client_inputs.is_empty());
}

#[test]
fn burn_execution_flies_plan_to_completion() {
    use thessa_maneuver::{BurnSegment, EngineSpec, FiniteBurnPlan, SegmentDirection};

    // Same control-subtracted harness as the node test: exec run finds
    // the cutoff time, the control run sampled there cancels orbital
    // dynamics, the burn remains.
    fn fly(with_execution: bool, until_s: Option<f64>) -> (f64, bool, f64, bool, f64) {
        let config: SystemConfig =
            toml::from_str(include_str!("../../../data/system.toml")).expect("system");
        let ephemeris = config.bake().expect("bake");
        let reference_body = ephemeris.body_id("thessa").expect("thessa");
        let mut sim = Sim::new(ephemeris, reference_body, false, true).expect("sim");
        sim.register("pilot");
        // Two 5 s full-throttle +X segments at t=40/60 s.
        let engine = EngineSpec {
            thrust_n: 100_000.0,
            exhaust_velocity_mps: 4_400.0,
        };
        let segment = |start: f64| BurnSegment {
            start: SimTime(start),
            duration_s: 5.0,
            planned_dv_mps: 50.0,
            direction: SegmentDirection::Inertial(glam::DVec3::X),
            throttle_01: 1.0,
        };
        let plan = FiniteBurnPlan::new(
            vec![segment(40.0), segment(60.0)],
            engine,
            20_000.0,
            sim.authority.state.position_inertial_m,
            sim.authority.state.velocity_inertial_mps,
            SimTime(0.0),
        )
        .unwrap();
        if with_execution {
            sim.start_burn_execution(plan).expect("starts");
            assert!(!sim.authority.scheduler.is_empty());
        }
        let initial_vx = sim.authority.state.velocity_inertial_mps.x;
        let mut saw_burn = false;
        let mut max_thrust = 0.0_f64;
        for _ in 0..1400 {
            let mut advanced = 0.0;
            for _ in 0..200 {
                advanced += sim.advance_chunk(0.05).expect("advance");
                if advanced > 0.0
                    || sim.authority.flight_error.is_some()
                    || !sim.authority.bake.has_pending()
                {
                    break;
                }
                std::thread::sleep(std::time::Duration::from_millis(5));
            }
            if let Some((_, propulsion)) = &sim.guidance
                && propulsion.normalized > 0.5
            {
                saw_burn = true;
            }
            max_thrust = max_thrust.max(sim.authority.thrust_n());
            if with_execution && sim.burn_execution.is_none() {
                break;
            }
            if let Some(until) = until_s
                && sim.authority.flight_time_s >= until
            {
                break;
            }
        }
        (
            sim.authority.state.velocity_inertial_mps.x - initial_vx,
            saw_burn,
            max_thrust,
            with_execution && sim.burn_execution.is_none(),
            sim.authority.flight_time_s,
        )
    }
    let (exec_dvx, saw_burn, max_thrust, done, t_done) = fly(true, None);
    assert!(done, "burn execution must complete and hand off");
    assert!(saw_burn, "executor must command throttle during arcs");
    assert!(max_thrust > 0.0, "engine must produce thrust");
    let (ctrl_dvx, _, _, _, _) = fly(false, Some(t_done));
    let burn_dvx = exec_dvx - ctrl_dvx;
    assert!(
        burn_dvx > 0.5,
        "burn-attributed dvx {burn_dvx} for two 5 s arcs"
    );
}

#[test]
fn burn_execution_rejects_bad_plans() {
    use thessa_maneuver::ManeuverNode;
    use thessa_maneuver::{BurnSegment, EngineSpec, FiniteBurnPlan, SegmentDirection};

    let config: SystemConfig =
        toml::from_str(include_str!("../../../data/system.toml")).expect("system");
    let ephemeris = config.bake().expect("bake");
    let reference_body = ephemeris.body_id("thessa").expect("thessa");
    let mut sim = Sim::new(ephemeris, reference_body, false, false).expect("sim");
    sim.register("pilot");
    let engine = EngineSpec {
        thrust_n: 100_000.0,
        exhaust_velocity_mps: 4_400.0,
    };
    let segment = BurnSegment {
        start: SimTime(40.0),
        duration_s: 5.0,
        planned_dv_mps: 50.0,
        direction: SegmentDirection::Inertial(glam::DVec3::X),
        throttle_01: 1.0,
    };
    let empty = FiniteBurnPlan::new(
        vec![],
        engine,
        20_000.0,
        sim.authority.state.position_inertial_m,
        sim.authority.state.velocity_inertial_mps,
        SimTime(0.0),
    )
    .unwrap();
    assert!(sim.start_burn_execution(empty).is_err());
    let stale = FiniteBurnPlan::new(
        vec![BurnSegment {
            start: SimTime(5.0),
            ..segment
        }],
        engine,
        20_000.0,
        sim.authority.state.position_inertial_m,
        sim.authority.state.velocity_inertial_mps,
        SimTime(10.0),
    )
    .unwrap();
    // Flight clock starts at 0: a plan starting at t=5 is... fresh
    // here; force staleness by advancing the clock past the segment.
    sim.authority.flight_time_s = 50.0;
    assert!(sim.start_burn_execution(stale).is_err());
    assert!(sim.burn_execution.is_none());
    // Mutual exclusion with node execution (both directions).
    sim.authority.flight_time_s = 0.0;
    let live = FiniteBurnPlan::new(
        vec![segment],
        engine,
        20_000.0,
        sim.authority.state.position_inertial_m,
        sim.authority.state.velocity_inertial_mps,
        SimTime(0.0),
    )
    .unwrap();
    sim.start_burn_execution(live).expect("burn starts");
    let node_plan = ManeuverPlan::new(
        vec![ManeuverNode::new(SimTime(40.0), glam::DVec3::X).unwrap()],
        sim.authority.state.position_inertial_m,
        sim.authority.state.velocity_inertial_mps,
        SimTime(0.0),
    )
    .unwrap();
    assert!(sim.start_maneuver_execution(node_plan).is_err());
    sim.clear_autopilot_controls();
    assert!(sim.burn_execution.is_none());
}

#[test]
fn execute_burn_plan_command_starts_and_rejects() {
    use thessa_flight_net::{BurnDirectionCommand, BurnSegmentCommand};

    fn fresh_sim() -> Sim {
        let config: SystemConfig =
            toml::from_str(include_str!("../../../data/system.toml")).expect("system");
        let ephemeris = config.bake().expect("bake");
        let reference_body = ephemeris.body_id("thessa").expect("thessa");
        let mut sim = Sim::new(ephemeris, reference_body, false, false).expect("sim");
        sim.register("pilot");
        sim
    }
    fn segment(start_s: f64) -> BurnSegmentCommand {
        BurnSegmentCommand {
            start_s,
            duration_s: 5.0,
            planned_dv_mps: 50.0,
            direction: BurnDirectionCommand::Inertial {
                unit: [1.0, 0.0, 0.0],
            },
            throttle_01: 1.0,
        }
    }
    // Valid command (inertial + RTN segments) starts execution.
    let mut sim = fresh_sim();
    let command = Command::ExecuteBurnPlan {
        engine_thrust_n: 100_000.0,
        engine_exhaust_velocity_mps: 4_400.0,
        initial_mass_kg: 20_000.0,
        segments: vec![
            segment(30.0),
            BurnSegmentCommand {
                direction: BurnDirectionCommand::Rtn {
                    central: "thessa".into(),
                    radial: 0.0,
                    transverse: 1.0,
                    normal: 0.0,
                },
                ..segment(60.0)
            },
        ],
    };
    assert!(sim.apply_input("pilot", &input(vec![command])));
    assert!(sim.burn_execution.is_some());
    assert!(!sim.authority.scheduler.is_empty());
    // Oversize is refused with a wake notice, nothing starts.
    let mut sim = fresh_sim();
    let big = Command::ExecuteBurnPlan {
        engine_thrust_n: 100_000.0,
        engine_exhaust_velocity_mps: 4_400.0,
        initial_mass_kg: 20_000.0,
        segments: vec![segment(30.0); 65],
    };
    assert!(!sim.apply_input("pilot", &input(vec![big])));
    assert!(sim.burn_execution.is_none());
    assert!(sim.last_client_inputs.is_empty());
    // Unknown RTN central and dead engine are refused the same way.
    let mut sim = fresh_sim();
    let lost = Command::ExecuteBurnPlan {
        engine_thrust_n: 100_000.0,
        engine_exhaust_velocity_mps: 4_400.0,
        initial_mass_kg: 20_000.0,
        segments: vec![BurnSegmentCommand {
            direction: BurnDirectionCommand::Rtn {
                central: "nope".into(),
                radial: 0.0,
                transverse: 1.0,
                normal: 0.0,
            },
            ..segment(30.0)
        }],
    };
    let _ = sim.apply_input("pilot", &input(vec![lost]));
    assert!(sim.burn_execution.is_none());
    assert!(sim.authority.wake_notice.is_some());
    let mut sim = fresh_sim();
    let dead = Command::ExecuteBurnPlan {
        engine_thrust_n: 0.0,
        engine_exhaust_velocity_mps: 4_400.0,
        initial_mass_kg: 20_000.0,
        segments: vec![segment(30.0)],
    };
    assert!(!sim.apply_input("pilot", &input(vec![dead])));
    assert!(sim.burn_execution.is_none());
    assert!(sim.last_client_inputs.is_empty());
}

#[test]
fn typed_guidance_reaches_the_authoritative_stepper() {
    let config: SystemConfig =
        toml::from_str(include_str!("../../../data/system.toml")).expect("system");
    let ephemeris = config.bake().expect("bake");
    let reference_body = ephemeris.body_id("thessa").expect("thessa");
    let mut sim = Sim::new(ephemeris, reference_body, false, true).expect("sim");
    sim.register("pilot");
    let guidance = GuidanceInput {
        tick: 0,
        intent: GuidanceIntent::Attitude {
            target_body_to_inertial: DQuat::from_rotation_y(0.05),
            roll_policy: thessa_flight_authority::RollPolicy::Hold,
        },
        propulsion: PropulsionDemand::new(0.0).unwrap(),
    };
    assert!(sim.apply_guidance("pilot", &guidance));
    sim.advance_chunk(0.02).expect("advance typed guidance");
    assert_eq!(sim.control_mode, ControlMode::Navball);
    assert!(sim.authority.state.position_inertial_m.is_finite());
}

#[test]
fn server_owns_script_waits_and_wakes_them_on_sim_time() {
    let config: SystemConfig =
        toml::from_str(include_str!("../../../data/system.toml")).expect("system");
    let ephemeris = config.bake().expect("bake");
    let reference_body = ephemeris.body_id("thessa").expect("thessa");
    let mut sim = Sim::new(ephemeris, reference_body, false, true).expect("sim");
    let mut host = AutopilotHost::new().expect("autopilot host");
    sim.register("pilot");

    let input = AutopilotInput {
        tick: 0,
        command: AutopilotCommand::StartScript {
            source: "await sim.sleep(0.05); return Guidance.angularRate(0.1, 0, 0);".into(),
        },
    };
    assert!(sim.apply_autopilot("pilot", &input, &mut host));
    assert_eq!(host.scheduler.pending(), 1);
    assert!(sim.guidance.is_none());
    assert_eq!(host.scheduler.next_time(), Some(SimTime(0.05)));

    sim.authority.flight_time_s = 0.05;
    assert!(sim.wake_autopilot(&mut host, &[]).expect("wake script"));
    assert!(matches!(
        sim.guidance,
        Some((
            GuidanceIntent::AngularRate { .. },
            PropulsionDemand { normalized: 0.0 }
        ))
    ));
    assert_eq!(sim.control_mode, ControlMode::Rate);
}

#[test]
fn neutral_input_and_warp_votes_do_not_cancel_a_waiting_script() {
    let (mut driver, _) = test_driver();
    driver.sim.register("pilot");
    let neutral = input(Vec::new());
    driver.apply_client_input("pilot", &neutral);
    assert!(driver.sim.apply_autopilot(
        "pilot",
        &AutopilotInput {
            tick: 0,
            command: AutopilotCommand::StartScript {
                source: "await sim.sleep(1); return Guidance.angularRate(0.1, 0, 0);".into(),
            },
        },
        &mut driver.autopilot,
    ));
    assert_eq!(driver.autopilot.scheduler.pending(), 1);

    driver.apply_client_input("pilot", &neutral);
    driver.apply_client_input("pilot", &input(vec![Command::SetWarp { factor: 128.0 }]));
    assert_eq!(driver.autopilot.scheduler.pending(), 1);

    driver.sim.plan_demand = Some(ControlDemand {
        force_body_n: DVec3::X * 100.0,
        moment_body_nm: DVec3::Y * 50.0,
        propulsion: PropulsionDemand::new(0.8).unwrap(),
    });
    driver.sim.guidance = Some((
        GuidanceIntent::AngularRate {
            rate_body_rps: DVec3::X,
        },
        PropulsionDemand::new(0.8).unwrap(),
    ));
    driver
        .sim
        .authority
        .set_propulsion_target(PropulsionDemand::new(0.8).unwrap())
        .unwrap();
    let mut manual = neutral;
    manual.control_input = [0.25, 0.0, 0.0];
    driver.apply_client_input("pilot", &manual);
    assert_eq!(driver.autopilot.scheduler.pending(), 0);
    assert!(driver.sim.plan_demand.is_none());
    assert!(driver.sim.guidance.is_none());
    assert_eq!(driver.sim.authority.thrust_n(), 0.0);
}

#[test]
fn cancel_and_graph_submit_drop_old_script_continuations() {
    let (mut driver, _) = test_driver();
    driver.sim.register("pilot");
    let start = AutopilotInput {
        tick: 0,
        command: AutopilotCommand::StartScript {
            source: "await sim.sleep(1); return Guidance.angularRate(0.1, 0, 0);".into(),
        },
    };
    assert!(
        driver
            .sim
            .apply_autopilot("pilot", &start, &mut driver.autopilot)
    );
    assert_eq!(driver.autopilot.scheduler.pending(), 1);
    assert!(driver.sim.apply_autopilot(
        "pilot",
        &AutopilotInput {
            tick: 1,
            command: AutopilotCommand::Cancel,
        },
        &mut driver.autopilot,
    ));
    assert_eq!(driver.autopilot.scheduler.pending(), 0);
    driver.sim.authority.flight_time_s = 2.0;
    assert!(
        driver
            .autopilot
            .scheduler
            .wake(&driver.autopilot.engine, SimTime(2.0), None)
            .unwrap()
            .is_empty()
    );
    assert!(driver.sim.guidance.is_none());

    assert!(
        driver
            .sim
            .apply_autopilot("pilot", &start, &mut driver.autopilot)
    );
    assert_eq!(driver.autopilot.scheduler.pending(), 1);
    assert!(driver.sim.apply_autopilot(
        "pilot",
        &AutopilotInput {
            tick: 2,
            command: AutopilotCommand::SubmitGraph {
                graph: timed_wait_graph(10.0),
            },
        },
        &mut driver.autopilot,
    ));
    assert_eq!(driver.autopilot.scheduler.pending(), 0);
    assert!(driver.sim.graph_runner.is_some());
    driver.sim.authority.flight_time_s = 3.0;
    assert!(
        driver
            .autopilot
            .scheduler
            .wake(&driver.autopilot.engine, SimTime(3.0), None)
            .unwrap()
            .is_empty()
    );
    assert!(driver.sim.guidance.is_none());
}

#[test]
fn graph_time_wake_clips_high_warp_before_bulk_stepping() {
    let (mut driver, _) = test_driver();
    driver.sim.register("pilot");
    assert!(driver.sim.apply_autopilot(
        "pilot",
        &AutopilotInput {
            tick: 0,
            command: AutopilotCommand::SubmitGraph {
                graph: event_timed_wait_graph(0.05, "impact"),
            },
        },
        &mut driver.autopilot,
    ));
    assert!(driver.sim.next_autopilot_wake(&driver.autopilot).is_none());
    driver
        .sim
        .wake_autopilot(&mut driver.autopilot, &[AutopilotEvent::Impact])
        .expect("remember graph event");
    let wake = driver
        .sim
        .next_autopilot_wake(&driver.autopilot)
        .expect("graph time wake");
    let tick_s = thessa_sim_core::WORLD_TICK_S;
    let clipped = clip_pacing_chunk_to_wake(
        100.0,
        driver.sim.authority.flight_time_s,
        driver.sim.authority.backlog_s(),
        wake,
        tick_s,
    );
    assert!(clipped < 100.0);
    assert!(driver.sim.authority.flight_time_s + clipped <= 0.05 + tick_s);
}

#[test]
fn server_stores_data_only_landing_and_impact_site_declarations() {
    let config: SystemConfig =
        toml::from_str(include_str!("../../../data/system.toml")).expect("system");
    let ephemeris = config.bake().expect("bake");
    let reference_body = ephemeris.body_id("thessa").expect("thessa");
    let mut sim = Sim::new(ephemeris, reference_body, false, true).expect("sim");
    let mut host = AutopilotHost::new().expect("autopilot host");
    sim.register("pilot");

    assert!(sim.apply_autopilot(
        "pilot",
        &AutopilotInput {
            tick: 0,
            command: AutopilotCommand::StartScript {
                source: "return Landing.site(0, 2, 0, 300);".into(),
            },
        },
        &mut host,
    ));
    assert_eq!(
        sim.landing_site,
        Some(LandingSite {
            center_dir: [0.0, 1.0, 0.0],
            radius_m: 300.0,
        })
    );
    assert!(sim.landing_obstacles.is_none());
    assert!(sim.plan_runner.is_none());

    assert!(sim.apply_autopilot(
        "pilot",
        &AutopilotInput {
            tick: 1,
            command: AutopilotCommand::StartScript {
                source: "return Impact.site(1, 0, 0, 50);".into(),
            },
        },
        &mut host,
    ));
    assert_eq!(
        sim.impact_site,
        Some(ImpactSite {
            center_dir: [1.0, 0.0, 0.0],
            radius_m: 50.0,
        })
    );
    assert!(sim.impact_obstacles.is_none());
    assert!(sim.authority.wake_notice.as_deref().is_some_and(|notice| {
        notice.contains("IMPACT SITE") && notice.contains("radius=50.0m")
    }));
}

#[test]
fn server_executes_and_deoptimizes_a_submitted_plan() {
    let config: SystemConfig =
        toml::from_str(include_str!("../../../data/system.toml")).expect("system");
    let ephemeris = config.bake().expect("bake");
    let reference_body = ephemeris.body_id("thessa").expect("thessa");
    let mut sim = Sim::new(ephemeris, reference_body, false, true).expect("sim");
    let mut host = AutopilotHost::new().expect("autopilot host");
    sim.register("pilot");
    let plan = TrajectoryPlan {
        id: thessa_flight_authority::TrajectoryPlanId(33),
        segments: vec![
            thessa_autopilot::TrajectorySegment::Coast { duration_s: 0.02 },
            thessa_autopilot::TrajectorySegment::Burn {
                duration_s: 0.1,
                demand: ControlDemand {
                    force_body_n: DVec3::Y * 100.0,
                    moment_body_nm: DVec3::X * 100.0,
                    propulsion: PropulsionDemand::new(0.0).unwrap(),
                },
            },
            thessa_autopilot::TrajectorySegment::Guidance {
                duration_s: 0.1,
                intent: GuidanceIntent::ManualAxes(Default::default()),
            },
        ],
        bakeability: thessa_autopilot::Bakeability::Guarded,
    };
    let input = AutopilotInput {
        tick: 0,
        command: AutopilotCommand::SubmitPlan { plan },
    };
    assert!(sim.apply_autopilot("pilot", &input, &mut host));
    assert!(sim.plan_runner.is_some());
    assert!(!sim.authority.engine_active);

    sim.authority.flight_time_s = 0.02;
    sim.poll_plan(None).expect("advance plan cursor");
    assert_eq!(sim.plan_runner.as_ref().unwrap().segment_index(), 1);
    assert!(sim.plan_runner.as_ref().unwrap().mode() == PlanExecutionMode::Baked);
    sim.advance_chunk(thessa_sim_core::WORLD_TICK_S)
        .expect("execute explicit burn");
    assert!(sim.authority.flight_error.is_none());
    assert!(sim.authority.state.angular_velocity_body_rps.x > 0.0);
    // Burn demand routes through the bounded least-squares RCS allocator,
    // so the sampled force is the achieved wrench within solver tolerance,
    // not the verbatim demand (allocator convention: 1e-6 N).
    let sampled_force_y = sim
        .authority
        .last_forces
        .as_ref()
        .expect("explicit force sample")
        .total_force_body_n
        .y;
    assert!(
        (sampled_force_y - 100.0).abs() < 1.0e-6,
        "achieved burn force {sampled_force_y} outside allocator tolerance of 100.0"
    );
    assert!(sim.authority.state.angular_velocity_body_rps.is_finite());

    sim.authority.flight_time_s = 0.13;
    sim.poll_plan(None).expect("advance to guidance cursor");
    assert_eq!(sim.plan_runner.as_ref().unwrap().segment_index(), 2);
    assert!(sim.plan_demand.is_none());
    assert!(sim.guidance.is_some());

    let deoptimize = AutopilotInput {
        tick: 1,
        command: AutopilotCommand::Deoptimize {
            reason: thessa_autopilot::PlanDeoptimizationReason::GuardInvalidated,
        },
    };
    assert!(sim.apply_autopilot("pilot", &deoptimize, &mut host));
    assert_eq!(
        sim.plan_runner.as_ref().unwrap().mode(),
        PlanExecutionMode::Live
    );
}

#[test]
fn server_wakes_a_plan_from_an_authoritative_event() {
    let config: SystemConfig =
        toml::from_str(include_str!("../../../data/system.toml")).expect("system");
    let ephemeris = config.bake().expect("bake");
    let reference_body = ephemeris.body_id("thessa").expect("thessa");
    let mut sim = Sim::new(ephemeris, reference_body, false, true).expect("sim");
    let mut host = AutopilotHost::new().expect("autopilot host");
    sim.register("pilot");
    let plan = TrajectoryPlan {
        id: thessa_flight_authority::TrajectoryPlanId(34),
        segments: vec![
            thessa_autopilot::TrajectorySegment::Wait {
                condition: thessa_autopilot::WaitCondition::Event("impact".into()),
            },
            thessa_autopilot::TrajectorySegment::Guidance {
                duration_s: 0.1,
                intent: GuidanceIntent::AngularRate {
                    rate_body_rps: DVec3::new(0.1, 0.0, 0.0),
                },
            },
        ],
        bakeability: thessa_autopilot::Bakeability::Live,
    };
    let input = AutopilotInput {
        tick: 0,
        command: AutopilotCommand::SubmitPlan { plan },
    };
    assert!(sim.apply_autopilot("pilot", &input, &mut host));
    assert_eq!(sim.plan_runner.as_ref().unwrap().segment_index(), 0);

    sim.autopilot_events.push_back(AutopilotEvent::Impact);
    let events = sim.take_autopilot_events();
    assert_eq!(events, vec![AutopilotEvent::Impact]);
    assert!(sim.poll_plan(Some("impact")).expect("wake plan"));
    assert_eq!(sim.plan_runner.as_ref().unwrap().segment_index(), 1);
    assert_eq!(sim.control_mode, ControlMode::Rate);
}

#[test]
fn typed_authority_wakes_preserve_order_and_duplicate_events() {
    let config: SystemConfig =
        toml::from_str(include_str!("../../../data/system.toml")).expect("system");
    let ephemeris = config.bake().expect("bake");
    let reference_body = ephemeris.body_id("thessa").expect("thessa");
    let mut sim = Sim::new(ephemeris, reference_body, false, false).expect("sim");
    sim.register("pilot");
    let tick_s = thessa_sim_core::WORLD_TICK_S;
    sim.authority
        .scheduler
        .arm(ScheduledKind::Alarm, SimTime(tick_s));
    sim.authority
        .scheduler
        .arm(ScheduledKind::Alarm, SimTime(tick_s));
    sim.advance_chunk(tick_s * 2.0).expect("advance alarms");
    assert_eq!(
        sim.take_autopilot_events(),
        vec![AutopilotEvent::Alarm, AutopilotEvent::Alarm]
    );
}

#[test]
fn server_validates_and_executes_graph_ir_on_submit() {
    let config: SystemConfig =
        toml::from_str(include_str!("../../../data/system.toml")).expect("system");
    let ephemeris = config.bake().expect("bake");
    let reference_body = ephemeris.body_id("thessa").expect("thessa");
    let mut sim = Sim::new(ephemeris, reference_body, false, true).expect("sim");
    let mut host = AutopilotHost::new().expect("autopilot host");
    sim.register("pilot");
    let graph = AutopilotGraph {
        nodes: vec![
            thessa_autopilot::GraphNode {
                id: thessa_autopilot::NodeId(1),
                name: "source".into(),
                kind: thessa_autopilot::NodeKind::Source,
                ports: vec![thessa_autopilot::Port::output(
                    "value",
                    thessa_autopilot::PortType::Number,
                )],
                config: None,
            },
            thessa_autopilot::GraphNode {
                id: thessa_autopilot::NodeId(2),
                name: "sink".into(),
                kind: thessa_autopilot::NodeKind::Sink,
                ports: vec![thessa_autopilot::Port::input(
                    "value",
                    thessa_autopilot::PortType::Number,
                    true,
                )],
                config: None,
            },
        ],
        edges: vec![thessa_autopilot::GraphEdge {
            from: thessa_autopilot::PortRef {
                node: thessa_autopilot::NodeId(1),
                port: "value".into(),
            },
            to: thessa_autopilot::PortRef {
                node: thessa_autopilot::NodeId(2),
                port: "value".into(),
            },
        }],
    };
    let input = AutopilotInput {
        tick: 0,
        command: AutopilotCommand::SubmitGraph {
            graph: graph.clone(),
        },
    };
    assert!(sim.apply_autopilot("pilot", &input, &mut host));
    assert_eq!(sim.autopilot_graph, Some(graph));
    assert!(
        sim.graph_runner
            .as_ref()
            .is_some_and(GraphRunner::is_complete)
    );
    assert!(
        sim.authority
            .wake_notice
            .as_deref()
            .is_some_and(|notice| notice.starts_with("AUTOPILOT GRAPH READY"))
    );
}

#[test]
fn server_applies_configured_graph_guidance_through_authority() {
    let config: SystemConfig =
        toml::from_str(include_str!("../../../data/system.toml")).expect("system");
    let ephemeris = config.bake().expect("bake");
    let reference_body = ephemeris.body_id("thessa").expect("thessa");
    let mut sim = Sim::new(ephemeris, reference_body, false, true).expect("sim");
    let mut host = AutopilotHost::new().expect("autopilot host");
    sim.register("pilot");
    let graph = AutopilotGraph {
        nodes: vec![thessa_autopilot::GraphNode {
            id: thessa_autopilot::NodeId(1),
            name: "rate-controller".into(),
            kind: thessa_autopilot::NodeKind::Controller {
                actuator_groups: vec![thessa_flight_control::ActuatorGroup::Rcs],
            },
            ports: vec![thessa_autopilot::Port::output(
                "target",
                thessa_autopilot::PortType::AngularRateTarget,
            )],
            config: Some(GraphNodeConfig::Guidance {
                intent: GuidanceIntent::AngularRate {
                    rate_body_rps: DVec3::new(0.1, 0.0, 0.0),
                },
                propulsion: PropulsionDemand::new(0.0).unwrap(),
            }),
        }],
        edges: Vec::new(),
    };
    assert!(sim.apply_autopilot(
        "pilot",
        &AutopilotInput {
            tick: 0,
            command: AutopilotCommand::SubmitGraph { graph },
        },
        &mut host,
    ));
    assert_eq!(sim.control_mode, ControlMode::Rate);
    assert!(matches!(
        sim.guidance,
        Some((
            GuidanceIntent::AngularRate { .. },
            PropulsionDemand { normalized: 0.0 }
        ))
    ));
    assert!(
        sim.graph_runner
            .as_ref()
            .is_some_and(GraphRunner::is_complete)
    );
}

#[test]
fn server_wakes_a_native_graph_wait_from_an_authoritative_event() {
    let config: SystemConfig =
        toml::from_str(include_str!("../../../data/system.toml")).expect("system");
    let ephemeris = config.bake().expect("bake");
    let reference_body = ephemeris.body_id("thessa").expect("thessa");
    let mut sim = Sim::new(ephemeris, reference_body, false, true).expect("sim");
    let mut host = AutopilotHost::new().expect("autopilot host");
    sim.register("pilot");
    let graph = AutopilotGraph {
        nodes: vec![
            thessa_autopilot::GraphNode {
                id: thessa_autopilot::NodeId(1),
                name: "impact".into(),
                kind: thessa_autopilot::NodeKind::Wait,
                ports: vec![thessa_autopilot::Port::output(
                    "done",
                    thessa_autopilot::PortType::Unit,
                )],
                config: None,
            },
            thessa_autopilot::GraphNode {
                id: thessa_autopilot::NodeId(2),
                name: "sink".into(),
                kind: thessa_autopilot::NodeKind::Sink,
                ports: vec![thessa_autopilot::Port::input(
                    "done",
                    thessa_autopilot::PortType::Unit,
                    true,
                )],
                config: None,
            },
        ],
        edges: vec![thessa_autopilot::GraphEdge {
            from: thessa_autopilot::PortRef {
                node: thessa_autopilot::NodeId(1),
                port: "done".into(),
            },
            to: thessa_autopilot::PortRef {
                node: thessa_autopilot::NodeId(2),
                port: "done".into(),
            },
        }],
    };
    assert!(sim.apply_autopilot(
        "pilot",
        &AutopilotInput {
            tick: 0,
            command: AutopilotCommand::SubmitGraph { graph },
        },
        &mut host,
    ));
    assert!(!sim.graph_runner.as_ref().unwrap().is_complete());

    sim.autopilot_events.push_back(AutopilotEvent::Impact);
    let events = sim.take_autopilot_events();
    assert_eq!(events, vec![AutopilotEvent::Impact]);
    sim.wake_autopilot(&mut host, &events).expect("wake graph");
    assert!(sim.graph_runner.as_ref().unwrap().is_complete());

    sim.authority.wake_notice = Some("AUTOPILOT GRAPH WAIT node=impact".into());
    assert!(sim.take_autopilot_events().is_empty());
}

#[test]
fn coalescing_keeps_latest_controls_and_all_event_commands() {
    let mut pending = PendingInput::default();
    let mut first = input(vec![Command::Stage]);
    first.control_input = [0.1, 0.0, 0.0];
    pending.push(first, 1);
    let mut second = input(vec![
        Command::Stage,
        Command::Pause { paused: true },
        Command::SetWarp { factor: 128.0 },
    ]);
    second.control_input = [0.2, 0.0, 0.0];
    pending.push(second, 2);
    let mut third = input(vec![
        Command::Pause { paused: false },
        Command::SetWarp { factor: 256.0 },
    ]);
    third.control_input = [0.3, 0.0, 0.0];
    pending.push(third, 3);

    let merged = pending.take().expect("coalesced input");
    assert_eq!(merged.control_input, [0.3, 0.0, 0.0]);
    assert_eq!(merged.commands[0], Command::Stage);
    assert_eq!(merged.commands[1], Command::Stage);
    assert_eq!(merged.commands[2], Command::Pause { paused: false });
    assert_eq!(merged.commands[3], Command::SetWarp { factor: 256.0 });
}

#[test]
fn coalescing_preserves_ordered_part_commands() {
    let commands = vec![
        Command::Part {
            command: thessa_sim_core::VehiclePartCommand::SetRcsEnabled { enabled: false },
        },
        Command::Part {
            command: thessa_sim_core::VehiclePartCommand::SetLandingGearDeployed { deployed: true },
        },
    ];
    let mut pending = PendingInput::default();
    pending.push(input(commands.clone()), 1);
    assert_eq!(pending.take().expect("part input").commands, commands);
}

#[test]
fn coalesced_trailing_toggle_after_an_edge_is_preserved() {
    // [Stage(seq1, echo=false), continuous(seq2, echo=true)]: the user
    // toggled after the edge. The merged batch must carry the newer
    // absolute intent as a trailing edge instead of dropping it.
    let mut pending = PendingInput::default();
    pending.push(input(vec![Command::Stage]), 1);
    let mut trailing = input(vec![]);
    trailing.engine_active = true;
    pending.push(trailing, 2);
    let merged = pending.take().expect("merged input");
    assert_eq!(
        merged.commands.last(),
        Some(&Command::Engine { active: true })
    );
}

#[test]
fn coalesced_stale_echo_does_not_undo_a_toggle() {
    // [Stage(seq1, echo=false), continuous(seq2, echo=false)]: no new
    // user intent — the echo must not override the toggle result.
    let mut pending = PendingInput::default();
    pending.push(input(vec![Command::Stage]), 1);
    pending.push(input(vec![]), 2);
    let merged = pending.take().expect("merged input");
    assert_eq!(merged.commands, vec![Command::Stage]);
}

#[test]
fn coalesced_engine_events_match_sequential_application() {
    let config: SystemConfig =
        toml::from_str(include_str!("../../../data/system.toml")).expect("system");
    let ephemeris = config.bake().expect("bake");
    let reference_body = ephemeris.body_id("thessa").expect("thessa");
    let mut sequential = Sim::new(ephemeris.clone(), reference_body, false, true).expect("sim");
    sequential.register("pilot");
    let first = input(vec![Command::Engine { active: true }]);
    let second = input(vec![Command::Stage]);
    let _ = sequential.apply_input("pilot", &first);
    let _ = sequential.apply_input("pilot", &second);
    let expected = sequential.authority.engine_active;

    let mut pending = PendingInput::default();
    pending.push(first, 1);
    pending.push(second, 2);
    let merged = pending.take().expect("merged input");
    let mut coalesced = Sim::new(ephemeris, reference_body, false, true).expect("sim");
    coalesced.register("pilot");
    let _ = coalesced.apply_input("pilot", &merged);
    assert_eq!(coalesced.authority.engine_active, expected);
}

#[test]
fn saturated_work_quantum_does_not_sleep_as_a_fixed_duty_cycle() {
    assert_eq!(
        driver_sleep_duration(false, true, false, 0.0, 256.0, Duration::from_millis(100),),
        Duration::ZERO
    );
    assert_eq!(
        driver_sleep_duration(true, false, false, 0.0, 256.0, Duration::from_millis(100),),
        Duration::ZERO
    );
    assert_eq!(
        driver_sleep_duration(false, false, true, 0.0, 256.0, Duration::ZERO,),
        BAKE_POLL_INTERVAL
    );
}

#[test]
fn pacing_demand_does_not_double_count_fractional_authority_backlog() {
    let tick_s = thessa_sim_core::WORLD_TICK_S;
    let config: SystemConfig =
        toml::from_str(include_str!("../../../data/system.toml")).expect("system");
    let ephemeris = config.bake().expect("bake");
    let reference_body = ephemeris.body_id("thessa").expect("thessa");
    let mut authority = FlightAuthority::new(&ephemeris, reference_body).expect("authority");
    let targets = [(0.012, 1_u64), (0.020, 2), (0.025, 3), (0.030, 3)];

    for (target_s, expected_ticks) in targets {
        let advanced_s = authority.flight_time_s;
        let backlog_s = authority.backlog_s();
        let demand_s = pacing_demand_s(target_s, advanced_s, backlog_s, tick_s);
        authority
            .advance(&ephemeris, ControlMode::Direct, demand_s)
            .expect("target advancement");

        let expected_time_s = expected_ticks as f64 * tick_s;
        assert!(
            (authority.flight_time_s - expected_time_s).abs() < 1.0e-12,
            "target={target_s} advanced={} expected={expected_time_s} backlog={}",
            authority.flight_time_s,
            authority.backlog_s()
        );
        assert!(authority.backlog_s() < tick_s);
        assert!(
            authority.flight_time_s + authority.backlog_s() <= target_s + 1.0e-12,
            "target={target_s} was overrun: covered={}",
            authority.flight_time_s + authority.backlog_s()
        );
        assert!(
            target_s - (authority.flight_time_s + authority.backlog_s()) < tick_s,
            "target={target_s} left more than one tick unserved: covered={}",
            authority.flight_time_s + authority.backlog_s()
        );
    }
}

#[test]
fn zero_elapsed_advance_services_a_queued_tick_at_a_scheduler_wake() {
    let tick_s = thessa_sim_core::WORLD_TICK_S;
    let config: SystemConfig =
        toml::from_str(include_str!("../../../data/system.toml")).expect("system");
    let ephemeris = config.bake().expect("bake");
    let reference_body = ephemeris.body_id("thessa").expect("thessa");
    let mut authority = FlightAuthority::new(&ephemeris, reference_body).expect("authority");
    authority.accumulator_s = tick_s;
    authority
        .scheduler
        .arm(thessa_sim_core::ScheduledKind::Alarm, SimTime(0.001));

    let clipped = clip_pacing_chunk_to_wake(
        0.0,
        authority.flight_time_s,
        authority.backlog_s(),
        SimTime(0.001),
        tick_s,
    );
    assert_eq!(clipped, 0.0);
    assert!(pacing_work_pending(clipped, authority.backlog_s(), tick_s));

    authority
        .advance(&ephemeris, ControlMode::Direct, 0.0)
        .expect("service queued tick");
    assert_eq!(authority.steps_this_frame, 1);
    assert!(authority.backlog_s() < tick_s);
    assert!(
        authority
            .wake_notice
            .as_deref()
            .is_some_and(|notice| notice.contains("WAKE ALARM"))
    );
}

#[test]
fn pacing_wake_clip_reaches_the_next_fixed_tick_without_overshoot() {
    let tick_s = thessa_sim_core::WORLD_TICK_S;
    let flight_time_s = tick_s;
    let backlog_s = 0.003666666666666667;
    let wake = SimTime(flight_time_s + tick_s * 0.25);
    let clipped = clip_pacing_chunk_to_wake(tick_s * 4.0, flight_time_s, backlog_s, wake, tick_s);

    assert!((clipped - (tick_s - backlog_s)).abs() < 1.0e-12);
    assert!(clipped < tick_s);
}

#[test]
fn first_client_owns_controls_and_disconnect_transfers_in_order() {
    let (mut driver, _ingress) = test_driver();
    driver.sim.register("first");
    driver.sim.register("second");
    assert_eq!(driver.sim.pilot_owner.as_deref(), Some("first"));

    let mut manual = input(Vec::new());
    manual.control_input = [1.0, 0.0, 0.0];
    manual.throttle = 1.0;
    manual.engine_active = true;
    assert!(!driver.sim.apply_input("first", &manual));
    let state_before_spectator = driver.sim.authority.state;
    let mut spectator_command = manual.clone();
    spectator_command.commands = vec![Command::Stage, Command::Reset];
    assert!(!driver.sim.apply_input("second", &spectator_command));
    assert_eq!(driver.sim.authority.state, state_before_spectator);
    assert_eq!(driver.sim.authority.control_input, DVec3::X);

    let spectator_vote = input(vec![Command::Pause { paused: true }]);
    assert!(driver.sim.apply_input("second", &spectator_vote));
    assert!(driver.sim.paused());

    let invalid_guidance = GuidanceInput {
        tick: 0,
        intent: GuidanceIntent::AngularRate {
            rate_body_rps: DVec3::splat(f64::NAN),
        },
        propulsion: PropulsionDemand {
            normalized: f64::NAN,
        },
    };
    assert!(!driver.sim.apply_guidance("second", &invalid_guidance));
    assert!(driver.sim.authority.flight_error.is_none());
    assert!(!driver.sim.apply_autopilot(
        "second",
        &AutopilotInput {
            tick: 0,
            command: AutopilotCommand::StartScript {
                source: String::new(),
            },
        },
        &mut driver.autopilot,
    ));

    driver.release_client("first");
    assert_eq!(driver.sim.pilot_owner.as_deref(), Some("second"));
    assert_eq!(driver.sim.authority.control_input, DVec3::ZERO);
    assert!(!driver.sim.authority.engine_active);
    assert_eq!(driver.autopilot.scheduler.pending(), 0);
}

#[test]
fn invalid_input_is_rejected_before_any_state_or_takeover_mutation() {
    let (mut driver, _ingress) = test_driver();
    driver.sim.register("pilot");
    let mut invalid = input(vec![Command::Reset]);
    invalid.control_input[0] = f64::NAN;
    invalid.throttle = f64::INFINITY;
    invalid.sas_target_xyzw[3] = f64::NAN;
    let state = driver.sim.authority.state;
    let control = driver.sim.authority.control_input;
    let engine = driver.sim.authority.engine_active;
    assert!(!driver.apply_client_input("pilot", &invalid));
    assert_eq!(driver.sim.authority.state, state);
    assert_eq!(driver.sim.authority.control_input, control);
    assert_eq!(driver.sim.authority.engine_active, engine);
    assert!(driver.sim.authority.flight_error.is_none());
    assert!(driver.sim.last_client_inputs.is_empty());
}

#[test]
fn input_tick_is_advisory_and_sas_quaternion_is_normalized() {
    let (mut driver, _ingress) = test_driver();
    driver.sim.register("pilot");
    let mut value = input(Vec::new());
    value.tick = u64::MAX;
    value.sas_target_xyzw = [0.0, 0.0, 2.0, 2.0];

    driver.apply_client_input("pilot", &value);
    assert!(driver.sim.last_client_inputs.contains_key("pilot"));
    let expected = DQuat::from_rotation_z(std::f64::consts::FRAC_PI_2);
    let actual = driver.sim.authority.sas_target_orientation;
    assert!((actual.dot(expected).abs() - 1.0).abs() < 1.0e-12);
}

#[test]
fn outbound_mailbox_keeps_welcome_order_and_latest_snapshot_only() {
    let mailbox = OutboundMailbox::new();
    assert!(mailbox.push_reliable(vec![1]));
    assert!(mailbox.push_reliable(vec![2]));
    for value in 3..100 {
        assert!(mailbox.replace_snapshot(Arc::new(vec![value])));
    }
    assert_eq!(mailbox.try_next().unwrap().as_ref(), [1]);
    assert_eq!(mailbox.try_next().unwrap().as_ref(), [2]);
    let latest_snapshot = Arc::new(vec![100]);
    assert!(mailbox.replace_snapshot(latest_snapshot.clone()));
    let delivered = mailbox.try_next().expect("latest snapshot");
    assert_eq!(delivered.as_ref(), [100]);
    let OutboundFrame::Shared(delivered_snapshot) = delivered else {
        panic!("latest snapshots must retain their shared frame");
    };
    assert!(Arc::ptr_eq(&delivered_snapshot, &latest_snapshot));
    assert!(mailbox.try_next().is_none());

    for _ in 0..RELIABLE_OUTBOUND_CAPACITY {
        assert!(mailbox.push_reliable(vec![0]));
    }
    assert!(!mailbox.push_reliable(vec![0]));
}

#[test]
fn ingress_coalesces_continuous_input_and_reserves_leave_cleanup() {
    let (ingress, mut receiver) = IngressSender::new();
    assert!(ingress.reserve_exact_connection("pilot".into()));
    assert!(ingress.mark_active_for_sender("pilot"));
    for tick in 0..10_000 {
        let mut value = input(Vec::new());
        value.tick = tick;
        assert!(ingress.send_input("pilot", value));
    }
    let (events, continuous, saturated) = receiver.take_batch(MAX_INGRESS_MESSAGES);
    assert!(events.is_empty());
    assert_eq!(continuous.len(), 1);
    assert!(!saturated);

    for _ in 0..MAX_INGRESS_MESSAGES {
        assert!(ingress.send_input("pilot", input(vec![Command::Stage])));
    }
    // Reliability: a full shared queue must NOT report success for a
    // dropped edge (Stage/Engine/Reset/ExecuteManeuver/ExecuteBurnPlan).
    // Backpressure (`false`) forces the caller to fail loudly
    // (disconnect/retry) instead of desyncing client intent from
    // authoritative state.
    assert!(!ingress.send_input("pilot", input(vec![Command::Stage])));
    assert!(ingress.send_leave("pilot"));
    assert!(receiver.take_leaves().contains("pilot"));
}

#[test]
fn eof_before_subscribe_does_not_resurrect_a_queued_connection() {
    let (mut driver, ingress) = test_driver();
    let id = ingress
        .reserve_connection("peer")
        .expect("connection token");
    let mailbox = Arc::new(OutboundMailbox::new());
    assert!(ingress.send_subscribe(&id, mailbox.clone()));
    assert!(ingress.send_leave(&id));

    let _ = driver.drain_upstream();
    assert!(driver.sim.clients.is_empty());
    assert!(driver.subscribers.is_empty());
    assert!(
        !ingress
            .connections
            .lock()
            .expect("connection registry")
            .contains_key(&id)
    );
    assert!(mailbox.try_next().is_none());
}

#[test]
fn sequence_order_preserves_input_edges_and_script_transitions() {
    let (mut driver, ingress) = test_driver();
    driver.sim.register("pilot");

    let mut continuous = input(Vec::new());
    continuous.control_input = [0.1, 0.0, 0.0];
    continuous.throttle = 0.1;
    assert!(ingress.send_input("pilot", continuous));
    let mut edge = input(vec![Command::Stage]);
    edge.control_input = [0.8, 0.0, 0.0];
    edge.throttle = 0.8;
    assert!(ingress.send_input("pilot", edge));
    let _ = driver.drain_upstream();
    assert_eq!(
        driver.sim.authority.control_input,
        DVec3::new(0.8, 0.0, 0.0)
    );
    assert!(driver.sim.authority.engine_active);

    let (mut driver, ingress) = test_driver();
    driver.sim.register("pilot");
    let mut edge = input(vec![Command::Stage]);
    edge.control_input = [0.8, 0.0, 0.0];
    edge.throttle = 0.8;
    assert!(ingress.send_input("pilot", edge));
    let mut continuous = input(Vec::new());
    continuous.control_input = [0.1, 0.0, 0.0];
    continuous.throttle = 0.1;
    assert!(ingress.send_input("pilot", continuous));
    let _ = driver.drain_upstream();
    assert_eq!(
        driver.sim.authority.control_input,
        DVec3::new(0.1, 0.0, 0.0)
    );
    assert!(driver.sim.authority.engine_active);

    let (mut driver, ingress) = test_driver();
    driver.sim.register("pilot");
    let mut before_script = input(Vec::new());
    before_script.control_input = [0.1, 0.0, 0.0];
    assert!(ingress.send_input("pilot", before_script));
    assert!(ingress.send_autopilot(
        "pilot",
        AutopilotInput {
            tick: 0,
            command: AutopilotCommand::StartScript {
                source: "await sim.sleep(1); return Guidance.angularRate(0.1, 0, 0);".into(),
            },
        },
    ));
    let _ = driver.drain_upstream();
    assert_eq!(driver.autopilot.scheduler.pending(), 1);

    let (mut driver, ingress) = test_driver();
    driver.sim.register("pilot");
    assert!(ingress.send_autopilot(
        "pilot",
        AutopilotInput {
            tick: 0,
            command: AutopilotCommand::StartScript {
                source: "await sim.sleep(1); return Guidance.angularRate(0.1, 0, 0);".into(),
            },
        },
    ));
    let mut deliberate_manual = input(Vec::new());
    deliberate_manual.control_input = [0.1, 0.0, 0.0];
    assert!(ingress.send_input("pilot", deliberate_manual));
    let _ = driver.drain_upstream();
    assert_eq!(driver.autopilot.scheduler.pending(), 0);
}

#[test]
fn ingress_watermark_defers_latest_state_behind_a_second_event_slice() {
    let (mut driver, ingress) = test_driver();
    driver.sim.register("pilot");
    let guidance = GuidanceInput {
        tick: 0,
        intent: GuidanceIntent::AngularRate {
            rate_body_rps: DVec3::ZERO,
        },
        propulsion: PropulsionDemand::new(0.0).expect("zero propulsion"),
    };
    for _ in 0..=MAX_UPSTREAM_MESSAGES_PER_ITERATION {
        assert!(ingress.send_guidance("pilot", guidance.clone()));
    }
    let mut manual = input(Vec::new());
    manual.control_input = [0.4, 0.0, 0.0];
    assert!(ingress.send_input("pilot", manual));

    let (_, first_saturated) = driver.drain_upstream();
    assert!(first_saturated);
    assert_eq!(driver.sim.authority.control_input, DVec3::ZERO);
    let (_, second_saturated) = driver.drain_upstream();
    assert!(!second_saturated);
    assert_eq!(
        driver.sim.authority.control_input,
        DVec3::new(0.4, 0.0, 0.0)
    );
}

#[test]
fn lowering_warp_rebases_old_pacing_debt() {
    let config: SystemConfig =
        toml::from_str(include_str!("../../../data/system.toml")).expect("system");
    let ephemeris = config.bake().expect("bake");
    let reference_body = ephemeris.body_id("thessa").expect("thessa");
    let sim = Sim::new(ephemeris, reference_body, false, true).expect("sim");
    let (upstream_tx, upstream_rx) = IngressSender::new();
    assert!(upstream_tx.reserve_exact_connection("pilot".into()));
    assert!(upstream_tx.mark_active_for_sender("pilot"));
    let mut driver = Driver {
        sim,
        autopilot: AutopilotHost::new().expect("autopilot host"),
        upstream: upstream_rx,
        subscribers: Vec::new(),
        pacing_target_s: 1.0e9,
        last_pacing: Instant::now(),
        last_snapshot: Instant::now(),
        last_status: Instant::now(),
        exit_when_empty: false,
    };
    driver.sim.register("pilot");
    driver.sim.clients.get_mut("pilot").expect("pilot").warp = 256.0;
    assert!(upstream_tx.send_input("pilot", input(vec![Command::SetWarp { factor: 64.0 }]),));

    assert!(!driver.iterate().expect("driver iteration"));
    assert!(
        driver.pacing_target_s < 100.0,
        "old pacing debt survived warp reduction: {}",
        driver.pacing_target_s
    );
}

#[test]
fn pause_vote_forces_a_prompt_snapshot_even_at_high_warp() {
    let config: SystemConfig =
        toml::from_str(include_str!("../../../data/system.toml")).expect("system");
    let ephemeris = config.bake().expect("bake");
    let reference_body = ephemeris.body_id("thessa").expect("thessa");
    let sim = Sim::new(ephemeris, reference_body, false, true).expect("sim");
    let (upstream_tx, upstream_rx) = IngressSender::new();
    assert!(upstream_tx.reserve_exact_connection("pilot".into()));
    assert!(upstream_tx.mark_active_for_sender("pilot"));
    let snapshot_mailbox = Arc::new(OutboundMailbox::new());
    let mut driver = Driver {
        sim,
        autopilot: AutopilotHost::new().expect("autopilot host"),
        upstream: upstream_rx,
        subscribers: vec![("pilot".into(), snapshot_mailbox.clone())],
        pacing_target_s: 0.0,
        last_pacing: Instant::now(),
        last_snapshot: Instant::now(),
        last_status: Instant::now(),
        exit_when_empty: false,
    };
    driver.sim.register("pilot");
    assert!(upstream_tx.send_input(
        "pilot",
        input(vec![
            Command::SetWarp { factor: MAX_WARP },
            Command::Pause { paused: true },
        ]),
    ));

    let started = Instant::now();
    assert!(!driver.iterate().expect("driver iteration"));
    assert!(started.elapsed() < Duration::from_millis(100));
    let frame = snapshot_mailbox.try_next().expect("forced snapshot");
    let mut decoder = FrameDecoder::new();
    let frames = decoder.push(frame.as_ref()).expect("snapshot frame");
    let envelope = thessa_flight_net::decode_frame(&frames[0]).expect("envelope");
    let snapshot: Snapshot = thessa_flight_net::decode_payload(&envelope).expect("snapshot");
    assert!(snapshot.paused);
    assert_eq!(snapshot.server_compute_s, 0.0);
    assert!(snapshot.server_wall_s > 0.0);
}

#[test]
fn transport_handshake_preserves_pipelined_and_partial_frames() {
    let hello = thessa_flight_net::encode_hello(&thessa_flight_net::Hello {
        client_name: "test".into(),
    })
    .expect("hello");
    let input = thessa_flight_net::encode_input(&input(vec![Command::Stage])).expect("input");
    let mut stream = hello.clone();
    stream.extend_from_slice(&input);
    let split = hello.len() + input.len() / 2;
    let mut decoder = FrameDecoder::new();
    let first = decoder.push(&stream[..split]).expect("first read");
    assert_eq!(first.len(), 1);
    assert_eq!(
        thessa_flight_net::decode_frame(&first[0])
            .expect("hello envelope")
            .kind,
        kind::HELLO
    );
    let second = decoder.push(&stream[split..]).expect("second read");
    assert_eq!(second.len(), 1);
    assert_eq!(
        thessa_flight_net::decode_frame(&second[0])
            .expect("input envelope")
            .kind,
        kind::CLIENT_INPUT
    );

    let mut pipelined = hello;
    pipelined.extend_from_slice(&input);
    pipelined.extend_from_slice(&input);
    let frames = FrameDecoder::new();
    let mut frames = frames;
    let all = frames.push(&pipelined).expect("pipelined read");
    assert_eq!(all.len(), 3, "all post-hello frames must survive");
}
