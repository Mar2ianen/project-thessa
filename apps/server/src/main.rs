//! Headless authoritative flight server.
//!
//! Owns one [`FlightAuthority`] on a dedicated sim thread.tick loop runs the
//! 120 Hz lattice with no frame budget: warped chunks are bounded only by
//! chunk size (input responsiveness), never by wall time. Transports:
//! length-prefixed frames over stdio (embedded local client) today, TCP
//! (remote clients) next. Stdout is the wire — logs go to stderr.

mod autopilot_host;
mod driver;
mod ingress;
mod outbound;
mod pacing;
mod sim;
mod sim_autopilot;
mod sim_execution;
mod thread_bake;
mod transport;

use std::collections::{BTreeMap, BTreeSet, VecDeque};
use std::sync::Arc;
use std::time::{Duration, Instant};

use autopilot_host::{AutopilotHost, NativeGraphBlock, PhasePark};
use driver::Driver;
use glam::DVec3;
#[cfg(test)]
use ingress::MAX_INGRESS_MESSAGES;
use ingress::{IngressReceiver, IngressSender, MAX_CLIENTS, PendingInput, Upstream};
use outbound::OutboundMailbox;
#[cfg(test)]
use outbound::{OutboundFrame, RELIABLE_OUTBOUND_CAPACITY};
use pacing::{
    clip_pacing_chunk_to_wake, driver_sleep_duration, pacing_demand_s, pacing_work_pending,
};
use sim::{AutopilotEvent, Sim, client_input_takes_over};
use thessa_autopilot::{
    AutopilotGraph, BlockStatus, GraphBlock, GraphControlAction, GraphNodeConfig, GraphRunner,
    ImpactSite, LandingSite, PlanAction, PlanPoll, TrajectoryPlan, TrajectoryPlanRunner,
};
#[cfg(test)]
use thessa_autopilot::{GraphNode, NodeKind, WaitCondition};
use thessa_autopilot_js::{ScriptResult, ScriptSchedulerStep};
use thessa_flight_authority::{
    ControlMode, FlightAuthority, FlightPolicy, GuidanceIntent, ObstacleReport, PropulsionDemand,
    canonical_launch_setup,
};
use thessa_flight_control::{ControlDemand, DirectionFrame, DirectionTarget, RollPolicy};
use thessa_flight_net::{
    AutopilotCommand, AutopilotInput, BurnDirectionCommand, ClientInput, Command, GuidanceInput,
    Snapshot,
};
use thessa_maneuver::{
    BurnSegment, EngineSpec, FiniteBurnPlan, ManeuverPlan, NodeExecutor, PlanValidation,
    SegmentDirection, SegmentExecutor, SteeringSample,
};
#[cfg(test)]
use thessa_protocol::{FrameDecoder, kind};
use thessa_sim_core::{BakedEphemeris, BodyId, BodyState, ScheduledKind, SimTime, SystemConfig};
use thread_bake::ThreadBakeQueue;
#[cfg(test)]
use transport::{DecodedClientMessage, decode_client_message};
use transport::{run_stdio, run_tcp};

/// Wall-time quantum for the authoritative driver. It bounds how long the
/// driver can stay away from input/snapshot handling; it is not a simulation
/// rate cap. Each quantum requests `warp * quantum` simulation seconds and
/// the authority serves as much as its work budget/rails path allows.
const DRIVER_WORK_QUANTUM_S: f64 = 0.020;
/// CPU work quantum for ordinary fixed-step physics. When the solver is slower
/// than the requested warp, unserved demand is dropped and effective warp
/// reports the actual result; rails batches remain the high-throughput path.
const SIM_WORK_BUDGET: Duration = Duration::from_millis(4);
/// Retry cadence while a background rails bake is still warming the cache.
const BAKE_POLL_INTERVAL: Duration = Duration::from_millis(2);
/// Wall-pacing for snapshots so high warp does not flood the pipe; the
/// client renders between them.
const SNAPSHOT_MIN_INTERVAL_S: f64 = 0.05;
/// Do not let a producer that ignores input coalescing monopolise the driver.
/// This is an ingress fairness quantum, not a simulation/TPS limit.
const MAX_UPSTREAM_MESSAGES_PER_ITERATION: usize = 256;
/// Upper clamp for requested warp (2^17, same ceiling as the client).
const MAX_WARP: f64 = 131072.0;

struct Args {
    system_path: Option<String>,
    tcp_addr: Option<String>,
    /// Batch throughput probe: advance this many sim-seconds flat out,
    /// print the effective warp, exit. No IO after setup.
    measure_s: Option<f64>,
    /// Cut the engine for the run: unpowered coast rides rails batches.
    drift: bool,
    /// Start in a 300 km circular coast instead of on the pad.
    vacuum: bool,
}

fn parse_args() -> Result<Args, String> {
    let mut args = Args {
        system_path: None,
        tcp_addr: None,
        measure_s: None,
        vacuum: false,
        drift: false,
    };
    let mut rest = std::env::args().skip(1);
    while let Some(arg) = rest.next() {
        match arg.as_str() {
            "--system" => {
                args.system_path = Some(rest.next().ok_or("--system needs a path")?);
            }
            "--measure" => {
                let seconds: f64 = rest
                    .next()
                    .ok_or("--measure needs sim-seconds")?
                    .parse()
                    .map_err(|_| "--measure needs a number")?;
                if seconds <= 0.0 {
                    return Err("--measure needs positive sim-seconds".into());
                }
                args.measure_s = Some(seconds);
            }
            "--vacuum" => args.vacuum = true,
            "--drift" => args.drift = true,
            "--tcp" => {
                args.tcp_addr = Some(rest.next().ok_or("--tcp needs ADDR:PORT")?);
            }
            "--help" | "-h" => {
                eprintln!(
                    "usage: thessa-server [--system PATH] [--measure SIM_S] [--vacuum] [--drift] [--tcp ADDR:PORT]\n\
                     default: stdio wire server (frames on stdin/stdout, logs on stderr)"
                );
                std::process::exit(0);
            }
            other => return Err(format!("unknown argument: {other}")),
        }
    }
    Ok(args)
}

fn find_system(cli_path: Option<String>) -> Result<String, String> {
    if let Some(path) = cli_path {
        return Ok(path);
    }
    for candidate in ["data/system.toml"] {
        if std::path::Path::new(candidate).exists() {
            return Ok(candidate.into());
        }
    }
    let exe = std::env::current_exe().map_err(|e| e.to_string())?;
    for ancestor in exe.ancestors().skip(1).take(4) {
        let candidate = ancestor.join("data/system.toml");
        if candidate.exists() {
            return Ok(candidate.to_string_lossy().into_owned());
        }
    }
    Err("system.toml not found: pass --system PATH".into())
}

fn load_system(path: &str) -> Result<(BakedEphemeris, BodyId), String> {
    let text = std::fs::read_to_string(path).map_err(|e| e.to_string())?;
    let config: SystemConfig = toml::from_str(&text).map_err(|e| e.to_string())?;
    let ephemeris = config.bake().map_err(|e| e.to_string())?;
    let body = ephemeris
        .body_id("thessa")
        .ok_or_else(|| "no thessa body in system".to_string())?;
    Ok((ephemeris, body))
}

fn place_in_circular_orbit(
    ephemeris: &BakedEphemeris,
    authority: &mut FlightAuthority,
    altitude_m: f64,
) -> Result<(), String> {
    use glam::DQuat;
    let body = ephemeris
        .body(authority.reference_body)
        .map_err(|e| e.to_string())?;
    let origin = ephemeris
        .body_state(authority.reference_body, SimTime::EPOCH)
        .map_err(|e| e.to_string())?;
    let radius = body.radius_m + altitude_m;
    authority.state.position_inertial_m = origin.position_inertial + DVec3::Z * radius;
    authority.state.velocity_inertial_mps =
        origin.velocity_inertial + DVec3::X * (body.mu / radius).sqrt();
    authority.state.orientation_body_to_inertial = DQuat::IDENTITY;
    authority.state.angular_velocity_body_rps = DVec3::ZERO;
    authority.sas_target_orientation = DQuat::IDENTITY;
    authority.set_legacy_propulsion(1.0, true);
    authority.control_input = DVec3::ZERO;
    Ok(())
}

fn graph_errors(errors: Vec<thessa_autopilot::GraphError>) -> String {
    errors
        .into_iter()
        .map(|error| error.to_string())
        .collect::<Vec<_>>()
        .join("; ")
}

fn run_measure(mut sim: Sim, target_s: f64) -> Result<(), String> {
    let t0 = Instant::now();
    sim.wall_started = t0;
    let mut chunks = 0u64;
    while sim.advanced_s < target_s {
        // One max-batch per chunk, like the live loop at high warp
        // (catch-up rule applies).
        let advanced = sim
            .advance_chunk(3600.0)
            .map_err(|e| format!("advance: {e}"))?;
        chunks += 1;
        if chunks.is_multiple_of(500) {
            eprintln!(
                "  [progress] sim={:.0} rails_total={:.0} steps_total={} bake_pending={}",
                sim.advanced_s,
                sim.rails_s,
                sim.steps,
                sim.authority.bake.has_pending()
            );
        }
        if advanced <= 0.0
            && sim.authority.waiting_for_rails_bake
            && sim.authority.bake.has_pending()
        {
            // A cold drift measurement waits for the same asynchronous bake
            // as the live driver. Avoid burning a core while the worker runs.
            std::thread::sleep(BAKE_POLL_INTERVAL);
        }
        if sim.authority.flight_error.is_some() {
            break;
        }
    }
    let wall = t0.elapsed().as_secs_f64();
    println!(
        "sim_s={:.1} wall_s={:.3} effective_warp=x{:.1} steps={} rails_s={:.1} flight_t={:.1} err={:?}",
        sim.advanced_s,
        wall,
        sim.advanced_s / wall.max(1e-9),
        sim.steps,
        sim.rails_s,
        sim.authority.flight_time_s,
        sim.authority.flight_error,
    );
    Ok(())
}

fn main() {
    if let Err(error) = run() {
        eprintln!("[server] fatal: {error}");
        std::process::exit(1);
    }
}

fn run() -> Result<(), String> {
    let args = parse_args()?;
    let system_path = find_system(args.system_path)?;
    eprintln!("[server] system: {system_path}");
    let (ephemeris, reference_body) = load_system(&system_path)?;
    let sim = Sim::new(ephemeris, reference_body, args.vacuum, args.drift)?;
    match args.measure_s {
        Some(target_s) => run_measure(sim, target_s),
        None => match args.tcp_addr {
            Some(addr) => run_tcp(sim, &addr),
            None => run_stdio(sim),
        },
    }
}

#[cfg(test)]
mod reset_tests;
#[cfg(test)]
mod tests;
