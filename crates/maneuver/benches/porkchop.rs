//! Staged porkchop search on the real system (doc 24 §15 benchmark C-lite):
//! moon tour Pelagos -> Thessa around Nereid. Broad Lambert grid, local
//! refinement, exact N-body revalidation of survivors. Reports candidates
//! evaluated per second, revalidation count, winning Δv and measured miss.
use std::{hint::black_box, time::Instant};

use thessa_maneuver::{SearchConfig, porkchop_search};
use thessa_sim_core::{BakedEphemeris, GravityField, SimTime, SystemConfig};

fn main() {
    let config: SystemConfig = toml::from_str(include_str!("../../../data/system.toml")).unwrap();
    let ephemeris: BakedEphemeris = config.bake().unwrap();
    let field = GravityField::from_ephemeris(&ephemeris);
    println!("sources: {}", field.source_count());
    let central = ephemeris.body_id("nereid").expect("nereid");
    let departure = ephemeris.body_id("pelagos").expect("pelagos");
    let arrival = ephemeris.body_id("thessa").expect("thessa");
    let standoff_m = 100_000.0;
    let search = SearchConfig {
        central_body: central,
        departure_body: departure,
        arrival_body: arrival,
        window_start: SimTime::EPOCH,
        departure_span_s: 10.0 * 86_400.0,
        departure_steps: 100,
        tof_min_s: 20.0 * 3_600.0,
        tof_max_s: 200.0 * 3_600.0,
        tof_steps: 100,
        keep_candidates: 10,
        standoff_m,
        max_broad_dv_mps: 5_000.0,
        max_miss_m: 1.0e6,
    };
    let started = Instant::now();
    let (ranked, stats) =
        porkchop_search(black_box(&ephemeris), black_box(&field), search).expect("search finds");
    let elapsed = started.elapsed();
    let winner = &ranked[0];
    println!(
        "porkchop pelagos->thessa: {} broad cells ({:.0}/s), {} degenerate, {} impact, {} exact revalidations ({} failed) in {elapsed:?}",
        stats.broad_evaluations,
        stats.broad_evaluations as f64 / elapsed.as_secs_f64(),
        stats.degenerate_cells,
        stats.impact_cells,
        stats.exact_revalidations,
        stats.failed_revalidations,
    );
    println!(
        "winner: depart T+{:.1}h, tof {:.1}h, broad dv {:.1} m/s, exact dv {:.1} m/s, miss {:.1} km",
        (winner.departure_epoch.0 - SimTime::EPOCH.0) / 3_600.0,
        winner.time_of_flight_s / 3_600.0,
        winner.broad_total_dv_mps,
        winner.exact_total_dv_mps,
        winner.exact_miss_m / 1_000.0,
    );
    // Encounter kinematics in the central-body frame: radii, orbital speeds,
    // and the arrival encounter angle between transfer velocity and target
    // velocity (0 = perfect chase, 180 = head-on). Explains arrival cost.
    {
        let arr_epoch = SimTime(winner.departure_epoch.0 + winner.time_of_flight_s);
        let dep = ephemeris
            .body_state(departure, winner.departure_epoch)
            .unwrap();
        let arr = ephemeris.body_state(arrival, arr_epoch).unwrap();
        let cen_dep = ephemeris
            .body_state(central, winner.departure_epoch)
            .unwrap();
        let cen_arr = ephemeris.body_state(central, arr_epoch).unwrap();
        let dep_r = dep.position_inertial - cen_dep.position_inertial;
        let dep_v = dep.velocity_inertial - cen_dep.velocity_inertial;
        let arr_r = arr.position_inertial - cen_arr.position_inertial;
        let arr_v = arr.velocity_inertial - cen_arr.velocity_inertial;
        let dv = winner.plan.nodes.last().unwrap().delta_v_mps;
        let v_tr = arr_v - dv;
        let encounter_deg = v_tr
            .normalize()
            .dot(arr_v.normalize())
            .clamp(-1.0, 1.0)
            .acos()
            .to_degrees();
        // Well anatomy: the arrival match nulls the full N-body velocity at
        // the standoff aim point, so its floor is the local escape velocity.
        let target = ephemeris.body(arrival).unwrap();
        let vesc_aim = (2.0 * target.mu / (target.radius_m + standoff_m)).sqrt();
        println!(
            "encounter: dep r={:.3e}m v={:.0}m/s, arr r={:.3e}m v={:.0}m/s, \
             transfer arrival v={:.0}m/s at {:.1}deg to target, vinf={:.0}m/s (vesc_aim={:.0}m/s)",
            dep_r.length(),
            dep_v.length(),
            arr_r.length(),
            arr_v.length(),
            v_tr.length(),
            encounter_deg,
            dv.length(),
            vesc_aim,
        );
    }
    for (index, plan) in ranked.iter().enumerate() {
        let nodes: Vec<String> = plan
            .plan
            .nodes
            .iter()
            .map(|node| format!("{:.0}", node.magnitude_mps()))
            .collect();
        // Arrival-burn vector anatomy (in-plane vs normal share).
        let arr_info = if plan.plan.nodes.len() >= 2 {
            let arr = plan.plan.nodes.last().unwrap().delta_v_mps;
            format!("arr=({:.0},{:.0},{:.0})", arr.x, arr.y, arr.z)
        } else {
            "arr=?".to_string()
        };
        println!(
            "  #{index}: dv {:.1} m/s, miss {:.1} km, nodes [{}] {arr_info}",
            plan.exact_total_dv_mps,
            plan.exact_miss_m / 1_000.0,
            nodes.join("+"),
        );
    }
}
