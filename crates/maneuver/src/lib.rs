//! Maneuver planning and node execution (Project Thessa automation layer).
//!
//! UX reference: MechJeb-style `Maneuver Planner` / `Node Executor`
//! (`Circularize`, `Hohmann transfer`, rendezvous) and the `ManeuverPlan`
//! typed port from docs/07. This is reference vocabulary, not a clone:
//! every high-level action is a composable block with typed inputs/outputs,
//! planning approximations are always revalidated against the exact
//! authoritative path before execution, and burns execute through real
//! propulsion over time — never as teleported velocity.
//!
//! Layering (docs/07 §7.2): this crate sits at "planner", between the
//! event-driven graph VM above and guidance/control laws below. It depends
//! only on `thessa-sim-core` (ephemerides, exact propagation, patches) and
//! never on vehicle control code: execution outputs are plain
//! direction+throttle commands with explicit frames, mapped to
//! `GuidanceIntent` by the runtime adapter.

mod execute;
mod lambert;
mod ops;
mod plan;
mod search;

pub use execute::{ExecutionCommand, ExecutorOutput, NodeExecutor};
pub use lambert::{LambertArc, LambertError, solve_lambert, solve_lambert_prograde};
pub use ops::{
    circularize_at_apse, hohmann_transfer, lambert_rendezvous, match_velocity, plane_change_dv,
};
pub use plan::{ManeuverNode, ManeuverPlan, PlanError, PlanValidation};
pub use search::{RankedPlan, SearchConfig, SearchStats, porkchop_search};
