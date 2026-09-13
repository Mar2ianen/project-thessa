//! Sandboxed QuickJS graph-block host.
//!
//! JavaScript in this crate is a high-level producer of typed guidance
//! values. It never receives a `FlightAuthority`, rigid-body state handle, or
//! actuator reference. The host exposes only deterministic constructors and
//! rejects ambient capabilities before evaluating user code.

use std::{
    error::Error,
    fmt,
    sync::{
        Arc,
        atomic::{AtomicBool, Ordering},
    },
};

use rquickjs::{Context, Runtime};
use serde::Deserialize;
use thessa_autopilot::{Bakeability, Diagnostic, TrajectoryPlan, TrajectorySegment, WaitCondition};
use thessa_flight_control::{DirectionFrame, DirectionTarget, GuidanceIntent, RollPolicy};

#[derive(Debug, Clone, Copy)]
pub struct ScriptLimits {
    pub memory_bytes: usize,
    pub stack_bytes: usize,
    pub max_source_bytes: usize,
}

impl Default for ScriptLimits {
    fn default() -> Self {
        Self {
            memory_bytes: 4 * 1024 * 1024,
            stack_bytes: 512 * 1024,
            max_source_bytes: 64 * 1024,
        }
    }
}

#[derive(Debug, Clone, PartialEq)]
pub enum ScriptResult {
    Guidance(GuidanceIntent),
    Plan(TrajectoryPlan),
    Wait(WaitCondition),
    Diagnostic(Diagnostic),
}

#[derive(Debug, Clone, PartialEq)]
pub enum ScriptError {
    InvalidLimits,
    SourceTooLarge { bytes: usize, limit: usize },
    Runtime(String),
    InvalidReturn(String),
}

impl fmt::Display for ScriptError {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::InvalidLimits => write!(formatter, "QuickJS limits must be positive"),
            Self::SourceTooLarge { bytes, limit } => {
                write!(formatter, "script is {bytes} bytes, limit is {limit}")
            }
            Self::Runtime(message) => write!(formatter, "QuickJS runtime error: {message}"),
            Self::InvalidReturn(message) => write!(formatter, "invalid script return: {message}"),
        }
    }
}

impl Error for ScriptError {}

pub struct ScriptEngine {
    runtime: Runtime,
    context: Context,
    limits: ScriptLimits,
    interrupt_requested: Arc<AtomicBool>,
}

impl ScriptEngine {
    pub fn new(limits: ScriptLimits) -> Result<Self, ScriptError> {
        if limits.memory_bytes == 0 || limits.stack_bytes == 0 || limits.max_source_bytes == 0 {
            return Err(ScriptError::InvalidLimits);
        }
        let runtime = Runtime::new().map_err(|error| ScriptError::Runtime(error.to_string()))?;
        runtime.set_memory_limit(limits.memory_bytes);
        runtime.set_max_stack_size(limits.stack_bytes);
        let interrupt_requested = Arc::new(AtomicBool::new(false));
        let interrupt_flag = Arc::clone(&interrupt_requested);
        runtime.set_interrupt_handler(Some(Box::new(move || {
            interrupt_flag.load(Ordering::Relaxed)
        })));
        let context =
            Context::full(&runtime).map_err(|error| ScriptError::Runtime(error.to_string()))?;
        Ok(Self {
            runtime,
            context,
            limits,
            interrupt_requested,
        })
    }

    /// Evaluate one graph block. The wrapper intentionally exposes no host
    /// objects: the only successful return values are JSON descriptions built
    /// by deterministic `Guidance`, `Wait`, or `Diagnostic` helpers.
    pub fn run(&self, source: &str) -> Result<ScriptResult, ScriptError> {
        if source.len() > self.limits.max_source_bytes {
            return Err(ScriptError::SourceTooLarge {
                bytes: source.len(),
                limit: self.limits.max_source_bytes,
            });
        }
        let wrapped = format!(
            "{}\n(function() {{\n'use strict';\n{}\n}})()",
            SANDBOX_PRELUDE, source
        );
        let json = self
            .context
            .with(|ctx| ctx.eval::<String, _>(wrapped.as_str()))
            .map_err(|error| ScriptError::Runtime(error.to_string()))?;
        parse_result(&json)
    }

    pub fn limits(&self) -> ScriptLimits {
        self.limits
    }

    /// Return a cancellation token that the owning scheduler can set while a
    /// script is running. QuickJS polls the installed interrupt handler and
    /// returns an uncatchable error instead of allowing an unbounded block to
    /// monopolize its worker.
    pub fn interrupt_handle(&self) -> Arc<AtomicBool> {
        Arc::clone(&self.interrupt_requested)
    }

    pub fn request_interrupt(&self) {
        self.interrupt_requested.store(true, Ordering::Relaxed);
    }

    pub fn clear_interrupt(&self) {
        self.interrupt_requested.store(false, Ordering::Relaxed);
    }

    /// Keep the runtime owned by this engine so QuickJS jobs cannot outlive
    /// the VM shard. The method makes the ownership invariant visible to
    /// embedders without exposing the raw QuickJS context.
    pub fn is_single_shard(&self) -> bool {
        let _ = &self.runtime;
        true
    }
}

const SANDBOX_PRELUDE: &str = r#"
(() => {
  const denied = () => { throw new Error('ambient capability is unavailable'); };
  globalThis.Date = undefined;
  globalThis.eval = undefined;
  globalThis.Function = undefined;
  globalThis.WebAssembly = undefined;
  Math.random = denied;
  globalThis.Guidance = Object.freeze({
    attitude: (x, y, z, w) => JSON.stringify({kind:'attitude', x, y, z, w}),
    angularRate: (x, y, z) => JSON.stringify({kind:'angular-rate', x, y, z}),
    velocityDirection: (x, y, z, frame) => JSON.stringify({kind:'velocity-direction', x, y, z, frame}),
  });
  globalThis.Wait = Object.freeze({
    at: (seconds) => JSON.stringify({kind:'wait-at', seconds}),
    event: (name) => JSON.stringify({kind:'wait-event', name}),
    any: (...conditions) => JSON.stringify({kind:'wait-any', conditions: conditions.map(JSON.parse)}),
    all: (...conditions) => JSON.stringify({kind:'wait-all', conditions: conditions.map(JSON.parse)}),
  });
  globalThis.Diagnostic = Object.freeze({
    warning: (code, message) => JSON.stringify({kind:'warning', code, message}),
  });
  globalThis.Plan = Object.freeze({
    coast: (duration_s) => JSON.stringify({kind:'plan', id:0, bakeability:'pure', segments:[{kind:'coast', duration_s}]}),
    burn: (duration_s, normalized) => JSON.stringify({kind:'plan', id:0, bakeability:'guarded', segments:[{kind:'burn', duration_s, normalized}]}),
    guidance: (duration_s, guidance) => JSON.stringify({kind:'plan', id:0, bakeability:'guarded', segments:[{kind:'guidance', duration_s, intent:JSON.parse(guidance)}]}),
    wait: (condition) => JSON.stringify({kind:'plan', id:0, bakeability:'live', segments:[{kind:'wait', condition:JSON.parse(condition)}]}),
  });
})();
"#;

#[derive(Debug, Deserialize)]
#[serde(tag = "kind", rename_all = "kebab-case")]
enum ScriptReturn {
    Attitude {
        x: f64,
        y: f64,
        z: f64,
        w: f64,
    },
    AngularRate {
        x: f64,
        y: f64,
        z: f64,
    },
    VelocityDirection {
        x: f64,
        y: f64,
        z: f64,
        frame: String,
    },
    WaitAt {
        seconds: f64,
    },
    WaitEvent {
        name: String,
    },
    WaitAny {
        conditions: Vec<ScriptReturn>,
    },
    WaitAll {
        conditions: Vec<ScriptReturn>,
    },
    Plan {
        id: u64,
        bakeability: String,
        segments: Vec<ScriptPlanSegment>,
    },
    Warning {
        code: String,
        message: String,
    },
}

#[derive(Debug, Deserialize)]
#[serde(tag = "kind", rename_all = "kebab-case")]
enum ScriptPlanSegment {
    Coast {
        duration_s: f64,
    },
    Burn {
        duration_s: f64,
        normalized: f64,
    },
    Guidance {
        duration_s: f64,
        intent: ScriptReturn,
    },
    Wait {
        condition: ScriptReturn,
    },
}

fn parse_result(json: &str) -> Result<ScriptResult, ScriptError> {
    let value: ScriptReturn = serde_json::from_str(json)
        .map_err(|error| ScriptError::InvalidReturn(error.to_string()))?;
    match value {
        ScriptReturn::Attitude { x, y, z, w } => Ok(ScriptResult::Guidance(parse_guidance(
            ScriptReturn::Attitude { x, y, z, w },
        )?)),
        ScriptReturn::AngularRate { x, y, z } => Ok(ScriptResult::Guidance(parse_guidance(
            ScriptReturn::AngularRate { x, y, z },
        )?)),
        ScriptReturn::VelocityDirection { x, y, z, frame } => Ok(ScriptResult::Guidance(
            parse_guidance(ScriptReturn::VelocityDirection { x, y, z, frame })?,
        )),
        ScriptReturn::WaitAt { seconds } if seconds.is_finite() && seconds >= 0.0 => Ok(
            ScriptResult::Wait(WaitCondition::At(thessa_sim_core::SimTime(seconds))),
        ),
        ScriptReturn::WaitAt { .. } => Err(ScriptError::InvalidReturn(
            "wait time must be finite and non-negative".into(),
        )),
        ScriptReturn::WaitEvent { name } if !name.trim().is_empty() => {
            Ok(ScriptResult::Wait(WaitCondition::Event(name)))
        }
        ScriptReturn::WaitEvent { .. } => Err(ScriptError::InvalidReturn(
            "wait event must not be empty".into(),
        )),
        ScriptReturn::WaitAny { conditions } => {
            parse_wait_composite(WaitCondition::Any, conditions)
        }
        ScriptReturn::WaitAll { conditions } => {
            parse_wait_composite(WaitCondition::All, conditions)
        }
        ScriptReturn::Plan {
            id,
            bakeability,
            segments,
        } => parse_plan(id, &bakeability, segments),
        ScriptReturn::Warning { code, message } => Ok(ScriptResult::Diagnostic(Diagnostic {
            kind: thessa_autopilot::DiagnosticKind::Warning,
            code,
            message,
            value: None,
            limit: None,
        })),
    }
}

fn parse_plan(
    id: u64,
    bakeability: &str,
    values: Vec<ScriptPlanSegment>,
) -> Result<ScriptResult, ScriptError> {
    let bakeability = match bakeability {
        "pure" => Bakeability::Pure,
        "guarded" => Bakeability::Guarded,
        "live" => Bakeability::Live,
        _ => {
            return Err(ScriptError::InvalidReturn(
                "unknown plan bakeability".into(),
            ));
        }
    };
    let segments = values
        .into_iter()
        .map(|value| match value {
            ScriptPlanSegment::Coast { duration_s } => Ok(TrajectorySegment::Coast { duration_s }),
            ScriptPlanSegment::Burn {
                duration_s,
                normalized,
            } => {
                let propulsion = thessa_flight_control::PropulsionDemand::new(normalized)
                    .map_err(|error| ScriptError::InvalidReturn(error.to_string()))?;
                Ok(TrajectorySegment::Burn {
                    duration_s,
                    demand: thessa_flight_control::ControlDemand {
                        propulsion,
                        ..thessa_flight_control::ControlDemand::zero()
                    },
                })
            }
            ScriptPlanSegment::Guidance { duration_s, intent } => Ok(TrajectorySegment::Guidance {
                duration_s,
                intent: parse_guidance(intent)?,
            }),
            ScriptPlanSegment::Wait { condition } => Ok(TrajectorySegment::Wait {
                condition: parse_wait_condition(condition)?,
            }),
        })
        .collect::<Result<Vec<_>, ScriptError>>()?;
    let plan = TrajectoryPlan {
        id: thessa_flight_control::TrajectoryPlanId(id),
        segments,
        bakeability,
    };
    plan.validate()
        .map_err(|error| ScriptError::InvalidReturn(error.to_string()))?;
    Ok(ScriptResult::Plan(plan))
}

fn parse_guidance(value: ScriptReturn) -> Result<GuidanceIntent, ScriptError> {
    match value {
        ScriptReturn::Attitude { x, y, z, w } => {
            let intent = GuidanceIntent::Attitude {
                target_body_to_inertial: glam::DQuat::from_xyzw(x, y, z, w),
                roll_policy: RollPolicy::Hold,
            };
            intent
                .validate()
                .map_err(|error| ScriptError::InvalidReturn(error.to_string()))?;
            Ok(intent)
        }
        ScriptReturn::AngularRate { x, y, z } => {
            let intent = GuidanceIntent::AngularRate {
                rate_body_rps: glam::DVec3::new(x, y, z),
            };
            intent
                .validate()
                .map_err(|error| ScriptError::InvalidReturn(error.to_string()))?;
            Ok(intent)
        }
        ScriptReturn::VelocityDirection { x, y, z, frame } => {
            let frame = match frame.as_str() {
                "body" => DirectionFrame::Body,
                "surface" => DirectionFrame::Surface,
                "orbit" => DirectionFrame::Orbit,
                "inertial" => DirectionFrame::Inertial,
                "target" => DirectionFrame::Target,
                _ => return Err(ScriptError::InvalidReturn("unknown direction frame".into())),
            };
            let direction = DirectionTarget::new(glam::DVec3::new(x, y, z), frame)
                .map_err(|error| ScriptError::InvalidReturn(error.to_string()))?;
            Ok(GuidanceIntent::VelocityDirection {
                direction,
                roll_policy: RollPolicy::Hold,
            })
        }
        _ => Err(ScriptError::InvalidReturn(
            "plan guidance segment contains a non-guidance value".into(),
        )),
    }
}

fn parse_wait_composite(
    constructor: fn(Vec<WaitCondition>) -> WaitCondition,
    values: Vec<ScriptReturn>,
) -> Result<ScriptResult, ScriptError> {
    let conditions = values
        .into_iter()
        .map(parse_wait_condition)
        .collect::<Result<Vec<_>, _>>()?;
    let condition = constructor(conditions);
    condition
        .validate()
        .map_err(|error| ScriptError::InvalidReturn(error.to_string()))?;
    Ok(ScriptResult::Wait(condition))
}

fn parse_wait_condition(value: ScriptReturn) -> Result<WaitCondition, ScriptError> {
    match value {
        ScriptReturn::WaitAt { seconds } if seconds.is_finite() && seconds >= 0.0 => {
            Ok(WaitCondition::At(thessa_sim_core::SimTime(seconds)))
        }
        ScriptReturn::WaitAt { .. } => Err(ScriptError::InvalidReturn(
            "wait time must be finite and non-negative".into(),
        )),
        ScriptReturn::WaitEvent { name } if !name.trim().is_empty() => {
            Ok(WaitCondition::Event(name))
        }
        ScriptReturn::WaitEvent { .. } => Err(ScriptError::InvalidReturn(
            "wait event must not be empty".into(),
        )),
        ScriptReturn::WaitAny { conditions } => {
            let result = parse_wait_composite(WaitCondition::Any, conditions)?;
            match result {
                ScriptResult::Wait(condition) => Ok(condition),
                _ => unreachable!("wait composite parser only returns waits"),
            }
        }
        ScriptReturn::WaitAll { conditions } => {
            let result = parse_wait_composite(WaitCondition::All, conditions)?;
            match result {
                ScriptResult::Wait(condition) => Ok(condition),
                _ => unreachable!("wait composite parser only returns waits"),
            }
        }
        _ => Err(ScriptError::InvalidReturn(
            "wait composite contains a non-wait value".into(),
        )),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn guidance_constructor_returns_typed_intent() {
        let engine = ScriptEngine::new(ScriptLimits::default()).unwrap();
        let result = engine
            .run("return Guidance.angularRate(0.1, 0.2, 0.3);")
            .unwrap();
        assert!(matches!(
            result,
            ScriptResult::Guidance(GuidanceIntent::AngularRate { .. })
        ));
    }

    #[test]
    fn ambient_capabilities_are_not_available() {
        let engine = ScriptEngine::new(ScriptLimits::default()).unwrap();
        assert!(engine.run("return Date.now();").is_err());
        assert!(engine.run("return Math.random();").is_err());
    }

    #[test]
    fn source_limit_is_enforced_before_quickjs() {
        let engine = ScriptEngine::new(ScriptLimits {
            max_source_bytes: 4,
            ..ScriptLimits::default()
        })
        .unwrap();
        assert!(matches!(
            engine.run("12345"),
            Err(ScriptError::SourceTooLarge { .. })
        ));
    }

    #[test]
    fn interrupt_handler_stops_an_unbounded_script() {
        let engine = ScriptEngine::new(ScriptLimits::default()).unwrap();
        engine.request_interrupt();
        assert!(matches!(
            engine.run("for (;;) {}"),
            Err(ScriptError::Runtime(_))
        ));
        engine.clear_interrupt();
    }

    #[test]
    fn wait_set_constructor_returns_typed_condition() {
        let engine = ScriptEngine::new(ScriptLimits::default()).unwrap();
        let result = engine
            .run("return Wait.any(Wait.at(10), Wait.event('impact'));")
            .unwrap();
        assert!(matches!(
            result,
            ScriptResult::Wait(WaitCondition::Any(conditions)) if conditions.len() == 2
        ));
    }

    #[test]
    fn plan_constructor_returns_a_validated_trajectory() {
        let engine = ScriptEngine::new(ScriptLimits::default()).unwrap();
        let result = engine
            .run("return Plan.guidance(5, Guidance.angularRate(0.1, 0, 0));")
            .unwrap();
        assert!(matches!(
            result,
            ScriptResult::Plan(TrajectoryPlan { segments, bakeability: Bakeability::Guarded, .. })
                if matches!(segments.as_slice(), [TrajectorySegment::Guidance { .. }])
        ));
    }

    #[test]
    fn pure_plan_rejects_a_live_wait_guard() {
        let engine = ScriptEngine::new(ScriptLimits::default()).unwrap();
        assert!(matches!(
            engine.run("return JSON.stringify({kind:'plan', id:0, bakeability:'pure', segments:[{kind:'wait', condition:{kind:'wait-event', name:'impact'}}]});"),
            Err(ScriptError::InvalidReturn(_))
        ));
    }
}
