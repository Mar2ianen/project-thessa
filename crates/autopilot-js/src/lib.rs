//! Sandboxed QuickJS graph-block host.
//!
//! JavaScript in this crate is a high-level producer of typed guidance
//! values. It never receives a `FlightAuthority`, rigid-body state handle, or
//! actuator reference. The host exposes only deterministic constructors and
//! rejects ambient capabilities before evaluating user code.

use std::{
    cell::RefCell,
    collections::BTreeMap,
    error::Error,
    fmt,
    rc::Rc,
    sync::{
        Arc,
        atomic::{AtomicBool, Ordering},
    },
};

use rquickjs::{
    Context, Ctx, Error as JsError, Function, Persistent, Promise, Runtime, prelude::Func,
};
use serde::Deserialize;
use thessa_autopilot::{
    Bakeability, Diagnostic, ImpactSite, LandingSite, TrajectoryPlan, TrajectorySegment,
    WaitCondition, WaitId, WaitSet,
};
use thessa_flight_control::{DirectionFrame, DirectionTarget, GuidanceIntent, RollPolicy};
use thessa_sim_core::SimTime;

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
    LandingSite(LandingSite),
    ImpactSite(ImpactSite),
    Wait(WaitCondition),
    Diagnostic(Diagnostic),
}

#[derive(Debug)]
pub struct ScriptContinuation {
    promise: Persistent<Promise<'static>>,
    resolver: Persistent<Function<'static>>,
}

#[derive(Debug, Clone, PartialEq)]
pub enum ScriptWait {
    /// A relative simulation delay. The owning scheduler resolves it against
    /// the current `SimTime` when the task is started or resumed.
    After(f64),
    Condition(WaitCondition),
}

#[derive(Debug)]
pub enum ScriptStep {
    Completed(ScriptResult),
    Waiting {
        wait: ScriptWait,
        continuation: ScriptContinuation,
    },
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, PartialOrd, Ord)]
pub struct ScriptTaskId(pub u64);

#[derive(Debug, Clone, PartialEq)]
pub enum ScriptSchedulerStep {
    Completed {
        task: ScriptTaskId,
        result: ScriptResult,
    },
    Waiting {
        task: ScriptTaskId,
        wait: WaitId,
        condition: WaitCondition,
    },
}

#[derive(Debug, Clone, PartialEq)]
pub enum ScriptError {
    InvalidLimits,
    SourceTooLarge { bytes: usize, limit: usize },
    TaskIdExhausted,
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
            Self::TaskIdExhausted => write!(formatter, "autopilot script task id space exhausted"),
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
    wait_request: Rc<RefCell<Option<PendingWait>>>,
}

#[derive(Debug)]
struct PendingWait {
    wait: ScriptWait,
    resolver: Persistent<Function<'static>>,
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
        let wait_request = Rc::new(RefCell::new(None));
        context
            .with(|ctx| install_wait_api(ctx, Rc::clone(&wait_request)))
            .map_err(|error| ScriptError::Runtime(error.to_string()))?;
        Ok(Self {
            runtime,
            context,
            limits,
            interrupt_requested,
            wait_request,
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

    /// Start an async graph block. `sim.sleep` and `sim.event` create native
    /// wait registrations and leave the JS continuation parked in QuickJS;
    /// this method never sleeps the Rust worker.
    pub fn run_async(&self, source: &str) -> Result<ScriptStep, ScriptError> {
        self.check_source_size(source)?;
        self.wait_request.borrow_mut().take();
        let wrapped = format!(
            "{}\n(async function() {{\n'use strict';\n{}\n}})()",
            SANDBOX_PRELUDE, source
        );
        self.context.with(|ctx| {
            let promise = ctx
                .eval::<Promise, _>(wrapped.as_str())
                .map_err(|error| ScriptError::Runtime(error.to_string()))?;
            self.finish_promise(promise)
        })
    }

    fn finish_promise(&self, promise: Promise<'_>) -> Result<ScriptStep, ScriptError> {
        let persistent_promise = Persistent::save(promise.ctx(), promise.clone());
        match promise.finish::<String>() {
            Ok(json) => Ok(ScriptStep::Completed(parse_result(&json)?)),
            Err(JsError::WouldBlock) => {
                let pending = self.wait_request.borrow_mut().take().ok_or_else(|| {
                    ScriptError::Runtime(
                        "async script suspended without a registered simulation wait".into(),
                    )
                })?;
                Ok(ScriptStep::Waiting {
                    wait: pending.wait,
                    continuation: ScriptContinuation {
                        promise: persistent_promise,
                        resolver: pending.resolver,
                    },
                })
            }
            Err(error) => Err(ScriptError::Runtime(error.to_string())),
        }
    }

    pub fn limits(&self) -> ScriptLimits {
        self.limits
    }

    fn check_source_size(&self, source: &str) -> Result<(), ScriptError> {
        if source.len() > self.limits.max_source_bytes {
            return Err(ScriptError::SourceTooLarge {
                bytes: source.len(),
                limit: self.limits.max_source_bytes,
            });
        }
        Ok(())
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

impl ScriptContinuation {
    /// Resolve one native wait and run the QuickJS job queue until the script
    /// completes or registers its next simulation wait.
    pub fn resume(self, engine: &ScriptEngine) -> Result<ScriptStep, ScriptError> {
        engine.context.with(|ctx| {
            let resolver = self
                .resolver
                .restore(&ctx)
                .map_err(|error| ScriptError::Runtime(error.to_string()))?;
            resolver
                .call::<(), ()>(())
                .map_err(|error| ScriptError::Runtime(error.to_string()))?;
            let promise = self
                .promise
                .restore(&ctx)
                .map_err(|error| ScriptError::Runtime(error.to_string()))?;
            engine.finish_promise(promise)
        })
    }
}

/// Bridges QuickJS continuations to the simulation-owned wait set. The
/// scheduler owns no wall-clock worker: callers wake it with simulation time
/// or a domain event, then receive all continuations that became runnable.
#[derive(Debug, Default)]
pub struct ScriptScheduler {
    next_task_id: u64,
    waits: WaitSet,
    continuations: BTreeMap<WaitId, (ScriptTaskId, ScriptContinuation)>,
}

impl ScriptScheduler {
    pub fn start(
        &mut self,
        engine: &ScriptEngine,
        now: SimTime,
        source: &str,
    ) -> Result<ScriptSchedulerStep, ScriptError> {
        let task = ScriptTaskId(
            self.next_task_id
                .checked_add(1)
                .ok_or(ScriptError::TaskIdExhausted)?,
        );
        self.next_task_id = task.0;
        self.attach(task, now, engine.run_async(source)?)
    }

    pub fn wake(
        &mut self,
        engine: &ScriptEngine,
        now: SimTime,
        event: Option<&str>,
    ) -> Result<Vec<ScriptSchedulerStep>, ScriptError> {
        let ready = self.waits.wake(now, event);
        let mut steps = Vec::with_capacity(ready.len());
        for wait in ready {
            let Some((task, continuation)) = self.continuations.remove(&wait) else {
                continue;
            };
            steps.push(self.attach(task, now, continuation.resume(engine)?)?);
        }
        Ok(steps)
    }

    pub fn cancel(&mut self, wait: WaitId) -> bool {
        let removed_wait = self.waits.cancel(wait);
        let removed_continuation = self.continuations.remove(&wait).is_some();
        removed_wait || removed_continuation
    }

    pub fn cancel_all(&mut self) {
        self.waits = WaitSet::default();
        self.continuations.clear();
    }

    pub fn next_time(&self) -> Option<SimTime> {
        self.waits.next_time()
    }

    pub fn pending(&self) -> usize {
        self.waits.len()
    }

    pub fn is_empty(&self) -> bool {
        self.waits.is_empty()
    }

    fn attach(
        &mut self,
        task: ScriptTaskId,
        now: SimTime,
        step: ScriptStep,
    ) -> Result<ScriptSchedulerStep, ScriptError> {
        match step {
            ScriptStep::Completed(result) => Ok(ScriptSchedulerStep::Completed { task, result }),
            ScriptStep::Waiting {
                wait: script_wait,
                continuation,
            } => {
                let condition = match script_wait {
                    ScriptWait::After(delay) => WaitCondition::At(now.offset(delay)),
                    ScriptWait::Condition(condition) => condition,
                };
                let wait = self
                    .waits
                    .register(condition.clone())
                    .map_err(|error| ScriptError::Runtime(error.to_string()))?;
                self.continuations.insert(wait, (task, continuation));
                Ok(ScriptSchedulerStep::Waiting {
                    task,
                    wait,
                    condition,
                })
            }
        }
    }
}

fn install_wait_api(
    ctx: Ctx<'_>,
    wait_request: Rc<RefCell<Option<PendingWait>>>,
) -> rquickjs::Result<()> {
    let sleep_request = Rc::clone(&wait_request);
    let sleep = Func::from(
        move |seconds: f64, resolver: Function| -> rquickjs::Result<()> {
            if !seconds.is_finite() || seconds < 0.0 {
                return Err(JsError::new_from_js_message(
                    "number",
                    "simulation wait",
                    "sleep duration must be finite and non-negative",
                ));
            }
            let condition = WaitCondition::At(SimTime(seconds));
            condition.validate().map_err(|error| {
                JsError::new_from_js_message("number", "simulation wait", error.to_string())
            })?;
            let mut pending = sleep_request.borrow_mut();
            if pending.is_some() {
                return Err(JsError::new_from_js_message(
                    "simulation wait",
                    "single pending wait",
                    "a graph block may park only one await at a time",
                ));
            }
            let callback_ctx = resolver.ctx().clone();
            *pending = Some(PendingWait {
                wait: ScriptWait::After(seconds),
                resolver: Persistent::save(&callback_ctx, resolver),
            });
            Ok(())
        },
    );

    let event_request = Rc::clone(&wait_request);
    let event = Func::from(
        move |name: String, resolver: Function| -> rquickjs::Result<()> {
            let condition = WaitCondition::Event(name);
            condition.validate().map_err(|error| {
                JsError::new_from_js_message("string", "simulation event", error.to_string())
            })?;
            let mut pending = event_request.borrow_mut();
            if pending.is_some() {
                return Err(JsError::new_from_js_message(
                    "simulation wait",
                    "single pending wait",
                    "a graph block may park only one await at a time",
                ));
            }
            let callback_ctx = resolver.ctx().clone();
            *pending = Some(PendingWait {
                wait: ScriptWait::Condition(condition),
                resolver: Persistent::save(&callback_ctx, resolver),
            });
            Ok(())
        },
    );

    ctx.globals().set("__thessa_register_sleep", sleep)?;
    ctx.globals().set("__thessa_register_event", event)?;
    Ok(())
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
    velocityDirection: (x, y, z, frame, target_body = null) => JSON.stringify({kind:'velocity-direction', x, y, z, frame, target_body}),
  });
  globalThis.Landing = Object.freeze({
    site: (x, y, z, radius_m) => JSON.stringify({kind:'landing-site', x, y, z, radius_m}),
  });
  globalThis.Impact = Object.freeze({
    site: (x, y, z, radius_m) => JSON.stringify({kind:'impact-site', x, y, z, radius_m}),
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
  globalThis.sim = Object.freeze({
    sleep: (seconds) => new Promise(resolve => __thessa_register_sleep(seconds, resolve)),
    timeout: (seconds) => new Promise(resolve => __thessa_register_sleep(seconds, resolve)),
    event: (name) => new Promise(resolve => __thessa_register_event(name, resolve)),
  });
  globalThis.Plan = Object.freeze({
    coast: (duration_s) => JSON.stringify({kind:'plan', id:0, bakeability:'pure', segments:[{kind:'coast', duration_s}]}),
    burn: (duration_s, normalized, fx=0, fy=0, fz=0, mx=0, my=0, mz=0) => JSON.stringify({kind:'plan', id:0, bakeability:'guarded', segments:[{kind:'burn', duration_s, normalized, force_body_n:[fx, fy, fz], moment_body_nm:[mx, my, mz]}]}),
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
        #[serde(default)]
        target_body: Option<u32>,
    },
    LandingSite {
        x: f64,
        y: f64,
        z: f64,
        radius_m: f64,
    },
    ImpactSite {
        x: f64,
        y: f64,
        z: f64,
        radius_m: f64,
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
        #[serde(default)]
        force_body_n: Option<[f64; 3]>,
        #[serde(default)]
        moment_body_nm: Option<[f64; 3]>,
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
        ScriptReturn::VelocityDirection {
            x,
            y,
            z,
            frame,
            target_body,
        } => Ok(ScriptResult::Guidance(parse_guidance(
            ScriptReturn::VelocityDirection {
                x,
                y,
                z,
                frame,
                target_body,
            },
        )?)),
        ScriptReturn::LandingSite { x, y, z, radius_m } => LandingSite::new([x, y, z], radius_m)
            .map(ScriptResult::LandingSite)
            .map_err(|error| ScriptError::InvalidReturn(error.to_string())),
        ScriptReturn::ImpactSite { x, y, z, radius_m } => ImpactSite::new([x, y, z], radius_m)
            .map(ScriptResult::ImpactSite)
            .map_err(|error| ScriptError::InvalidReturn(error.to_string())),
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
                force_body_n,
                moment_body_nm,
            } => {
                let propulsion = thessa_flight_control::PropulsionDemand::new(normalized)
                    .map_err(|error| ScriptError::InvalidReturn(error.to_string()))?;
                let demand = thessa_flight_control::ControlDemand {
                    force_body_n: force_body_n
                        .map(glam::DVec3::from_array)
                        .unwrap_or(glam::DVec3::ZERO),
                    moment_body_nm: moment_body_nm
                        .map(glam::DVec3::from_array)
                        .unwrap_or(glam::DVec3::ZERO),
                    propulsion,
                };
                demand
                    .validate()
                    .map_err(|error| ScriptError::InvalidReturn(error.to_string()))?;
                Ok(TrajectorySegment::Burn { duration_s, demand })
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
        ScriptReturn::VelocityDirection {
            x,
            y,
            z,
            frame,
            target_body,
        } => {
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
            let direction = match target_body {
                Some(body) => DirectionTarget::for_target(direction.direction, body)
                    .map_err(|error| ScriptError::InvalidReturn(error.to_string()))?,
                None => direction,
            };
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
    fn target_guidance_constructor_keeps_the_resolved_body_id() {
        let engine = ScriptEngine::new(ScriptLimits::default()).unwrap();
        let result = engine
            .run("return Guidance.velocityDirection(1, 0, 0, 'target', 7);")
            .unwrap();
        assert!(matches!(
            result,
            ScriptResult::Guidance(GuidanceIntent::VelocityDirection { direction, .. })
                if direction.frame == DirectionFrame::Target && direction.target_body == Some(7)
        ));
    }

    #[test]
    fn site_constructors_return_normalized_typed_targets() {
        let engine = ScriptEngine::new(ScriptLimits::default()).unwrap();
        let landing = engine.run("return Landing.site(0, 2, 0, 250);").unwrap();
        assert!(matches!(
            landing,
            ScriptResult::LandingSite(site)
                if site.center_dir == [0.0, 1.0, 0.0] && site.radius_m == 250.0
        ));
        let impact = engine.run("return Impact.site(1, 0, 0, 75);").unwrap();
        assert!(matches!(
            impact,
            ScriptResult::ImpactSite(site)
                if site.center_dir == [1.0, 0.0, 0.0] && site.radius_m == 75.0
        ));
    }

    #[test]
    fn site_constructor_rejects_invalid_radius() {
        let engine = ScriptEngine::new(ScriptLimits::default()).unwrap();
        assert!(matches!(
            engine.run("return Landing.site(1, 0, 0, -1);"),
            Err(ScriptError::InvalidReturn(_))
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
    fn plan_burn_can_emit_a_physical_wrench() {
        let engine = ScriptEngine::new(ScriptLimits::default()).unwrap();
        let result = engine
            .run("return Plan.burn(5, 0.4, 0, 100, 0, 2, 0, 0);")
            .unwrap();
        let ScriptResult::Plan(plan) = result else {
            panic!("Plan.burn must return a plan");
        };
        let [TrajectorySegment::Burn { demand, .. }] = plan.segments.as_slice() else {
            panic!("Plan.burn must return one burn segment");
        };
        assert_eq!(demand.force_body_n, glam::DVec3::Y * 100.0);
        assert_eq!(demand.moment_body_nm, glam::DVec3::X * 2.0);
        assert_eq!(demand.propulsion.normalized, 0.4);
    }

    #[test]
    fn pure_plan_rejects_a_live_wait_guard() {
        let engine = ScriptEngine::new(ScriptLimits::default()).unwrap();
        assert!(matches!(
            engine.run("return JSON.stringify({kind:'plan', id:0, bakeability:'pure', segments:[{kind:'wait', condition:{kind:'wait-event', name:'impact'}}]});"),
            Err(ScriptError::InvalidReturn(_))
        ));
    }

    #[test]
    fn async_sleep_parks_and_resumes_the_continuation() {
        let engine = ScriptEngine::new(ScriptLimits::default()).unwrap();
        let step = engine
            .run_async("await sim.sleep(3); return Guidance.angularRate(0.1, 0, 0);")
            .unwrap();
        let continuation = match step {
            ScriptStep::Waiting {
                wait: ScriptWait::After(delay),
                continuation,
            } => {
                assert_eq!(delay, 3.0);
                continuation
            }
            ScriptStep::Completed(_) => panic!("sleep should park the script"),
            ScriptStep::Waiting { .. } => panic!("unexpected wait kind"),
        };

        let resumed = continuation.resume(&engine).unwrap();
        assert!(matches!(
            resumed,
            ScriptStep::Completed(ScriptResult::Guidance(GuidanceIntent::AngularRate { rate_body_rps }))
                if rate_body_rps == glam::DVec3::new(0.1, 0.0, 0.0)
        ));
    }

    #[test]
    fn async_event_registers_a_named_wait() {
        let engine = ScriptEngine::new(ScriptLimits::default()).unwrap();
        let step = engine
            .run_async("await sim.event('impact'); return Guidance.angularRate(0, 0.2, 0);")
            .unwrap();
        assert!(matches!(
            step,
            ScriptStep::Waiting {
                wait: ScriptWait::Condition(WaitCondition::Event(ref name)),
                ..
            } if name == "impact"
        ));
    }

    #[test]
    fn scheduler_wakes_and_reparks_async_tasks() {
        let engine = ScriptEngine::new(ScriptLimits::default()).unwrap();
        let mut scheduler = ScriptScheduler::default();
        let started = scheduler
            .start(
                &engine,
                SimTime(100.0),
                "await sim.sleep(3); await sim.event('impact'); return Guidance.angularRate(0, 0.2, 0);",
            )
            .unwrap();
        let task = match started {
            ScriptSchedulerStep::Waiting {
                task,
                wait,
                condition: WaitCondition::At(time),
            } => {
                assert_eq!(time, SimTime(103.0));
                assert_eq!(scheduler.next_time(), Some(SimTime(103.0)));
                assert_eq!(scheduler.pending(), 1);
                assert!(!scheduler.cancel(WaitId(999)));
                (task, wait)
            }
            ScriptSchedulerStep::Completed { .. } => panic!("sleep should park the task"),
            ScriptSchedulerStep::Waiting { .. } => panic!("unexpected wait condition"),
        };

        assert!(
            scheduler
                .wake(&engine, SimTime(102.0), None)
                .unwrap()
                .is_empty()
        );
        let reparks = scheduler.wake(&engine, SimTime(103.0), None).unwrap();
        assert_eq!(scheduler.pending(), 1);
        assert!(matches!(
            reparks.as_slice(),
            [ScriptSchedulerStep::Waiting {
                task: resumed_task,
                condition: WaitCondition::Event(name),
                ..
            }] if *resumed_task == task.0 && name == "impact"
        ));

        let completed = scheduler
            .wake(&engine, SimTime(103.0), Some("impact"))
            .unwrap();
        assert_eq!(scheduler.pending(), 0);
        assert!(matches!(
            completed.as_slice(),
            [ScriptSchedulerStep::Completed {
                task: completed_task,
                result: ScriptResult::Guidance(GuidanceIntent::AngularRate { rate_body_rps }),
            }] if *completed_task == task.0 && *rate_body_rps == glam::DVec3::new(0.0, 0.2, 0.0)
        ));

        assert!(!scheduler.cancel(task.1));
    }
}
