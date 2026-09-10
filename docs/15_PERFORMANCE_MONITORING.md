# Performance Monitoring and Profiling

Status: design document / implementation target.

Scope: client rendering, authoritative simulation, time warp, world/terrain runtime, memory, and capture tooling.

The performance subsystem exists to answer one question reliably:

> what consumed the frame / simulation budget, on this build, on this machine, with these resolved settings?

The project should instrument before it optimizes. A low average frame time is not sufficient if terrain generation, asset work, time warp, clouds, atmosphere, or ray-traced effects introduce long-tail stalls.

The core rule is:

> named measurements are first-class data; overlays, logs, captures and external profilers are consumers of the same measurements.

---

## 1. Goals

The monitoring layer should make it easy to distinguish:

- CPU-bound vs GPU-bound frames;
- render cost vs simulation cost;
- average load vs rare hitching;
- normal flight vs time-warp overload;
- terrain/world-generation work vs steady-state rendering;
- memory growth vs stable cache residency;
- requested graphics settings vs actually resolved runtime settings.

It should support both development profiling and lightweight in-game diagnostics.

The first useful implementation should provide:

- frame time and FPS;
- CPU frame time;
- GPU frame time where supported;
- rolling median / p95 / p99 / maximum;
- subsystem timing scopes;
- simulation fixed-step counters and wall-clock cost;
- time-warp budget information;
- terrain/world counters;
- process/system memory counters where practical;
- a small rolling history buffer;
- a toggleable performance overlay;
- short JSON/CSV captures with build and graphics metadata.

---

## 2. Non-goals

The first implementation does not need to be:

- a replacement for Tracy, RenderDoc, perf, Instruments, PIX, Radeon GPU Profiler, Nsight or platform profilers;
- a full statistical telemetry backend;
- an always-on network analytics service;
- a deterministic part of gameplay simulation;
- zero-overhead at arbitrary instrumentation density;
- a reason to optimize code before a measured bottleneck exists.

External profilers remain valuable. The in-engine layer provides correlation and game-specific context they do not know automatically.

---

## 3. Measurement model

Use stable hierarchical names.

Examples:

```text
frame.cpu
frame.gpu

sim.total
sim.gravity
sim.aero
sim.rigid_body
sim.contacts
sim.guidance
sim.factory

world.total
world.terrain_sampling
world.terrain_meshing
world.landmarks
world.streaming

render.total
render.terrain
render.atmosphere
render.clouds
render.water
render.solari
render.shadows
render.postprocess
render.ui

io.assets
io.persistence
```

Exact names may evolve, but they should be stable enough that captures from two commits can be compared.

Prefer a small number of useful scopes over thousands of noisy scopes in release builds.

Conceptually:

```rust
perf_scope!("sim.aero");
perf_scope!("render.atmosphere");
```

The implementation may use Bevy diagnostics, `tracing`, custom RAII guards, GPU timestamp queries, or a combination. The document does not require one profiling library.

---

## 4. CPU timing

CPU timings should use a monotonic high-resolution clock.

Useful measurements:

```text
whole client frame
simulation wall-clock cost
render preparation / extraction / submission
terrain generation
asset work
UI
background tasks that can hitch the frame
```

For multithreaded work, distinguish when practical between:

- wall-clock latency of a stage;
- aggregate worker CPU time;
- jobs queued / completed.

Do not sum parallel task durations and label the result as frame latency.

---

## 5. GPU timing

GPU execution time is required for graphics work because CPU submission time is not a useful proxy for shader cost.

Where the backend supports timestamps, measure at least:

```text
whole GPU frame
terrain
atmosphere
clouds
ray-traced / Solari lighting
postprocess
```

Additional scopes are useful only when the implementation can obtain them without excessive synchronization or query overhead.

Never force a per-frame CPU/GPU stall solely to display profiling data. Read timestamp results asynchronously / with normal frame latency where possible.

When GPU timing is unavailable, report that explicitly rather than presenting CPU submit time as GPU time.

---

## 6. Frame statistics

Do not judge performance from instantaneous FPS alone.

Maintain a rolling ring buffer, initially around `10-30 s`, containing at least frame time and selected top-level timings.

Display / record:

```text
current
median / p50
p95
p99
max over capture/window
```

Percentiles are especially important for terrain generation and streaming: a game that renders most frames in 8 ms but occasionally stalls for 70 ms is not an 8 ms experience.

A reasonable overlay example:

```text
FRAME
  8.6 ms   116 fps
  CPU      3.1 ms
  GPU      7.9 ms
  p50      8.4 ms
  p95     10.2 ms
  p99     15.7 ms
  max     42.0 ms
```

The exact UI is not canonical.

---

## 7. Simulation budget and time warp

Simulation performance must be measured independently from render FPS.

For fixed-step flight simulation expose at least:

```text
fixed step duration in simulation time
fixed steps executed this render frame
wall-clock time spent in simulation
simulation-time advanced this frame
requested time-warp factor
resolved/effective time-warp factor
accumulator/backlog
```

Example:

```text
SIM
  fixed dt        8.333 ms
  steps/frame          12
  CPU             1.7 ms
  requested warp    x100
  effective warp     x96
  backlog          4.1 ms
```

This lets the game distinguish:

- render slowdown;
- a simulation that cannot keep up with requested warp;
- intentional transition to an analytic / coarse propagation mode later.

Time warp must never silently produce a different physical result because frame rate happened to fall. If the simulation cannot keep up, the runtime should expose the condition and resolve warp according to the simulation policy.

---

## 8. World / terrain counters

Timings alone are hard to interpret without workload counters.

Expose relevant counts such as:

```text
active celestial bodies
active physics vehicles
active aerodynamic panels
terrain patches visible
terrain patches generated this frame
terrain vertices / triangles submitted
terrain cache hits / misses
landmark zones active
streaming requests queued
assets loaded / pending
lights visible
ray-tracing instances / proxies
```

Counters should describe actual work, not arbitrary graphics-preset labels.

This is especially useful for future adaptive planetary terrain: a 15 ms terrain frame is much easier to understand when the capture also says `312 patches generated`.

---

## 9. Memory metrics

At minimum, capture process memory where it is inexpensive and portable:

```text
RSS / resident memory
allocator-tracked game allocations if available
asset cache size
terrain cache size
```

GPU / shared-memory reporting is backend-dependent.

On integrated GPUs, a driver-reported `VRAM used / budget` value may refer to a UMA reservation/budget rather than physically separate memory. Do not treat it as an independent dedicated-VRAM pool in diagnostics.

If reliable GPU memory data is unavailable, report `unknown` rather than synthesizing a misleading number.

---

## 10. Performance overlay

Suggested control:

```text
F4        toggle performance overlay
Shift+F4 start/stop short capture
```

Key bindings are provisional and may move into the controls/settings system.

Suggested compact overlay groups:

```text
FRAME
CPU/GPU + percentiles

SIM
fixed steps + warp + backlog

RENDER
terrain / atmosphere / clouds / RT / post

WORLD
patches / vehicles / triangles / lights

MEM
process / caches / GPU if available
```

The overlay should be cheap enough to leave enabled while diagnosing a problem.

Avoid rendering enormous scrolling profiler trees in the normal flight HUD.

---

## 11. Capture format

A short profiling capture should contain:

- metadata header;
- per-frame timestamp / simulation time;
- top-level CPU timings;
- GPU timings where available;
- simulation counters;
- selected workload counters;
- memory samples;
- optional event markers.

Support a human-inspectable format first.

Recommended outputs:

```text
perf-YYYYMMDD-HHMMSS.json
perf-YYYYMMDD-HHMMSS.csv
```

JSON is useful for structured metadata and nested scopes. CSV is useful for quick plotting and comparisons. Supporting both may share one in-memory capture model.

Do not capture every internal object state by default; this is a performance trace, not a save game.

---

## 12. Capture metadata

Every capture should identify the conditions that produced it.

At minimum:

```text
build commit / version
debug vs release build
OS
CPU
GPU / adapter
render backend
window resolution
resolution scale
vsync / frame cap
requested graphics preset/settings
resolved graphics settings
ray-tracing mode
simulation scenario / active body where practical
```

This is important because graphics settings are designed as requested values resolved through backend/GPU capabilities before reaching the renderer.

A comparison such as:

```text
commit A, Atmosphere High, RT Local -> GPU p95 10.8 ms
commit B, Atmosphere High, RT Local -> GPU p95 14.2 ms
```

is only meaningful if both captures store the resolved settings and hardware/backend information.

See `14_VISUAL_ATMOSPHERE.md` for the requested -> resolved graphics-settings model.

---

## 13. Graphics-setting correlation

The performance system should not interpret graphics quality itself.

It consumes `ResolvedGraphicsSettings` (or equivalent) as metadata and exposes measured costs.

This keeps responsibilities clear:

```text
graphics.toml
    -> requested settings
    -> capability resolution
    -> ResolvedGraphicsSettings
             |
             +--> renderer
             +--> perf capture metadata
```

If atmosphere, clouds, Solari, terrain detail or aurora change quality, their cost should appear naturally through named render scopes and counters.

---

## 14. Event markers

Allow lightweight point markers in captures for expensive or important transitions.

Examples:

```text
terrain LOD rebuild
large asset upload
body focus changed
entered atmosphere
left atmosphere
time warp changed
Solari enabled/disabled
graphics settings reapplied
major GC/allocator maintenance if relevant
```

Markers make a 40 ms spike easier to correlate with game events.

They should not become authoritative gameplay events.

---

## 15. Profiling levels

A useful implementation may expose instrumentation levels:

### Off / production-minimal

Keep only measurements needed for basic FPS / health reporting.

### Normal

Top-level CPU/GPU scopes, simulation budget, world counters and ring buffer.

### Detailed / developer

Additional subsystem scopes, queue depths, cache statistics and event markers.

Do not require a separate game build for every level where runtime toggling is cheap, but expensive instrumentation may remain compile-time/dev-only.

---

## 16. Overhead policy

The profiler must profile the game, not become the game.

Targets:

- top-level CPU scopes should have negligible frame impact;
- GPU timestamp count should remain bounded;
- ring buffers should use fixed-capacity storage;
- capture writing should not block the render thread;
- file serialization should happen after capture or on an appropriate background task;
- disabled detailed instrumentation should not perform expensive formatting/string allocation every frame.

String scope names should preferably resolve to stable IDs internally rather than allocate repeatedly.

---

## 17. Threading and async work

Tokio / worker-pool jobs need correlation without pretending that async task duration equals frame time.

For important asynchronous tasks record:

```text
queue time
start time
finish time
result / workload size
```

Useful future examples:

- terrain patch generation;
- asset decode/upload preparation;
- world save/load;
- factory batch work;
- network snapshot processing.

The frame overlay normally shows only work that affects current latency or backlog.

---

## 18. Network/server direction

The authoritative server should eventually expose the same style of named simulation measurements independently of the graphics client.

Server-focused metrics may include:

```text
simulation tick wall time
vehicle count
factory entity count
network input/output
snapshot encode/decode
persistence queue
warp/propagation mode
worker utilization
```

Client GPU metrics must not leak into simulation APIs.

The common naming/data model may be shared, but rendering instrumentation belongs to the client.

---

## 19. Suggested first implementation order

A practical first vertical slice:

1. frame-time ring buffer;
2. p50/p95/p99/max calculations;
3. `F4` overlay;
4. named CPU RAII scopes;
5. simulation fixed-step / warp counters;
6. terrain/world counters;
7. GPU whole-frame timestamp;
8. GPU subsystem timestamps for atmosphere/clouds/Solari as those systems land;
9. JSON capture;
10. CSV export and event markers.

Do not block visual-atmosphere work on a perfect profiler. The goal is to have enough instrumentation before several expensive rendering systems overlap.

---

## 20. Example capture header

Illustrative only:

```json
{
  "format_version": 1,
  "build": {
    "commit": "...",
    "profile": "release"
  },
  "platform": {
    "os": "linux",
    "cpu": "...",
    "gpu": "...",
    "backend": "vulkan"
  },
  "display": {
    "width": 1920,
    "height": 1080,
    "resolution_scale": 1.0,
    "vsync": false
  },
  "graphics": {
    "preset": "custom",
    "ray_tracing": "local"
  }
}
```

Do not make platform-specific fields mandatory when the backend cannot report them reliably.

---

## 21. Regression workflow

When investigating a regression:

```text
same scenario
same simulation initial state
same display resolution
same resolved graphics settings
same measurement window
```

Then compare:

```text
frame p50 / p95 / p99
CPU/GPU split
subsystem timings
workload counters
memory trend
```

Do not compare two random screenshots of `btop` and conclude that a subsystem became faster or slower.

System monitors remain useful for broad sanity checks, thermals, clocks, total process memory and GPU utilization; in-engine timing is needed to attribute cost inside Thessa.

---

## 22. Acceptance target

The first monitoring milestone is complete when a developer can perform a flight, press one key, and answer from the resulting overlay/capture:

```text
Was this frame CPU- or GPU-bound?
Which major subsystem consumed the time?
Was there a long-tail hitch?
Did terrain/world workload spike?
Could the simulation keep up with requested time warp?
How much memory was resident?
Which build/backend/resolved graphics settings produced the trace?
```

If those questions cannot be answered, add measurement before adding speculative optimization.
