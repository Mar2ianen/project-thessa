# WORLD142 GPU reset and desktop-session loss — 2026-10-27

The first WORLD142 raster capture triggered a full AMD GPU reset and loss of the
user's Niri desktop session. This is a failed capture, not visual evidence or
a stable build. Automatic desktop GPU runs are blocked; current investigation is CPU/log-only.

## Confirmed sequence

- 16:34:17 MSK: kernel reports `gfx_0.0.0 timeout`, identifying `native` PID3737704.
- 16:34:19: MES/local-ring recovery fails and the kernel begins a full reset.
- During hard recovery the capture loses its guilty context and aborts in wgpu
  `Device::poll` (core timestamp 16:34:19, journal entry 16:34:21). The panic is a
  consequence of device loss. MODE2 reset succeeds at 16:34:21.
- 16:34:22: Niri PID62819 reports an innocent lost context and aborts.
- 16:34:41: a new Niri session starts.

The failed capture used RT Off, MSAA4 and 24 cloud steps. It exited -6 after
85.911 s. The planned ray-calibration test had not started. Kernel logs in this
time window contain no OOM-killer or GPU page-fault explanation.

## Workload regression

A CPU-only reconstruction of the same frozen survey context prepared 17,992
instances in 37 grouped ranges and **148,572,518 submitted mesh triangles per pass**,
versus 32,954,976 in the captured WORLD136 baseline: **4.508×**.
Prototype storage is 44,765,568 B and passes the raised 48 MB admission limit.
That limit previously was 7 MB; neither value bounds instanced draw work.

The primary loop in `crates/render-wgpu/src/mesh.rs` submits all instance ranges.
Its `cast_shadow` loop does the same for each active layer; `shadows.rs` calls
it inside the layer loop. Same-epoch WORLD136/WORLD139 logs record 12 active
shadow layers (four cascades for three light slots). At those 12 layers the
candidate would submit **1,931,442,734 mesh triangles per frame** including primary,
before terrain/sky work. This is submitted index/instance work, not a claim that
all those triangles cover visible pixels. WORLD142 did not reach final GPU
statistics, so the 12-layer total is conditional on the retained light state.

The headless frame encodes terrain, all shadow layers, opaque meshes and sky
into one command buffer before `queue.submit`, then waits in `Device::poll`.
There is no per-instance mesh frustum culling in the inspected draw loops or
execution-time admission derived from the prototype-byte cap. This amplification
is a strong workload-timeout candidate. It is not proof of the exact hung draw,
or proof that Mesa/firmware rather than workload duration caused the stall.

## Limits and containment

No RADV command-marker/wave trace was collected for this run. The kernel
device dump exists but is unreadable without elevated access; it was not
cleared. [Mesa's crash guide](https://docs.mesa3d.org/graphics-debugging/debugging-misrenderings-crashes.html)
explains why a watchdog report alone does not distinguish excessive execution
time from a wedged hardware state. Its [RADV hang guide](https://docs.mesa3d.org/drivers/amd/hang-debugging.html)
describes command/shader traces needed for that distinction. No system watchdog
or recovery settings were changed, and no reproduction was attempted after
the session loss.

Automatic capture/motion/calibration launchers now reject execution while
`target/surface-showcase/GPU-RUNS-BLOCKED.json` exists. Their refusal was checked
without creating a GPU device. Temporary CPU instrumentation was restored
byte-for-byte; exact failed-capture source hashes still match. The candidate
remains uncommitted and must not be called GPU validated. Workload bounding,
instance/cascade visibility and submission boundaries need review before any
new hardware run; any reproducer must be isolated from this desktop GPU.

Raw journals, CPU-only probe source/command/result, core metadata, failed launch
profile and pins are retained locally in `/home/chechulin/Documents/Codex/2026-10-23/new-chat-3/outputs/gpu-reset-2026-10-27`. The structured record is
[`native-gpu-reset-2026-10-27.json`](native-gpu-reset-2026-10-27.json).
