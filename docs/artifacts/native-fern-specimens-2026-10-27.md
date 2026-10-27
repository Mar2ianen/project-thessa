# Native fern specimens and isolated soil inspection — 2026-10-27

This is a continuation after the confirmed WORLD142 Radeon reset/Niri session
loss. It is **not reference acceptance or physical-GPU stability validation**.
The physical replay block remains in place; incident evidence is in
`native-gpu-reset-2026-10-27.md` and renderer tracing in
`native-gpu-trace-caster-2026-10-27.md`.

## Isolation and actual-render boundary

The captures use the production plant producer, material atlases, geometric
ambient-visibility bake, shared mesh renderer, shadow atlas, HDR display transform
and 4× MSAA. A neutral support plane and QA light isolate anatomy; they are not
the coastal environment or calibrated botanical reflectance. Each shadow/primary
stage has the diagnostic queue fence and one-million mesh-triangle admission cap.

The private, SHA-256-verified Arch Mesa 26.2.5 software ICD is loaded inside
bubblewrap with a fresh `/dev`, no `/dev/dri`, no display selection/sockets, and
private PID/network namespaces. The fixture also rejects visible `/dev/dri`
**before** backend bring-up and requires a CPU adapter afterwards. Stage records
identify llvmpipe LLVM 23.1.3, device type CPU. No system driver was installed or
watchdog setting changed. This establishes software execution, not Radeon
performance, desktop safety under hardware rendering, or RT convergence.

## Iteration record

The raw stages are under
`target/surface-showcase/overnight-coastal-2026-10-26/`.

- WORLD146 `146-compound-specimens`: all twelve real full/reduced fern/tree
  specimens rendered in 45.45 s. Inspection found broad planar reduced wings,
  overly fine ground pinnules and bright orange senescent organs.
- WORLD147 `147-divided-wing-specimens`: twelve renders completed in 54.73 s.
  Broader full pinnules and segmented coarse wings are visible. Four-, six- and
  eight-bin trial reductions failed the existing displacement guard on different
  tree forms; their failed logs are retained. The later adaptive candidate passed
  that guard, but failed three native preparation tests at the existing 48 MB
  aggregate source limit. It is not an admitted scene checkpoint.
- WORLD151 `151-broader-ground-specimens`: twelve renders completed in 50.70 s.
  Ground secondary limits of 8/12/16 made the individual organs clearer. The
  complete worldgen run caught an obsolete density fixture (>800 oblong organs
  per test frond); geometric attachment/error tests passed. Further inspection
  still found fine comb silhouettes and oversized terminal spears, prompting a
  smaller ground cadence and restoration of the short terminal lamina.

- WORLD158 `158-ground-leaf-mass-specimens`: the retained source uses 6/8/12
  secondary pairs for juvenile/mature/hero ground forms, width/reach 0.55 for
  ground versus 0.34 for crowns, and the former short terminal lamina. All
  twelve actual specimens completed in 60.01 s. Canonical ground triangles are
  17,956 full / 3,390 reduced; crown 141,274 / 21,884. Both geometric normals
  and bounded surface/area/attachment tests remain. The organ-count fixture
  now requires >300 oblong pinnules in the explicit eight-pair test frond,
  replacing the old >800 density prior; the numerical error guards are unchanged.
  Native full canopy/understory/soil preparation again fits the retained 47 MB
  plant-only test and 48 MB aggregate admission limits.

Three matched specimen pairs (canonical ground, hero ground, reduced crown)
were prepared with `tools/agent-qa/arena.py`. Their origin key is outside Git
and the reviewer directory; no votes or preference score are recorded.

These are author-inspected diagnostic renders, not blind preference votes.
No reference artwork or botanical photograph was copied into runtime materials.

## Soil diagnostic boundary

WORLD149's requested 0.7 m eye and WORLD153's requested 6 m survey distance were
rejected by the existing CLI bounds before rendering. The bounds were retained.
WORLD150 `150-soil-software-baseline` completed on the WORLD147 release at the
canonical coastal coordinates, 2 m eye/80 m survey distance, 512×288, frozen
epoch, with plants/clouds/RT/reflections disabled and one 1024 shadow cascade
per light. It rendered 31 frames including the existing 30 warmups; the trace
ends after successful PNG readback. The 97.69 s wall time includes CPU scene
preparation and 40 s prestream and is not a frame-performance measurement.

The real producer reports 6,692 pebbles, 192 moss clumps, 1,271 litter fragments,
24 pools/1,960 water triangles, 233,476 submitted mesh triangles per primary
pass and 700,428 across three shadow layers. Source payload is 986,760 B.
The clear-sky shot is useful for substrate inspection but cannot establish the
colour of the production overcast scene. Soil source is unchanged by these
plant-only edits. The follow-up overcast diagnostic retains production clouds;
its settings and results are recorded separately.

## Remaining limits

Full coastal hardware/moving-camera and independent blind reference acceptance
remain open. Coarse triangle count increases relative to WORLD142, so smaller
source bytes must never be described as a bound on expanded GPU work. TAA is
still disabled. No main integration is performed by this continuation.

## Retained checkpoint validation

WORLD157 completed eleven CPU/build checks on the same production source hash
as WORLD158: `53cc24c16f8dd215b07db13560f4ce1c1e548d5cbbc09373890af9cef97eb255`.
There are 462 passed CPU cases (284 worldgen, 43 render-core, 3 caster-volume,
132 native), plus formatting, strict affected all-target/all-feature Clippy,
feature-off compile, architecture, debug/release builds and compile-only unified
oracle. Actual hardware/RT tests remain ignored; compiling them is not execution.

WORLD155 completed its 31 real software-rendered frames/readback in 332.62 s
including setup/prestream. Inspection still finds the moss transfer too bright
and yellow-green for the reference. No soil/reference acceptance is recorded;
a subsequent material iteration must preserve matched capture settings and
source provenance. Source/capture hashes and commands are in the companion JSON.
