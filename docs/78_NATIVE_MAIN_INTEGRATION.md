# 78 — Native replacement and main integration readiness

**Status:** default-off native capability integrated into main on 2026-10-23;
replacement and visual acceptance remain open.

The surface candidate incorporated `origin/main` through `d2bb846` without
rewriting history, passed the local gates below and was promoted to main as
`99a1462`. The incoming main commit only installed the Ponytail workflow and
updated agent instructions; it introduced no competing product changes at that
checkpoint. Main integration and deleting the old host remain separate
decisions: the merged opt-in capability is not full client replacement.

At the preparation checkpoint `e23467c`, main is an ancestor and the committed
topic adds 106 commits affecting 476 paths. The delta includes shared authority,
wire/geometry, server transport, old-host adapters and native rendering, not just
surface assets. Preserve that history and rerun cross-host/server tests; a
conflict-free Git merge is not sufficient evidence for changing launch defaults.

## Sequential closure order

| Layer | Existing owner / next closure criterion | Current boundary |
| --- | --- | --- |
| Source, identity, precision, residency | docs 63/70/72/77; preserve canonical sources, world-local IDs, exact zero material normals, bounded fallback and regular/conforming oracles | scoped tests exist; not universal terrain/visual accuracy |
| Near ground cover and update lifetime | `surface_plants`, `SceneMeshes`, shared `terrain_stream::JobHandle`; retain valid applied source, coalesce requests, reject stale output, measure request/worker/publication separately | WORLD07a same-site worker and compatible GPU-source reuse; setup/site changes and capacity/layout-changing reconstruction remain synchronous; 73 m cover and high GPU cost remain |
| Ground optical transport | doc 69 and WORLD06h–l; consistent direct/sky/substrate normal treatment with projected-area, reciprocity and energy oracles before a matched capture | characteristic ground bands unresolved; no grain-amplitude concealment |
| Foliage, ground contact, near/far composition | docs 70/75/77; WORLD07b fixes sideways-biased stellar rays with analytic and actual-producer visibility oracles; still inspect streamed-root agreement, then compare matched standing/close/overview and moving views | authored prototypes, sparse cover, finite boundary, subpixel coverage and finite-bias visibility gaps; no final botanical or blind preference acceptance |
| Water, shore, sky and shadows | docs 56/60/65/74; preserve shared surface/epoch, caster/receiver and finite-ray transport contracts through a composed camera path | remaining finite aerial perspective, water/sky transport and wider convergence stay open; no fake optical compensation |
| Movement and surface-to-orbit | existing survey/headless capture and bounded terrain streamer; frozen simulation camera sweep, repeated window crossings, mode changes, coarse fallback and return to exact identities | static frames alone do not close this gate; no whole-planet high-detail instantiation |
| Temporal reconstruction | temporal commits through `f931a8f` merged as `c1ecf0e`; opt-in scene history now uses the shared resolve in both native loops, with submitted-only advancement and publication resets | T4c lifecycle integration is partial; composed depth, full cut/reset coverage and real-scene ghosting/visual acceptance remain open; no SDK/FG or foliage-history claim |
| Host replacement | doc 66 source-to-consumer ledger and doc 39; account for launchers, controls, maps, server, audio, fixtures, settings and supported fallbacks | do not remove Bevy, old assets/tests or entry points before the ledger closes; do not label unavailable settings as applied |
| Main promotion | latest-main ancestry, scope review, full gates on exact integration candidate, immutable capture hashes, honest README/status/launch defaults | completed as `99a1462` after the T4c gates and documentation-only evidence commit; history preserved and defaults retained |

Temporal reconciliation: committed history through `f931a8f` is now merged,
preserving both terrain anisotropy and temporal alpha-mask shader generation.
The separate temporal worktree still has its five uncommitted drafts in
graphics settings/metadata/resolution, `motion.rs` and `graphics.toml`; none was
overwritten, discarded or silently staged. The surface integration independently
extends the committed contract. T4c is not complete temporal acceptance and
cannot certify foliage stability: all rigid meshes, including plants, currently
reject history. The current implementation and remaining gates are in doc 76.

## Promotion evidence

- Run formatting, architecture, all-target/all-feature Clippy, workspace lib/bin
  and integration tests, actual native/renderer hardware tests on 880M/RADV,
  feature-off, server/native release and the retained WORLD00 numeric regression.
  Record skipped/zero-test legacy commands as such, never as hardware coverage.
- Compare the **same** scene/settings/epoch/resolution against the retained
  baseline. Separate exact geometry/energy/lifecycle oracles, subjective blind
  preference, and CPU/GPU latency. Keep time spent waiting for workers distinct
  from foreground request and publication costs.
- Preserve origin-main history and author identity. Keep experimental effects
  disabled by default until accepted; merging an opt-in capability must not
  silently replace launch behavior or claim the old host retired.
- Record remaining gaps explicitly. No all-layers-complete, stable-performance,
  temporal or production-client-replacement claim follows from compiling or
  merging this branch.

## Executed WORLD07a integration checks

The 12 local command gates pass on 756 unchanged source/configuration inputs:
formatting, architecture, whole-workspace all-target/all-feature Clippy,
2,050 lib/bin and 108 integration tests, debug server build, 46 native ignored
hardware/CLI checks, 86 renderer hardware checks, native feature-off, the WORLD00
release regression, and joint viewer/server release. The legacy Bevy ignored
command selects zero tests and is not GPU coverage. This is local evidence,
not remote CI or closure of the remaining visual/host obligations above.

Record the **package selection** as part of a reproducible release recipe.
`cargo build --release -p thessa-terrain-native` reproduces capture SHA256
`7945e0cd259b5ac3b2ce113aa92f9e3ada3472e71cb4938fa55287f531aa5385`.
Building viewer and server together yields
`016a1a31593ca9a21c970a59a609a6fac48286aba94d20a166ff6924f7de9ac5`
with unchanged sources: `cargo tree` confirms widened unified `syn` and
`hashbrown` features. The initial joint-build identity assertion failed for that
recipe difference; it was resolved by rebuilding with the exact capture recipe,
not by changing source or editing the expected capture hash. Evidence:
[`artifacts/native-async-understory-2026-10-23.json`](artifacts/native-async-understory-2026-10-23.json).

## Executed WORLD07b integration checks

The physical stellar-ray correction passes 13 local gates: formatting,
architecture, all-target/all-feature workspace Clippy, 2,051 lib/bin tests,
109 integration tests (including the new analytic GPU ray-line regression),
47 native hardware/CLI checks (including actual-producer visibility), 86
renderer hardware checks, native feature-off, WORLD00 regression and debug/
release server plus release viewer builds. The legacy Bevy ignored command
selects zero tests, not additional coverage. All 757 compiled/configuration
inputs are unchanged, and the exact native-only release recipe reproduces
captured SHA256
`4c89c555a1f847c8cce0eb16f0050318cb382e577e4a4a130472aac9310cd422`.
No remote-CI, blind preference, stable-FPS or full replacement acceptance follows.
The remaining major diagonal branch marks, sparse/subpixel foliage, ground
normal transport and temporal/host obligations remain in the sequential ledger.
Evidence:
[`artifacts/native-stellar-ray-line-2026-10-23.json`](artifacts/native-stellar-ray-line-2026-10-23.json).

## Executed T4c integration checks

The default-off native history lifecycle and masked-source correction pass all
13 local gates on 645 unchanged compiled/configuration inputs: formatting,
architecture, whole-workspace all-target/all-feature Clippy, 2,080 lib/bin tests,
117 integration tests, 47 native hardware/CLI checks, 87 renderer hardware checks,
native feature-off, WORLD00 regression and debug/release server plus release
viewer builds. The legacy Bevy ignored command selects zero tests; this is not
additional GPU coverage or remote CI.

The exact release recipe is `cargo build --release -p thessa-terrain-native`,
with SHA256
`86c0ba6cff670e4e56f10135ce4479d185d925c76f2d0d0bc3161f2b29880fcb`.
GPU regressions pin age-zero masked history, conservative four-tap source
validation, first-frame static eligibility, recovery to age one, submitted-only
history, overlay exclusion and lifecycle resets. The initial orbit frame exposed
duplicated tree silhouettes; the corrected diagnostic frame no longer shows
those silhouettes. That inspection is not independent visual preference or a
general moving-scene ghosting bound.

All nine final captures exited zero with simulation time/step zero; all eight
current-build captures match the certified executable hash above. At 1280×720,
the standing/close/overview Medium-TAA runs each submit 78 real frames with one
global reset; resolve medians are 0.890/1.034/1.073 ms on Radeon 880M,
Vulkan/RADV, Mesa 26.2.5-arch1.1. Their off/on pairs match all 23 compared scene
keys. CPU preparation/publication timings are recorded separately from scene
mismatches. Three blind pairs are prepared with a separate private key, but no
independent reviewer votes have been collected.

The final orbit run resets globally in 111 of 126 submitted frames. Its off/on
material residency differs, so orbit is excluded from blind preference and kept
only as descriptive telemetry. The final corrected still was inspected without
observing the former duplicated silhouettes; this does not bound moving-scene
ghosting. The retained `305e373` baseline differs from current standing-off at
six pixels (maximum RGB code delta 20), and predates the temporal foundation
merge; it is neither bit-exact nor an isolated T4c reference. No stable-FPS,
foliage/ocean antialiasing or visual acceptance claim follows.

Evidence:
[`artifacts/native-temporal-history-2026-10-23.json`](artifacts/native-temporal-history-2026-10-23.json).
Raw captures, gate logs and blind review files remain local under
`target/surface-showcase/temporal-history/`; the tracked artifact records their
hashes, commands, settings, counts and limitations.

## Main promotion record — 2026-10-23

- Fetched origin immediately before promotion; remote main remained `d2bb846`,
  an ancestor of implementation `95a3888` and evidence candidate `99a1462`.
- Verified a clean candidate, local-main ancestry and that main was not checked
  out in another worktree. Preserved identity Mar2ianen /
  `92949303+Mar2ianen@users.noreply.github.com` and all existing commit authors.
- Pushed `feat/native-surface-showcase` from `305e373` to `99a1462`, then
  fast-forwarded local main and pushed remote main from `d2bb846` to `99a1462`.
  Both pushes succeeded without force; `git ls-remote --heads origin` confirmed
  the two remote refs at `99a146200bf2c49bfbca6308acef640e332753c3`.
- The evidence candidate changes only README/docs relative to the tested
  implementation. All 645 compiled/configuration input hashes and the certified
  release hash were rechecked, together with all nine image/capture/settings
  identities, 13 gate log hashes/counts and the three eligible blind cases.
  The architecture guard and `git diff --check` passed again. The full cargo
  suite was not repeated for documentation-only changes; no remote-CI outcome
  is claimed. This promotion-record follow-up also changes documentation only.
- Ground detail, understory and TAA remain default-off. Launch defaults and the
  old host are retained. The five separate temporal-worktree drafts still match
  their recorded hashes and were neither edited nor staged.

The remaining sequential visual, transport, streaming, temporal and host
obligations above are unchanged by this promotion. Blind preference remains
unreviewed; orbit remains ineligible because residency differs.
