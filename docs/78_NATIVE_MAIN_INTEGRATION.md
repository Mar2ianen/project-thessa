# 78 — Native replacement and main integration readiness

**Status:** preparation in progress; not replacement or visual acceptance.

The surface branch now incorporates `origin/main` through `d2bb846` without
rewriting history. The incoming commit only installs the Ponytail workflow and
updates agent instructions; there are no competing product changes at this
checkpoint. Further work stays on `feat/native-surface-showcase` until the
following gates have evidence. Main integration and deleting the old host are
separate decisions: a green opt-in renderer is not full client replacement.

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
| Foliage, ground contact, near/far composition | docs 70/75/77; inspect diagonal foliage defects and streamed-root agreement, then compare matched standing/close/overview and moving views | authored prototypes, sparse cover, finite boundary; no final botanical or blind preference acceptance |
| Water, shore, sky and shadows | docs 56/60/65/74; preserve shared surface/epoch, caster/receiver and finite-ray transport contracts through a composed camera path | remaining finite aerial perspective, water/sky transport and wider convergence stay open; no fake optical compensation |
| Movement and surface-to-orbit | existing survey/headless capture and bounded terrain streamer; frozen simulation camera sweep, repeated window crossings, mode changes, coarse fallback and return to exact identities | static frames alone do not close this gate; no whole-planet high-detail instantiation |
| Temporal reconstruction | separate `feat/native-temporal-foundation` worktree; history/reset/disocclusion and simulation-ownership tests on the integrated tree | not merged or presented as finished here; generated frames must never advance physics/history |
| Host replacement | doc 66 source-to-consumer ledger and doc 39; account for launchers, controls, maps, server, audio, fixtures, settings and supported fallbacks | do not remove Bevy, old assets/tests or entry points before the ledger closes; do not label unavailable settings as applied |
| Main promotion | latest-main ancestry, scope review, full gates on exact integration candidate, immutable capture hashes, honest README/status/launch defaults | main unchanged by this preparation; no force push or destructive branch replacement |

Temporal checkpoint: the separate worktree is at `f931a8f` (stateless validated
GPU resolve), not completed native scene/history integration. It also has
uncommitted changes in graphics settings/metadata/resolution, `motion.rs` and
`graphics.toml`. These drafts were inspected only for inventory; they are not
silently imported, overwritten or discarded by the surface work. Reconcile
their ownership and verify T4c before claiming temporal completion or using it
to certify foliage stability.

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
